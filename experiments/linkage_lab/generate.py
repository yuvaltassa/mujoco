"""Generate the original Robotiq and a coupler-split shadow-weld variant.

Uses only the standard library. Source: pinned Menagerie robotiq_2f85, not v4.
"""
import argparse
import copy
import json
import hashlib
import math
import pathlib
import xml.etree.ElementTree as ET

MENAGERIE_COMMIT = '8161bba264d7fa7c99ca301e91e7fb44737676ad'
SOURCE_SHA256 = 'd48aca5f9151798ffd38111ce4e8b2081f3ec2d4f525161b33643451580010de'


def vec(text):
  return [float(x) for x in text.split()]


def fmt(values):
  return ' '.join(f'{x:.17g}' for x in values)


def mul(a, b):
  w, x, y, z = a
  v, i, j, k = b
  return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
          w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def inv(q):
  return [q[0], -q[1], -q[2], -q[3]]


def rot(q, v):
  return mul(mul(q, [0, *v]), inv(q))[1:]


def poses(root):
  result = {}

  def walk(body, p, q):
    r = vec(body.get('quat', '1 0 0 0'))
    r = [x / math.sqrt(sum(v*v for v in r)) for x in r]
    pos = [a+b for a, b in zip(p, rot(q, vec(body.get('pos', '0 0 0'))))]
    quat = mul(q, r)
    result[body.get('name')] = (pos, quat)
    for child in body.findall('body'):
      walk(child, pos, quat)

  for body in root.find('worldbody').findall('body'):
    walk(body, [0, 0, 0], [1, 0, 0, 0])
  return result


def generate(source, out, torquescale, passive_armature=None, weld_impedance=None, aligned_base=False):
  out.mkdir(parents=True, exist_ok=True)
  digest = hashlib.sha256((source / '2f85.xml').read_bytes()).hexdigest()
  if digest != SOURCE_SHA256:
    raise ValueError('Source XML differs from the pinned Menagerie model')
  root = ET.parse(source / '2f85.xml').getroot()
  if passive_armature is not None:
    for cls in ['follower', 'spring_link', 'coupler']:
      root.find(f"./default/default/default[@class='{cls}']/joint").set('armature', str(passive_armature))
  # Relative paths let the generated folder and source_assets move together.
  import os
  root.find('compiler').set('meshdir', os.path.relpath(source / 'assets', out))
  opt = root.find('option')
  opt.set('integrator', 'implicitfast')
  opt.set('iterations', '100')
  opt.set('tolerance', '1e-10')
  opt.set('jacobian', 'dense')
  ET.SubElement(opt, 'flag', contact='disable', energy='enable')
  body = {b.get('name'): b for b in root.iter('body')}
  if aligned_base:
    body['base'].set('quat', '1 0 0 0')
  frames = poses(root)
  for side in ['right', 'left']:
    f, c = side + '_follower', side + '_coupler'
    pf, qf = frames[f]
    pc, qc = frames[c]
    anchor = rot(inv(qc), [a-b for a, b in zip(pf, pc)])
    ET.SubElement(body[f], 'site', name=side+'_closure_f', pos='0 0 0', size='.001')
    ET.SubElement(body[c], 'site', name=side+'_closure_c', pos=fmt(anchor), size='.001')
    ET.SubElement(body[side+'_pad'], 'site', name=side+'_pad_center',
                  pos='0 -0.0025 0.0185', size='.001')
  ET.indent(root, space='  ')
  ET.ElementTree(root).write(out / 'connect.xml', encoding='unicode')

  for side in ['right', 'left']:
    f, c, s = side+'_follower', side+'_coupler', side+'_shadow'
    pf, qf = frames[f]
    pc, qc = frames[c]
    original = body[c]
    assert not original.findall('body'), 'Only leaf couplers can be split here'
    inertia = original.find('inertial')
    for key in ['mass', 'diaginertia']:
      inertia.set(key, fmt([x/2 for x in vec(inertia.get(key))]))
    shadow = ET.SubElement(body[f], 'body', name=s,
                          pos=fmt(rot(inv(qf), [a-b for a, b in zip(pc, pf)])),
                          quat=fmt(mul(inv(qf), qc)))
    shadow.append(copy.deepcopy(inertia))
    # Restore the cut physical hinge. Do not duplicate coupler armature/damping.
    ET.SubElement(shadow, 'joint', name=side+'_closure_hinge', type='hinge',
                  pos=fmt(rot(inv(qc), [a-b for a, b in zip(pf, pc)])),
                  axis=fmt(rot(mul(inv(qc), qf), [1, 0, 0])),
                  limited='false', armature='0', damping='0', stiffness='0')
    for geom in original.findall('geom'):
      if geom.get('class') == 'visual':
        visual = copy.deepcopy(geom)
        visual.attrib.pop('material', None)
        visual.set('rgba', '0.2 0.7 1 0.28')
        shadow.append(visual)
    ET.SubElement(original, 'site', name=side+'_weld_c', pos=inertia.get('pos'), size='.001')
    ET.SubElement(shadow, 'site', name=side+'_weld_s', pos=inertia.get('pos'), size='.001')
    eq = root.find('equality')
    connect = next(e for e in eq.findall('connect') if e.get('body1') == f)
    attrs = {key: connect.get(key) for key in ['solref', 'solimp']}
    if weld_impedance is not None:
      attrs['solimp'] = f'{weld_impedance} {weld_impedance} 0.001'
    eq.remove(connect)
    ET.SubElement(eq, 'weld', name=side+'_shadow_weld', site1=side+'_weld_c',
                  site2=side+'_weld_s', torquescale=str(torquescale), **attrs)
    for other in [c, side+'_driver', side+'_spring_link']:
      ET.SubElement(root.find('contact'), 'exclude', body1=s, body2=other)
  ET.indent(root, space='  ')
  ET.ElementTree(root).write(out / 'shadow.xml', encoding='unicode')
  (out / 'provenance.json').write_text(json.dumps({
      'repository': 'https://github.com/google-deepmind/mujoco_menagerie',
      'commit': MENAGERIE_COMMIT, 'model': 'robotiq_2f85/2f85.xml',
      'source_xml_sha256': digest,
      'shadow_torquescale_m': torquescale,
      'passive_armature_override': passive_armature,
      'weld_impedance_override': weld_impedance,
      'aligned_base': aligned_base,
      'notes': 'Original model, not v4. Actuation retained. Null overrides retain source defaults.'
  }, indent=2) + '\n')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('source', type=pathlib.Path)
  parser.add_argument('out', type=pathlib.Path)
  parser.add_argument('--torquescale', type=float, default=1.)
  parser.add_argument('--passive-armature', type=float)
  parser.add_argument('--weld-impedance', type=float)
  parser.add_argument('--aligned-base', action='store_true')
  args = parser.parse_args()
  generate(args.source.resolve(), args.out.resolve(), args.torquescale,
           args.passive_armature, args.weld_impedance, args.aligned_base)
