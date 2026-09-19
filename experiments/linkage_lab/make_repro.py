"""Remove mesh dependencies while preserving the compiled body inertias."""
import argparse
import json
import pathlib
import subprocess
import xml.etree.ElementTree as ET

from generate import fmt


def make(runner, models, output):
  output.mkdir(parents=True, exist_ok=True)
  for name in ['connect', 'shadow']:
    source = models/(name+'.xml')
    snapshot = json.loads(subprocess.check_output([str(runner), str(source), '--audit'], text=True))
    inertias = {b['name']: b for b in snapshot['bodies']}
    root = ET.parse(source).getroot()
    root.remove(root.find('asset'))
    root.find('compiler').attrib.pop('meshdir', None)
    root.find('./option/flag').set('diagexact', 'enable')
    for body in root.iter('body'):
      data = inertias[body.get('name')]
      if body.find('inertial') is None and data['mass'] > 0:
        ET.SubElement(body, 'inertial', mass=str(data['mass']), pos=fmt(data['local_com']),
                      quat=fmt(data['local_iquat']), diaginertia=fmt(data['inertia']))
      for geom in list(body.findall('geom')):
        if 'mesh' in geom.attrib:
          body.remove(geom)
    ET.indent(root, space='  ')
    dest = output/(name+'_exact.xml')
    dest.write_text('<!-- Derived from Menagerie Robotiq 2F-85; see ROBOTIQ_LICENSE. -->\n'
                    + ET.tostring(root, encoding='unicode') + '\n')
    check = json.loads(subprocess.check_output([str(runner), str(dest), '--audit'], text=True))
    error = max(abs(a-b) for a, b in zip(snapshot['mass_matrix'], check['mass_matrix']))
    if error > 1e-12:
      raise ValueError(f'Mass matrix changed when stripping meshes: {error}')
    print(dest, 'mass matrix error', error)


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('runner', type=pathlib.Path)
  p.add_argument('models', type=pathlib.Path)
  p.add_argument('output', type=pathlib.Path)
  a = p.parse_args()
  make(a.runner.resolve(), a.models.resolve(), a.output.resolve())
