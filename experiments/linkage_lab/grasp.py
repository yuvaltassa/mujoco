"""Generate and run a released-box grasp-and-shake experiment."""
import argparse
import hashlib
import json
import os
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from generate import fmt, generate, poses, rot

BASE = pathlib.Path(__file__).resolve().parent
ROOT = BASE.parents[1]


def model(source, output, closure, scale=1, impedance=None, mass=.25):
  generate(source, output, scale, weld_impedance=impedance)
  path = output/(closure+'.xml')
  root = ET.parse(path).getroot()
  root.find('./option/flag').set('contact','enable')
  root.find('./worldbody/body').set('pos','0 0 .35')
  root.find('./worldbody/body').set('quat','0 1 0 0')
  p,q = poses(root)['base']
  center = [a+b for a,b in zip(p,rot(q,[0,0,.145]))]
  world = root.find('worldbody')
  obj = ET.SubElement(world,'body',name='object',pos=fmt(center))
  ET.SubElement(obj,'freejoint',name='object_joint')
  ET.SubElement(obj,'geom',name='object_geom',type='box',size='.02 .012 .02',mass=str(mass),
                friction='.7 .005 .0001',rgba='.95 .55 .12 1')
  # A temporary fixture allows controlled placement during closing; release at 1.2 s.
  ET.SubElement(root.find('equality'),'weld',name='fixture',body1='object',
                solref='.002 1',solimp='.999 .999 .001')
  ET.SubElement(world,'light',pos='.3 -.4 .6',dir='-.3 .4 -.4',diffuse='.8 .8 .8')
  ET.SubElement(world,'geom',type='plane',size='1 1 .01',contype='0',conaffinity='0',rgba='.18 .2 .24 1')
  visual = ET.SubElement(root,'visual')
  ET.SubElement(visual,'global',offwidth='1200',offheight='800')
  ET.SubElement(visual,'headlight',ambient='.4 .4 .4')
  ET.indent(root,space='  ')
  path = output/'grasp.xml'
  ET.ElementTree(root).write(path,encoding='unicode')
  return path


def outcome(metrics):
  if metrics['failed']:
    return 'numerical failure'
  if metrics['dropped']:
    return 'dropped'
  if not metrics['held_before_shake']:
    return 'no initial grasp'
  if metrics.get('other_object_contacts_max', 0):
    return 'non-pad contact'
  if metrics.get('slip_peak_mm', float('inf')) > 5 or metrics.get('angle_peak_deg', 0) > 10:
    return 'large object motion'
  if metrics.get('closure_peak_mm', float('inf')) > 1:
    return 'large loop error'
  return 'clean hold'


def run(path, output, precision='double', exact=0, peak=0, axis=2, control=255,
        dt=.002, mode='raw', frequency=5):
  output.mkdir(parents=True,exist_ok=True)
  runner = ROOT/('build-linkage-'+precision)/'grasp_runner'
  csv = output/'trace.csv'
  env = dict(os.environ,LINKAGE_REGULARIZATION=mode,LINKAGE_FLOOR='1e-6')
  command = [str(runner),str(path),str(csv),str(exact),str(dt),str(peak),str(axis),str(control),str(frequency),'4']
  proc = subprocess.run(command,env=env,capture_output=True,text=True)
  if proc.returncode not in [0,2]:
    raise RuntimeError(proc.stderr+proc.stdout)
  status = json.loads(proc.stdout.strip().splitlines()[-1])
  assert status['num_bytes'] == (8 if precision == 'double' else 4)
  data = np.genfromtxt(csv,delimiter=',',names=True)
  held = data[(data['time'] >= 1.8)&(data['time']<2)]
  active = data[data['time']>=2]
  metrics = dict(precision=precision,exact=exact,peak=peak,axis=axis,control=control,dt=dt,
                 mode=mode,frequency=frequency,**status,end_time=float(data['time'][-1]))
  metrics['held_before_shake'] = bool(len(held) and np.min(held['right_normal_N'])>0 and np.min(held['left_normal_N'])>0)
  if len(active):
    points=np.column_stack([active['object_'+a] for a in 'xyz'])
    baseline=np.array([np.mean(held['object_'+a]) for a in 'xyz']) if len(held) else points[0]
    metrics.update(slip_peak_mm=float(np.max(np.linalg.norm(points-baseline,axis=1)))*1000,
                   angle_peak_deg=float(np.max(active['angle_rad']))*180/np.pi,
                   closure_peak_mm=float(np.max(active['closure_m']))*1000,
                   normal_min_N=float(min(np.min(active['right_normal_N']),np.min(active['left_normal_N']))),
                   normal_mean_N=float(np.mean(active['right_normal_N']+active['left_normal_N'])),
                   penetration_max_mm=float(np.max(active['penetration_m']))*1000,
                   other_object_contacts_max=int(np.max(active['other_object_contacts'])),
                   self_contacts_max=int(np.max(active['self_contacts'])))
  metrics['outcome'] = outcome(metrics)
  binaries = [runner, *sorted((runner.parent/'lib').glob('*mujoco*'))]
  hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [path, *binaries] if p.is_file()}
  (output/'metadata.json').write_text(json.dumps(dict(command=command,model=str(path),sha256=hashes,metrics=metrics,stderr=proc.stderr),indent=2)+'\n')
  return metrics


if __name__ == '__main__':
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('source',type=pathlib.Path)
  p.add_argument('--closure',choices=['connect','shadow'],default='connect')
  p.add_argument('--scale',type=float,default=1)
  p.add_argument('--impedance',type=float)
  p.add_argument('--mass',type=float,default=.25)
  p.add_argument('--precision',choices=['double','single'],default='double')
  p.add_argument('--exact',type=int,choices=[0,1],default=0)
  p.add_argument('--peak',type=float,default=0)
  p.add_argument('--axis',type=int,default=2)
  p.add_argument('--control',type=float,default=255)
  p.add_argument('--dt',type=float,default=.002)
  p.add_argument('--mode',choices=['raw','floor','block'],default='raw')
  p.add_argument('--name',default='preview')
  a=p.parse_args()
  path=model(a.source.resolve(),BASE/'generated/grasp'/a.name,a.closure,a.scale,a.impedance,a.mass)
  print(json.dumps(run(path,BASE/'raw/grasp'/a.name,a.precision,a.exact,a.peak,a.axis,a.control,a.dt,a.mode),indent=2))
