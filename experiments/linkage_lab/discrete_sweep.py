"""Native integrator comparison with force-bearing equalities and Robotiq grasps."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from fourbar import BASE, ROOT, model
from grasp import run as grasp_run, model as grasp_model

RESULTS=BASE/'results'
RAW=BASE/'raw'/'discrete'
MODELS=BASE/'generated'/'discrete'
ROWS=[]
COUNT=0
FITS=json.loads((RESULTS/'fourbar.json').read_text())['fits']


def run(closure,variant,dt,scale=1,mode='dynamic',precision='double',exact=1):
  global COUNT
  COUNT+=1
  key=f'{COUNT:04d}_{closure}_{variant}_{mode}_{scale}_{dt}_{precision}_{exact}'
  fit=FITS[closure+('_0' if closure=='tendon' else '_0.2')]
  path=model(MODELS/(key+'.xml'),closure,rotation=0,impedance=fit['impedance'],
             timeconst=fit['timeconst']*scale)
  root=ET.parse(path).getroot()
  root.find('option').set('integrator','discrete' if variant=='discrete' else 'implicitfast')
  if variant=='unclamped':root.find('option/flag').set('refsafe','disable')
  ET.ElementTree(root).write(path,encoding='unicode')
  output=RAW/(key+'.csv')
  args=[str(ROOT/f'build-linkage-{precision}'/'fourbar_runner'),str(path),str(output),mode,
        '0',str(exact),str(dt),'1' if mode=='dynamic' else '10','3' if mode=='dynamic' else '2','2','5']
  process=subprocess.run(args,capture_output=True,text=True)
  if process.returncode not in (0,2):raise RuntimeError(process.stdout+process.stderr)
  status=json.loads([line for line in process.stdout.splitlines() if line.startswith('{')][-1])
  x=np.genfromtxt(output,delimiter=',',names=True)
  row=dict(closure=closure,variant=variant,dt=dt,scale=scale,mode=mode,precision=precision,exact=exact,
           timeconst=fit['timeconst']*scale,**status)
  row['end_time']=float(x['time'][-1])
  if len(x)>2 and not row['failed']:
    mask=x['time']>=1.5
    row['deflection_rms_mm']=float(np.sqrt(np.mean(x['deflection_m'][mask]**2))*1000)
    row['closure_peak_mm']=float(np.max(x['closure_m'][mask])*1000)
    row['static_torque_error_Nm']=float(np.mean((x['input_torque']-x['static_reference_torque'])[mask]))
    row['compliance_um_N']=float(abs(np.mean(x['deflection_m'][mask]))*1e6/10) if mode=='static' else None
    # Per-step delivered work: output torque is held constant throughout each step.
    row['delivered_work_J']=float(-np.sum(x['output_torque'][:-1]*np.diff(x['output_angle'])))
    if variant=='discrete':
      row['step_acceleration_error']=float(np.max(abs(np.diff(x['input_velocity'])/dt-x['input_acceleration'][:-1])))
  ROWS.append(row)
  return row,x


def compare(row,x,reference):
  if row['failed']:return
  mask=(x['time']>=1)&(x['time']<=reference['time'][-1])
  for field,key,unit in [('output_angle','output_angle_error_mrad',1000),
                         ('output_velocity','output_load_error_Nm',1),
                         ('input_torque','requested_torque_error_Nm',1)]:
    expected=np.interp(x['time'][mask],reference['time'],reference[field])
    row[key]=float(np.sqrt(np.mean((x[field][mask]-expected)**2))*unit)
  design=np.column_stack([np.sin(4*np.pi*x['time'][mask]),np.cos(4*np.pi*x['time'][mask]),np.ones(sum(mask))])
  actual=np.linalg.lstsq(design,x['output_angle'][mask],rcond=None)[0]
  expected=np.linalg.lstsq(design,np.interp(x['time'][mask],reference['time'],reference['output_angle']),rcond=None)[0]
  phase=np.arctan2(actual[1],actual[0])-np.arctan2(expected[1],expected[0])
  row['output_phase_error_deg']=float(np.degrees(np.arctan2(np.sin(phase),np.cos(phase))))
  row['output_amplitude_error_percent']=float(100*(np.hypot(*actual[:2])/np.hypot(*expected[:2])-1))


def grasps(robotiq_source=None):
  results=[]
  for closure,variant in [('connect','connect'),('shadow','shadow-stiff')]:
    source=BASE/'generated/grasp'/f'{variant}-m1.0'/'grasp.xml'
    if robotiq_source is not None or not source.exists():
      assets=robotiq_source or BASE/'source_assets/robotiq_2f85'
      source=grasp_model(assets,MODELS/f'grasp_source_{closure}',closure,1,
                         .999 if closure=='shadow' else None,1.)
    for integrator in ['implicitfast','discrete']:
      tree=ET.parse(source);root=tree.getroot()
      root.find('option').set('integrator',integrator)
      # Resolve all mesh references before writing the integrator variant elsewhere.
      compiler=root.find('compiler')
      for name in ['meshdir','texturedir']:
        if name in compiler.attrib:compiler.set(name,str((source.parent/compiler.get(name)).resolve()))
      path=MODELS/f'grasp_{closure}_{integrator}.xml';tree.write(path,encoding='unicode')
      for exact in [0,1]:
        for peak in [0,60]:
          for dt in [.0005,.002,.004,.008,.016]:
            key=f'grasp_{closure}_{integrator}_{exact}_{peak}_{dt}'
            result=grasp_run(path,RAW/key,exact=exact,peak=peak,axis=2,dt=dt)
            results.append(dict(closure=closure,integrator=integrator,**result))
    print('Grasp controls finished:',closure,flush=True)
  return results


def main(robotiq_source=None):
  RAW.mkdir(parents=True,exist_ok=True);MODELS.mkdir(parents=True,exist_ok=True)
  for closure in ['connect','weld','tendon']:
    for scale in [1,.1]:
      # Two fine steps give an empirical reference-discretization error.
      _,reference=run(closure,'implicitfast',.00003125,scale)
      for integrator in ['implicitfast','discrete']:
        r,x=run(closure,integrator,.0000625,scale)
        compare(r,x,reference)
      for variant in ['implicitfast','unclamped','discrete']:
        for dt in [.000125,.0005,.002,.004,.008,.016]:
          for mode in ['static','dynamic']:
            r,x=run(closure,variant,dt,scale,mode)
            if mode=='dynamic':compare(r,x,reference)
      print('Four-bar sweep finished:',closure,scale,flush=True)
  for closure in ['connect','weld','tendon']:
    for variant in ['implicitfast','discrete']:
      for precision in ['single','double']:
        for exact in [0,1]:
          for dt in [.0005,.008]:run(closure,variant,dt,.1,precision=precision,exact=exact)
  grasp=grasps(robotiq_source)
  fingerprints={}
  for precision in ['single','double']:
    for name in ['fourbar_runner','grasp_runner','lib/libmujoco.3.13.1.dylib']:
      path=ROOT/f'build-linkage-{precision}'/name
      fingerprints[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
  p=dict(fingerprints=fingerprints,fourbar=ROWS,grasp=grasp)
  (RESULTS/'discrete.json').write_text(json.dumps(p,indent=2)+'\n')
  print('Saved',len(ROWS),'four-bar runs and',len(grasp),'grasp runs',flush=True)


if __name__=='__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--robotiq-source',type=Path)
  main(parser.parse_args().robotiq_source)
