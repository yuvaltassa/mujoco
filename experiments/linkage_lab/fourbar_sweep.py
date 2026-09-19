"""Force-transfer, precision, and massless-rod experiments using native MuJoCo."""
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np

from fourbar import A, B, G, L, MA, MB, RADIUS, BASE, ROOT, model, geometry, effective_inertia

RAW = BASE/'raw'/'fourbar'
GENERATED = BASE/'generated'/'fourbar'
RESULTS = BASE/'results'
COUNTER = 0
RECORDS = []


def native(closure, *, rows=1, precision='double', exact=1, theta=90, mass=.2,
           impedance=.99, timeconst=.01, damping=1, rotation=90, mode='static', dt=.0005,
           load=10, duration=2, frequency=1, drive_kv=5, group='pilot', keep=False):
  global COUNTER
  COUNTER += 1
  name = f'{COUNTER:04d}_{group}_{closure}_{precision}_{rows}'
  path = model(GENERATED/f'{name}.xml', closure, theta, mass, impedance, timeconst, rotation, damping=damping)
  output = RAW/f'{name}.csv'
  args = [str(ROOT/f'build-linkage-{precision}'/'fourbar_runner'), str(path), str(output),
          mode, str(rows), str(exact), str(dt), str(load), str(duration), str(frequency), str(drive_kv)]
  process = subprocess.run(args, capture_output=True, text=True)
  objects = [line for line in process.stdout.splitlines() if line.startswith('{')]
  if not objects:
    raise RuntimeError(f'{args}: {process.stderr} {process.stdout}')
  result = json.loads(objects[-1])
  if process.returncode not in (0, 2):
    raise RuntimeError(process.stderr)
  x = np.genfromtxt(output, delimiter=',', names=True)
  result.update(group=group, closure=closure, rows_mode=rows, precision=precision, exact=exact,
                theta=theta, mass=mass if closure!='tendon' else 0, impedance=impedance,
                timeconst=timeconst, damping=damping, rotation=rotation, mode=mode, dt=dt, load=load,
                duration=duration, frequency=frequency, drive_kv=drive_kv)
  if len(x)>2:
    result.update(metrics(x, result))
  if keep:
    trace = RESULTS/f'fourbar_trace_{group}_{closure}.csv'
    np.savetxt(trace, np.column_stack([x[n][::max(1,round(.002/dt))] for n in x.dtype.names]), delimiter=',',
               header=','.join(x.dtype.names), comments='', fmt='%.9g')
    result['trace'] = trace.name
  RECORDS.append(result)
  return result, x


def rigid_inertia(theta, mass):
  """Vectorized H(theta), independently checked against native mass matrices."""
  bx, by = A*np.cos(theta), A*np.sin(theta)
  vx, vy = G-bx, -by
  distance = np.hypot(vx,vy)
  along = (L*L-B*B+distance*distance)/(2*distance)
  height = np.sqrt(np.maximum(0,L*L-along*along))
  cx,cy = bx+along*vx/distance-height*vy/distance, by+along*vy/distance+height*vx/distance
  phi = np.arctan2(cy,cx-G)
  rx,ry = cx-bx,cy-by
  dx,dy = -A*np.sin(theta),A*np.cos(theta)
  tx,ty = -B*np.sin(phi), B*np.cos(phi)
  ratio = (rx*dx+ry*dy)/(rx*tx+ry*ty)
  psid = (rx*(ty*ratio-dy)-ry*(tx*ratio-dx))/(L*L)
  H = MA*(A*A/3+RADIUS*RADIUS/4)+MB*(B*B/3+RADIUS*RADIUS/4)*ratio**2
  H += mass*((dx+tx*ratio)**2+(dy+ty*ratio)**2)/4
  H += mass*(L*L+3*RADIUS*RADIUS)/12*psid**2
  return H


def metrics(x, p):
  t=x['time']; late=t>max(1,p['duration']-.5)
  if not np.any(late): late=t>.5*t[-1]
  out=dict(max_closure_m=float(np.max(x['closure_m'])),
           deflection_m=float(np.mean(x['deflection_m'][late])),
           input_torque_Nm=float(np.mean(x['input_torque'][late])),
           static_reference_Nm=float(np.mean(x['static_reference_torque'][late])),
           static_error_Nm=float(np.mean(x['input_torque'][late]-x['static_reference_torque'][late])))
  if p['mode']=='calibrate':
    plateau=(t>1.1)&(t<1.24); release=t>=1.25
    if np.any(plateau) and np.any(release):
      C=abs(np.mean(x['deflection_m'][plateau]))/abs(p['load'])
      out['compliance_m_N']=float(C)
      out['decay_s']=float(np.trapezoid(abs(x['deflection_m'][release]),t[release])/(C*abs(p['load'])))
  theta=x['input_angle']; mass=p['mass']
  H=rigid_inertia(theta,mass)
  Hprime=(rigid_inertia(theta+1e-5,mass)-rigid_inertia(theta-1e-5,mass))/2e-5
  ref=H*x['input_acceleration']+.5*Hprime*x['input_velocity']**2+x['static_reference_torque']
  window=t>=1 if p['mode']=='dynamic' else np.ones(len(t),dtype=bool)
  if not np.any(window): window=np.ones(len(t),dtype=bool)
  out['inverse_dynamics_error_Nm']=float(np.sqrt(np.mean((x['input_torque'][window]-ref[window])**2)))
  out['reference_rms_Nm']=float(np.sqrt(np.mean(ref[window]**2)))
  for field,key in [('input_power_W','input_work_J'),('output_power_W','output_work_J'),('equality_power_W','equality_work_J')]:
    out[key]=float(np.trapezoid(x[field],t))
  out['energy_change_J']=float(x['kinetic_J'][-1]-x['kinetic_J'][0])
  out['energy_initial_J']=float(x['kinetic_J'][0])
  out['work_balance_error_J']=out['energy_change_J']-out['input_work_J']-out['output_work_J']-out['equality_work_J']
  out['deflection_rms_m']=float(np.sqrt(np.mean(x['deflection_m'][window]**2)))
  return out


def audit():
  records=[]
  for precision in ['double','single']:
    for rotation in [0,37,90]:
      for closure in ['connect','weld','tendon']:
        for rows in ([0,1] if closure!='tendon' else [0]):
          path=model(GENERATED/'audit.xml',closure,rotation=rotation)
          exe=ROOT/f'build-linkage-{precision}'/'fourbar_runner'
          a=json.loads(subprocess.check_output([str(exe),str(path),'--audit',str(rows),'1']))
          J=np.array(a['J']).reshape(a['ne'],a['nv']); M=np.array(a['M']).reshape(a['nv'],a['nv'])
          phi,psi,ratio,psid,*_=geometry(np.pi/2)
          v=np.array([dict(input=1,output=ratio,coupler=psid-1,shadow=psid-ratio)[j] for j in a['joints']])
          H=effective_inertia(np.pi/2,.2 if closure!='tendon' else 0)
          sv=np.linalg.svd(J,compute_uv=False)
          rank=int(sum(sv>sv[0]*(1e-5 if precision=='single' else 1e-12)))
          assert rank==a['nv']-1,(closure,precision,rotation,rows,sv)
          assert abs(v@M@v/H-1)<(2e-6 if precision=='single' else 1e-12)
          assert max(abs(J@v))<(2e-7 if precision=='single' else 1e-12)
          assert abs(rigid_inertia(np.pi/2,.2 if closure!='tendon' else 0)/H-1)<1e-12
          records.append(dict(closure=closure,precision=precision,rotation=rotation,rows=rows,
                              nv=a['nv'],ne=a['ne'],rank=rank,Jv_max=float(max(abs(J@v))),
                              inertia_relative_error=float(v@M@v/H-1),singular_values=sv.tolist()))
  return records


def audit_toggle():
  theta=math.acos(((A+L)**2+G*G-B*B)/(2*G*(A+L)))
  records=[]
  for closure in ['connect','weld','tendon']:
    path=model(GENERATED/'toggle.xml',closure,theta=math.degrees(theta))
    a=json.loads(subprocess.check_output([str(ROOT/'build-linkage-double'/'fourbar_runner'),
                                         str(path),'--audit','1','1']))
    J=np.array(a['J']).reshape(a['ne'],a['nv'])
    rank=int(np.linalg.matrix_rank(J))
    assert rank==a['nv']-1
    assert abs(geometry(theta)[2])<1e-14
    records.append(dict(closure=closure,theta=math.degrees(theta),nv=a['nv'],rows=a['ne'],rank=rank))
  return records


def calibrate(closure, target, mass=.2, flexible_damping=False):
  """Fit two measured physical responses, with explicit impedance/time bounds."""
  x=np.log([.5 if flexible_damping else .01,.01]); history=[]
  def evaluate(z):
    p=dict(impedance=.99989999 if flexible_damping else 1-math.exp(z[0]),
           timeconst=math.exp(z[1]),mass=mass,damping=math.exp(z[0]) if flexible_damping else 1)
    r,_=native(closure,group='calibration',mode='calibrate',load=1,**p)
    if r['failed']: raise RuntimeError('calibration failed')
    y=np.log([r['compliance_m_N']/target[0],r['decay_s']/target[1]])
    history.append(dict(**p,compliance=r['compliance_m_N'],decay=r['decay_s']))
    return y
  bounds=(np.log([.02 if flexible_damping else .00010001,.002]),
          np.log([5 if flexible_damping else .5,.05]))
  if mass and mass<.001 and not flexible_damping:
    candidates=[np.log([d,tau]) for d in [.00010001,.0002,.0005,.001,.01]
                for tau in [.002,.004,.006,.01,.02]]
    scores=[np.linalg.norm(evaluate(z)) for z in candidates]
    x=candidates[int(np.argmin(scores))]
  for iteration in range(10):
    y=evaluate(x)
    if max(abs(y))<.003: break
    J=np.column_stack([(evaluate(x+np.eye(2)[j]*.03)-y)/.03 for j in range(2)])
    delta=np.clip(np.linalg.lstsq(J,-y,rcond=None)[0],-1.5,1.5)
    improved=False
    for alpha in [1,.5,.25,.125]:
      trial=np.clip(x+alpha*delta,*bounds)
      if np.linalg.norm(evaluate(trial))<np.linalg.norm(y):
        x=trial;improved=True;break
    if not improved: break
  y=evaluate(x)
  if max(abs(y))>=.02 and not flexible_damping:
    fallback=calibrate(closure,target,mass,True)
    fallback['unit_damping_fit']=history
    return fallback
  return dict(impedance=.99989999 if flexible_damping else 1-math.exp(x[0]),
              timeconst=math.exp(x[1]),mass=mass,damping=math.exp(x[0]) if flexible_damping else 1,
              matched=bool(max(abs(y))<.02),relative_errors=np.expm1(y).tolist(),history=history)


def validate(payload):
  """Physical acceptance checks; precision failures remain intentional diagnostics."""
  records=payload['records']
  assert all(not r['failed'] for r in records), 'Simulation warning or divergence'
  assert all(f['matched'] for f in payload['fits'].values()), 'Unmatched calibration'
  assert max(abs(r['static_error_Nm']) for r in records if r['group']=='static')<.001
  for r in records:
    if r['group']=='drive_control' and r['drive_kv']==0:
      assert r['inverse_dynamics_error_Nm']/r['reference_rms_Nm']<.001
  for r in payload['equivalence']:
    assert r['max_angle_difference']<1e-9 and r['max_torque_difference']<1e-8
  print('Physical calibration, loaded virtual work, inverse dynamics, and row-equivalence checks passed',flush=True)


def main():
  for path in [RAW,GENERATED,RESULTS]: path.mkdir(parents=True,exist_ok=True)
  audits=audit();print('30 geometry / inertia / rank audits passed',flush=True)
  baseline,_=native('connect',mode='calibrate',load=1,group='calibration')
  target=[baseline['compliance_m_N'],baseline['decay_s']]
  fits={}
  for closure,mass in [('connect',.2),('weld',.2),('tendon',0),('connect',.02),('connect',.002),('connect',.0002)]:
    key=f'{closure}_{mass}'
    fits[key]=calibrate(closure,target,mass)
    print('fit',key, {k:v for k,v in fits[key].items() if k not in ['history','unit_damping_fit']},flush=True)
  def parameters(closure,mass=.2):
    return {k:fits[f'{closure}_{mass if closure!="tendon" else 0}'][k] for k in ['mass','impedance','timeconst','damping']}
  for closure in ['connect','weld','tendon']:
    native(closure,group='matched_calibration',mode='calibrate',load=1,keep=True,**parameters(closure))
    model(BASE/'repro'/f'fourbar_{closure}.xml',closure,**parameters(closure))
  native('connect',group='light_calibration',mode='calibrate',load=1,keep=True,**parameters('connect',.0002))
  # Force-bearing precision/orientation controls, including a diagnostic floor.
  for precision in ['double','single']:
    for rotation in [0,37,90]:
      for closure in ['connect','weld','tendon']:
        for rows in ([0,1,2] if closure!='tendon' else [0]):
          for exact in [0,1]:
            native(closure,group='precision',precision=precision,rotation=rotation,rows=rows,exact=exact,
                   **parameters(closure))
  print('84 loaded precision/orientation controls finished',flush=True)
  for closure in ['connect','weld','tendon']:
    for theta in [36,38,60,90,120]:
      for load in [-10,-1,1,10]:
        native(closure,group='static',theta=theta,load=load,**parameters(closure))
    for frequency in [.5,2,5]:
      for dt in [.001,.0005,.00025]:
        native(closure,group='dynamic',mode='dynamic',frequency=frequency,dt=dt,load=1,duration=3,
               **parameters(closure))
    for dt in [.001,.0005,.00025]:
      native(closure,group='passive',mode='passive',dt=dt,duration=1,load=0,**parameters(closure))
  print('Loaded configurations, dynamic transmission, and energy controls finished',flush=True)
  _,tendon_reference=native('tendon',group='mass_reference',mode='dynamic',load=1,frequency=2,
                            drive_kv=0,duration=3,**parameters('tendon'))
  for mass in [.2,.02,.002,.0002]:
    for tuned in [False,True]:
      p=parameters('connect',mass) if tuned else dict(mass=mass)
      native('connect',group='mass_tuned' if tuned else 'mass_fixed',mode='calibrate',load=1,**p)
      r,x=native('connect',group='mass_tuned' if tuned else 'mass_fixed',mode='dynamic',load=1,
             drive_kv=0,frequency=2,duration=3,**p)
      window=x['time']>=1
      for field,metric in [('input_torque','torque_difference_from_tendon_Nm'),
                           ('output_angle','angle_difference_from_tendon_rad')]:
        r[metric]=float(np.sqrt(np.mean((x[field][window]-tendon_reference[field][window])**2)))
  # Refine time and actuator damping independently: integration can dominate closure error.
  for closure in ['connect','weld','tendon']:
    for drive_kv in [0,.5,5]:
      for dt in [.0005,.000125,.00003125]:
        native(closure,group='drive_control',mode='dynamic',load=1,frequency=2,duration=3,
               drive_kv=drive_kv,dt=dt,keep=drive_kv==0 and dt==.0005,**parameters(closure))
  # Direct full-vs-planar trajectory checks in double, aligned plane.
  equivalence=[]
  for closure in ['connect','weld']:
    r,a=native(closure,group='equivalence',mode='dynamic',rotation=0,rows=0,load=1,duration=3,frequency=2,**parameters(closure))
    r,b=native(closure,group='equivalence',mode='dynamic',rotation=0,rows=1,load=1,duration=3,frequency=2,**parameters(closure))
    equivalence.append(dict(closure=closure,max_angle_difference=float(max(abs(a['input_angle']-b['input_angle']))),
                            max_torque_difference=float(max(abs(a['input_torque']-b['input_torque'])))))
  binaries={}
  for precision in ['double','single']:
    exe=ROOT/f'build-linkage-{precision}'/'fourbar_runner'
    library=ROOT/f'build-linkage-{precision}'/'lib'/'libmujoco.3.13.1.dylib'
    binaries[precision]={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [exe,library]}
  payload=dict(binaries=binaries,source_head=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
               target_compliance_m_N=target[0],target_decay_s=target[1],fits=fits,audits=audits,toggle_audits=audit_toggle(),
               equivalence=equivalence,records=RECORDS)
  validate(payload)
  (RESULTS/'fourbar.json').write_text(json.dumps(payload,indent=2)+'\n')
  fields=sorted(set().union(*(r.keys() for r in RECORDS)))
  with (RESULTS/'fourbar.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(RECORDS)
  print(f'Saved {len(RECORDS)} runs; equivalence={equivalence}',flush=True)


if __name__=='__main__': main()
