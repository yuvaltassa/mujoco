"""Run controlled grasp tests across closure, diagonal, load and shake direction."""
import argparse
import itertools
import json
import pathlib

from grasp import BASE, model, run

VARIANTS = [
    ('connect','connect',1,None),
    ('shadow-small','shadow',.03,None),
    ('shadow-default','shadow',1,None),
    ('shadow-stiff-small','shadow',.03,.999),
    ('shadow-stiff','shadow',1,.999),
]


def sweep(source, pilot=False):
  rows=[]
  if pilot:
    variants=VARIANTS[:1]
    scenarios=list(itertools.product([.25,1.],[128,160,200,255],[0,60,100]))
  else:
    variants=VARIANTS
    scenarios=list(itertools.product([.25,1.],[255],[0,30,60,100]))
  for name,closure,scale,impedance in variants:
    paths={mass:model(source,BASE/'generated/grasp'/f'{name}-m{mass}',closure,scale,impedance,mass)
           for mass in [.25,1.]}
    for mass,control,peak in scenarios:
      for exact,axis in itertools.product([0] if pilot else [0,1], [2] if peak==0 or pilot else [1,2]):
        label=f'{name}-m{mass}-c{control}-a{axis}-p{peak}-e{exact}'
        out=BASE/'raw/grasp'/label
        result=run(paths[mass],out,exact=exact,peak=peak,axis=axis,control=control)
        row=dict(variant=name,mass=mass,scale=scale,impedance=impedance,label=label,**result)
        rows.append(row)
        print(label,result['outcome'],
              result.get('slip_peak_mm'),flush=True)
  if not pilot:
    for name,closure,scale,impedance in [VARIANTS[0],VARIANTS[-1]]:
      for precision,mode,exact in [('single','raw',0),('single','raw',1),('single','floor',1),('double','floor',1)]:
        for mass,peak in itertools.product([.25,1.],[0,60]):
          path=BASE/'generated/grasp'/f'{name}-m{mass}'/'grasp.xml'
          label=f'{name}-m{mass}-p{peak}-{precision}-{mode}-e{exact}'
          result=run(path,BASE/'raw/grasp'/label,precision,exact,peak,2,255,.002,mode)
          rows.append(dict(variant=name,mass=mass,scale=scale,impedance=impedance,label=label,**result))
          print(label,result['outcome'],
                result.get('slip_peak_mm'),flush=True)
  output=BASE/'results/grasp'
  output.mkdir(parents=True,exist_ok=True)
  (output/('pilot.json' if pilot else 'summary.json')).write_text(json.dumps(rows,indent=2)+'\n')


def convergence(source):
  rows=[]
  for variant,closure,scale,impedance in [VARIANTS[0],VARIANTS[-1]]:
    path=model(source,BASE/'generated/grasp'/f'{variant}-m1.0',closure,scale,impedance,1.)
    for exact,peak,dt in itertools.product([0,1],[0,60,100],[.001,.0005]):
      label=f'{variant}-convergence-e{exact}-p{peak}-h{dt}'
      result=run(path,BASE/'raw/grasp'/label,exact=exact,peak=peak,axis=2,dt=dt)
      rows.append(dict(variant=variant,mass=1.,label=label,**result))
      print(label,result['outcome'],result.get('slip_peak_mm'),flush=True)
  (BASE/'results/grasp/convergence.json').write_text(json.dumps(rows,indent=2)+'\n')


if __name__=='__main__':
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('source',type=pathlib.Path)
  group=p.add_mutually_exclusive_group()
  group.add_argument('--pilot',action='store_true')
  group.add_argument('--convergence',action='store_true')
  a=p.parse_args()
  if a.convergence:
    convergence(a.source.resolve())
  else:
    sweep(a.source.resolve(),a.pilot)
