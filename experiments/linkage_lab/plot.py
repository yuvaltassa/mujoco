"""Create figures and a compact, portable result table from saved sweeps."""
import csv
import json
import pathlib
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE = pathlib.Path(__file__).resolve().parent
RAW, OUT = BASE/'raw', BASE/'results'
GROUPS = ['baseline', 'scale1', 'noarmature', 'single', 'stiff', 'convergence',
          'aligned-single', 'aligned-double', 'stiff-single']


def main():
  OUT.mkdir(exist_ok=True)
  if all((RAW/g/'summary.json').exists() for g in GROUPS):
    groups = {g: json.loads((RAW/g/'summary.json').read_text()) for g in GROUPS}
  else:
    saved = json.loads((OUT/'summary.json').read_text())
    groups = {g: [{k: v for k, v in r.items() if k != 'group'}
                  for r in saved if r['group'] == g] for g in GROUPS}
  trace_file = OUT/'plot-traces.npz'
  traces = dict(np.load(trace_file)) if trace_file.exists() else {}
  rows = [dict(group=g, **r) for g, data in groups.items() for r in data]
  columns = list(dict.fromkeys(k for r in rows for k in r))
  with (OUT/'summary.csv').open('w') as f:
    writer = csv.DictWriter(f, fieldnames=columns, lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)
  (OUT/'summary.json').write_text('[\n' + ',\n'.join(json.dumps(r) for r in rows) + '\n]\n')
  for name in ['audit-double', 'audit-single']:
    if (RAW/(name+'.json')).exists():
      shutil.copy(RAW/(name+'.json'), OUT/(name+'.json'))

  def pick(group, **query):
    candidates = [r for r in groups[group] if all(r[k] == v for k, v in query.items())]
    assert len(candidates) == 1, (group, query, len(candidates))
    return candidates[0]

  plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                       'axes.spines.right': False, 'figure.facecolor': 'white'})
  fig, axes = plt.subplots(2, 2, figsize=(13, 9.5), layout='constrained')
  fig.suptitle('Linkage lab · Robotiq without an object', fontsize=20, fontweight='bold')
  ax = axes[0, 0]
  labels = ['Connect', 'Shadow weld', 'Stiffer shadow weld']
  for exact in [0, 1]:
    values = [pick(g, model=m, exact=exact, axis=0, peak=50, control=128,
                   dt=.002, integrator='implicitfast')['closure_max_mm']
              for g, m in [('scale1','connect'), ('scale1','shadow'), ('stiff','shadow')]]
    bars=ax.bar(np.arange(3)+(exact-.5)*.34, values, width=.32,
                color=['#1b668c', '#84b6ce'][exact], label=['Approx. diagonal','Exact diagonal'][exact])
    ax.bar_label(bars, fmt='%.3f', fontsize=9, padding=3)
  ax.set_xticks(range(3), labels, fontsize=10)
  ax.set_ylabel('Peak joint-cut separation (mm)')
  ax.set_ylim(0,.18)
  ax.set_title('A · A weld needs its own stiffness settings', loc='left', fontsize=12)
  ax.legend(fontsize=9)
  ax.text(.02,.97,'Double · 2 ms · 5 Hz · 50 m/s² · command 128', transform=ax.transAxes,
          va='top', fontsize=9)

  ax=axes[0,1]
  for i,(g,m,e,label) in enumerate([
      ('convergence','connect',0,'Connect / approximate'),
      ('convergence','shadow',0,'Shadow / approximate'),
      ('convergence','shadow',1,'Shadow / exact'),
      ('stiff','shadow',0,'Stiffer shadow / approximate')]):
    rr=[pick(g,model=m,exact=e,axis=0,peak=50,control=128,dt=h,integrator='implicitfast')
        for h in [.002,.001,.0005]]
    ax.plot([2,1,.5],[r['closure_max_mm'] for r in rr],'o-',label=label)
  ax.set_yscale('log');ax.set_xticks([.5,1,2]);ax.invert_xaxis()
  ax.set_xlabel('Timestep (ms)');ax.set_ylabel('Peak separation (mm, log scale)')
  ax.set_title('B · Residual compliance persists as timestep falls',loc='left',fontsize=12)
  ax.legend(fontsize=9);ax.grid(axis='y',alpha=.2)

  ax=axes[1,0]
  specs=[('convergence','connect',0,'Double / approximate','#1b668c'),
         ('convergence','connect',1,'Double / exact','#31865a'),
         ('single','connect',1,'Single / exact / original frame','#bd483b'),
         ('aligned-single','connect',1,'Single / exact / rotated frame','#6f5aa3')]
  for group,model,exact,label,color in specs:
    row=pick(group,model=model,exact=exact,axis=1 if group.startswith('aligned') else 0,
             peak=0,control=128,dt=.002,integrator='implicitfast')
    key = f'{group}_{model}_{exact}'
    if (RAW/group/row['trace']).exists():
      raw = np.genfromtxt(RAW/group/row['trace'], delimiter=',', names=True)
      traces[key] = np.empty(len(raw), dtype=[('time', 'f8'), ('closure_m', 'f8')])
      for field in ['time', 'closure_m']:
        traces[key][field] = raw[field]
    trace = traces[key]
    ax.plot(trace['time'][::3],np.maximum(trace['closure_m'][::3]*1e3,1e-7),label=label,color=color,lw=1)
  ax.set_ylim(.001,30);ax.set_yscale('log');ax.set_xlim(0,9)
  ax.set_xlabel('Time (s)');ax.set_ylabel('Separation (mm, log scale)')
  ax.set_title('C · A precision problem appears even without shaking',loc='left',fontsize=12)
  ax.legend(fontsize=8);ax.grid(axis='y',alpha=.2)

  ax=axes[1,1]
  for exact in [0,1]:
    vv=[pick(g,model='shadow',exact=exact,axis=0,peak=0,control=128,dt=.002,
             integrator='implicitfast')['settled_closure_mm']
        for g in ['baseline','scale1','stiff']]
    bars=ax.bar(np.arange(3)+(exact-.5)*.34,vv,width=.32,
                color=['#1b668c','#84b6ce'][exact],label=['Approx. diagonal','Exact diagonal'][exact])
    ax.bar_label(bars,fmt='%.3g',fontsize=9,padding=3)
  ax.set_yscale('log');ax.set_ylim(.001,40)
  ax.set_xticks(range(3),['Scale 0.03 m','Scale 1 m','Scale 1 m\nimpedance 0.999'],fontsize=10)
  ax.set_ylabel('Settled separation (mm, log scale)')
  ax.set_title('D · Weld scaling changes the effective compliance',loc='left',fontsize=12)
  ax.legend(fontsize=9);ax.grid(axis='y',alpha=.2)
  fig.supxlabel('Contacts off; original Menagerie armature retained except in the separate ablation.\n'
                'The stiffer weld is a sensitivity test, not a fit to hardware.',fontsize=10)
  np.savez_compressed(trace_file, **traces)
  fig.savefig(OUT/'first-results.png',dpi=170)
  fig.savefig(OUT/'first-results.svg')
  print('Saved figures and',len(rows),'run summaries;',sum(r['failed'] for r in rows),'failed runs.')


if __name__=='__main__':
  main()
