"""Plot the diagonal investigation and actual grasp measurements."""
import json
import pathlib

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE=pathlib.Path(__file__).resolve().parent
OUT=BASE/'results'


def main():
  scaling=json.loads((OUT/'investigation/scaling.json').read_text())
  grasps=json.loads((OUT/'grasp/summary.json').read_text())
  packed=OUT/'phase2-traces.npz'
  traces=dict(np.load(packed)) if packed.exists() else {}
  for label in ['double-connect-raw','single-connect-raw','single-connect-floor']:
    path=BASE/'raw/investigation'/(label+'.csv')
    if path.exists():
      data=np.genfromtxt(path,delimiter=',',names=True)
      traces[label]=np.column_stack([data['time'],data['closure_m']])
  np.savez_compressed(packed,**traces)
  plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
  fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained')
  fig.suptitle('Linkage lab · exact diagonal, torque scale, and grasping',fontsize=19,fontweight='bold')
  ax=axes[0,0]
  for exact,color in [(0,'#cf6733'),(1,'#246786')]:
    rows=sorted([r for r in scaling if r['exact']==exact and r['impedance']=='.99'],key=lambda r:r['scale'])
    ax.plot([r['scale'] for r in rows],[r['settled_mm'] for r in rows],'o-',color=color,
            label='Exact diagonal' if exact else 'Approximate diagonal')
  ax.set_xscale('log');ax.set_yscale('log');ax.grid(alpha=.15)
  ax.set_xlabel('Weld torque scale (m)');ax.set_ylabel('Settled loop gap (mm)')
  ax.set_title('A · Exact scaling works at constant impedance',loc='left',fontsize=12)
  ax.text(.04,.08,'Impedance 0.99 · double precision · empty gripper',transform=ax.transAxes,fontsize=9)
  ax.legend(fontsize=9)
  ax=axes[0,1]
  for label,title,color,width in [('single-connect-raw','Single / exact','#c14b3d',.8),
      ('double-connect-raw','Double / exact','#246786',3),
      ('single-connect-floor','Single / exact + row floor','#48a878',1)]:
    trace=traces[label]
    ax.plot(trace[:,0],np.maximum(trace[:,1]*1000,1e-7),label=title,color=color,lw=width)
  ax.set_xlim(0,3);ax.set_ylim(.001,10);ax.set_yscale('log');ax.grid(axis='y',alpha=.15)
  ax.set_xlabel('Time (s)');ax.set_ylabel('Loop gap (mm)')
  ax.set_title('B · Near-null row regularization resolves this failure',loc='left',fontsize=12)
  ax.legend(fontsize=9,loc='lower right')
  ax=axes[1,0]
  variants=['connect','shadow-default','shadow-stiff']
  for exact,color in [(0,'#cf6733'),(1,'#246786')]:
    values=[]
    for variant in variants:
      row=next(r for r in grasps if r['variant']==variant and r['precision']=='double' and
               r['mode']=='raw' and r['exact']==exact and r['mass']==1 and r['peak']==60 and r['axis']==2)
      assert row['outcome']=='clean hold'
      values.append(row['slip_peak_mm'])
    bars=ax.bar(np.arange(3)+(exact-.5)*.34,values,.32,color=color,label='Exact' if exact else 'Approximate')
    ax.bar_label(bars,fmt='%.3f',fontsize=9,padding=3)
  ax.set_xticks(range(3),['Connect','Shadow\nsource impedance','Shadow\nimpedance 0.999'])
  ax.set_ylim(0,1.45);ax.set_ylabel('Peak object motion after t = 2 s (mm)')
  ax.set_title('C · Less loop error does not guarantee less object slip',loc='left',fontsize=12)
  ax.text(.03,.95,'1 kg · vertical 60 m/s² · 5 Hz · double · scale 1 m',transform=ax.transAxes,fontsize=9,va='top')
  ax.legend(fontsize=9,loc='upper left',bbox_to_anchor=(0,.88))
  ax=axes[1,1]
  settings=[('double',1,'raw'),('single',0,'raw'),('single',1,'raw'),('single',1,'floor')]
  counts=[]
  for precision,exact,mode in settings:
    rr=[r for r in grasps if r['variant'] in ['connect','shadow-stiff'] and r['precision']==precision and
        r['exact']==exact and r['mode']==mode and r['peak'] in [0,60] and r['axis']==2]
    assert len(rr)==8
    counts.append(sum(r['outcome']=='clean hold' for r in rr))
  bars=ax.bar(range(4),counts,color=['#246786','#7094a5','#c14b3d','#48a878'])
  ax.bar_label(bars,fmt='%d / 8',padding=4)
  ax.set_xticks(range(4),['Double\nexact','Single\napproximate','Single\nexact','Single exact\n+ row floor'])
  ax.set_ylim(0,9.5);ax.set_yticks([0,2,4,6,8]);ax.set_ylabel('Clean holds')
  ax.set_title('D · The precision control also works during grasping',loc='left',fontsize=12)
  ax.text(.02,.96,'Two closures × two masses × two accelerations',transform=ax.transAxes,fontsize=9,va='top')
  fig.supxlabel('The row floor is an experimental diagnostic. Engine source is unchanged.\n'
                'Grasp tests retain stock armature and contacts; motion includes post-release settling.',fontsize=10)
  fig.savefig(OUT/'phase2-results.png',dpi=170)
  print(OUT/'phase2-results.png')


if __name__=='__main__':
  main()
