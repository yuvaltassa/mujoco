"""Plot the committed integrator comparison without re-running simulations."""
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import NullFormatter

from fourbar import BASE

ROOT=BASE/'results'
COLORS=dict(connect='#2764a5',weld='#99569a',tendon='#c17918')
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,
                     'axes.titleweight':'bold','axes.titlesize':12})


def main():
  data=json.loads((ROOT/'discrete.json').read_text())
  fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
  fig.suptitle('Linkage Lab | Does the discrete integrator help?',fontsize=18,fontweight='bold')
  def bars(closure,variant,mode,scale):
    # Precision controls repeat several settings; only the reference-comparison matrix
    # has accuracy metrics. Static runs are unique.
    return sorted([r for r in data['fourbar'] if r['closure']==closure and r['variant']==variant
                   and r['mode']==mode and r['scale']==scale and r['precision']=='double' and r['exact']==1
                   and not r['failed'] and r['dt']>=.000125
                   and (mode=='static' or 'output_load_error_Nm' in r)],key=lambda r:r['dt'])
  ax=axes[0,0]
  for variant,color,label in [('implicitfast','#bf4747','Implicitfast, refsafe on'),('unclamped','.5','Implicitfast, refsafe off'),('discrete','#358878','Discrete')]:
    r=bars('connect',variant,'static',.1)
    ax.loglog([x['dt']*1000 for x in r],[x['compliance_um_N'] for x in r],'o-',color=color,label=label)
  ax.axvline(.5,color='.5',ls=':',label='Old clamp begins (τ = 1 ms)')
  ax.set(title='A  Discrete preserves the stiff equality much better',xlabel='Timestep (ms)',ylabel='Static output compliance (µm/N)')
  ax.legend(fontsize=9);ax.grid(alpha=.18,which='both')
  ax=axes[0,1]
  for closure,color in COLORS.items():
    for variant,style in [('implicitfast','--'),('discrete','-')]:
      r=bars(closure,variant,'dynamic',1)
      ax.loglog([x['dt']*1000 for x in r],[x['deflection_rms_mm'] for x in r],style+'o',color=color,label=closure if variant=='discrete' else None)
  ax.axvline(5,color='.5',ls=':')
  ax.set(title='B  Driven-loop error improves before the clamp',xlabel='Timestep (ms)',ylabel='Output deflection RMS (mm)')
  ax.text(.03,.97,'Solid: discrete; dashed: implicitfast\n2 Hz drive; authored τ ≈ 10 ms; old clamp ≈ 5 ms',va='top',transform=ax.transAxes,fontsize=9)
  ax.legend(fontsize=9,loc='lower right');ax.grid(alpha=.18,which='both')
  ax=axes[1,0]
  for scale,color,label in [(1,COLORS['connect'],'τ = 10 ms'),(.1,COLORS['tendon'],'τ = 1 ms')]:
    for variant,style in [('implicitfast','--'),('discrete','-')]:
      r=bars('connect',variant,'dynamic',scale)
      ax.loglog([x['dt']*1000 for x in r],[x['output_load_error_Nm']*1000 for x in r],style+'o',color=color,label=label if variant=='discrete' else None)
  ax.set(title='C  Tighter closure does not remove phase error',xlabel='Timestep (ms)',ylabel='Output-load torque RMS error (mN m)')
  ax.text(.03,.97,'Connect; error against 0.03125 ms reference\nSolid: discrete; dashed: implicitfast',va='top',transform=ax.transAxes,fontsize=9)
  ax.legend(fontsize=9,loc='lower right');ax.grid(alpha=.18,which='both')
  ax=axes[1,1]
  for closure,color in [('connect',COLORS['connect']),('shadow',COLORS['weld'])]:
    for integrator,style in [('implicitfast','--'),('discrete','-')]:
      r=[x for x in data['grasp'] if x['closure']==closure and x['integrator']==integrator and x['exact']==1 and x['peak']==60]
      ax.plot([x['dt']*1000 for x in r],[x['closure_peak_mm'] for x in r],style+'o',color=color,label=closure if integrator=='discrete' else None)
  ax.set(title='D  The improvement carries over to Robotiq grasps',xlabel='Timestep (ms)',ylabel='Peak loop error during shake (mm)')
  ax.text(.03,.97,'1 kg box; 60 m/s² vertical shake at 5 Hz\nSolid: discrete; dashed: implicitfast',va='top',transform=ax.transAxes,fontsize=9)
  ax.legend(fontsize=9,loc='center right');ax.grid(alpha=.18)
  for ax in axes.flat:
    if ax.get_xscale()=='log':
      ax.set_xticks([.125,.5,2,8,16],['0.125','0.5','2','8','16'])
      ax.xaxis.set_minor_formatter(NullFormatter())
  fig.savefig(ROOT/'discrete-results.png',dpi=150)


if __name__=='__main__':main()
