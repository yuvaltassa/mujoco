"""Regenerate the four-bar scientific figures from committed compact results."""
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import NullFormatter
import numpy as np

from fourbar import BASE, A, B, G, geometry

RESULTS=BASE/'results'
COLORS=dict(connect='#2764a5',weld='#99569a',tendon='#c17918')
LABELS=dict(connect='Connect2',weld='Shadow weld3',tendon='Tendon')
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,
                     'axes.titleweight':'bold','axes.titlesize':12,'figure.facecolor':'white'})


def main():
  p=json.loads((RESULTS/'fourbar.json').read_text());rows=p['records']
  def select(group,**kw):
    return [r for r in rows if r['group']==group and all(r[k]==v for k,v in kw.items())]
  fig,axes=plt.subplots(3,2,figsize=(13,12),layout='constrained')
  fig.suptitle('Linkage Lab | Force transfer through a four-bar',fontsize=19,fontweight='bold')
  ax=axes[0,0]
  theta=math.pi/2;phi,*_=geometry(theta)
  P=np.array([0.,0]);Q=np.array([G,0]);C=A*np.array([np.cos(theta),np.sin(theta)]);D=Q+B*np.array([np.cos(phi),np.sin(phi)])
  ax.plot([*P[:1],Q[0]],[P[1],Q[1]],color='.6',lw=5)
  ax.plot([C[0],D[0]],[C[1],D[1]],color=COLORS['weld'],lw=16,alpha=.3)
  for start,end,color in [(P,C,'#d75b32'),(C,D,COLORS['connect']),(Q,D,'#35916b')]:
    ax.plot([start[0],end[0]],[start[1],end[1]],lw=7,color=color,solid_capstyle='round')
  ax.scatter(*np.array([P,Q,C,D]).T,s=65,c='white',edgecolors='.2',zorder=4)
  tip=D+np.array([math.sin(phi),-math.cos(phi)])*.06
  ax.annotate('',xy=tip,xytext=D,arrowprops=dict(arrowstyle='->',lw=2,color='black'))
  ax.text(tip[0]+.005,tip[1], 'Output load',va='center')
  ax.annotate('Input torque',xy=P,xytext=(-.08,-.045),arrowprops=dict(arrowstyle='->',color='.3'))
  ax.text(.012,.145,'Coupler: body / split body / distance equality',fontsize=10)
  ax.text(.15,-.052,'Ground: 0.30 m',ha='center',color='.35')
  ax.set(xlim=(-.1,.4),ylim=(-.08,.215),aspect='equal')
  ax.set_title('A  Drive one side; resist motion at the other',loc='left');ax.axis('off')
  ax=axes[0,1]
  angles=np.linspace(35,125,300)
  ax.plot(angles,[2*geometry(math.radians(t))[2] for t in angles],color='.25',label='Rigid virtual-work reference',lw=2)
  for closure,marker in [('connect','o'),('weld','x'),('tendon','+')]:
    rr=select('static',closure=closure,load=10)
    ax.plot([r['theta'] for r in rr],[r['input_torque_Nm'] for r in rr],marker,ms=7,color=COLORS[closure],label=LABELS[closure])
  ax.set(title='B  All three transfer the static load',xlabel='Input angle (degrees)',ylabel='Input torque for a 10 N output load (N m)')
  ax.legend(fontsize=9,loc='lower right');ax.grid(alpha=.18)
  ax=axes[1,0]
  for closure in COLORS:
    for kv,style in [(5,'-'),(0,'--')]:
      rr=sorted(select('drive_control',closure=closure,drive_kv=kv),key=lambda r:r['dt'])
      ax.loglog([r['dt']*1e3 for r in rr],[r['inverse_dynamics_error_Nm']*1e3 for r in rr],style+'o',color=COLORS[closure],ms=4,
                label=LABELS[closure] if kv==5 else None)
  ax.set(title='C  Actuator damping dominates coarse-step error',xlabel='Timestep (ms)',ylabel='Torque error against rigid dynamics (mN m)')
  ax.text(.04,.97,'Solid: drive damping 5 N m s/rad\nDashed: drive damping zero',transform=ax.transAxes,va='top',fontsize=9)
  ax.set_xticks([.03125,.125,.5],['0.03125','0.125','0.5']);ax.xaxis.set_minor_formatter(NullFormatter())
  ax.legend(fontsize=9,loc='lower right');ax.grid(alpha=.18,which='both')
  ax=axes[1,1]
  for mode,name,color in [(0,'Full weld','#bf4747'),(1,'Planar weld3',COLORS['weld']),(2,'Full weld + row floor','#358878')]:
    values=[]
    for angle in [0,37,90]:
      s=select('precision',closure='weld',rows_mode=mode,precision='single',rotation=angle,exact=1)[0]
      d=select('precision',closure='weld',rows_mode=mode,precision='double',rotation=angle,exact=1)[0]
      values.append(abs(s['input_torque_Nm']-d['input_torque_Nm'])*1e3)
    ax.plot([0,37,90],values,'o-',label=name,color=color)
  ax.set_yscale('symlog',linthresh=.01)
  ax.set(title='D  Removing redundant rows prevents float errors',xlabel='Whole-mechanism rotation (degrees)',ylabel='Single vs double holding torque (mN m)',xticks=[0,37,90])
  ax.legend(fontsize=9);ax.grid(alpha=.18,which='both')
  ax=axes[2,0]
  for group,label,color in [('mass_fixed','Fixed solver parameters','#bf4747'),('mass_tuned','Matched compliance + release area',COLORS['connect'])]:
    rr=select(group,mode='calibrate')
    ax.loglog([r['mass']*1000 for r in rr],[r['compliance_m_N']*1e6 for r in rr],'o-',label=label,color=color)
  ax.axhline(p['target_compliance_m_N']*1e6,color=COLORS['tendon'],ls='--',label='Calibrated tendon / target')
  ax.invert_xaxis();ax.set(title='E  A lighter rod gets softer unless retuned',xlabel='Coupler mass (g), decreasing →',ylabel='Output compliance (µm/N)')
  ax.legend(fontsize=9);ax.grid(alpha=.18,which='both')
  ax=axes[2,1]
  for group,label,color in [('mass_fixed','Fixed solver parameters','#bf4747'),('mass_tuned','Matched compliance + release area',COLORS['connect'])]:
    rr=select(group,mode='dynamic')
    ax.loglog([r['mass']*1000 for r in rr],[r['torque_difference_from_tendon_Nm']*1e3 for r in rr],'o-',label=label,color=color)
  ax.invert_xaxis();ax.set(title='F  Retuning reveals convergence toward the tendon',xlabel='Coupler mass (g), decreasing →',ylabel='RMS input-torque difference from tendon (mN m)')
  ax.text(.03,.96,'2 Hz drive; viscous output load; drive damping zero',transform=ax.transAxes,fontsize=9,va='top')
  ax.legend(fontsize=9,loc='lower left');ax.grid(alpha=.18,which='both')
  fig.savefig(RESULTS/'fourbar-results.png',dpi=155)
  plt.close(fig)
  fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
  for closure in COLORS:
    x=np.genfromtxt(RESULTS/f'fourbar_trace_matched_calibration_{closure}.csv',delimiter=',',names=True)
    mask=(x['time']>=1.24)&(x['time']<=1.4)
    axes[0].plot(x['time'][mask]-1.25,-x['deflection_m'][mask]*1e6,label=LABELS[closure],color=COLORS[closure])
  x=np.genfromtxt(RESULTS/'fourbar_trace_light_calibration_connect.csv',delimiter=',',names=True)
  mask=(x['time']>=1.24)&(x['time']<=1.4)
  axes[0].plot(x['time'][mask]-1.25,-x['deflection_m'][mask]*1e6,'--',label='Connect2, 0.2 g rod',color='.3')
  axes[0].set(title='Two calibration numbers do not match every mode',xlabel='Time after releasing 1 N output load (s)',ylabel='Output deflection (µm)')
  axes[0].legend(fontsize=9);axes[0].grid(alpha=.2)
  for closure in COLORS:
    rr=select('drive_control',closure=closure,drive_kv=0,dt=.0005)[0]
    axes[1].bar(list(COLORS).index(closure)-.17,rr['input_work_J'],width=.32,color=COLORS[closure],alpha=.45)
    axes[1].bar(list(COLORS).index(closure)+.17,-rr['output_work_J'],width=.32,color=COLORS[closure])
  axes[1].set(title='Useful work delivered to the output load',ylabel='Work over 3 s (J)',xticks=range(3),xticklabels=list(LABELS.values()))
  axes[1].text(.04,.96,'Pale: actuator work\nSolid: work absorbed by output load\nRemaining budget: kinetic energy, equality work,\nand time-integration error',transform=axes[1].transAxes,va='top',fontsize=9)
  axes[1].set_ylim(0,1.7);axes[1].grid(axis='y',alpha=.2)
  fig.savefig(RESULTS/'fourbar-response.png',dpi=155)
  plt.close(fig)


if __name__=='__main__':main()
