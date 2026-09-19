"""Planar four-bar geometry, uniform-link inertia, and rigid transmission reference."""
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

BASE = pathlib.Path(__file__).resolve().parent
ROOT = BASE.parents[1]
A, B, L, G = .1, .2, .25, .3
MA, MB, RADIUS = .3, .4, .006


def geometry(theta, length=L):
  crank = A*np.array([math.cos(theta), math.sin(theta)])
  delta = np.array([G, 0])-crank
  distance = np.linalg.norm(delta)
  along = (length*length-B*B+distance*distance)/(2*distance)
  height = math.sqrt(max(0, length*length-along*along))
  unit = delta/distance
  tip = crank+along*unit+height*np.array([-unit[1],unit[0]])
  phi = math.atan2(tip[1], tip[0]-G)
  psi = math.atan2(tip[1]-crank[1], tip[0]-crank[0])
  rod = tip-crank
  crank_derivative = A*np.array([-math.sin(theta),math.cos(theta)])
  rocker_tangent = B*np.array([-math.sin(phi),math.cos(phi)])
  ratio = np.dot(rod,crank_derivative)/np.dot(rod,rocker_tangent)
  tip_derivative = rocker_tangent*ratio
  psi_derivative = (rod[0]*(tip_derivative-crank_derivative)[1]-rod[1]*(tip_derivative-crank_derivative)[0])/(length*length)
  return phi, psi, ratio, psi_derivative, crank_derivative, tip_derivative


def effective_inertia(theta, mass, length=L):
  phi,psi,ratio,psi_derivative,da,db = geometry(theta,length)
  inertia = MA*(A*A/3+RADIUS*RADIUS/4)+MB*(B*B/3+RADIUS*RADIUS/4)*ratio*ratio
  inertia += mass*np.dot((da+db)/2,(da+db)/2)
  inertia += mass*(length*length+3*RADIUS*RADIUS)/12*psi_derivative**2
  return inertia


def fmt(values):
  return ' '.join(f'{x:.17g}' for x in values)


def quat(angle):
  return fmt([math.cos(angle/2),0,0,math.sin(angle/2)])


def rod(parent,name,length,mass,pos,angle,joint,anchor='0 0 0',color='.3 .55 .8 1'):
  body=ET.SubElement(parent,'body',name=name,pos=pos,quat=quat(angle))
  ET.SubElement(body,'joint',name=joint,axis='0 0 1',pos=anchor)
  ET.SubElement(body,'inertial',pos=fmt([length/2,0,0]),mass=str(mass),
                diaginertia=fmt([mass*RADIUS**2/2,*([mass*(length**2+3*RADIUS**2)/12]*2)]))
  ET.SubElement(body,'geom',type='capsule',fromto=fmt([0,0,0,length,0,0]),size=str(RADIUS),rgba=color)
  return body


def model(path,closure,theta=90,mass=.2,impedance=.99,timeconst=.01,rotation=90,length=L,damping=1):
  theta=math.radians(theta);phi,psi,*_=geometry(theta,length)
  root=ET.Element('mujoco',model='force_transfer_fourbar')
  ET.SubElement(root,'compiler',angle='radian')
  opt=ET.SubElement(root,'option',gravity='0 0 0',timestep='.0005',integrator='implicitfast',
                    jacobian='dense',solver='Newton',iterations='100',tolerance='1e-12')
  ET.SubElement(opt,'flag',contact='disable',island='disable',energy='enable',diagexact='enable')
  default=ET.SubElement(root,'default')
  ET.SubElement(default,'joint',limited='false',damping='0',armature='0')
  ET.SubElement(default,'geom',contype='0',conaffinity='0')
  ET.SubElement(default,'site',size='.008')
  world=ET.SubElement(root,'worldbody')
  tilt=math.radians(rotation)
  frame=ET.SubElement(world,'body',name='frame',pos='0 0 .15',quat=fmt([math.cos(tilt/2),math.sin(tilt/2),0,0]))
  ET.SubElement(frame,'geom',type='capsule',fromto=fmt([0,0,0,G,0,0]),size='.009',rgba='.35 .35 .35 1')
  crank=rod(frame,'crank',A,MA,'0 0 0',theta,'input',color='.85 .35 .2 1')
  ET.SubElement(crank,'site',name='input_tip',pos=fmt([A,0,0]))
  rocker=rod(frame,'rocker',B,MB,fmt([G,0,0]),phi,'output',color='.2 .7 .4 1')
  ET.SubElement(rocker,'site',name='output_tip',pos=fmt([B,0,0]))
  eq=ET.SubElement(root,'equality')
  parameters=dict(name='loop',solref=f'{timeconst} {damping}',solimp=f'{impedance} {impedance} .001')
  if closure=='tendon':
    tendons=ET.SubElement(root,'tendon')
    tendon=ET.SubElement(tendons,'spatial',name='rod',width='.004',rgba='.8 .7 .15 1')
    ET.SubElement(tendon,'site',site='input_tip');ET.SubElement(tendon,'site',site='output_tip')
    ET.SubElement(eq,'tendon',tendon1='rod',**parameters)
  else:
    split=closure=='weld'
    coupler=rod(crank,'coupler',length,mass/(2 if split else 1),fmt([A,0,0]),psi-theta,'coupler')
    ET.SubElement(coupler,'site',name='coupler_tip',pos=fmt([length,0,0]))
    if split:
      offset=[B-length*math.cos(psi-phi),-length*math.sin(psi-phi),0]
      shadow=rod(rocker,'shadow',length,mass/2,fmt(offset),psi-phi,'shadow',anchor=fmt([length,0,0]),color='.2 .8 1 .3')
      ET.SubElement(coupler,'site',name='weld_c',pos=fmt([length/2,0,0]))
      ET.SubElement(shadow,'site',name='weld_s',pos=fmt([length/2,0,0]))
      ET.SubElement(eq,'weld',site1='weld_c',site2='weld_s',torquescale='.1',**parameters)
    else:
      ET.SubElement(eq,'connect',site1='coupler_tip',site2='output_tip',**parameters)
  actuator=ET.SubElement(root,'actuator')
  ET.SubElement(actuator,'position',name='drive',joint='input',kp='500',kv='5')
  custom=ET.SubElement(root,'custom')
  ET.SubElement(custom,'numeric',name='reference',data=fmt([theta,phi,psi,mass if closure!='tendon' else 0,length]))
  visual=ET.SubElement(root,'visual');ET.SubElement(visual,'headlight',ambient='.5 .5 .5')
  ET.SubElement(world,'light',pos='.2 -1 1',dir='0 1 -1')
  ET.indent(root,space='  ')
  path.parent.mkdir(parents=True,exist_ok=True)
  path.write_text(ET.tostring(root,encoding='unicode')+'\n')
  return path
