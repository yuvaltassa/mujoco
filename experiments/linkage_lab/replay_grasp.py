"""Render native-runner states; the Python MuJoCo package supplies rendering only."""
import argparse
import json
import pathlib
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BASE=pathlib.Path(__file__).resolve().parent
CASES=[('connect-m1.0-c255-a2-p60-e0','Connect / approximate'),
       ('shadow-small-m1.0-c255-a2-p60-e0','Shadow / approximate'),
       ('shadow-small-m1.0-c255-a2-p60-e1','Shadow / exact')]


def main(ffmpeg):
  output=BASE/'results/grasp'
  models=[]
  for label,title in CASES:
    folder=BASE/'raw/grasp'/label
    meta=json.loads((folder/'metadata.json').read_text())
    model=mujoco.MjModel.from_xml_path(meta['model'])
    data=mujoco.MjData(model)
    trace=np.genfromtxt(folder/'trace.csv',delimiter=',',names=True)
    # Reconstruct prescribed base translation, starting at rest, for the movie.
    t=trace['time'];a=trace['acceleration']
    velocity=np.r_[0,np.cumsum((a[1:]+a[:-1])*.5*np.diff(t))]
    displacement=np.r_[0,np.cumsum((velocity[1:]+velocity[:-1])*.5*np.diff(t))]
    renderer=mujoco.Renderer(model,400,400)
    camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.24];camera.distance=.46
    camera.azimuth=95;camera.elevation=-8
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_mount')
    obj=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'object_joint')
    models.append((title,model,data,trace,displacement,renderer,camera,body,obj,meta))
  try:
    font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',21)
    small=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf',17)
  except OSError:
    font=small=ImageFont.load_default()
  command=[ffmpeg,'-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','1200x540',
           '-r','25','-i','-','-an','-vcodec','libx264','-crf','22','-pix_fmt','yuv420p',
           '-movflags','+faststart',str(output/'grasp-comparison.mp4')]
  process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.DEVNULL)
  for frame in range(176):
    time=frame/25
    canvas=Image.new('RGB',(1200,540),'#f5f6f8');draw=ImageDraw.Draw(canvas)
    draw.text((20,10),'Linkage lab: release, hold, shake',font=font,fill='#16232d')
    draw.text((20,40),'1 kg box | vertical 60 m/s² at 5 Hz | double precision | shadow torque scale 0.03 m',font=small,fill='#334450')
    for col,(title,m,d,trace,displacement,renderer,camera,body,obj,meta) in enumerate(models):
      index=min(np.searchsorted(trace['time'],time),len(trace)-1);row=trace[index]
      d.qpos[:]=[row['q'+str(j)] for j in range(m.nq)]
      dz=displacement[index]
      m.body_pos[body,2]=.35+dz
      d.qpos[m.jnt_qposadr[obj]+2]+=dz
      d.eq_active[-1]=row['fixture']
      mujoco.mj_forward(m,d)
      renderer.update_scene(d,camera=camera)
      canvas.paste(Image.fromarray(renderer.render()),(400*col,92))
      draw.text((400*col+12,68),title,font=font,fill='#16232d')
      stopped=time>trace['time'][-1]+.01
      status='DROP (stopped)' if stopped else 'Fixture holds' if time<1.2 else 'Released' if time<2 else 'Shaking' if time<6 else 'Recovering'
      draw.text((400*col+12,495),status,font=small,fill='#b9342d' if stopped else '#14674c')
      draw.text((400*col+12,517),f"Loop gap {row['closure_m']*1000:.3f} mm",font=small,fill='#334450')
    draw.text((1090,12),f'{time:.2f} s',font=font,fill='#16232d')
    if frame==80:
      canvas.save(output/'grasp-comparison.png')
    process.stdin.write(canvas.tobytes())
  process.stdin.close()
  if process.wait()!=0:
    raise RuntimeError('Video encoding failed')
  for values in models:
    values[5].close()
  print(output/'grasp-comparison.mp4')


if __name__=='__main__':
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('ffmpeg',help='ffmpeg executable')
  main(p.parse_args().ffmpeg)
