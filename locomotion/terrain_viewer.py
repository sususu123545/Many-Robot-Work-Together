"""Display actual snapshots from all physics workers; no duplicated animations."""
import argparse
import json
import os
from pathlib import Path
import tkinter as tk
import time
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from PIL import Image,ImageTk
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--run',default='runs/ppo_terrain_v1');args=p.parse_args()
root=(Path(__file__).resolve().parent/args.run).resolve()
app=tk.Tk();app.title('ZSL1 多地形并行训练');app.geometry('1280x880')
header=tk.Label(app,text='正在等待训练状态…',font=('Noto Sans CJK SC',14));header.pack()
page=0;available=16
pager=tk.Frame(app);pager.pack()
def change_page(direction):
    global page
    page=(page+direction)%max(1,(available+15)//16)
tk.Button(pager,text='上一组机器狗',command=lambda:change_page(-1)).pack(side='left')
tk.Button(pager,text='下一组机器狗',command=lambda:change_page(1)).pack(side='left')
grid=tk.Frame(app);grid.pack(fill='both',expand=True);labels=[]
for i in range(16):
    f=tk.Frame(grid,borderwidth=1,relief='solid');f.grid(row=i//4,column=i%4,sticky='nsew')
    image=tk.Label(f);image.pack();text=tk.Label(f,font=('Noto Sans CJK SC',10));text.pack();labels.append((image,text))
for i in range(4):grid.rowconfigure(i,weight=1);grid.columnconfigure(i,weight=1)
cache={};photos=[];last=None;names={'flat':'平地','stairs':'台阶','slope':'坡道','rough':'碎石'}
def update():
    global photos,last,available
    try:
        progress=json.loads((root/'progress.json').read_text());payload=json.loads((root/'live-states.json').read_text())
        age=time.time()-payload['wall'];state=progress.get('status','training')
        available=len(payload['states'])
        heading={'training':'正在训练','final_evaluation':'正在做最终测试','completed':'训练完成 · 最终采样状态','interrupted':'训练已停止'}.get(state,state)
        header.config(text=f"{heading}  |  本段新增 {progress.get('additional_steps',0):,} 步  |  {available} 个环境 · 当前 {page*16+1}—{min(available,(page+1)*16)}  |  更新距今 {age:.0f} 秒")
        if (payload['wall'],page)!=last:
            photos=[]
            for index,(label,text) in enumerate(labels):
                if page*16+index>=available:label.config(image='');text.config(text='');continue
                s=payload['states'][page*16+index]
                path=s['scene']
                if path not in cache:
                    m=mujoco.MjModel.from_xml_path(path);m.vis.headlight.active=1;m.vis.headlight.ambient[:]=.5;m.vis.headlight.diffuse[:]=.8
                    cache[path]=(m,mujoco.MjData(m),mujoco.Renderer(m,180,315))
                m,d,renderer=cache[path];d.qpos[:]=s['qpos'];d.time=s['time'];mujoco.mj_forward(m,d)
                cam=mujoco.MjvCamera();cam.lookat[:]=[d.qpos[0]+.45,d.qpos[1],d.qpos[2]-.1];cam.distance=2.8;cam.azimuth=110;cam.elevation=-24
                option=mujoco.MjvOption();option.geomgroup[5]=1
                renderer.update_scene(d,camera=cam,scene_option=option)
                photo=ImageTk.PhotoImage(Image.fromarray(renderer.render()));photos.append(photo);label.config(image=photo)
                text.config(text=f"{s['rank']+1:02d} · {names[s['terrain']]} · 难度 {s['level']+1} · 成功 {s['successes']}/{s['episodes']}")
            last=(payload['wall'],page)
    except FileNotFoundError:pass
    except Exception as error:header.config(text=f'状态读取异常：{error}')
    app.after(1500,update)
def close():
    for m,d,r in cache.values():r.close()
    app.destroy()
app.protocol('WM_DELETE_WINDOW',close);update();app.mainloop()
