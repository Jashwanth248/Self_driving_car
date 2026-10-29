#!/usr/bin/env python3
"""Reproducibly render plots and a 2-D driving animation from C++ telemetry."""
import argparse
import csv
import math
import subprocess
import shutil
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon, Circle
from PIL import Image

MAP = ['...........', '..###......', '...........', '......###..', '...........']
BG = '#0b1423'
FG = '#e6f0ff'
ACCENT = '#38cbb4'
ORANGE = '#e4aa68'


def load(csv_path):
    with open(csv_path, newline='', encoding='utf-8') as f:
        return [{k: (v if k == 'mode' else float(v)) for k, v in row.items()}
                for row in csv.DictReader(f)]


def plot_metric(rows, out, name, series, ylabel):
    fig, ax = plt.subplots(figsize=(11, 4.2), dpi=160)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor('#121f32')
    times = [r['time_s'] for r in rows]
    for key, label, color in series:
        ax.plot(times, [r[key] for r in rows], label=label, color=color, linewidth=2.3)
    ax.set(xlabel='Simulated time (seconds)', ylabel=ylabel, title=name)
    ax.grid(True, alpha=.15)
    ax.tick_params(colors=FG)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.title.set_color(FG)
    for spine in ax.spines.values():
        spine.set_color('#4b647b')
    leg = ax.legend(frameon=False, loc='best')
    for text in leg.get_texts():
        text.set_color(FG)
    fig.tight_layout(pad=1.7)
    fig.savefig(out, facecolor=BG)
    plt.close(fig)


def create_path_figure(rows, out):
    fig, ax = plt.subplots(figsize=(11, 5), dpi=160)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor('#121f32')
    for y, line in enumerate(MAP):
        for x, cell in enumerate(line):
            if cell == '#':
                ax.add_patch(Rectangle((x*4-1.75,y*4-1.75),3.5,3.5,
                                       facecolor='#495a6c',alpha=.8))
    ax.plot([r['x_m'] for r in rows],[r['y_m'] for r in rows],color=ACCENT,lw=3,label='Driven trajectory')
    ax.plot([0,40,40],[0,0,16],color='#9daabd',ls='--',lw=1.5,label='Raw A* corridor (schematic)')
    ax.scatter([rows[0]['x_m']],[rows[0]['y_m']],color='#88c1ff',s=65,zorder=4,label='Start')
    ax.scatter([rows[-1]['x_m']],[rows[-1]['y_m']],color=ORANGE,s=65,zorder=4,label='Natural stop')
    ax.scatter([40],[20],color='#ff6b72',s=90,marker='X',zorder=5,label='Virtual obstacle')
    ax.set_aspect('equal', adjustable='box')
    ax.set(xlim=(-3,48),ylim=(-5,26),xlabel='X position (m)',ylabel='Y position (m)',title='2-D vehicle trajectory and virtual obstacle')
    ax.grid(True,alpha=.12)
    ax.tick_params(colors=FG)
    for obj in [ax.xaxis.label,ax.yaxis.label,ax.title]:obj.set_color(FG)
    for spine in ax.spines.values():spine.set_color('#4b647b')
    leg=ax.legend(frameon=False,loc='upper left')
    for text in leg.get_texts():text.set_color(FG)
    fig.tight_layout(pad=1.3)
    fig.savefig(out,facecolor=BG)
    plt.close(fig)


def draw_frame(rows, idx, filename):
    row=rows[idx]
    fig,ax=plt.subplots(figsize=(10,5),dpi=100)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor('#122338')
    for y,line in enumerate(MAP):
        for x,cell in enumerate(line):
            if cell=='#':
                ax.add_patch(Rectangle((x*4-1.6,y*4-1.6),3.2,3.2,
                                       facecolor='#475b6d',alpha=.7))
    ax.plot([r['x_m'] for r in rows[:idx+1]],[r['y_m'] for r in rows[:idx+1]],
            color=ACCENT,lw=2.3,alpha=.85)
    ax.scatter([40],[20],marker='X',s=250,c='#ff6e76',linewidths=2)
    ax.text(42,21.8,'Virtual obstacle',color='#ff9da1',fontsize=10)
    x,y,theta=row['x_m'],row['y_m'],row['heading_rad']
    corners=[(-2.0,-.95),(2.0,-.95),(2.0,.95),(-2.0,.95)]
    rotated=[(x+dx*math.cos(theta)-dy*math.sin(theta),y+dx*math.sin(theta)+dy*math.cos(theta))for dx,dy in corners]
    ax.add_patch(Polygon(rotated,closed=True,facecolor='#48b9fb',edgecolor='#e7f5ff',linewidth=1.5,zorder=4))
    ax.add_patch(Circle((x+1.2*math.cos(theta),y+1.2*math.sin(theta)),.23,facecolor='#ffffff',zorder=5))
    ax.set_xlim(-5,50);ax.set_ylim(-7,27);ax.set_aspect('equal',adjustable='box')
    ax.grid(alpha=.12)
    ax.set_title(f'AutoDrive Lab  |  {row["mode"]}  |  t = {row["time_s"]:.1f} s',color=FG,fontsize=14,pad=12)
    ax.text(-3.7,23.5,f'Speed     {row["speed_mps"]:4.2f} m/s\nBrake     {row["brake"]*100:4.0f}%\nRisk      {row["risk"]:4.2f}\nClearance {row["obstacle_m"]:4.1f} m',
            va='top',fontfamily='monospace',fontsize=11,color=FG,
            bbox=dict(boxstyle='round,pad=.5',facecolor='#0b1423',edgecolor='#304b64',alpha=.95))
    ax.tick_params(colors='#a7bacf')
    for spine in ax.spines.values():spine.set_color('#3c536a')
    fig.tight_layout(pad=1.2)
    fig.savefig(filename,facecolor=BG)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--csv',required=True)
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    out=Path(args.out)
    out.mkdir(parents=True,exist_ok=True)
    rows=load(args.csv)
    if not rows:raise SystemExit('telemetry CSV is empty')
    plot_metric(rows,out/'speed_plot.png','Speed control', [('speed_mps','Actual speed',ACCENT),('target_mps','Target speed',ORANGE)],'m/s')
    plot_metric(rows,out/'position_plot.png','2-D position over time', [('x_m','X position','#67b7ff'),('y_m','Y position',ACCENT)],'meters')
    plot_metric(rows,out/'control_plot.png','Throttle and brake commands', [('throttle','Throttle',ACCENT),('brake','Brake',ORANGE)],'Command [0, 1]')
    plot_metric(rows,out/'risk_plot.png','Synthetic ML risk score', [('risk','ML risk','#f0ae83')],'Score [0, 1]')
    create_path_figure(rows,out/'route_map.png')
    # One frame for about every 0.2 simulated seconds, including the final state.
    chosen=list(range(0,len(rows),2))
    if chosen[-1] != len(rows)-1:chosen.append(len(rows)-1)
    frames=out/'_frames'
    frames.mkdir(exist_ok=True)
    for k,i in enumerate(chosen):draw_frame(rows,i,frames/f'frame_{k:04d}.png')
    images=[Image.open(frames/f'frame_{k:04d}.png').convert('RGB') for k in range(len(chosen))]
    images[0].save(out/'autodrive_demo.gif',save_all=True,append_images=images[1:],duration=110,loop=0,optimize=True)
    if shutil.which('ffmpeg'):
        cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate','9','-i',str(frames/'frame_%04d.png'),
             '-c:v','libx264','-pix_fmt','yuv420p','-crf','27','-movflags','+faststart',str(out/'autodrive_demo.mp4')]
        subprocess.run(cmd,check=True)
    else:
        print('ffmpeg unavailable: GIF created, MP4 skipped')
    for image in images:image.close()
    for image in frames.iterdir():image.unlink()
    frames.rmdir()
    print(f'Rendered {len(chosen)} animation frames; charts and media in {out}')

if __name__=='__main__':main()
