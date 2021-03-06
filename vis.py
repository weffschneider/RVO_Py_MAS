#!/usr/bin/env python
import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors

from math import pi as PI
from math import atan2, sin, cos, sqrt



def visualize_traj_dynamic(ws_model, X, U, theta, goal, time = None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1,1,1)
    cmap = get_cmap(len(X))
    # ---plot target---
    target = ws_model['target']
    target_vel = ws_model['target_vel']
    srec = matplotlib.patches.Circle(
        (target[0], target[1]),
        ws_model['target_radius'],
        facecolor= 'red',
        fill = True,
        alpha=1)
    ax.arrow(target[0], target[1], target_vel[0], target_vel[1], head_width=0.05, head_length=0.1)
    ax.add_patch(srec)

    # ---plot traj---
    for i in range(0,len(X)):
        #-------plot car
        robot = matplotlib.patches.Circle(
            (X[i][0],X[i][1]),
            radius = ws_model['robot_radius'],
            facecolor='blue', #cmap(i),
            edgecolor='black',
            linewidth=1.0,
            ls='solid',
            alpha=1,
            zorder=2)
        gripper = matplotlib.patches.Rectangle(( X[i][0] + ws_model['robot_radius']*cos(theta[i]+PI/3.5), \
                                                 X[i][1] + ws_model['robot_radius']*sin(theta[i]+PI/3.5)), # gripper corner location
                                               ws_model['robot_radius']*1.5, # gripper width
                                               ws_model['robot_radius']*0.3, # gripper height
                                               theta[i]*180.0/PI - 90.0, # gripper orientation
                                               facecolor='green',
                                               edgecolor='black',
                                               linewidth=0.2,
                                               ls='solid',
                                               alpha=1,
                                               zorder=3)
        ax.add_patch(robot)
        ax.add_patch(gripper)
        #----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], head_width=0.05, head_length=0.1, fc=cmap(i), ec=cmap(i))
        ax.text(X[i][0]-0.1, X[i][1]-0.1, r'$%s$' %i, fontsize=15, fontweight = 'bold',zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize =15,linewidth=3.0)
    if time:
        ax.text(2,5.5,'$t=%.1f s$' %time,
                fontsize=20, fontweight ='bold')                
    # ---set axes ---
    ax.set_aspect('equal')
    ax.set_xlim(-3.0 + target[0], 3.0 + target[0])
    ax.set_ylim(-2.5 + target[1], 2.5 + target[1])
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid('on')
    if name:
        pyplot.savefig(name, dpi = 200)
        #pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color    
