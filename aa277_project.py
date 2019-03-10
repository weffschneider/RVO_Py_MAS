import sys


from RVO import RVO_update, reach, compute_V_des, att_control
from vis import visualize_traj_dynamic
from math import pi as PI
import imageio

#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
ws_model['target_radius'] = 0.4
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for target
ws_model['target'] = [0.4, 0.4]
ws_model['target_vel'] = [0.2,0.12]
target_goal = [2.0, 1.4]

#------------------------------
#initialization for robot 
# position of [x,y]
X = [[-1.0, -1.0], [0.0, -1.0]]
# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [1.0 for i in range(len(X))]
# goal of [x,y]
goal = [[2.0, 2.0], [2.6, 1.4]]
theta = [0.0, 0.0]
theta_goal = [-PI/2.0, PI] # TODO: compute this based on target velocity

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.01

#------------------------------
#initialize images to create animation
images = []

#------------------------------
#simulation starts
t = 0
t_vis = 5 # set the time steps to visualize
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step

    # update target position
    X_T = ws_model['target']
    V_T = ws_model['target_vel']
    if X_T[0] >= target_goal[0] and X_T[1] >= target_goal[1]:
        ws_model['target_vel'] = [0.0,0.0]
    else:
        X_T[0] += V_T[0]*step
        X_T[1] += V_T[1]*step
        ws_model['target'] = X_T
    
    # update attitude
    # TODO: add in an attiude 
    for i in range(len(X)):
        theta[i] += att_control(theta[i], theta_goal[i])*step
    #----------------------------------------
    # visualization
    if t%t_vis == 0:
        img_name = 'data/snap%s.png'%str(t/t_vis)
        # visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.pdf'%str(t/10))
        visualize_traj_dynamic(ws_model, X, V, theta, goal, time=t*step, name=img_name)
        images.append(imageio.imread(img_name))
        
    t += 1
    
imageio.mimsave('data/anim_attitude.gif', images)

    
