import sys


from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic

#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.2
ws_model['target_radius'] = 0.4
#circular obstacles, format [x,y,rad]
# with obstacles
ws_model['circular_obstacles'] = [[0.4, 0.4, 0.4], [2.0, 1.4, 0.4]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for robot 
# position of [x,y]
X = [[2.0, 0.0], [3.0, 0.0]]
# velocity of [vx,vy]
V = [[0,0] for i in range(len(X))]
# maximal velocity norm
V_max = [1.0 for i in range(len(X))]
# goal of [x,y]
goal = [[2.0, 2.0], [2.6, 1.4]]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 6
# simulation step
step = 0.01

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%100 == 0:
        
        # visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.pdf'%str(t/10))
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
    t += 1

    
