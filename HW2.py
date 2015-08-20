#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import Astar_8_euclidean as Astar

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####

        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.

    # initialize start state
    initial = Astar.State(x=-34, y=-14, goalX=26, goalY=-13)

    # initialize A* solver to find path
    solver = Astar.Solver(initial)
    solver.search()

    
    # draw red lines around tables

    handles = []
    
    handles.append(env.drawlinestrip(points=array(((-3.3,-1.0,0.02),(-3.3,1.0,0.02),(-0.2,1.0,0.02),(-0.2,-1.0,0.02),(-3.3,-1.0,0.02))),
                                     linewidth=3.0, colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))

    handles.append(env.drawlinestrip(points=array(((2.8,-1.8,0.02),(2.8,-0.2,0.02),(3.8,-0.2,0.02),(3.8,-1.8,0.02),(2.8,-1.8,0.02))),
                                     linewidth=3.0, colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))

    handles.append(env.drawlinestrip(points=array(((2.8,0.2,0.02),(2.8,1.8,0.02),(3.8,1.8,0.02),(3.8,0.2,0.02),(2.8,0.2,0.02))),
                                     linewidth=3.0, colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))



    # waitrobot(robot)

        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)

    path = solver.getPath()

    if len(path) != 0:
        handles.append(env.drawlinestrip(points=array(path), 
                                         linewidth=4.0,
                                         colors=array(((0,0,0)))))

        #### Draw the X and Y components of the configurations explored by A*

    closedSet = solver.getSet(solver.closedSet)
    collisionSet = solver.getSet(solver.collisionSet)

    if len(closedSet) != 0:
        handles.append(env.plot3(points = array(closedSet),
                                 pointsize = 0.03,
                                 colors = array(((0,0,1))),
                                 drawstyle = 1))

    if len(collisionSet) != 0:
        handles.append(env.plot3(points = array(collisionSet),
                                      pointsize = 0.03,
                                      colors = array(((1,0,0))),
                                      drawstyle = 1))


        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);

     # create empty trajectory
    traj = RaveCreateTrajectory(env, '')

    # get config spec for trajectory, choose linear interpolation method, add timestamp to config spec
    config = robot.GetActiveConfigurationSpecification('linear')
    config.AddDeltaTimeGroup()

    # initialize the trajectory
    traj.Init(config)

    # create C-space path with time stamps from path curve
    # set delta time to be 0.003
    myPath = [[point[0], point[1], -pi/2, i*0.004] for i, point in enumerate(path)]

    # fill trajectory with waypoints
    for i, wayPoint in enumerate(myPath):
        traj.Insert(i, wayPoint, config, True)
        
    
    robot.GetController().SetPath(traj)
    

        #### END OF YOUR CODE ###
    # waitrobot(robot)

    raw_input("Press enter to exit...")

