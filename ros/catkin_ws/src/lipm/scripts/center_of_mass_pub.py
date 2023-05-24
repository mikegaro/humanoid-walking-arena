#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import numpy as np
from scipy import signal

xTorso  = 0.12
g       = 9.81
z_c     = 0.68
z_robot = 0.75

stepHeight = 0.2
stepLength = 0.2
dy_mid = 0.06

# Tiempo de muestreo
Ts = 0.01

class LIPMStateSpaceModel():
    a = np.array([
                 [0,    1,  0,  0],
                 [g/z_c,0,  0,  0],
                 [0,    0,  0,  1],
                 [0,    0,g/z_c,0]
                 ])

    b = np.array([
                  [0,0],
                  [1,0],
                  [0,0],
                  [0,1]
                 ])
    
    c = np.array([
                  [1,0,0,0],
                  [0,0,1,0],
                 ])

    d = np.array([
                  [0,0],
                  [0,0]
                 ])
    sys = signal.StateSpace(a, b, c ,d)
    disc_sys = sys.to_discrete(0.01)


def main():

    pub = rospy.Publisher('/limp_trajectory', Int16, queue_size=10)
    rospy.init_node('limp', anonymous=True)
    rate = rospy.Rate(2)

    [dx0, y0, dy0, singleSupportTime] = findInitialConditions(stepLength, dy_mid, xTorso, zModel, g)

    #Initial conditions vector that guarantees symetric trajectories in LIPM
    state0 = [x0, dx0, y0, dy0]
    u0 = [0, 0]

    robotpos0 = [0, 0, z_robot]

    bodyposVector = np.array([])

    footposVector = np.array([])

    timeVector = np.array([])

    #First, lower the body of the robot and move its COM to the right foot
    #This is a harcoded value that can be modify later 
    move_to_starting_Points = [[0,0.67*x0],
                     [0,0],
                     [z_robot, z_c]];

    timepoints = [0,1]
    
    for i in np.arange(timepoints[0],timepoints[1], Ts):
        timeVector.append(i)

    q, qd = 
    

    while not rospy.is_shutdown():
        data = 6
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
  
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

def findInitialConditions(stepLength, dy_mid, x0, zModel, g):
    
    #Desired midstance and state
    y_mid = 0
    
    #Corresponding orbital energy is
    E = -g/(2*zModel) * y_mid**2 + 0.5*dy_mid**2

    y0 = -stepLength/2

    #Finding dy0 from midstance energy level
    dy0 = sqrt(2*(E + g/(2*zModel) * y0**2))

    # using relationship between final body state and initial body state,
    # we can find time it will take to reach midstance given final velocity
    # (dy = dy_mid) and final position (which is y = 0 at midstance)

    tsinglesupport = 2*asinh( stepLength / (2*sqrt(zModel/g)*dy_mid) ) * sqrt(zModel/g)
    
    tf = tsinglesupport/2

    dx0 = -x0/sqrt(zModel/g) * sinh(tf/sqrt(zModel/g)) / cosh(tf/sqrt(zModel/g))

    return [dx0, y0, dy0, singleSupportTime]

    