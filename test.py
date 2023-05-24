import numpy as np
from scipy import signal, interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

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
    sys = signal.StateSpace(a, b, c, d, dt=Ts)

def main():

    x0 = xTorso

    [dx0, y0, dy0, singleSupportTime] = findInitialConditions(stepLength, dy_mid, x0, z_c, g)

    #Initial conditions vector that guarantees symetric trajectories in LIPM
    state0 = [x0, dx0, y0, dy0]
    u0 = [0, 0]

    robotpos0 = [0, 0, z_robot]

    bodyposVector = np.array([[0,0,0]])

    # Store left and right foothold position (left:odd, right:even)
    footposVector = np.array([
                            [-xTorso, xTorso],
                            [0,0],
                            [0,0]
                            ])

    timeVector = np.array([])

    #First, lower the body of the robot and move its COM to the right foot
    #This is a harcoded value that can be modify later 
    startingPoints = np.array([[0,0.67*xTorso],
                     [0,0],
                     [z_robot, z_c]]);

    timepoints = [0,1]
    
    timeVector = np.arange(timepoints[0],timepoints[1], Ts)

   # [print(i) for i in timeVector]

    print("Size of timeVector is ", timeVector.size)


    m_x = (startingPoints[0][1] - startingPoints[0][0])/(timepoints[1]-timepoints[0])
    print(m_x)
    m_z = (startingPoints[2][1] - startingPoints[2][0])/(timepoints[1]-timepoints[0])
    print(m_z)
    q_x = interpolate.interp1d(timeVector, timeVector*m_x/100, fill_value="extrapolate")
    q_y = 0
    q_z = interpolate.interp1d(timeVector, startingPoints[2][0] + timeVector*m_z/100, fill_value="extrapolate")

    p_z = startingPoints[2][0] + timeVector*m_z

    for i in np.arange(0,len(timeVector)):
        aux = np.array([[q_x(i), q_y, q_z(i)]])
    #    print(bodyposVector[i])
        bodyposVector = np.concatenate((bodyposVector,aux), axis=0)
        
    bodyposVector = np.delete(bodyposVector, 0, axis=0)


    ax = plt.figure().add_subplot(projection='3d')
    #plt.plot(bodyposVector[:,0], bodyposVector[:,1], bodyposVector[:,2], "o")
    #plt.show()

    ## PART2: MAKE A HALF STEP

    starting_halfstep = np.array([[0],[0.1],[z_c]])

    startingPoints = np.concatenate((startingPoints, starting_halfstep), axis=1)

    y_off = startingPoints[1][2]
    print(y_off)

    #Initial walking position
    fhold_x = -x0
    fhold_y = -y0 + y_off
    print("fhold_x: ", fhold_x)
    print("fhold_y: ", fhold_y)

    footposVector = np.concatenate((footposVector, [[fhold_x],[fhold_y],[0]]), axis=1) #left

    readyPos1 = startingPoints[:,1]
    readyPos2 = startingPoints[:,2]
    print(readyPos1)
    print(readyPos2)
    readyVel1 = np.array([0,0,0])
    readyVel2 = np.array([dx0, dy0, 0])

    timepoints = [1, 1.5] #HARDCODING 0.5 SECONDS TO TAKE A HALF STEP

    timeVector = np.array([])
    timeVector = np.arange(timepoints[0],timepoints[1], Ts)

    #Ahi va quedando

    print(timeVector.size)
    print(footposVector)

def findInitialConditions(stepLength, dy_mid, x0, zModel, g):
    
    #Desired midstance and state
    y_mid = 0
    
    #Corresponding orbital energy is
    E = -g/(2*zModel) * y_mid**2 + 0.5*dy_mid**2

    y0 = -stepLength/2

    #Finding dy0 from midstance energy level
    dy0 = math.sqrt(2*(E + g/(2*zModel) * y0**2))

    # using relationship between final body state and initial body state,
    # we can find time it will take to reach midstance given final velocity
    # (dy = dy_mid) and final position (which is y = 0 at midstance)

    tsinglesupport = 2*math.asinh( stepLength / (2*math.sqrt(zModel/g)*dy_mid) ) * math.sqrt(zModel/g)
    
    tf = tsinglesupport/2

    dx0 = -x0/math.sqrt(zModel/g) * math.sinh(tf/math.sqrt(zModel/g)) / math.cosh(tf/math.sqrt(zModel/g))

    return [dx0, y0, dy0, tsinglesupport]

main()
