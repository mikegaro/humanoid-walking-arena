#!/usr/bin/env python
  
import rospy
from std_msgs.msg import Int16
  
  
def publisher():
    pub = rospy.Publisher('/limp_trajectory', Int16, queue_size=10)
    rospy.init_node('limp', anonymous=True)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        data = 6
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()
  
  
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def findInitialConditions(stepLength, dy_mid, x0, zModel, g, Ts):
    
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

    pass