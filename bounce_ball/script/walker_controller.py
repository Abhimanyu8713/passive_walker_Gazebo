#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.msg import JointControllerState
import math
import message_filters
import numpy as np
x =  0
def callback(data_leg1):
    # print(round(data_leg1.process_value_dot,3),' ',round(data_leg2.process_value_dot,3))
    global x
   
    q1 =  round(data_leg1.position[0],3)
    q2 =  round(data_leg1.position[1],3)
    Dq1 = round(data_leg1.velocity[0],3) 
    Dq2 = round(data_leg1.velocity[1],3)
    u = (energy_calculator(q1,q2,Dq1,Dq2))
    x  = u
    # print(x)
    
# def callback2(data):
#     print('Leg2',' ',round(data.process_value_dot,3))

def energy_calculator(q1,q2,Dq1,Dq2):
    mh = 10; m1 = 5; m2 = 5;l = 1;a = 0.5; b= 0.5;g = 9.8;
    p1 = mh*(l**2)+m1*(b**2)+m2*(l**2);
    p2 = m2*l*a;
    p3 = m2*(a**2);
    p4 = (mh*(l)+m1*(b)+m2*(l))*g;
    p5 = m2*a*g;
    # for i = 1:1234
    # q1 = x(i,1);
    # q2 = x(i,2);
    # Dq1 = x(i,3);
    # Dq2 = x(i,4);
    # Dq = [Dq1;Dq2];
    m11 = p1;
    m12 = -p2 * math.cos(q1 - q2);
    m21 = -p2 * math.cos(q1 - q2);
    m22 = p3;

    M = np.array([[m11,m12],[m21,m22]]);

    KE = 0.5*np.matmul(np.matmul(np.array([Dq1,Dq2]),M),np.array([[Dq1],[Dq2]]));
    PE = 9.8*((m1*a+(mh+m2)*l)*math.cos(q1)-m2*b*math.cos(q2));
    diff = (Dq1-Dq2)
    if ((Dq1-Dq2))<0.0001 or ((Dq1-Dq2))>-0.0001 :
        diff = 0.0001
    u = (-0.1/(180*math.pi*diff))*((KE+PE)-155);
    u = int(u)
    # print(u)
    # print(u,KE+PE)
    return(u)
 

def talker():
    rospy.init_node('walker_controller', anonymous=True)
    pub1 = rospy.Publisher('/passive_walker/joint1_effort_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/passive_walker/joint2_effort_controller/command', Float64, queue_size=10)
    leg_sub = message_filters.Subscriber('/passive_walker/joint_states', JointState)

    ts = message_filters.TimeSynchronizer([leg_sub], 10)
    ts.registerCallback(callback)

   
    rate = rospy.Rate(3) # 10hz
    i = 0
    
    
    while not rospy.is_shutdown():
        print(x)
        position1 =  x
        position2 = -x
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub1.publish(position1)
        pub2.publish(-position1)
        # print(position1,position2)
        i = i+(math.pi/100)
        rate.sleep()
        
if __name__ == '__main__':
    
   
    try:
        # rospy.Subscriber('/passive_walker/joint1_effort_controller/state', JointControllerState, callback1)
        # rospy.Subscriber('/passive_walker/joint2_effort_controller/state', JointControllerState, callback2)
   
        talker()
    except rospy.ROSInterruptException:
        pass
