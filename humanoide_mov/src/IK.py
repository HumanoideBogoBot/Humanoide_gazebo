#!/usr/bin/env python

import tf, rospy
import numpy as np
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, Point, Quaternion,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class Humanoide():
    def __init__(self):
        rospy.init_node("Humanoide_IK")
        self.pJS = rospy.Publisher('/Bogobot_model/joint_states', JointState,queue_size=10)
        self.fs = 10
        self.tf = 1
        self.tf_inicial = 3
        self.L = 0.21
        rate = rospy.Rate(self.fs)
        self.inicial = (0,0,-2*self.L+0.075)
        self.js = JointState()
    
    def q_interpolation(self,qo,qf,t,tf):
        q = (qf -qo)*(10-15*(t/tf)+6*(t/tf)**2)*(t/tf)**3 + qo
        return q
    def IK_piz(self,Px,Py,Pz):
        c3 = ((Px**2 + Py**2 + Pz**2 - 2*self.L**2)/(2*self.L**2))
        q3 = np.arccos(c3)
        q5 = np.arctan2(-Px,-Pz)
        q4 = np.arctan2(self.L*np.sin(q3)*(Pz*np.cos(q5)+Px*np.sin(q5))-self.L*(1+np.cos(q3))*Py,-self.L*np.sin(q3)*Py-self.L*(1+np.cos(q3))*(Pz*np.cos(q5)+Px*np.sin(q5)))
        q2 = q3+q4
        q1 = q5
        return (q1,q2,q3,q4,q5)
    
    def IK_pder(self,Px,Py,Pz):
        c9 = ((Px**2 + Py**2 + Pz**2)/(2*self.L**2))-1
        q9 = -np.arccos(c9)
        q11 = np.arctan2(-Px,-Pz)
        q10 = np.arctan2(Py,-Pz*np.cos(q11)-Px*np.sin(q11))-np.arctan2(self.L*np.sin(q9),self.L*(np.cos(q9)+1))
        q8 = q9+q10
        q7 = q11
        return (q7,q8,q9,q10,q11)
    def IK_brazo_izq(self,Px,Py,Pz):
        L1 = 0.2
        L2 = 0.19
        c15 = ((Px**2+Py**2+Pz**2-L1**2-L2**2)/(2*L1*L2))
        q15 = -np.arccos(c15)
        if (1-np.cos(q15)**2) < 0:
            q15 = np.arccos(c15)
        q14 = np.arctan2(Px,np.sqrt(Py**2+Pz**2))-np.arctan2(-L2*np.sin(q15),L1+L2*np.cos(q15))
        s = np.sin(q14-q15)
        c = np.cos(q14-q15)
        r = L1*np.cos(q14) + L2*c
        q13 = np.arctan2(-Py,-Pz)
        print(r)
        if r <= 0:
            q13 = np.arctan2(-Py/r,-Pz/r)
        return (q13,q14,q15)
    def IK_brazo_der(self,Px,Py,Pz):
        L1 = 0.2
        L2 = 0.19
        c18 = ((Px**2+Py**2+Pz**2-L1**2-L2**2)/(2*L1*L2))
        q18 = np.arccos(c18)
        if (1-np.cos(q18)**2) < 0:
            q18 = -np.arccos(c18)
        q17 = np.arctan2(Px,np.sqrt(Py**2+Pz**2))-np.arctan2(-L2*np.sin(q18),L1+L2*np.cos(q18))
        s = np.sin(q17-q18)
        c = np.cos(q17-q18)
        r = L1*np.cos(q17) + L2*c
        q16 = np.arctan2(Py/r,-Pz/r)
        self.q = []
        return (q16,q17,q18)

    
    def posicion_inicial(self):
        T = 1.0/self.fs
        to = rospy.Time.now()
        t0 = rospy.get_rostime().to_sec()
        t = 0
        grados = self.IK_piz(self.inicial[0],self.inicial[1],self.inicial[2])
        grados_der = self.IK_pder(self.inicial[0],self.inicial[1],self.inicial[2])
        grados_brazo_izq = self.IK_brazo_izq(0.05,0.20,-0.2)
        grados_brazo_der = self.IK_brazo_der(-0.05,0.20,-0.2)
        print(grados_brazo_der)
        while t <= self.tf_inicial:
            tf = rospy.get_rostime().to_sec()
            dt = tf-t0
            if dt > 0.1:
                continue
            q1 = self.q_interpolation(0,grados[0],t,self.tf_inicial)
            q2 = self.q_interpolation(0,grados[1],t,self.tf_inicial)
            q3 = self.q_interpolation(0,grados[2],t,self.tf_inicial)
            q4 = self.q_interpolation(0,grados[3],t,self.tf_inicial)
            q5 = self.q_interpolation(0,grados[4],t,self.tf_inicial)
            q7 = self.q_interpolation(0,grados_der[0],t,self.tf_inicial)
            q8 = self.q_interpolation(0,grados_der[1],t,self.tf_inicial)
            q9 = self.q_interpolation(0,grados_der[2],t,self.tf_inicial)
            q10 = self.q_interpolation(0,grados_der[3],t,self.tf_inicial)
            q11 = self.q_interpolation(0,grados_der[4],t,self.tf_inicial)
            q13 = self.q_interpolation(0,grados_brazo_izq[0],t,self.tf_inicial)
            q14 = self.q_interpolation(0,grados_brazo_izq[1],t,self.tf_inicial)
            q15 = self.q_interpolation(0,grados_brazo_izq[2],t,self.tf_inicial)
            q16 = self.q_interpolation(0,grados_brazo_der[0],t,self.tf_inicial)
            q17 = self.q_interpolation(0,grados_brazo_der[1],t,self.tf_inicial)
            q18 = self.q_interpolation(0,grados_brazo_der[2],t,self.tf_inicial)
            cTime = rospy.Time.now()
            self.js.name = ["q19", "q20", "q13", "q14", "q15", "q16", "q17", "q18", "q5", "q4", "q3", "q2", "q1", "q11", "q10", "q9", "q8", "q7"]
            self.q = [0.0, 0.0, q13, q14, q15, q16,q17,q18, q5, q4, q3, q2, q1, q11, q10, q9, q8, q7]
            self.js.position = self.q
            self.js.header.stamp = cTime
            self.pJS.publish(self.js)
            t0 = tf
            t += dt
            #print(t)



    
    def main(self):
        T = 1.0/self.fs
        to = rospy.Time.now()
        t0 = rospy.get_rostime().to_sec()
        t = 0
        self.posicion_inicial()
        grados_brazo_der = self.IK_brazo_der(0.05,0.20,-0.2)
        while not rospy.is_shutdown():
            cTime = rospy.Time.now()
            self.js.header.stamp = cTime
            self.pJS.publish(self.js)


if __name__ == '__main__':
    Human = Humanoide()
    Human.main()
