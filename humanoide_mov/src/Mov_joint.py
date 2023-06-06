#!/usr/bin/env python
#Leonardo Gracida Munoz A01379812
#Daniel Fuentes Castro A01750425
#Santiago Ortiz Suzarte A01750402

import rospy
import numpy as np
from trajectory_msgs.msg  import JointTrajectory,JointTrajectoryPoint

class Bogobot_mov():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("Humanoide_Joint_controll")
        #Creamos los publishers de los controladores de cada parte del robot
        self.left_leg_pos = rospy.Publisher("/Bogobot_model/left_leg_controller/command", JointTrajectory,queue_size=10)
        self.left_arm_pos = rospy.Publisher("/Bogobot_model/left_arm_controller/command", JointTrajectory,queue_size=10)
        self.right_leg_pos = rospy.Publisher("/Bogobot_model/right_leg_controller/command", JointTrajectory,queue_size=10)
        self.right_arm_pos = rospy.Publisher("/Bogobot_model/right_arm_controller/command", JointTrajectory,queue_size=10)
        #Largo de las piernas
        self.L = 0.21
        #Posición incial de las piernas
        self.inicial = (0,0,-0.3)
        #Tiempo en el que quiero que complete la trayectoria
        self.tf_inicial = 2
        #Mensajes por segundo
        self.rate = rospy.Rate(50)
    
    #Funcion de interpolacion 
    def q_interpolation(self,qo,qf,t,tf):
        q = (qf -qo)*(10-15*(t/tf)+6*(t/tf)**2)*(t/tf)**3 + qo
        return q
    
    #Cinemática inversa de la pierna izquierda
    def IK_piz(self,Px,Py,Pz):
        c3 = ((Px**2 + Py**2 + Pz**2 - 2*self.L**2)/(2*self.L**2))
        q3 = np.arccos(c3)
        q5 = np.arctan2(-Px,-Pz)
        q4 = np.arctan2(self.L*np.sin(q3)*(Pz*np.cos(q5)+Px*np.sin(q5))-self.L*(1+np.cos(q3))*Py,-self.L*np.sin(q3)*Py-self.L*(1+np.cos(q3))*(Pz*np.cos(q5)+Px*np.sin(q5)))
        q2 = q3+q4
        q1 = q5
        return (q1,q2,q3,q4,q5)
    
    #Cinemática inversa de la pierna derecha
    def IK_pder(self,Px,Py,Pz):
        c9 = ((Px**2 + Py**2 + Pz**2)/(2*self.L**2))-1
        q9 = -np.arccos(c9)
        q11 = np.arctan2(-Px,-Pz)
        q10 = np.arctan2(Py,-Pz*np.cos(q11)-Px*np.sin(q11))-np.arctan2(self.L*np.sin(q9),self.L*(np.cos(q9)+1))
        q8 = q9+q10
        q7 = q11
        return (q7,q8,q9,q10,q11)
    
    #Cinematica inversa del brazo izquierdo
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
        if r <= 0:
            q13 = np.arctan2(-Py/r,-Pz/r)
        return (q13,q14,q15)
    
    #Cinematica inversa brazo derecho
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
    
    #Fucion principal
    def main(self):
        t0 = rospy.get_rostime().to_sec()
        t = 0
        #A todas las IK ingresamos en putno en el espacio que queremos mover los brzoa y piernas
        #Obtenemos cinematica inversa de la pierna izquierda
        grados = self.IK_piz(self.inicial[0],self.inicial[1],self.inicial[2])
        #Obtenemos cinematica inversa de la pierna derecha
        grados_der = self.IK_pder(self.inicial[0],self.inicial[1],self.inicial[2])
        #Obtenemos cinematica inversa del brazo izquierda
        grados_brazo_izq = self.IK_brazo_izq(0.05,0.20,-0.2)
        #Obtenemos cinematica inversa del brazo derecho
        grados_brazo_der = self.IK_brazo_der(-0.05,0.20,-0.2)
        #Iniciamos un contador de tiempo para haer interpolación
        t0 = rospy.get_rostime().to_sec()
        #iniciamos el contador
        while t <= self.tf_inicial:
            #Grupo de joints pierna izquierda
            point = JointTrajectory()
            point.joint_names =  ["q1","q2","q3","q4","q5"]
            #Grupo de joints pierna derecha
            point_der = JointTrajectory()
            point_der.joint_names =  ["q7","q8","q9","q10","q11"]}
            #Grupo de joints brazo izquierdo
            point_arm = JointTrajectory()
            point_arm.joint_names =  ["q13","q14","q15"]
            #Grupo de joints brazo derecho
            point_arm_der = JointTrajectory()
            point_arm_der.joint_names =  ["q16","q17","q18"]
            tf = rospy.get_rostime().to_sec()
            #Obtenemos el diferencial
            dt = tf-t0
            if dt > 0.1:
                dt = 0.001
            #Ingresamos la posicion de los angulos de su posicion inicial a la posicion deseada a lo largo del tiempo para cada joint
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
            #Publicamos la posicion deseada con el tiempo que queremos que la complete

            #Pierna izquierda
            p1 = JointTrajectoryPoint()
            point.header.stamp = rospy.Time.now()
            p1.positions = [q1,q2,q3,q4,q5]
            p1.time_from_start = rospy.Duration.from_sec(dt)
            point.points.append(p1)
            self.left_leg_pos.publish(point)

            #Pierna derecha
            p1 = JointTrajectoryPoint()
            point_arm.header.stamp = rospy.Time.now()
            p1.positions = [q7,q8,q9,q10,q11]
            p1.time_from_start = rospy.Duration.from_sec(dt)
            point_der.points.append(p1)
            self.right_leg_pos.publish(point_der)

            #Brazo izquierdo
            p1 = JointTrajectoryPoint()
            point_arm.header.stamp = rospy.Time.now()
            p1.positions = [q13,q14,q15]
            p1.time_from_start = rospy.Duration.from_sec(dt)
            point_arm.points.append(p1)
            self.left_arm_pos.publish(point_arm)

            #Brazo derecho
            p1 = JointTrajectoryPoint()
            point_arm_der.header.stamp = rospy.Time.now()
            p1.positions = [q16,q17,q18]
            p1.time_from_start = rospy.Duration.from_sec(dt)
            point_arm_der.points.append(p1)
            self.right_arm_pos.publish(point_arm_der)

            #Actualizamos los tiempos
            t0 = tf
            #Aumetnamos el contador
            t += dt
            self.rate.sleep()
if __name__ == '__main__':
    Human = Bogobot_mov()
    Human.main()
