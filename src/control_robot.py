#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, Point


class ControlNode:
    def __init__(self):
        rospy.init_node('control_node')
        #----Inizializzazione del service
        self.start_service = rospy.Service('start', Empty, self.start_callback)

        #----Publisher per i vari topic----

        #Publisher per Panda
        self.status_job_pub = rospy.Publisher('status_job', Bool, queue_size=10)
        self.target_arm_pub = rospy.Publisher('target_arm', Pose, queue_size=10)
        self.endeffector_status_pub = rospy.Publisher('endeffector_status', Point, queue_size=10)

        #Publisher per la macchina a stati
        self.target_topic_pub = rospy.Publisher('target_topic', Bool, queue_size=10)
        self.picked_topic_pub = rospy.Publisher('picked_topic', Bool, queue_size=10)
        self.stop_topic_pub = rospy.Publisher('stop_topic', Bool, queue_size=10)
        self.start_topic_pub = rospy.Publisher('start_topic', Bool, queue_size=10)

        #----Subscriber per i vari topic----
        rospy.Subscriber('/status', String, self.status_callback)
        rospy.Subscriber('status_job', Bool, self.status_job_callback)

        #----Definizione delle variabili di controllo----
        self.status = None
        self.status_job = False        

        #----Posizioni predefinite----
        self.home_pose = Pose()
        self.home_pose.position.x = 0.2
        self.home_pose.position.y = 0.4
        self.home_pose.position.z = 0.4
        self.home_pose.orientation.x=1

        self.pick_pose = Pose()
        self.pick_pose.position.x = 0.2
        self.pick_pose.position.y = 0.6
        self.pick_pose.position.z = 0.4
        self.pick_pose.orientation.x=1

        self.place_pose = Pose()
        self.place_pose.position.x = 0.2
        self.place_pose.position.y = 0.2
        self.place_pose.position.z = 0.4
        self.place_pose.orientation.x=1

        self.close_pose = Point()
        self.close_pose.x = 0.001
        self.close_pose.y = 0.001

        self.open_pose = Point()
        self.open_pose.x = 0.035
        self.open_pose.y = 0.035

    # Callback per aggiornare lo stato corrente
    def status_callback(self, msg):
        self.status = msg.data

    # Callback per aggiornare lo stato di Panda
    def status_job_callback(self, msg):
        self.status_job = msg.data

    # Funzione per eseguire le azioni in base allo stato corrente
    def start_action(self):
        # Ciclo fino a quando lo stato di stopping non diventa True
        while True:
            # Definisci qui le azioni di avvio in base allo stato corrente
            rospy.loginfo("Start action...")
            if self.status == "Wait":
                rospy.loginfo("WAIT action...")
                # Pubblica sul topic "start_topic" un messaggio booleano True
                self.start_topic_pub.publish(True)

            elif self.status == "GoTo":
                rospy.loginfo("GOTO action...")
                # Pubblica sul topic "target_arm" il messaggio pose "Home_pose"
                #self.target_arm_pub.publish(self.home_pose)
                # Aspetta che il topic "status_job" sia True
                #self.wait_for_job_completion()

                # Pubblica sul topic "target_topic" un messaggio booleano True
                self.target_topic_pub.publish(True)
            elif self.status == "Pick":
                rospy.loginfo("PICK action...")
                # Pubblica sul topic "target_Arm" la posa "posa_pick"
                self.target_arm_pub.publish(self.pick_pose)
                # Aspetta che il topic "status_job" sia True
                #self.wait_for_job_completion()
                
                # Pubblica sul topic "Endeffector_status" la posa "Close"
                self.endeffector_status_pub.publish(self.close_pose)
                # Aspetta che il topic "status_job" sia True
                #self.wait_for_job_completion()
                #rospy.sleep(6)

                self.status_job_pub=True
                while not rospy.is_shutdown() and self.status_job:
                    # Pubblica sul topic "picked_topic" il valore booleano True
                    self.picked_topic_pub.publish(True)

            elif self.status == "Place":
                rospy.loginfo("PLACE action...")
                # Pubblica sul topic "target_Arm" la posa "posa_place"
                self.target_arm_pub.publish(self.place_pose)
                # Aspetta che il topic "status_job" sia True
                #self.wait_for_job_completion()

                # Pubblica sul topic "Endeffector_status" la posa "Open"
                self.endeffector_status_pub.publish(self.open_pose)                
                
                self.status_job_pub=True
                
                while not rospy.is_shutdown() and self.status_job:
                    # Pubblica sui topic "picked_topic" e "target_topic" il valore booleano False
                    self.picked_topic_pub.publish(False)
                    self.target_topic_pub.publish(False)

    # Funzione per attendere il completamento del lavoro
    def wait_for_job_completion(self):
        while not rospy.is_shutdown() and not self.status_job:
            rospy.sleep(1)

    # Funzione per eseguire le azioni di stop
    def stop_action(self):
        # Definisci qui le azioni di stop
        rospy.loginfo("Stop action...")
        self.stop_topic_pub.publish(True) # Pubblica sul topic "stop_topic"
        rospy.sleep(3)
        rospy.loginfo("Stop action completed.")
    
    # Callback per gestire la richiesta di avvio
    def start_callback(self, req):
        self.running = True
        rospy.loginfo("Start command received. Starting...")
        self.start_action()     

    # Funzione per eseguire il nodo
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    control_node = ControlNode()
    control_node.run()
