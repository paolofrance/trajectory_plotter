#!/usr/bin/env python3
# license removed for brevity
import copy
import numpy as np
import random
import rospy
from geometry_msgs.msg import PoseStamped, Pose
# from std_msgs.msg import Float32
import os
import signal
import subprocess
import rospkg


class CurrentPoseMonitor:

    def __init__(self):
        self.current_pose = Pose()

    def current_callback(self, data):
        self.current_pose = data.pose


def traj_publisher():
    rospy.init_node('traj_publisher')

    cpm = CurrentPoseMonitor()
    rospy.sleep(.1)

    nominal_traj_topic = rospy.get_param('nominal_traj_topic')
    #human_traj_topic   = rospy.get_param('human_traj_topic')
    current_traj_topic = rospy.get_param('current_traj_topic')
    obst_x_coord_topic = rospy.get_param('obst_x_coord_topic')
    current_sub        = rospy.Subscriber(current_traj_topic  , PoseStamped, cpm.current_callback)

    print('\n PUBLISHED TOPICS:')
    print(nominal_traj_topic)
    #print(human_traj_topic)
    print(obst_x_coord_topic)

    nom_pub  = rospy.Publisher(nominal_traj_topic, PoseStamped, queue_size=10)
    #hum_pub  = rospy.Publisher(human_traj_topic  , PoseStamped, queue_size=10)
    obst_pub = rospy.Publisher(obst_x_coord_topic, PoseStamped, queue_size=10)
    rate     = 125.0
    ros_rate = rospy.Rate(rate)
    t        = 0.0


    rospy.sleep(8)
    reference_pose = copy.deepcopy(cpm.current_pose)
    initial_pose   = copy.deepcopy(cpm.current_pose)
    print('\n INITIAL POSE: ')
    print(str(initial_pose) + '\n')

    # PARAM FOR CREATING THE TRAJECTORY
    # ang vel of coord X
    cos_freq     = 0.05
    omega        = 2 * np.pi * cos_freq
    vel_return   = 0.2
    # param for sine shape
    amp_traj2 = 0.15
    #rospy.set_param("amp_traj2", amp_traj2)
    # param for S-shape
    cos_freq3 = 0.09
    omega3    = 2 * np.pi * cos_freq3
    amp_traj3 = 0.15
    #rospy.set_param("amp_traj3", amp_traj3)
    t_vert = 3 

    x0 = 0              # start
    x5 = 0.5            # end, total length along x
    p1 = 0.4            # begin circ traj in x5%
    R =  x5 * (0.5 - p1)# Radius of cric trajs
    p3 = 0.6            # begin second circ traj in x5%

    # Ask if you want a random init pause
    rand_pause = input("Do you want random initial pause? y/n \n")

    # Time [s] wait time before/after the reference point start/end moving, during these pauses rosbag is recording!
    if rand_pause == 'y' or rand_pause == 'Y':
        wait = random.randint(2, 5)
    else:
        wait = 2
    end_wait = 1
    #final_wait = 5

    # TRAJECTORY CASES:
    case_traj = input("Choose the trajectory case? (1: linear - 2: sine - 3: S shape) \n")

    # HEIGHT OF NOM.TRAJ FROM STARTING POINT
    delta_z = input("Insert height of nom. traj from starting point [cm] ( \n")
    
    #rospy.set_param("wait_param", False)
    # 1: nom traj line
    # 2: nom traj sine
    # 3: nom traj "S-shape"   "-sin" -> "vertical line" -> "sin"

    # Run a bash file containing 'rosbag record -a'
    record = input("Do you want to record? y/n \n")

    if record == 'y' or record == 'Y':
        record = True
        PATH = rospkg.RosPack().get_path('trajectory_plotter')
        run = "0"
        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        print("Acquisition of bag record, DON'T QUIT \n")
    else:
        record = False
        run = '0'
        print("Script is running - NO ACQUISITION \n")
    rec = record

    rnd_obs = input("Do you want rand obstacle? y/n \n")
    if rnd_obs == 'y' or rnd_obs == 'Y':
        rand_obst = True
    else:
        rand_obst = False

    obst_pose_msg = PoseStamped()
    obst_pose_msg.pose.position.x = 0.275

    while run != "stop" and (not rospy.is_shutdown()):
        
        # CREATE THE OBSTACLE
        if t == 0 and rand_obst == True and case_traj == "1":
            obst_pose_msg.pose.position.x = (np.random.randint(100, 400, 1) / 1000) - 0.03
            obst_pose_msg.pose.position.y = 0
        elif t == 0 and rand_obst == True and case_traj == "2":
            obst_pose_msg.pose.position.x = (np.random.randint(150, 400, 1) / 1000) - 0.03
            obst_pose_msg.pose.position.y = amp_traj2 * np.sin((obst_pose_msg.pose.position.x) * np.pi/x5) - 0.02
        elif t == 0 and rand_obst == True and case_traj == "3":
            #rospy.set_param("case_traj", "3")
            obst_pose_msg.pose.position.x = 0.5 * x5 - 0.03
            obst_pose_msg.pose.position.y = np.random.rand(1) * (amp_traj3 - R - 0.02) - 0.02
        elif t == 0 and rand_obst == False:
            obst_pose_msg.pose.position.x = 0
            obst_pose_msg.pose.position.y = 0
        

        t += 1.0/rate

        # HUMAN POSE
        hum_pose_msg = PoseStamped()


        if case_traj == "1":
            # HUMAN AND ROBOT REFERNCE X:
            if t < wait:   # wait 3 seconds
                #hum_pose_msg.pose.position.x = initial_pose.position.x
                reference_pose.position.x    = initial_pose.position.x
            elif (t-wait) <= (0.5/cos_freq):
                #hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
            elif (t - wait) <= (0.5 / cos_freq) + end_wait:
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5
                reference_pose.position.x = initial_pose.position.x + x5
            elif (t-wait) <= (0.5 / cos_freq) + (x5 / vel_return) + end_wait:
                if rec:
                    os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_return * (t - wait - (0.5 / cos_freq) - end_wait))
                reference_pose.position.x = initial_pose.position.x + x5 - (vel_return * (t - wait - (0.5 / cos_freq) - end_wait))
            else:
                reference_pose.position.x    = initial_pose.position.x
                run = input('Press enter to continue, else digit "stop" \n')
                if run != "stop":
                    t = 0
                    rec = record
                    if rec:
                        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                        print("Acquisition of bag record, DON'T QUIT \n")
                    else:
                        print("Script is running... \n")
            
            # human target Y:
            # hum_pose_msg.pose.position.y = initial_pose.position.y
            # REFERENCE POSITION Y
            reference_pose.position.y    = initial_pose.position.y

        elif case_traj == "2":
            # HUMAN AND ROBOT REFERNCE X:
            if t < wait:   # wait 3 seconds
                #hum_pose_msg.pose.position.x = initial_pose.position.x
                reference_pose.position.x    = initial_pose.position.x
            elif (t-wait) <= (0.5/cos_freq):
                #hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega * (t - wait)))
            elif (t - wait) <= (0.5 / cos_freq) + end_wait:
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5
                reference_pose.position.x = initial_pose.position.x + x5
            elif (t-wait) <= (0.5 / cos_freq) + (x5 / vel_return) + end_wait:
                if rec:
                    os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_return * (t - wait - (0.5 / cos_freq) - end_wait))
                reference_pose.position.x = initial_pose.position.x + x5 - \
                                            (vel_return * (t - wait - (0.5 / cos_freq) - end_wait))
            else:
                reference_pose.position.x    = initial_pose.position.x
                run = input('Press enter to continue, else digit "stop" \n')
                if run != "stop":
                    t = 0
                    rec = record
                    if rec:
                        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                        print("Acquisition of bag record, DON'T QUIT \n")
                    else:
                        print("Script is running... \n")
            
            # HUMAN AND ROBOT REFERNCE Y:
            if (t - wait) <= (0.5 / cos_freq) + end_wait:
                #hum_pose_msg.pose.position.y = initial_pose.position.y + amp_traj2 * np.sin((hum_pose_msg.pose.position.x - initial_pose.position.x) * np.pi/x5)
                reference_pose.position.y    = initial_pose.position.y + amp_traj2 * np.sin((reference_pose.position.x - initial_pose.position.x) * np.pi/x5)
            else:
                #hum_pose_msg.pose.position.y = initial_pose.position.y
                reference_pose.position.y    = initial_pose.position.y

        elif case_traj == "3":        # X as cosine -> vert (x=const) -> X as cosine
            if t < wait:   # wait 3 seconds
                #hum_pose_msg.pose.position.x = initial_pose.position.x
                reference_pose.position.x    = initial_pose.position.x
            elif (t-wait) <= (0.5/cos_freq3):
                #hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega3 * (t - wait))) * 0.5
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 * (1 - np.cos(omega3 * (t - wait))) * 0.5
            elif (t-wait) <= (0.5/cos_freq3) + t_vert:
                #hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5
            elif (t-wait) <= (0.5/cos_freq3) + t_vert + (0.5/cos_freq3):
                #hum_pose_msg.pose.position.x = initial_pose.position.x + 0.5 * x5 + 0.5 * x5 * (1 - np.cos(omega3 * (t - wait - (0.5/cos_freq3) - t_vert))) * 0.5 
                reference_pose.position.x    = initial_pose.position.x + 0.5 * x5 + 0.5 * x5 * (1 - np.cos(omega3 * (t - wait - (0.5/cos_freq3) - t_vert))) * 0.5 
            elif (t - wait) <= (0.5/cos_freq3) + t_vert + (0.5/cos_freq3) + end_wait:
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5
                reference_pose.position.x = initial_pose.position.x + x5
            elif (t-wait) <= (0.5/cos_freq3) + t_vert + (0.5/cos_freq3) + (x5 / vel_return) + end_wait:
                if rec:
                    os.killpg(os.getpgid(record_process.pid), signal.SIGTERM)
                #hum_pose_msg.pose.position.x = initial_pose.position.x + x5 - (vel_return * (t - wait - ((0.5/cos_freq3) + t_vert + (0.5/cos_freq3) + end_wait)))
                reference_pose.position.x =    initial_pose.position.x + x5 - (vel_return * (t - wait - ((0.5/cos_freq3) + t_vert + (0.5/cos_freq3) + end_wait)))
            else:
                reference_pose.position.x    = initial_pose.position.x
                run = input('Press enter to continue, else digit "stop" \n')
                if run != "stop":
                    t = 0
                    rec = record
                    if rec:
                        record_process = subprocess.Popen(PATH + "/script/bag_record.sh", start_new_session=True,
                                                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                        print("Acquisition of bag record, DON'T QUIT \n")
                    else:
                        print("Script is running... \n")
            
            # HUMAN AND ROBOT REFERNCE Y:
            if (t-wait) <= np.arccos(1 - 4 * p1) / omega3:
                #hum_pose_msg.pose.position.y = initial_pose.position.y - amp_traj3 * np.sin((hum_pose_msg.pose.position.x - initial_pose.position.x) * np.pi/(x5 * p1 *2))
                reference_pose.position.y    = initial_pose.position.y - amp_traj3 * np.sin((reference_pose.position.x - initial_pose.position.x) * np.pi/(x5 * p1 *2))
            elif (t-wait) <= np.arccos(1 - 4 * 0.5) / omega3:
                #hum_pose_msg.pose.position.y = initial_pose.position.y + R - amp_traj3 - np.sqrt(R**2 - (reference_pose.position.x - initial_pose.position.x - p1 * x5)**2)
                reference_pose.position.y    = initial_pose.position.y + R - amp_traj3 - np.sqrt(R**2 - (reference_pose.position.x - initial_pose.position.x - p1 * x5)**2)
            elif (t-wait) <= np.arccos(1 - 4 * 0.5) / omega3 + t_vert:
                t_v = t - wait - np.arccos(1 - 4 * 0.5) / omega3
                #hum_pose_msg.pose.position.y = initial_pose.position.y - amp_traj3 + R + 2*(amp_traj3-R) * t_v / t_vert
                reference_pose.position.y = initial_pose.position.y - amp_traj3 + R + 2*(amp_traj3-R) * t_v / t_vert
            elif (t-wait) <= np.arccos(1 - 4 * 0.5) / omega3 + t_vert + np.arccos(1 - 4 * (p3 - 0.5)) / omega3:
                #hum_pose_msg.pose.position.y = initial_pose.position.y - R + amp_traj3 + np.sqrt(R**2 - (reference_pose.position.x - initial_pose.position.x - 0.5*x5 - R)**2)
                reference_pose.position.y = initial_pose.position.y - R + amp_traj3 + np.sqrt(R**2 - (reference_pose.position.x - initial_pose.position.x - 0.5*x5 - R)**2)
            elif (t-wait) <= np.arccos(1 - 4 * 0.5) / omega3 + t_vert + np.arccos(1 - 4 * 0.5) / omega3:
                #hum_pose_msg.pose.position.y = initial_pose.position.y + amp_traj3 - amp_traj3 * (1 - np.sin((hum_pose_msg.pose.position.x - initial_pose.position.x - 0.5*x5*p1) * np.pi/(x5 * p1 *2)))
                reference_pose.position.y    = initial_pose.position.y + amp_traj3 - amp_traj3 * (1 - np.sin((reference_pose.position.x - initial_pose.position.x - 0.5*x5*p1) * np.pi/(x5 * p1 *2)))
            else:
                #hum_pose_msg.pose.position.y = initial_pose.position.y
                reference_pose.position.y = initial_pose.position.y


        # REFERENCE POSITION Z
        # REFERENCE POSITION Z
        if t < wait:
            reference_pose.position.z    = initial_pose.position.z
        else:
            reference_pose.position.z    = initial_pose.position.z + delta_z/100
        #hum_pose_msg.pose.position.z = initial_pose.position.z

        

        stamp = rospy.Time.now()
        #hum_pose_msg.header.stamp = stamp
        obst_pose_msg.header.stamp = stamp
        nom_pose_msg              = PoseStamped()
        nom_pose_msg.pose         = reference_pose
        nom_pose_msg.header.stamp = stamp

        #hum_pub.publish(hum_pose_msg)
        nom_pub.publish(nom_pose_msg)
        obst_pub.publish(obst_pose_msg)
        ros_rate.sleep()


if __name__ == '__main__':
    traj_publisher()
