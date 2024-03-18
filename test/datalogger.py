#!/usr/bin/env python3

import rospy
import csv
from fssim_common.msg import CarInfo, State
from sensor_msgs.msg import JointState

class DataLoggerNode:
    def __init__(self):
        rospy.init_node('data_logger_node', anonymous=True)
        
        # Subscriber for car_info
        self.car_info_sub = rospy.Subscriber("/fssim/car_info", CarInfo, self.car_info_callback)
        
        # Subscriber for base_pose_ground_truth
        self.base_pose_sub = rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.base_pose_callback)
        
        # Subscriber for joint_states
        # self.joint_states_sub = rospy.Subscriber("/fssim/joint_states", JointState, self.joint_states_callback)
        
        # Open CSV files for writing
        self.car_info_csv = open("car_info.csv", "w", newline='')
        self.car_info_writer = csv.writer(self.car_info_csv)
        self.car_info_writer.writerow(["Time", "Drag_Force_X", "Drag_Force_Y", "Drag_Force_Z", "delta", "DC", "Front_Left_Steering_Angle", "Front_Right_Steering_Angle", "Delta_Measured", "Vx", "Vy", "R", "Alpha_F", "Alpha_F_Left", "Alpha_F_Right", "Alpha_R_Left", "Alpha_R", "Alpha_R_Right", "Fy_F", "Fy_F_Left", "Fy_F_Right", "Fy_R", "Fy_R_Left", "Fy_R_Right", "Fx"])
        
        self.base_pose_csv = open("base_pose_ground_truth.csv", "w", newline='')
        self.base_pose_writer = csv.writer(self.base_pose_csv)
        self.base_pose_writer.writerow(["Time", "X", "Y", "Yaw", "Vx", "Vy", "R"])
        
        # self.joint_states_csv = open("joint_states.csv", "w", newline='')
        # self.joint_states_writer = csv.writer(self.joint_states_csv)
        # self.joint_states_writer.writerow(["Time", "Name", "Position", "Velocity", "Effort"])

    def car_info_callback(self, data):
        row = [
            rospy.Time.now(),
            data.drag_force.vec.x,
            data.drag_force.vec.y,
            data.drag_force.vec.z,
            data.delta,
            data.dc,
            data.front_left_steering_angle,
            data.front_right_steering_angle,
            data.delta_measured,
            data.vx,
            data.vy,
            data.r,

            data.alpha_f,
            data.alpha_f_l,
            data.alpha_f_r,
            data.alpha_r_l,
            data.alpha_r,
            data.alpha_r_r,
            data.Fy_f,
            data.Fy_f_l,
            data.Fy_f_r,
            data.Fy_r,
            data.Fy_r_l,
            data.Fy_r_r,
            data.Fx
        ]
        self.car_info_writer.writerow(row)

    def base_pose_callback(self, data):
        row = [
            rospy.Time.now(),
            data.x,
            data.y,
            data.yaw,
            data.vx,
            data.vy,
            data.r
        ]
        self.base_pose_writer.writerow(row)

    # def joint_states_callback(self, data):
    # # Check if data fields are populated
    #     if data.name and data.position and data.velocity:
    #         # Iterate over the length of the shortest list
    #         for i in range(min(len(data.name), len(data.position), len(data.velocity))):
    #             # Check if effort list is available
    #             if data.effort:
    #                 effort = data.effort[i] if len(data.effort) > i else None
    #             else:
    #                 effort = None

    #         row = [
    #             rospy.Time.now(),
    #             data.name[i],
    #             data.position[i] if len(data.position) > i else None,
    #             data.velocity[i] if len(data.velocity) > i else None,
    #             effort
    #         ]
    #         self.joint_states_writer.writerow(row)
    #     else:
    #         rospy.logwarn("Received JointState message with missing data fields")


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        data_logger_node = DataLoggerNode()
        data_logger_node.run()
    except rospy.ROSInterruptException:
        pass