#!/usr/bin/python2

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CompressedImage, Image
import argparse
import os
import numpy as np
import cv2
from cv_bridge import CvBridge


class dump_stamps:
    def __init__(self, root_folder):

        self.bridge = CvBridge()
        rospy.Subscriber('/gopro/image/compressed',
                         CompressedImage, self.compressed_image_sub)
        rospy.Subscriber('/gopro/imu', Imu, self.imu_sub)
        rospy.Subscriber('/gopro/image', Image, self.image_sub)

        if not os.path.exists(root_folder):
            os.mkdir(root_folder)
        img_folder = os.path.join(root_folder, 'cam0')
        if not os.path.exists(img_folder):
            os.mkdir(img_folder)
        self.img_data_folder = os.path.join(img_folder, 'data')
        if not os.path.exists(self.img_data_folder):
            os.mkdir(self.img_data_folder)
        file = os.path.join(img_folder, 'data.csv')
        self.img_file = open(file, 'w')
        self.img_file.write('#timestamp [ns],filename\n')

        imu_folder = os.path.join(root_folder, 'imu0')
        if not os.path.exists(imu_folder):
            os.mkdir(imu_folder)
        file = os.path.join(imu_folder, 'data.csv')
        self.imu_file = open(file, 'w')
        self.imu_file.write('#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],'
                            'w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n')

    def __del__(self):
        self.imu_file.close()
        self.img_file.close()

    def imu_sub(self, imu_msg):
        acc = imu_msg.linear_acceleration
        ang_vel = imu_msg.angular_velocity
        self.imu_file.write('{},{},{},{},{},{},{}\n'.format(str(imu_msg.header.stamp), ang_vel.x, ang_vel.y, ang_vel.z,
                                                            acc.x, acc.y, acc.z))

    def compressed_image_sub(self, img_msg):
        np_arr = np.fromstring(img_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imwrite(os.path.join(self.img_data_folder,
                    '{}.png'.format(str(img_msg.header.stamp))), image_np)
        self.img_file.write('{},{}\n'.format(
            str(img_msg.header.stamp), '{}.png'.format(str(img_msg.header.stamp))))

    def image_sub(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')
        cv2.imwrite(os.path.join(self.img_data_folder,
                    '{}.png'.format(str(img_msg.header.stamp))), cv_image)
        self.img_file.write('{},{}\n'.format(
            str(img_msg.header.stamp), '{}.png'.format(str(img_msg.header.stamp))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Save memory and CPU usage")
    parser.add_argument('--base_dir', help='file to record pose')
    args = parser.parse_args()
    root_folder = args.base_dir

    rospy.init_node("asl_format")

    dump_stamps(root_folder)

    while not rospy.is_shutdown():
        rospy.spin()
