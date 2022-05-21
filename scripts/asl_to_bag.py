#!/usr/bin/python2
import os

import rosbag
import rospy
from sensor_msgs.msg import Image, CompressedImage, Imu
from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
import cv2
import glob
from tqdm import tqdm

if __name__ == "__main__":
    base_dir = "/home/bjoshi/GoPro9/vio_test/mav0"
    bag_name = "/home/bjoshi/GoPro9/gopro9_vio.bag"
    topic = "cam0/image_raw"

    use_stereo = False
    use_imu = True

    cam0_folder = os.path.join(base_dir, "cam0", "data")
    indx = 0
    files = glob.glob(cam0_folder+"/*")
    files = sorted(files)

    outbag = rosbag.Bag(bag_name, 'w')

    for file in tqdm(files):
        stamp = os.path.split(file)[1].split('.')[0]
        secs = int(float(stamp) * 1e-9)
        n_secs = int(float(stamp) - secs*1e9)
        ros_time = rospy.Time(secs, n_secs)

        header = Header()
        header.stamp = ros_time
        header.frame_id = '/gopro'
        header.seq = indx

        image = cv2.imread(file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        msg.header = header
        msg.encoding = 'mono8'
        outbag.write('/cam0/image_raw', msg, header.stamp)
        indx += 1

    if use_imu:
        imu_file = open(os.path.join(base_dir, "imu0", "data.csv"))
        indx = 0
        # skip the first line asl format
        imu_file.readline()
        lines = imu_file.readlines()
        for line in lines:
            line = line.strip()
            line_arr = line.split(',')
            stamp = line_arr[0]
            secs = int(float(stamp) * 1e-9)
            n_secs = int(float(stamp) - secs*1e9)
            ros_time = rospy.Time(secs, n_secs)

            header = Header()
            header.stamp = ros_time
            header.frame_id = '/gopro'
            header.seq = indx

            angular_vel = Vector3()
            angular_vel.x = float(line_arr[1])
            angular_vel.y = float(line_arr[2])
            angular_vel.z = float(line_arr[3])

            linear_acc = Vector3()
            linear_acc.x = float(line_arr[4])
            linear_acc.y = float(line_arr[5])
            linear_acc.z = float(line_arr[6])

            imu = Imu()
            imu.header = header
            imu.angular_velocity = angular_vel
            imu.linear_acceleration = linear_acc

            outbag.write('/imu0', imu, header.stamp)

    outbag.close()


# cv2.destroyAllWindows()
