#!/usr/bin/python2
import os

import rosbag
import rospy
from sensor_msgs.msg import Image, CompressedImage, Imu
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
import glob
from tqdm import tqdm

if __name__ == "__main__":
    base_dir = "/home/bjoshi/GoPro9/vio_test/mav0"
    cam0_folder = os.path.join(base_dir, "cam0", "data")
    indx = 0
    files = glob.glob(cam0_folder+"/*")
    files = sorted(files)

    orb_file = open('/home/bjoshi/GoPro9/orbslam_vio_test.txt', 'w')

    for file in tqdm(files):
        stamp = os.path.split(file)[1].split('.')[0]
        orb_file.write(stamp)
        orb_file.write('\n')
        indx += 1

    orb_file.close()


# cv2.destroyAllWindows()
