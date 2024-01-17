#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import Pose

def detect_color_object(cv_image, lower_color, upper_color):
    # Görüntüyü HSV renk uzayına dönüştür
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Renk sınırlarını kullanarak maske oluştur
    mask = cv2.inRange(hsv, lower_color, upper_color)
    # Kontur bul ve en büyüğünü seç
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        return max(contours, key=cv2.contourArea)
    return None


def detect_red_object(cv_image):
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    return detect_color_object(cv_image, lower_red1, upper_red1)

def detect_green_object(cv_image):
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])
    return detect_color_object(cv_image, lower_green, upper_green)

def detect_blue_object(cv_image):
    lower_blue = np.array([110, 100, 100])
    upper_blue = np.array([130, 255, 255])
    return detect_color_object(cv_image, lower_blue, upper_blue)


def compute_3d_coordinates(cx, cy):
    x = cx * 0.001
    y = cy * 0.001
    z = 0.5
    return x, y, z

def move_robot(x, y, z):
    robot = RobotCommander()
    group = MoveGroupCommander("panda_arm")

    target_pose = Pose()
    target_pose.orientation.w = 1.0
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    group.set_planning_time(10)
    group.set_planner_id("geometric::RRTConnect")
    group.set_pose_target(target_pose)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def grasp_object():
    rospy.loginfo("Nesne kavranıyor...")
    rospy.sleep(2)

def image_callback(msg):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        # Kırmızı nesneyi tespit et
        red_contour = detect_red_object(cv_image)
        if red_contour is not None:
            M = cv2.moments(red_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                x, y, z = compute_3d_coordinates(cx, cy)
                move_robot(x, y, z)
                grasp_object()

        # Yeşil nesneyi tespit et
        green_contour = detect_green_object(cv_image)
        if green_contour is not None:
            M = cv2.moments(green_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                x, y, z = compute_3d_coordinates(cx, cy)
                move_robot(x, y, z)
                grasp_object()

        # Mavi nesneyi tespit et
        blue_contour = detect_blue_object(cv_image)
        if blue_contour is not None:
            M = cv2.moments(blue_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                x, y, z = compute_3d_coordinates(cx, cy)
                move_robot(x, y, z)
                grasp_object()

    except Exception as e:
        rospy.logerr("Görüntü işlenirken hata: %s", e)

def main():
    rospy.init_node('color_object_detector')
    rospy.Subscriber("robot/camera1/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

