#!/usr/bin/env python3
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from geometry_msgs.msg import Pose

def move_to_pose(pose, group):
    # Hedef pozisyonu ayarla
    target_pose = Pose()
    target_pose.position.x = pose["x"]
    target_pose.position.y = pose["y"]
    target_pose.position.z = pose["z"]
    target_pose.orientation.x = pose["orientation"]["x"]
    target_pose.orientation.y = pose["orientation"]["y"]
    target_pose.orientation.z = pose["orientation"]["z"]
    target_pose.orientation.w = pose["orientation"]["w"]

    group.set_pose_target(target_pose)

    # Hareket planını yürüt
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def open():
    # Gripper'ı açma işlemleri burada yapılır
    rospy.loginfo("Gripper opened")

def close():
    # Gripper'ı kapatma işlemleri burada yapılır
    rospy.loginfo("Gripper closed")

def perform_pick_and_place(pick_pose, place_pose):
    # Panda arm grubunu oluştur
    group = MoveGroupCommander("panda_arm")

    # Alınacak nesneyi al
    move_to_pose(pick_pose, group)
    open()  # Gripper'ı aç

    rospy.sleep(2)  # Gripper'ın kapanmasını bekleyin (simülasyon amaçlı)

    # Yerleştirilecek konumu ayarla
    move_to_pose(place_pose, group)
    close()  # Gripper'ı kapat

    rospy.sleep(2)  # Gripper'ın açılmasını bekleyin (simülasyon amaçlı)

def move_to_ready_position():
    # Panda arm grubunu oluştur
    group = MoveGroupCommander("panda_arm")

    # Ready pozisyonuna git
    ready_pose = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.3,
        "orientation": {"x": 0.0, "y": 0.0, "z": -1.57, "w": 1.0}  # Yeni orientation eklenmiştir
    }

    move_to_pose(ready_pose, group)

def main():
    
    rospy.init_node('pick_and_place_demo')
    open()
    close()

    # İlk nesnenin alınacağı ve yerleştirileceği konumlar
    pick_pose_1 = {
        "x": 0.25,
        "y": -0.30,
        "z": 0.05,
        "orientation": {"x": 0.0, "y": 0.0, "z": -1.0, "w": 0.0}
    }

    place_pose_1 = {
        "x": -0.39,
        "y": -0.500002,
        "z": 0.125,
        "orientation": {"x": -0.5, "y": 0.5, "z": -0.5, "w": 0.5}
    }

    # İlk nesneyi al ve yerleştir
    perform_pick_and_place(pick_pose_1, place_pose_1)
    move_to_ready_position()
    
    pick_pose_2 = {
        "x": 0.25,
        "y": -0.021332,
        "z": 0.05,
        "orientation": {"x": 0.0, "y": 0.0, "z": -9.9e-05, "w": 0.0}
    }

    place_pose_2 = {
        "x": -0.399998,
        "y": 2e-06,
        "z": 0.125,
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }

    # İkinci nesneyi al ve yerleştir
    perform_pick_and_place(pick_pose_2, place_pose_2)

    # Ready pozisyonuna git
    move_to_ready_position()

    # İkinci nesnenin alınacağı ve yerleştirileceği konumlar
    pick_pose_3 = {
        "x": 0.25,
        "y": 0.3,
        "z": 0.05,
        "orientation": {"x": 0.0, "y": 0.0, "z": -9.9e-05, "w": 0.0}
    }

    place_pose_3 = {
        "x": -0.399998,
        "y": 0.499998,
        "z": 0.125,
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }

    # İkinci nesneyi al ve yerleştir
    perform_pick_and_place(pick_pose_3, place_pose_3)
    move_to_ready_position()

    # Ready pozisyonuna git

if __name__ == '__main__':
    main()
