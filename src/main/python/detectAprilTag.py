#!/usr/bin/env python3

from cscore import CameraServer, VideoMode
from ntcore import NetworkTableInstance, EventFlags

import cv2
import json
import numpy as np
import time

import robotpy_apriltag
from wpimath.geometry import Transform3d
import math

team = 3373
server = False

def main():
    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera = config['cameras'][0]

    width = camera['width']
    height = camera['height']
    print("w, h from file = (%s,%s)" % (width,height))

    video = CameraServer.startAutomaticCapture()
    video.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, 30)
    # video.setResolution(width, height)

    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Processed', width, height)
    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    # Table for vision output information
    vision_nt = ntinst.getTable('Vision')

    # Wait for NetworkTables to start
    time.sleep(0.5)

    prev_time = time.time()
    while True:
        start_time = time.time()

        frame_time, input_img = input_stream.grabFrame(img)
        # print("type = %s" % type(input_img))
        # print("dir = %s" % dir(input_img))
        output_img = np.copy(input_img)

        # Coordinates of found targets, for NT output:
        img_y_list = []
        img_z_list = []
        pose_tx_list = []
        pose_ty_list = []
        pose_tz_list = []
        pose_rx_list = []
        pose_ry_list = []
        pose_rz_list = []
        id_list = []

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        # April Tag detection:
        detector = robotpy_apriltag.AprilTagDetector()
        detector.addFamily("tag16h5")
        #detector.addFamily("tag36h11")
        estimator = robotpy_apriltag.AprilTagPoseEstimator(
            robotpy_apriltag.AprilTagPoseEstimator.Config(
                0.2, 500, 500, width / 2.0, height / 2.0
            )
        )

        # Detect apriltag
        DETECTION_MARGIN_THRESHOLD = 100
        DETECTION_ITERATIONS = 50

        gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
        tag_info = detector.detect(gray)
        filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]

        # OPTIONAL: Ignore any tags not in the set used on the 2023 FRC field:
        # filter_tags = [tag for tag in filter_tags if ((tag.getId() > 0) & (tag.getId() < 9))]

        for tag in filter_tags:

            est = estimator.estimateOrthogonalIteration(tag, DETECTION_ITERATIONS)
            pose = est.pose1

            tag_id = tag.getId()
            center = tag.getCenter()
            #hamming = tag.getHamming()
            #decision_margin = tag.getDecisionMargin()

            # What calculations do we want to make on the Pi?
            # Or just use Rotation Matrix to direct robot?
            # 
            # Have robot rotate to Z to 0.
            # 



            print(f"{tag_id}: {pose}")

            # Highlight the edges of all recognized tags and label them with their IDs:

            if ((tag_id > 0) & (tag_id < 9)):
                col_box = (0,255,0)
                col_txt = (255,255,255)
            else:
                col_box = (0,0,255)
                col_txt = (0,255,255)

            # Draw a frame around the tag:
            corner0 = (int(tag.getCorner(0).x), int(tag.getCorner(0).y))
            corner1 = (int(tag.getCorner(1).x), int(tag.getCorner(1).y))
            corner2 = (int(tag.getCorner(2).x), int(tag.getCorner(2).y))
            corner3 = (int(tag.getCorner(3).x), int(tag.getCorner(3).y))
            cv2.line(output_img, corner0, corner1, color = col_box, thickness = 2)
            cv2.line(output_img, corner1, corner2, color = col_box, thickness = 2)
            cv2.line(output_img, corner2, corner3, color = col_box, thickness = 2)
            cv2.line(output_img, corner3, corner0, color = col_box, thickness = 2)

            # Label the tag with the ID:
            cv2.putText(output_img, f"{tag_id}", (int(center.x), int(center.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, col_txt, 2)

            img_y_list.append(-(center.x - width / 2) / (width / 2))
            img_z_list.append(-(center.y - height / 2) / (height / 2))
            
            # Convert Vision system coords to Robot-centric coords
            # Vision X is Robot -Y
            # Vision Y is Robot -Z
            # Vision Z is Robot X
            pose_tx_list.append(pose.translation().z_feet)
            pose_ty_list.append(-pose.translation().x_feet)
            pose_tz_list.append(-pose.translation().y_feet)
            pose_rx_list.append(pose.rotation().z_degrees)
            pose_ry_list.append(-pose.rotation().x_degrees)
            pose_rz_list.append(-pose.rotation().y_degrees)


            # Vision system coords (original code)
            # pose_tx_list.append(pose.translation().x_feet)
            # pose_ty_list.append(pose.translation().y_feet)
            # pose_tz_list.append(pose.translation().z_feet)
            # pose_rx_list.append(pose.rotation().x_degrees)
            # pose_ry_list.append(pose.rotation().y_degrees)
            # pose_rz_list.append(pose.rotation().z_degrees)
            id_list.append(tag_id)


        vision_nt.putNumberArray('target_img_z', img_z_list)
        vision_nt.putNumberArray('target_img_y', img_y_list)
        vision_nt.putNumberArray('target_pose_tx', pose_tx_list)
        vision_nt.putNumberArray('target_pose_ty', pose_ty_list)
        vision_nt.putNumberArray('target_pose_tz', pose_tz_list)
        vision_nt.putNumberArray('target_pose_rx', pose_rx_list)
        vision_nt.putNumberArray('target_pose_ry', pose_ry_list)
        vision_nt.putNumberArray('target_pose_rz', pose_rz_list)
        
        # These sends the values in Vision system coords
        # vision_nt.putNumberArray('robot_pose_tx', robot_tx)
        # vision_nt.putNumberArray('robot_pose_ty', robot_ty)
        # vision_nt.putNumberArray('robot_pose_tz', robot_tz)
        # vision_nt.putNumberArray('robot_pose_rx', robot_rx)
        # vision_nt.putNumberArray('robot_pose_ry', robot_ry)
        # vision_nt.putNumberArray('robot_pose_rz', robot_rz)
        vision_nt.putNumberArray('robot_id', id_list)

        processing_time = start_time - prev_time
        prev_time = start_time

        fps = 1 / processing_time
        cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        
        # Resize output image for improved streaming bandwidth
        output_size = [320,240]
        output_img_small = cv2.resize(output_img, (output_size), interpolation=cv2.INTER_AREA)
        
        output_stream.putFrame(output_img_small)


main()
