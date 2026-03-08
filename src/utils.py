#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

import cv2
import numpy as np
import rospy
from std_msgs.msg import String


class TubeBranch:
    def __init__(self, tube_id, x, y, z, confidence=0.0, detections=1):
        self.id = tube_id
        self.x = x
        self.y = y
        self.z = z
        self.confidence = confidence
        self.detections = detections

    def to_dict(self):
        return {
            'id': self.id,
            'x': float(self.x),
            'y': float(self.y),
            'z': float(self.z),
            'confidence': float(self.confidence),
            'detections': int(self.detections),
        }

    def update_position(self, new_x, new_y, new_z):
        self.x = (self.x + new_x) / 2.0
        self.y = (self.y + new_y) / 2.0
        self.z = (self.z + new_z) / 2.0
        self.confidence += 1.0
        self.detections += 1


class ArUcoMarkerMap:
    def __init__(self, grid_width, grid_height, grid_spacing):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.grid_spacing = grid_spacing

        self.marker_positions = {}
        marker_id = 0
        for row in range(self.grid_height):
            for col in range(self.grid_width):
                x = col * self.grid_spacing
                y = (self.grid_height - 1 - row) * self.grid_spacing
                self.marker_positions[marker_id] = (x, y)
                marker_id += 1

        self.detected_markers = {}

    def detect_markers(self, frame):
        self.detected_markers = {}

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        detector_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, dictionary, parameters=detector_params
        )

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                cx = np.mean(corner[:, 0])
                cy = np.mean(corner[:, 1])
                self.detected_markers[int(marker_id)] = (cx, cy, corner)

        return self.detected_markers

    def get_reference_markers(self, branch_pixel_pos, frame_shape):
        del frame_shape
        if len(self.detected_markers) < 3:
            return None

        branch_x, branch_y = branch_pixel_pos
        distances = []
        for marker_id, (px, py, corner) in self.detected_markers.items():
            dist = np.sqrt((px - branch_x) ** 2 + (py - branch_y) ** 2)
            distances.append((dist, marker_id, px, py))

        distances.sort()
        return distances[:3]

    def triangulate_position(self, branch_pixel_pos, ref_markers):
        if not ref_markers or len(ref_markers) < 3:
            return None

        try:
            markers_data = []
            for _, marker_id, px, py in ref_markers:
                aruco_x, aruco_y = self.marker_positions[marker_id]
                markers_data.append({
                    'id': marker_id,
                    'pixel': (px, py),
                    'world': (aruco_x, aruco_y),
                })

            src_points = np.float32([m['pixel'] for m in markers_data])
            dst_points = np.float32([m['world'] for m in markers_data])
            matrix = cv2.getAffineTransform(src_points, dst_points)

            branch_point = np.float32([list(branch_pixel_pos)]).reshape(-1, 1, 2)
            transformed = cv2.transform(branch_point, matrix)
            world_x = float(transformed[0][0][0])
            world_y = float(transformed[0][0][1])
            world_z = 0.05

            if world_x > 1 and world_y > 1:
                return (world_x, world_y, world_z)
            raise Exception('Wrong calculations')
        except Exception as e:
            rospy.logwarn('Triangulation error: %s', str(e))
            return None


def detect_tubes_in_frame(frame, aruco_map, detection_threshold):
    aruco_map.detect_markers(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 78, 67])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected_tubes = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < detection_threshold:
            continue

        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            continue

        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        detected_tubes.append({
            'center': (cx, cy),
            'area': area,
            'contour': contour,
        })

    return detected_tubes


def find_nearest_tube(tubes, new_x, new_y, uniqueness_radius):
    nearest_tube = None
    min_distance = float('inf')
    is_unique = True

    for existing_tube in tubes:
        dist = np.sqrt((new_x - existing_tube.x) ** 2 + (new_y - existing_tube.y) ** 2)
        if dist < min_distance:
            min_distance = dist
            nearest_tube = existing_tube

        if dist < uniqueness_radius:
            is_unique = False
            break

    return is_unique, nearest_tube, min_distance


def build_branch_message(timestamp, frame_id, branch, status):
    msg = String()
    msg.data = json.dumps({
        'timestamp': timestamp,
        'frame_id': frame_id,
        'branch': branch.to_dict(),
        'status': status,
    })
    return msg
