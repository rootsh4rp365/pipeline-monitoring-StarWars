#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import json
import threading
import time
from collections import defaultdict

import rospy
from clover import srv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from utils import (
    ArUcoMarkerMap,
    TubeBranch,
    build_branch_message,
    detect_tubes_in_frame,
    find_nearest_tube,
)


class PipelineScannerNode:

    def __init__(self):
        rospy.init_node('pipeline_scanner', anonymous=False)

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', Trigger)

        self.flight_height = rospy.get_param('~flight_height', 1.0)
        self.scan_speed = rospy.get_param('~scan_speed', 0.5)
        self.navigation_threshold = rospy.get_param('~navigation_threshold', 0.2)

        self.grid_width = rospy.get_param('~grid_width', 10)
        self.grid_height = rospy.get_param('~grid_height', 10)
        self.grid_spacing = rospy.get_param('~grid_spacing', 1.0)
        self.aruco_map = ArUcoMarkerMap(self.grid_width, self.grid_height, self.grid_spacing)

        self.mission_running = False
        self.killed = False
        self.mission_paused = False
        self.mission_thread = None

        self.mission_control_sub = rospy.Subscriber(
            '/mission_control',
            String,
            self.mission_control_callback,
            queue_size=10,
        )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.image_sub = rospy.Subscriber(
            '/main_camera/image_raw',
            Image,
            self.image_callback,
        )

        self.tubes = []
        self.tube_centers = defaultdict(list)
        self.detection_threshold = rospy.get_param('~detection_threshold', 75)
        self.uniqueness_radius = rospy.get_param('~uniqueness_radius', 0.8)
        self.branch_pub = rospy.Publisher('/tubes', String, queue_size=10)

        rospy.loginfo('Waiting to start...')

    def image_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        with self.frame_lock:
            self.latest_frame = frame

    def mission_control_callback(self, msg):
        cmd = json.loads(msg.data)
        command = cmd.get('command', '')

        if command == 'start':
            self.start_mission()
        elif command == 'stop':
            self.stop_mission()
        elif command == 'kill':
            self.kill_mission()
        elif command == 'pause':
            self.pause_mission()
        elif command == 'resume':
            self.resume_mission()

    def start_mission(self):
        if self.mission_running:
            rospy.logwarn('Mission already running')
            return

        self.mission_running = True
        self.mission_paused = False
        self.tubes = []
        self.tube_centers = defaultdict(list)

        self.mission_thread = threading.Thread(target=self.main, daemon=True)
        self.mission_thread.start()

        rospy.loginfo('Mission started')

    def stop_mission(self):
        if not self.mission_running:
            return

        self.mission_running = False
        rospy.loginfo('Stopping mission, Returning to start...')

    def kill_mission(self):
        self.mission_running = False
        self.killed = True
        rospy.logerr('KILL SWITCH ACTIVATED - DISARMING DRONE')

        self.land()
        rospy.sleep(2.0)

    def pause_mission(self):
        self.mission_paused = True
        rospy.loginfo('Mission paused')

    def resume_mission(self):
        self.mission_paused = False
        rospy.loginfo('Mission resumed')

    def get_current_position(self, frame_id='aruco_map'):
        try:
            telemetry = self.get_telemetry(frame_id=frame_id)
            x, y, z = telemetry.x, telemetry.y, telemetry.z
            return (x, y, z)
        except rospy.ServiceException as e:
            rospy.logwarn('Error getting telemetry: %s', str(e))
            return None

    def wait_for_position(self, target_x, target_y, target_z, strict=True, timeout=60):
        start_time = time.time()

        while time.time() - start_time < timeout:
            pos = self.get_current_position()
            if pos is None:
                rospy.sleep(0.1)
                continue

            if not self.mission_running and (target_x != 0.0 or target_y != 0.0):
                return True

            while self.mission_paused and self.mission_running:
                rospy.sleep(0.5)

            dx = abs(pos[0] - target_x) if strict else 0
            dy = abs(pos[1] - target_y) if strict else 0
            dz = abs(pos[2] - target_z)

            if (
                dx < self.navigation_threshold
                and dy < self.navigation_threshold
                and dz < self.navigation_threshold
            ):
                if strict:
                    rospy.loginfo(
                        'Position reached: (%.2f, %.2f, %.2f)', target_x, target_y, target_z
                    )
                return True

            rospy.sleep(0.2)

        rospy.logwarn('Failed to reach position after %d seconds', timeout)
        return False

    def navigate_to_point(self, x, y, z, frame_id='aruco_map', auto_arm=False, strict=True):
        self.navigate(
            x=x,
            y=y,
            z=z,
            frame_id=frame_id,
            speed=self.scan_speed,
            auto_arm=auto_arm,
        )

        return self.wait_for_position(x, y, z, strict=strict)

    def detect_tubes_in_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return []
            frame = self.latest_frame.copy()

        return detect_tubes_in_frame(frame, self.aruco_map, self.detection_threshold)

    def process_branch_detection(self, pixel_coords, frame):
        ref_markers = self.aruco_map.get_reference_markers(pixel_coords, frame.shape)
        if ref_markers is None:
            rospy.logwarn('Not enough reference markers for triangulation')
            return None

        world_pos = self.aruco_map.triangulate_position(pixel_coords, ref_markers)
        if world_pos is None:
            rospy.logwarn('Triangulation failed')
            return None

        new_x, new_y, new_z = world_pos

        is_unique, nearest_tube, _ = find_nearest_tube(
            self.tubes,
            new_x,
            new_y,
            self.uniqueness_radius,
        )

        if not is_unique and nearest_tube is not None:
            old_x = nearest_tube.x
            old_y = nearest_tube.y
            old_z = nearest_tube.z

            nearest_tube.update_position(new_x, new_y, new_z)

            rospy.loginfo(
                'Updated branch %d: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f), confidence=%.1f',
                nearest_tube.id,
                old_x,
                old_y,
                old_z,
                nearest_tube.x,
                nearest_tube.y,
                nearest_tube.z,
                nearest_tube.confidence,
            )

            msg = build_branch_message(
                timestamp=rospy.get_time(),
                frame_id='aruco_map',
                branch=nearest_tube,
                status='updated',
            )
            self.branch_pub.publish(msg)

            return nearest_tube

        tube_id = len(self.tubes)
        tube = TubeBranch(tube_id, new_x, new_y, new_z, confidence=1.0)
        self.tubes.append(tube)

        msg = build_branch_message(
            timestamp=rospy.get_time(),
            frame_id='aruco_map',
            branch=tube,
            status='detected',
        )
        self.branch_pub.publish(msg)

        rospy.loginfo(
            'Branch %d detected: (%.2f, %.2f, %.2f), confidence=1',
            tube_id,
            new_x,
            new_y,
            new_z,
        )

        return tube

    def main(self):
        start_position = self.get_current_position(frame_id='map')
        rospy.loginfo('Taking off to %.2fm', self.flight_height)
        if not self.navigate_to_point(
            start_position[0],
            start_position[1],
            self.flight_height,
            frame_id='map',
            auto_arm=True,
            strict=False,
        ):
            rospy.logerr('Takeoff failed')
            return False

        rospy.sleep(3.0)

        scanned_points = 0
        total_points = 15

        for i in range(0, self.grid_width, 2):
            if not self.mission_running:
                break

            for j in range(0, self.grid_height, 4):
                if not self.mission_running:
                    break

                while self.mission_paused and self.mission_running:
                    rospy.sleep(0.5)

                scan_x = i * self.grid_spacing
                scan_y = j * self.grid_spacing

                if not self.navigate_to_point(scan_x, scan_y, self.flight_height):
                    rospy.logwarn('Failed to reach scan point')
                    continue

                rospy.sleep(5.0)

                tubes = self.detect_tubes_in_frame()

                for tube in tubes:
                    self.process_branch_detection(
                        tube['center'],
                        self.latest_frame,
                    )

                    rospy.sleep(0.5)

                scanned_points += 1
                rospy.loginfo('Progress: %d/%d', scanned_points, total_points)

        if not self.killed:
            rospy.loginfo('Returning to start position')
            if not self.navigate_to_point(
                0.0,
                0.0,
                self.flight_height,
            ):
                rospy.logwarn('Failed to return to start')

            rospy.sleep(2.0)
            rospy.loginfo('Landing...')
            self.land()
            rospy.sleep(2.0)

            rospy.loginfo('Results:')
            rospy.loginfo('Total branches found: %d', len(self.tubes))
            rospy.loginfo('Branch coordinates (aruco_map):')
            for i, tube in enumerate(self.tubes):
                rospy.loginfo(
                    '  Branch %d: x=%.2fm, y=%.2fm, z=%.2fm (confidence=%d)',
                    i,
                    tube.x,
                    tube.y,
                    tube.z,
                    tube.confidence,
                )

        return True

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        scanner = PipelineScannerNode()
        scanner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Node shutdown')
