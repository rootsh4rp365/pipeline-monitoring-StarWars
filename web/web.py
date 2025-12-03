#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from clover import srv
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import json
import threading
import base64
import os
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(script_dir, 'templates')
app = Flask(__name__, template_folder=template_dir)
app.config['SECRET_KEY'] = 'pipeline_monitoring_secret'
socketio = SocketIO(app, cors_allowed_origins="*", max_http_buffer_size=1e7, 
                   ping_timeout=60, ping_interval=25)

current_camera_frame = None
current_aruco_map_frame = None
current_aruco_debug_frame = None
current_telemetry = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
detected_branches = []
mission_state = 'idle'
branch_markers = []
frame_lock = threading.Lock()
telemetry_lock = threading.Lock()
branches_lock = threading.Lock()

last_camera_emit = 0
last_aruco_map_emit = 0
last_aruco_debug_emit = 0
last_aruco_map_broadcast = 0
CAMERA_EMIT_INTERVAL = 0.1
ARUCO_EMIT_INTERVAL = 0.1
ARUCO_MAP_BROADCAST_INTERVAL = 0.5

aruco_map_frame_cached = None
aruco_map_frame_lock = threading.Lock()
aruco_map_base_frame = None
aruco_map_base_lock = threading.Lock()

get_telemetry = None
navigate = None
land = None
arm = None
disarm = None

camera_sub = None
aruco_map_sub = None
aruco_debug_sub = None
tubes_sub = None
telemetry_sub = None
velocity_sub = None

mission_control_pub = None

bridge = CvBridge()


def init_ros():
    global get_telemetry, navigate, land, arm, disarm
    global camera_sub, aruco_map_sub, aruco_debug_sub, tubes_sub, telemetry_sub, velocity_sub
    global mission_control_pub
    
    rospy.init_node('pipeline_web_monitor', anonymous=True, disable_signals=True)
    
    try:
        get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    except:
        pass
    
    try:
        navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    except:
        pass
    
    try:
        land = rospy.ServiceProxy('land', Trigger)
    except:
        pass
    
    camera_sub = rospy.Subscriber('/main_camera/image_raw', Image, camera_callback, queue_size=1)
    aruco_map_sub = rospy.Subscriber('/aruco_map/image', Image, aruco_map_callback, queue_size=1)
    aruco_debug_sub = rospy.Subscriber('/aruco_map/debug', Image, aruco_debug_callback, queue_size=1)
    tubes_sub = rospy.Subscriber('/tubes', String, tubes_callback, queue_size=10)
    telemetry_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, telemetry_callback, queue_size=1)
    velocity_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, velocity_callback, queue_size=1)
    
    mission_control_pub = rospy.Publisher('/mission_control', String, queue_size=10)


def camera_callback(msg):
    global current_camera_frame, last_camera_emit
    
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    frame = cv2.resize(frame, (640, 480))
    
    with frame_lock:
        current_camera_frame = frame
    
    current_time = time.time()
    if current_time - last_camera_emit >= CAMERA_EMIT_INTERVAL:
        last_camera_emit = current_time
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
        frame_b64 = base64.b64encode(buffer).decode()
        socketio.emit('camera_frame', {'image': frame_b64})


def update_aruco_map_with_markers():
    global aruco_map_frame_cached, aruco_map_base_frame, branch_markers
    
    with aruco_map_base_lock:
        if aruco_map_base_frame is None:
            return
        frame_with_markers = aruco_map_base_frame.copy()
    
    with branches_lock:
        for marker in branch_markers:
            cx, cy = marker['pixel']
            if 0 <= cx < frame_with_markers.shape[1] and 0 <= cy < frame_with_markers.shape[0]:
                cv2.circle(frame_with_markers, (cx, cy), 10, (0, 0, 255), 2)
                cv2.circle(frame_with_markers, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(frame_with_markers, str(marker['id']), (cx + 15, cy - 15),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    _, buffer = cv2.imencode('.jpg', frame_with_markers, [cv2.IMWRITE_JPEG_QUALITY, 50])
    frame_b64 = base64.b64encode(buffer).decode()
    
    with aruco_map_frame_lock:
        aruco_map_frame_cached = frame_b64
    
    return frame_b64


def aruco_map_callback(msg):
    global aruco_map_base_frame, current_aruco_map_frame
    
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    frame = cv2.resize(frame, (480, 480))
    
    with aruco_map_base_lock:
        aruco_map_base_frame = frame.copy()
    
    with frame_lock:
        current_aruco_map_frame = frame
    
    update_aruco_map_with_markers()


def aruco_debug_callback(msg):
    global current_aruco_debug_frame, last_aruco_debug_emit
    
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    frame = cv2.resize(frame, (640, 480))
    
    with frame_lock:
        current_aruco_debug_frame = frame
    
    current_time = time.time()
    if current_time - last_aruco_debug_emit >= ARUCO_EMIT_INTERVAL:
        last_aruco_debug_emit = current_time
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
        frame_b64 = base64.b64encode(buffer).decode()
        socketio.emit('aruco_debug_frame', {'image': frame_b64})


def tubes_callback(msg):
    global detected_branches, branch_markers, last_aruco_map_broadcast
    
    data = json.loads(msg.data)
    
    with branches_lock:
        if data['status'] == 'detected':
            branch = data['branch']
            detected_branches.append({
                'id': branch['id'],
                'x': branch['x'],
                'y': branch['y'],
                'z': branch['z'],
                'confidence': branch['confidence'],
                'detections': branch['detections'],
                'status': 'detected'
            })
            branch_markers.append({
                'id': branch['id'],
                'x': branch['x'],
                'y': branch['y'],
                'pixel': pixel_from_world(branch['x'], branch['y'])
            })
            
        elif data['status'] == 'updated':
            branch = data['branch']
            for i, b in enumerate(detected_branches):
                if b['id'] == branch['id']:
                    detected_branches[i].update({
                        'x': branch['x'],
                        'y': branch['y'],
                        'z': branch['z'],
                        'confidence': branch['confidence'],
                        'detections': branch['detections'],
                        'status': 'updated'
                    })
                    for m in branch_markers:
                        if m['id'] == branch['id']:
                            m['x'] = branch['x']
                            m['y'] = branch['y']
                            m['pixel'] = pixel_from_world(branch['x'], branch['y'])
                            break
                    break
    
    frame_b64 = update_aruco_map_with_markers()
    
    current_time = time.time()
    if current_time - last_aruco_map_broadcast >= ARUCO_MAP_BROADCAST_INTERVAL:
        last_aruco_map_broadcast = current_time
        if frame_b64:
            socketio.emit('aruco_frame', {'image': frame_b64})
    
    socketio.emit('branches_update', {'branches': detected_branches})


def telemetry_callback(msg):
    global current_telemetry
    
    with telemetry_lock:
        current_telemetry['x'] = round(msg.pose.position.x, 3)
        current_telemetry['y'] = round(msg.pose.position.y, 3)
        current_telemetry['z'] = round(msg.pose.position.z, 3)
    
    socketio.emit('telemetry_update', current_telemetry)


def velocity_callback(msg):
    global current_telemetry
    
    with telemetry_lock:
        current_telemetry['vx'] = round(msg.twist.linear.x, 3)
        current_telemetry['vy'] = round(msg.twist.linear.y, 3)
        current_telemetry['vz'] = round(msg.twist.linear.z, 3)


def pixel_from_world(world_x, world_y):
    image_width = 480
    image_height = 480
    grid_size = 10

    pixel_x = int((world_x / grid_size) * image_width) + 45
    pixel_y = int(image_height - (world_y / grid_size) * image_height) - 45
    
    pixel_x = max(0, min(image_width - 1, pixel_x))
    pixel_y = max(0, min(image_height - 1, pixel_y))
    
    return (pixel_x, pixel_y)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    with telemetry_lock:
        telem = current_telemetry.copy()
    
    with branches_lock:
        branches = detected_branches.copy()
    
    return jsonify({
        'mission_state': mission_state,
        'telemetry': telem,
        'branches_count': len(branches),
        'branches': branches
    })


@socketio.on('connect')
def handle_connect():
    with telemetry_lock:
        telem = current_telemetry.copy()
    
    with branches_lock:
        branches = detected_branches.copy()
    
    emit('initial_status', {
        'mission_state': mission_state,
        'telemetry': telem,
        'branches': branches
    })
    
    with aruco_map_frame_lock:
        if aruco_map_frame_cached:
            emit('aruco_frame', {'image': aruco_map_frame_cached})


@socketio.on('mission_start')
def handle_mission_start():
    global mission_state
    
    if mission_state == 'running':
        mission_cmd = json.dumps({'command': 'pause'})
        mission_control_pub.publish(mission_cmd)
        mission_state = 'paused'
        socketio.emit('mission_status', {'state': mission_state})
    
    elif mission_state == 'paused':
        mission_cmd = json.dumps({'command': 'resume'})
        mission_control_pub.publish(mission_cmd)
        mission_state = 'running'
        socketio.emit('mission_status', {'state': mission_state})
    
    else:
        mission_cmd = json.dumps({'command': 'start'})
        mission_control_pub.publish(mission_cmd)
        mission_state = 'running'
        socketio.emit('mission_status', {'state': mission_state})


@socketio.on('mission_stop')
def handle_mission_stop():
    global mission_state
    
    mission_cmd = json.dumps({'command': 'stop'})
    mission_control_pub.publish(mission_cmd)
    
    mission_state = 'landed'
    socketio.emit('mission_status', {'state': mission_state})


@socketio.on('mission_kill')
def handle_mission_kill():
    global mission_state
    
    mission_cmd = json.dumps({'command': 'kill'})
    mission_control_pub.publish(mission_cmd)
    
    mission_state = 'idle'
    socketio.emit('mission_status', {'state': mission_state})

@socketio.on('get_telemetry')
def handle_get_telemetry():
    with telemetry_lock:
        telem = current_telemetry.copy()
    emit('telemetry_update', telem)


@socketio.on('get_branches')
def handle_get_branches():
    with branches_lock:
        branches = detected_branches.copy()
    emit('branches_update', {'branches': branches})


@socketio.on('request_aruco_map')
def handle_request_aruco_map():
    with aruco_map_frame_lock:
        if aruco_map_frame_cached:
            emit('aruco_frame', {'image': aruco_map_frame_cached})
        else:
            emit('error', {'message': 'ArUco map image not available yet'})


def ros_thread():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if get_telemetry:
            try:
                telem = get_telemetry(frame_id='aruco_map')
                with telemetry_lock:
                    current_telemetry['x'] = round(telem.x, 3)
                    current_telemetry['y'] = round(telem.y, 3)
                    current_telemetry['z'] = round(telem.z, 3)
                    current_telemetry['vx'] = round(telem.vx, 3)
                    current_telemetry['vy'] = round(telem.vy, 3)
                    current_telemetry['vz'] = round(telem.vz, 3)
            except:
                pass
        rate.sleep()


if __name__ == '__main__':
    init_ros()
    ros_thread_handle = threading.Thread(target=ros_thread, daemon=True)
    ros_thread_handle.start()
    socketio.run(app, host='0.0.0.0', port=5555, debug=True, allow_unsafe_werkzeug=True, use_reloader=False)
