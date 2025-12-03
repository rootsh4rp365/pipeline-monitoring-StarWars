#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import argparse


class PipelineGenerator:
    def __init__(self, seed=None):
        self.main_start = (1.0, 1.0)
        self.main_min_length = 5.0
        self.main_max_length = 10.0
        self.main_diameter = 0.20
        self.max_bend_angle = 30.0
        
        self.branch_count = 5
        self.branch_max_length = 2.0
        self.branch_min_length = 0.5
        self.branch_diameter = 0.10
        self.min_branch_distance = 0.75
        
        self.main_segments = []
        self.branches = []
        self.aruco_markers = []
        self.bend_angle_deg = 0
        
    def _distance(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    
    def _interpolate_point(self, p1, p2, t):
        return (
            p1[0] + (p2[0] - p1[0]) * t,
            p1[1] + (p2[1] - p1[1]) * t
        )
    
    def generate_main_pipe(self):
        length = random.uniform(self.main_min_length, self.main_max_length)
        
        bend_angle_deg = random.uniform(-self.max_bend_angle, self.max_bend_angle)
        bend_angle_rad = math.radians(bend_angle_deg)
        
        x0, y0 = self.main_start
        segment1_length = length * 0.6
        
        initial_angle = random.uniform(math.radians(40), math.radians(50))
        
        x1 = x0 + segment1_length * math.cos(initial_angle)
        y1 = y0 + segment1_length * math.sin(initial_angle)
        
        segment2_length = length * 0.4
        final_angle = initial_angle + bend_angle_rad
        
        x2 = x1 + segment2_length * math.cos(final_angle)
        y2 = y1 + segment2_length * math.sin(final_angle)
        
        self.main_segments = [
            {
                'start': (x0, y0),
                'end': (x1, y1),
                'angle': initial_angle,
                'length': segment1_length
            },
            {
                'start': (x1, y1),
                'end': (x2, y2),
                'angle': final_angle,
                'length': segment2_length
            }
        ]
        
        self.bend_angle_deg = bend_angle_deg
        return self.main_segments
    
    def generate_branches(self):
        self.branches = []
        total_length = sum(seg['length'] for seg in self.main_segments)
        
        attempts = 0
        max_attempts = 10000
        
        while len(self.branches) < self.branch_count and attempts < max_attempts:
            attempts += 1
            
            distance_along_pipe = random.uniform(0.5, total_length - 0.5)
            current_distance = 0
            branch_point = None
            branch_angle = None
            
            for segment in self.main_segments:
                seg_length = segment['length']
                
                if current_distance + seg_length >= distance_along_pipe:
                    t = (distance_along_pipe - current_distance) / seg_length
                    branch_point = self._interpolate_point(
                        segment['start'],
                        segment['end'],
                        t
                    )
                    branch_angle = segment['angle'] + math.radians(random.choice([90, 270]))
                    break
                
                current_distance += seg_length
            
            if branch_point is None:
                continue
            
            too_close = False
            for existing_branch in self.branches:
                dist = self._distance(branch_point, existing_branch['point'])
                if dist < self.min_branch_distance:
                    too_close = True
                    break
            
            if too_close:
                continue
            
            branch_length = random.uniform(
                self.branch_min_length,
                self.branch_max_length
            )
            
            self.branches.append({
                'point': branch_point,
                'angle': branch_angle,
                'length': branch_length
            })
        
        if len(self.branches) < self.branch_count:
            print("!!!!!")
            return self.generate_branches()
        
        return self.branches
    
    def generate_aruco_map(self):
        marker_size = 0.44
        spacing = 1.0
        
        marker_id = 0
        for i in range(9, -1, -1):
            for j in range(10):
                self.aruco_markers.append({
                    'id': marker_id,
                    'size': marker_size,
                    'x': j * spacing,
                    'y': i * spacing,
                    'z': 0.0,
                    'yaw': 0,
                    'pitch': 0,
                    'roll': 0
                })
                marker_id += 1
        
        return self.aruco_markers
    
    def generate_world_sdf(self, output_path):
        self.generate_main_pipe()
        self.generate_branches()
        
        sdf_content = '''<?xml version="1.0"?>
<sdf version="1.5">
  <world name="pipeline_world">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://parquet_plane</uri>
      <pose>0 0 -0.01 0 0 0</pose>
    </include>

    <include>
      <uri>model://aruco_map_txt</uri>
    </include>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.8 0.9 1 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>
  
    <physics name='default_physics' default='0' type='ode'>
      <gravity>0 0 -9.8066</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
      <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>
    </physics>

'''
        
        for i, seg in enumerate(self.main_segments):
            cx = (seg['start'][0] + seg['end'][0]) / 2.0
            cy = (seg['start'][1] + seg['end'][1]) / 2.0
            cz = self.main_diameter / 2.0
            length = seg['length']
            angle = seg['angle']
            
            sdf_content += '''    <model name="main_pipe_{}">
      <pose>{} {} {} 0 1.5708 {}</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{}</radius>
              <length>{}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{}</radius>
              <length>{}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

'''.format(i, cx, cy, cz, angle, self.main_diameter/2, length, 
           self.main_diameter/2, length)
        
        for i, branch in enumerate(self.branches):
            cx = branch['point'][0] + (branch['length'] / 2) * math.cos(branch['angle'])
            cy = branch['point'][1] + (branch['length'] / 2) * math.sin(branch['angle'])
            cz = self.branch_diameter / 2.0
            
            sdf_content += '''    <model name="branch_{}">
      <pose>{} {} {} 0 1.5708 {}</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{}</radius>
              <length>{}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{}</radius>
              <length>{}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.9 0.2 0.2 1</diffuse>
            <specular>0.5 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

'''.format(i, cx, cy, cz, branch['angle'], self.branch_diameter/2, 
           branch['length'], self.branch_diameter/2, branch['length'])
        
        sdf_content += '''  </world>
</sdf>'''
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(sdf_content)
        
        print("World generated: {}".format(output_path))
        
        return output_path
    
    def save_aruco_map(self, output_path):
        self.generate_aruco_map()
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('# id size x y z yaw pitch roll\n')
            for marker in self.aruco_markers:
                f.write("{} {} {} {} {} {} {} {}\n".format(
                    marker['id'], marker['size'], marker['x'], marker['y'],
                    marker['z'], marker['yaw'], marker['pitch'], marker['roll']
                ))
        
        print("ArUco map saved: {}".format(output_path))
    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        type=str,
        default='../worlds/pipeline_world.world',
    )
    parser.add_argument(
        '--map',
        type=str,
        default='../map/map.txt',
    )
    
    args = parser.parse_args()
    
    try:
        generator = PipelineGenerator(seed=args.seed)
        generator.generate_world_sdf(args.output)
        generator.save_aruco_map(args.map)
        
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
