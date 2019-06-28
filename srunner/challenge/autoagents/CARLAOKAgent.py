import scipy.misc

import carla
from agents.navigation.basic_agent import *

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track

import numpy as np

# import sys, os
# sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/yolov3")
# import importlib
# print(importlib.find_loader)
# from yolov3.YoloDetect import YoloDetect
# import yolov3

class CARLAOKAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.track = Track.ALL_SENSORS_HDMAP_WAYPOINTS

        self.route_assigned = False
        self._agent = None
        # print(yolov3.__file__)
        # print(yolov3.__dict__.keys())
        # self.yolo = YoloDetect()

    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}


        """

        # sensors = [
        #     {'type':'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        #      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
        # ]
        sensors = [
            {'type':'sensor.camera.rgb', 'x': 0, 'y': 0, 'z': 2.2, 'roll': 0.0, 'pitch': -10, 'yaw': 0.0,
             'width': 600, 'height': 300, 'fov': 70, 'id': 'Left'},
            {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
            {'type': 'sensor.can_bus', 'reading_frequency': 25, 'id': 'can_bus'},
            {'type': 'sensor.hd_map', 'reading_frequency': 1, 'id': 'hdmap'},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        # image = input_data['Left'][1] # ndarray of (h*w*4)
        # im = Image.fromarray(image)
        # im.save('aaa.jpg')
        # np.save('../image_npy/cam_img%06d'%(input_data['Left'][0]), image)
        # print("image saved ")
        
        # array = input_data['Left'][1]
        # array = np.ascontiguousarray(array[:, :, :3])
        # array = np.ascontiguousarray(array[:, :, ::-1])
        # detections = self.yolo.run(array, frame_id=input_data['Left'][0])

        
        if not self._agent:
            hero_actor = None
            for actor in CarlaDataProvider.get_world().get_actors():
                if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                    hero_actor = actor
                    break
            if hero_actor:
                # self._agent = BasicAgent(hero_actor)
                self._agent = LocalPlanner(hero_actor)

            return control

        if not self.route_assigned:
            if self._global_plan:
                plan = []

                for transform, road_option in self._global_plan_world_coord:
                    wp = CarlaDataProvider.get_map().get_waypoint(transform.location)
                    plan.append((wp, road_option))

                # self._agent._local_planner.set_global_plan(plan)
                self._agent.set_global_plan(plan)
                self.route_assigned = True

        else:
            print("[Timestamp: {}]".format(timestamp))
            control = self._agent.run_step(input_data)

        return control
