import unittest
import pygazebo_ros

import os
import time

test_path = os.path.dirname(os.path.abspath(__file__))
world_path = os.path.join(test_path, 'launch/world_test.world')
gazebo_ros = pygazebo_ros.GazeboROS(world_name=world_path, is_core=False)


class TestGazeboROS(unittest.TestCase):
    def test_apply_body_force(self):
        gazebo_ros.apply_body_force(
            model_name='model_test_4',
            link_name= 'link',
            reference_point=[0, 0, 0],
            force=[0, 0.1, 0.1],
            start_time=1.000001,
            reference_frame='model_test_4::link')

    def test_apply_body_torque(self):
        gazebo_ros.apply_body_torque(
            model_name='model_test_4',
            link_name='link',
            torque=[0.1, 0.2, 0.3],
            start_time=2.000001,
            reference_frame='model_test_4::link')
        time.sleep(5)
    
    def test_clear_body_wrenches(self):
        gazebo_ros.clear_body_wrenches(
            model_name='model_test_4',
            link_name='link')

    def test_get_sim_time(self):
        self.assertIsInstance(gazebo_ros.get_sim_time(), float)

    def test_spawn_light(self):
        gazebo_ros.spawn_light(
            light_name='testing_light',
            position=[1.0, 2.0, 3.0],
            yaw=0.1, pitch=0.2, roll=-0.9,
            diffuse_red=0.9, diffuse_green=0.8,
            diffuse_blue=0.7, diffuse_alpha=0.6,
            specular_red=0.5, specular_green=0.4,
            specular_blue=0.3, specular_alpha=0.2,
            attenuation_range=10,
            attenuation_constant=0.55,
            attenuation_linear=0.4,
            attenuation_quadratic=0.3,
            cast_shadows=True)
        attenuation_constant = gazebo_ros.get_light_properties(
            light_name='testing_light')['attenuation_constant']
        self.assertAlmostEqual(attenuation_constant, 0.55)
