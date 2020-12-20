# -*- coding:utf:8 -*-

import time
from pygazebo_ros._gazeboo_ros import _GazeboROS

from typing import List, Optional


class GazeboROS(_GazeboROS):

    """Encapsulated ROS services, topics and parameters to work with Gazebo

    Args:
        is_core (bool, optional): set False if a rosocre is already running.
          Defaults to True.
        paused (bool, optional): start Gazebo in a paused state. Defaults to
          False.
        use_sim_time (bool, optional): tells ROS nodes asking for time to get
          the Gazebo-published simulation time, published over the ROS topic
          `/clock`. Defaults to True.
        extra_gazebo_args (str, optional): extra args. Defaults to ''.
        gui (bool, optional): launch the user interface window of Gazebo.
          Defaults to True.
        recording (bool, optional): (previously called headless) enable gazebo
          state log recording. Defaults to False.
        debug (bool, optional): start gzserver (Gazebo Server) in debug mode
          using gdb. Defaults to False.
        physics (str, optional): Specify a physics engine
          (ode|bullet|dart|simbody). Defaults to 'ode'.
        verbose (bool, optional): run gzserver and gzclient with --verbose,
          printing errors and warnings to the terminal. Defaults to False.
        output (str, optional): ('log'|'screen') if 'screen', stdout/stderr
          from the node will be sent to the screen ; if 'log', the
          stdout/stderr output will be sent to a log file in `$ROS_HOME/log`,
          and stderr will continue to be sent to screen. Defaults to 'screen'.
        world_name (str, optional): the world_name with respect to
          GAZEBO_RESOURCE_PATH environmental variable. Defaults to
          'worlds/empty.world'.
        respawn_gazebo (bool, optional): restart the Gazebo node automatically
          if it quits. Defaults to False.
        use_clock_frequency (bool, optional): whether you modify Gazebo's
          `/clock` frequency (default to 1000 Hz); if True, set new clock
          frequency with pub_clock_frequency arg. Defaults to False.
        pub_clock_frequency (int, optional): set Gazebo’s `/clock` publish
          frequency (Hz); requires use_clock_frequency to be true. Defaults to
          100.
        enable_ros_network (bool, optional): if False, disable all the Gazebo
          ROS topics (except `/clock`) and services that are created from the
          gazebo_ros package. Beware, by choosing False, you will no longer be
          able to use most of the methods in this class. Defaults to True.
        server_required (bool, optional): terminate launch script when gzserver
          (Gazebo Server) exits. Defaults to False.
        gui_required (bool, optional): terminate launch script when gzclient
          (user interface window) exits. Defaults to False.
    """

    def __init__(
            self, is_core: bool = True, paused: bool = False,
            use_sim_time: bool = True, extra_gazebo_args: str = '',
            gui: bool = True, recording: bool = False, debug: bool = False,
            physics: str = 'ode', verbose: bool = False,
            output: str = 'screen', world_name: str = 'worlds/empty.world',
            respawn_gazebo: bool = False, use_clock_frequency: bool = False,
            pub_clock_frequency: int = 100, enable_ros_network: bool = True,
            server_required: bool = False, gui_required: bool = False):
        """Constructor."""

        super().__init__(
            is_core, paused, use_sim_time, extra_gazebo_args, gui, recording,
            debug, physics, verbose, output, world_name, respawn_gazebo,
            use_clock_frequency, pub_clock_frequency, enable_ros_network,
            server_required, gui_required)

    # -- User method --
    # The following methods are intented to the user.
    # They must use the service methods and not the services directly.
    # They must be user friendly and provide all the features that a user
    #   can expect.
    # region
    def apply_body_force(
            self, model_name: str, link_name: str, force: List[float],
            reference_point: List[float] = [0, 0, 0], start_time: float = 0,
            duration: float = -1.0, reference_frame: str = '') -> None:
        """Apply force to a body.

        Args:
            model_name (str): the model containing the link to which to
              apply force.
            link_name (str): the link to which to apply the force.
            force (List[float]): force applied to the body
            reference_point (List[float], optional): wrench is defined at this
              location in the reference frame. Use [0, 0, 0] to apply at the
              center of mass. Default [0, 0, 0].
            start_time (float, optional): wrench application start time
              (seconds); if start_time < current time, start as soon as
              possible. Defaults to 0.0.
            duration (float, optionnal): apply force for a given duration
              (seconds); if duration < 0, apply force continuously.
              Defaults to -1.0.
            reference_frame (str, optional): wrench is defined in the reference
              frame of this entity; use inertial frame if left empty.
              Defaults to ''.
        """
        body_name = '{}::{}'.format(model_name, link_name)
        torque = [0, 0, 0]
        start_time_secs = int(start_time)
        start_time_nsecs = int((start_time-start_time_secs)*1e9)
        duration_secs = int(duration)
        duration_nsecs = int((duration-duration_secs)*1e9)
        reference_point = reference_point.copy()  # list is default arg: safer
        super().apply_body_wrench(
            body_name, reference_frame, reference_point, force, torque,
            start_time_secs, start_time_nsecs, duration_secs, duration_nsecs)

    def apply_body_torque(
            self, model_name: str, link_name: str, torque: List[float],
            start_time: float = 0, duration: float = -1.0,
            reference_frame: str = '') -> None:
        """Apply torque to Gazebo Body.

        Args:
            model_name (str): the model containing the link to which to
              apply torque.
            link_name (str): the link to which to apply the torque.
            torque (List[float]): torque applied to the body
            start_time (float, optional): wrench application start time
              (seconds); if start_time < current time, start as soon as
              possible. Defaults to 0.0.
            duration (float, optionnal): apply torque for a given duration
              (seconds); if duration < 0, apply torque continuously.
              Defaults to -1.0.
            reference_frame (str, optional): torque is defined in the reference
            frame of this entity; use inertial frame if left empty.
            Defaults to ''.
        """
        body_name = '{}::{}'.format(model_name, link_name)
        force = [0, 0, 0]
        # Why reference_point not in args ? Because changment of reference
        # point does not affect torque (see Varignon's theorem)
        reference_point = [0, 0, 0]
        start_time_secs = int(start_time)
        start_time_nsecs = int((start_time-start_time_secs)*1e9)
        duration_secs = int(duration)
        duration_nsecs = int((duration-duration_secs)*1e9)
        super().apply_body_wrench(
            body_name, reference_frame, reference_point, force, torque,
            start_time_secs, start_time_nsecs, duration_secs, duration_nsecs)

    def clear_body_wrenches(self, model_name: str, link_name: str) -> None:
        """Clear body wrenches.

        Args:
            model_name (str): the model containing the link from which wrenches
              are cleared.
              clear wrenches.
            link_name (str): link from which wrenches are cleared.
        """
        body_name = '{}::{}'.format(model_name, link_name)
        super().clear_body_wrenches(body_name=body_name)

    def get_sim_time(self) -> float:
        """Get the current simulation time.

        Returns:
            float: Current simulation time (seconds).
        """
        ans = super().get_world_properties()
        return ans['sim_time']

    def spawn_light(
            self, light_name: str, position: List[float], yaw: float = 0,
            pitch: float = 0, roll: float = -1.0, diffuse_red: float = 0.5,
            diffuse_green: float = 0.5, diffuse_blue: float = 0.5,
            diffuse_alpha: float = 1.0, specular_red: float = 0.1,
            specular_green: float = 0.1, specular_blue: float = 0.1,
            specular_alpha: float = 1.0, attenuation_range: float = 20,
            attenuation_constant: float = 0.5, attenuation_linear: float = 0.1,
            attenuation_quadratic: float = 0.0, cast_shadows: bool = False,
            reference_frame: str = ''):
        """Spawn a light.

        Args:
            light_name (str): light name
            position (List): position [x, y, z]
            yaw (float, optional): yaw. Defaults to 0.
            pitch (float, optional): pitch. Defaults to 0.
            roll (float, optional): roll. Defaults to -1.0.
            diffuse_red (float, optional): red beam rate. Defaults to 0.5.
            diffuse_green (float, optional): green beam rate. Defaults to 0.5.
            diffuse_blue (float, optional): blue beam rate. Defaults to 0.5.
            diffuse_alpha (float, optional): opacity rate. Defaults to 1.0.
            specular_red (float, optional): specular red beam rate.
              Defaults to 0.1.
            specular_green (float, optional): specular green beam rate.
              Defaults to 0.1.
            specular_blue (float, optional): specular blue beam rate.
              Defaults to 0.1.
            specular_alpha (float, optional): specular opacity rate.
              Defaults to 1.0.
            attenuation_range (float, optional): light range. Defaults to 20.
            attenuation_constant (float, optional): constant attenuation.
              Defaults to 0.5.
            attenuation_linear (float, optional): linear attenuation.
              Defaults to 0.1.
            attenuation_constant (float, optional): quadratic attenuation.
              Defaults to 0.0.
            cast_shadows (bool, optional): cast shadows. Defaults to False.
        """
        template = '''<?xml version="1.0" ?>
            <sdf version="1.5">
            <light type="point" name='unused_name'>
                <pose>{x} {y} {z} {yaw} {pitch} {roll}</pose>
                <diffuse>{dred} {dgreen} {dblue} {dalpha}</diffuse>
                <specular>{sred} {sgreen} {sblue} {salpha}</specular>
                <attenuation>
                    <range>{attenuation_range}</range>
                    <constant>{attenuation_constant}</constant>
                    <linear>{attenuation_linear}</linear>
                    <quadratic>{attenuation_quadratic}</quadratic>
                </attenuation>
                <cast_shadows>{cast_shadows}</cast_shadows>
            </light>
            </sdf>'''

        model_xml = template.format(
            x=position[0], y=position[1], z=position[2],
            yaw=yaw, pitch=pitch, roll=roll,
            dred=diffuse_red, dgreen=diffuse_green,
            dblue=diffuse_blue, dalpha=diffuse_alpha,
            sred=specular_red, sgreen=specular_green,
            sblue=specular_blue, salpha=specular_alpha,
            attenuation_range=attenuation_range,
            attenuation_constant=attenuation_constant,
            attenuation_linear=attenuation_linear,
            attenuation_quadratic=attenuation_quadratic,
            cast_shadows=str(cast_shadows).lower())

        super().spawn_sdf_model(
            model_name=light_name,
            model_xml=model_xml,
            robot_namespace='',
            initial_position=[0.0, 0.0, 0.0],
            initial_orientation=[0.0, 0.0, 0.0, 0.0],
            reference_frame=reference_frame)

    # endregion


if __name__ == "__main__":
    gr = GazeboROS()
    model_xml = '''<?xml version="1.0" ?>
            <sdf version="1.5">
                <model name="model_test_1">
                    <self_collide>false</self_collide>
                    <pose>0 0 0.5 0 0 0</pose>
                    <static>false</static>
                    <link name="link_1">
                        <pose>0 0 0 0 0 0</pose>
                        <inertial>
                            <inertia>
                                <ixx>1</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>1</iyy>
                                <iyz>0</iyz>
                                <izz>1</izz>
                            </inertia>
                            <mass>1.0</mass>
                        </inertial>

                        <collision name="collision">
                            <pose>0 0 0.5 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>1 1 1</size>
                                </box>
                            </geometry>
                        </collision>

                        <visual name="visual">
                            <pose>0 0 0.5 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>1 1 1</size>
                                </box>
                            </geometry>
                            <material>
                                <diffuse>1.0 0.3 0.5 1.0</diffuse>
                                <specular>.1 .1 .1 1.0</specular>
                            </material>
                        </visual>
                    </link>
                    <link name="link_2">
                        <pose>0 0 2 0 0 0</pose>
                        <inertial>
                            <inertia>
                                <ixx>1</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>1</iyy>
                                <iyz>0</iyz>
                                <izz>1</izz>
                            </inertia>
                            <mass>1.0</mass>
                        </inertial>

                        <collision name="collision">
                            <pose>0 0 0.25 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.5 0.5 0.5</size>
                                </box>
                            </geometry>
                        </collision>

                        <visual name="visual">
                            <pose>0 0 0.25 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.5 0.5 0.5</size>
                                </box>
                            </geometry>
                            <material>
                                <diffuse>0.0 0.9 0.1 1.0</diffuse>
                                <specular>.1 .1 .1 1.0</specular>
                            </material>
                        </visual>
                    </link>
                    <joint name="joint" type="revolute2">
                        <parent>link_1</parent>
                        <child>link_2</child>
                        <pose>0 0 0 0 0 0</pose>
                        <axis>
                            <xyz>0 1 0</xyz>
                        </axis>
                        <axis2>
                            <xyz>0 0 1</xyz>
                        </axis2>
                    </joint>
                </model>
            </sdf>'''
    time.sleep(0.5)
    gr.spawn_sdf_model(
        model_name='test_model_sdf',
        model_xml=model_xml,
        robot_namespace='test_robot_namespace_sdf',
        initial_position=[1.0, 1.0, 3.0],
        initial_orientation=[0.0, 0.0, 0.0, 0.1],
        reference_frame='')

    time.sleep(1)

    gr.apply_joint_effort(
        joint_name='test_model_sdf::joint',
        effort=0.5,
        start_time_secs=1,
        start_time_nsecs=1,
        duration_secs=-1,
        duration_nsecs=0)

    time.sleep(2)


# (continuous : pivot infini) a hinge joint that rotates on a single
# axis with a continuous range of motion,
# (revolute : pivot fini) a hinge joint that rotates on a single axis with
# a fixed range of motion,
# (gearbox : reducteur) geared revolute joints,
# (revolute2 : rotule à doigt) same as two revolute joints connected in series,
# (prismatic : glissière) a sliding joint that slides along an axis with a
# limited range specified by upper and lower limits,
# (ball : rotule) a ball and socket joint,
# (screw : helicoidale) a single degree of freedom joint with coupled sliding
# and rotational motion,
# (universal, rotule à doigt) like a ball joint, but constrains one degree of
# freedom,
# (fixed, encastrement) a joint with zero degrees of freedom that rigidly
# connects two links.
