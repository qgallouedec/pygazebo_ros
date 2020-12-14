# -*- coding:utf-8 -*-

from contextlib import contextmanager
import time
import os

import rospy
import roslaunch
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from dynamic_reconfigure.msg import *

from typing import List, Dict


def subscribe(name, data_class, callback, timeout=None):
    """TODO:"""
    try:
        rospy.logdebug("Waiting for message on topic %s" % name)
        if timeout != 0:
            rospy.wait_for_message(name, data_class, timeout)
    except rospy.ROSInterruptException as e:
        rospy.logdebug(
            "Waiting for topic %s interrupted" % name)
        raise e
    except rospy.ROSException as e:
        rospy.logerr(
            "Timeout exceded, no message received on topic %s" % name)
        raise e
    else:
        rospy.logdebug("Subscriber to %s ready" % (name))
        return rospy.Subscriber(name, data_class, callback)


def publisher(name, data_class, timeout=None):
    """TODO:"""
    start = time.time()
    pub = rospy.Publisher(name, data_class, queue_size=1)
    rate = rospy.Rate(5)  # 2hz #FIXME: if paused, time is paused
    rospy.logdebug("Looking for subscriber for topic %s" % name)
    while pub.get_num_connections() == 0:
        if timeout is not None and time.time() - start > timeout:
            rospy.logerr(
                "Timeout execeded, no subscriber found for topic %s" % name)
            raise rospy.ROSException()
        else:
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                # To avoid error when world is rested, time when backwards.
                rospy.logdebug("Time moved backward, ignoring")
            except rospy.ROSInterruptException as e:
                rospy.logdebug(
                    "Waiting for subscriber for topic %s interrupted" % name)
                raise e

    rospy.logdebug("Subscriber found for topic %s, publisher connected" % name)
    return pub


def service(name, service_class, timeout=None):
    """TODO:"""
    while not rospy.is_shutdown():
        try:
            rospy.logdebug("Waiting for service %s" % name)
            rospy.wait_for_service(name, timeout)
            srv = rospy.ServiceProxy(name, service_class)
        except rospy.ROSInterruptException as e:
            rospy.logdebug(
                "Waiting for service %s interrupted" % name)
            raise e
        except rospy.ROSException as e:
            rospy.logerr(
                "Timeout exceded, no service %s found" % name)
            raise e
        else:
            rospy.logdebug("Service %s ready" % name)
            return srv


class GazeboROS(object):
    """The class that encapsulated services and parameters to work with Gazebo
    """

    _JOINT_TYPE = [
        'REVOLUTE',   # 1 DOF
        'CONTINUOUS',  # 1 DOF (revolute w/o joints)
        'PRISMATIC',  # single DOF
        'FIXED',      # 0 DOF
        'BALL',       # 3 DOF
        'UNIVERSAL'   # 2 DOF
    ]

    def __init__(self):
        self.link_states = {}
        self.model_states = {}

        # TODO: init note if not done
        # rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(
            options_runid=None, options_wait_for_master=False)
        roslaunch.configure_logging(uuid)
        # gazebo_ros_path = rospack.get_path('gazebo_ros')
        test_path = os.path.dirname(os.path.abspath(__file__))
        empty_world_path = os.path.join(test_path, 'launch/test_launch.launch')
        launch = roslaunch.parent.ROSLaunchParent(
            uuid, [empty_world_path], is_core=True, is_rostest=True)
        launch.start()
        time.sleep(2)  # wait for the core to be ready
        rospy.init_node('test_node', anonymous=True, log_level=rospy.ERROR)

        self._connect_services()
        self._connect_topics()

    def _connect_services(self, timeout=None):
        # /gazebo/apply_body_wrench
        self._apply_body_wrench_srv = service(
            '/gazebo/apply_body_wrench', ApplyBodyWrench, timeout)
        # /gazebo/apply_joint_effort
        self._apply_joint_effort_srv = service(
            '/gazebo/apply_joint_effort', ApplyJointEffort, timeout)
        # /gazebo/clear_body_wrenches
        self._clear_body_wrenches_srv = service(
            '/gazebo/clear_body_wrenches', BodyRequest, timeout)
        # /gazebo/clear_joint_forces
        self._clear_joint_forces_srv = service(
            '/gazebo/clear_joint_forces', JointRequest, timeout)
        # /gazebo/delete_light
        self._delete_light_srv = service(
            '/gazebo/delete_light', DeleteLight, timeout)
        # /gazebo/delete_model
        self._delete_model_srv = service(
            '/gazebo/delete_model', DeleteModel, timeout)
        # /gazebo/get_joint_properties
        self._get_joint_properties_srv = service(
            '/gazebo/get_joint_properties', GetJointProperties, timeout)
        # /gazebo/get_light_properties
        self._get_light_properties_srv = service(
            '/gazebo/get_light_properties', GetLightProperties, timeout)
        # /gazebo/get_link_properties
        self._get_link_properties_srv = service(
            '/gazebo/get_link_properties', GetLinkProperties, timeout)
        # /gazebo/get_link_state
        self._get_link_state_srv = service(
            '/gazebo/get_link_state', GetLinkState, timeout)
        # /gazebo/get_loggers
        self._get_loggers_srv = service(
            '/gazebo/get_loggers', Empty, timeout)
        # /gazebo/get_model_properties
        self._get_model_properties_srv = service(
            '/gazebo/get_model_properties', GetModelProperties, timeout)
        # /gazebo/get_model_state
        self._get_model_state_srv = service(
            '/gazebo/get_model_state', GetModelState, timeout)
        # /gazebo/get_physics_properties
        self._get_physics_properties_srv = service(
            '/gazebo/get_physics_properties', GetPhysicsProperties, timeout)
        # /gazebo/get_world_properties
        self._get_world_properties_srv = service(
            '/gazebo/get_world_properties', GetWorldProperties, timeout)
        # /gazebo/pause_physics
        self._pause_physics_srv = service(
            '/gazebo/pause_physics', Empty, timeout)
        # /gazebo/reset_simulation
        self._reset_simulation_srv = service(
            '/gazebo/reset_simulation', Empty, timeout)
        # /gazebo/reset_world
        self._reset_world_srv = service(
            '/gazebo/reset_world', Empty, timeout)
        # /gazebo/set_joint_properties
        self._set_joint_properties_srv = service(
            '/gazebo/set_joint_properties', SetJointProperties, timeout)
        # /gazebo/set_light_properties
        self._set_light_properties_srv = service(
            '/gazebo/set_light_properties', SetLightProperties, timeout)
        # /gazebo/set_link_properties
        self._set_link_properties_srv = service(
            '/gazebo/set_link_properties', SetLinkProperties, timeout)
        # /gazebo/set_link_state
        self._set_link_state_srv = service(
            '/gazebo/set_link_state', SetLinkState, timeout)
        # /gazebo/set_logger_level
        self._set_logger_level_srv = service(
            '/gazebo/set_logger_level', Empty, timeout)  # FIXME: Sure about empty ?
        # /gazebo/set_model_configuration
        self._set_model_configuration_srv = service(
            '/gazebo/set_model_configuration', SetModelConfiguration, timeout)
        # /gazebo/set_model_state
        self._set_model_state_srv = service(
            '/gazebo/set_model_state', SetModelState, timeout)
        # /gazebo/set_parameters
        self._set_parameters_srv = service(
            '/gazebo/set_parameters', Empty, timeout)  # Sure about empty ?
        # /gazebo/set_physics_properties
        self._set_physics_properties_srv = service(
            '/gazebo/set_physics_properties', SetPhysicsProperties, timeout)
        # /gazebo/spawn_sdf_model
        self._spawn_sdf_model_srv = service(
            '/gazebo/spawn_sdf_model', SpawnModel, timeout)
        # /gazebo/spawn_urdf_model
        self._spawn_urdf_model_srv = service(
            '/gazebo/spawn_urdf_model', SpawnModel, timeout)
        # /gazebo/unpause_physics
        self._unpause_physics_srv = service(
            '/gazebo/unpause_physics', Empty, timeout)

        rospy.loginfo('All Gazebo services ready')

    def _connect_topics(self, timeout=None):
        # /gazebo/link_states
        self._link_states_sub = subscribe('/gazebo/link_states', LinkStates,
                                          self._link_states_callback, timeout=0)
        # /gazebo/model_states
        self._model_states_sub = subscribe('/gazebo/model_states', ModelStates,
                                           self._model_states_callback, timeout)
        # /gazebo/parameter_descriptions
        self._parameter_descriptions_sub = subscribe('/gazebo/parameter_descriptions', ConfigDescription,
                                                     self._parameter_descriptions_callback, timeout)
        # /gazebo/parameter_updates
        self._parameter_updates_sub = subscribe('/gazebo/parameter_updates', Config,
                                                self._parameter_updates_callback, timeout)
        # /gazebo/set_link_state
        self._set_link_state_sub = subscribe('/gazebo/set_link_state', LinkState,
                                             self._set_link_state_callback, timeout=0)
        # /gazebo/set_model_state
        self._set_model_state_sub = subscribe('/gazebo/set_model_state', ModelState,
                                              self._set_model_state_callback, timeout=0)
        rospy.loginfo('All Gazebo subscribers ready')

    # --- Getters and setters --
    # region

    @property
    def time_step(self) -> float:
        """Get or set the current time_step. Setting the time_step to a new value
        will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._time_step

    @time_step.setter
    def time_step(self, value: float) -> None:
        self._set_some_physics_properties(time_step=value)

    @property
    def paused(self):
        """Get or set the current paused. Setting the paused to a new value
        will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._paused

    @paused.setter
    def paused(self, value: bool) -> None:
        if value:
            self.pause_physics()
        else:
            self.unpause_physics()

    @property
    def max_update_rate(self):
        """Get or set the current max_update_rate. Setting the max_update_rate
        to a new value will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._max_update_rate

    @max_update_rate.setter
    def max_update_rate(self, value: float) -> None:
        self._set_some_physics_properties(max_update_rate=value)

    @property
    def gravity(self) -> List[float]:
        """Get or set the current gravity. Setting the gravity to a new value
        will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._gravity.copy()

    @gravity.setter
    def gravity(self, value: List[float]) -> None:
        self._set_some_physics_properties(gravity=value)

    @property
    def auto_disable_bodies(self):
        """The current version does not allow to set auto_disable_bodies.
        Get or set the current auto_disable_bodies. Setting the
        auto_disable_bodies to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._auto_disable_bodies

    @auto_disable_bodies.setter
    def auto_disable_bodies(self, value: bool) -> None:
        rospy.logwarn(
            'The current version does not allow to set auto_disable_bodies.')
        self._set_some_physics_properties(auto_disable_bodies=value)

    @property
    def sor_pgs_precon_iters(self) -> int:
        """Get or set the current sor_pgs_precon_iters. Setting the
        sor_pgs_precon_iters to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_precon_iters

    @sor_pgs_precon_iters.setter
    def sor_pgs_precon_iters(self, value: int) -> None:
        self._set_some_physics_properties(sor_pgs_precon_iters=value)

    @property
    def sor_pgs_iters(self) -> int:
        """Get or set the current sor_pgs_iters. Setting the sor_pgs_iters to a
        new value will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_iters

    @sor_pgs_iters.setter
    def sor_pgs_iters(self, value: int) -> None:
        self._set_some_physics_properties(sor_pgs_iters=value)

    @property
    def sor_pgs_w(self) -> float:
        """Get or set the current sor_pgs_w. Setting the sor_pgs_w to a new
        value will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_w

    @sor_pgs_w.setter
    def sor_pgs_w(self, value: float) -> None:
        self._set_some_physics_properties(sor_pgs_w=value)

    @property
    def sor_pgs_rms_error_tol(self) -> float:
        """Get or set the current sor_pgs_rms_error_tol. Setting the
        sor_pgs_rms_error_tol to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_rms_error_tol

    @sor_pgs_rms_error_tol.setter
    def sor_pgs_rms_error_tol(self, value: float) -> None:
        rospy.logwarn(
            'The current version does not allow to set sor_pgs_rms_error_tol.')
        self._set_some_physics_properties(sor_pgs_rms_error_tol=value)

    @property
    def contact_surface_layer(self) -> float:
        """Get or set the current contact_surface_layer. Setting the
        contact_surface_layer to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._contact_surface_layer

    @contact_surface_layer.setter
    def contact_surface_layer(self, value: float) -> None:
        self._set_some_physics_properties(contact_surface_layer=value)

    @property
    def contact_max_correcting_vel(self) -> float:
        """Get or set the current contact_max_correcting_vel. Setting the
        contact_max_correcting_vel to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._contact_max_correcting_vel

    @contact_max_correcting_vel.setter
    def contact_max_correcting_vel(self, value: float) -> None:
        self._set_some_physics_properties(contact_max_correcting_vel=value)

    @property
    def cfm(self) -> float:
        """Get or set the current cfm. Setting the cfm to a new value
        will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._cfm

    @cfm.setter
    def cfm(self, value: float) -> None:
        self._set_some_physics_properties(cfm=value)

    @property
    def erp(self) -> float:
        """Get or set the current erp. Setting the erp to a new value
        will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._erp

    @erp.setter
    def erp(self, value: float) -> None:
        self._set_some_physics_properties(erp=value)

    @property
    def max_contacts(self) -> int:
        """Get or set the current max_contacts. Setting the max_contacts to a
        new value will reconfigure the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._max_contacts

    @max_contacts.setter
    def max_contacts(self, value: int) -> None:
        self._set_some_physics_properties(max_contacts=value)

    # endregion

    # -- Topic callbacks --
    # region
    def _link_states_callback(self, value):
        nb_links = len(value.name)
        for i in range(nb_links):
            self.link_states[value.name[i]] = {
                'postion': [value.pose[i].position.x, value.pose[i].position.y, value.pose[i].position.z],
                'orientation': [value.pose[i].orientation.x, value.pose[i].orientation.y, value.pose[i].orientation.z, value.pose[i].orientation.w],
                'linear_velocity': [value.twist[i].linear.x, value.twist[i].linear.y, value.twist[i].linear.z],
                'angular_velocity': [value.twist[i].angular.x, value.twist[i].angular.y, value.twist[i].angular.z]}

    def _model_states_callback(self, value):
        nb_models = len(value.name)
        for i in range(nb_models):
            self.model_states[value.name[i]] = {
                'postion': [value.pose[i].position.x, value.pose[i].position.y, value.pose[i].position.z],
                'orientation': [value.pose[i].orientation.x, value.pose[i].orientation.y, value.pose[i].orientation.z, value.pose[i].orientation.w],
                'linear_velocity': [value.twist[i].linear.x, value.twist[i].linear.y, value.twist[i].linear.z],
                'angular_velocity': [value.twist[i].angular.x, value.twist[i].angular.y, value.twist[i].angular.z]}

        rospy.logwarn_once('model_states_callback not implemented')

    def _parameter_descriptions_callback(self, value):
        # NOTE: it seems like gazebo subscribe to this this topic instead of publishing.
        # if it is the case, this callback is not necessary..
        # a message is published once when gazebo start.
        # I should look for the publisher
        rospy.logwarn_once('parameter_descriptions_callback not implemented')

    def _parameter_updates_callback(self, value):
        # NOTE: it seems like gazebo subscribe to this this topic instead of publishing.
        # if it is the case, this callback is not necessary.
        # a message is published once when gazebo start.
        # I should look for the publisher
        rospy.logwarn_once('parameter_updates_callback not implemented')

    def _set_link_state_callback(self, value):
        # NOTE: it seems like gazebo subscribe to this this topic instead of publishing.
        # if it is the case, this callback is not necessary.
        # a message is published once when gazebo start.
        # I should look for the publisher
        rospy.logwarn_once('set_link_state_callback not implemented')

    def _set_model_state_callback(self, value):
        # NOTE: it seems like gazebo subscribe to this this topic instead of publishing.
        # if it is the case, this callback is not necessary.
        # a message is published once when gazebo start.
        # I should look for the publisher
        rospy.logwarn_once('set_model_state_callback not implemented')

    # endregion

    # -- Services methods --
    # In this section, there is one method per ROS service.
    #  - The arguments of the method are exactly the same as those of the ROS service.
    #  - No optional arguments.
    #  - Arguments must be standard (floats, integers, dictionaries, strings, lists).
    #  - These methods convert the arguments to a Request object and then call the service.
    #  - If needed, the answer is converted into a Dict or List[Dict]
    #  - If the call fails, it raises an Exception.

    # region
    def apply_body_wrench(self, body_name: str, reference_frame: str, reference_point: List[float], force: List[float], torque: List[float], start_time_secs: int, start_time_nsecs: int, duration_secs: int, duration_nsecs: int) -> None:
        """Apply Wrench to Gazebo Body. Via the callback mechanism all Gazebo operations are made in world frame

        Args:
            body_name (str): Gazebo body to apply wrench (linear force and torque); body names are prefixed by model name, e.g. `pr2::base_link`
            reference_frame (str, optional): wrench is defined in the reference frame of this entity; use inertial frame if left empty. Defaults to ''. 
            reference_point (List[float]): wrench is defined at this location in the reference frame
            force (List[float]): force applied to the origin of the body
            torque (List[float]): torque applied to the origin of the body
            start_time_secs (int): seconds portion of wrench application start time (start_time = start_time_secs + start_time_nsecs/10e9); if start_time < current time, start as soon as possible
            start_time_nsecs (int): nano seconds portion of wrench application start time; see `start_time_secs` arg.
            duration_secs (int): seconds portion of wrench application duration (duration = duration_secs + duration_nsecs/10e9);
                                  if duration < 0, apply wrench continuously without end
                                  if duration = 0, do nothing
                                  if duration < step size, apply wrench for one step size
            duration_nsecs (int): nano seconds portion of wrench application duration; see `duration_secs` arg.

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = ApplyBodyWrenchRequest()
        req.body_name = body_name
        req.reference_frame = reference_frame
        req.reference_point.x = reference_point[0]
        req.reference_point.y = reference_point[1]
        req.reference_point.z = reference_point[2]
        req.wrench.force.x = force[0]
        req.wrench.force.y = force[1]
        req.wrench.force.z = force[2]
        req.wrench.torque.x = torque[0]
        req.wrench.torque.y = torque[1]
        req.wrench.torque.z = torque[2]
        req.start_time.secs = start_time_secs
        req.start_time.nsecs = start_time_nsecs
        req.duration.secs = duration_nsecs
        req.duration.nsecs = duration_nsecs
        self._send_request(self._apply_body_wrench_srv, req)

    def apply_joint_effort(self, joint_name: str, effort: float, start_time_secs: int, start_time_nsecs: int, duration_secs: int, duration_nsecs: int) -> None:
        """Set urdf joint effort

        Args:
            joint_name (str): joint to apply wrench (linear force and torque)
            effort (float): effort to apply
            start_time_secs (int): seconds portion of wrench application start time (start_time = start_time_secs + start_time_nsecs/10e9); if start_time < current time, start as soon as possible
            start_time_nsecs (int): nano seconds portion of wrench application start time; see `start_time_secs` arg.
            duration_secs (int): seconds portion of wrench application duration (duration = duration_secs + duration_nsecs/10e9);
                                  if duration < 0, apply wrench continuously without end
                                  if duration = 0, do nothing
                                  if duration < step size, assume step size and display warning in status_message
            duration_nsecs (int): nano seconds portion of wrench application duration; see `duration_secs` arg.

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = ApplyJointEffortRequest()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time.secs = start_time_secs
        req.start_time.nsecs = start_time_nsecs
        req.duration.secs = duration_secs
        req.duration.nsecs = duration_nsecs
        self._send_request(self._apply_joint_effort_srv, req)

    def clear_body_wrenches(self, body_name: str) -> None:
        """Clear body wrenches

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = BodyRequestRequest()
        req.body_name = body_name
        self._send_request_no_succes_attribut(
            self._clear_body_wrenches_srv, req)

    def clear_joint_forces(self, joint_name: str) -> None:
        """Clear joint forces

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = JointRequestRequest()
        req.joint_name = joint_name
        self._send_request_no_succes_attribut(
            self._clear_joint_forces_srv, req)

    def delete_light(self, light_name: str) -> None:
        """Delete a light

        Args:
            light_name (str): name of the light to be deleted

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = DeleteLightRequest()
        req.light_name = light_name
        self._send_request(self._delete_light_srv, req)

    def delete_model(self, model_name: str) -> None:
        """Delete a model

        Args:
            model_name (str): name of the Gazebo Model to be deleted

        Raises:
            rospy.ServiceException: if service calling failed
        """
        req = DeleteModelRequest()
        req.model_name = model_name
        self._send_request(self._delete_model_srv, req)

    def get_joint_properties(self, joint_name: str) -> Dict:
        """Get joint properties.

        Args:
            joint_name (str): the joint name

        Raises:
            rospy.ServiceException: if service call fail
            Exception: if unknwown error encoutered during service call
            Exception: If calling was not successfull.

        Returns:
            dict: containing 'joint_type' (str), 'position' (float) and 'rate' (float) 
            Dict:
                "joint_type" (str): 'REVOLUTE', 'CONTINUOUS', 'PRISMATIC', 'FIXED', 'BALL' or 'UNIVERSAL'
                "position" (List[float]): positions; number of elements depends on the joint type
                "rate" (List[float]): velocities; number of elements depends on the joint type
        """
        req = GetJointPropertiesRequest()
        req.joint_name = joint_name

        ans = self._send_request(self._get_joint_properties_srv, req)
        out = {
            'joint_type': self._JOINT_TYPE[ans.type],
            'damping': list(ans.damping),
            'position': list(ans.position),
            'rate': list(ans.rate)}
        return out

    def get_light_properties(self, light_name: str) -> Dict:
        """Get light properties

        Args:
            light_name (str): name of Gazebo Light

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            Dict:
                "diffuse" (List[float]): diffuse color as red, green, blue, alpha
                "attenuation_constant" (float): 
                "attenuation_linear" (float): 
                "attenuation_quadratic" (float): 
        """
        raise NotImplementedError()

    def get_link_properties(self, link_name: str) -> Dict:
        """Get link properties

        Args:
            link_name (str): name of link; link names are prefixed by model name, e.g. `pr2::base_link`

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            Dict:
                "position" (List[float]): cartesian position center of mass in link frame
                "orientation" (List[float]): orientation as quaternion of the center of mass in link frame
                "gravity_mode" (bool): gravity mode
                "mass" (float): linear mass of link
                "ixx" (float): moment of inertia
                "ixy" (float): moment of inertia
                "ixz" (float): moment of inertia
                "iyy" (float): moment of inertia
                "iyz" (float): moment of inertia
                "izz" (float): moment of inertia
        """
        raise NotImplementedError()

    def get_link_state(self, link_name: str) -> Dict:
        """Get link state.

        Args:
            link_name (str): the link name

        Raises:
            rospy.ServiceException: if service call fail
            Exception: if unknwown error encoutered during service call
            Exception: If calling was not successfull.

        Returns:
            Dict:
                "link_name" (str): link name, link_names are in gazebo scoped name notation, `[model_name::body_name]`
                "position" (List[float]): cartesian position center of mass in reference frame
                "orientation" (List[float]): orientation as quaternion of the center of mass in reference frame
                "linear_velocity" (List[float]): linear velocity of the center of mass in reference frame
                "angular_velocity" (List[float]): angular velocity of the center of mass in reference frame
                "reference_frame" (str): pose/twist relative to the frame of this link/body; leave empty or "world" or "map"; defaults to world frame
        """
        rospy.logdebug("Getting link %s state" % link_name)
        try:
            req = GetLinkStateRequest(link_name=link_name)
            ans = self._get_link_state_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Getting link %s state service call failed")
            raise e
        except Exception as e:
            rospy.logerr("Unknwon error")
            raise e
        if not ans.success:
            rospy.logerr(ans.status_message)
            raise Exception(ans.status_message)
        else:
            ls = ans.link_state
            return {
                'link_name': ls.link_name,
                'position': [ls.pose.position.x, ls.pose.position.y, ls.pose.position.z],
                'orientation': [ls.pose.orientation.x, ls.pose.orientation.y, ls.pose.orientation.z, ls.pose.orientation.w],
                'linear_velocity': [ls.twist.linear.x, ls.twist.linear.y, ls.twist.linear.z],
                'angular_velocity': [ls.twist.angular.x, ls.twist.angular.y, ls.twist.angular.z],
                'reference_frame': ls.reference_frame
            }

    def get_loggers(self) -> List[Dict]:
        """Get list of loggers

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            List[Dict]: Each dict of the list contains keywords
                'name' (str): logger name
                'level' (str): logger level ('debug', 'info', 'warn', 'error', 'fatal')
        """
        raise NotImplementedError()

    def get_model_properties(self, model_name: str) -> Dict:
        """Get model properties

        Args:
            model_name (str): name of Gazebo Model

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            Dict:
                "parent_model_name" (str): parent model
                "canonical_body_name" (str): name of canonical body, body names are prefixed by model name, e.g. `pr2::base_link`
                "body_names" List[str]: list of bodies, body names are prefixed by model name, e.g. ` pr2::base_link` 
                "geom_names" List[str]: list of geoms
                "joint_names"  List[str]: list of joints attached to the model
                "child_model_names" List[str]: list of child models
                "is_static" (bool): if model is static
        """
        raise NotImplementedError()

    def get_model_state(self, model_name: str, relative_entity_name: str) -> Dict:
        """Get model state

        Args:
            model_name (str): name of Gazebo Model
            relative_entity_name (str): return pose and twist relative to this entity  an entity can be a model, body, or geom; be sure to use gazebo scoped naming notation (e.g. `[model_name::body_name]`); leave empty or "world" will use inertial world frame

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            Dict:
                "seq" (int): holds the number of requests since the plugin started
                "secs" (int): seconds (stamp_secs) since pose
                "nsecs" (int): nanoseconds since stamp_secs
                "frame_id" (str): not used but currently filled with the relative_entity_name
                "position" (List[float]): cartesian position center of mass in relative entity frame
                "orientation" (List[float]): orientation as quaternion of the center of mass in relative entity frame
                "linear_velocity" (List[float]): linear velocity of the center of mass in relative entity frame
                "angular_velocity" (List[float]): angular velocity of the center of mass in relative entity frame
        """
        raise NotImplementedError()

    def get_physics_properties(self) -> Dict:
        """Get physics properties

        Raises:
            rospy.ServiceException: if service call fail

        Returns:
            Dict:
                "time_step" (float): dt in seconds
                "pause" (bool): true if physics engine is paused
                "max_update_rate" (float): throttle maximum physics update rate
                "gravity" (List[float]): gravity vector (e.g. earth ~[0,0,-9.81])
                "auto_disable_bodies" (bool): enable auto disabling of bodies, default false
                "sor_pgs_precon_iters" (int): preconditioning inner iterations when uisng projected Gauss Seidel
                "sor_pgs_iters" (int): inner iterations when uisng projected Gauss Seidel
                "sor_pgs_w" (float): relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
                "sor_pgs_rms_error_tol" (float): rms error tolerance before stopping inner iterations
                "contact_surface_layer" (float): contact "dead-band" width
                "contact_max_correcting_vel" (float): contact maximum correction velocity
                "cfm" (float): global constraint force mixing
                "erp" (float): global error reduction parameter
                "max_contacts" (int): maximum contact joints between two geoms
        """
        ans = self._send_request(
            self._get_physics_properties_srv)

        physics = {
            'time_step': ans.time_step,
            'pause': ans.pause,
            'max_update_rate': ans.max_update_rate,
            'gravity': [ans.gravity.x, ans.gravity.y, ans.gravity.z],
            'auto_disable_bodies': ans.ode_config.auto_disable_bodies,
            'sor_pgs_precon_iters': ans.ode_config.sor_pgs_precon_iters,
            'sor_pgs_iters': ans.ode_config.sor_pgs_iters,
            'sor_pgs_w': ans.ode_config.sor_pgs_w,
            'sor_pgs_rms_error_tol': ans.ode_config.sor_pgs_rms_error_tol,
            'contact_surface_layer': ans.ode_config.contact_surface_layer,
            'contact_max_correcting_vel': ans.ode_config.contact_max_correcting_vel,
            'cfm': ans.ode_config.cfm,
            'erp': ans.ode_config.erp,
            'max_contacts': ans.ode_config.max_contacts
        }

        return physics

    def get_world_properties(self) -> Dict:
        """Get world properties

        Raises:
            rospy.ServiceException if pausing fails

        Returns:
            Dict: [description]
                "sim_time" (float): current sim time
                "model_names" (List[str]): list of models in the world
                "rendering_enabled" (bool): whether gazebo rendering engine is enabled, currently always True
        """
        raise NotImplementedError()

    def pause_physics(self) -> None:
        """Pauses the simulation

        Raises:
            rospy.ServiceException if pausing fails
        """
        rospy.logdebug("Pausing the simulation")
        try:
            r = self._pause_physics_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation pausing service call failed")
            raise e

    def reset_simulation(self) -> None:
        """Reset the simulation

        Raises:
            rospy.ServiceException if reseting fails
        """
        rospy.loginfo("Reseting the simulation")
        try:
            self._reset_sim_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation reset service call failed")
            raise e

    def reset_world(self) -> None:
        """Reset the world

        Raises:
            rospy.ServiceException if reseting fails
        """
        rospy.loginfo("Reseting the world")
        try:
            self._reset_world_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation reset service call failed")
            raise e

    def set_joint_properties(self, joint_name: str, damping: float, hiStop: float, loStop: float, erp: float, cfm: float, stop_erp: float, stop_cfm: float, fudge_factor: float, fmax: float, vel: float) -> None:
        """Set joint properties

        Args:
            joint_name (str): name of joint
            damping (float): joint damping
            hiStop (float): joint limit
            loStop (float): joint limit
            erp (float): set joint erp
            cfm (float): set joint cfm
            stop_erp (float): set joint erp for joint limit "contact" joint
            stop_cfm (float): set joint cfm for joint limit "contact" joint
            fudge_factor (float): joint fudge_factor applied at limits, see ODE manual for info.
            fmax (float): ode joint param fmax
            vel (float): ode joint param vel

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_light_properties(self, light_name: str, diffuse: List[float], attenuation_constant: float, attenuation_linear: float, attenuation_quadratic: float) -> None:
        """Set light properties

        Args:
            light_name (str): name of Gazebo Light
            diffuse (List[float]): diffuse color as red, green, blue, alpha
            attenuation_constant (float): constant attenuation
            attenuation_linear (float): linear attenuation
            attenuation_quadratic (float): quadratic attenuation

        Raises:
            rospy.ServiceException if reseting fails
        """
        req = SetLightPropertiesRequest()
        req.light_name = light_name
        req.diffuse.r = diffuse[0]
        req.diffuse.g = diffuse[1]
        req.diffuse.b = diffuse[2]
        req.diffuse.a = diffuse[3]
        req.attenuation_constant = attenuation_constant
        req.attenuation_linear = attenuation_linear
        req.attenuation_quadratic = attenuation_quadratic
        self._send_request(self._set_light_properties_srv, req)

    def set_link_properties(self, link_name: str, position: List[float], orientation: List[float], gravity_mode: bool, mass: float, ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float) -> None:
        """Set link properties

        Args:
            link_name (str): name of link; link names are prefixed by model name, e.g. `pr2::base_link`
            position (List[float]): center of mass location in link frame
            orientation (List[float]): orientation as quaternion of the moment of inertias relative to the link frame
            gravity_mode (bool): set gravity mode on/off
            mass (float): linear mass of link
            ixx (float): moment of inertia
            ixy (float): moment of inertia
            ixz (float): moment of inertia
            iyy (float): moment of inertia
            iyz (float): moment of inertia
            izz (float): moment of inertia

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_link_state(self, link_name: str, position: List[float], orientation: List[float], linear_velocity: List[float], angular_velocity: List[float], reference_frame: str) -> None:
        """Set link state

        Args:
            link_name (str): link name, link_names are in gazebo scoped name notation, `[model_name::body_name]`
            position (List[float]): desired position of the center of mass in reference frame
            orientation (List[float]): desired orientation (as quaternion) of the center of mass in reference frame
            linear_velocity (List[float]): desired linear velocity of the center of mass in reference frame
            angular_velocity (List[float]): desired angular velocity of the center of mass in reference frame
            reference_frame (str): set pose/twist relative to the frame of this link/body; leave empty or "world" or "map" defaults to world-frame

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_logger_level(self, logger: str, level: str) -> None:
        """Set logger level

        Args:
            logger (str): logger name
            level (str): logger level (`'debug'`, `'info'`, `'warn'`, `'error'` or `'fatal'`)

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_model_configuration(self, model_name: str, urdf_param_name: str, joint_names: List[str], joint_positions: List[float]) -> None:
        """Set model configuration

        Args:
            model_name (str): model to set state
            urdf_param_name (str): UNUSED
            joint_names (List[str]): list of joints to set positions; if joint is not listed here, preserve current position.
            joint_positions (List[float]): set to this position.

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_model_state(self, model_name: str, position: List[float], orientation: List[float], linear_velocity: List[float], angular_velocity: List[float], reference_frame: str) -> None:
        """Set Gazebo model pose and twist

        Args:
            model_name (str): model to set state (pose and twist)
            position (List[float]): desired position in reference frame
            orientation (List[float]): desired orientation (as quaternion) in reference frame
            linear_velocity (List[float]): desired linear velocity in reference frame
            angular_velocity (List[float]): desired angular velocity in reference frame
            reference_frame (str): set pose/twist relative to the frame of this entity (Body/Model); leave empty or "world" or "map" defaults to world-frame

        Raises:
            rospy.ServiceException if reseting fails
        """
        raise NotImplementedError()

    def set_parameters(self, bools: List[Dict],  ints: List[Dict], strs: List[Dict], doubles: List[Dict], groups: List[Dict]) -> Dict:
        """Set or get simulation parameters

        Args:
            bools (List[Dict]): bools, e.g.  `{'name': 'auto_disable_bodies', 'value': False}`
            ints (List[Dict]): ints, e.g.  `{'name': 'max_contact', 'value': 20}`
            strs (List[Dict]): strs
            doubles (List[Dict]): doubles, e.g.  `{'name': 'time_step', 'value': 0.001}`
            groups (List[Dict]): groups, e.g.  `{'name': 'Default', 'state': True, 'id': 0, 'parent': 0}`

        Raises:
            rospy.ServiceException if reseting fails

        Returns:
            Dict
                'bools' (List[Dict]): bools, e.g.  `{'name': 'auto_disable_bodies', 'value': False}`
                'ints' (List[Dict]): ints, e.g.  `{'name': 'max_contact', 'value': 20}`
                'strs' (List[Dict]): strs
                'doubles' (List[Dict]): doubles, e.g.  `{'name': 'time_step', 'value': 0.001}`
                'groups' (List[Dict]): groups, e.g.  `{'name': 'Default', 'state': True, 'id': 0, 'parent': 0}`
        """
        raise NotImplementedError()

    def set_physics_properties(self,  time_step: float, max_update_rate: float, gravity: List[float], auto_disable_bodies: bool, sor_pgs_precon_iters: int, sor_pgs_iters: int, sor_pgs_w: float, sor_pgs_rms_error_tol: float, contact_surface_layer: float, contact_max_correcting_vel: float, cfm: float, erp: float, max_contacts: int) -> None:
        """Set physics properties

        Args:
            time_step (float): dt in seconds
            max_update_rate (float): throttle maximum physics update rate
            gravity (List[float]): gravity vector (e.g. earth ~[0,0,-9.81])
            auto_disable_bodies (bool): enable auto disabling of bodies, default false
            sor_pgs_precon_iters (int): preconditioning inner iterations when uisng projected Gauss Seidel
            sor_pgs_iters (int): inner iterations when uisng projected Gauss Seidel
            sor_pgs_w (float): relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
            sor_pgs_rms_error_tol (float): rms error tolerance before stopping inner iterations
            contact_surface_layer (float): contact "dead-band" width
            contact_max_correcting_vel (float): contact maximum correction velocity
            cfm (float): global constraint force mixing
            erp (float): global error reduction parameter
            max_contacts (int): maximum contact joints between two geoms

        Raises:
            rospy.ServiceException if reseting fails
        """
        req = SetPhysicsPropertiesRequest()
        req.time_step = time_step
        req.max_update_rate = max_update_rate
        gravity_vec = Vector3()
        gravity_vec.x = gravity[0]
        gravity_vec.y = gravity[1]
        gravity_vec.z = gravity[2]
        req.gravity = gravity_vec
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = auto_disable_bodies
        ode_config.sor_pgs_precon_iters = sor_pgs_precon_iters
        ode_config.sor_pgs_iters = sor_pgs_iters
        ode_config.sor_pgs_w = sor_pgs_w
        ode_config.sor_pgs_rms_error_tol = sor_pgs_rms_error_tol
        ode_config.contact_surface_layer = contact_surface_layer
        ode_config.contact_max_correcting_vel = contact_max_correcting_vel
        ode_config.cfm = cfm
        ode_config.erp = erp
        ode_config.max_contacts = max_contacts
        req.ode_config = ode_config
        self._send_request(self._set_physics_properties_srv, req)

    def spawn_sdf_model(self, model_name: str, model_xml: str, robot_namespace: str, initial_position: List[float], initial_orientation: List[float], reference_frame: str) -> None:
        """Spawn an sdf model

        Args:
            model_name (str): name of the model to be spawn
            model_xml (str): this should be an sdf or gazebo xml
            robot_namespace (str): spawn robot and all ROS interfaces under this namespace
            initial_position (List[float]): only applied to canonical body
            initial_orientation (List[float]): only applied to canonical body
            reference_frame (str): initial_pose is defined relative to the frame of this model/body, if left empty or "world", then gazebo world frame is used

        Raises:
            rospy.ServiceException if reseting fails
        """
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = Pose(
            Point(*initial_position),
            Quaternion(*initial_orientation)
        )
        req.reference_frame = reference_frame
        self._send_request(self._spawn_sdf_model_srv, req)

    def spawn_urdf_model(self, model_name: str, model_xml: str, robot_namespace: str, initial_position: List[float], initial_orientation: List[float], reference_frame: str) -> None:
        """Spawn an urdf model

        Args:
            model_name (str): name of the model to be spawn
            model_xml (str): this should be an urdf or gazebo xml
            robot_namespace (str): spawn robot and all ROS interfaces under this namespace
            initial_position (List[float]): catesian position [x, y, z], only applied to canonical body
            initial_orientation (List[float]): quaternion [x, y, z, w], only applied to canonical body
            reference_frame (str): initial_pose is defined relative to the frame of this model/body, if left empty or "world", then gazebo world frame is used
        """
        req = SpawnModelRequest()
        req.model_name = model_name
        req.model_xml = model_xml
        req.robot_namespace = robot_namespace
        req.initial_pose = Pose(
            Point(*initial_position),
            Quaternion(*initial_orientation)
        )
        req.reference_frame = reference_frame
        self._send_request(self._spawn_urdf_model_srv, req)

    def unpause_physics(self):
        """Unpause the simulation

        Raises:
            rospy.ServiceException if unpausing fails
        """
        rospy.logdebug("Unpausing the simulation")
        try:
            self._unpause_physics_srv()
        except rospy.ServiceException as e:
            rospy.logerr("Simulation unpausing service call failed")
            raise e
    # endregion

    # -- Derived method --
    # The following methods are intented to the user.
    # They must use the service methods and not the services directly.
    # They must be user friendly and provide all the features that a user can expect.

    # region
    def get_sim_time(self):
        """Get the current simulation time.
        """
        ans = self._get_world_properties_srv()
        return ans.sim_time

    def apply_body_force(self, body_name: str, reference_point: List[float], force: List[float], start_time: float = 0, duration: float = -1.0, reference_frame: str = '',):
        """Apply force to Gazebo Body

        Args:
            body_name (str): Gazebo body to apply wrench (linear force and torque); body names are prefixed by model name, e.g. pr2::base_link 
            reference_point (List[float]): wrench is defined at this location in the reference frame
            force (List[float]): force applied to the origin of the body
            start_time (float, optional): wrench application start time (seconds); if start_time is not specified, or start_time < current time, start as soon as possible
            duration (float, optionnal): if duration < 0, apply wrench continuously without end
                              if duration = 0, do nothing
                              if duration < step size, apply wrench for one step size
            reference_frame (str, optional): wrench is defined in the reference frame of this entity; use inertial frame if left empty. Defaults to ''.

        Raises:
            rospy.ServiceException: if service calling failed
        """
        torque = [0, 0, 0]
        self.apply_body_wrench(body_name, reference_frame,
                               reference_point, force, torque, start_time, duration)

    def spawn_light(self, light_name: str, position: List[float], yaw: float = 0, pitch: float = 0, roll: float = -1.0, diffuse_red: float = 0.5, diffuse_green: float = 0.5, diffuse_blue: float = 0.5, diffuse_alpha: float = 1.0, specular_red: float = 0.1, specular_green: float = 0.1, specular_blue: float = 0.1, specular_alpha: float = 1.0, attenuation_range: float = 20, attenuation_constant: float = 0.5, attenuation_linear: float = 0.1, attenuation_quadratic: float = 0.0, cast_shadows: bool = False, reference_frame: str = ''):
        """Spawn a light

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
            specular_red (float, optional): specular red beam rate. Defaults to 0.1.
            specular_green (float, optional): specular green beam rate. Defaults to 0.1.
            specular_blue (float, optional): specular blue beam rate. Defaults to 0.1.
            specular_alpha (float, optional): specular opacity rate. Defaults to 1.0.
            attenuation_range (float, optional): light range. Defaults to 20.
            attenuation_constant (float, optional): constant attenuation. Defaults to 0.5.
            attenuation_linear (float, optional): linear attenuation. Defaults to 0.1.
            attenuation_constant (float, optional): quadratic attenuation. Defaults to 0.0.
            cast_shadows (bool, optional): cast shadows. Defaults to False.
        """
        template = '''<?xml version="1.0" ?>
            <sdf version="1.5">
            <light type="point" name='unused_name'>
                <pose>{x} {y} {z} {yaw} {pitch} {roll}</pose>
                <diffuse>{diffuse_red} {diffuse_green} {diffuse_blue} {diffuse_alpha}</diffuse>
                <specular>{specular_red} {specular_green} {specular_blue} {specular_alpha}</specular>
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
            diffuse_red=diffuse_red, diffuse_green=diffuse_green, diffuse_blue=diffuse_blue, diffuse_alpha=diffuse_alpha,
            specular_red=specular_red, specular_green=specular_green, specular_blue=specular_blue, specular_alpha=specular_alpha,
            attenuation_range=attenuation_range,
            attenuation_constant=attenuation_constant,
            attenuation_linear=attenuation_linear,
            attenuation_quadratic=attenuation_quadratic,
            cast_shadows=str(cast_shadows).lower())

        self.spawn_sdf_model(
            model_name=light_name,
            model_xml=model_xml,
            robot_namespace='',
            initial_position=[0.0, 0.0, 0.0],
            initial_orientation=[0.0, 0.0, 0.0, 0.0],
            reference_frame=reference_frame)

    # endregion

    # -- Helper (private) methods
    # region

    def _fetch_physics_properties(self):
        """Prive method that update the physics private attributes
        """
        physics = self.get_physics_properties()
        rospy.logdebug("Updating physics attributes")

        self._time_step = physics['time_step']
        self._paused = physics['pause']
        self._max_update_rate = physics['max_update_rate']
        self._gravity = physics['gravity']
        self._auto_disable_bodies = physics['auto_disable_bodies']
        self._sor_pgs_precon_iters = physics['sor_pgs_precon_iters']
        self._sor_pgs_iters = physics['sor_pgs_iters']
        self._sor_pgs_w = physics['sor_pgs_w']
        self._sor_pgs_rms_error_tol = physics['sor_pgs_rms_error_tol']
        self._contact_surface_layer = physics['contact_surface_layer']
        self._contact_max_correcting_vel = physics['contact_max_correcting_vel']
        self._cfm = physics['cfm']
        self._erp = physics['erp']
        self._max_contacts = physics['max_contacts']

    def _set_some_physics_properties(self, **kwargs):
        """Private method that update physics attributes contained in kwargs
        """
        if not kwargs:
            rospy.logwarn("An unnecessary request will be sent")

        physics = self.get_physics_properties()
        physics.update(kwargs)
        # Unlike the Answer, 'pause' is not a keyword of the request
        physics.pop('pause')

        if not self._paused:
            self.pause_physics()
            self.set_physics_properties(**physics)
            self.unpause_physics()
        else:
            self.set_physics_properties(**physics)

    def _send_request(self, srv, req=None):
        """Send a request through a service.

        Args:
            srv (Service): the service
            req (Request): the request

        Raises:
            Exception: if the request is rejected
        """

        rospy.logdebug('Calling service %s' % srv.resolved_name)
        if req:
            ans = srv(req)
        else:
            ans = srv()

        if not ans.success:
            rospy.logerr(ans.status_message)
            raise Exception(ans.status_message)
        else:
            rospy.logdebug("Calling to service %s succeed, %s" %
                           (srv.resolved_name, ans.status_message))
            return ans

    def _send_request_no_succes_attribut(self, srv, req=None):
        """Identical as _send_request, for answers without `success` and `statut_message`
        """
        # For some reason, some answers does not have success attribute and statut_message

        rospy.logdebug('Calling service %s' % srv.resolved_name)
        if req:
            ans = srv(req)
        else:
            ans = srv()

        rospy.logdebug("Calling to service %s succeed" %
                       (srv.resolved_name))
        return ans
    # endregion

    # -- Miscellaneous --

    @contextmanager
    def pausing(self):
        """Pausing context
        """
        try:
            self.pause_physics()
            yield
        finally:
            self.unpause_physics()

    def __del__(self):
        try:
            self._link_states_sub.unsubscribe()
        except:
            pass
        try:
            self._model_states_sub.unsubscribe()
        except:
            pass
        try:
            self._parameter_descriptions_sub.unsubscribe()
        except:
            pass
        try:
            self._parameter_updates_sub.unsubscribe()
        except:
            pass
        try:
            self._set_link_state_sub.unsubscribe()
        except:
            pass
        try:
            self._set_model_state_sub.unsubscribe()
        except:
            pass

        rospy.loginfo('All Gazebo subscribers deleted')
