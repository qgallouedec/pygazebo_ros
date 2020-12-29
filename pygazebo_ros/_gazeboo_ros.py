# -*- coding:utf-8 -*-

import time
import os
import sys

import rospy
import roslaunch
import rospkg
import rosgraph
from std_srvs.srv import Empty

from gazebo_msgs.msg import LinkStates, ModelStates, LinkState, ModelState
from gazebo_msgs.srv import (
    ApplyBodyWrench, ApplyBodyWrenchRequest,
    ApplyJointEffort, ApplyJointEffortRequest,
    BodyRequest, BodyRequestRequest,
    JointRequest, JointRequestRequest,
    DeleteLight, DeleteLightRequest,
    DeleteModel, DeleteModelRequest,
    GetJointProperties, GetJointPropertiesRequest,
    GetLightProperties, GetLightPropertiesRequest,
    GetLinkProperties, GetLinkPropertiesRequest,
    GetLinkState, GetLinkStateRequest,
    GetModelProperties, GetModelPropertiesRequest,
    GetModelState, GetModelStateRequest,
    GetPhysicsProperties,
    GetWorldProperties, GetWorldPropertiesRequest,
    SetJointProperties, SetJointPropertiesRequest,
    SetLightProperties, SetLightPropertiesRequest,
    SetLinkProperties, SetLinkPropertiesRequest,
    SetLinkState, SetLinkStateRequest,
    SetModelConfiguration, SetModelConfigurationRequest,
    SetModelState, SetModelStateRequest,
    SetPhysicsProperties, SetPhysicsPropertiesRequest,
    SpawnModel, SpawnModelRequest)

from dynamic_reconfigure.msg import (
    ConfigDescription, Config,
    BoolParameter, IntParameter, StrParameter, DoubleParameter, GroupState)
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from roscpp.srv import (
    GetLoggers, GetLoggersRequest,
    SetLoggerLevel, SetLoggerLevelRequest)

from typing import List, Dict, Optional, Callable, Any, Type, Tuple
import genpy

from contextlib import contextmanager

Message = genpy.Message
Request = genpy.Message
Response = genpy.Message
Dataclass = Type[genpy.Message]
Service = Callable[[Request], Response]
Position = Tuple[float, float, float]
Orientation = Tuple[float, float, float, float]
Force = Tuple[float, float, float]
Torque = Tuple[float, float, float]
Direction = Tuple[float, float, float]
Color = Tuple[float, float, float]
Attenuation = Tuple[float, float, float]


def subscribe(name: str, data_class: Dataclass,
              callback: Callable[[Message], None],
              timeout: Optional[float] = None) -> rospy.Subscriber:
    """Registering as a subscriber to a topic.

    Args:
        name (str): Topic name.
        data_class (Type[Message]): Data class of the topic.
        callback (Callable[[Message], None]): The function called each
            time a message is published on the topic.
        timeout (float, optional): Raise error if no message is published on
            the topic after a time. If `None`, does not wait for a message to
            be published. Defaults to `None`.

    Raises:
        ROSInterruptException: If waiting is interrupted.
        TimeoutError: If no message is published on the topic after
          `timeout` seconds. Set `timeout` to `None` not to raise this error.

    Returns:
        Subscriber: The subscriber. To stop subscription, call its method
            `unregister()`.
    """
    try:
        rospy.logdebug("Waiting for message on topic %s" % name)
        if timeout != 0:
            rospy.wait_for_message(name, data_class, timeout)
    except rospy.ROSInterruptException as e:
        rospy.logdebug(
            "Waiting for topic %s interrupted" % name)
        raise e
    except rospy.ROSException:
        err_msg = "Timeout exceded, no message received on topic %s" % name
        rospy.logerr(err_msg)
        raise TimeoutError(err_msg)
    else:
        rospy.logdebug("Subscriber to %s ready" % (name))
        return rospy.Subscriber(name, data_class, callback)


def publisher(name: str, data_class: Dataclass,
              timeout: Optional[float] = None) -> rospy.Publisher:
    """Registering as a publisher of a ROS topic.

    Args:
        name (str): Topic name.
        data_class (Type[Message]): Data class of the topic.
        timeout (float, optional): Raises an error if no subscriber is detected
            on the topic after `timeout` seconds. If `None`, does not wait for
            a subscriber. Defaults to `None`.

    Raises:
        ROSInterruptException: If waiting is interrupted.
        TimeoutError: If no subscriber is detected on the topic after `timeout`
            seconds. Set `timeout` to `None` not to raise this exception.

    Returns:
        Publisher: The publisher. To publish a message, call its method
            `publish(your_msg)`.
    """
    start = time.time()
    pub = rospy.Publisher(name, data_class, queue_size=1)
    rate = rospy.Rate(5)  # 2hz #FIXME: if paused, time is paused
    rospy.logdebug("Looking for subscriber for topic %s" % name)
    while pub.get_num_connections() == 0:
        if timeout is not None and time.time() - start > timeout:
            err_msg = "Timeout execeded, no subscriber\
                found for topic %s" % name
            rospy.logerr(err_msg)
            raise TimeoutError(err_msg)
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


def service(name: str, service_class: Any,
            timeout: Optional[float] = None) -> rospy.ServiceProxy:
    """Create a handle to a ROS service for invoking calls.

    Args:
        name (str): Service name (e.g., `'/gazebo/delete_light'`)
        service_class (Any): Service class 
            (e.g., `gazebo_msgs.srv.DeleteLight`)
        timeout (float, optional): Raise error if timeout exceded. If `None`,
            wait indefinitely. Defaults to `None`.

    Raises:
        ROSInterruptException: If waiting is interrupted.
        TimeoutError: If timeout exceded.

    Returns:
        ServiceProxy: The service proxy. To send a request, call this instance:
            `my_service(my_resquest)`.
    """
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
            err_msg = "Timeout exceded, no service %s found" % name
            rospy.logerr(err_msg)
            raise TimeoutError(err_msg)
        else:
            rospy.logdebug("Service %s ready" % name)
            return srv


def send_request(srv: Service, req: Optional[Request] = None) -> Response:
    """Send a request to a service and return the answer.

    Args:
        srv (Callable[[Message], Message]): The service.
        req (Message, optional): The request. Defaults to `None`.

    Raises:
        Exception: If service call fails

    Returns:
        Message: The answer.
    """
    rospy.logdebug('Calling service %s' % srv.resolved_name)
    if req:
        ans = srv(req)
    else:
        ans = srv()
    # if answer does not have success attribut, consider it is a success
    try:
        success = ans.success
        status_message = ans.status_message
    except AttributeError:
        success = True
        status_message = 'no status message'

    if not success:
        rospy.logerr(status_message)
        raise Exception(status_message)
    else:
        rospy.logdebug("Calling to service %s succeed, %s" %
                       (srv.resolved_name, status_message))
        return ans


def _parse_args(paused, use_sim_time, extra_gazebo_args, gui,
                recording, debug, physics, verbose, output, world_name,
                respawn_gazebo, use_clock_frequency, pub_clock_frequency,
                enable_ros_network, server_required, gui_required):
    """Add the args to command line arguments passed to the Python script."""
    sys.argv.append('paused:={}'.format(str(paused).lower()))
    sys.argv.append('use_sim_time:={}'.format(str(use_sim_time).lower()))
    sys.argv.append('extra_gazebo_args:={}'.format(extra_gazebo_args))
    sys.argv.append('gui:={}'.format(str(gui).lower()))
    sys.argv.append('recording:={}'.format(str(recording).lower()))
    sys.argv.append('debug:={}'.format(str(debug).lower()))
    sys.argv.append('physics:={}'.format(physics))
    sys.argv.append('verbose:={}'.format(str(verbose).lower()))
    sys.argv.append('output:={}'.format(output))
    sys.argv.append('world_name:={}'.format(world_name))
    sys.argv.append('respawn_gazebo:={}'.format(str(respawn_gazebo).lower()))
    sys.argv.append('use_clock_frequency:={}'.format(
        str(use_clock_frequency).lower()))
    sys.argv.append('pub_clock_frequency:={}'.format(pub_clock_frequency))
    sys.argv.append('enable_ros_network:={}'.format(
        str(enable_ros_network).lower()))
    sys.argv.append('server_required:={}'.format(str(server_required).lower()))
    sys.argv.append('gui_required:={}'.format(str(gui_required).lower()))


def _state_to_dict(state, i):
    """Helper function that turn state object in dict object."""
    return {
        'postion': [state.pose[i].position.x,
                    state.pose[i].position.y,
                    state.pose[i].position.z],
        'orientation': [state.pose[i].orientation.x,
                        state.pose[i].orientation.y,
                        state.pose[i].orientation.z,
                        state.pose[i].orientation.w],
        'linear_velocity': [state.twist[i].linear.x,
                            state.twist[i].linear.y,
                            state.twist[i].linear.z],
        'angular_velocity': [state.twist[i].angular.x,
                             state.twist[i].angular.y,
                             state.twist[i].angular.z]}


def _spawn_model(
        model_name: str, model_xml: str, robot_namespace: str,
        initial_position: Position,
        initial_orientation: Tuple[float, float, float, float],
        srv: Service, reference_frame: str) -> None:
    """Helper that avoid duplication between `spawn_urdf` and `spawn_sdf`."""
    req = SpawnModelRequest()
    req.model_name = model_name
    req.model_xml = model_xml
    req.robot_namespace = robot_namespace
    req.initial_pose.position.x = initial_position[0]
    req.initial_pose.position.y = initial_position[1]
    req.initial_pose.position.z = initial_position[2]
    req.initial_pose.orientation.x = initial_orientation[0]
    req.initial_pose.orientation.y = initial_orientation[1]
    req.initial_pose.orientation.z = initial_orientation[2]
    req.initial_pose.orientation.w = initial_orientation[3]
    req.reference_frame = reference_frame
    send_request(srv, req)


def run_roscore():
    """Run a roscore, after verifying no roscore is running"""
    if rosgraph.is_master_online():
        raise Exception('A master is already running.')
    uuid = roslaunch.rlutil.get_or_generate_uuid(
        options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [], True)
    launch.start()
    return launch


class _GazeboROS(object):
    """Class containing a user-friendly functions for `gazebo_ros` services.

    Args:
        is_core (bool): Set `False` if a rosocre is already running.
            If `False`, but no core is running, it starts one.
        paused (bool): Start Gazebo in a paused state.
        use_sim_time (bool): Use Gazebo time instead of real time.
        extra_gazebo_args (str): Extra args. Defaults to `''`.
        gui (bool): Launch the user interface window of Gazebo.
        recording (bool): Enable Gazebo state log recording. Previously called
            `headless`.
        debug (bool): Start `gzserver` (Gazebo Server) in debug mode
            using `gdb`.
        physics (str): Specify a physics engine.
            (`'ode'`|`'bullet'`|`'dart'`|`'simbody'`).
        verbose (bool): Run `gzserver` and `gzclient` with in verbose
            mode : printing errors and warnings to the terminal.
        output (str): (`'log'`|`'screen'`)
            If `'screen'`, stdout/stderr is sent to the screen
            If `'log'`, the stdout/stderr is sent to a log file in
            `$ROS_HOME/log`, and stderr continue to be sent to screen.
        world_name (str): The world name with respect to
            `GAZEBO_RESOURCE_PATH` environmental variable.
        respawn_gazebo (bool): Restart the Gazebo node automatically
            if it quits.
        use_clock_frequency (bool): Whether you modify Gazebo's
            `/clock` frequency (default to 1000 Hz); if `True`, set new clock
            frequency with `pub_clock_frequency` arg.
        pub_clock_frequency (int): Set Gazebo’s `/clock` publish
            frequency (Hz). Requires `use_clock_frequency` to be `True`.
        enable_ros_network (bool): If `False`, disable all the Gazebo
            ROS topics (except `/clock`) and services that are created from the
            `gazebo_ros` package. Beware, by choosing `False`, you will no
            longer be able to use most of the methods in this class.
        server_required (bool): Terminate launch script when
            `gzserver` Gazebo Server exits.
        gui_required (bool): Terminate launch script when `gzclient`
            (user interface window) exits.
    """

    _JOINT_TYPE = [
        'REVOLUTE',    # 1 DOF (rotation)
        'CONTINUOUS',  # 1 DOF (rotation but limited range)
        'PRISMATIC',   # 1 DOF (translation)
        'FIXED',       # 0 DOF
        'BALL',        # 3 DOF (3 rotations)
        'UNIVERSAL'    # 2 DOF (2 rotations)
    ]

    def __init__(
            self, is_core: bool, paused: bool, use_sim_time: bool,
            extra_gazebo_args: str, gui: bool, recording: bool, debug: bool,
            physics: str, verbose: bool, output: str, world_name: str,
            respawn_gazebo: bool, use_clock_frequency: bool,
            pub_clock_frequency: int, enable_ros_network: bool,
            server_required: bool, gui_required: bool):
        self.link_states = {}
        self.model_states = {}
        self._subscribers = []

        if not enable_ros_network:
            rospy.logwarn("`enable_ros_network` is set to False: GazeboROS() \
                           instance won't start")

        # currently, I haven't found a cleaner way to give args to empty_world
        # than to add args to sys.argv
        _parse_args(
            paused, use_sim_time, extra_gazebo_args, gui, recording, debug,
            physics, verbose, output, world_name, respawn_gazebo,
            use_clock_frequency, pub_clock_frequency, enable_ros_network,
            server_required, gui_required)

        if is_core:
            self.core = run_roscore()
        else:
            self.core = None

        # launch world (and roscore)
        self._launch_world()

        # init this node
        rospy.init_node('pygazebo_node', anonymous=True, log_level=rospy.ERROR)

        # connect services and subscribers
        self._connect_services()
        self._connect_topics()

        rospy.on_shutdown(self.shutdown)

        # --- Init methods ---
    # For some parameters it is convenient to use getters and setters.
    # Should not be used with parameters that could change without user
    # action (example: sim_time).
    # region

    def _launch_world(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(
            options_runid=None, options_wait_for_master=True)
        roslaunch.configure_logging(self.uuid)
        gazebo_ros_path = rospkg.RosPack().get_path('gazebo_ros')
        launch_path = os.path.join(
            gazebo_ros_path, 'launch/empty_world.launch')
        self._launch = roslaunch.parent.ROSLaunchParent(
            self.uuid, [launch_path], False)
        self._launch.start()

    def _connect_services(self, timeout=None):
        """Connects to all available gazebo_ros services."""
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
            '/gazebo/get_loggers', GetLoggers, timeout)
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
            '/gazebo/set_logger_level', SetLoggerLevel, timeout)
        # /gazebo/set_model_configuration
        self._set_model_configuration_srv = service(
            '/gazebo/set_model_configuration', SetModelConfiguration, timeout)
        # /gazebo/set_model_state
        self._set_model_state_srv = service(
            '/gazebo/set_model_state', SetModelState, timeout)
        # /gazebo/set_parameters
        self._set_parameters_srv = service(
            '/gazebo/set_parameters', Reconfigure, timeout)
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
        """Subscribe to every gazebo_ros topics."""
        # /gazebo/link_states
        self._link_states_sub = subscribe(
            '/gazebo/link_states', LinkStates,
            self._link_states_callback, timeout=0)
        self._subscribers.append(self._link_states_sub)
        # /gazebo/model_states
        self._model_states_sub = subscribe(
            '/gazebo/model_states', ModelStates,
            self._model_states_callback, timeout)
        self._subscribers.append(self._model_states_sub)
        # /gazebo/parameter_descriptions
        self._parameter_descriptions_sub = subscribe(
            '/gazebo/parameter_descriptions', ConfigDescription,
            self._parameter_descriptions_callback, timeout)
        self._subscribers.append(self._parameter_descriptions_sub)
        # /gazebo/parameter_updates
        self._parameter_updates_sub = subscribe(
            '/gazebo/parameter_updates', Config,
            self._parameter_updates_callback, timeout)
        self._subscribers.append(self._parameter_updates_sub)
        # /gazebo/set_link_state
        self._set_link_state_sub = subscribe(
            '/gazebo/set_link_state', LinkState,
            self._set_link_state_callback, timeout=0)
        self._subscribers.append(self._set_link_state_sub)
        # /gazebo/set_model_state
        self._set_model_state_sub = subscribe(
            '/gazebo/set_model_state', ModelState,
            self._set_model_state_callback, timeout=0)
        self._subscribers.append(self._set_model_state_sub)

        rospy.loginfo('All Gazebo subscribers ready')

    # endregion

    # -- Topic callbacks --
    # Every Gazebo topic is subscribed. Each time a message is received on a
    # topic, the eponym callback is called.
    # region

    def _link_states_callback(self, value):
        nb_links = len(value.name)
        for i in range(nb_links):
            self.link_states[value.name[i]] = _state_to_dict(value, i)

    def _model_states_callback(self, value):
        nb_models = len(value.name)
        for i in range(nb_models):
            self.model_states[value.name[i]] = _state_to_dict(value, i)

    def _parameter_descriptions_callback(self, value):
        # NOTE: it seems like Gazebo subscribe to this this topic instead of
        # publishing. If it is the case, this callback is not necessary.
        # a message is published once when Gazebo start.
        # I should look for the publisher.
        rospy.logwarn_once('parameter_descriptions_callback not implemented')

    def _parameter_updates_callback(self, value):
        # NOTE: it seems like Gazebo subscribe to this this topic instead of
        # publishing. If it is the case, this callback is not necessary.
        # a message is published once when Gazebo start.
        # I should look for the publisher.
        rospy.logwarn_once('parameter_updates_callback not implemented')

    def _set_link_state_callback(self, value):
        # NOTE: it seems like Gazebo subscribe to this this topic instead of
        # publishing. If it is the case, this callback is not necessary.
        # a message is published once when Gazebo start.
        # I should look for the publisher.
        rospy.logwarn_once('set_link_state_callback not implemented')

    def _set_model_state_callback(self, value):
        # NOTE: it seems like Gazebo subscribe to this this topic instead of
        # publishing. If it is the case, this callback is not necessary.
        # a message is published once when Gazebo start.
        # I should look for the publisher.
        rospy.logwarn_once('set_model_state_callback not implemented')

    # endregion

    # -- Services methods --
    # In this section, there is one method per ROS service.
    #  - The arguments are exactly the same as those of the ROS service.
    #  - No optional arguments.
    #  - Arguments must be standard (floats, integers, dictionaries, strings,
    #    lists).
    #  - These methods convert the arguments to a Request object and then call
    #    the service with send_request().
    #  - If needed, the answer is converted into a Dict or List[Dict]
    #  - If the call fails, it raises an Exception.
    # region

    def apply_body_wrench(
            self, body_name: str, reference_point: Tuple[float, float, float],
            force: Force, torque: Torque, start_time_secs: int,
            start_time_nsecs: int, duration_secs: int, duration_nsecs: int,
            reference_frame: str) -> None:
        """Apply wrench (force and torque) to a link.

        Args:
            body_name (str): The name of the body, respecting the format
                `'<model_name>::<link_name>'`.
            reference_point (Tuple[float, float, float], optional): Wrench is
                defined at this location in the reference frame. Use
                `(0, 0, 0)` to apply at the center of mass. Default
                `(0, 0, 0)`.
            force (Tuple[float, float, float]): Force applied to the body as
                (fx, fy, fz).
            torque (Tuple[float, float, float]): Torque applied to the body as
                (cx, cy, cz).
            start_time_secs (int): Wrench application start time
                (in seconds).
                Seconds portion of wrench application start time:
                `start_time = start_time_secs + start_time_nsecs/10e9`. If
                `start_time` < current time, start as soon as possible.
            start_time_nsecs (int): Nano seconds portion of wrench application
                start time. For more details, see `start_time_secs`.
            duration_secs (int): Seconds portion of wrench application
                duration: `duration = duration_secs + duration_nsecs/10e9`
                If `duration` < 0, apply wrench continuously.
            duration_nsecs (int): Nano seconds portion of wrench application
                duration. For more details, see `duration_secs` arg.
            reference_frame (str): Wrench is defined in the reference
                frame of this entity. Use inertial frame if left empty.
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
        req.duration.secs = duration_secs
        req.duration.nsecs = duration_nsecs
        send_request(self._apply_body_wrench_srv, req)

    def apply_joint_effort(
            self, joint_name: str, effort: float, start_time_secs: int,
            start_time_nsecs: int, duration_secs: int,
            duration_nsecs: int) -> None:
        """Set joint effort.

        Args:
            joint_name (str): Joint to apply wrench (linear force and torque),
                respecting the format `'<model_name>::<joint_name>'`.
            effort (float): Effort to apply.
            start_time_secs (int): Seconds portion of wrench application start
                time: `start_time = start_time_secs + start_time_nsecs/1e9`.
                If `start_time` < current time, start as soon as possible.
            start_time_nsecs (int): Nano seconds portion of wrench application
                start time. For more details, see `start_time_secs` arg.
            duration_secs (int): Seconds portion of wrench application 
                duration: `duration = duration_secs + duration_nsecs/10e9`.
                If `duration` < 0, apply wrench continuously without end.
                If `duration` = 0, do nothing.
                If `duration` < step size, assume step size.
            duration_nsecs (int): Nano seconds portion of wrench application
                duration. For more details see `duration_secs` arg.

        Warning:
            You should not use this method on joints revolute2, universal,
            fixed and ball.
        """
        # After verification in the source code of gazebo_ros, you can't use
        # this srv to apply joint effort of 2nd axis of revolute2 and
        # universal. See details in source code :
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/a63566be22361fa1f02ebcca4a9857d233e1c2ac/gazebo_ros/include/gazebo_ros/gazebo_ros_api_plugin.h#L413
        req = ApplyJointEffortRequest()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time.secs = start_time_secs
        req.start_time.nsecs = start_time_nsecs
        req.duration.secs = duration_secs
        req.duration.nsecs = duration_nsecs
        send_request(self._apply_joint_effort_srv, req)

    def clear_body_wrenches(self, body_name: str) -> None:
        """Clear body wrenches.

        Args:
            body_name (str): The body name, respecting the format
            `'<model_name>::<link_name>'`.
        """
        req = BodyRequestRequest()
        req.body_name = body_name
        send_request(self._clear_body_wrenches_srv, req)

    def clear_joint_forces(self, joint_name: str) -> None:
        """Clear joint forces.

        Args:
            joint_name (str): The joint name, respecting the format
                `'<model_name>::<joint_name>'`.
        """
        req = JointRequestRequest()
        req.joint_name = joint_name
        send_request(self._clear_joint_forces_srv, req)

    def delete_light(self, light_name: str) -> None:
        """Delete a light.

        Args:
            light_name (str): Name of the light to be deleted.
        """
        req = DeleteLightRequest()
        req.light_name = light_name
        send_request(self._delete_light_srv, req)

    def delete_model(self, model_name: str) -> None:
        """Delete a model.

        Args:
            model_name (str): Name of the model to be deleted.
        """
        req = DeleteModelRequest()
        req.model_name = model_name
        send_request(self._delete_model_srv, req)

    def get_joint_properties(self, joint_name: str) -> Dict:
        """Get joint properties.

        Args:
            joint_name (str): The joint name, respecting the format
                `'<model_name>::<joint_name>'`.

        Returns:
            Dict:
                'joint_type' (str): (`'FIXED'`|`'REVOLUTE'`|`'REVOLUTE2'`|
                    `'PRISMATIC'`|`'SCREW'`|`'UNIVERSAL'`|`'BALL'`|`'GEAR'`)
                'position' (List[float]): Positions. The number of elements
                    depends on the joint type.
                'rate' (List[float]): Velocities. Number of elements depends
                    on the joint type.

        Warning:
            For the moment, I can't figure out why this method always returns
            a joint type `'REVOLUTE'`, and the number of elements of `position`
            and `rate` are always 1 (even for multiple dof joints). As far as
            I know, it comes from the service.
        """
        req = GetJointPropertiesRequest()
        req.joint_name = joint_name
        ans = send_request(self._get_joint_properties_srv, req)
        out = {
            'joint_type': self._JOINT_TYPE[ans.type],
            'damping': list(ans.damping),
            'position': list(ans.position),
            'rate': list(ans.rate)}
        return out

    def get_light_properties(self, light_name: str) -> Dict:
        """Get light properties.

        Args:
            light_name (str): Name of Gazebo Light

        Returns:
            Dict:
                'diffuse' (Tuple[float, float, float, float]): Diffuse color as 
                    (r, g, b, a).
                'attenuation' (Tuple[float, float, float]): Attenuation as
                    (constant, linear, quadratic).
        """
        req = GetLightPropertiesRequest()
        req.light_name = light_name
        ans = send_request(self._get_light_properties_srv, req)
        out = {
            'diffuse': (ans.diffuse.r,
                        ans.diffuse.g,
                        ans.diffuse.b,
                        ans.diffuse.a),
            'attenuation': (ans.attenuation_constant,
                            ans.attenuation_linear,
                            ans.attenuation_quadratic)}
        return out

    def get_link_properties(self, link_name: str) -> Dict:
        """Get link properties.

        Args:
            link_name (str): The link name, respecting the format
                `'<model_name>::<link_name>'`.

        Returns:
            Dict:
                'position' (Tuple[float, float, float]): Position as (x, y, z).
                'orientation' (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w).
                'gravity_mode' (bool): Whether the link is subject to gravity.
                'mass' (float): Mass of link.
                'moments' (Tuple[float, float, float, float, float, float]):
                    Components of the inertia matrix as
                    (ixx, ixy, ixz, iyy, iyz, izz).
        """
        req = GetLinkPropertiesRequest()
        req.link_name = link_name

        ans = send_request(self._get_link_properties_srv, req)
        out = {
            'position': (ans.com.position.x,
                         ans.com.position.y,
                         ans.com.position.z),
            'orientation': (ans.com.orientation.x,
                            ans.com.orientation.y,
                            ans.com.orientation.z,
                            ans.com.orientation.w),
            'gravity_mode': ans.gravity_mode,
            'mass': ans.mass,
            'moments': (ans.ixx, ans.ixy, ans.ixz, ans.iyy, ans.iyz, ans.izz)}
        return out

    def get_link_state(self, link_name: str, reference_frame: str) -> Dict:
        """Get link state (position and velocities).

        Args:
            link_name (str): The link name, respecting the format
                `'<model_name>::<link_name>'`.
            reference_frame (str): Pose/twist relative to the frame of
                this link/body; leave empty or `'world'` or `'map'`.

        Returns:
            Dict:
                'link_name' (str):The link name, respecting the format
                    `'<model_name>::<link_name>'`.
                'position' (Tuple[float, float, float]): Position as
                    (x, y, z).
                'orientation' (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w).
                'linear_velocity' (Tuple[float, float, float]): Linear
                    velocity as (vx, vy, vz).
                'angular_velocity' (Tuple[float, float, float]): Angular
                    velocity as (rx, ry, rz).
                'reference_frame' (str): Pose/twist relative to the frame of
                    this link/bodyL leave empty or `'world'` or `'map'`.
        """
        req = GetLinkStateRequest()
        req.link_name = link_name
        req.reference_frame = reference_frame
        ans = send_request(self._get_link_state_srv, req)
        out = {
            'link_name': ans.link_state.link_name,
            'position': (ans.link_state.pose.position.x,
                         ans.link_state.pose.position.y,
                         ans.link_state.pose.position.z),
            'orientation': (ans.link_state.pose.orientation.x,
                            ans.link_state.pose.orientation.y,
                            ans.link_state.pose.orientation.z,
                            ans.link_state.pose.orientation.w),
            'linear_velocity': (ans.link_state.twist.linear.x,
                                ans.link_state.twist.linear.y,
                                ans.link_state.twist.linear.z),
            'angular_velocity': (ans.link_state.twist.angular.x,
                                 ans.link_state.twist.angular.y,
                                 ans.link_state.twist.angular.z),
            'reference_frame': ans.link_state.reference_frame}
        return out

    def get_loggers(self) -> Dict:
        """Get list of loggers.

        Returns:
            Dict:
                keys: Logger names.
                values: Dict containing
                    'level' (str) : Logger level ('`debug`'|'`info`'|'`warn`'|
                        '`error`'|'`fatal`')
        """
        # TODO: make this better.
        req = GetLoggersRequest()
        ans = send_request(self._get_loggers_srv, req)
        out = {}
        for item in ans.loggers:
            out[item.name] = {'level': item.level}
        return out

    def get_model_properties(self, model_name: str) -> Dict:
        """Get model properties.

        Args:
            model_name (str): Name of the model.

        Returns:
            Dict:
                'parent_model_name' (str): Name of the parent model. If no
                    parent model : `''`.
                'canonical_body_name' (str): Name of canonical body.
                'body_names' List[str]: List of bodies.
                'geom_names' List[str]: List of collision names.
                'joint_names'  List[str]: List of joints attached to the model.
                'child_model_names' List[str]: List of child models.
                'is_static' (bool): Whether model is static.

        Warning:
            Canonical body is not returned due to gazebo_ros limitation.
            It returns always `''`.
        """
        req = GetModelPropertiesRequest()
        req.model_name = model_name
        ans = send_request(self._get_model_properties_srv, req)
        out = {
            'parent_model_name': ans.parent_model_name,
            'canonical_body_name': ans.canonical_body_name,
            'body_names': ans.body_names,
            'geom_names': ans.geom_names,
            'joint_names': ans.joint_names,
            'child_model_names': ans.child_model_names,
            'is_static': ans.is_static}
        return out

    def get_model_state(self, model_name: str,
                        relative_entity_name: str) -> Dict:
        """Get model state (state of the canonical link).

        Args:
            model_name (str): Name of the model.
            relative_entity_name (str): Return pose and twist relative to this
                entity. This entity can be a model, body, or geom respecting
                    the format `'<model_name>::<link_name>'`. Leave empty or
                    `'world'` to use inertial world frame.

        Returns:
            Dict:
                'seq' (int): Holds the number of requests since the plugin
                    started.
                'secs' (int): Seconds (`stamp_secs`) since pose.
                'nsecs' (int): Nanoseconds since `stamp_secs`.
                'frame_id' (str): Not used but currently filled with the
                    `relative_entity_name`.
                'position' (Tuple[float, float, float]): Position as (x, y, z)
                    in relative entity frame.
                'orientation' (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w) in relative entity frame.
                'linear_velocity' (Tuple[float, float, float]): Linear
                    velocity as (vx, vy, vz) in relative entity frame.
                'angular_velocity' (Tuple[float, float, float]): Angular
                    velocity as (rx, ry, rz) in relative entity frame.
        """
        req = GetModelStateRequest()
        req.model_name = model_name
        req.relative_entity_name = relative_entity_name
        ans = send_request(self._get_model_state_srv, req)
        out = {
            'seq': ans.header.seq,
            'secs': ans.header.stamp.secs,
            'nsecs': ans.header.stamp.nsecs,
            'frame_id': ans.header.frame_id,
            'position': [ans.pose.position.x,
                         ans.pose.position.y,
                         ans.pose.position.z],
            'orientation': [ans.pose.orientation.x,
                            ans.pose.orientation.y,
                            ans.pose.orientation.z,
                            ans.pose.orientation.w],
            'linear_velocity': [ans.twist.linear.x,
                                ans.twist.linear.y,
                                ans.twist.linear.z],
            'angular_velocity': [ans.twist.angular.x,
                                 ans.twist.angular.y,
                                 ans.twist.angular.z]}
        return out

    def get_physics_properties(self) -> Dict:
        """Get physics properties.

        Returns:
            Dict:
                'time_step' (float): Timestep in seconds.
                'pause' (bool): `True` if physics engine is paused
                'max_update_rate' (float): Throttle maximum physics update rate
                'gravity' (Tuple[float, float, float]): Gravity vector as 
                    (fx, fy, fz). E.g. earth ~`(0.0, 0.0, -9.8)`.
                'auto_disable_bodies' (bool): Enable auto disabling of bodies.
                'sor_pgs_precon_iters' (int): Preconditioning inner iterations
                    when uisng projected Gauss Seidel.
                'sor_pgs_iters' (int): Inner iterations when uisng projected
                    Gauss Seidel.
                'sor_pgs_w' (float): Relaxation parameter when using projected
                    Gauss Seidel, 1 = no relaxation.
                'sor_pgs_rms_error_tol' (float): RMS error tolerance before
                    stopping inner iterations..
                'contact_surface_layer' (float): Contact "dead-band" width.
                'contact_max_correcting_vel' (float): Contact maximum
                    correction velocity.
                'cfm' (float): Global constraint force mixing.
                'erp' (float): Global error reduction parameter.
                'max_contacts' (int): Maximum contact joints between two geoms.
        """
        ans = send_request(self._get_physics_properties_srv)
        ode = ans.ode_config  # to shorten the following
        physics = {
            'time_step': ans.time_step,
            'pause': ans.pause,
            'max_update_rate': ans.max_update_rate,
            'gravity': (ans.gravity.x, ans.gravity.y, ans.gravity.z),
            'auto_disable_bodies': ode.auto_disable_bodies,
            'sor_pgs_precon_iters': ode.sor_pgs_precon_iters,
            'sor_pgs_iters': ode.sor_pgs_iters,
            'sor_pgs_w': ode.sor_pgs_w,
            'sor_pgs_rms_error_tol': ode.sor_pgs_rms_error_tol,
            'contact_surface_layer': ode.contact_surface_layer,
            'contact_max_correcting_vel': ode.contact_max_correcting_vel,
            'cfm': ode.cfm,
            'erp': ode.erp,
            'max_contacts': ode.max_contacts}
        return physics

    def get_world_properties(self) -> Dict:
        """Get world properties.

        Returns:
            Dict:
                'sim_time (float): Current simulation time.
                'model_names' (List[str]): List of models in the world.
                'rendering_enabled' (bool): whether Gazebo rendering engine
                    is enabled, currently always `True`.
        """
        req = GetWorldPropertiesRequest()
        ans = send_request(self._get_world_properties_srv, req)
        out = {
            'sim_time': ans.sim_time,
            'model_names': ans.model_names.copy(),
            'rendering_enabled': ans.rendering_enabled}
        return out

    def pause_physics(self) -> None:
        """Pause the simulation."""
        send_request(self._pause_physics_srv)

    def reset_simulation(self) -> None:
        """Reset the simulation."""
        send_request(self._reset_simulation_srv)

    def reset_world(self) -> None:
        """Reset the world."""
        send_request(self._reset_world_srv)

    def set_joint_properties(
            self, joint_name: str, damping: List[float], hiStop: List[float],
            loStop: List[float], erp: List[float], cfm: List[float],
            stop_erp: List[float], stop_cfm: List[float],
            fudge_factor: List[float], fmax: List[float],
            vel: List[float]) -> None:
        """Set joint properties.

        Args are lists. The lenght and the units depends on the the joint type.

        Args:
            joint_name (str): The joint name, respecting the format
                `'<model_name>::<joint_name>'`.
            damping (List[float]): Joint damping.
            hiStop (List[float]): Joint limit.
            loStop (List[float]): Joint limit.
            erp (List[float]): Set joint erp.
            cfm (List[float]): Set joint cfm.
            stop_erp (List[float]): Set joint erp for joint limit "contact"
                joint.
            stop_cfm (List[float]): Set joint cfm for joint limit "contact" 
                joint.
            fudge_factor (List[float]): Joint `fudge_factor` applied at limits,
                see ODE manual for info.
            fmax (List[float]): ODE joint param `fmax`.
            vel (List[float]): ODE joint param `vel`.
        """
        req = SetJointPropertiesRequest()
        req.joint_name = joint_name
        req.ode_joint_config.damping = damping.copy()
        req.ode_joint_config.hiStop = hiStop.copy()
        req.ode_joint_config.loStop = loStop.copy()
        req.ode_joint_config.erp = erp.copy()
        req.ode_joint_config.cfm = cfm.copy()
        req.ode_joint_config.stop_erp = stop_erp.copy()
        req.ode_joint_config.stop_cfm = stop_cfm.copy()
        req.ode_joint_config.fudge_factor = fudge_factor.copy()
        req.ode_joint_config.fmax = fmax.copy()
        req.ode_joint_config.vel = vel.copy()
        send_request(self._set_joint_properties_srv, req)

    def set_light_properties(
            self, light_name: str, diffuse: Tuple[float, float, float, float],
            attenuation: Attenuation) -> None:
        """Set light properties.

        Args:
            light_name (str): Name of Gazebo Light.
            diffuse (Tuple[float, float, float, float]): Diffuse color as
                (red, green, blue, alpha).
            attenuation (Tuple[float, float, float]): Attenuation as
                (constant, linear, quadratic).
        """
        req = SetLightPropertiesRequest()
        req.light_name = light_name
        req.diffuse.r = diffuse[0]
        req.diffuse.g = diffuse[1]
        req.diffuse.b = diffuse[2]
        req.diffuse.a = diffuse[3]
        req.attenuation_constant = attenuation[0]
        req.attenuation_linear = attenuation[1]
        req.attenuation_quadratic = attenuation[2]
        send_request(self._set_light_properties_srv, req)

    def set_link_properties(
            self, link_name: str, position: Position,
            orientation: Orientation, gravity_mode: bool, mass: float,
            moments: Tuple[float, float, float, float, float, float]) -> None:
        """Set link properties.

        Args:
            link_name (str): The link name, respecting the format
                `'<model_name>::<link_name>'`.
            position (Tuple[float, float, float]): Position as (x, y, z).
            orientation (Tuple[float, float, float, float]): Orientation
                as quaternion (x, y, z, w).
            gravity_mode (bool): Whether the link is subject to gravity.
            mass (float): Mass of link.
            moments (Tuple[float, float, float, float, float, float]):
                Components of the inertia matrix as
                (ixx, ixy, ixz, iyy, iyz, izz).
        """
        req = SetLinkPropertiesRequest()
        req.link_name = link_name
        req.com.position.x = position[0]
        req.com.position.y = position[1]
        req.com.position.z = position[2]
        req.com.orientation.x = orientation[0]
        req.com.orientation.y = orientation[1]
        req.com.orientation.z = orientation[2]
        req.com.orientation.w = orientation[3]
        req.gravity_mode = gravity_mode
        req.mass = mass
        req.ixx = moments[0]
        req.ixy = moments[1]
        req.ixz = moments[2]
        req.iyy = moments[3]
        req.iyz = moments[4]
        req.izz = moments[5]
        send_request(self._set_link_properties_srv, req)

    def set_link_state(
            self, link_name: str, position: Position, orientation: Orientation,
            linear_velocity: Tuple[float, float, float],
            angular_velocity: Tuple[float, float, float],
            reference_frame: str) -> None:
        """Set link state (position and velocities).

        Args:
            link_name (str): The link name, respecting the format
                `'<model_name>::<link_name>'`.
            position (Tuple[float, float, float]): Desired position of the
                link as (x, y, z).
            orientation (Tuple[float, float, float, float]): Desire
                orientation of the link as quaternion(x, y, z, w).
            linear_velocity (Tuple[float, float, float]): Desired linear
                velocity of the link as (vx, vy, vz).
            angular_velocity (Tuple[float, float, float]): Desired angular
                velocity of the link as (rx, ry, rz).
            reference_frame (str): Pose/twist relative to the frame of
                this link/bodyL leave empty or `'world'` or `'map'`.
        Args:

        """
        req = SetLinkStateRequest()
        req.link_state.link_name = link_name
        req.link_state.pose.position.x = position[0]
        req.link_state.pose.position.y = position[1]
        req.link_state.pose.position.z = position[2]
        req.link_state.pose.orientation.x = orientation[0]
        req.link_state.pose.orientation.y = orientation[1]
        req.link_state.pose.orientation.z = orientation[2]
        req.link_state.pose.orientation.w = orientation[3]
        req.link_state.twist.linear.x = linear_velocity[0]
        req.link_state.twist.linear.y = linear_velocity[1]
        req.link_state.twist.linear.z = linear_velocity[2]
        req.link_state.twist.angular.x = angular_velocity[0]
        req.link_state.twist.angular.y = angular_velocity[1]
        req.link_state.twist.angular.z = angular_velocity[2]
        req.link_state.reference_frame = reference_frame
        send_request(self._set_link_state_srv, req)

    def set_logger_level(self, logger: str, level: str) -> None:
        """Set logger level.

        Args:
            logger (str): Logger name.
            level (str): Logger level 
                (`'debug'`|`'info'`|`'warn'`|`'error'`|`'fatal'`).
        """
        req = SetLoggerLevelRequest()
        req.logger = logger
        req.level = level
        send_request(self._set_logger_level_srv, req)

    def set_model_configuration(
            self, model_name: str, urdf_param_name: str,
            joint_names: List[str], joint_positions: List[float]) -> None:
        """Set model configuration.

        Args:
            model_name (str): Model to set state.
            urdf_param_name (str): Arg unused.
            joint_names (List[str]): List of joints to set positions. For
                joints in the model that are not listed here, the current
                position is preserved.
            joint_positions (List[float]): Joint positions. Units depends on
                joint types.
        """
        req = SetModelConfigurationRequest()
        req.model_name = model_name
        req.urdf_param_name = urdf_param_name
        req.joint_names = joint_names
        req.joint_positions = joint_positions
        send_request(self._set_model_configuration_srv, req)

    def set_model_state(
            self, model_name: str, position: Position, 
            orientation: Orientation,
            linear_velocity: Tuple[float, float, float],
            angular_velocity: Tuple[float, float, float],
            reference_frame: str) -> None:
        """Set Gazebo model position, orientation and velocities.

        Equivalent to set_link_state() on canonical link.

        Args:
            model_name (str): Model to set state.
            position (Tuple[float, float, float]): Desired position as
                (x, y, z).
            orientation (Tuple[float, float, float, float]): Desired
                orientation as quaternion
                (x, y, z, w).
            linear_velocity (Tuple[float, float, float]): Desired linear
                velocity as (vx, vy, vz).
            angular_velocity (Tuple[float, float, float]): Desired angular
                velocity as (rx, ry, rz).
            reference_frame (str): Pose/twist relative to the frame of
                this link/bodyL leave empty or `'world'` or `'map'`.
        """
        req = SetModelStateRequest()
        req.model_state.model_name = model_name
        req.model_state.pose.position.x = position[0]
        req.model_state.pose.position.y = position[1]
        req.model_state.pose.position.z = position[2]
        req.model_state.pose.orientation.x = orientation[0]
        req.model_state.pose.orientation.y = orientation[1]
        req.model_state.pose.orientation.z = orientation[2]
        req.model_state.pose.orientation.w = orientation[3]
        req.model_state.twist.linear.x = linear_velocity[0]
        req.model_state.twist.linear.y = linear_velocity[1]
        req.model_state.twist.linear.z = linear_velocity[2]
        req.model_state.twist.angular.x = angular_velocity[0]
        req.model_state.twist.angular.y = angular_velocity[1]
        req.model_state.twist.angular.z = angular_velocity[2]
        req.model_state.reference_frame = reference_frame
        send_request(self._set_model_state_srv, req)

    def set_parameters(
            self, bools: List[Dict],  ints: List[Dict], strs: List[Dict],
            doubles: List[Dict], groups: List[Dict]) -> Dict:
        """Set or get simulation parameters.

        Args:
            bools (List[Dict]): Booleans.
                e.g.  [{'name': 'auto_disable_bodies', 'value': False}]
            ints (List[Dict]): Integers.
                e.g.  [{'name': 'max_contact', 'value': 20}]
            strs (List[Dict]): Strings.
            doubles (List[Dict]): Doubles.
                e.g.  [{'name': 'time_step', 'value': 0.001}]
            groups (List[Dict]): Groups.
                e.g.  [{'name': 'Default', 'state': True,
                        'id': 0, 'parent': 0}]

        Returns:
            Current parameters as Dict:
                'bools' (List[Dict]): Booleans.
                    e.g.  {'name': 'auto_disable_bodies', 'value': False}
                'ints' (List[Dict]): Integers.
                    e.g.  {'name': 'max_contact', 'value': 20}
                'strs' (List[Dict]): Strings.
                'doubles' (List[Dict]): Doubles.
                    e.g.  {'name': 'time_step', 'value': 0.001}
                'groups' (List[Dict]): Groups.
                    e.g.  {'name': 'Default', 'state': True,
                         'id': 0, 'parent': 0}
        """
        req = ReconfigureRequest()
        for item in bools:
            param = BoolParameter()
            param.name = item['name']
            param.value = item['value']
            req.config.bools.append(param)
        for item in ints:
            param = IntParameter()
            param.name = item['name']
            param.value = item['value']
            req.config.ints.append(param)
        for item in strs:
            param = StrParameter()
            param.name = item['name']
            param.value = item['value']
            req.config.strs.append(param)
        for item in doubles:
            param = DoubleParameter()
            param.name = item['name']
            param.value = item['value']
            req.config.doubles.append(param)
        for item in groups:
            param = GroupState()
            param.name = item['name']
            param.state = item['state']
            param.id = item['id']
            param.parent = item['parent']
            req.config.groups.append(param)
        ans = send_request(self._set_parameters_srv, req)
        out = {
            'bools': [
                {'name': item.name, 'value': item.value}
                for item in ans.config.bools],
            'ints': [
                {'name': item.name, 'value': item.value}
                for item in ans.config.ints],
            'strs': [
                {'name': item.name, 'value': item.value}
                for item in ans.config.strs],
            'doubles': [
                {'name': item.name, 'value': item.value}
                for item in ans.config.doubles],
            'groups': [
                {'name': item.name, 'state': item.state,
                 'id': item.id, 'parent': item.parent}
                for item in ans.config.groups]}
        return out

    def set_physics_properties(
            self,  time_step: float, max_update_rate: float,
            gravity: Tuple[float, float, float], auto_disable_bodies: bool,
            sor_pgs_precon_iters: int, sor_pgs_iters: int, sor_pgs_w: float,
            sor_pgs_rms_error_tol: float, contact_surface_layer: float,
            contact_max_correcting_vel: float, cfm: float, erp: float,
            max_contacts: int) -> None:
        """Set physics properties.

        Args:
            time_step (float): Timestep in seconds.
            max_update_rate (float): Throttle maximum physics update rate
            gravity (Tuple[float, float, float]): Gravity vector as 
                (fx, fy, fz). E.g. earth ~`(0.0, 0.0, -9.8)`.
            auto_disable_bodies (bool): Enable auto disabling of bodies.
            sor_pgs_precon_iters (int): Preconditioning inner iterations when
                uisng projected Gauss Seidel.
            sor_pgs_iters (int): Inner iterations when uisng projected
                Gauss Seidel.
            sor_pgs_w (float): Relaxation parameter when using projected
                Gauss Seidel, 1 = no relaxation.
            sor_pgs_rms_error_tol (float): RMS error tolerance before
                stopping inner iterations..
            contact_surface_layer (float): Contact "dead-band" width.
            contact_max_correcting_vel (float): Contact maximum correction
                velocity.
            cfm (float): Global constraint force mixing.
            erp (float): Global error reduction parameter.
            max_contacts (int): Maximum contact joints between two geoms.
        """
        req = SetPhysicsPropertiesRequest()
        req.time_step = time_step
        req.max_update_rate = max_update_rate
        req.gravity.x = gravity[0]
        req.gravity.y = gravity[1]
        req.gravity.z = gravity[2]
        req.ode_config.auto_disable_bodies = auto_disable_bodies
        req.ode_config.sor_pgs_precon_iters = sor_pgs_precon_iters
        req.ode_config.sor_pgs_iters = sor_pgs_iters
        req.ode_config.sor_pgs_w = sor_pgs_w
        req.ode_config.sor_pgs_rms_error_tol = sor_pgs_rms_error_tol
        req.ode_config.contact_surface_layer = contact_surface_layer
        req.ode_config.contact_max_correcting_vel = contact_max_correcting_vel
        req.ode_config.cfm = cfm
        req.ode_config.erp = erp
        req.ode_config.max_contacts = max_contacts
        send_request(self._set_physics_properties_srv, req)

    def spawn_sdf_model(
            self, model_name: str, model_xml: str, robot_namespace: str,
            initial_position: Position, initial_orientation: Orientation,
            reference_frame: str) -> None:
        """Spawn a model from sdf-formated string.

        Args:
            model_name (str): Desired model name, must be unique in
                the simulation.
            model_xml (str): The model as sdf. See https://sdformat.org/
                for more details.
            robot_namespace (str): Spawn robot and all ROS interfaces under
                this namespace.
            initial_position (Tuple[float, float, float]): Position of
                canonical body as (x, y, z).
            initial_orientation (Tuple[float, float, float, float]): 
                Orientation of canonical body as quaternion (x, y, z, w).
            reference_frame (str): Pose/twist relative to the frame of
                this link/bodyL leave empty or `'world'` or `'map'`.
        """
        _spawn_model(
            model_name, model_xml, robot_namespace, initial_position,
            initial_orientation, self._spawn_sdf_model_srv, reference_frame)

    def spawn_urdf_model(
            self, model_name: str, model_xml: str, robot_namespace: str,
            initial_position: Position, initial_orientation: Orientation,
            reference_frame: str) -> None:
        """Spawn a model from urdf-formated string.

        Args:
            model_name (str): Desired model name, must be unique in
                the simulation.
            model_xml (str): The model as urdf.
            robot_namespace (str): Spawn robot and all ROS interfaces under
                this namespace.
            initial_position (Tuple[float, float, float]): Position of
                canonical body as (x, y, z).
            initial_orientation (Tuple[float, float, float, float]): 
                Orientation of canonical body as quaternion (x, y, z, w).
            reference_frame (str): Pose/twist relative to the frame of
                this link/bodyL leave empty or `'world'` or `'map'`.
        """
        _spawn_model(
            model_name, model_xml, robot_namespace, initial_position,
            initial_orientation, self._spawn_urdf_model_srv, reference_frame)

    def unpause_physics(self):
        """Unpause the simulation."""
        send_request(self._unpause_physics_srv)
    # endregion

    def shutdown(self):
        """Shutdown everything."""
        #rospy.signal_shutdown('shutdown invoked')
        for subscriber in self._subscribers:
            subscriber.unregister()
        self._launch.shutdown()
        if self.core is not None:
            self.core.shutdown()
