# -*- coding:utf:8 -*-

import time
from pygazebo_ros._gazeboo_ros import _GazeboROS
from contextlib import contextmanager
import warnings

from typing import List, Tuple, Optional, Dict

Position = Tuple[float, float, float]
Orientation = Tuple[float, float, float, float]
Force = Tuple[float, float, float]
Torque = Tuple[float, float, float]
Direction = Tuple[float, float, float]
Color = Tuple[float, float, float]
Attenuation = Tuple[float, float, float]


class GazeboROS(_GazeboROS):
    """Control Gazebo simulation."""

    def __init__(self):
        """Constructor."""
        super().__init__()
        self._fetch_physics_properties()

    @property
    def time_step(self) -> float:
        """Get or set the current time_step.

        Setting the time_step to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._time_step

    @time_step.setter
    def time_step(self, value: float) -> None:
        self._set_some_physics_properties(time_step=value)

    @property
    def paused(self):
        """Get or set the current paused.

        Setting the paused to a new value will reconfigure the gazebo
        automatically.
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
        """Get or set the current max_update_rate.

        Setting the max_update_rate to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._max_update_rate

    @max_update_rate.setter
    def max_update_rate(self, value: float) -> None:
        self._set_some_physics_properties(max_update_rate=value)

    @property
    def gravity(self) -> Tuple[float]:
        """Get or set the current gravity.

        Setting the gravity to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._gravity

    @gravity.setter
    def gravity(self, value: List[float]) -> None:
        self._set_some_physics_properties(gravity=value)

    @property
    def auto_disable_bodies(self):
        """Get the current auto_disable_bodies.

        The current version does not allow to set auto_disable_bodies.
        """
        self._fetch_physics_properties()
        return self._auto_disable_bodies

    @auto_disable_bodies.setter
    def auto_disable_bodies(self, value: bool) -> None:
        warnings.warn('The current version does not allow \
            to set auto_disable_bodies.', RuntimeWarning)
        self._set_some_physics_properties(auto_disable_bodies=value)

    @property
    def sor_pgs_precon_iters(self) -> int:
        """Get or set the current sor_pgs_precon_iters.

        Setting the sor_pgs_precon_iters to a new value will reconfigure the
        gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_precon_iters

    @sor_pgs_precon_iters.setter
    def sor_pgs_precon_iters(self, value: int) -> None:
        self._set_some_physics_properties(sor_pgs_precon_iters=value)

    @property
    def sor_pgs_iters(self) -> int:
        """Get or set the current sor_pgs_iters.

        Setting the sor_pgs_iters to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_iters

    @sor_pgs_iters.setter
    def sor_pgs_iters(self, value: int) -> None:
        self._set_some_physics_properties(sor_pgs_iters=value)

    @property
    def sor_pgs_w(self) -> float:
        """Get or set the current sor_pgs_w.

        Setting the sor_pgs_w to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_w

    @sor_pgs_w.setter
    def sor_pgs_w(self, value: float) -> None:
        self._set_some_physics_properties(sor_pgs_w=value)

    @property
    def sor_pgs_rms_error_tol(self) -> float:
        """Get or the current sor_pgs_rms_error_tol.

        The current version does not allow to set sor_pgs_rms_error_tol.
        """
        self._fetch_physics_properties()
        return self._sor_pgs_rms_error_tol

    @sor_pgs_rms_error_tol.setter
    def sor_pgs_rms_error_tol(self, value: float) -> None:
        warnings.warn('The current version does not allow \
            to set sor_pgs_rms_error_tol.', RuntimeWarning)
        self._set_some_physics_properties(sor_pgs_rms_error_tol=value)

    @property
    def contact_surface_layer(self) -> float:
        """Get or set the current contact_surface_layer.

        Setting the contact_surface_layer to a new value will reconfigure the
        gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._contact_surface_layer

    @contact_surface_layer.setter
    def contact_surface_layer(self, value: float) -> None:
        self._set_some_physics_properties(contact_surface_layer=value)

    @property
    def contact_max_correcting_vel(self) -> float:
        """Get or set the current contact_max_correcting_vel.

        Setting the contact_max_correcting_vel to a new value will reconfigure
        the gazebo automatically.
        """
        self._fetch_physics_properties()
        return self._contact_max_correcting_vel

    @contact_max_correcting_vel.setter
    def contact_max_correcting_vel(self, value: float) -> None:
        self._set_some_physics_properties(contact_max_correcting_vel=value)

    @property
    def cfm(self) -> float:
        """Get or set the current cfm.

        Setting the cfm to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._cfm

    @cfm.setter
    def cfm(self, value: float) -> None:
        self._set_some_physics_properties(cfm=value)

    @property
    def erp(self) -> float:
        """Get or set the current erp.

        Setting the erp to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._erp

    @erp.setter
    def erp(self, value: float) -> None:
        self._set_some_physics_properties(erp=value)

    @property
    def max_contacts(self) -> int:
        """Get or set the current max_contacts.

        Setting the max_contacts to a new value will reconfigure the gazebo
        automatically.
        """
        self._fetch_physics_properties()
        return self._max_contacts

    @max_contacts.setter
    def max_contacts(self, value: int) -> None:
        self._set_some_physics_properties(max_contacts=value)

    def apply_body_wrench(
            self, model_name: str, link_name: str, force: Force,
            torque: Torque, reference_point: Position = (0, 0, 0),
            start_time: float = 0, duration: float = -1.0) -> None:
        """Apply wrench (force and torque) to a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            force (Tuple[float, float, float]): Force applied to the body.
            torque (Tuple[float, float, float]): Torque applied to the body.
            reference_point (Tuple[float, float, float], optional): Wrench is
                defined at this location in the reference frame. Use
                `(0, 0, 0)` to apply at the center of mass. Default
                `(0, 0, 0)`.
            start_time (float, optional): Wrench application start time
                (in seconds).
                If `start_time` < current time, start as soon as possible.
                Defaults to `0.0`.
            duration (float, optionnal): The length of time (in seconds) the
                wrench is applied.
                If `duration` < 0, apply wrench continuously.
                Defaults to `-1.0`.
        """
        body_name = '{}::{}'.format(model_name, link_name)
        start_time_secs = int(start_time)
        start_time_nsecs = int((start_time-start_time_secs)*1e9)
        # Actually, duration connot take all negative values. Thus, if it is
        # negative, I just replace by a negative value that works.
        # See : https://answers.ros.org/question/209452/exception-thrown-while-processing-service-call-time-is-out-of-dual-32-bit-range/
        duration = - 1e-9 if duration < 0 else duration
        duration_secs = int(duration)
        duration_nsecs = int((duration-duration_secs)*1e9)
        reference_point = reference_point  # list is default arg: safer
        # For the moment I consider reference frame always = `''`
        super().apply_body_wrench(
            body_name, reference_point, force, torque, start_time_secs,
            start_time_nsecs, duration_secs, duration_nsecs,
            reference_frame='')

    def apply_body_force(
            self, model_name: str, link_name: str, force: Force,
            reference_point: Position = (0, 0, 0), start_time: float = 0,
            duration: float = -1.0) -> None:
        """Apply force to a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            force (Tuple[float, float, float]): Force applied to the body.
            reference_point (Tuple[float, float, float], optional): Force is
                defined at this location in the reference frame. Use
                `(0, 0, 0)` to apply at the center of mass. Default
                `(0, 0, 0)`.
            start_time (float, optional): Force application start time
                (in seconds).
                If `start_time` < current time, start as soon as possible.
                Defaults to `0.0`.
            duration (float, optionnal): The length of time (in seconds) the
                wrench is applied.
                If `duration` < 0, apply wrench continuously.
                Defaults to `-1.0`.
        """
        torque = (0, 0, 0)
        self.apply_body_wrench(
            model_name, link_name, force, torque, reference_point, start_time,
            duration)

    def apply_body_torque(
            self, model_name: str, link_name: str, torque: Torque,
            start_time: float = 0, duration: float = -1.0) -> None:
        """Apply torque to a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            torque (Tuple[float, float, float]): Torque applied to the body.
            start_time (float, optional): Torque application start time
                (in seconds).
                If `start_time` < current time, start as soon as possible.
                Defaults to `0.0`.
            duration (float, optionnal): The length of time (in seconds) the
                wrench is applied.
                If `duration` < 0, apply wrench continuously.
                Defaults to `-1.0`.
        """
        force = (0, 0, 0)
        # Why reference_point not in args ? Because changment of reference
        # point does not affect torque (see Varignon's theorem)
        reference_point = (0, 0, 0)
        # For the moment I consider reference frame always = `''`
        self.apply_body_wrench(
            model_name, link_name, force, torque, reference_point, start_time,
            duration)

    def apply_joint_effort(
            self, model_name: str, joint_name: str, effort: float,
            start_time: float = 0, duration: float = -1.0) -> None:
        """Apply effort to a joint.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.
            effort (float): Effort to apply. Unit depends on the joint type,
                can be Nm or N.
            start_time (float, optional): Effort application start time
                (in seconds).
                If `start_time` < current time, start as soon as possible.
                Defaults to `0.0`.
            duration (float, optionnal): The length of time (in seconds) the
                effort is applied.
                If `duration` < 0, apply effort continuously.
                Defaults to `-1.0`.
        """
        body_name = '{}::{}'.format(model_name, joint_name)
        start_time_secs = int(start_time)
        start_time_nsecs = int((start_time-start_time_secs)*1e9)
        # Actually, duration connot take all negative values. Thus, if it is
        # negative, I just replace by a negative value that works.
        # See : https://answers.ros.org/question/209452/exception-thrown-while-processing-service-call-time-is-out-of-dual-32-bit-range/
        duration = - 1e-9 if duration < 0 else duration
        duration_secs = int(duration)
        duration_nsecs = int((duration-duration_secs)*1e9)

        super().apply_joint_effort(
            body_name, effort, start_time_secs,
            start_time_nsecs, duration_secs,
            duration_nsecs)

    def clear_body_wrenches(self, model_name: str, link_name: str) -> None:
        """Clear body wrenches.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
        """
        body_name = '{}::{}'.format(model_name, link_name)
        super().clear_body_wrenches(body_name=body_name)

    def clear_joint_forces(self, model_name: str, joint_name: str) -> None:
        """Clear joint forces.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.
        """
        name = '{}::{}'.format(model_name, joint_name)
        super().clear_joint_forces(name)

    def delete_light(self, light_name: str) -> None:
        """Delete a light.

        Args:
            light_name (str): The name of the light.
        """
        super().delete_light(light_name)

    def delete_model(self, model_name: str) -> None:
        """Delete a model.

        Args:
            model_name (str): The name of the model.
        """
        super().delete_model(model_name)

    def get_joint_properties(self, model_name: str, joint_name: str) -> Dict:
        """Get joint properties (type, position and rate).

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.

        Returns:
            Dict:
                'type': (str) The joint type.
                'position': (List[float]) Position of the joint. The units and
                    the lenght of the list depends on the joint type.
                'rate': (List[float]) Velocity of the joint. The units and the
                    lenght of the list depends on the joint type.
        """
        name = '{}::{}'.format(model_name, joint_name)
        out = super().get_joint_properties(name)
        return out.copy()

    def get_joint_type(self, model_name: str, joint_name: str) -> str:
        """Get joint type.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.

        Returns:
            str: The joint type. It can be:
                - `'FIXED'`: A joint with zero degrees of freedom that rigidly
                    connects two links.
                - `'REVOLUTE'`: A hinge joint that rotates on a single axis
                    with a fixed range of motion.
                - `'REVOLUTE2'`: Same as two revolute joints connected in
                    serie.
                - `'PRISMATIC'`: A sliding joint that slides along an axis.
                - `'SCREW'`: A single degree of freedom joint with coupled
                    sliding and rotational motion, (universal) like a ball
                    joint, but constrains one degree of freedom.
                - `'UNIVERSAL'`: Like a ball joint, but constrains one degree
                    of freedom.
                - `'BALL'`: A ball and socket joint.
                - `'GEAR'`: Gear revolute joints.
        Warning:
            For the moment, I can't figure out why this method always returns
            a joint type `'REVOLUTE'`. As far as I know, it comes from the
            service which always return type: 0.
        """
        # TODO: solve the issue described in warnings.
        out = self.get_joint_properties(model_name, joint_name)
        return out['joint_type']

    def get_joint_position(self, model_name: str, joint_name: str) -> float:
        """Get joint position.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.

        Returns:
            List[float]: Position of the joint. The units and the lenght of
                the list depends on the joint type.

        Warning:
            This method does not work properly for all joints:
            OK:
                - `'FIXED'`: Do not apply, raise exception.
                - `'REVOLUTE'`: Returns the angular position in radians.
                - `'PRISMATIC'`: Returns the linear positon in meters.
                - `'SCREW'`: Returns the angular position in radians.
            NOT OK:
                - `'REVOLUTE2'`: Just return the angular position in the first
                    axis.
                - `'UNIVERSAL'`: Just return the angular position in the first
                    axis.
                - `'BALL'`: Does not work, unknown reason.
                - `'GEAR'`: Does not work, unknown reason.
        """
        out = self.get_joint_properties(model_name, joint_name)
        return out['position'][0]

    def get_joint_rate(self, model_name: str, joint_name: str) -> float:
        """Get joint velocity.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.

        Returns:
            List[float]: Velocitiy of the joint. The unit depends on the
                joint type.

        Warning:
            This method does not work properly for all joints:
            OK:
                - `'FIXED'`: Do not apply, raise exception.
                - `'REVOLUTE'`: Returns the angular velocity in radians/sec.
                - `'PRISMATIC'`: Returns the linear positon in meters/sec.
                - `'SCREW'`: Returns the angular velocity in radians/sec.
            NOT OK:
                - `'REVOLUTE2'`: Just return the angular velocity in the first
                    axis.
                - `'UNIVERSAL'`: Just return the angular velocity in the first
                    axis.
                - `'BALL'`: Does not work, unknown reason.
                - `'GEAR'`: Does not work, unknown reason.
        """
        out = self.get_joint_properties(model_name, joint_name)
        return out['rate'][0]

    def get_light_properties(self, light_name: str) -> Dict:
        """Get the light properties (color and attenuation).

        Args:
            light_name (str): Name of the light.

        Returns:
            Dict
                Tuple[float, float, float, float]: Color as
                    (red, green, blue, alpha), values are between 0 and 1.
                Tuple[float, float, float]: Attenuation as
                    (constant, linear, quadratic).
        """
        out = super().get_light_properties(light_name)
        # Replace `'diffuse'` by `'color'`
        return {
            'color': out['diffuse'],
            'attenuation': out['attenuation']}

    def get_light_color(self, light_name: str) -> Tuple:
        """Get the light color.

        Args:
            light_name (str): Name of the light.

        Returns:
            Tuple[float, float, float, float]: Color as
                (red, green, blue, alpha), values are between 0 and 1.
        """
        out = self.get_light_properties(light_name)
        return out['color']

    def get_light_attenuation(self, light_name: str) -> Tuple:
        """Get light attenuation.

        Args:
            light_name (str): Name of the light.

        Return:
            Tuple[float, float, float]: Attenuation as
                (constant, linear, quadratic).
        """
        out = self.get_light_properties(light_name)
        return out['attenuation']

    def get_link_properties(self, model_name: str, link_name: str) -> Dict:
        """Get the link properties (position, orientation)

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Dict:
                'position': (Tuple[float, float, float]): Position as
                    (x, y, z).
                'orientation': (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w).
                'gravity_mode' (bool): Whether the link is subject to gravity.
                'mass' (float): Mass of link.
                'moments' (Tuple[float, float, float, float, float, float]):
                    Components of the inertia matrix as
                    (ixx, ixy, ixz, iyy, iyz, izz).

        Warning:
            For the moment, I can't figure out why this method always returns
            a position of `(0, 0, 0)` and orientation of `(0, 0, 0, 1)`. As far
            as I know, it comes from the service which always return this
            value.
        """
        name = '{}::{}'.format(model_name, link_name)
        out = super().get_link_properties(name)
        return {
            'position': out['position'],
            'orientation': out['orientation'],
            'gravity_mode': out['gravity_mode'],
            'mass': out['mass'],
            'moments': out['moments']}

    def get_link_gravity_mode(self, model_name: str, link_name: str) -> bool:
        """Whether the link is subject to the gravity.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            bool: Whether the link is subject to the gravity.
        """
        return self.get_link_properties(model_name, link_name)['gravity_mode']

    def get_link_mass(self, model_name: str, link_name: str) -> float:
        """Get the mass of a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            float: The mass of the link.
        """
        return self.get_link_properties(model_name, link_name)['mass']

    def get_link_moments(
        self, model_name: str, link_name: str
    ) -> Tuple[float, float, float, float, float, float]:
        """Get the components of inertia matrix of a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Tuple[float, float, float, float, float, float]: Components of the
                inertia matrix as (ixx, ixy, ixz, iyy, iyz, izz).
        """
        return self.get_link_properties(model_name, link_name)['moments']

    def get_link_state(self, model_name: str, link_name: str) -> Dict:
        """Get the link state (position, orientation and velocity)

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Dict:
                'position': (Tuple[float, float, float]): Position as
                    (x, y, z).
                'orientation': (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w).
                'linear_velocity': (Tuple[float, float, float]): Linear
                    velocity as (vx, vy, vz).
                'angular_velocity': (Tuple[float, float, float]): Angular
                    velocity as (rx, ry, rz).
        """
        name = '{}::{}'.format(model_name, link_name)
        # So far the reference frame is left empty. I may add this
        # functionnality later.
        out = super().get_link_state(name, reference_frame='')
        return {
            'position': out['position'],
            'orientation': out['orientation'],
            'linear_velocity': out['linear_velocity'],
            'angular_velocity': out['angular_velocity']}

    def get_link_position(self, model_name: str, link_name: str) -> Position:
        """Get the link position.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Tuple[float, float, float]: Position as (x, y, z).

        """
        out = self.get_link_state(model_name, link_name)
        return out['position']

    def get_link_orientation(
            self, model_name: str, link_name: str) -> Orientation:
        """Get the link orienation.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Tuple[float, float, float, float]: Orientation as quaternion
                (x, y, z, w).
        """
        out = self.get_link_state(model_name, link_name)
        return out['orientation']

    def get_link_linear_velocity(
            self, model_name: str, link_name: str
    ) -> Tuple[float, float, float]:
        """Get the link linear velocity.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Tuple[float, float, float]: Linear velocity as (vx, vy, vz).
        """
        out = self.get_link_state(model_name, link_name)
        return out['linear_velocity']

    def get_link_angular_velocity(
            self, model_name: str, link_name: str
    ) -> Tuple[float, float, float]:
        """Get the link angular velocity.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            Tuple[float, float, float]: Angular velocity as (rx, ry, rz).
        """
        out = self.get_link_state(model_name, link_name)
        return out['angular_velocity']

    def get_loggers(self) -> Dict:
        """Get list of loggers.

        Returns:
            Dict:
                keys: logger names
                values: Dict containing
                    'level' (str) : logger level
                    (`'debug'`|`'info'`|`'warn'`|`'error'`|`'fatal'`)
        """
        # TODO: to be improved.
        return super().get_loggers()

    def get_model_properties(self, model_name: str) -> Dict:
        """Get model properties.

        Args:
            model_name (str): The name of the model.

        Returns:
            Dict:
                "parent_model_name" (str): The parent model name.
                "canonical_body_name" (str): name of canonical body, body names
                    are prefixed by model name, e.g. `model_name::link_name`
                "link_names" List[str]: list of bodies, body names are prefixed
                    by model name, e.g. ` model_name::link_name`
                "geom_names" List[str]: list of geoms
                "joint_names"  List[str]: list of joints attached to the model
                "child_model_names" List[str]: list of child models
                "is_static" (bool): if model is static
        """
        properties = super().get_model_properties(model_name)
        properties['canonical_link_name'] = properties.pop(
            'canonical_body_name')
        properties['link_names'] = properties.pop('body_names')
        return properties

    def get_parent_model_name(self, model_name: str) -> str:
        """Get the parent name of the model.

        Args:
            model_name (str): The name of the model.

        Returns:
            str: The parent model name. If no parent, return `''`.
        """
        return self.get_model_properties(model_name)['parent_model_name']

    def get_canonical_link_name(self, model_name: str) -> str:
        """Get canonical link name.

        The state of a model is the state of its canonical link.

        Args:
            model_name (str): The name of the model.

        Returns:
            str: The cannonical model name.

        Warnings:
            I don't know why this this method always return `''`, even when I
            set a connonical link manually in the sdf file.
        """
        # TODO: fix warning
        return self.get_model_properties(model_name)['canonical_link_name']

    def get_link_names(self, model_name: str) -> List[str]:
        """Get child link names of a model.

        Args:
            model_name (str): The name of the model.

        Returns:
            List[str]: List of link names.
        """
        return self.get_model_properties(model_name)['link_names']

    def get_geom_name(self, model_name: str, link_name: str) -> str:
        """Get the name of the collision geometry.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.

        Returns:
            str: The geometry name.
        """
        properties = self.get_model_properties(model_name)
        geom_names = properties['geom_names']
        link_names = properties['link_names']
        idx = link_names.index(link_name)
        return geom_names[idx]

    def get_joint_names(self, model_name: str) -> List[str]:
        """Get a list of joint names of a model.

        Args:
            model_name (str): The name of the model.

        Returns:
            List[str]: List of joint names.
        """
        return self.get_model_properties(model_name)['joint_names']

    def get_child_model_names(self, model_name: str) -> List[str]:
        """Get a list of child model names.

        Args:
            model_name (str): The name of the model.

        Returns:
            List[str]: List of child model names.
        """
        return self.get_model_properties(model_name)['child_model_names']

    def is_model_static(self, model_name: str) -> bool:
        """Return true if the model is static.

        Args:
            model_name (str): The name of the model.

        Returns:
            bool: True if the model is static.
        """
        return self.get_model_properties(model_name)['is_static']

    def get_model_state(self, model_name: str) -> Dict:
        """Get the state (position, orientation and velocity) of the canonical
        link of the body.

        Args:
            model_name (str): The name of the model.

        Returns:
            Dict:
                'position': (Tuple[float, float, float]): Position as
                    (x, y, z).
                'orientation': (Tuple[float, float, float, float]): Orientation
                    as quaternion (x, y, z, w).
                'linear_velocity': (Tuple[float, float, float]): Linear
                    velocity as (vx, vy, vz).
                'angular_velocity': (Tuple[float, float, float]): Angular
                    velocity as (rx, ry, rz).
        """
        # NOTE: I don't like to manipulate model, I'd prefer link. That's why I
        # won't code function like get_model_position. I would recommend to
        # users to use links instead of model
        # So far the reference frame is left empty. I may add this
        # functionnality later.
        relative_entity_name = ''
        out = super().get_model_state(model_name, relative_entity_name)
        # To be consistent with get_link_state:
        out = {
            'position': out['position'],
            'orientation': out['orientation'],
            'linear_velocity': out['linear_velocity'],
            'angular_velocity': out['angular_velocity']}
        return out

    def get_physics_properties(self) -> Dict:
        """Get physics properties.

        It is not recommended to change these settings. However, it is possible
        to do it using the associated setters. The corresponding property is
        automatically changed in Gazebo simulation.

            gazebo_ros = GazeboROS()
            gazebo_ros.time_step = 1e-2

        For more information about ODE physics, see the site
        https://www.ode.org/

        Returns:
            Dict:
                "time_step" (float): dt in seconds.
                "pause" (bool): `True` if physics engine is paused.
                "max_update_rate" (float): Throttle maximum physics update
                    rate.
                "gravity" (Tuple[float]): Gravity vector (e.g.
                    earth ~`[0.0, 0.0, -9.8]`)
                "auto_disable_bodies" (bool): Enable auto disabling of bodies.
                "sor_pgs_precon_iters" (int): Preconditioning inner iterations
                    when uisng projected Gauss Seidel.
                "sor_pgs_iters" (int): Inner iterations when uisng projected
                    Gauss Seidel
                "sor_pgs_w" (float): Relaxation parameter when using projected
                    Gauss Seidel, `1` = no relaxation.
                "sor_pgs_rms_error_tol" (float): rms error tolerance before
                    stopping inner iterations.
                "contact_surface_layer" (float): Contact "dead-band" width.
                "contact_max_correcting_vel" (float): Contact maximum
                    correction velocity.
                "cfm" (float): global Constraint force mixing.
                "erp" (float): global Error reduction parameter.
                "max_contacts" (int): Maximum contact joints between two geoms.
        """
        return super().get_physics_properties()

    def get_world_properties(self) -> Dict:
        """Get world properties.

        Returns:
            Dict:
                "sim_time" (float): Current simulation time.
                "model_names" (List[str]): List of models in the world.
                "rendering_enabled" (bool): Whether gazebo rendering engine
                    is enabled, currently always `True`.
        """
        return super().get_world_properties()

    def get_sim_time(self) -> float:
        """Get the current simulation time.

        Returns:
            float: Current simulation time (in seconds).
        """
        ans = self.get_world_properties()
        return ans['sim_time']

    def get_model_names(self) -> List[str]:
        """Get the model names in the simulation.

        Returns:
            List[str]: The list of model names.
        """
        ans = self.get_world_properties()
        return ans['model_names'].copy()

    def pause(self) -> None:
        """Pause the simulation."""
        while not self.paused:
            super().pause_physics()

    def reset_simulation(self) -> None:
        """Reset the entire simulation including the time.

        Warning:
            `reset_simulation()` corresponds to `Edit` > `Reset World` while
            `reset_world()` corresponds to `Edit` > `Reset Model poses`. Thus,
            if you just want to reset model poses, prefer `reset_world()`
        """
        super().reset_simulation()

    def reset_world(self) -> None:
        """Reset the model's poses.

        Warning:
            `reset_simulation()` corresponds to `Edit` > `Reset World` while
            `reset_world()` corresponds to `Edit` > `Reset Model poses`. Thus,
            if you just want to reset time as well, prefer
            `reset_simulation()`.
        """
        super().reset_world()

    def set_joint_properties(
            self, joint_name: str, damping: Optional[List[float]] = None,
            hiStop: Optional[List[float]] = None,
            loStop: Optional[List[float]] = None,
            erp: Optional[List[float]] = None,
            cfm: Optional[List[float]] = None,
            stop_erp: Optional[List[float]] = None,
            stop_cfm: Optional[List[float]] = None,
            fudge_factor: Optional[List[float]] = None,
            fmax: Optional[List[float]] = None,
            vel: Optional[List[float]] = None) -> None:
        """Set joint properties. Not implemented.

        Args are lists. The number of elements in the list and their units
        depend on the joint type. If `None`, keep the current value.

        Args:
            joint_name (str): The name of the joint
            damping (Optional[List[float]], optional): Damping values. Defaults
                to `None`.
            hiStop (Optional[List[float]], optional): Higher possible
                positions. Defaults to `None`.
            loStop (Optional[List[float]], optional): Lower possible position.
                Defaults to `None`.
            erp (Optional[List[float]], optional): Error reduction parameters.
                Defaults to None.
            cfm (Optional[List[float]], optional): Constraint force mixing.
                Defaults to `None`.
            stop_erp (Optional[List[float]], optional): Error reduction
                parameters for joint limit "contact" joint. Defaults to `None`.
            stop_cfm (Optional[List[float]], optional): Constraint force mixing
                for joint limit "contact" joint. Defaults to `None`.
            fudge_factor (Optional[List[float]], optional): Fudge factors
                applied at limits. Defaults to `None`.
            fmax (Optional[List[float]], optional): The maximum wrench that the
                motor will use to achieve the desired velocity. Defaults to
                `None`.
            vel (Optional[List[float]], optional): Desired motors velocity.
                Defaults to `None`.
        """
        # I want to create a function that allows to change some parameters,
        # leaving the others at their initial value (using `arg=None` in the
        # call). The ROS service requires to define all the new parameters in
        # a request. To be able to change a single joint parameter, it is
        # therefore necessary to import the current joint parameters, then
        # change the value you want to change and return the new setting.
        # However, it is not possible to import the joint parameters (see
        # function `get_joint_parameters` that return almost nothing). So I
        # decided not to code a function until I have found a way to import
        # these joint parameters.
        raise NotImplementedError('See the source code for more details')

    def set_light_properties(
            self, light_name: str, color: Color, attenuation: Attenuation
    ) -> None:
        """Set light properties.

        Args:
            light_name (str): Name of the light.
            color (Tuple[float, float, float]): Diffuse color as
                (red, green, blue, alpha).
            attenuation (Tuple[float, float, float]): Attenuation as
                (constant, linear, quadratic).
        """
        super().set_light_properties(light_name, color, attenuation)
        # FIXME: if no waiting, the light does not have the time to change.
        # thus, if you get the light prop just after, the values won't be
        # changed. Sleeping a bit helps.
        time.sleep(0.2)

    def set_light_color(self, light_name: str, color: Color) -> None:
        """Set light color.

        Args:
            light_name (str): Name of the light.
            color (Tuple[float, float, float]): Diffuse color as
                (red, green, blue, alpha).
        """
        attenuation = self.get_light_attenuation(light_name)
        self.set_light_properties(light_name, color, attenuation)

    def set_light_attenuation(
            self, light_name: str, attenuation: Attenuation) -> None:
        """Set light attenuation.

        Args:
            light_name (str): Name of the light.
            attenuation (Tuple[float, float, float]): Attenuation as
                (constant, linear, quadratic).
        """
        color = self.get_light_color(light_name)
        self.set_light_properties(light_name, color, attenuation)

    def set_link_properties(
            self, model_name: str, link_name: str, position: Position,
            orientation: Orientation, gravity_mode: bool, mass: float,
            moments: Tuple[float, float, float, float, float, float]) -> None:
        """Set link properties.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            position (Tuple[float, float, float]): Reference postion for
                inertial frame as (x, y, z) in the link frame.
            orientation: (Tuple[float, float, float, float]): Reference
                orientation of inertial frame as quaternion (x, y, z, w).
            gravity_mode (bool):  Whether the link is subject to gravity.
            mass (float): Mass of the link.
            moments (Tuple[float, float, float, float, float, float]):
                Components of the inertia matrix as
                (ixx, ixy, ixz, iyy, iyz, izz).
        """
        name = '{}::{}'.format(model_name, link_name)
        super().set_link_properties(name, position,
                                    orientation, gravity_mode, mass, moments)

    def set_link_inertia_position(
            self, model_name: str, link_name: str, position: Position) -> None:
        """Set the position of the inertial frame.

        Most of the time, the inertial position is the same as the position of
        the link. Unless you are sure, you should leave it at (0, 0, 0).

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            position (Position): Desired position as (x, y, z).
        """
        # TODO: figure out how this function affects the simulation. My various
        # tests give wierd results.
        properties = self.get_link_properties(model_name, link_name)
        orientation = properties['orientation']
        gravity_mode = properties['gravity_mode']
        mass = properties['mass']
        moments = properties['moments']
        self.set_link_properties(model_name, link_name, position, orientation,
                                 gravity_mode, mass, moments)

    def set_link_inertia_orientation(
            self, model_name: str, link_name: str,
            orientation: Orientation) -> None:
        """Set the orientation of inertial frame.

        Most of the time, the inertial orientation is the same as the
        orientation of the link. Unless you are sure, you should leave it
        at (0, 0, 0, 1).

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            orientation (Orientation): Desired orientation as quaternion
                (x, y, z, w).
        """
        # TODO: figure out how this function affects the simulation. My various
        # tests give wierd results.
        properties = self.get_link_properties(model_name, link_name)
        position = properties['position']
        gravity_mode = properties['gravity_mode']
        mass = properties['mass']
        moments = properties['moments']
        self.set_link_properties(model_name, link_name, position, orientation,
                                 gravity_mode, mass, moments)

    def set_link_gravity_mode(
            self, model_name: str, link_name: str,
            gravity_mode: bool) -> None:
        """Whether the link is subject to the gravity.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            gravity_mode (bool): Whether the link is subject to the gravity.
        """
        properties = self.get_link_properties(model_name, link_name)
        position = properties['position']
        orientation = properties['orientation']
        mass = properties['mass']
        moments = properties['moments']
        self.set_link_properties(model_name, link_name, position, orientation,
                                 gravity_mode, mass, moments)

    def set_link_mass(
            self, model_name: str, link_name: str,
            mass: float) -> None:
        """Set the mass of a link.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            mass (float): The mass of the link.
        """
        properties = self.get_link_properties(model_name, link_name)
        position = properties['position']
        orientation = properties['orientation']
        gravity_mode = properties['gravity_mode']
        moments = properties['moments']
        self.set_link_properties(model_name, link_name, position, orientation,
                                 gravity_mode, mass, moments)

    def set_link_moments(
            self, model_name: str, link_name: str,
            moments: Tuple[float, float, float, float, float, float]) -> None:
        """Set the omponents of the inertia matrix.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            moments (Tuple[float, float, float, float, float, float]):
                Components of the inertia matrix as
                (ixx, ixy, ixz, iyy, iyz, izz).
        """
        properties = self.get_link_properties(model_name, link_name)
        position = properties['position']
        orientation = properties['orientation']
        gravity_mode = properties['gravity_mode']
        mass = properties['mass']
        self.set_link_properties(model_name, link_name, position, orientation,
                                 gravity_mode, mass, moments)

    def set_link_state(
            self, model_name: str, link_name: str, position: Position,
            orientation: Orientation,
            linear_velocity: Tuple[float, float, float],
            angular_velocity: Tuple[float, float, float]) -> None:
        """Set link state.

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            position (Tuple[float, float, float]): Desired position of the
                link as (x, y, z).
            orientation (Tuple[float, float, float, float]): desire
                orientation of the link as quaternion(x, y, z, w).
            linear_velocity (Tuple[float, float, float]): desired linear
                velocity of the link as (vx, vy, vz).
            angular_velocity (Tuple[float, float, float]): desired angular
                velocity of the link as (rx, ry, rz).

        Warning:
            It is not recommended to use this function on a model containing
            joints.
        """
        name = '{}::{}'.format(model_name, link_name)
        super().set_link_state(
            name, position, orientation, linear_velocity,
            angular_velocity, reference_frame='')

    def set_link_position(
            self, model_name: str, link_name: str, position: Position) -> None:
        """Set the postion of the link.

        The velocities and the orientation are not affected when you call this
        function. If you want to 'drop' the link, you should use
        set_link_state().

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            position (Tuple[float, float, float]): Desired position of the
                link as (x, y, z).
        """
        state = self.get_link_state(model_name, link_name)
        orientation = state['orientation']
        linear_velocity = state['linear_velocity']
        angular_velocity = state['angular_velocity']
        self.set_link_state(
            model_name, link_name, position, orientation, linear_velocity,
            angular_velocity)

    def set_link_orientation(
            self, model_name: str, link_name: str,
            orientation: Orientation) -> None:
        """Set the orientation of the link.

        The velocities and the position are not affected when you call this
        function. If you want to 'drop' the link, you should use
        set_link_state().

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            orientation (Tuple[float, float, float, float]): desire
                orientation of the link as quaternion(x, y, z, w).
        """
        state = self.get_link_state(model_name, link_name)
        position = state['position']
        linear_velocity = state['linear_velocity']
        angular_velocity = state['angular_velocity']
        self.set_link_state(
            model_name, link_name, position, orientation, linear_velocity,
            angular_velocity)

    def set_link_linear_velocity(
            self, model_name: str, link_name: str,
            linear_velocity: Tuple[float, float, float]) -> None:
        """Set the linear velocity of the link.

        The angular velocity, the position and the orientation are not affected
        when you call this function. If you want to 'drop' the link, you should
        use set_link_state().

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            linear_velocity (Tuple[float, float, float]): desired linear
                velocity of the link as (vx, vy, vz).
        """
        state = self.get_link_state(model_name, link_name)
        position = state['position']
        orientation = state['orientation']
        angular_velocity = state['angular_velocity']
        self.set_link_state(
            model_name, link_name, position, orientation, linear_velocity,
            angular_velocity)

    def set_link_angular_velocity(
            self, model_name: str, link_name: str,
            angular_velocity: Tuple[float, float, float]) -> None:
        """Set the angular velocity of the link.

        The linear velocity, the position and the orientation are not affected
        when you call this function. If you want to 'drop' the link, you should
        use set_link_state().

        Args:
            model_name (str): The model containing the link.
            link_name (str): The name of the link.
            angular_velocity (Tuple[float, float, float]): desired angular
                velocity of the link as (rx, ry, rz).
        """
        state = self.get_link_state(model_name, link_name)
        position = state['position']
        orientation = state['orientation']
        linear_velocity = state['linear_velocity']
        self.set_link_state(
            model_name, link_name, position, orientation, linear_velocity,
            angular_velocity)

    def set_logger_level(self, logger: str, level: str) -> None:
        """Set logger level.

        To see the available loggers, run method `get_loggers()`.

        Args:
            logger (str): Logger name.
            level (str): Logger level
                (`'debug'`|`'info'`|`'warn'`|`'error'`|`'fatal'`).
        """
        super().set_logger_level(logger, level)

    def set_joint_positions(
            self, model_name: str, joint_names: List[str],
            joint_positions: List[float]) -> None:
        """Set model joint positions of a model.

        Args:
            model_name (str): Model to set state.
            joint_names (List[str]): List of joints to set positions. For
                joints in the model that are not listed here, the current
                position is preserved. To see available joint names, call
                the method get_joint_names(model_name)
            joint_positions (List[float]): Joint positions. Units depends on
                joint types.
        """
        super().set_model_configuration(
            model_name, '', joint_names, joint_positions)

    def set_model_state(
            self, model_name: str, position: List[float],
            orientation: List[float], linear_velocity: List[float],
            angular_velocity: List[float]) -> None:
        """Set Gazebo model position, orientation and velocities.

        Equivalent to set_link_state() on canonical link.

        Args:
            model_name (str): Model to set state.
            position (List[float]): Desired position as (x, y, z).
            orientation (List[float]): Desired orientation as quaternion
                (x, y, z, w).
            linear_velocity (List[float]): Desired linear velocity as
                (vx, vy, vz).
            angular_velocity (List[float]): Desired angular velocity as
                (rx, ry, rz).
        """
        super().set_model_state(
            model_name, position, orientation, linear_velocity,
            angular_velocity, '')

    def set_parameters(
            self, bools: Optional[List[Dict]] = None,
            ints: Optional[List[Dict]] = None,
            strs: Optional[List[Dict]] = None,
            doubles: Optional[List[Dict]] = None,
            groups: Optional[List[Dict]] = None) -> Dict:
        """Set or get simulation parameters. Not implemented.

        Args:
            bools (List[Dict], optional): bools,
                e.g.  [{'name': 'auto_disable_bodies', 'value': False}]
            ints (List[Dict], optional): ints,
                e.g.  [{'name': 'max_contact', 'value': 20}]
            strs (List[Dict], optional): strs
            doubles (List[Dict], optional): doubles,
                e.g.  [{'name': 'time_step', 'value': 0.001}]
            groups (List[Dict], optional): groups,
                e.g.  [{'name': 'Default', 'state': True, 'id': 0, 'parent': 0}]
        """
        raise NotImplementedError('What is the utility ?')

    def spawn_sdf_model(
            self, model_xml: str, position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1),
            model_name: Optional[str] = None,
            prefix: str = 'model_sdf') -> str:
        """Spawn a model from sdf-formated string.

        Args:
            model_xml (str): The model as sdf. See https://http://sdformat.org/
                for more details.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'model_sdf'`.

        Returns:
            str: model name
        """
        # Model name
        if model_name is not None:
            self._check_model_name(model_name)
        if model_name is None:
            model_name = self._generate_model_name(prefix)
        super().spawn_sdf_model(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace='',
            initial_position=position,
            initial_orientation=orientation,
            reference_frame='')
        return model_name

    def spawn_sdf_file(
            self, path_model_xml: str, position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1),
            model_name: Optional[str] = None,
            prefix: str = 'model_sdf') -> str:
        """Spawn a model from sdf file.

        Args:
            path_model_xml (str): The model as sdf file. See
                https://http://sdformat.org/ for more details.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'model_sdf'`.

        Returns:
            str: model name
        """
        with open(path_model_xml, 'r') as f:
            model_xml = f.read()
            model_name = self.spawn_sdf_model(
                model_xml, position, orientation, model_name, prefix)
        return model_name

    def spawn_cuboid(
            self, width: float = 1, depth: float = 1, height: float = 1,
            mass: float = 1, position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1), static: bool = False,
            model_name: Optional[str] = None, prefix: str = 'cuboid') -> str:
        """Spawn a cuboid.

        Note:
            The inertia matrix is calculated automatically according to the
            args.

        Args:
            width (float, optional): Length in x-axis. Defaults to `1`.
            depth (float, optional): Length in y-axis. Defaults to `1`.
            height (float, optional): Length in z-axis. Defaults to `1`.
            mass (float, optional): Mass in kg. Defaults to `1`.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            static (bool, optional): Whether link is static. Defaults to
                `False`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'cuboid'`.

        Return:
            str: The model name.
        """
        # compute inertia moment
        # see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        ixx = 1/12*mass*(depth**2 + height**2)
        ixy = 0
        ixz = 0
        iyy = 1/12*mass*(width**2 + height**2)
        iyz = 0
        izz = 1/12*mass*(width**2 + depth**2)

        template = '''
            <?xml version="1.0" ?>
            <sdf version="1.5">
                <model name="unused_name">
                    <static>{static}</static>

                    <link name="link">
                    <pose>0 0 0 0 0 0</pose>
                        <inertial>
                            <inertia>
                                <ixx>{ixx}</ixx>
                                <ixy>{ixy}</ixy>
                                <ixz>{ixz}</ixz>
                                <iyy>{iyy}</iyy>
                                <iyz>{iyz}</iyz>
                                <izz>{izz}</izz>
                            </inertia>
                            <mass>{mass}</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                            <box>
                                <size>{width} {depth} {height}</size>
                            </box>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                            <box>
                                <size>{width} {depth} {height}</size>
                            </box>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>'''
        model_xml = template.format(
            static=static,
            ixx=ixx, ixy=ixy, ixz=ixz, iyy=iyy, iyz=iyz, izz=izz,
            mass=mass, width=width, depth=depth, height=height)
        return self.spawn_sdf_model(
            model_xml, position, orientation, model_name, prefix)

    def spawn_cube(
            self, width: float = 1, mass: float = 1,
            position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1), static: bool = False,
            model_name: Optional[str] = None, prefix: str = 'cube') -> str:
        """Spawn a cube.

        Note:
            The inertia matrix is calculated automatically according to the
            args.

        Args:
            width (float, optional): Edge length. Defaults to `1`.
            mass (float, optional): Mass in kg. Defaults to `1`.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            static (bool, optional): Whether link is static. Defaults to
                `False`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'cube'`.

        Return:
            str: The model name.
        """
        return self.spawn_cuboid(
            width=width, depth=width, height=width,
            mass=mass, position=position, orientation=orientation,
            static=static, model_name=model_name, prefix=prefix)

    def spawn_cylinder(
            self, radius: float = 0.5, length: float = 1, mass: float = 1,
            position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1), static: bool = False,
            model_name: Optional[str] = None, prefix: str = 'sphere') -> str:
        """Spawn a cylinder.

        Note:
            The inertia matrix is calculated automatically according to the
            args.

        Args:
            radius (float, optional): Cylinder radius. Defaults to `0.5`.
            length (float, optional): Cylinder length. Defaults to `1`.
            mass (float, optional): Mass in kg. Defaults to `1`.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            static (bool, optional): Whether link is static. Defaults to
                `False`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'cylinder'`.

        Return:
            str: The model name.
        """
        # compute inertia moment
        # see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        ixx = 1/12*mass*(3*radius**2+length**2)
        ixy = 0
        ixz = 0
        iyy = 1/12*mass*(3*radius**2+length**2)
        iyz = 0
        izz = 1/2*mass*radius**2
        template = '''
            <?xml version="1.0" ?>
            <sdf version="1.5">
                <model name="unused_name">
                    <static>{static}</static>

                    <link name="link">
                    <pose>0 0 0 0 0 0</pose>
                        <inertial>
                            <inertia>
                                <ixx>{ixx}</ixx>
                                <ixy>{ixy}</ixy>
                                <ixz>{ixz}</ixz>
                                <iyy>{iyy}</iyy>
                                <iyz>{iyz}</iyz>
                                <izz>{izz}</izz>
                            </inertia>
                            <mass>{mass}</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                            <cylinder>
                                <radius>{radius}</radius>
                                <length>{length}</length>
                            </cylinder>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                            <cylinder>
                                <radius>{radius}</radius>
                                <length>{length}</length>
                            </cylinder>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>'''
        model_xml = template.format(
            static=static,
            ixx=ixx, ixy=ixy, ixz=ixz, iyy=iyy, iyz=iyz, izz=izz,
            mass=mass, radius=radius, length=length)
        return self.spawn_sdf_model(
            model_xml, position, orientation, model_name, prefix)

    def spawn_sphere(
            self, radius: float = 0.5, mass: float = 1,
            position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1), static: bool = False,
            model_name: Optional[str] = None, prefix: str = 'sphere') -> str:
        """Spawn a sphere.

        Note:
            The inertia matrix is calculated automatically according to the
            args.

        Args:
            radius (float, optional): Sphere radius. Defaults to `0.5`.
            mass (float, optional): Mass in kg. Defaults to `1`.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            static (bool, optional): Whether link is static. Defaults to
                `False`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'sphere'`.

        Return:
            str: The model name.
        """
        # compute inertia moment
        # see https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        ixx = 2/5*mass*radius**2
        ixy = 0
        ixz = 0
        iyy = 2/5*mass*radius**2
        iyz = 0
        izz = 2/5*mass*radius**2
        template = '''
            <?xml version="1.0" ?>
            <sdf version="1.5">
                <model name="unused_name">
                    <static>{static}</static>
                    <link name="link">
                        <inertial>
                            <inertia>
                                <ixx>{ixx}</ixx>
                                <ixy>{ixy}</ixy>
                                <ixz>{ixz}</ixz>
                                <iyy>{iyy}</iyy>
                                <iyz>{iyz}</iyz>
                                <izz>{izz}</izz>
                            </inertia>
                            <mass>{mass}</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                            <sphere>
                                <radius>{radius}</radius>
                            </sphere>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                            <sphere>
                                <radius>{radius}</radius>
                            </sphere>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>'''
        model_xml = template.format(
            static=static,
            ixx=ixx, ixy=ixy, ixz=ixz, iyy=iyy, iyz=iyz, izz=izz,
            mass=mass, radius=radius)

        return self.spawn_sdf_model(
            model_xml, position, orientation, model_name, prefix)

    def spawn_light(
            self, position: Position, light_type: str = 'point',
            direction: Direction = (0.0, 0.0, -1.0),
            diffuse: Color = (0.5, 0.5, 0.5, 1.0),
            specular: Color = (0.1, 0.1, 0.1, 1.0), light_range: float = 20,
            attenuation: Attenuation = (0.5, 0.1, 0.0),
            cast_shadows: bool = False, light_name: Optional[str] = None,
            prefix: str = 'light') -> str:
        """Spawn a light. Not implemented.

        Args:
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            light_type (str, optional): Light type.
                (`'point'`|`'directional'`|`'sport'`).
                Defaults to `'point'`.
            direction (Tuple[float, float, float], optional): Light direction
                as (yaw, pitch, roll). Defaults to `(0.0, 0.0, -1.0)`.
            diffuse (Tuple[float, float, float], optional): Diffuse color as
                (red, green, blue, alpha). Defaults to `(0.5, 0.5, 0.5, 1.0)`.
            specular (Tuple[float, float, float], optional): Specular color as
                (red, green, blue, alpha). Defaults to `(0.1, 0.1, 0.1, 1.0)`.
            light_range (float, optional): Maximum lighting distance. Defaults to
                `20`.
            attenuation (Tuple[float, float, float], optional): Attenuation
                as (constant, linear, quadratic). Defaults to
                `(0.5, 0.1, 0.0)`.
            cast_shadows (bool, optional): Cast shadows. Defaults to `False`.
            light_name (Optional[str] = None, optional): Desired light name,
                must be unique in the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `light_name` is `None`, the model name
                is generated automatically from the `prefix`. See `light_name`
                for more details. Defaults to `'light'`.

        Returns:
            str: Light name
        """
        # So far, GazeboROS API does not provide a native way to spawn lights.
        # See :
        # https://github.com/ros-simulation/gazebo_ros_pkgs/blob/4da3acf708812d00905c301eb66dd1616f0154c8/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L758
        # To spawn a light, it is possible to use spawn sdf, but you won't be
        # able to choose position and orientation : coordinates are doubles and
        # orienation is choosen as a weird mix between orientation and
        # direction. If possible, don't do this. Prefer to specify  the lights
        # in the world file you load when creating the instance.
        raise NotImplementedError('See the source code for more details')

    def spawn_urdf_model(
            self, model_xml: str, position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1),
            model_name: Optional[str] = None,
            prefix: str = 'model_urdf') -> str:
        """Spawn a model from urdf-formated string.

        Args:
            model_xml (str): The model as urdf.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'model_urdf'`.

        Returns:
            str: model name
        """
        # Model name
        if model_name is not None:
            self._check_model_name(model_name)
        if model_name is None:
            model_name = self._generate_model_name(prefix)

        robot_namespace = ''
        initial_position = position
        initial_orientation = orientation
        reference_frame = ''
        super().spawn_urdf_model(
            model_name, model_xml, robot_namespace, initial_position,
            initial_orientation, reference_frame)

        return model_name

    def spawn_urdf_file(
            self, path_model_xml: str, position: Position = (0, 0, 1),
            orientation: Orientation = (0, 0, 0, 1),
            model_name: Optional[str] = None,
            prefix: str = 'model_urdf') -> str:
        """Spawn a model from sdf file.

        Args:
            path_model_xml (str): The model as urdf file.
            position (Tuple[float, float, float], optional): Position as
                (x, y, z). Defaults to `(0, 0, 1)`.
            orientation (Tuple[float, float, float, float], optional):
                Orientation as quaternion (x, y, z, w). Defaults
                to `(0, 0, 0, 1)`.
            model_name (str, optional): Desired model name, must be unique in
                the simulation.
                If `None`, generates automatically a name from the `prefix`
                followed by an index (`<prefix>_<idx>`).
                Defaults to `None`.
            prefix (str, optional): If `model_name` is `None`, the model name
                is generated automatically from the `prefix`. See `model_name`
                for more details. Defaults to `'model_sdf'`.

        Returns:
            str: model name
        """
        with open(path_model_xml, 'r') as f:
            model_xml = f.read()
            model_name = self.spawn_urdf_model(
                model_xml, position, orientation, model_name, prefix)
        return model_name

    def unpause(self):
        """Unpause the simulation."""
        while self.paused:
            super().unpause_physics()

    #################################################

    def get_joint_positions(
            self, model_name: str) -> Tuple[List[float], List[float]]:
        """Get joint positions of a model.

        Args:
            model_name (str): The model to get state.

        Returns:
            Tuple:
                - joint_names (List[float]): The names of the joints.
                - joint_positions (List[float]): The positions of the joints.
                    The units depends on the joint type.

        Warning:
            Do not call this method on models that contain joints among
            REVOLUTE2, UNIVERSAL, BALL, GEAR. More details in
            get_joint_position docstring.
        """
        joint_names = self.get_joint_names(model_name)
        joint_posititions = [
            self.get_joint_position(model_name, joint_name)
            for joint_name in joint_names]
        return joint_names, joint_posititions

    def get_joint_rates(
            self, model_name: str) -> Tuple[List[float], List[float]]:
        """Get joint rates of a model.

        Args:
            model_name (str): The model to get state.

        Returns:
            Tuple:
                - joint_names (List[float]): The names of the joints.
                - joint_rates (List[float]): The rates of the joints.
                    The units depends on the joint type.

        Warning:
            Do not call this method on models that contain joints among
            REVOLUTE2, UNIVERSAL, BALL, GEAR. More details in
            get_joint_position docstring.
        """
        joint_names = self.get_joint_names(model_name)
        joint_rates = [
            self.get_joint_rate(model_name, joint_name)
            for joint_name in joint_names]
        return joint_names, joint_rates

    def set_joint_position(
            self, model_name: str, joint_name: str, joint_position: float):
        """Set a single joint position.

        Args:
            model_name (str): The model containing the joint.
            joint_name (str): The name of the joint.
            joint_position (float): Joint position. Unit depends on joint type.
        """
        self.set_joint_positions(model_name, [joint_name], [joint_position])

    #################################################

    def _generate_model_name(self, prefix):
        """Returns the first <prefix>_<idx> available model name"""
        model_names = self.get_model_names()
        i = 0
        while '_'.join((prefix, str(i))) in model_names:
            i += 1
        return '_'.join((prefix, str(i)))

    def _check_model_name(self, model_name: str):
        model_names = self.get_model_names()
        if model_name in model_names:
            raise Exception()

    def _fetch_physics_properties(self):
        """Fetch and store physics properties into corresponding attributes."""
        physics = self.get_physics_properties()

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
        self._contact_max_correcting_vel = physics[
            'contact_max_correcting_vel']
        self._cfm = physics['cfm']
        self._erp = physics['erp']
        self._max_contacts = physics['max_contacts']

    def _set_some_physics_properties(self, **kwargs):
        """Update some physics attributes contained in kwargs."""
        physics = super().get_physics_properties()
        physics.update(kwargs)
        # Unlike the answer, 'pause' is not a keyword of the request
        physics.pop('pause')

        with self.pausing():
            super().set_physics_properties(**physics)

    @ contextmanager
    def pausing(self):
        """Pausing context.

        If the simulation is paused before the context, it remains
        paused after.
        """
        initially_paused = self._paused
        try:
            self.pause()
            yield
        finally:
            if not initially_paused:
                self.unpause()

    def __getattribute__(self, name):
        # Since I want to replace `pause_physics` with `pause`, I need to
        # remove the inherited method `pause_physics`. Here is the cleanest way
        # I've found to do this.
        # See https://medium.com/@maouu/sorry-but-youre-wrong-aea1b88ffc03
        # I also replaced other functions.
        if name in ['pause_physics', 'unpause_physics',
                    'set_model_configuration', 'set_physics_properties']:
            raise AttributeError(name)
        else:
            return super().__getattribute__(name)

    def __dir__(self):
        return sorted(
            (set(dir(self.__class__)) | set(self.__dict__.keys())) - set(['pause_physics', 'unpause_physics', 'set_model_configuration', 'set_physics_properties']))
