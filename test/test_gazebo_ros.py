import unittest
import pygazebo_ros

import os
import time
import warnings


class TestGazeboROS(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        warnings.simplefilter("ignore", ResourceWarning)
        warnings.simplefilter("ignore", DeprecationWarning)
        cls.gazebo_ros = pygazebo_ros.GazeboROS()

    @classmethod
    def tearDownClass(cls):
        cls.gazebo_ros.shutdown()

    # def tearDown(self):
    #     time.sleep(0.1)

    def setUp(self):
        warnings.simplefilter("ignore", ResourceWarning)
        warnings.simplefilter("ignore", DeprecationWarning)

    def test_time_step(self):
        self.gazebo_ros.time_step = 0.002
        time_step = self.gazebo_ros.time_step
        self.assertEqual(time_step, 0.002)
        self.gazebo_ros.time_step = 0.005
        time_step = self.gazebo_ros.time_step
        self.assertEqual(time_step, 0.005)

    def test_max_update_rate(self):
        self.gazebo_ros.max_update_rate = 500
        max_update_rate = self.gazebo_ros.max_update_rate
        self.assertEqual(max_update_rate, 500)
        self.gazebo_ros.max_update_rate = 1000
        max_update_rate = self.gazebo_ros.max_update_rate
        self.assertEqual(max_update_rate, 1000)

    def test_gravity(self):
        self.gazebo_ros.gravity = [1.0, 1.0, -1.0]
        gravity = self.gazebo_ros.gravity
        self.assertEqual(gravity, (1.0, 1.0, -1.0))
        self.gazebo_ros.gravity = [0.0, 0.0, -9.8]
        gravity = self.gazebo_ros.gravity
        self.assertEqual(gravity, (0.0, 0.0, -9.8))

    def test_auto_disable_bodies(self):
        # FIXME: with can't I set auto_disable_bodies ????
        # self.gazebo_ros.auto_disable_bodies = True
        # auto_disable_bodies = self.gazebo_ros.auto_disable_bodies
        # self.assertTrue(auto_disable_bodies)
        # self.gazebo_ros.auto_disable_bodies = False
        # auto_disable_bodies = self.gazebo_ros.auto_disable_bodies
        # self.assertFalse(auto_disable_bodies)
        pass

    def test_sor_pgs_precon_iters(self):
        self.gazebo_ros.sor_pgs_precon_iters = 10
        sor_pgs_precon_iters = self.gazebo_ros.sor_pgs_precon_iters
        self.assertEqual(sor_pgs_precon_iters, 10)
        self.gazebo_ros.sor_pgs_precon_iters = 0
        sor_pgs_precon_iters = self.gazebo_ros.sor_pgs_precon_iters
        self.assertEqual(sor_pgs_precon_iters, 0)

    def test_sor_pgs_iters(self):
        self.gazebo_ros.sor_pgs_iters = 51
        sor_pgs_iters = self.gazebo_ros.sor_pgs_iters
        self.assertEqual(sor_pgs_iters, 51)
        self.gazebo_ros.sor_pgs_iters = 50
        sor_pgs_iters = self.gazebo_ros.sor_pgs_iters
        self.assertEqual(sor_pgs_iters, 50)

    def test_sor_pgs_w(self):
        self.gazebo_ros.sor_pgs_w = 1.4
        sor_pgs_w = self.gazebo_ros.sor_pgs_w
        self.assertEqual(sor_pgs_w, 1.4)
        self.gazebo_ros.sor_pgs_w = 1.3
        sor_pgs_w = self.gazebo_ros.sor_pgs_w
        self.assertEqual(sor_pgs_w, 1.3)

    def test_sor_pgs_rms_error_tol(self):
        # FIXME: why can't I change sor_pgs_rms_error_tol ??
        # self.gazebo_ros.sor_pgs_rms_error_tol = 0.1
        # sor_pgs_rms_error_tol = self.gazebo_ros.sor_pgs_rms_error_tol
        # self.assertEqual(sor_pgs_rms_error_tol, 0.1)
        # self.gazebo_ros.sor_pgs_rms_error_tol = 0
        # sor_pgs_rms_error_tol = self.gazebo_ros.sor_pgs_rms_error_tol
        # self.assertEqual(sor_pgs_rms_error_tol, 0)
        pass

    def test_contact_surface_layer(self):
        self.gazebo_ros.contact_surface_layer = 0.002
        contact_surface_layer = self.gazebo_ros.contact_surface_layer
        self.assertEqual(contact_surface_layer, 0.002)
        self.gazebo_ros.contact_surface_layer = 0.001
        contact_surface_layer = self.gazebo_ros.contact_surface_layer
        self.assertEqual(contact_surface_layer, 0.001)

    def test_contact_max_correcting_vel(self):
        self.gazebo_ros.contact_max_correcting_vel = 200
        contact_max_correcting_vel = self.gazebo_ros.contact_max_correcting_vel
        self.assertEqual(contact_max_correcting_vel, 200)
        self.gazebo_ros.contact_max_correcting_vel = 100
        contact_max_correcting_vel = self.gazebo_ros.contact_max_correcting_vel
        self.assertEqual(contact_max_correcting_vel, 100)

    def test_cfm(self):
        self.gazebo_ros.cfm = 0.1
        cfm = self.gazebo_ros.cfm
        self.assertEqual(cfm, 0.1)
        self.gazebo_ros.cfm = 0.0
        cfm = self.gazebo_ros.cfm
        self.assertEqual(cfm, 0.0)

    def test_erp(self):
        self.gazebo_ros.erp = 0.3
        erp = self.gazebo_ros.erp
        self.assertEqual(erp, 0.3)
        self.gazebo_ros.erp = 0.2
        erp = self.gazebo_ros.erp
        self.assertEqual(erp, 0.2)

    def test_max_contacts(self):
        self.gazebo_ros.max_contacts = 30
        max_contacts = self.gazebo_ros.max_contacts
        self.assertEqual(max_contacts, 30)
        self.gazebo_ros.max_contacts = 20
        max_contacts = self.gazebo_ros.max_contacts
        self.assertEqual(max_contacts, 20)

    def test_apply_body_wrench(self):
        self.gazebo_ros.apply_body_wrench(
            model_name='model_test_apply_body_wrench',
            link_name='link_1',
            force=(-1, -2, 10),
            torque=(1, 2, 3),
            reference_point=(0, 0.1, 0),
            start_time=0.01,
            duration=-8)
        time.sleep(10)

    def test_apply_body_force(self):
        self.gazebo_ros.apply_body_force(
            model_name='model_test_apply_body_force',
            link_name='link_1',
            reference_point=[0, 0, 0],
            force=[0, -1, 20],
            start_time=1.000001,
            duration=3)

    def test_apply_body_torque(self):
        self.gazebo_ros.apply_body_torque(
            model_name='model_test_apply_body_torque',
            link_name='link_2',
            torque=(0, 0, 0.2))

    def test_apply_joint_effort(self):
        self.gazebo_ros.apply_joint_effort(
            model_name='model_test_apply_joint_effort',
            joint_name='joint',
            effort=1)

    def test_clear_body_wrenches(self):
        self.gazebo_ros.clear_body_wrenches(
            model_name='model_test_clear_body_wrenches',
            link_name='link_1')

    def test_clear_joint_forces(self):
        self.gazebo_ros.clear_joint_forces(
            model_name='model_test_clear_joint_forces',
            joint_name='joint')

    def test_delete_light(self):
        self.gazebo_ros.delete_light(
            light_name='light_test_delete_light')

    def test_delete_model(self):
        self.gazebo_ros.delete_model(
            model_name='model_test_delete_model')

    def test_get_joint_properties(self):
        out = self.gazebo_ros.get_joint_properties(
            model_name='model_test_get_joint_properties',
            joint_name='joint')
        self.assertIsInstance(out, dict)

    def test_get_joint_type(self):
        out = self.gazebo_ros.get_joint_type(
            model_name='model_test_get_joint_type',
            joint_name='joint')
        self.assertIsInstance(out, str)
        self.assertIn(out, ['REVOLUTE', 'CONTINUOUS',
                            'PRISMATIC', 'FIXED', 'BALL', 'UNIVERSAL'])

    def test_get_joint_position(self):
        out = self.gazebo_ros.get_joint_position(
            model_name='model_test_get_joint_position',
            joint_name='joint')
        self.assertIsInstance(out, float)
        self.assertAlmostEqual(out, 0, 2)

    def test_get_joint_rate(self):
        self.gazebo_ros.get_joint_rate(
            model_name='model_test_get_joint_rate',
            joint_name='joint')

    def test_get_light_properties(self):
        out = self.gazebo_ros.get_light_properties(
            light_name='light_test_get_light_properties')
        r, g, b, a = out['color']
        c, l, q = out['attenuation']
        self.assertAlmostEqual(r, 0.1)
        self.assertAlmostEqual(g, 0.2)
        self.assertAlmostEqual(b, 0.3)
        self.assertAlmostEqual(a, 0.4)
        self.assertAlmostEqual(c, 0.5)
        self.assertAlmostEqual(l, 0.6)
        self.assertAlmostEqual(q, 0.7)

    def test_get_light_color(self):
        r, g, b, a = self.gazebo_ros.get_light_color(
            light_name='light_test_get_light_color')
        self.assertAlmostEqual(r, 0.2)
        self.assertAlmostEqual(g, 0.3)
        self.assertAlmostEqual(b, 0.4)
        self.assertAlmostEqual(a, 0.5)

    def test_get_light_attenuation(self):
        c, l, q = self.gazebo_ros.get_light_attenuation(
            light_name='light_test_get_light_attenuation')
        self.assertAlmostEqual(c, 0.4)
        self.assertAlmostEqual(l, 0.5)
        self.assertAlmostEqual(q, 0.6)

    def test_get_link_properties(self):
        self.gazebo_ros.get_link_properties(
            model_name='model_test_get_link_properties',
            link_name='link_2')

    def test_get_link_gravity_mode(self):
        out = self.gazebo_ros.get_link_gravity_mode(
            model_name='model_test_get_link_gravity_mode',
            link_name='link_1')
        self.assertIsInstance(out, bool)
        self.assertTrue(out)
        out = self.gazebo_ros.get_link_gravity_mode(
            model_name='model_test_get_link_gravity_mode',
            link_name='link_2')
        self.assertIsInstance(out, bool)
        self.assertFalse(out)

    def test_get_link_mass(self):
        out = self.gazebo_ros.get_link_mass(
            model_name='model_test_get_link_mass',
            link_name='link_1')
        self.assertIsInstance(out, float)
        self.assertEqual(out, 1.0)
        out = self.gazebo_ros.get_link_mass(
            model_name='model_test_get_link_mass',
            link_name='link_2')
        self.assertIsInstance(out, float)
        self.assertEqual(out, 2.0)

    def test_get_link_moments(self):
        out = self.gazebo_ros.get_link_moments(
            model_name='model_test_get_link_moments',
            link_name='link_1')
        self.assertIsInstance(out, tuple)
        self.assertTupleEqual(out, (0.16, 0.01, 0.02, 0.17, 0.03, 0.18))

    def test_get_link_state(self):
        self.gazebo_ros.get_link_state(
            model_name='model_test_get_link_position',
            link_name='link_2')

    def test_get_link_position(self):
        x, y, z = self.gazebo_ros.get_link_position(
            model_name='model_test_get_link_position',
            link_name='link_2')

    def test_get_link_orientation(self):
        x, y, z, w = self.gazebo_ros.get_link_orientation(
            model_name='model_test_apply_body_force',
            link_name='link_2')

    def test_get_link_linear_velocity(self):
        self.gazebo_ros.apply_body_force(
            model_name='model_test_get_link_linear_velocity',
            link_name='link_2', force=(0, 0, 30))
        time.sleep(1)
        vx, vy, vz = self.gazebo_ros.get_link_linear_velocity(
            model_name='model_test_get_link_linear_velocity',
            link_name='link_2')
        self.assertGreater(vz, 0)  # because positive force is applied in z

    def test_get_link_angular_velocity(self):
        self.gazebo_ros.apply_body_torque(
            model_name='model_test_get_link_angular_velocity',
            link_name='link_2',
            torque=(0, 0, 0.2))
        time.sleep(1)
        rx, ry, rz = self.gazebo_ros.get_link_angular_velocity(
            model_name='model_test_get_link_angular_velocity',
            link_name='link_2')
        self.assertGreater(rz, 0)  # because positive torque is applied in z

    def test_get_loggers(self):
        out = self.gazebo_ros.get_loggers()
        for key, value in out.items():
            self.assertIsInstance(key, str)
            self.assertIn(value['level'], [
                          'debug', 'info', 'warn', 'error', 'fatal'])

    def test_get_model_properties(self):
        out = self.gazebo_ros.get_model_properties(
            model_name='model_test_get_model_properties')
        self.assertEqual(out['parent_model_name'], '')
        # FIXME: does not work so far
        # self.assertEqual(out['canonical_body_name'], 'link_1')
        self.assertListEqual(out['link_names'], ['link_1', 'link_2'])
        self.assertListEqual(out['geom_names'], ['col_link_1', 'col_link_2'])
        self.assertListEqual(out['joint_names'], ['my_joint'])
        self.assertListEqual(out['child_model_names'], [])
        self.assertEqual(out['is_static'], False)

    def test_get_parent_model_name(self):
        out = self.gazebo_ros.get_parent_model_name(
            model_name='model_test_get_parent_model_name')
        self.assertEqual(out, 'model_test_get_parent_model_name_parent')

    def test_get_canonical_link_name(self):
        out = self.gazebo_ros.get_canonical_link_name(
            model_name='model_test_get_canonical_link_name')
        # FIXME: this function never return a non-empty string
        # self.assertEqual(out, 'link_1')

    def test_get_link_names(self):
        out = self.gazebo_ros.get_link_names(
            model_name='model_test_get_link_names')
        self.assertListEqual(out, ['first_link', 'second_link'])

    def test_get_geom_name(self):
        out = self.gazebo_ros.get_geom_name(
            model_name='model_test_get_geom_name',
            link_name='link_2')
        self.assertEqual(out, 'link_2_collision')

    def test_get_joint_names(self):
        out = self.gazebo_ros.get_joint_names(
            model_name='model_test_get_joint_names')
        self.assertEqual(out, ['first_joint', 'second_joint'])

    def test_get_child_model_names(self):
        out = self.gazebo_ros.get_child_model_names(
            model_name='model_test_get_child_model_names')
        self.assertListEqual(out, ['model_test_get_child_model_names_child1',
                                   'model_test_get_child_model_names_child2'])

    def test_is_model_static(self):
        out = self.gazebo_ros.is_model_static(
            model_name='model_test_is_model_static')
        self.assertIsInstance(out, bool)
        self.assertEqual(out, False)

    def test_get_model_state(self):
        out = self.gazebo_ros.get_model_state(
            model_name='model_test_get_model_state')
        self.assertAlmostEqual(out['position'][0], -2, 1)
        self.assertAlmostEqual(out['position'][1], 2, 1)
        self.assertAlmostEqual(out['position'][2], 0.5, 1)
        self.assertAlmostEqual(out['orientation'][0], 0, 1)
        self.assertAlmostEqual(out['orientation'][1], 0, 1)
        self.assertAlmostEqual(out['orientation'][2], 0.247, 1)
        self.assertAlmostEqual(out['orientation'][3], 0.969, 1)
        self.assertAlmostEqual(out['linear_velocity'][0], 0, 1)
        self.assertAlmostEqual(out['linear_velocity'][1], 0, 1)
        self.assertAlmostEqual(out['linear_velocity'][2], 0, 1)
        self.assertAlmostEqual(out['angular_velocity'][0], 0, 1)
        self.assertAlmostEqual(out['angular_velocity'][1], 0, 1)
        self.assertAlmostEqual(out['angular_velocity'][2], 0, 1)

    def test_get_physics_properties(self):
        out = self.gazebo_ros.get_physics_properties()
        self.assertEqual(out['time_step'], 0.001)
        self.assertEqual(out['contact_surface_layer'], 0.001)
        self.assertEqual(out['max_contacts'], 20)

    def test_get_world_properties(self):
        out = self.gazebo_ros.get_world_properties()
        self.assertIsInstance(out, dict)
        self.assertGreater(out['sim_time'], 0.1)
        self.assertIn('model_test_get_world_properties', out['model_names'])
        self.assertTrue(out['rendering_enabled'])

    def test_get_sim_time(self):
        self.assertIsInstance(self.gazebo_ros.get_sim_time(), float)

    def test_get_model_names(self):
        model_names = self.gazebo_ros.get_model_names()
        self.assertIn('model_test_get_model_names', model_names)

    def test_pause(self):
        self.gazebo_ros.pause()
        self.assertTrue(self.gazebo_ros.paused)
        self.gazebo_ros.unpause()
        self.assertFalse(self.gazebo_ros.paused)

    def test_reset_simulation(self):
        self.gazebo_ros.reset_simulation()
        # time is reseted, then it must be low just after reset
        self.assertLess(self.gazebo_ros.get_sim_time(), 1.0)

    def test_reset_world(self):
        # if last reset is close, this reset can failed and link_1 still over
        # 5m. To avoid this, wait a little before reseting.
        time.sleep(2)
        self.gazebo_ros.reset_world()
        # time is not reset (can fail if simulation is more than 10x real slow)
        self.assertGreater(self.gazebo_ros.get_sim_time(), 0.2)
        z = self.gazebo_ros.get_link_position(
            model_name='model_test_reset_world',
            link_name='link_1')[2]
        # the model is re-dropped, then it must be over 5 m.
        self.assertGreater(z, 5)

    def test_set_joint_properties(self):
        pass  # see source code

    def test_set_light_properties(self):
        self.gazebo_ros.set_light_properties(
            light_name='light_test_set_light_properties',
            color=(0.2, 0.3, 0.4, 0.5),
            attenuation=(0.6, 0.7, 0.8))
        out = self.gazebo_ros.get_light_properties(
            light_name='light_test_set_light_properties')
        self.assertAlmostEqual(out['color'][0], 0.2, 2)
        self.assertAlmostEqual(out['attenuation'][0], 0.6, 2)

    def test_set_light_color(self):
        self.gazebo_ros.set_light_color(
            light_name='light_test_set_light_color',
            color=(0.2, 0.3, 0.4, 0.5))
        out = self.gazebo_ros.get_light_color(
            light_name='light_test_set_light_color')
        self.assertAlmostEqual(out[0], 0.2, 2)

    def test_set_light_attenuation(self):
        self.gazebo_ros.set_light_attenuation(
            light_name='light_test_set_light_attenuation',
            attenuation=(0.6, 0.7, 0.8))
        out = self.gazebo_ros.get_light_attenuation(
            light_name='light_test_set_light_attenuation')
        self.assertAlmostEqual(out[0], 0.6, 2)

    def test_set_link_properties(self):
        self.gazebo_ros.set_link_properties(
            model_name='model_test_set_link_properties',
            link_name='link_1',
            position=(0.0, 0.0, 0.1),
            orientation=(0, 0, 0.2873479, 0.9578263),
            gravity_mode=False,
            mass=2,
            moments=(0.333, 0.000, 0.000, 0.333, 0.000, 0.333))
        out = self.gazebo_ros.get_link_properties(
            model_name='model_test_set_link_properties',
            link_name='link_1')
        self.assertAlmostEqual(out['position'][2], 0.1)
        # FIXME: I can't change orientation
        # self.assertAlmostEqual(out['orientation'][2], 0.2873479)
        self.assertFalse(out['gravity_mode'])
        self.assertAlmostEqual(out['mass'], 2.0)
        self.assertAlmostEqual(out['moments'][5], 0.333)

    def test_set_link_inertia_position(self):
        self.gazebo_ros.set_link_inertia_position(
            model_name='model_test_set_link_inertia_position',
            link_name='link_1',
            position=(0.0, 0.0, 0.1))
        out = self.gazebo_ros.get_link_properties(
            model_name='model_test_set_link_inertia_position',
            link_name='link_1')
        self.assertAlmostEqual(out['position'][2], 0.1)

    def test_set_link_inertia_orientation(self):
        self.gazebo_ros.set_link_inertia_orientation(
            model_name='model_test_set_link_inertia_orientation',
            link_name='link_1',
            orientation=(0, 0, 0.2873479, 0.9578263))
        out = self.gazebo_ros.get_link_properties(
            model_name='model_test_set_link_inertia_orientation',
            link_name='link_1')
        # FIXME: I can't change orientation
        # self.assertAlmostEqual(out['orientation'][2], 0.2873479)

    def test_set_link_gravity_mode(self):
        self.gazebo_ros.set_link_gravity_mode(
            model_name='model_test_set_link_gravity_mode',
            link_name='link_1',
            gravity_mode=False)
        out = self.gazebo_ros.get_link_gravity_mode(
            model_name='model_test_set_link_gravity_mode',
            link_name='link_1')
        self.assertFalse(out)

    def test_set_link_mass(self):
        self.gazebo_ros.set_link_mass(
            model_name='model_test_set_link_mass',
            link_name='link_1',
            mass=2)
        out = self.gazebo_ros.get_link_mass(
            model_name='model_test_set_link_mass',
            link_name='link_1')
        self.assertAlmostEqual(out, 2.0)

    def test_set_link_moments(self):
        self.gazebo_ros.set_link_moments(
            model_name='model_test_set_link_moments',
            link_name='link_1',
            moments=(0.333, 0.000, 0.000, 0.333, 0.000, 0.333))
        out = self.gazebo_ros.get_link_moments(
            model_name='model_test_set_link_moments',
            link_name='link_1')
        self.assertAlmostEqual(out[5], 0.333)

    def test_set_link_state(self):
        self.gazebo_ros.set_link_state(
            model_name='model_test_set_link_state',
            link_name='link_1',
            position=(0, 8, 2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 0.0, 0.0))

    def test_set_link_position(self):
        self.gazebo_ros.set_link_position(
            model_name='model_test_set_link_position',
            link_name='link_1',
            position=(2, -8, 2))
        out = self.gazebo_ros.get_link_position(
            model_name='model_test_set_link_position',
            link_name='link_1')
        self.assertAlmostEqual(out[0], 2)
        self.assertAlmostEqual(out[1], -8)
        self.assertAlmostEqual(out[2], 2)

    def test_set_link_orientation(self):
        self.gazebo_ros.set_link_orientation(
            model_name='model_test_set_link_orientation',
            link_name='link_1',
            orientation=(0.0936586, 0.1873172, 0.2809757, 0.9365858))
        out = self.gazebo_ros.get_link_orientation(
            model_name='model_test_set_link_orientation',
            link_name='link_1')
        self.assertAlmostEqual(out[0], 0.0936586)
        self.assertAlmostEqual(out[1], 0.1873172)
        self.assertAlmostEqual(out[2], 0.2809757)
        self.assertAlmostEqual(out[3], 0.9365858)

    def test_set_link_linear_velocity(self):
        self.gazebo_ros.set_link_linear_velocity(
            model_name='model_test_set_link_linear_velocity',
            link_name='link_1',
            linear_velocity=(0.0936586, 0.1873172, 0.9365858))
        out = self.gazebo_ros.get_link_linear_velocity(
            model_name='model_test_set_link_linear_velocity',
            link_name='link_1')
        self.assertAlmostEqual(out[0], 0.0936586)
        self.assertAlmostEqual(out[1], 0.1873172)
        self.assertAlmostEqual(out[2], 0.9365858)

    def test_set_link_angular_velocity(self):
        self.gazebo_ros.set_link_angular_velocity(
            model_name='model_test_set_link_angular_velocity',
            link_name='link_1',
            angular_velocity=(0.0936586, 0.1873172, 0.9365858))
        out = self.gazebo_ros.get_link_angular_velocity(
            model_name='model_test_set_link_angular_velocity',
            link_name='link_1')
        self.assertAlmostEqual(out[0], 0.0936586)
        self.assertAlmostEqual(out[1], 0.1873172)
        self.assertAlmostEqual(out[2], 0.9365858)

    def test_set_logger_level(self):
        self.gazebo_ros.set_logger_level('ros.gazebo_ros.api_plugin', 'warn')
        out = self.gazebo_ros.get_loggers()
        self.assertEqual(out['ros.gazebo_ros.api_plugin']['level'], 'warn')

    def test_set_joint_positions(self):
        self.gazebo_ros.set_joint_positions(
            model_name='model_test_set_joint_positions',
            joint_names=['joint_1', 'joint_3'],
            joint_positions=[0.75, 0.5])

    def test_set_model_state(self):
        self.gazebo_ros.set_model_state(
            model_name='model_test_set_model_state',
            position=(2, 2, 1),
            orientation=(0.0936586, 0.1873172, 0.2809757, 0.9365858),
            linear_velocity=(0, 0, 1),
            angular_velocity=(0, 0, 1))

    def test_set_parameters(self):
        pass  # Not implemented

    def test_spawn_sdf_model(self):
        model_xml = '''
            <?xml version="1.0" ?>
            <sdf version="1.5">
                <model name="unused_name">
                    <static>False</static>
                    <link name="link">
                    <pose>0 0 0 0 0 0</pose>
                        <inertial>
                            <inertia>
                                <ixx>0.1</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.1</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.1</izz>
                            </inertia>
                            <mass>1</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                                <polyline>
                                    <point> 0.00 -0.50</point>
                                    <point> 0.50 -0.25</point>
                                    <point> 0.50  0.25</point>
                                    <point> 0.00  0.50</point>
                                    <point>-0.50  0.25</point>
                                    <point>-0.50 -0.25</point>
                                    <height>2.0</height>
                                </polyline>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                                <polyline>
                                    <point> 0.00 -0.50</point>
                                    <point> 0.50 -0.25</point>
                                    <point> 0.50  0.25</point>
                                    <point> 0.00  0.50</point>
                                    <point>-0.50  0.25</point>
                                    <point>-0.50 -0.25</point>
                                    <height>2.0</height>
                                </polyline>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>'''
        self.gazebo_ros.spawn_sdf_model(
            model_xml=model_xml,
            position=(4, -8, 1),
            orientation=(0.1, 0.2, 0.0, 1.0))

    def test_spawn_sdf_file(self):
        test_path = os.path.dirname(os.path.abspath(__file__))
        path_model_xml = os.path.join(test_path, 'models/model_test.sdf')
        self.gazebo_ros.spawn_sdf_file(
            path_model_xml=path_model_xml,
            position=(4, -6, 1),
            orientation=(0.1, 0.02, 0.0, 1.0))

    def test_spawn_cuboid(self):
        self.gazebo_ros.spawn_cuboid(
            width=0.3,
            depth=0.4,
            height=0.5,
            mass=0.6,
            position=(4, -4, 1),
            orientation=(0.0, 0.0, 0.1, 1.0),
            static=True,
            model_name='model_test_spawn_cuboid')

    def test_spawn_cube(self):
        self.gazebo_ros.spawn_cube(
            width=0.4,
            mass=3.2,
            position=(4, -2, 1),
            orientation=(0.1, -0.1, 0.1, 1.0),
            static=False,
            model_name=None)

    def test_spawn_cylinder(self):
        self.gazebo_ros.spawn_cylinder(
            radius=0.2,
            length=1.1,
            mass=11.9,
            position=(4, 0, 1),
            orientation=(0.1, 0.2, 0.0, 1.0),
            static=False,
            model_name='my_cylinder')

    def test_spawn_sphere(self):
        self.gazebo_ros.spawn_sphere(
            radius=0.45,
            mass=3.14,
            position=(4, 2, 1),
            orientation=(0.1, 0.0, 0.0, 1.0),
            static=False)

    def test_spawn_light(self):
        # Not implemented, no test.
        pass

    def test_spawn_urdf_model(self):
        model_xml = '''<?xml version="1.0" ?>
            <robot name="cube">
            <link name="baseLink">
                <contact>
                    <lateral_friction value="1.0"/>
                    <rolling_friction value="0.0"/>
                    <contact_cfm value="0.0"/>
                    <contact_erp value="1.0"/>
                </contact>
                <inertial>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                    <mass value="1.0"/>
                    <inertia ixx="0.166" ixy="0" ixz="0" iyy="0.166" iyz="0" izz="0.166"/>
                </inertial>
                <visual>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                    <geometry>
                        <box size="1 1 1"/>
                    </geometry>
                    <material name="white">
                        <color rgba="1 1 1 1"/>
                    </material>
                </visual>
                <collision>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                    <geometry>
                        <box size="1 1 1"/>
                    </geometry>
                </collision>
            </link>
            </robot>'''
        self.gazebo_ros.spawn_urdf_model(
            model_xml=model_xml,
            position=(4, 4, 2),
            orientation=(0, 0, 0, 1),
            model_name='my_urdf')

    def test_spawn_urdf_file(self):
        test_path = os.path.dirname(os.path.abspath(__file__))
        path_model_xml = os.path.join(test_path, 'models/model_test.urdf')
        self.gazebo_ros.spawn_urdf_file(
            path_model_xml=path_model_xml,
            position=(4, 6, 1),
            orientation=(0.02, 0.01, 0.03, 0.9))

    def test_unpause(self):
        pass  # See self.test_pause()

    def test_get_joint_positions(self):
        out = self.gazebo_ros.get_joint_positions(
            model_name='model_test_get_joint_positions')
        self.assertIsInstance(out, tuple)
        names, positions = out
        self.assertIsInstance(names, list)
        self.assertIsInstance(positions, list)
        self.assertIsInstance(names[0], str)
        self.assertIsInstance(positions[0], float)

    def test_get_joint_rates(self):
        out = self.gazebo_ros.get_joint_rates(
            model_name='model_test_get_joint_rates')
        self.assertIsInstance(out, tuple)
        names, rates = out
        self.assertIsInstance(names, list)
        self.assertIsInstance(rates, list)
        self.assertIsInstance(names[0], str)
        self.assertIsInstance(rates[0], float)

    def test_set_joint_position(self):
        self.gazebo_ros.set_joint_position(
            model_name='model_test_set_joint_position',
            joint_name='joint',
            joint_position=0.75)

    def test_pausing_context(self):
        self.gazebo_ros.unpause()
        with self.gazebo_ros.pausing():
            self.assertTrue(self.gazebo_ros.paused)
        self.assertFalse(self.gazebo_ros.paused)

        self.gazebo_ros.pause()
        with self.gazebo_ros.pausing():
            self.assertTrue(self.gazebo_ros.paused)
        self.assertTrue(self.gazebo_ros.paused)

        self.gazebo_ros.unpause()
