"""Planning latency and continuous tracking regressions; no actuator output."""
import concurrent.futures
import unittest
from unittest import mock
import numpy as np
from racelib.ackermann import AckermannMission, CollisionWorld
from racelib.config import load_all
from sim.io_sim import SimIO


class ArrivalEfficiencyTests(unittest.TestCase):
    def mission(self, asynchronous=False):
        io = SimIO()
        io.params = {'async_planning': asynchronous}
        mission = AckermannMission(io, load_all())
        mission.estimator.set_pose(1., 2., 0.)
        mission._last_dt = .02
        return mission, io

    def ready_plan(self, mission):
        future = concurrent.futures.Future()
        path = [((x, 2., 0.), 1) for x in np.linspace(1., 4., 80)]
        future.set_result((path, .1, .005))
        mission._executor = mock.Mock()
        mission._executor.submit.return_value = future
        return path

    def test_completed_async_plan_is_consumed_next_tick_not_after_backoff(self):
        mission, io = self.mission(True)
        self.ready_plan(mission)
        self.assertEqual(mission._drive_to((4., 2.), .4), (0., 0.))
        io.advance(.02)
        v, w = mission._drive_to((4., 2.), .4)
        self.assertGreater(v, 0.)
        self.assertAlmostEqual(w, 0.)
        self.assertEqual(mission._executor.submit.call_count, 1)
        event = next(e for e in mission.events if e['event'] == 'path_planned')
        self.assertEqual(event['planning_margin_m'], .1)

    def test_old_goal_async_result_is_not_executed(self):
        mission, io = self.mission(True)
        self.ready_plan(mission)
        mission._drive_to((4., 2.), .4)
        io.advance(.02)
        self.assertEqual(mission._drive_to((4., 3.), .4), (0., 0.))
        self.assertFalse(mission._segment)
        self.assertTrue(any(e.get('reason') == 'stale_plan_origin_or_goal' for e in mission.events))

    def test_async_failure_never_runs_synchronous_reserve_search(self):
        mission, io = self.mission(True)
        future = concurrent.futures.Future()
        future.set_result((None, .1, .01))
        mission._executor = mock.Mock()
        mission._executor.submit.return_value = future
        with mock.patch('racelib.ackermann.hybrid_astar') as search, \
             mock.patch.object(mission, '_escape_command', return_value=None):
            mission._drive_to((4., 2.), .4)
            io.advance(.02)
            self.assertEqual(mission._drive_to((4., 2.), .4), (0., 0.))
            search.assert_not_called()

    def test_worker_tries_reserves_without_changing_execution_margin(self):
        mission, _ = self.mission()
        worlds = [CollisionWorld(mission.field, [], margin=m) for m in (.1, .08, .06)]
        path = [((1., 2., 0.), 1), ((1.1, 2., 0.), 1)]
        with mock.patch('racelib.ackermann.hybrid_astar', side_effect=[None, None, path]) as search:
            result, margin, elapsed = mission._search_reserves((1., 2., 0.), (4., 2.),
                worlds, .3, .52, True, .66)
        self.assertEqual(result, path)
        self.assertEqual(margin, .06)
        self.assertEqual(search.call_count, 3)
        self.assertGreaterEqual(elapsed, 0.)

    def test_result_from_old_heading_is_discarded(self):
        mission, io = self.mission(True)
        self.ready_plan(mission)
        mission._drive_to((4., 2.), .4)
        mission.estimator.set_pose(1., 2., .4)
        io.advance(.02)
        self.assertEqual(mission._drive_to((4., 2.), .4), (0., 0.))
        self.assertFalse(mission._segment)

    def test_worker_obstacles_are_copied_before_tracker_updates(self):
        from types import SimpleNamespace
        mission, _ = self.mission(True)
        self.ready_plan(mission)
        track = SimpleNamespace(x=2., y=3., radius=.115)
        mission.cones.snapshot = lambda: [track]
        mission._drive_to((4., 2.), .4)
        worlds = mission._executor.submit.call_args.args[3]
        track.x = 6.
        self.assertTrue(all(w._cones[0][0] == 2. for w in worlds))

    def test_all_reserve_retries_run_in_real_worker_thread(self):
        import threading
        mission, io = self.mission(True)
        entered, release = threading.Event(), threading.Event()
        callback_thread = threading.get_ident()
        threads, margins = [], []
        path = [((x, 2., 0.), 1) for x in np.linspace(1., 4., 80)]
        def search(*args, **kwargs):
            threads.append(threading.get_ident())
            margins.append(args[2].margin)
            entered.set()
            release.wait(timeout=2.)
            return path if len(margins) == 3 else None
        try:
            with mock.patch('racelib.ackermann.hybrid_astar', side_effect=search):
                self.assertEqual(mission._drive_to((4., 2.), .4), (0., 0.))
                self.assertTrue(entered.wait(timeout=1.))
                io.advance(.02)
                self.assertEqual(mission._drive_to((4., 2.), .4), (0., 0.))
                release.set()
                mission._plan_future.result(timeout=2.)
                io.advance(.02)
                self.assertGreater(mission._drive_to((4., 2.), .4)[0], 0.)
            self.assertEqual(margins, [.1, .08, .06])
            self.assertTrue(all(t != callback_thread for t in threads))
        finally:
            release.set()
            if mission._executor is not None:
                mission._executor.shutdown(wait=True)

    def test_mapping_prefetch_preserves_stationary_wait(self):
        from racelib.mission import State
        mission, io = self.mission(True)
        self.ready_plan(mission)
        mission.state, mission.state_t0 = State.MAPPING, 0.
        mission._scan_count = 18
        io.advance(1.7)
        self.assertEqual(mission._st_mapping(.02), (0., 0.))
        self.assertEqual(mission.state, State.MAPPING)
        self.assertEqual(mission._plan_goal, tuple(mission._target_zone('stronghold').center))
        self.assertEqual(io.cmd_history, [])

    def test_hold_prefetch_does_not_shorten_measured_hold(self):
        from types import SimpleNamespace
        from racelib.mission import State
        mission, io = self.mission(True)
        self.ready_plan(mission)
        mission.state, mission.state_t0, mission._hold_started = State.HOLD, 0., 0.
        mission._last_odom = SimpleNamespace(v=0.)
        mission._last_imu = SimpleNamespace(yaw_rate=0.)
        io.advance(2.8)
        self.assertEqual(mission._st_hold(.02), (0., 0.))
        self.assertEqual(mission.state, State.HOLD)
        self.assertEqual(mission._plan_goal, tuple(mission._target_zone('camp').center))
        io.advance(.21)
        self.assertEqual(mission._st_hold(.02), (0., 0.))
        self.assertEqual(mission.state, State.CROSS)
        self.assertGreaterEqual(mission.metrics['hold_actual_s'], 3.)

    def terminal(self):
        from racelib.mission import State
        mission, io = self.mission()
        mission.state = State.CROSS
        mission._direction = 1
        mission._hybrid_path = []
        mission._segment = [(5.4, 1.5, 0.), (5.9, 1.8, 0.),
                            (6.5, 1.8, 0.), (6.25, .75, -1.)]
        return mission, io

    def test_terminal_shortcut_keeps_whole_body_reserve_and_limits(self):
        mission, _ = self.terminal()
        before = dict(mission.motion)
        mission._shortcut_terminal(mission._world(), (5.4, 1.5, 0.), (6.25, .75), .4, 10.)
        self.assertEqual(mission.metrics.get('terminal_shortcuts'), 1)
        reserve = CollisionWorld(mission.field, [], margin=.1)
        self.assertTrue(all(reserve.clear(p) for p in mission._segment))
        self.assertEqual(mission.motion, before)
        np.testing.assert_allclose(mission._segment[-1][:2], (6.25, .75))

    def test_terminal_shortcut_rejects_obstructed_arc(self):
        from racelib.field import Cone
        mission, _ = self.terminal()
        old = mission._segment
        mission.cones.snapshot = lambda: [Cone(5.85, 1.35, 'unknown', .115)]
        mission._shortcut_terminal(mission._world(), (5.4, 1.5, 0.), (6.25, .75), .4, 10.)
        self.assertIs(mission._segment, old)

    def test_terminal_shortcut_does_not_replace_minefield_or_gear_change(self):
        mission, _ = self.terminal()
        old = mission._segment
        mission._shortcut_terminal(mission._world(), (3., 1.5, 0.), (6.25, .75), .4, 10.)
        self.assertIs(mission._segment, old)
        mission._hybrid_path = [((5.3, 1.5, 0.), -1)]
        mission._shortcut_terminal(mission._world(), (5.4, 1.5, 0.), (6.25, .75), .4, 11.)
        self.assertIs(mission._segment, old)


class ArrivalComparisonTests(unittest.TestCase):
    def row(self, name, success, seconds, speed=.4, digest='same'):
        return dict(case=dict(name=name), success=success, input_sha256=digest, scene_input_sha256=digest,
                    summary=dict(sim_time_s=seconds, run_time_s=seconds,
                                 motion_settings=dict(forward_mps=speed),
                                 cone_contacts=0, wall_contacts=0))

    def test_faster_run_with_higher_limits_cannot_pass(self):
        from tools.benchmark_arrival import comparison
        a = [self.row('a', True, 40.)]
        b = [self.row('a', True, 30., speed=.6)]
        self.assertFalse(comparison(a, b)['passed'])
        b = [self.row('a', True, 30., digest=None)]
        self.assertFalse(comparison(a, b)['passed'])

    def test_equal_success_count_cannot_hide_new_failure(self):
        from tools.benchmark_arrival import comparison
        a = [self.row('a', True, 40.), self.row('b', False, 90.)]
        b = [self.row('a', False, 90.), self.row('b', True, 30.)]
        result = comparison(a, b)
        self.assertFalse(result['passed'])
        self.assertEqual(result['new_failures'], ['a'])

    def test_changed_physics_or_time_budget_cannot_pass(self):
        from tools.benchmark_arrival import compare_records
        a = dict(runs=[self.row('a', True, 40.)], max_time_s=90.,
                 source_sha256={'sim/world.py': 'original'})
        b = dict(runs=[self.row('a', True, 30.)], max_time_s=90.,
                 source_sha256={'sim/world.py': 'changed'})
        self.assertFalse(compare_records(a, b)['passed'])
        b['source_sha256'] = a['source_sha256']
        b['max_time_s'] = 180.
        self.assertFalse(compare_records(a, b)['passed'])


if __name__ == '__main__':
    unittest.main()
