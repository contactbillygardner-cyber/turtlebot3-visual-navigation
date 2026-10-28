"""Batch CLI regression checks without rendering full navigation trials."""
import contextlib
import io
import json
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
from unittest.mock import patch


class EvaluationTests(unittest.TestCase):
    def test_requested_q_r_scales_reach_every_trial_and_summary(self):
        calls = []

        def record_trial(args):
            parameters = vars(args).copy()
            calls.append(parameters)
            return {'success': True, 'parameters': parameters}

        with tempfile.TemporaryDirectory() as output:
            argv = ['evaluate_track_a.py', '--trials', '2', '--seconds', '7',
                    '--q-scale', '10', '--r-scale', '2', '--output', output]
            with patch.object(sys, 'argv', argv), \
                    patch('track_a_project.run', side_effect=record_trial), \
                    contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path(str(Path(__file__).with_name('evaluate_track_a.py')),
                               run_name='__main__')
            summary = json.loads((Path(output) / 'summary.json').read_text())

        self.assertEqual(len(calls), 8)
        self.assertEqual(len(summary), 8)
        for parameters in calls + [row['parameters'] for row in summary]:
            self.assertEqual(parameters['q_scale'], 10)
            self.assertEqual(parameters['r_scale'], 2)
            self.assertEqual(parameters['seconds'], 7)
            self.assertTrue(parameters['headless'])
        for condition in ('nominal', 'camera_noise_dropout', 'shifted_obstacle',
                          'slippery_floor'):
            trials = [row['parameters'] for row in summary
                      if row['condition'] == condition]
            self.assertEqual([row['seed'] for row in trials], [0, 1])
        # Scenario overrides must survive the common tuning parameters.
        self.assertEqual(summary[2]['parameters']['dropout'], .30)
        self.assertEqual(summary[2]['parameters']['pixel_noise'], 5)
        self.assertEqual(summary[4]['parameters']['obstacle_shift'], .15)
        self.assertEqual(summary[4]['parameters']['encoder_bias'], .004)
        self.assertEqual(summary[6]['parameters']['friction'], .6)


if __name__ == '__main__':
    unittest.main()
