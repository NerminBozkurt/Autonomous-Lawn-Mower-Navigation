import importlib.util
import os

import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def _generator():
    path = os.path.join(ROOT, 'scripts', 'generate_switching_config.py')
    spec = importlib.util.spec_from_file_location('gen', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_switching_config_matches_the_tuned_configs():
    # Fails when a fair config was re-tuned without regenerating.
    gen = _generator()
    expected = gen.build(os.path.join(ROOT, 'config'))
    with open(os.path.join(ROOT, 'config', 'nav2_switching.yaml')) as f:
        actual = yaml.safe_load(f)
    assert actual == expected


def test_switching_config_loads_all_three_controllers():
    with open(os.path.join(ROOT, 'config', 'nav2_switching.yaml')) as f:
        server = yaml.safe_load(f)['controller_server']['ros__parameters']
    assert server['controller_plugins'] == ['RPP', 'MPPI', 'DWB']
    assert 'FollowPath' not in server
    assert 'RegulatedPurePursuit' in server['RPP']['plugin']
    assert 'MPPI' in server['MPPI']['plugin']
    assert 'DWB' in server['DWB']['plugin']
