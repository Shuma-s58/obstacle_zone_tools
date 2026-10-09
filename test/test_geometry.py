import math
from pathlib import Path
import tempfile
import unittest
from obstacle_zone_tools.geometry import Zone, load_zones, save_zones


def test_rotated_rectangle_and_edges():
    z = Zone('zone', 8, -3, math.pi / 2, 4, 2)
    assert z.contains(8, -1)
    assert z.contains(9, -3)
    assert not z.contains(9.01, -3)
    assert not z.contains(8, -0.99)
    assert z.contains(9.04, -3, margin=0.05)
    assert not z.contains(9.06, -3, margin=0.05)


def test_negative_coordinates_arbitrary_rotation():
    z = Zone('zone', -3, -5, 0.71, 5, 0.2)
    assert z.contains(*z.world(2.4, 0.09))
    assert not z.contains(*z.world(2.4, 0.11))


def test_union_overlap():
    zones = [Zone('a', 0, 0, 0, 2, 2), Zone('b', 1, 0, 0, 2, 2)]
    assert {z.name for z in zones if z.contains(0.5, 0)} == {'a', 'b'}
    assert {z.name for z in zones if z.contains(1.5, 0)} == {'b'}
    assert not any(z.contains(3, 0) for z in zones)


def test_file_roundtrip_empty_and_missing(tmp_path):
    p = tmp_path / 'sub' / 'zones.yaml'
    assert load_zones(p, 'map') == []
    zones = [Zone('a', -3, 5, 0.7, 3, 2)]
    save_zones(p, 'map', zones)
    assert load_zones(p, 'map') == zones
    save_zones(p, 'map', [])
    assert load_zones(p, 'map') == []


INVALID_FILES = [
    'frame_id: odom\nzones: []',
    'frame_id: map\nzones: null',
    'frame_id: map\nzones: [{name: a, x: 0, y: 0, yaw: 0, width: 0, height: 1}]',
    'frame_id: map\nzones: [{name: a, x: .nan, y: 0, yaw: 0, width: 1, height: 1}]',
    'frame_id: map\nzones: [{name: a__size, x: 0, y: 0, yaw: 0, width: 1, height: 1}]',
]

def test_reject_invalid_file(tmp_path, text):
    p = tmp_path / 'zones.yaml'
    p.write_text(text)
    with unittest.TestCase().assertRaises(ValueError):
        load_zones(p, 'map')


def load_tests(loader, tests, pattern):
    suite = unittest.TestSuite()
    for fn in (test_rotated_rectangle_and_edges, test_negative_coordinates_arbitrary_rotation, test_union_overlap):
        suite.addTest(unittest.FunctionTestCase(fn))
    def roundtrip():
        with tempfile.TemporaryDirectory() as folder:
            test_file_roundtrip_empty_and_missing(Path(folder))
    suite.addTest(unittest.FunctionTestCase(roundtrip))
    for text in INVALID_FILES:
        def invalid(text=text):
            with tempfile.TemporaryDirectory() as folder:
                test_reject_invalid_file(Path(folder), text)
        suite.addTest(unittest.FunctionTestCase(invalid, description=text))
    return suite
