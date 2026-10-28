import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from limo_autorace_sim.control import Envelope


def test_twist_is_yaw_rate_and_ackermann_cannot_spin():
    e=Envelope(acceleration=100,steering_rate=100)
    v,w=e.step(.3,.2,1.0)
    assert v == pytest.approx(.3)
    assert w == pytest.approx(.2)
    assert e.steering == pytest.approx(math.atan(.2*.2/.3))
    v,w=e.step(0.,1.,1.0)
    assert (v,w) == (0.,0.)


def test_reverse_geometry_and_limits():
    e=Envelope(acceleration=100,steering_rate=100)
    v,w=e.step(-.3,-.2,1.0)
    assert v < 0 and w < 0 and e.steering > 0
    e.reset()
    v,w=e.step(100,100,1.0)
    assert abs(v)<=e.max_speed and abs(e.steering)<=e.central_steering_limit
    assert abs(w)<=e.max_yaw_rate


def test_acceleration_braking_and_invalid_command():
    e=Envelope()
    assert e.step(.6,0,.1)[0] == pytest.approx(.05)
    assert e.step(-.6,0,.1)[0] == 0
    assert e.step(float('nan'),0,.1) == (0.,0.)


def test_track_has_no_paint_collision_and_metric_extent():
    p=Path(__file__).resolve().parents[1]
    paint=ET.parse(p/'models/autorace_track/model.sdf')
    assert not paint.findall('.//collision')
    world=ET.parse(p/'worlds/autorace.sdf')
    assert len(world.findall('.//collision')) == 1  # only floor in default scenario
    assert world.find('.//world').get('name') == 'autorace'


def test_optical_axes_and_modes():
    import numpy as np
    # Rx(-pi/2) followed in parent by Rz(-pi/2): optical +Z -> body +X,
    # optical +X -> body -Y, optical +Y -> body -Z.
    rx=np.array([[1,0,0],[0,0,1],[0,-1,0]])
    rz=np.array([[0,1,0],[-1,0,0],[0,0,1]])
    assert np.allclose(rz@rx@np.array([0,0,1]),[1,0,0])
