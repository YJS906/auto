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


@pytest.fixture
def command_guard():
    import rclpy
    from limo_autorace_sim.command_guard import CommandGuard
    rclpy.init(args=['--ros-args', '-p', 'require_localization:=true', '-r', '__ns:=/guard_test',
                    '-r', '/sim/cmd_vel:=/guard_test/output',
                    '-r', '/localization/ready:=/guard_test/ready'])
    node = CommandGuard()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_localization_timeout_brakes_and_requires_fresh_command(command_guard):
    import time
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    node = command_guard
    node.localization(Bool(data=True))
    msg = Twist(); msg.linear.x = .2; node.command(msg)
    node.env.speed = .05
    node.localization_wall = time.monotonic()-1
    node.previous = node.get_clock().now().nanoseconds*1e-9-.1
    node.tick()
    assert node.env.speed == 0
    node.localization(Bool(data=True))
    node.previous = node.get_clock().now().nanoseconds*1e-9-.1
    node.tick()
    assert node.env.speed == 0  # Old commands must not resume on heartbeat recovery.
    node.command(msg)
    node.previous = node.get_clock().now().nanoseconds*1e-9-.1
    node.tick()
    assert node.env.speed > 0


def test_manual_mode_ignores_localization_gate(command_guard):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    node = command_guard
    node.require_localization = False
    msg = Twist(); msg.linear.x = .2; node.command(msg)
    node.localization(Bool(data=False))
    node.previous = node.get_clock().now().nanoseconds*1e-9-.1
    node.tick()
    assert node.env.speed > 0
