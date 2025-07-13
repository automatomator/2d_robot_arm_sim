import pytest
from rob_arm_sim.arm import RobotArm, OutOfReachError
import numpy as np

def test_robot_arm_initialisation():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    assert arm.L1 == 100
    assert arm.L2 == 80
    assert arm.base_x == 0
    assert arm.base_y == 0

def test_robot_arm_invalid_lengths():
    with pytest.raises(ValueError):
        RobotArm(L1=0, L2=80, base_x=0, base_y=0)
    with pytest.raises(ValueError):
        RobotArm(L1=100, L2=0, base_x=0, base_y=0)

def test_forward_kinematics_straight():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    # Arm fully extended along X-axis
    x, y = arm.forward_kinematics(0, 0)
    assert np.isclose(x, 180)
    assert np.isclose(y, 0)

def test_forward_kinematics_folded():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    # Arm folded back on itself (theta2 = pi)
    x, y = arm.forward_kinematics(0, np.pi)
    assert np.isclose(x, 20) # 100 - 80
    assert np.isclose(y, 0)

def test_is_reachable_inside_bounds():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    assert arm.is_reachable(150, 0) # Well within range
    assert arm.is_reachable(180, 0) # Max reach
    assert arm.is_reachable(20, 0)  # Min reach

def test_is_reachable_outside_bounds():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    assert not arm.is_reachable(200, 0) # Too far
    assert not arm.is_reachable(10, 0)  # Too close
    assert not arm.is_reachable(0, 0)   # Base itself is technically not reachable by end-effector

def test_inverse_kinematics_reachable():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    target_x, target_y = 120, 50
    theta1, theta2 = arm.inv_kinematics(target_x, target_y)

    # Verify by forward kinematics
    fx, fy = arm.forward_kinematics(theta1, theta2)
    assert np.isclose(fx, target_x)
    assert np.isclose(fy, target_y)

    # A known point where theta1=0, theta2=0
    theta1_test, theta2_test = arm.inv_kinematics(180, 0)
    assert np.isclose(theta1_test, 0)
    assert np.isclose(theta2_test, 0)

    # A known point where theta1=0, theta2=pi
    theta1_test, theta2_test = arm.inv_kinematics(20, 0)
    assert np.isclose(theta1_test, 0)
    assert np.isclose(theta2_test, np.pi)

def test_inverse_kinematics_unreachable():
    arm = RobotArm(L1=100, L2=80, base_x=0, base_y=0)
    with pytest.raises(OutOfReachError):
        arm.inv_kinematics(200, 0) # Too far
    with pytest.raises(OutOfReachError):
        arm.inv_kinematics(10, 0) # Too close    