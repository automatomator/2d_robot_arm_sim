# rob_arm_sim/simulation.py
import numpy as np
import logging
from .arm import RobotArm, OutOfReachError

logger = logging.getLogger(__name__) # Get logger for this module

def simulate_circular_path(
    arm: RobotArm,
    circle_center_x: float,
    circle_center_y: float,
    circle_radius: float,
    speed_v: float,
    dt: float
) -> dict:
    """
    Simulates the robotic arm tracing a circular path.

    Args:
        arm (RobotArm): The RobotArm instance.
        circle_center_x (float): X-coordinate of the circle's center.
        circle_center_y (float): Y-coordinate of the circle's center.
        circle_radius (float): Radius of the circle.
        speed_v (float): Desired speed of the end effector along the circle.
        dt (float): Time step for the simulation.

    Returns:
        dict: A dictionary containing simulation data (time, angles, velocities, accelerations, end effector positions).

    Raises:
        OutOfReachError: If any point on the circle is unreachable by the arm.
    """
    logger.info(f"Starting simulation for circular path. Center=({circle_center_x},{circle_center_y}), Radius={circle_radius}, Speed={speed_v}, dt={dt}")

    circumference = 2 * np.pi * circle_radius
    total_time = circumference / speed_v
    num_steps = int(total_time / dt)
    
    if num_steps <= 1: # Ensure at least 2 steps for velocity/acceleration calculation
        logger.warning(f"Number of simulation steps ({num_steps}) too low. Adjusting total_time to ensure at least 2 steps for meaningful data.")
        num_steps = 2
        total_time = (num_steps - 1) * dt
        
    time_points = np.linspace(0, total_time, num_steps)
    logger.debug(f"Simulation will run for {total_time:.2f}s over {num_steps} steps.")

    theta1_data = []
    theta2_data = []
    omega1_data = [0.0] # Initial angular velocities
    omega2_data = [0.0]
    alpha1_data = [0.0] # Initial angular accelerations
    alpha2_data = [0.0]
    end_effector_x_data = []
    end_effector_y_data = []

    # Calculate initial position
    initial_angle = 0 # Start at (a + r, b)
    target_x_init = circle_center_x + circle_radius * np.cos(initial_angle)
    target_y_init = circle_center_y + circle_radius * np.sin(initial_angle)
    
    # Check reachability of the first point
    if not arm.is_reachable(target_x_init, target_y_init):
        logger.error(f"Initial point ({target_x_init:.2f},{target_y_init:.2f}) on circle is unreachable.")
        raise OutOfReachError(f"Initial point ({target_x_init:.2f}, {target_y_init:.2f}) is out of arm's reach.")

    try:
        theta1_prev, theta2_prev = arm.inv_kinematics(target_x_init, target_y_init)
    except OutOfReachError as e:
        logger.error(f"IK failed for initial point: {e}")
        raise # Re-raise the error to be caught by the GUI

    theta1_data.append(theta1_prev)
    theta2_data.append(theta2_prev)
    ee_x_curr, ee_y_curr = arm.forward_kinematics(theta1_prev, theta2_prev)
    end_effector_x_data.append(ee_x_curr)
    end_effector_y_data.append(ee_y_curr)
    logger.debug(f"Initial arm configuration: theta1={np.degrees(theta1_prev):.2f}deg, theta2={np.degrees(theta2_prev):.2f}deg at ({ee_x_curr:.2f},{ee_y_curr:.2f})")


    # Simulate path
    for i in range(1, num_steps):
        t_curr = time_points[i]
        angle_on_circle = (speed_v * t_curr) / circle_radius # Current angle based on arc length
        
        # Normalize angle to be within -pi to pi for consistent kinematics if needed,
        # but for continuous path, just let it increase.
        
        target_x = circle_center_x + circle_radius * np.cos(angle_on_circle)
        target_y = circle_center_y + circle_radius * np.sin(angle_on_circle)
        
        # Check reachability before attempting IK
        if not arm.is_reachable(target_x, target_y):
            logger.error(f"Point ({target_x:.2f},{target_y:.2f}) at time {t_curr:.2f}s is unreachable.")
            raise OutOfReachError(f"Point ({target_x:.2f}, {target_y:.2f}) at time {t_curr:.2f}s is out of arm's reach.")

        try:
            theta1_curr, theta2_curr = arm.inv_kinematics(target_x, target_y)
        except OutOfReachError as e: # This should ideally not be hit if is_reachable is correctly used
            logger.error(f"IK failed unexpectedly for point ({target_x:.2f},{target_y:.2f}) at time {t_curr:.2f}s: {e}")
            raise # Re-raise the error to be caught by the GUI
            
        theta1_data.append(theta1_curr)
        theta2_data.append(theta2_curr)

        # Calculate angular velocities and accelerations
        dt_val = time_points[i] - time_points[i-1]
        
        # Fallback for dt_val if for some reason it's zero or negative
        if dt_val <= 0: 
            logger.warning(f"Calculated dt_val is non-positive ({dt_val:.4f}) at step {i}. Using provided dt ({dt}). This might indicate an issue with time_points generation.")
            dt_val = dt

        omega1_curr = (theta1_curr - theta1_prev) / dt_val
        omega2_curr = (theta2_curr - theta2_prev) / dt_val
        omega1_data.append(omega1_curr)
        omega2_data.append(omega2_curr)

        # Ensure we have enough previous data for acceleration calculation
        alpha1_curr = (omega1_curr - omega1_data[-2]) / dt_val if len(omega1_data) > 1 else 0.0
        alpha2_curr = (omega2_curr - omega2_data[-2]) / dt_val if len(omega2_data) > 1 else 0.0
        alpha1_data.append(alpha1_curr)
        alpha2_data.append(alpha2_curr)
        
        ee_x_curr, ee_y_curr = arm.forward_kinematics(theta1_curr, theta2_curr)
        end_effector_x_data.append(ee_x_curr)
        end_effector_y_data.append(ee_y_curr)

        theta1_prev, theta2_prev = theta1_curr, theta2_curr
        logger.debug(f"Step {i}: t={t_curr:.2f}s, EE=({ee_x_curr:.2f},{ee_y_curr:.2f}), th1={np.degrees(theta1_curr):.2f}deg, th2={np.degrees(theta2_curr):.2f}deg")

    logger.info(f"Simulation completed for {len(time_points)} steps. Total time: {time_points[-1]:.2f}s.")
    return {
        'time': np.array(time_points),
        'theta1': np.array(theta1_data),
        'theta2': np.array(theta2_data),
        'omega1': np.array(omega1_data),
        'omega2': np.array(omega2_data),
        'alpha1': np.array(alpha1_data),
        'alpha2': np.array(alpha2_data),
        'end_effector_x': np.array(end_effector_x_data),
        'end_effector_y': np.array(end_effector_y_data)
    }