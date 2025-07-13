# rob_arm_sim/plotting.py
import matplotlib.pyplot as plt
import numpy as np
import logging

logger = logging.getLogger(__name__) # Get logger for this module

def plot_sim_data_on_axes(sim_data: dict, ax1, ax2, ax3):
    """
    Generates plots for joint angles, angular velocities,
    and angular accelerations over time on provided matplotlib axes.

    Args:
        sim_data (dict): Dictionary containing simulation data.
        ax1 (matplotlib.axes.Axes): Axes for joint angles plot.
        ax2 (matplotlib.axes.Axes): Axes for angular velocities plot.
        ax3 (matplotlib.axes.Axes): Axes for angular accelerations plot.
    """
    logger.info("Starting generation of static plots on provided axes.")
    try:
        time = sim_data['time']
        
        # Ensure data is not empty
        if len(time) == 0:
            logger.warning("No time data available for plotting. Skipping plot generation.")
            return

        theta1 = np.degrees(sim_data['theta1']) # Convert to degrees for readability
        theta2 = np.degrees(sim_data['theta2'])
        omega1 = sim_data['omega1']
        omega2 = sim_data['omega2']
        alpha1 = sim_data['alpha1']
        alpha2 = sim_data['alpha2']

        # Plot 1: Joint Angles
        ax1.clear() # Clear existing content if redrawing
        ax1.plot(time, theta1, label=r'$\theta_1$ (Link 1 Angle)')
        ax1.plot(time, theta2, label=r'$\theta_2$ (Link 2 Angle)')
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Angle (degrees)")
        ax1.set_title("Joint Angles vs. Time")
        ax1.legend()
        ax1.grid(True)
        logger.debug("Joint Angles plot generated successfully.")

        # Plot 2: Angular Velocities
        ax2.clear()
        ax2.plot(time, omega1, label=r'$\omega_1$ (Link 1 Angular Velocity)')
        ax2.plot(time, omega2, label=r'$\omega_2$ (Link 2 Angular Velocity)')
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Angular Velocity (rad/s)")
        ax2.set_title("Joint Angular Velocities vs. Time")
        ax2.legend()
        ax2.grid(True)
        logger.debug("Angular Velocities plot generated successfully.")

        # Plot 3: Angular Accelerations
        ax3.clear()
        ax3.plot(time, alpha1, label=r'$\alpha_1$ (Link 1 Angular Acceleration)')
        ax3.plot(time, alpha2, label=r'$\alpha_2$ (Link 2 Angular Acceleration)')
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Angular Acceleration (rad/s^2)")
        ax3.set_title("Joint Angular Accelerations vs. Time")
        ax3.legend()
        ax3.grid(True)
        logger.debug("Angular Accelerations plot generated successfully.")
        
        logger.info("All static plots successfully prepared on axes.")

    except Exception as e:
        logger.error(f"Error encountered during plotting on axes: {e}", exc_info=True)
        # Re-raise the exception so the calling GUI function can handle the display of an error message.
        raise