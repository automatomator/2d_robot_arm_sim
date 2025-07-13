# 2-DOF Robotic Arm Simulator

A Python-based GUI application to simulate the kinematics of a 2-Degrees-Of-Freedom (2-DOF) robotic arm. The arm's end effector traces a circular path at a uniform speed, providing both an animated visualization and detailed plots of joint angles, angular velocities, and angular accelerations.

## Features

* **2-DOF Arm Kinematics:** Simulates forward and inverse kinematics for a two-link robotic arm.

* **Circular Path Tracing:** The end effector traces a user-defined circular path.

* **Real-time Animation:** Visualizes the arm's movement as it traces the path.

* **Reachability Check:** Validates if all points on the target circle are within the arm's reachable workspace.

* **Detailed Data Plots:** Generates and displays graphs for:

    * Joint Angles ($\theta_1$, $\theta_2$) vs. Time

    * Joint Angular Velocities ($\omega_1$, $\omega_2$) vs. Time

    * Joint Angular Accelerations ($\alpha_1$, $\alpha_2$) vs. Time

* **Dedicated Plot Window:** Plots are displayed in a separate, dedicated window for better visibility and analysis, accessible via a "Show Plots" button.

* **Input Validation:** Ensures user-provided parameters are valid.

* **Logging:** Comprehensive logging of application events, warnings, and errors to a `/data/logs` directory.

## System Requirements

* Python 3.8+

* Operating System: Windows, macOS, or Linux (with PyQt5 support)

## Installation

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/automatomator/2d_robot_arm_sim.git
    ```

2.  **Create and activate a virtual environment (recommended):**

    ```bash
    python -m venv .venv
    # On Windows:
    .venv\Scripts\activate
    # On macOS/Linux:
    source .venv/bin/activate
    ```

3.  **Install dependencies:**

    ```bash
    pip install -e .
    ```

    The `-e .` installs the project in "editable" mode, so changes to the source code are reflected immediately without re-installation.

## Usage

1.  **Run the application:**
    With your virtual environment activated, navigate to the project root and run:

    ```bash
    python gui_app.py
    ```

    Alternatively, if you installed with `pip install -e .`, you might be able to run it directly from your terminal after activation (depending on your `entry_points` configuration in `setup.py`):

    ```bash
    rob_arm_sim_gui
    ```

    __packaging still not perfect, could use some inputs about it,__

2.  **Input Parameters:**
    The GUI allows you to input the following parameters:

    * **Link 1 (L1) [mm]:** Length of the first robotic arm link.

    * **Link 2 (L2) [mm]:** Length of the second robotic arm link.

    * **Base X [mm]:** X-coordinate of the arm's base.

    * **Base Y [mm]:** Y-coordinate of the arm's base.

    * **Circle Center X (a) [mm]:** X-coordinate of the target circle's center.

    * **Circle Center Y (b) [mm]:** Y-coordinate of the target circle's center.

    * **Circle Radius (r) [mm]:** Radius of the target circle.

    * **Speed (v) [mm/s]:** Desired uniform speed of the end effector along the circle.

    * **Time Step (dt) [s]:** Simulation time step (determines granularity of calculation).

    * **Anim. Interval [ms]:** Interval between animation frames in milliseconds (can be a float for finer control).

3.  **Run Simulation:**
    Click the "Run Simulation" button.

    * If any part of the circle is unreachable, an "Out of Bounds" error will be displayed.

    * If successful, the animation will start, and the "Show Plots" button will become active.

4.  **View Plots:**
    Click the "Show Plots" button to open a separate window displaying the joint angles, angular velocities, and angular accelerations over time.

## Project Structure Overview

The project is organized into the following key directories and files:

* **`rob_arm_sim/`**: This is the core Python package containing the main logic.

    * `arm.py`: Defines the `RobotArm` class, handling kinematics (forward, inverse, and reachability).

    * `simulation.py`: Contains the logic for simulating the arm's movement along a circular path and calculating dynamic properties.

    * `plotting.py`: Provides functions for generating the static data plots.

* **`data/`**: Stores application-generated data.

    * `logs/`: Contains log files generated during application execution.

* **`tests/`**: Holds unit tests for verifying the correctness of core modules.

* **`gui_app.py`**: The main entry point for the graphical user interface application.

* **`setup.py`**: Defines how the project is packaged and installed.

* **`requirements.txt`**: Lists all Python dependencies required for the project.

* **`LICENSE`**: Specifies the licensing terms for the project.

* **`README.md`**: This documentation file.

* **`.gitignore`**: Specifies files and directories that Git should ignore.

## Logging

The application uses a logging system to record events, warnings, and errors. All log outputs are directed to files within the `data/logs/` directory in the project root.

The main log file is `data/logs/robot_arm_simulator.log`.

### Log Levels:

* **INFO:** General information about the application's flow and major events.

* **WARNING:** Indicates potential issues that don't prevent the application from running.

* **ERROR:** Indicates errors that occurred but were handled (e.g., invalid user input, points out of reach).

* **CRITICAL:** Indicates severe errors that might affect the stability or execution.

* **DEBUG:** Detailed information useful for debugging (enabled internally in core modules).

You can inspect these log files to understand the application's behavior and diagnose issues.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

## Contact

For questions or feedback, please contact Utkarsh Singh at automatomator@gmail.com
