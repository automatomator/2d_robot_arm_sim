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
    git clone [https://github.com/yourusername/rob_arm_simulator.git](https://github.com/yourusername/rob_arm_simulator.git)
    cd rob_arm_simulator
    ```
    (Replace `yourusername` with your actual GitHub username if you create a repo)

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

## Project Structure

ROB_ARM/
├── data/
│   └── logs/               # Directory for application logs
├── rob_arm_sim/
│   ├── __init__.py
│   ├── arm.py              # Robot arm kinematics (forward, inverse, reachability)
│   ├── plotting.py         # Functions for generating static plots
│   └── simulation.py       # Core simulation logic for circular path tracing
├── tests/
│   ├── __init__.py
│   └── test_arm.py
├── .gitignore
├── gui_app.py              # Main GUI application
├── LICENSE                 # Project license file
├── README.md               # This README file
├── requirements.txt        # List of Python dependencies
└── setup.py                # Project setup and packaging file

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

## Contributing

(Optional section: Add guidelines for how others can contribute to your project)

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

## Contact

For questions or feedback, please contact [Your Name] at [automatomator@gmail.com].