# ROB_ARM/gui_app.py
import sys
import numpy as np
import matplotlib
import logging
import os # For path manipulation

# Ensure data/logs directory exists
LOG_DIR = os.path.join(os.getcwd(), 'data', 'logs')
os.makedirs(LOG_DIR, exist_ok=True) # Create if it doesn't exist

# Configure logging
LOG_FILE = os.path.join(LOG_DIR, 'robot_arm_simulator.log')
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    handlers=[
                        logging.FileHandler(LOG_FILE),
                        logging.StreamHandler(sys.stdout) # Also log to console
                    ])
logger = logging.getLogger(__name__)

# Set the Matplotlib backend for PyQt5
try:
    matplotlib.use('Qt5Agg')
    logger.info("Matplotlib backend set to Qt5Agg.")
except ImportError:
    logger.warning("Qt5Agg backend not available. Attempting TkAgg.")
    try:
        matplotlib.use('TkAgg')
        logger.info("Matplotlib backend set to TkAgg.")
    except ImportError:
        logger.error("No suitable Matplotlib backend found. Animation might not display.")
        # Adding a GUI popup for critical error
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.critical(None, "Backend Error", "No suitable Matplotlib backend found. Animation might not display correctly.")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QLineEdit, QMessageBox, QFrame, QSizePolicy, QSpacerItem
)
from PyQt5.QtCore import Qt, QSize

# Import your core simulation logic
from rob_arm_sim.arm import RobotArm, OutOfReachError
from rob_arm_sim.simulation import simulate_circular_path
from rob_arm_sim.plotting import plot_sim_data_on_axes


class StaticPlotsWindow(QMainWindow):
    """A separate window to display the static simulation plots."""
    def __init__(self, sim_data: dict):
        super().__init__()
        self.setWindowTitle("Simulation Data Plots")
        self.setGeometry(200, 200, 800, 800) # Initial size and position for the plots window
        logger.info("StaticPlotsWindow created.")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.fig_plots = Figure(figsize=(8, 8)) # Give a good default size for separate window
        self.canvas_plots = FigureCanvas(self.fig_plots)
        self.canvas_plots.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas_plots.setMinimumSize(600, 600) # Ensure it expands nicely
        layout.addWidget(self.canvas_plots)

        # Create subplots and plot data
        self.fig_plots.clear()
        ax_angles = self.fig_plots.add_subplot(311)
        ax_velocity = self.fig_plots.add_subplot(312)
        ax_acceleration = self.fig_plots.add_subplot(313)
        
        try:
            plot_sim_data_on_axes(sim_data, ax_angles, ax_velocity, ax_acceleration)
            self.fig_plots.tight_layout() # Apply tight layout after plotting
            self.canvas_plots.draw_idle()
            logger.info("Static plots generated successfully.")
        except Exception as e:
            logger.error(f"Error generating static plots: {e}", exc_info=True)
            QMessageBox.critical(self, "Plotting Error", f"Failed to generate static plots: {e}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2-DOF Robotic Arm Simulator")
        self.setGeometry(100, 100, 1500, 950) # Increased initial window size

        self.arm = None
        self.sim_data = None
        self.ani = None # To hold the animation object and prevent garbage collection
        self.static_plots_window = None # Reference to the separate plots window
        logger.info("MainWindow initialized.")

        self._init_ui()

    def _init_ui(self):
        logger.info("Initializing UI components.")
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Left Panel: Controls and Input ---
        control_frame = QFrame()
        control_frame.setFrameShape(QFrame.StyledPanel)
        control_layout = QVBoxLayout(control_frame)
        control_layout.setContentsMargins(15, 15, 15, 15)

        # Helper for creating input rows with validation
        self.input_fields = {} # Store QLineEdit references for easy access
        def add_input_row(layout, label_text, default_value, key, input_type=float):
            hbox = QHBoxLayout()
            label = QLabel(label_text)
            label.setFixedWidth(160) # Consistent label width
            line_edit = QLineEdit(str(default_value))
            # Basic validation: ensure only numbers can be typed
            if input_type == float:
                from PyQt5.QtGui import QDoubleValidator
                line_edit.setValidator(QDoubleValidator())
            elif input_type == int:
                from PyQt5.QtGui import QIntValidator
                line_edit.setValidator(QIntValidator())
            
            hbox.addWidget(label)
            hbox.addWidget(line_edit)
            layout.addLayout(hbox)
            self.input_fields[key] = line_edit

        add_input_row(control_layout, "Link 1 (L1) [mm]:", 1200, 'L1')
        add_input_row(control_layout, "Link 2 (L2) [mm]:", 800, 'L2')
        add_input_row(control_layout, "Base X [mm]:", 0, 'base_x')
        add_input_row(control_layout, "Base Y [mm]:", 0, 'base_y')
        add_input_row(control_layout, "Circle Center X (a) [mm]:", 0, 'a')
        add_input_row(control_layout, "Circle Center Y (b) [mm]:", 1500, 'b')
        add_input_row(control_layout, "Circle Radius (r) [mm]:", 200, 'r')
        add_input_row(control_layout, "Speed (v) [mm/s]:", 100, 'v')
        add_input_row(control_layout, "Time Step (dt) [s]:", 0.01, 'dt')
        
        # --- MODIFIED: Anim. Interval now accepts float ---
        add_input_row(control_layout, "Anim. Interval [ms]:", 20.0, 'anim_interval', float)
        # --------------------------------------------------

        control_layout.addStretch(1)

        run_button = QPushButton("Run Simulation")
        run_button.setMinimumHeight(40)
        run_button.clicked.connect(self._run_simulation)
        control_layout.addWidget(run_button)
        logger.info("Run Simulation button added.")

        self.show_plots_button = QPushButton("Show Plots")
        self.show_plots_button.setMinimumHeight(40)
        self.show_plots_button.clicked.connect(self._show_static_plots)
        self.show_plots_button.setEnabled(False) # Disable until simulation runs successfully
        control_layout.addWidget(self.show_plots_button)
        logger.info("Show Plots button added (initially disabled).")

        main_layout.addWidget(control_frame, 1)

        # --- Right Panel: Animation Plot ---
        plots_frame = QFrame()
        plots_frame.setFrameShape(QFrame.StyledPanel)
        plots_layout = QVBoxLayout(plots_frame)

        self.fig_anim = Figure()
        self.canvas_anim = FigureCanvas(self.fig_anim)
        self.canvas_anim.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas_anim.setMinimumSize(400, 400)
        plots_layout.addWidget(self.canvas_anim, 1)
        logger.info("Animation canvas added to UI.")

        main_layout.addWidget(plots_frame, 2)
        logger.info("UI initialization complete.")

    def _get_input_value(self, key, input_type=float):
        """Helper to get and validate input field values."""
        try:
            value_str = self.input_fields[key].text()
            if not value_str:
                logger.warning(f"Input field '{key}' is empty.")
                raise ValueError(f"'{key}' cannot be empty.")
            
            # This line handles conversion to float or int based on input_type
            value = input_type(value_str)

            logger.debug(f"Input '{key}' read as: {value}")
            return value
        except ValueError as e:
            logger.error(f"Validation error for input '{key}': {e}")
            raise ValueError(f"Invalid input for '{key}': {e}. Please enter a valid number.")

    def _run_simulation(self):
        logger.info("Run Simulation button clicked. Starting simulation process.")
        # Stop any existing animation before starting a new one
        if self.ani:
            self.ani.event_source.stop()
            self.ani = None
            logger.info("Stopped existing animation.")
        
        # Close static plots window if open
        if self.static_plots_window and self.static_plots_window.isVisible():
            self.static_plots_window.close()
            self.static_plots_window = None
            logger.info("Closed existing static plots window.")

        # Clear existing animation plot contents
        self.fig_anim.clear()
        self.canvas_anim.draw_idle()
        logger.info("Cleared animation plot.")
        
        self.show_plots_button.setEnabled(False)
        logger.info("Show Plots button disabled.")

        try:
            # Read and validate input parameters
            params = {
                'L1': self._get_input_value('L1'),
                'L2': self._get_input_value('L2'),
                'base_x': self._get_input_value('base_x'),
                'base_y': self._get_input_value('base_y'),
                'a': self._get_input_value('a'),
                'b': self._get_input_value('b'),
                'r': self._get_input_value('r'),
                'v': self._get_input_value('v'),
                'dt': self._get_input_value('dt'),
                
                # --- MODIFIED: anim_interval is retrieved as float ---
                'anim_interval': self._get_input_value('anim_interval', float)
                # ----------------------------------------------------
            }
            logger.info(f"Input parameters read: {params}")

            if params['L1'] <= 0 or params['L2'] <= 0:
                raise ValueError("Link lengths (L1, L2) must be positive.")
            if params['r'] <= 0:
                raise ValueError("Circle radius (r) must be positive.")
            if params['v'] <= 0:
                raise ValueError("Speed (v) must be positive.")
            if params['dt'] <= 0:
                raise ValueError("Time step (dt) must be positive.")
            if params['anim_interval'] <= 0:
                raise ValueError("Animation interval must be positive.")


            self.arm = RobotArm(L1=params['L1'], L2=params['L2'],
                                base_x=params['base_x'], base_y=params['base_y'])
            logger.info(f"RobotArm initialized with L1={self.arm.L1}, L2={self.arm.L2}, base=({self.arm.base_x},{self.arm.base_y}).")

            # Run the core simulation logic
            self.sim_data = simulate_circular_path(
                self.arm,
                circle_center_x=params['a'],
                circle_center_y=params['b'],
                circle_radius=params['r'],
                speed_v=params['v'],
                dt=params['dt']
            )
            logger.info("Simulation data generated successfully.")
            QMessageBox.information(self, "Simulation Status", "Simulation successful. All points on the circle are reachable.")
            self.show_plots_button.setEnabled(True)
            logger.info("Show Plots button enabled.")

            # --- Configure and start animation ---
            ax_anim = self.fig_anim.add_subplot(111)
            ax_anim.set_aspect('equal', adjustable='box')
            ax_anim.set_title("2-DOF Robotic Arm Animation")
            ax_anim.grid(True)
            ax_anim.set_xlabel("X-coordinate (mm)")
            ax_anim.set_ylabel("Y-coordinate (mm)")
            logger.info("Animation plot axes configured.")

            # DYNAMICALLY SET X AND Y LIMITS FOR ANIMATION PLOT
            max_reach = self.arm.L1 + self.arm.L2
            
            min_x_pts = [params['base_x']]
            max_x_pts = [params['base_x']]
            min_y_pts = [params['base_y']]
            max_y_pts = [params['base_y']]

            min_x_pts.append(params['base_x'] - max_reach)
            max_x_pts.append(params['base_x'] + max_reach)
            min_y_pts.append(params['base_y'] - max_reach)
            max_y_pts.append(params['base_y'] + max_reach)

            min_x_pts.append(params['a'] - params['r'])
            max_x_pts.append(params['a'] + params['r'])
            min_y_pts.append(params['b'] - params['r'])
            max_y_pts.append(params['b'] + params['r'])
            
            x_min_tight = min(min_x_pts)
            x_max_tight = max(max_x_pts)
            y_min_tight = min(min_y_pts)
            y_max_tight = max(max_y_pts)

            padding_factor = 0.15
            x_range = x_max_tight - x_min_tight
            y_range = y_max_tight - y_min_tight

            if x_range < 1: x_range = max_reach * 2 if max_reach > 0 else 1000
            if y_range < 1: y_range = max_reach * 2 if max_reach > 0 else 1000
            
            if x_range == 0: x_range = 1
            if y_range == 0: y_range = 1
            
            ax_anim.set_xlim(x_min_tight - x_range * padding_factor, x_max_tight + x_range * padding_factor)
            ax_anim.set_ylim(y_min_tight - y_range * padding_factor, y_max_tight + y_range * padding_factor)
            logger.info(f"Animation plot limits set to X:[{ax_anim.get_xlim()[0]:.2f}, {ax_anim.get_xlim()[1]:.2f}], Y:[{ax_anim.get_ylim()[0]:.2f}, {ax_anim.get_ylim()[1]:.2f}].")

            self.fig_anim.tight_layout()
            self.canvas_anim.draw_idle()
            logger.info("Animation plot redrawn with initial state.")

            # Initial plot elements for animation
            ax_anim.plot(self.arm.base_x, self.arm.base_y, 'ro', markersize=8, label='Arm Base')
            circle_patch = plt.Circle((params['a'], params['b']), params['r'], color='grey', fill=False, linestyle='--', label='Target Circle')
            ax_anim.add_patch(circle_patch)

            line1, = ax_anim.plot([], [], 'b-', lw=3, label='Link 1')
            line2, = ax_anim.plot([], [], 'g-', lw=3, label='Link 2')
            joint_point, = ax_anim.plot([], [], 'bo', markersize=5, label='Joint')
            end_effector_point, = ax_anim.plot([], [], 'go', markersize=7, label='End Effector')
            trajectory_line, = ax_anim.plot([], [], 'r--', lw=1, alpha=0.6, label='End Effector Trajectory')
            ax_anim.legend(loc='upper right')
            logger.info("Animation elements initialized.")

            def init_anim():
                line1.set_data([], [])
                line2.set_data([], [])
                joint_point.set_data([], [])
                end_effector_point.set_data([], [])
                trajectory_line.set_data([], [])
                return line1, line2, joint_point, end_effector_point, trajectory_line

            def update_anim(frame):
                t1 = self.sim_data['theta1'][frame]
                t2 = self.sim_data['theta2'][frame]
                joint_x, joint_y, ee_x, ee_y = self.arm.get_joint_positions(t1, t2)
                line1.set_data([self.arm.base_x, joint_x], [self.arm.base_y, joint_y])
                line2.set_data([joint_x, ee_x], [joint_y, ee_y])
                joint_point.set_data([joint_x], [joint_y])
                end_effector_point.set_data([ee_x], [ee_y])
                trajectory_line.set_data(self.sim_data['end_effector_x'][:frame+1], self.sim_data['end_effector_y'][:frame+1])
                return line1, line2, joint_point, end_effector_point, trajectory_line

            # The 'interval' parameter of FuncAnimation can accept a float
            self.ani = animation.FuncAnimation(
                self.fig_anim, update_anim, frames=len(self.sim_data['time']),
                init_func=init_anim, blit=True, interval=params['anim_interval'], repeat=True
            )
            logger.info(f"Animation started with {len(self.sim_data['time'])} frames and interval {params['anim_interval']}ms.")

        except ValueError as e:
            logger.error(f"Input Error during simulation: {e}")
            QMessageBox.critical(self, "Input Error", f"Invalid input: {e}")
            self._clear_all_plots()
        except OutOfReachError as e:
            logger.error(f"Simulation Error: Out of Bounds: {e}")
            QMessageBox.critical(self, "Simulation Error: Out of Bounds", f"The arm cannot reach the entire path. {e}")
            self._clear_all_plots()
        except Exception as e:
            logger.critical(f"An unexpected error occurred during simulation: {e}", exc_info=True)
            QMessageBox.critical(self, "Unexpected Error", f"An unexpected error occurred: {e}")
            self._clear_all_plots()

    def _show_static_plots(self):
        logger.info("Show Plots button clicked.")
        if self.sim_data is None:
            logger.warning("Attempted to show plots before simulation data was available.")
            QMessageBox.warning(self, "No Data", "Please run the simulation first to generate data for plots.")
            return
        
        if self.static_plots_window and self.static_plots_window.isVisible():
            self.static_plots_window.activateWindow()
            self.static_plots_window.raise_()
            logger.info("Static plots window already open, brought to front.")
        else:
            try:
                self.static_plots_window = StaticPlotsWindow(self.sim_data)
                self.static_plots_window.show()
                logger.info("New static plots window opened.")
            except Exception as e:
                logger.error(f"Failed to open static plots window: {e}", exc_info=True)
                QMessageBox.critical(self, "Error", f"Could not open plots window: {e}")

    def _clear_all_plots(self):
        """Helper to clear both animation and static plots and disable button."""
        logger.info("Clearing all plots and resetting state.")
        if self.ani:
            self.ani.event_source.stop()
            self.ani = None
            logger.info("Stopped animation during plot clear.")
        self.fig_anim.clear()
        self.canvas_anim.draw_idle()
        self.show_plots_button.setEnabled(False)
        logger.info("Animation plot cleared and 'Show Plots' button disabled.")
        
        if self.static_plots_window and self.static_plots_window.isVisible():
            self.static_plots_window.close()
            self.static_plots_window = None
            logger.info("Closed static plots window during plot clear.")


if __name__ == "__main__":
    logger.info("Application starting.")
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())