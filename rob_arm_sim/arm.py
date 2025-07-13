import numpy as np
import logging
logger = logging.getLogger(__name__)

class OutOfReachError(Exception):
    """custom exception for when a target point is out of the arm's reach"""
    pass

class RobotArm:
    """
    represents a 2-DOF robotic arm moving in 2D space

    The arm has 2 fixed links, L1 and L2, with specified lengths
    L1 is fixed at a base point (base_x, base_y)
    L2's end acts as the end effector
    """

    def __init__(self, L1: float, L2: float, base_x: float, base_y: float):
        """
        Initialises the robot arm with link length and base coords

        Args: 
            L1 (float): length of the first link
            L2 (float): length of the second link
            base_x (float): X-coord of the arm's fixed base
            base_y (float): Y-coord of the arm's fixed base
        """
        if L1<=0 or L2<=0:
            raise ValueError("Link lengths must be positive")
        
        self.L1=L1
        self.L2=L2
        self.base_x=base_x
        self.base_y=base_y

        logger.info(f"RobotArm initialised: L1={L1}, L2={L2}, Base=({base_x},{base_y}) ")

    def forward_kinematics(self, theta1: float, theta2: float) -> tuple[float,float]:
        """
        Calculate the (x,y) coordinates of the end effector given joint angles

        Args:
            theta1 (float): Angle of the first link with respect to the horizontal, in radians
            theta2 (float): Angle of the second link relative to the first link, in radians

        Returns:
            tuple[float, float]: A tuple (end_effector_x, end_effector_y) repping the end effector's position    
        """

        #intermediate joint
        joint_x = self.base_x + self.L1 * np.cos(theta1)
        joint_y = self.base_y + self.L1 * np.sin(theta1)

        #end effector pos
        end_effector_x = joint_x + self.L2 * np.cos(theta1+theta2)
        end_effector_y = joint_y + self.L2 * np.sin(theta1+theta2)

        logger.debug(f"FK: theta1={np.degrees(theta1):.2f}deg, theta2={np.degrees(theta2):.2f}deg -> EE=({end_effector_x:.2f},{end_effector_y:.2f})")

        return end_effector_x, end_effector_y
    
    def is_reachable(self, target_x: float, target_y: float) -> bool:
        """
        checks if a given target point (x,y) is within the arm's reach

        Args:
            target_x (float): X_coord of the target
            target_y (float): Y_coord of the target

        Returns:
            bool: True if the point is reachable, else false
        """

        #dist from the base to the target
        dx = target_x - self.base_x
        dy = target_y - self.base_y
        dist2tar = np.sqrt(dx**2 + dy**2)

        #max reach is L1+L2
        max_reach = self.L1 + self.L2

        #min reach is |L1-L2|
        min_reach = abs(self.L1-self.L2)

        logger.debug(f"Target ({target_x:.2f},{target_y:.2f}) is reachable. Distance={dist2tar:.2f}")

        return min_reach <= dist2tar <= max_reach
    
    def inv_kinematics(self, target_x: float, target_y: float) -> tuple[float,float]:
        """
        Calculates the joint angles (theta1,theta2) required to reach a target position (x,y)
        uses law of cosines. It returns one valid solution, the one where the "elbow" is up

        Args:
            target_x(float): X-coord of the target point
            target_y(float): Y-coord of the target point

        Returns:
            tuple[float,float]: (theta1,theta2) in radians

        Raises:
            OutOfReachError: if the target point is outside the arm's reachable workspace    
        """

        if not self.is_reachable(target_x,target_y):
            logger.warning(f"Attempted IK for unreachable point ({target_x:.2f}, {target_y:.2f}).")
            raise OutOfReachError(f"Target point ({target_x: .2f}, {target_y: .2f}) is out of arm's reach")
        
        dx = target_x - self.base_x
        dy = target_y - self.base_y
        D = np.sqrt(dx**2 + dy**2)

        #calculate theta2 using cosine law
        #cos(theta2)=(D^2-L1^2-L2^2)/(2*L1*L2), using cos(pi-x) as -cos(x)

        # Clamp argument to acos to prevent NaN due to floating point inaccuracies
        # Ensure the value is within [-1, 1]

        arg_theta2 = (D**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        arg_theta2 = np.clip(arg_theta2, -1.0, 1.0)
        theta2 = np.arccos(arg_theta2)

        #calculate theta1
        #alpha=angle of the target from the base
        alpha= np.arctan2(dy,dx)

        # beta = angle between the line from base to target and the first link
        # cos(beta) = (L1^2+D^2-L2^2)/(2*L1*D)
        arg_beta = (self.L1**2 + D**2 - self.L2**2) / (2 * self.L1 * D)
        arg_beta = np.clip(arg_beta, -1.0, 1.0)
        beta = np.arccos(arg_beta)

        theta1=alpha-beta

        logger.debug(f"IK: Target ({target_x:.2f},{target_y:.2f}) -> theta1={np.degrees(theta1):.2f}deg, theta2={np.degrees(theta2):.2f}deg")

        return theta1, theta2
    
    def get_joint_positions(self,theta1: float, theta2: float) -> tuple[float, float]:
        """
        Returns the (x,y) coordinates of the intermediate joint and end effector.

        Args:
            theta1 (float): Angle of the first link with respect to the horizontal (radians).
            theta2 (float): Angle of the second link relative to the first link (radians).

        Returns:
            tuple[float, float, float, float]: (joint_x, joint_y, end_effector_x, end_effector_y)
        """
        joint_x = self.base_x + self.L1 * np.cos(theta1)
        joint_y = self.base_y + self.L1 * np.sin(theta1)
        end_effector_x, end_effector_y = self.forward_kinematics(theta1, theta2)
        return joint_x, joint_y, end_effector_x, end_effector_y