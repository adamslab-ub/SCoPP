"""This code contains algorithm parameters to be loaded for use by the SCoPP algorithm. To specify parameters different
from the default, the user must create an instance of the classes below, initialize it with their own values, and feed
it into the algorithm as an argument.
"""


class plots:
    """
    # Plot settings
        plot_cell_boundary_size: int - width of the cell outlines
        plot_robot_path_size: int - width of the robot paths
        plot_robot_size: int - size of the robots
        plot_cell_size: int - size of the cell fillings

    """

    def __init__(
            self,
            robot_path_size=1,
            robot_size=20,
            cell_boundary_size=1,
            cell_size=10
    ):
        self.robot_path_size = robot_path_size
        self.robot_size = robot_size
        self.cell_boundary = cell_boundary_size
        self.cell_size = cell_size


class algorithm:
    """
    # Algorithm settings
        bias_factor: float - conflict resolution bias
        sampling_rate: int - frequency at which the robots' position is updated in simulation when calculating area
            surveyed over time
        leaf_size: int - determines the minimum number of cells required to use KD Tree before brute force is used
        partition_tolerance: float - tolerance for iterative clustering
        partition_max_iter: int - maximum number of iterations to run the iterative clustering
    """

    def __init__(
            self,
            bias_factor=0.5,
            sampling_rate=1 / 10,
            leaf_size=10,
            partition_tolerance=None,
            partition_max_iter=10,
            conflict_resolution_mode="bias",
            planner="nn",
    ):
        self.bias_factor = bias_factor
        self.sampling_rate = sampling_rate
        self.leaf_size = leaf_size
        self.partition_tolerance = partition_tolerance
        self.partition_max_iter = partition_max_iter
        self.conflict_resolution_mode = conflict_resolution_mode
        self.planner = planner
