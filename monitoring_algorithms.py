import numpy as np
import matplotlib.pyplot as plt
from pyDOE import lhs
from shapely.geometry import Point, Polygon
from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans
import sklearn.neighbors as skn
from math import sqrt, tan, radians
#from Swarm_Surveillance.SCoPP 
import latlongcartconv as lc
import copy
import time
import pickle
import networkx as nx
import six #Added for a mlrose error that keeps popping up
import sys #Ditto
sys.modules['sklearn.externals.six'] = six #Ditto
import mlrose_hiive
#from Swarm_Surveillance.SCoPP 
import SCoPP_settings
import random
from file_utils import create_dir
from matplotlib.patches import Rectangle
#from descartes import PolygonPatch


class QLB:
    """
        Base class for the Quick, Load-Balanced (SCoPP) Monitoring algorithm.
        This class's attributes contain details of the partitions created. See "Class Attributes" below for a list of
        other attributes attributes.

        Parameters
        ----------
        number_of_robots: int - number of robots intended to survey the area
        environment: class object - environment object which contains information pertaining to the environment. See
            "environments.py" for details.
        plot (optional): string or tuple of strings - choose between plotting none, all, or the final phase(s) of the
            algorithm, by passing either no argument, "full", or "partial", respectively. Note that choosing the "full"
            plot option will increase computation time. Additionally, the user may specify whether or not to display the
            coverage over time plot using "area".
                Example: To see only the final plot and area surveyed per second, the argument passed would be
                ("partial", "area")
        plot_settings (optional): "plots" class object - specify the size of points, lines, etc. of the plot(s). See
            the "plots" class in "SCoPP_settings.py" for details.
        algorithm_settings (optional): "algorithm" class object - "algorithm" class object - specify the algorithm
            settings. See the "algorithm" class in "SCoPP_settings.py" for details.

        Useful attributes
        ----------
        area_covered_over_time: list - area surveyed as a function of time. Use for plotting and visualization of
            results

        Returns
        ----------
        None: Initializes SCoPP environment instance. To run the algorithm, the user must call the "run" method. See "run"
        below for details.
    """

    def __init__(
            self,
            environment,
            number_of_robots=None,
            priority_points = 1, #Added 4.12.22
            plot=False,
            plot_settings=SCoPP_settings.plots(),
            algorithm_settings=SCoPP_settings.algorithm()
    ):
        # Initialize environment parameters
        if len(environment.starting_position) > 1:
            geographical_starting_positions = environment.starting_position
        else:
            geographical_starting_positions = []
            for agent in range(number_of_robots):
                geographical_starting_positions.extend(environment.starting_position)
        geographical_boundary_points = environment.boundary_points
        geographical_priority_points = environment.priority_points #Added 6.26.2021 for Priority_CPP
        geographical_boundary = Polygon(environment.boundary_points)
        min_lon = geographical_boundary.bounds[0]
        min_lat = geographical_boundary.bounds[1]
        max_lon = geographical_boundary.bounds[2]
        max_lat = geographical_boundary.bounds[3]
        while len(geographical_priority_points) != (priority_points): #Added 7.20.2021 for Scalability Analysis- Comment when adding user points only
            random_lon = random.uniform(min_lon, max_lon)
            random_lat = random.uniform(min_lat, max_lat)
            point = Point(random_lon,random_lat)
            if geographical_boundary.contains(point):
                geographical_priority_points.append([random_lon,random_lat])
                #print([random_lon,random_lat])
        self.robot_FOV = environment.robot_FOV
        self.robot_operating_height = environment.robot_operating_height
        self.robot_velocity = environment.robot_velocity
        geographical_geo_fencing_zones = environment.geo_fencing_holes
        self.save_path = create_dir(environment.save_path)

        # Initialize reference frame
        self.robot_initial_positions_in_cartesian = []
        self.boundary_points_in_cartesian = []
        self.boundary_points_in_geographical = []
        self.robot_initial_positions_in_geographical = []
        self.priority_points_in_cartesian = [] #Added 6.26.2021 for Priority_CPP
        self.priority_points_in_geographical = [] #Added 6.26.2021 for Priority_CPP
        origin = [0, 0]
        origin[1] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[0], np.transpose(geographical_boundary_points)[0])))
        origin[0] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[1], np.transpose(geographical_boundary_points)[1])))
        self.origin = origin
        if geographical_geo_fencing_zones:
            self.geographical_geo_fencing_zones_in_cartesian = [[] for item in
                                                                range(len(geographical_geo_fencing_zones))]
            self.geographical_geo_fencing_zones_in_geographical = [[] for item in
                                                                   range(len(geographical_geo_fencing_zones))]
        else:
            self.geographical_geo_fencing_zones_in_cartesian = None
            self.geographical_geo_fencing_zones_in_geographical = None
        for robot, position in enumerate(geographical_starting_positions):
            self.robot_initial_positions_in_geographical.append([position[1], position[0]])
        for point in geographical_boundary_points:
            self.boundary_points_in_geographical.append([point[1], point[0]])
        for point in geographical_priority_points:  #Added 6.26.2021 for Priority_CPP
            self.priority_points_in_geographical.append([point[1], point[0]])  #Added 6.26.2021 for Priority_CPP
        if geographical_geo_fencing_zones:
            for list_id, fence_list in enumerate(geographical_geo_fencing_zones):
                for point in fence_list:
                    self.geographical_geo_fencing_zones_in_geographical[list_id].append([point[1], point[0]])
        #print("line 127")
        # Initialize method variables
        self.optimal_paths = None
        if number_of_robots is None:
            self.number_of_partitions = len(environment.starting_position)
        else:
            self.number_of_partitions = number_of_robots
        self.dominated_cells_x = [[] for robot in range(self.number_of_partitions)]
        self.dominated_cells_y = [[] for robot in range(self.number_of_partitions)]
        self.robot_WOV = tan(radians(self.robot_FOV / 2)) * self.robot_operating_height * 2  # in meters
        self.cell_size = int(self.robot_WOV)
        self.cell_wall_interval = 3
        if self.cell_size % 2 != 0:
            self.cell_size -= 1
        self.cell_area = self.cell_size ** 2  # in meters squared
        self.robot_assignment_information = None

        # Algorithm settings
        self.bias_factor = algorithm_settings.bias_factor
        self.sampling_rate = algorithm_settings.sampling_rate
        self.leaf_size = algorithm_settings.leaf_size
        self.conflict_resolution_mode = algorithm_settings.conflict_resolution_mode
        self.planner = algorithm_settings.planner
        self.tests = algorithm_settings.tests
        if algorithm_settings.partition_tolerance is None:
            self.partition_tolerance = 1 / 8 * self.cell_size
        else:
            self.partition_tolerance = algorithm_settings.partition_tolerance
        self.partition_max_iter = algorithm_settings.partition_max_iter

        # Initialize computation time attributes
        self.time_for_conversion = 0
        self.time_for_partitioning = 0
        self.time_for_discretization = 0
        self.time_for_conflict_resolution = 0
        self.time_for_path_planning = 0
        self.total_time = 0

        # Initialize run data attributes for statistics
        self.data_tasks_per_robot = None
        self.data_distance_travelled_per_robot = None
        self.data_completion_time_per_robot = None
        self.data_total_mission_completion_time = None
        self.data_computation_time_without_path_planning = None
        self.data_computation_time_with_path_planning = None

        # Initialize variables for plotting
        self.rough_partitioning_x = None
        self.rough_partitioning_y = None
        self.cluster_centers = None
        self.conflict_cells_x = None
        self.conflict_cells_y = None
        self.final_partitioning_x = None
        self.final_partitioning_y = None
        self.data_tasks_per_robot = None

        # Plot settings
        self.plot = plot
        self.plot_cell_boundary_size = plot_settings.cell_boundary
        self.plot_robot_path_size = plot_settings.robot_path_size
        self.plot_robot_size = plot_settings.robot_size
        self.plot_cell_size = plot_settings.cell_size
        self.partition_colors = None

        # Initialize other useful information
        self.total_survey_area = None
        self.area_per_robot = None
        self.area_covered_over_time = []
        self.area_covered_over_time_time_vector = []

    def run(self, info=False):
        """
        Runs the SCoPP-Monitoring algorithm in its entirety.
        Parameters
        ----------
        info (optional): string, tuple of strings - choose what kind of information of the algorithm results to display
        in the run window. Takes any of the following strings as inputs:
            "verbose" - display all available relevant information
            "time" - display computation times per phase of the algorithm
            "mission" - display information pertaining to the performance of the resulting paths determined for the
                robots
            "area" - display area allocation information
            ""
        Returns
        ----------
        final_paths_in_geographical: list of lists of tuples - the final, ordered list of waypoints for each robot,
            beginning at their starting position.
        """
        t0 = time.time()
        # Convert from geographical to cartesian coordinates
        time_stamp = time.time()
        #print("working here")
        coordinate_converter = lc.LLCCV(self.origin)
        #print("conversion done here")
        for item in self.robot_initial_positions_in_geographical:
            self.robot_initial_positions_in_cartesian.append(coordinate_converter.get_cartesian(item))
        
        for item in self.boundary_points_in_geographical:
            self.boundary_points_in_cartesian.append(coordinate_converter.get_cartesian(item))
        #print(self.boundary_points_in_cartesian)
        
        for item in self.priority_points_in_geographical: #Added 6.26.2021 for Priority_CPP
            self.priority_points_in_cartesian.append(coordinate_converter.get_cartesian(item)) #Added 6.26.2021 for Priority_CPP
        if self.geographical_geo_fencing_zones_in_geographical:
            for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones_in_geographical):
                for item in fence_list:
                    self.geographical_geo_fencing_zones_in_cartesian[list_id].append(
                        coordinate_converter.get_cartesian(item))
        self.time_for_conversion += time.time() - time_stamp
        priority_points_in_cartesian = self.priority_points_in_cartesian #Added 6.28.2021 for Priority_CPP
        #print(self.geographical_geo_fencing_zones_in_cartesian)
        #ewerwerwe
        self.priority_points_in_cartesian_x = [] #Added 6.29.2021 for Priority_CPP
        self.priority_points_in_cartesian_y = [] #Added 6.29.2021 for Priority_CPP
        for points in priority_points_in_cartesian: #Added 6.29.2021 for Priority_CPP
            self.priority_points_in_cartesian_x.append(points[0]) #Added 6.29.2021 for Priority_CPP
            self.priority_points_in_cartesian_y.append(points[1]) #Added 6.29.2021 for Priority_CPP
        #print("priority points:",priority_points_in_cartesian) #Added 6.28.2021 for Priority_CPP

        # Commence discretization of the area
        time_stamp = time.time()
        cell_space, cells = self.discretize_massive_area()
        self.time_for_discretization += time.time() - time_stamp

        # Commence partitioning of the discretized area
        time_stamp = time.time()
        if "ablate-partition" not in self.tests:
            cells_as_dict, cell_space_as_dict, robot_assignment_information = self.partition_area(cell_space, cells) 
        else:
            cells_as_dict, cell_space_as_dict, robot_assignment_information = self.ablate_partition_area(cell_space, cells)
        self.time_for_partitioning += time.time() - time_stamp

        # Commence conflict resolution
        time_stamp = time.time()
        robot_assignment_information, waypoints_for_robots, priority_waypoints_for_robots = \
            self.resolve_conflicts(cells_as_dict, cell_space_as_dict, robot_assignment_information,
                                   self.conflict_resolution_mode, priority_points_in_cartesian) #Added priority stuff on 6.28.2021 for priority_CPP
        self.time_for_conflict_resolution += time.time() - time_stamp
         
        # Commence path planning
        timestamp = time.time()
        paths, distances = self.find_optimal_paths(self.planner, robot_assignment_information, waypoints_for_robots, priority_waypoints_for_robots) #Ditto
        self.time_for_path_planning = time.time() - timestamp

        # Calculate surveillance rate
        self.calculate_surveillance_rate(distances)

        # Convert from cartesian to geographical
        time_stamp = time.time()
        final_paths_in_geographical = []
        for path in paths:
            entry = []
            for item in path:
                entry.append(coordinate_converter.get_geographic(item))
            final_paths_in_geographical.append(entry)
        self.time_for_conversion += time.time() - time_stamp
        self.total_time = time.time() - t0
        self.area_per_robot = np.array(self.data_tasks_per_robot) * self.cell_area

        # Display algorithm attributes
        if info:
            if info == "verbose":
                # Display computation time attributes
                print("Computation times for various operations (in seconds):")
                print("     Conversions:", self.time_for_conversion)
                print("     Initial partitioning:", self.time_for_partitioning)
                print("     Discretization:", self.time_for_discretization)
                print("     Conflict resolution:", self.time_for_conflict_resolution)
                print("     Path planning:", self.time_for_path_planning)
                print("     Entire algorithm:", self.total_time)
                # Display data attributes for statistics
                print("Statistics:")
                print("     Tasks per robot:", self.data_tasks_per_robot)
                print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
                print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
                print("     Mission completion time of the swarm:", self.data_total_mission_completion_time, "seconds")
                # Display other environment information
                print("Area allocation information:")
                print("     Cell width:", self.cell_size, "meters")
                print("     Cell area:", self.cell_area, "square meters")
                print("     Total area to be surveyed:", self.total_survey_area, "square meters")
                print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")

            else:
                if info == "time":
                    # Display computation time attributes
                    print("Computation times for various operations (in seconds):")
                    print("     Conversions:", self.time_for_conversion)
                    print("     Initial partitioning:", self.time_for_partitioning)
                    print("     Discretization:", self.time_for_discretization)
                    print("     Conflict resolution:", self.time_for_conflict_resolution)
                    print("     Path planning:", self.time_for_path_planning)
                    print("     Entire algorithm:", self.total_time)

                if info == "mission":
                    # Display data attributes for statistics
                    print("Statistics:")
                    print("     Tasks per robot:", self.data_tasks_per_robot)
                    print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
                    print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
                    print("     Mission completion time of the swarm:", self.data_total_mission_completion_time,
                          "seconds")

                if info == "area":
                    # Display other environment information
                    print("Area allocation information:")
                    print("     Cell width:", self.cell_size, "meters")
                    print("     Cell area:", self.cell_area, "square meters")
                    print("     Total area to be surveyed:", self.total_survey_area, "square meters")
                    print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")
        # Plot results
        #if self.plot:
        #self.plot_partitioning(self.planner) #Added Self.planner on 6.29.2021
        
        info_dict = dict(
            time_travelled = self.data_completion_time_per_robot,
            final_paths_latlon=final_paths_in_geographical,
            priority_points_xy=priority_points_in_cartesian,
            #psr_average=self.averagepsurvey,
            robot_paths=paths,
            time=dict(
                conversions=self.time_for_conversion,
                initial_partitioning=self.time_for_partitioning,
                path_planning=self.time_for_path_planning,
                total_comp=self.total_time,
            ),
            mission=dict(
                completion_time=self.data_total_mission_completion_time,
            ),
        )

        # Return final list of paths per robot
        return info_dict #Added for scalability testing on 7.30.2021

    def discretize_area(self):
        """Discretization phase: distretizes the surveyable area and finds cells which lie within the bounds specified
        by the environment parameters
        """
        x_max = np.NINF
        y_max = np.NINF
        for item in self.robot_initial_positions_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        for item in self.boundary_points_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        if self.geographical_geo_fencing_zones_in_cartesian:
            boundary_polygon = Polygon(
                self.boundary_points_in_cartesian,
                holes=self.geographical_geo_fencing_zones_in_cartesian
            )
        else:
            boundary_polygon = Polygon(self.boundary_points_in_cartesian)
        self.total_survey_area = boundary_polygon.area
        print("Total Survey Area:", self.total_survey_area)
        #print("") #Added on 6.29.2021 for priority_CPP

        # Create initial grid zone
        grid_space = np.zeros((y_max, x_max)) + np.NINF
        column_1 = 0
        column_2 = column_1 + self.cell_size
        row_1 = 0
        row_2 = row_1 + self.cell_size
        cells_within_boundary_x = []
        cells_within_boundary_y = []
        while True:
            cell_boundary = []
            for i in range(self.cell_size + 1):
                for j in range(self.cell_size + 1):
                    if Point((i + row_1, j + column_1)).within(boundary_polygon):
                        if i == 0 or i == self.cell_size or j == 0 or j == self.cell_size:
                            cell_boundary.append((i + row_1, j + column_1))
            if len(cell_boundary) == self.cell_size * 4:
                cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
                for count in range(len(cell_boundary_points_x)):
                    grid_space[cell_boundary_points_x[count], cell_boundary_points_y[count]] = -1
                    cells_within_boundary_x.append(cell_boundary_points_x[count])
                    cells_within_boundary_y.append(cell_boundary_points_y[count])
            column_1 = int(column_1 + self.cell_size)
            column_2 = int(column_2 + self.cell_size)
            if column_2 >= len(grid_space[0]):
                row_1 = int(row_1 + self.cell_size)
                row_2 = int(row_2 + self.cell_size)
                column_1 = 0
                column_2 = column_1 + self.cell_size
            if row_2 >= len(grid_space):
                break
        return [cells_within_boundary_x, cells_within_boundary_y], grid_space

    def discretize_massive_area(self):
        """Discretization phase: distretizes the surveyable area and finds cells which lie within the bounds specified
        by the environment parameters
        """
        x_max = np.NINF
        y_max = np.NINF
        for item in self.robot_initial_positions_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        for item in self.boundary_points_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        if self.geographical_geo_fencing_zones_in_cartesian:
            boundary_polygon = Polygon(
                self.boundary_points_in_cartesian,
                holes=self.geographical_geo_fencing_zones_in_cartesian
            )
        else:
            boundary_polygon = Polygon(self.boundary_points_in_cartesian)
        self.boundary_polygon_for_plot = boundary_polygon
        self.total_survey_area = boundary_polygon.area
        # boundary_polygon.plot()
        # plt.show()
        titlesize = 18  # fontsize of the title
        axeslabelsize = 15  # fontsize of the x and y labels
        xticklabelsize = 13  # fontsize of the tick labels
        yticklabelsize = 13  # fontsize of the tick labels
        legendsize = 15  # fontsize of the legend
        #
        plt.rc('axes', titlesize=titlesize)  # fontsize of the title
        plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
        plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
        plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
        plt.rc('legend', fontsize=legendsize)  # fontsize of the legend
        plt.axis("equal")
        plt.plot(*boundary_polygon.exterior.xy, linewidth=12)
        #plt.show()
        #
        # import geopandas as gpd
        # p = gpd.GeoSeries(boundary_polygon)
        # p.plot()
        # plt.show()
        # plt.axis("equal")
        # titlesize = 18  # fontsize of the title
        # axeslabelsize = 15  # fontsize of the x and y labels
        # xticklabelsize = 13  # fontsize of the tick labels
        # yticklabelsize = 13  # fontsize of the tick labels
        # legendsize = 15  # fontsize of the legend
        #
        # plt.rc('axes', titlesize=titlesize)  # fontsize of the title
        # plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
        # plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
        # plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
        # plt.rc('legend', fontsize=legendsize)  # fontsize of the legend
        # plt.axis("equal")
        # plt.show()
        #print("Total survey area: " + str(self.total_survey_area) + " square meters") #Should add "in square meters" to the end of this-Changed it on 6.29.2021
        #print("") #Added on 6.29.2021 for priority_CPP

        # Create initial grid zone
        cells = np.zeros((int(x_max / self.cell_size), int(y_max / self.cell_size))) + np.NINF
        #print(cells)
        cell_squares_within_boundary_x = []
        cell_squares_within_boundary_y = []
        for row in range(len(cells)):
            for column in range(len(cells[0])):
                cell_boundary = []
                for i in range(self.cell_size + 1):
                    if i % self.cell_wall_interval == 0 or i == self.cell_size:
                        for j in range(self.cell_size + 1):
                            if j % self.cell_wall_interval == 0 or j == self.cell_size:
                                if Point((i + column * self.cell_size, j + row * self.cell_size)).within(
                                        boundary_polygon):
                                    if i == 0 or i == self.cell_size or j == 0 or j == self.cell_size:
                                        cell_boundary.append((i + column * self.cell_size, j + row * self.cell_size))
                if self.cell_size % self.cell_wall_interval == 0:
                    if len(cell_boundary) == (self.cell_size / self.cell_wall_interval) * 4:
                        cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
                        for count in range(len(cell_boundary_points_x)):
                            cell_squares_within_boundary_x.append(cell_boundary_points_x[count])
                            cell_squares_within_boundary_y.append(cell_boundary_points_y[count])
                else:
                    if len(cell_boundary) == (np.floor(self.cell_size / self.cell_wall_interval) + 1) * 4:
                        cell_boundary_points_x, cell_boundary_points_y = zip(*cell_boundary)
                        for count in range(len(cell_boundary_points_x)):
                            cell_squares_within_boundary_x.append(cell_boundary_points_x[count])
                            cell_squares_within_boundary_y.append(cell_boundary_points_y[count])
        if len(cells) < self.number_of_partitions:
            print("Allocatable cells less than total number of robots provided")
            raise
        else:
        # Iterate over the list of cell boundary points that are within the boundary
            # Iterate over the list of cell boundary points that are within the boundary
            for x, y in zip(cell_squares_within_boundary_x, cell_squares_within_boundary_y):
                # Define the corners of the square around the point
                half_size = self.cell_size / 2
                square_corners = [(x - half_size, y - half_size), (x - half_size, y + half_size),
                                  (x + half_size, y + half_size), (x + half_size, y - half_size), 
                                  (x - half_size, y - half_size)]  # Close the polygon by repeating the first point
                # Create a Shapely Polygon
                square = Polygon(square_corners)
                # Extract the x and y coordinates of the polygon corners
                x, y = square.exterior.xy
                # Use matplotlib to fill the polygon area
                plt.fill(x, y, alpha=0.5, edgecolor='blue', facecolor='lightblue')

            plt.axis("equal")  # Ensure aspect ratio remains equal
            #plt.show()
            plt.scatter(cell_squares_within_boundary_x, cell_squares_within_boundary_y, marker="s", linewidths=1)
            plt.axis("equal")
            titlesize = 18  # fontsize of the title
            axeslabelsize = 15  # fontsize of the x and y labels
            xticklabelsize = 13  # fontsize of the tick labels
            yticklabelsize = 13  # fontsize of the tick labels
            legendsize = 15  # fontsize of the legend
            
            plt.rc('axes', titlesize=titlesize)  # fontsize of the title
            plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
            plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
            plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
            plt.rc('legend', fontsize=legendsize)  # fontsize of the legend
            plt.axis("equal")
            #plt.show()
            return [cell_squares_within_boundary_x, cell_squares_within_boundary_y], cells

    def partition_area(self, cell_space, cells):
        """
        Partitioning phase: divides surveyable area are into equally sized areas; one for each robot.
        """
        cell_squares_within_boundary_x = cell_space[0]
        cell_squares_within_boundary_y = cell_space[1]
        cells_within_boundary_x = []
        cells_within_boundary_y = []
        for row, _ in enumerate(cells):
            for column, _ in enumerate(cells[0]):
                cells_within_boundary_x.append(row)
                cells_within_boundary_y.append(column)
        clustering_set = np.transpose([cell_squares_within_boundary_x, cell_squares_within_boundary_y])
        kmeans = MiniBatchKMeans(n_clusters=self.number_of_partitions, max_iter=self.partition_max_iter,
                                 tol=self.partition_tolerance)
        cluster_indices = kmeans.fit_predict(clustering_set)
        self.cluster_centers = [kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1]]
        self.partition_colors = lhs(3, samples=self.number_of_partitions)
        self.partition_colors = np.round(self.partition_colors, decimals=1)
        robot_assignment_information = [[] for robot in range(self.number_of_partitions)]
        robot_initial_positions_copy = self.robot_initial_positions_in_cartesian.copy()
        self.rough_partitioning_x = [[] for robot in range(self.number_of_partitions)]
        self.rough_partitioning_y = [[] for robot in range(self.number_of_partitions)]
        minimum_distance = [[np.Inf] for robot in range(self.number_of_partitions)]
        cell_space_as_dict = dict()
        cells_as_dict = dict()
        for robot_id in range(self.number_of_partitions):
            for point_index, point in enumerate(clustering_set):
                if cluster_indices[point_index] != robot_id:
                    continue
                else:
                    row = point[0]
                    column = point[1]
                    self.rough_partitioning_x[robot_id].append(row)
                    self.rough_partitioning_y[robot_id].append(column)
                    cell_space_as_dict[column, row] = robot_id
                    if row % self.cell_size == 0 and column % self.cell_size == 0:
                        cells_as_dict[column, row] = robot_id
                        for pos in robot_initial_positions_copy:
                            if np.linalg.norm(
                                    [row - pos[0] + 0.0001,
                                     column - pos[1] + 0.0001]) < minimum_distance[robot_id]:
                                minimum_distance[robot_id] = np.linalg.norm(
                                    [row - pos[0] + 0.0001, column - pos[1] + 0.0001])
                                assignment_information = [pos[0], pos[1], row + self.cell_size / 2,
                                                          column + self.cell_size / 2,
                                                          round(minimum_distance[robot_id] / self.cell_size)]
            robot_assignment_information[robot_id] = assignment_information
            if len(robot_initial_positions_copy):
                robot_initial_positions_copy.remove([assignment_information[0], assignment_information[1]])

        return cells_as_dict, cell_space_as_dict, robot_assignment_information
    
    def ablate_partition_area(self, cell_space, cells):
        '''
        Partitioning phase with ablation: divides surveyable area are into equally sized areas; one for each robot.
        Each robot will receive an ~equal amount of cells, they will be partitioned not according to kmeans but N to S and E to W.
        '''
        cell_squares_within_boundary_x = cell_space[0]
        cell_squares_within_boundary_y = cell_space[1]
        cells_within_boundary_x = []
        cells_within_boundary_y = []
        for row, _ in enumerate(cells):
            for column, _ in enumerate(cells[0]):
                cells_within_boundary_x.append(row)
                cells_within_boundary_y.append(column)
        clustering_set = np.transpose([cell_squares_within_boundary_x, cell_squares_within_boundary_y])
        self.partition_colors = lhs(3, samples=self.number_of_partitions)
        self.partition_colors = np.round(self.partition_colors, decimals=1)
        robot_assignment_information = [[] for robot in range(self.number_of_partitions)]
        robot_initial_positions_copy = self.robot_initial_positions_in_cartesian.copy()
        self.rough_partitioning_x = [[] for robot in range(self.number_of_partitions)]
        self.rough_partitioning_y = [[] for robot in range(self.number_of_partitions)]
        minimum_distance = [[np.Inf] for robot in range(self.number_of_partitions)]
        cells_per_robot = np.array([len(clustering_set) / self.number_of_partitions] * self.number_of_partitions)
        any_cells_remaining = len(clustering_set) % self.number_of_partitions
        if any_cells_remaining:
            cells_per_robot = np.floor(cells_per_robot)
            cells_per_robot[:any_cells_remaining] += 1
            print(f"cells_per_robot: {cells_per_robot}")
        cell_space_as_dict = dict()
        cells_as_dict = dict()
        robot_cell_indices = dict()
        temp_robot_id = 0
        for cell in range(len(clustering_set)):
            robot_cell_indices[cell] = temp_robot_id
            cells_per_robot[temp_robot_id] -= 1
            if cells_per_robot[temp_robot_id] == 0:
                temp_robot_id += 1

        cluster_centers_x, cluster_centers_y = [], []
        temp_sum_x, temp_sum_y = 0, 0
        for robot_id in range(self.number_of_partitions):
            for point_index, point in enumerate(clustering_set):
                if robot_cell_indices[point_index] != robot_id:
                    continue
                else:
                    row = point[0]
                    column = point[1]
                    temp_sum_x += row
                    temp_sum_y += column
                    self.rough_partitioning_x[robot_id].append(row)
                    self.rough_partitioning_y[robot_id].append(column)
                    cell_space_as_dict[column, row] = robot_id
                    if row % self.cell_size == 0 and column % self.cell_size == 0:
                        cells_as_dict[column, row] = robot_id
                        for pos in robot_initial_positions_copy:
                            if np.linalg.norm(
                                    [row - pos[0] + 0.0001,
                                    column - pos[1] + 0.0001]) < minimum_distance[robot_id]:
                                minimum_distance[robot_id] = np.linalg.norm(
                                    [row - pos[0] + 0.0001, column - pos[1] + 0.0001])
                                assignment_information = [pos[0], pos[1], row + self.cell_size / 2,
                                                        column + self.cell_size / 2,
                                                        round(minimum_distance[robot_id] / self.cell_size)]
            # Updating centroid information
            centroid_x = temp_sum_x / len(self.rough_partitioning_x[robot_id])
            centroid_y = temp_sum_y / len(self.rough_partitioning_y[robot_id])
            temp_sum_x, temp_sum_y = 0, 0
            cluster_centers_x.append(centroid_x)
            cluster_centers_y.append(centroid_y)
            robot_assignment_information[robot_id] = assignment_information
            if len(robot_initial_positions_copy):
                robot_initial_positions_copy.remove([assignment_information[0], assignment_information[1]])
        self.cluster_centers = [cluster_centers_x, cluster_centers_y]

        return cells_as_dict, cell_space_as_dict, robot_assignment_information

    def resolve_conflicts(self, cells_as_dict, cell_space_as_dict, robot_assignment_information, mode, priority_waypoints_for_robots):
        """
        Conflict resolution phase: resolves conflicts at the boundaries between initial partition regions based on
        robot states.
        """
        # FIND INHERENT CONFLICTS
        self.conflict_cells_x = []
        self.conflict_cells_y = []
        self.data_tasks_per_robot = [0 for robot in range(self.number_of_partitions)]
        non_dominated_cells = []
        for corner in cells_as_dict:
            cell_boundary_values = []
            cell_boundary_points_y = []
            cell_boundary_points_x = []
            for i in range(self.cell_size + 1):
                for j in range(self.cell_size + 1):
                    try:
                        cell_boundary_values.append(cell_space_as_dict[i + corner[0], j + corner[1]])
                        cell_boundary_points_x.append(i + corner[1])
                        cell_boundary_points_y.append(j + corner[0])
                    except KeyError:
                        continue
            if len(cell_boundary_values) >= self.cell_size / self.cell_wall_interval * 4:
                if not all(elem == cell_boundary_values[0] for elem in cell_boundary_values):
                    self.conflict_cells_y.extend(cell_boundary_points_y)
                    self.conflict_cells_x.extend(cell_boundary_points_x)
                    occupied_partitions = []
                    for partition_number in cell_boundary_values:
                        if partition_number not in occupied_partitions:
                            occupied_partitions.append(partition_number)
                    non_dominated_cells.append([[corner[0], corner[1]], occupied_partitions])
                else:
                    self.data_tasks_per_robot[int(cell_boundary_values[0])] += 1
                    self.dominated_cells_x[int(cell_boundary_values[0])].append(int(corner[1] + self.cell_size / 2))
                    self.dominated_cells_y[int(cell_boundary_values[0])].append(int(corner[0] + self.cell_size / 2))

        # Get initial partitioning distribution information
        # self.data_tasks_per_robot, non_dominated_cells = self.sum_cells_per_robot(cell_space_as_dict)
        if mode == "bias":
            # Add bias to robots based on robot states
            distance_bias = np.transpose(robot_assignment_information.copy())[4] * self.bias_factor

            # Resolve conflicts (nondominated cells)
            self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
            self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
            for cell in non_dominated_cells:
                column = int(cell[0][0])
                row = int(cell[0][1])
                lower_cell_count_robot = cell[1][0]
                for region in cell[1]:
                    if self.data_tasks_per_robot[int(region)] + distance_bias[int(region)] < \
                            self.data_tasks_per_robot[int(lower_cell_count_robot)] +\
                            distance_bias[int(lower_cell_count_robot)]:
                        lower_cell_count_robot = region
                cell_space_as_dict[cell_space_as_dict[cell[0][0], cell[0][1]]] = lower_cell_count_robot
                self.final_partitioning_y[int(lower_cell_count_robot)].append(int(column + self.cell_size / 2))
                self.final_partitioning_x[int(lower_cell_count_robot)].append(int(row + self.cell_size / 2))
                self.data_tasks_per_robot[int(lower_cell_count_robot)] += 1
        else:
            # Resolve conflicts using random walk (nondominated cells)
            self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
            self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
            for cell in non_dominated_cells:
                row = int(cell[0][0] - self.cell_size / 2)
                column = int(cell[0][1] - self.cell_size / 2)
                robot = np.random.randint(0, self.number_of_partitions, )
                self.final_partitioning_y[robot].append(int(column + self.cell_size / 2))
                self.final_partitioning_x[robot].append(int(row + self.cell_size / 2))
                self.data_tasks_per_robot[robot] += 1

        # SAVE FINAL PARTITIONS
        self.priority_final_partitioning_x = copy.deepcopy(self.final_partitioning_x) #Added on 6.29.2021 for priority_CPP
        self.priority_final_partitioning_y = copy.deepcopy(self.final_partitioning_y) #Added on 6.29.2021 for priority_CPP
        final_partitions = [[] for robot in range(self.number_of_partitions)]
        priority_final_partitions = [[] for robot in range(self.number_of_partitions)] #Added on 6.28.2021 for priority_CPP
        allocated_priority_points = []
        self.easier_priority_list = [[] for robot in range(self.number_of_partitions)]
        points_added = 0
        for robot_id in range(self.number_of_partitions):
            final_partitions[robot_id] = [self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id]] 
            priority_final_partitions[robot_id] = [self.priority_final_partitioning_x[robot_id], self.priority_final_partitioning_y[robot_id]] #Added on 6.28.2021 for priority_CPP
            for ppoints in range(len(priority_final_partitions[robot_id][0])): 
                self.easier_priority_list[robot_id].append([priority_final_partitions[robot_id][0], priority_final_partitions[robot_id][1]])

            for ppoints in range(len(priority_waypoints_for_robots)): #Added on 7.11.2021 for priority_CPP
                for points in range(len(final_partitions[robot_id][0])): #Added on 7.11.2021 for priority_CPP
                    if abs(priority_waypoints_for_robots[ppoints][0] - final_partitions[robot_id][0][points]) <= (self.cell_size*2) \
                        and abs(priority_waypoints_for_robots[ppoints][1] - final_partitions[robot_id][1][points]) <= (self.cell_size*2): #Added on 7.11.2021 for priority_CPP

                        if [priority_waypoints_for_robots[ppoints][0],priority_waypoints_for_robots[ppoints][1]] not in self.easier_priority_list[robot_id]: #Added on 7.11.2021 for priority_CPP

                            if [priority_waypoints_for_robots[ppoints][0],priority_waypoints_for_robots[ppoints][1]] not in allocated_priority_points: #Added on 7.11.2021 for priority_CPP
                                try:
                                    priority_final_partitions[robot_id][0].insert(0,priority_waypoints_for_robots[ppoints][0]) #Added on 6.29.2021 for priority_CPP
                                    priority_final_partitions[robot_id][1].insert(0,priority_waypoints_for_robots[ppoints][1]) #Added on 6.29.2021 for priority_CPP
                                    allocated_priority_points.append([priority_waypoints_for_robots[ppoints][0],priority_waypoints_for_robots[ppoints][1]])
                                    points_added += 1
                                    #print("Priority waypoint" , [priority_waypoints_for_robots[ppoints][0],priority_waypoints_for_robots[ppoints][1]] , "added to robot", robot_id) #Added on 7.11.2021 for priority_CPP
                                except IndexError: 
                                    continue 

            # try:
            #     priority_final_partitions[robot_id][0].insert(0,priority_waypoints_for_robots[robot_id][0]) #Added on 6.29.2021 for priority_CPP
            #     priority_final_partitions[robot_id][1].insert(0,priority_waypoints_for_robots[robot_id][1]) #Added on 6.29.2021 for priority_CPP
            # except IndexError: #Added on 6.28.2021 for priority_CPP
            #     continue #Added on 6.28.2021 for priority_CPP
        

        #print("")
        #print("priority waypoints for robots:",priority_final_partitions) #Added on 6.28.2021 for priority_CPP
        #print("Total Priority Points added: ", points_added,"/", len(self.priority_points_in_cartesian))

        return robot_assignment_information, final_partitions, priority_final_partitions

    def find_optimal_paths(self, planner, robot_assignment_information, waypoints_for_robots, priority_waypoints_for_robots):
        """
        Path planning phase: find the optimal path for each robot based on their assigned list of cells
        """
        self.optimal_paths = [[] for robot in range(self.number_of_partitions)]
        waypoint_distances = [[] for robot in range(self.number_of_partitions)]
        #print("") #Added on 6.29.2021 for priority_CPP
        #print("waypoints for robots:",waypoints_for_robots)
        if planner == "nn":
            for robot, cell_list in enumerate(waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                 (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='brute', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                #print('total time for robot',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)
                self.averagepsurvey = None

        elif planner == "SCoPP_Surveillance":
            goal_time = 100
            reshaped_priority_list = [[] for robot in range(self.number_of_partitions)]
            priority_points_visited = [0 for robot in range(self.number_of_partitions)]
            priority_points_total = [0 for robot in range(self.number_of_partitions)]
            priority_points_robot = [[] for robot in range(self.number_of_partitions)]
            psurvey = []
            for robot, cell_list in enumerate(waypoints_for_robots):
                time_elapsed = 0
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                for ppoints in range(len(priority_waypoints_for_robots[robot][0])): 
                    reshaped_priority_list[robot].append([priority_waypoints_for_robots[robot][0][ppoints], priority_waypoints_for_robots[robot][1][ppoints]])
                for ppoints in range(len(self.priority_points_in_cartesian)): #Added 7.10.2021 for attempted varied path 
                    if [self.priority_points_in_cartesian[ppoints][0] , self.priority_points_in_cartesian[ppoints][1]] in reshaped_priority_list[robot]:
                        task_list.append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        priority_points_total[robot] += 1
                        priority_points_robot[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                print('total priority points for robot',robot+1,':',priority_points_total[robot])
                        
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                 (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break
                    if current_position in priority_points_robot[robot] and (time_elapsed < goal_time):
                        try:
                            priority_points_robot[robot].remove(current_position)
                            priority_points_visited[robot] += 1
                        except:
                            pass
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                    time_elapsed += distances[task_list.index(current_position)][1] / self.robot_velocity #Added 4.13.22
                if priority_points_total[robot] != 0:
                    priority_surveillance_rate = priority_points_visited[robot] / priority_points_total[robot]
                    psurvey.append(priority_surveillance_rate)
                    # print('total time for robot',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)
                    print('priority surveillance rate for robot',robot+1,'is:',priority_surveillance_rate)
            self.averagepsurvey = np.average(psurvey)
            print('average surveillance rate:',self.averagepsurvey)
        elif planner == "random_walk":
            for robot, cell_list in enumerate(waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()

                self.optimal_paths[robot].append(current_position)
                radius = 1
                while len(task_list) > 0:
                    for task in random.sample(task_list, len(task_list)):
                        distance = sqrt((current_position[0] - task[0]) ** 2 +
                                        (current_position[1] - task[1]) ** 2)
                        if distance <= radius:
                            waypoint_distances[robot].append(distance)
                            task_list.remove(task)
                            current_position = task
                            self.optimal_paths[robot].append(task)
                            radius = 0
                            continue
                    radius += 1

        elif planner == "tsp:brute":
            for robot, path in enumerate(waypoints_for_robots):
                shortest_distance = np.Inf
                start = self.robot_assignment_information[robot][0:2]
                path = np.concatenate((np.array([start]), np.transpose(path).tolist())).tolist()
                import itertools
                all_permutations = itertools.permutations(path)
                for path_permutation in all_permutations:
                    if path_permutation[0] == start:
                        distance = 0
                        distances = []
                        current_position = start
                        for next_position in path_permutation[1:]:
                            distance += sqrt((current_position[0] - next_position[0]) ** 2 +
                                             (current_position[1] - next_position[1]) ** 2)
                            distances.append(sqrt((current_position[0] - next_position[0]) ** 2 +
                                                  (current_position[1] - next_position[1]) ** 2))
                            current_position = next_position
                        if distance < shortest_distance:
                            self.optimal_paths[robot] = list(path_permutation)
                            waypoint_distances[robot] = distances

        elif planner == "tsp:ga":
            optimal_paths_init = [None for robot in range(self.number_of_partitions)]
            for robot, partition in enumerate(waypoints_for_robots):
                start = self.robot_assignment_information[robot][0:2]
                coords_list = np.concatenate((np.array([start]), np.transpose(partition)))
                coord_graph = nx.Graph()
                node = 0
                dist_list = []
                for coord in coords_list:
                    coord_graph.add_node(node, location=coord)
                    node += 1
                for coord_from in coord_graph.nodes:
                    for coord_to in coord_graph.nodes:
                        if coord_from == coord_to:
                            continue
                        x2 = coord_graph.nodes[coord_to]["location"][0]
                        x1 = coord_graph.nodes[coord_from]["location"][0]
                        y2 = coord_graph.nodes[coord_to]["location"][1]
                        y1 = coord_graph.nodes[coord_from]["location"][1]
                        if sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) == 0:
                            continue
                        dist_list.append((coord_from, coord_to, sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)))
                fit_dists = mlrose_hiive.TravellingSales(distances=dist_list)
                problem_fit = mlrose_hiive.TSPOpt(length=node, fitness_fn=fit_dists, maximize=False)
                best_state, best_fitness = mlrose_hiive.genetic_alg(problem_fit, pop_size=400, mutation_prob=0.2,
                                                              max_attempts=5, max_iters=100)
                entry = []
                for node in best_state:
                    entry.append(tuple(coord_graph.nodes[node]["location"]))
                optimal_paths_init[robot] = entry
            self.optimal_paths = []
            for robot, path in enumerate(optimal_paths_init):
                ind = path.index(tuple(start))
                path = [path[(i + ind) % len(path)]
                        for i, x in enumerate(path)]
                self.optimal_paths.append(path)
            for robot, optimal_path in enumerate(self.optimal_paths):
                distances = []
                current_position = start
                for next_position in optimal_path[1:]:
                    distances.append(sqrt((current_position[0] - next_position[0]) ** 2 +
                                          (current_position[1] - next_position[1]) ** 2))
                    current_position = next_position
                waypoint_distances[robot] = distances

        elif planner == "Priority_CPP": ##Adding planner for Priority Coverage Path Planning-PCPP
            reshaped_priority_list = [[] for robot in range(self.number_of_partitions)]
            time_elapsed_list = []
            #print(self.priority_points_in_cartesian)
            priority_task_list = [[] for robot in range(self.number_of_partitions)]
            priority_task_index = [0 for robot in range(self.number_of_partitions)]
            for robot, cell_list in enumerate(priority_waypoints_for_robots):
                time_elapsed = 0 #Recording the time it takes for all priority points to be surveyed
                for ppoints in range(len(priority_waypoints_for_robots[robot][0])): 
                    reshaped_priority_list[robot].append([priority_waypoints_for_robots[robot][0][ppoints], priority_waypoints_for_robots[robot][1][ppoints]])
                #print(reshaped_priority_list)
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                priority_task_list[robot].append(current_position)
                for ppoints in range(len(self.priority_points_in_cartesian)): #Added 7.10.2021 for attempted varied path 
                    if [self.priority_points_in_cartesian[ppoints][0] , self.priority_points_in_cartesian[ppoints][1]] in reshaped_priority_list[robot]:
                        priority_task_list[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        #print(priority_task_list[robot])
                        priority_task_index[robot] += 1
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                 (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break
                    while priority_task_index[robot] != 0:
                        # print("robot",robot,"priority points remaining",priority_task_index[robot])
                        self.optimal_paths[robot].append(current_position)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            priority_task_list[robot])
                        distances, indices = nbrs.kneighbors(priority_task_list[robot])
                        waypoint_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                        next_position = priority_task_list[robot][indices[priority_task_list[robot].index(current_position)][1]]
                        if next_position == current_position:
                            priority_task_index[robot] -= 1
                            continue
                        else:
                            task_list.remove(current_position)
                            priority_task_list[robot].remove(current_position)
                            current_position = next_position
                            priority_task_index[robot] -= 1
                        time_elapsed += distances[priority_task_list[robot].index(current_position)][1] / self.robot_velocity #Added 4.5.22
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                time_elapsed_list.append(time_elapsed)
                # print('robot',robot,'time to survey all priority points:',time_elapsed)
                self.averagepsurvey = np.average(time_elapsed_list)
            print('average time to survey all points',self.averagepsurvey)
       
       
        elif planner == "SCoP3_Surveillance": ##Adding planner for Priority Coverage Path Planning-PCPP
            goal_time = 100
            priority_points_visited = [0 for robot in range(self.number_of_partitions)]
            priority_points_total = [0 for robot in range(self.number_of_partitions)]
            priority_points_robot = [[] for robot in range(self.number_of_partitions)]
            psurvey = []
            reshaped_priority_list = [[] for robot in range(self.number_of_partitions)]
            time_elapsed_list = []
            #print(self.priority_points_in_cartesian)
            priority_task_list = [[] for robot in range(self.number_of_partitions)]
            priority_task_index = [0 for robot in range(self.number_of_partitions)]
            for robot, cell_list in enumerate(priority_waypoints_for_robots):
                time_elapsed = 0 #Recording the time it takes for all priority points to be surveyed
                for ppoints in range(len(priority_waypoints_for_robots[robot][0])): 
                    reshaped_priority_list[robot].append([priority_waypoints_for_robots[robot][0][ppoints], priority_waypoints_for_robots[robot][1][ppoints]])
                #print(reshaped_priority_list)
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                priority_task_list[robot].append(current_position)
                for ppoints in range(len(self.priority_points_in_cartesian)): #Added 7.10.2021 for attempted varied path 
                    if [self.priority_points_in_cartesian[ppoints][0] , self.priority_points_in_cartesian[ppoints][1]] in reshaped_priority_list[robot]:
                        priority_task_list[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        #print(priority_task_list[robot])
                        priority_task_index[robot] += 1
                        priority_points_total[robot] += 1
                        priority_points_robot[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                print('robot',robot,'has',priority_points_total[robot],'priority points')
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                    (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break
                    while priority_task_index[robot] != 0:
                        # print("robot",robot,"priority points remaining",priority_task_index[robot])
                        self.optimal_paths[robot].append(current_position)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            priority_task_list[robot])
                        distances, indices = nbrs.kneighbors(priority_task_list[robot])
                        waypoint_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                        next_position = priority_task_list[robot][indices[priority_task_list[robot].index(current_position)][1]]
                        if next_position == current_position:
                            priority_task_index[robot] -= 1
                            if time_elapsed < goal_time:
                                priority_points_visited[robot] += 1
                            continue
                        else:
                            task_list.remove(current_position)
                            priority_task_list[robot].remove(current_position)
                            current_position = next_position
                            priority_task_index[robot] -= 1
                            if time_elapsed < goal_time:
                                priority_points_visited[robot] += 1
                        time_elapsed += distances[priority_task_list[robot].index(current_position)][1] / self.robot_velocity #Added 4.5.22
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                if priority_points_total[robot] != 0:
                    priority_surveillance_rate = priority_points_visited[robot] / priority_points_total[robot]
                    psurvey.append(priority_surveillance_rate)
                    print('total time for robot',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)
                    print('priority surveillance rate for robot',robot+1,'is:',priority_surveillance_rate)
            self.averagepsurvey = np.average(psurvey)
            print('average surveillance rate:',self.averagepsurvey)

       
       
       
        elif planner == "Elapsed_Priority": #Added for Elapsed Priority CPP on 7.17.2021
            #goal_time = float(input("Visit these priority points within the next _ seconds:  "))
            goal_time = 50
            data_completion_time_per_robot = [0 for robot in range(self.number_of_partitions)]
            priority_survey_distances = [[] for robot in range(self.number_of_partitions)]
            trip_distances = [0 for robot in range(self.number_of_partitions)]
            priority_task_list = [[] for robot in range(self.number_of_partitions)]
            priority_task_index = [0 for robot in range(self.number_of_partitions)]
            initial_travel_distance = [0 for robot in range(self.number_of_partitions)]
            for robot, cell_list in enumerate(priority_waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                #priority_task_list[robot].append(current_position)
                for ppoints in range(len(self.priority_points_in_cartesian)): 
                    if self.priority_points_in_cartesian[ppoints][0] in priority_waypoints_for_robots[robot][0] \
                        and self.priority_points_in_cartesian[ppoints][1] in priority_waypoints_for_robots[robot][1]:
                        priority_task_list[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        priority_task_index[robot] += 1
                area_covered = 0
                time_elapsed = 0
                step = 0
                while time_elapsed < goal_time:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        print("Time elapsed never came close to user input time")
                        break   
                    # if step == 2:
                    #     step_size = time_elapsed
                    #     print("Robot ", (robot+1) , "step size is ", step_size)
                    if current_position in priority_task_list[robot]:
                        priority_task_list[robot].remove(current_position)
                        priority_task_index[robot] -= 1
                        print("priority point visited")
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    priority_survey_distances[robot].append(distances[task_list.index(current_position)][1])
                    trip_distances[robot] = sum(waypoint_distances[robot])
                    if initial_travel_distance[robot] == 0:
                        initial_travel_distance[robot] = distances[task_list.index(current_position)][1]
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                    while any(priority_survey_distances[robot]) and (time_elapsed<goal_time):
                        if trip_distances[robot] <= 0:
                            continue
                        elif initial_travel_distance[robot] > 0:
                            initial_travel_distance[robot] -= self.robot_velocity * self.sampling_rate
                        elif initial_travel_distance[robot] <= 0:
                            if len(priority_survey_distances[robot]) != 0:
                                priority_survey_distances[robot][0] -= self.robot_velocity * self.sampling_rate
                                if priority_survey_distances[robot][0] <= 0:
                                    priority_survey_distances[robot].pop(0)
                                    area_covered += self.cell_area
                                    #print(area_covered)
                            if len(priority_survey_distances[robot]) == 0 and data_completion_time_per_robot[robot] == 0:
                                data_completion_time_per_robot[robot] += time_elapsed
                        time_elapsed += self.sampling_rate
                    # time_elapsed += trip_distances[robot] / self.robot_velocity
                    # print('Time elapsed for robot',robot,' is ',time_elapsed)
                    step += 1
                    # print("step ",step)
                        


                print("Time elapsed for robot " +  str(robot+1) + " is currently: " + str(time_elapsed))

                if priority_task_index[robot] != 0:
                    priority_task_list[robot].append(current_position)
                while priority_task_index[robot] != 0:
                    print(priority_task_index[robot])
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        priority_task_list[robot])
                    distances, indices = nbrs.kneighbors(priority_task_list[robot])
                    waypoint_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                    next_position = priority_task_list[robot][indices[priority_task_list[robot].index(current_position)][1]]
                    try:
                        task_list.remove(current_position)
                    except ValueError:
                        print("Point already visited")
                    priority_task_list[robot].remove(current_position)
                    current_position = next_position
                    priority_task_index[robot] -= 1
                task_list.append(current_position)
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break        
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position

        elif planner == "True_Elapsed": #Added for new Elapsed Priority CPP on 11.1.2021
            # goal_time = float(input("Visit these priority points within the next _ seconds:  "))
            goal_time = 5
            elapsed_task_list = [[] for robot in range(self.number_of_partitions)]
            data_completion_time_per_robot = [0 for robot in range(self.number_of_partitions)]
            priority_survey_distances = [[] for robot in range(self.number_of_partitions)]
            trip_distances = [0 for robot in range(self.number_of_partitions)]
            trip_distances_copy = copy.deepcopy(trip_distances)
            priority_task_list = [[] for robot in range(self.number_of_partitions)]
            priority_task_index = [0 for robot in range(self.number_of_partitions)]
            initial_travel_distance = [0 for robot in range(self.number_of_partitions)]
            initial_travel_distance_copy = copy.deepcopy(initial_travel_distance)
            for robot, cell_list in enumerate(priority_waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                #priority_task_list[robot].append(current_position)
                for ppoints in range(len(self.priority_points_in_cartesian)): 
                    if self.priority_points_in_cartesian[ppoints][0] in priority_waypoints_for_robots[robot][0] \
                        and self.priority_points_in_cartesian[ppoints][1] in priority_waypoints_for_robots[robot][1]:
                        priority_task_list[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        priority_task_index[robot] += 1
                priority_task_list_copy = copy.deepcopy(priority_task_list)
                ppoints_robot = priority_task_index[robot]
                area_covered = 0
                time_elapsed = 0
                priority_survey_time = 0 
                step = 0

                #If the robot has some priority way-points, then the sub-routine will take effect
                if priority_task_index[robot] != 0:
                    priority_task_list_copy[robot].append(current_position)
                    priority_marker = priority_task_index[robot]
                    iter_time = 0
                    iter = 0
                    while priority_marker != 0:
                        # print(priority_marker)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            priority_task_list_copy[robot])
                        distances, indices = nbrs.kneighbors(priority_task_list_copy[robot])
                        priority_survey_distances[robot].append(distances[priority_task_list_copy[robot].index(current_position)][1])
                        next_position = priority_task_list_copy[robot][indices[priority_task_list_copy[robot].index(current_position)][1]]
                        try:
                            task_list.remove(current_position)
                        except ValueError:
                            print("Point already visited")
                        if iter > 0:
                            iter_time = (sum(priority_survey_distances[robot]) - priority_survey_distances[robot][0]) / self.robot_velocity
                        if iter_time > goal_time:
                            print('robot',robot,'will make it to',iter-1,'priority points out of',ppoints_robot) #Added 4.6.22
                        iter += 1
                        priority_task_list_copy[robot].remove(current_position)
                        current_position = next_position
                        priority_marker -= 1
                        
                        
                    trip_distances_copy[robot] = sum(priority_survey_distances[robot]) #Iterate these instead, so I can get a feel for how many points each robot visits
                    survey_time = trip_distances_copy[robot] / self.robot_velocity
                    # print('distance over velocity is',survey_time)
                    task_list.append(current_position) 
                    priority_survey_time = survey_time
                    time_to_fin = goal_time - priority_survey_time
                    if time_to_fin < 0:
                        print('Robot',robot,'will not make it in time')
                    else:
                        print('goal time is now', time_to_fin)
                    print('Time it will take to cover all priority points:',survey_time,'seconds')
                
                    priority_survey_time_elapsed = 0
                    time_elapsed = 0
                    while priority_survey_time_elapsed < time_to_fin:  
                        if len(task_list) == 1:
                            waypoint_distances[robot].append(
                                sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                    (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                            self.optimal_paths[robot].append(current_position)
                            task_list.remove(current_position)
                            print("Time elapsed never came close to user input time")
                            break   
                        
                        if current_position in priority_task_list[robot]:
                            priority_task_list[robot].remove(current_position)
                            priority_task_index[robot] -= 1
                            # print("priority point visited")
                        self.optimal_paths[robot].append(current_position)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            task_list)
                        distances, indices = nbrs.kneighbors(task_list)
                        waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                        priority_survey_distances[robot].append(distances[task_list.index(current_position)][1])
                        trip_distances[robot] = sum(waypoint_distances[robot])
                        if initial_travel_distance[robot] == 0:
                            initial_travel_distance[robot] = distances[task_list.index(current_position)][1]
                        next_position = task_list[indices[task_list.index(current_position)][1]]
                        task_list.remove(current_position)
                        current_position = next_position
                        priority_survey_time_elapsed += distances[task_list.index(current_position)][1] / self.robot_velocity
                        step += 1
                        # print("step ",step)
                        

                # print('distance over velocity is',priority_survey_time_elapsed)
                # print("Time elapsed for robot " +  str(robot+1) + " is currently: " + str(priority_survey_time_elapsed) + " Moving to priority points")

                if priority_task_index[robot] != 0:
                    priority_task_list[robot].append(current_position)
                while priority_task_index[robot] != 0:
                    # print('Priority points left for robot',robot+1,'is:',priority_task_index[robot])
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        priority_task_list[robot])
                    distances, indices = nbrs.kneighbors(priority_task_list[robot])
                    waypoint_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                    priority_survey_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                    priority_survey_time_elapsed += distances[priority_task_list[robot].index(current_position)][1] / self.robot_velocity
                    next_position = priority_task_list[robot][indices[priority_task_list[robot].index(current_position)][1]]
                    try:
                        task_list.remove(current_position)
                    except ValueError:
                        print("Point already visited")
                    priority_task_list[robot].remove(current_position)
                    current_position = next_position
                    priority_task_index[robot] -= 1
                task_list.append(current_position)
                # print('priority points surveyed, time elapsed is now',priority_survey_time_elapsed)

                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break        
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                # print('Total time for robot ',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)

        elif planner == "SCoPE_Surveillance": #Added for new Elapsed Priority CPP on 11.1.2021
            # goal_time = float(input("Visit these priority points within the next _ seconds:  "))
            priority_points_visited = [0 for robot in range(self.number_of_partitions)]
            priority_points_total = [0 for robot in range(self.number_of_partitions)]
            psurvey = []
            
            
            goal_time = 100
            elapsed_task_list = [[] for robot in range(self.number_of_partitions)]
            data_completion_time_per_robot = [0 for robot in range(self.number_of_partitions)]
            priority_survey_distances = [[] for robot in range(self.number_of_partitions)]
            trip_distances = [0 for robot in range(self.number_of_partitions)]
            trip_distances_copy = copy.deepcopy(trip_distances)
            priority_task_list = [[] for robot in range(self.number_of_partitions)]
            priority_task_index = [0 for robot in range(self.number_of_partitions)]
            initial_travel_distance = [0 for robot in range(self.number_of_partitions)]
            initial_travel_distance_copy = copy.deepcopy(initial_travel_distance)
            for robot, cell_list in enumerate(priority_waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                #priority_task_list[robot].append(current_position)
                for ppoints in range(len(self.priority_points_in_cartesian)): 
                    if self.priority_points_in_cartesian[ppoints][0] in priority_waypoints_for_robots[robot][0] \
                        and self.priority_points_in_cartesian[ppoints][1] in priority_waypoints_for_robots[robot][1]:
                        priority_task_list[robot].append([self.priority_points_in_cartesian[ppoints][0],self.priority_points_in_cartesian[ppoints][1]])
                        priority_task_index[robot] += 1
                        priority_points_total[robot] += 1
                priority_task_list_copy = copy.deepcopy(priority_task_list)
                ppoints_robot = priority_task_index[robot]
                area_covered = 0
                time_elapsed = 0
                priority_survey_time = 0 
                step = 0

                #If the robot has some priority way-points, then the sub-routine will take effect
                if priority_task_index[robot] != 0:
                    priority_task_list_copy[robot].append(current_position)
                    priority_marker = priority_task_index[robot]
                    iter_time = 0
                    iter = 0
                    while priority_marker != 0:
                        # print(priority_marker)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            priority_task_list_copy[robot])
                        distances, indices = nbrs.kneighbors(priority_task_list_copy[robot])
                        priority_survey_distances[robot].append(distances[priority_task_list_copy[robot].index(current_position)][1])
                        next_position = priority_task_list_copy[robot][indices[priority_task_list_copy[robot].index(current_position)][1]]
                        try:
                            task_list.remove(current_position)
                        except ValueError:
                            # print("Point already visited")
                            pass
                        iter_time = sum(priority_survey_distances[robot]) / self.robot_velocity
                        if iter_time < goal_time:
                            priority_points_visited[robot] += 1
                            # print('robot',robot,'will make it to',iter-1,'priority points out of',ppoints_robot) #Added 4.6.22
                        iter += 1
                        priority_task_list_copy[robot].remove(current_position)
                        current_position = next_position
                        priority_marker -= 1
                        
                        
                    trip_distances_copy[robot] = sum(priority_survey_distances[robot]) #Iterate these instead, so I can get a feel for how many points each robot visits
                    survey_time = trip_distances_copy[robot] / self.robot_velocity
                    # print('distance over velocity is',survey_time)
                    task_list.append(current_position) 
                    priority_survey_time = survey_time
                    time_to_fin = goal_time - priority_survey_time
                    # if time_to_fin < 0:
                    #     print('Robot',robot,'will not make it in time')
                    # else:
                    #     print('goal time is now', time_to_fin)
                    # print('Time it will take to cover all priority points:',survey_time,'seconds')
                
                    priority_survey_time_elapsed = 0
                    time_elapsed = 0
                    while priority_survey_time_elapsed < time_to_fin:  
                        if len(task_list) == 1:
                            waypoint_distances[robot].append(
                                sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                    (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                            self.optimal_paths[robot].append(current_position)
                            task_list.remove(current_position)
                            # print("Time elapsed never came close to user input time")
                            break   
                        
                        if current_position in priority_task_list[robot]:
                            priority_task_list[robot].remove(current_position)
                            priority_task_index[robot] -= 1
                            # print("priority point visited")
                        self.optimal_paths[robot].append(current_position)
                        nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                            task_list)
                        distances, indices = nbrs.kneighbors(task_list)
                        waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                        priority_survey_distances[robot].append(distances[task_list.index(current_position)][1])
                        trip_distances[robot] = sum(waypoint_distances[robot])
                        if initial_travel_distance[robot] == 0:
                            initial_travel_distance[robot] = distances[task_list.index(current_position)][1]
                        next_position = task_list[indices[task_list.index(current_position)][1]]
                        task_list.remove(current_position)
                        current_position = next_position
                        priority_survey_time_elapsed += distances[task_list.index(current_position)][1] / self.robot_velocity
                        step += 1
                        # print("step ",step)
                        

                # print('distance over velocity is',priority_survey_time_elapsed)
                # print("Time elapsed for robot " +  str(robot+1) + " is currently: " + str(priority_survey_time_elapsed) + " Moving to priority points")

                if priority_task_index[robot] != 0:
                    priority_task_list[robot].append(current_position)
                while priority_task_index[robot] != 0:
                    # print('Priority points left for robot',robot+1,'is:',priority_task_index[robot])
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        priority_task_list[robot])
                    distances, indices = nbrs.kneighbors(priority_task_list[robot])
                    waypoint_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                    priority_survey_distances[robot].append(distances[priority_task_list[robot].index(current_position)][1])
                    priority_survey_time_elapsed += distances[priority_task_list[robot].index(current_position)][1] / self.robot_velocity
                    next_position = priority_task_list[robot][indices[priority_task_list[robot].index(current_position)][1]]
                    try:
                        task_list.remove(current_position)
                    except ValueError:
                        # print("Point already visited")
                        pass
                    priority_task_list[robot].remove(current_position)
                    current_position = next_position
                    priority_task_index[robot] -= 1
                task_list.append(current_position)
                # print('priority points surveyed, time elapsed is now',priority_survey_time_elapsed)

                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break        
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                # print('Total time for robot ',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)
                if priority_points_total[robot] != 0:
                    priority_surveillance_rate = priority_points_visited[robot] / priority_points_total[robot]
                    psurvey.append(priority_surveillance_rate)
                    # print('total time for robot',robot+1,'is',sum(waypoint_distances[robot])/self.robot_velocity)
                    print('priority surveillance rate for robot',robot+1,'is:',priority_surveillance_rate)
            self.averagepsurvey = np.average(psurvey)
            # print('average surveillance rate:',self.averagepsurvey)






        # Save optimal path information to specified save path
        with open(self.save_path + "waypoints", 'wb') as f:
            pickle.dump(self.optimal_paths, f)
        return self.optimal_paths, waypoint_distances

    def plot_partitioning(self, planner): #Adding planner on 6.29.2021 to get priority points plotted
        """
        Plot algorithm run results
        """
        if "full" in self.plot:
            import pickle
            with open(self.save_path + "/rough_partitioning.txt", "wb") as fp:  # Pickling
                pickle.dump([self.rough_partitioning_x, self.rough_partitioning_y], fp)
            if planner == "Priority_CPP": #Added on 6.29.2021 for priority_CPP
                with open(self.save_path + "/priority_final_partitioning.txt", "wb") as fp:  # Pickling
                    pickle.dump([self.priority_final_partitioning_x, self.priority_final_partitioning_y], fp)
            else:
                with open(self.save_path + "/final_partitioning.txt", "wb") as fp:  # Pickling
                    pickle.dump([self.final_partitioning_x, self.final_partitioning_y], fp)
            with open(self.save_path + "/dominated_cells.txt", "wb") as fp:  # Pickling
                pickle.dump([self.dominated_cells_x, self.dominated_cells_y], fp)
            with open(self.save_path + "/conflict_cells.txt", "wb") as fp:  # Pickling
                pickle.dump([self.conflict_cells_x, self.conflict_cells_y], fp)
            with open(self.save_path + "/partition_colors.txt", "wb") as fp:  # Pickling
                pickle.dump(self.partition_colors, fp)

            with open(self.save_path + "/robot_initial_positions_in_cartesian.txt", "wb") as fp:  # Pickling
                pickle.dump(self.robot_initial_positions_in_cartesian, fp)

            with open(self.save_path + "/cluster_centers.txt", "wb") as fp:  # Pickling
                pickle.dump(self.cluster_centers, fp)

            with open(self.save_path + "/number_of_partitions.txt", "wb") as fp:  # Pickling
                pickle.dump(self.number_of_partitions, fp)

            plt.subplot(2, 2, 1)
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_boundary_size,
                            c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.scatter(self.cluster_centers[0], self.cluster_centers[1], s=self.plot_robot_size, c='black')
            plt.axis("equal")

            plt.subplot(2, 2, 2)
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_boundary_size,
                            c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.scatter(self.dominated_cells_x[robot_id], self.dominated_cells_y[robot_id], marker="s",
                            s=self.plot_cell_size,
                            c=np.ones((len(self.dominated_cells_x[robot_id]), 3)) * self.partition_colors[robot_id])
            plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
                        c="black")
            plt.axis("equal")

            plt.subplot(2, 2, 3)
            if planner == "Priority_CPP" or planner == "Elapsed_Priority" or planner == "True_Elapsed": #Added on 6.29.2021 for priority_CPP
                for robot_id in range(self.number_of_partitions):
                    plt.scatter(self.priority_final_partitioning_x[robot_id], self.priority_final_partitioning_y[robot_id], marker="s",
                                s=self.plot_cell_size,
                                c=np.ones((len(self.priority_final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
                            c="black")
                plt.axis("equal")
            else:
                for robot_id in range(self.number_of_partitions):
                    color = self.partition_colors[robot_id]  # Assuming partition_colors is correctly formatted as RGB
                    for x, y in zip(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id]):
                        # Create a square centered at (x, y)
                        rect = Rectangle((x - self.plot_cell_size / 2, y - self.plot_cell_size / 2), 
                                         self.plot_cell_size, self.plot_cell_size, 
                                         color=color, edgecolor=None)  # edgecolor=None if you don't want borders
                        plt.gca().add_patch(rect)

                # Plot conflict cells with a larger square size
                for x, y in zip(self.conflict_cells_x, self.conflict_cells_y):
                    rect = Rectangle((x - self.plot_cell_boundary_size * 1.5 / 2, y - self.plot_cell_boundary_size * 1.5 / 2),
                                     self.plot_cell_boundary_size * 1.5, self.plot_cell_boundary_size * 1.5,
                                     color="black", edgecolor=None)  # Larger and black for conflict cells
                    plt.gca().add_patch(rect)

                plt.axis("equal")  # Ensure aspect ratio remains equal
                plt.plot(*self.boundary_polygon_for_plot.exterior.xy, linewidth=12)
                data = {
                'number_of_partitions': self.number_of_partitions,
                'partition_colors': self.partition_colors,
                'final_partitioning_x': self.final_partitioning_x,
                'final_partitioning_y': self.final_partitioning_y,
                'plot_cell_size': self.plot_cell_size,
                'conflict_cells_x': self.conflict_cells_x,
                'conflict_cells_y': self.conflict_cells_y,
                'plot_cell_boundary_size': self.plot_cell_boundary_size,
                'boundary_polygon_for_plot': self.boundary_polygon_for_plot.exterior.xy
            }

            # Save data to file using pickle
            with open('plot_data.pkl', 'wb') as file:
                pickle.dump(data, file)

            # Alternatively, if you're dealing with large arrays, consider using numpy's save feature
            np.save('boundary_polygon_for_plot.npy', self.boundary_polygon_for_plot.exterior.xy)
            ax4 = plt.subplot(2, 2, 4)

            if planner == "Priority_CPP" or planner == "Elapsed_Priority" or planner == "True_Elapsed": #Added on 6.29.2021 for priority_CPP
                ax4.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
                            np.transpose(self.robot_initial_positions_in_cartesian)[1],
                            s=self.plot_robot_size, c="black")
                for robot_id in range(self.number_of_partitions):
                    ax4.scatter(self.priority_final_partitioning_x[robot_id], self.priority_final_partitioning_y[robot_id], marker="s",
                                s=self.plot_cell_size,
                                c=np.ones((len(self.priority_final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                    ax4.scatter(self.priority_points_in_cartesian_x, self.priority_points_in_cartesian_y, s = 20, c = "red")
                plt.axis("equal")
            else:
                ax4.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
                            np.transpose(self.robot_initial_positions_in_cartesian)[1],
                            s=self.plot_robot_size, c="black")
                for robot_id in range(self.number_of_partitions):
                    ax4.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
                                s=self.plot_cell_size,
                                c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.axis("equal")
        elif "partial" in self.plot:
            plt.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
                        np.transpose(self.robot_initial_positions_in_cartesian)[1],
                        s=self.plot_robot_size, c="black")
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
                            s=self.cell_size,
                            c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
        self.data_tasks_per_robot = np.round(self.data_tasks_per_robot)
        self.area_per_robot = np.array(self.data_tasks_per_robot) * (self.cell_size ** 2)

        optimal_paths_clone = []
        for robot_id in range(self.number_of_partitions):
            optimal_paths_clone.append(np.array(self.optimal_paths[robot_id]))
        if "full" in self.plot:
            for robot_id in range(self.number_of_partitions):
                ax4.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
                         c=self.partition_colors[robot_id])
            # with open("Lejeune/optimal_paths_clone.txt", "wb") as fp:  # Pickling
            #     pickle.dump(optimal_paths_clone, fp)

        elif "partial" in self.plot:
            for robot_id in range(self.number_of_partitions):
                plt.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
                         c=self.partition_colors[robot_id])
                plt.axis("equal")

        if "area" in self.plot:
            plt.figure()
            plt.plot(self.area_covered_over_time_time_vector, self.area_covered_over_time)
            plt.title("Area Surveyed Over Time")
            plt.xlabel("Time (s)")
            plt.ylabel("Area Surveyed (Percentage)")
        plt.show()
        
    def calculate_surveillance_rate(self, waypoint_distances):
        """
        Calculate the total area surveyed as a function of time
        """
        # Calculate area coverage per second
        trip_distances = [0 for robot in range(self.number_of_partitions)]
        for robot in range(self.number_of_partitions):
            trip_distances[robot] = sum(waypoint_distances[robot])
        # np.save("Baseline_Environment/QLB_runs/trip_distances.npy", trip_distances)
        # np.save("Baseline_Environment/QLB_runs/trip_times.npy", [trip_distances[i] / self.robot_velocity for i in range(self.number_of_partitions)])

        initial_travel_distance = [0 for robot in range(self.number_of_partitions)]
        for robot, d in enumerate(waypoint_distances):
            initial_travel_distance[robot] = d[0]
            waypoint_distances[robot].pop(0)
        trip_distances = np.array(trip_distances) - np.array(initial_travel_distance)
        area_covered = 0
        t = 0
        self.data_completion_time_per_robot = [0 for robot in range(self.number_of_partitions)]
        while any(waypoint_distances):
            for robot in range(self.number_of_partitions):
                if trip_distances[robot] <= 0:
                    continue
                elif initial_travel_distance[robot] > 0:
                    initial_travel_distance[robot] -= self.robot_velocity * self.sampling_rate
                elif initial_travel_distance[robot] <= 0:
                    if len(waypoint_distances[robot]) != 0:
                        waypoint_distances[robot][0] -= self.robot_velocity * self.sampling_rate
                        if waypoint_distances[robot][0] <= 0:
                            waypoint_distances[robot].pop(0)
                            area_covered += self.cell_area
                    if len(waypoint_distances[robot]) == 0 and self.data_completion_time_per_robot[robot] == 0:
                        self.data_completion_time_per_robot[robot] += t
            self.area_covered_over_time_time_vector.append(round(t, 2))
            t += self.sampling_rate
            self.area_covered_over_time.append(round(area_covered / self.total_survey_area, 2))
        self.data_total_mission_completion_time = t


class Baseline:
    """
        Baseline class for comparison with the SCoPP Monitoring algorithm.
    """
    def __init__(
            self,
            number_of_robots,
            environment,
            plot=False,
            plot_settings=SCoPP_settings.plots()
    ):
        # Initialize environment parameters
        if len(environment.starting_position) > 1:
            geographical_starting_positions = environment.starting_position
        else:
            geographical_starting_positions = []
            for agent in range(number_of_robots):
                geographical_starting_positions.extend(environment.starting_position)
        geographical_boundary_points = environment.boundary_points
        self.robot_FOV = environment.robot_FOV
        self.robot_operating_height = environment.robot_operating_height
        self.robot_velocity = environment.robot_velocity
        geographical_geo_fencing_zones = environment.geo_fencing_holes
        self.save_path = environment.save_path

        # Initialize reference frame
        self.robot_initial_positions_in_cartesian = []
        self.boundary_points_in_cartesian = []
        self.boundary_points_in_geographical = []
        self.robot_initial_positions_in_geographical = []
        origin = [0, 0]
        origin[1] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[0], np.transpose(geographical_boundary_points)[0])))
        origin[0] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[1], np.transpose(geographical_boundary_points)[1])))
        self.origin = origin
        if geographical_geo_fencing_zones:
            self.geographical_geo_fencing_zones_in_cartesian = [[] for item in
                                                                range(len(geographical_geo_fencing_zones))]
            self.geographical_geo_fencing_zones_in_geographical = [[] for item in
                                                                   range(len(geographical_geo_fencing_zones))]
        else:
            self.geographical_geo_fencing_zones_in_cartesian = None
            self.geographical_geo_fencing_zones_in_geographical = None
        for robot, position in enumerate(geographical_starting_positions):
            self.robot_initial_positions_in_geographical.append([position[1], position[0]])
        for point in geographical_boundary_points:
            self.boundary_points_in_geographical.append([point[1], point[0]])
        if geographical_geo_fencing_zones:
            for list_id, fence_list in enumerate(geographical_geo_fencing_zones):
                for point in fence_list:
                    self.geographical_geo_fencing_zones_in_geographical[list_id].append([point[1], point[0]])

        # Initialize method variables
        self.number_of_partitions = number_of_robots
        self.dominated_cells_x = [[] for robot in range(self.number_of_partitions)]
        self.dominated_cells_y = [[] for robot in range(self.number_of_partitions)]
        self.robot_WOV = tan(radians(self.robot_FOV / 2)) * self.robot_operating_height * 2  # in meters
        self.cell_size = int(self.robot_WOV)
        self.cell_area = self.cell_size ** 2  # in meters squared
        self.robot_assignment_information = None

        # Initialize computation time attributes
        self.time_for_conversion = 0
        self.time_for_partitioning = 0
        self.time_for_discretization = 0
        self.total_time = 0

        # Initialize run data attributes for statistics
        self.data_tasks_per_robot = None
        self.data_distance_travelled_per_robot = None
        self.data_completion_time_per_robot = None
        self.data_total_mission_completion_time = None
        self.data_computation_time_without_path_planning = None
        self.data_computation_time_with_path_planning = None

        # Initialize variables for plotting
        self.rough_partitioning_x = None
        self.rough_partitioning_y = None
        self.cluster_centers = None
        self.conflict_cells_x = None
        self.conflict_cells_y = None
        self.final_partitioning_x = None
        self.final_partitioning_y = None
        self.data_tasks_per_robot = None

        # Plot settings
        self.plot = plot
        self.plot_cell_boundary_size = plot_settings.cell_boundary
        self.plot_robot_path_size = plot_settings.robot_path_size
        self.plot_robot_size = plot_settings.robot_size
        self.plot_cell_size = plot_settings.cell_size
        self.partition_colors = None

        # Initialize other useful information
        self.total_survey_area = None
        self.area_per_robot = None
        self.area_covered_over_time = []

    def run(self):
        t0 = time.time()
        convert_to_cart_start_time_stamp = time.time()
        coordinate_converter = lc.LLCCV(self.origin)
        for item in self.robot_initial_positions_in_geographical:
            self.robot_initial_positions_in_cartesian.append(coordinate_converter.get_cartesian(item))
        for item in self.boundary_points_in_geographical:
            self.boundary_points_in_cartesian.append(coordinate_converter.get_cartesian(item))
        if self.geographical_geo_fencing_zones_in_geographical:
            for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones_in_geographical):
                for item in fence_list:
                    self.geographical_geo_fencing_zones_in_cartesian[list_id].append(
                        coordinate_converter.get_cartesian(item))
        self.time_for_conversion += time.time() - convert_to_cart_start_time_stamp
        x_max = np.NINF
        y_max = np.NINF
        for item in self.robot_initial_positions_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        for item in self.boundary_points_in_cartesian:
            if item[1] > x_max:
                x_max = item[1]
            if item[0] > y_max:
                y_max = item[0]
        if self.geographical_geo_fencing_zones_in_cartesian:
            boundary_polygon = Polygon(
                self.boundary_points_in_cartesian,
                holes=self.geographical_geo_fencing_zones_in_cartesian
            )
        else:
            boundary_polygon = Polygon(self.boundary_points_in_cartesian)
        # Create initial grid world
        grid_space = np.zeros((y_max, x_max)) + np.NINF
        points_within_boundary_rows = []
        points_within_boundary_columns = []
        for point in range(len(grid_space)):
            for column in range(len(grid_space[0])):
                if (point % self.cell_size == 0 and column % self.cell_size == 0) \
                        and Point((point, column)).within(boundary_polygon):
                    points_within_boundary_rows.append(point)
                    points_within_boundary_columns.append(column)
        clustering_set = np.transpose([points_within_boundary_rows, points_within_boundary_columns])
        kmeans = KMeans(n_clusters=self.number_of_partitions, max_iter=10, tol=1,
                        precompute_distances=True)
        cluster_indices = kmeans.fit_predict(clustering_set)
        partitions = [[] for robot in self.robot_initial_positions_in_cartesian]
        for index, point in enumerate(clustering_set):
            partitions[cluster_indices[index]].append(list(point))
        first_point = [[] for robot in self.robot_initial_positions_in_cartesian]
        initial_travel_distance = [0 for robot in partitions]
        for robot, position in enumerate(self.robot_initial_positions_in_cartesian):
            first_point[robot] = partitions[robot][0]
            initial_travel_distance[robot] += sqrt((partitions[robot][0][0] - position[0]) ** 2 +
                                                   (partitions[robot][0][1] - position[1]) ** 2)
        first_point = np.array(first_point)
        plt.scatter(clustering_set[:, 0], clustering_set[:, 1], c=cluster_indices)
        plt.scatter(first_point[0:len(self.robot_initial_positions_in_cartesian), 0],
                    first_point[0:len(self.robot_initial_positions_in_cartesian), 1], c="black",
                    s=30)
        self.data_tasks_per_robot = [0 for robot in range(self.number_of_partitions)]
        sweep_lengths = [0 for robot in partitions]
        for robot, partition in enumerate(partitions):
            sweep_lengths[robot] += len(partition) + self.cell_size
            self.data_tasks_per_robot[robot] += len(partition)
        self.data_completion_time_per_robot = np.array(
            sweep_lengths) * self.cell_size / self.robot_velocity + np.array(initial_travel_distance)
        self.data_total_mission_completion_time = max(self.data_completion_time_per_robot)
        self.total_time = time.time() - t0
