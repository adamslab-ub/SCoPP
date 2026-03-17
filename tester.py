#from Swarm_Surveillance.SCoPP 
import monitoring_algorithms
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
#from Swarm_Surveillance.SCoPP 
import environments as envs
#from Swarm_Surveillance.SCoPP 
import SCoPP_settings as QLBSet

"""This codes purpose is to demonstrate the SCoPP Monitoring algorithm on multiple robot population sizes, and generate 
visual results to illustrate its effectiveness. Actual usage of the SCoPP Monitoring algorithm only requires a few lines 
of code. See the README.txt and demo.py files for instructions and examples.
cont
disc

nopath planner=random_walk
path planner=nn
conres mode=bias
noconres 
"""
completion_times = []
for i in range(1):
    debug = False
    baseline = False
    run = True
    plot = "full"
    if debug:
        environment = envs.Debugger()
        print("Field of view:", environment.robot_FOV)
        print("Operating height:", environment.robot_operating_height)
        print("Velocity", environment.robot_velocity)
        algorithm_settings = QLBSet.algorithm()
    else:
        environment = envs.MediumLafayetteFLood(UAV=0)
        print("Field of view:", environment.robot_FOV)
        print("Operating height:", environment.robot_operating_height)
        print("Velocity:", environment.robot_velocity)
        print("Robot starting positions:", environment.starting_position)
        algorithm_settings = QLBSet.algorithm(planner="nn", sampling_rate=1/10)
        plot_settings = QLBSet.plots()

    number_of_robots = [5, 10, 20, 30, 40, 50, 75, 100, 125, 150]
    number_of_robots = [5, 20]
    number_of_robots = [40]

    total_computation_time = [[] for run in number_of_robots]
    no_path_planning_time = [[] for run in number_of_robots]
    tasks_per_robot = [[] for run in number_of_robots]
    time_per_robot = [[] for run in number_of_robots]
    total_completion_time = [[] for run in number_of_robots]
    tasks_data = pd.DataFrame()
    time_per_data = pd.DataFrame()

    if run:
        # RUN PARTITIONER FOR DIFFERENT NUMBERS OF ROBOTS AND RECORD RUN DATA
        for run, robot_population in enumerate(number_of_robots):
            if baseline:
                way_point_allocator = monitoring_algorithms.Baseline(
                    number_of_robots[run],
                    environment,
                    plot=plot,
                )
            else:
                way_point_allocator = monitoring_algorithms.QLB(
                    environment,
                    number_of_robots=number_of_robots[run],
                    plot=plot,
                    algorithm_settings=algorithm_settings,
                    plot_settings=plot_settings,
                )
            if not run:
                print("Cell size:", way_point_allocator.cell_size)
            print("SWARM SURVEILLANCE RUN", run, ":", robot_population, "robots")
            way_points = way_point_allocator.run(info="time")
            if len(number_of_robots) > 1:
                total_computation_time[run] = way_point_allocator.total_time
                tasks_per_robot[run] = np.round(way_point_allocator.data_tasks_per_robot)
                time_per_robot[run] = np.array(way_point_allocator.data_completion_time_per_robot)
                total_completion_time[run] = way_point_allocator.data_total_mission_completion_time
                run_task_data = pd.DataFrame(np.transpose([[robot_population] * robot_population, tasks_per_robot[run]]),
                                             columns=["Number of Robots", "Tasks Per Robot"])
                tasks_data = pd.concat([tasks_data, run_task_data])

                run_time_per_data = pd.DataFrame(
                    np.transpose([np.round([robot_population] * robot_population), time_per_robot[run]]),
                    columns=["Number of Robots", "Completion Time Per Robot"])
                time_per_data = pd.concat([time_per_data, run_time_per_data])

        if not debug:
            if len(number_of_robots) > 1:
                np.save(environment.save_path + "total_computation_time", total_computation_time)
                np.save(environment.save_path + "total_mission_completion_time", total_completion_time)
                tasks_data.to_pickle(environment.save_path + "tasks_data.pkl")
                time_per_data.to_pickle(environment.save_path + "time_per_data.pkl")
            else:
                total_completion_time = np.linspace(0,
                                                    way_point_allocator.data_total_mission_completion_time,
                                                    len(way_point_allocator.area_covered_over_time))
                np.save(environment.save_path + "total_mission_completion_time", total_completion_time)
                np.save(environment.save_path + "area_coverage.npy", way_point_allocator.area_covered_over_time)
                tasks_data.to_pickle(environment.save_path + "tasks_data.pkl")
                time_per_data.to_pickle(environment.save_path + "time_per_data.pkl")

    else:
        if len(number_of_robots) > 1:
            total_computation_time = np.load(environment.save_path + "total_computation_time.npy")
            total_completion_time = np.load(environment.save_path + "total_mission_completion_time.npy")
            tasks_data = pd.read_pickle(environment.save_path + "tasks_data.pkl")
            time_per_data = pd.read_pickle(environment.save_path + "time_per_data.pkl")

    if len(number_of_robots) > 1:
        print("END DATA:")
        print("Computation time:", total_computation_time)
        print("Total mission completion time:", total_completion_time)

        titlesize=18  # fontsize of the title
        axeslabelsize=15  # fontsize of the x and y labels
        xticklabelsize=13  # fontsize of the tick labels
        yticklabelsize=13  # fontsize of the tick labels
        legendsize=15  # fontsize of the legend

        plt.rc('axes', titlesize=titlesize)  # fontsize of the title
        plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
        plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
        plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
        plt.rc('legend', fontsize=legendsize)  # fontsize of the legend
        plt.subplot(2, 2, 1)
        ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data)
        plt.title("Mission Completion Time Per Robot \n for Various Robot Populations")
        plt.xlabel("Robot Population Size")
        plt.ylabel("Completion Time \n Per Robot (Seconds)")

        plt.subplot(2, 2, 2)
        ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data)
        plt.title("Number of Tasks Assigned to \n Each Robot for Various Robot Populations")
        plt.xlabel("Robot Population Size")
        plt.ylabel("Number of Tasks Per Robot")

        plt.subplot(2, 2, 3)
        plt.plot(number_of_robots, total_completion_time)
        plt.title("Total Mission Completion Time \n for Various Robot Populations")
        plt.xlabel("Robot Population Size")
        plt.ylabel("Total Mission \n Completion Time (Seconds)")

        plt.subplot(2, 2, 4)
        plt.plot(number_of_robots, total_computation_time)
        plt.title("Computation Time for Various Robot Populations")
        plt.xlabel("Robot Population Size")
        plt.ylabel("Computation Time (Seconds)")

        left = 0.125  # the left side of the subplots of the figure
        right = 0.9  # the right side of the subplots of the figure
        bottom = 0.1  # the bottom of the subplots of the figure
        top = 0.9  # the top of the subplots of the figure
        wspace = 0.2  # the amount of width reserved for space between subplots,
        # expressed as a fraction of the average axis width
        hspace = 0.4  # the amount of height reserved for space between subplots,
        # expressed as a fraction of the average axis height
        plt.subplots_adjust(left, bottom, right, top, wspace, hspace)

    else:
        print("END DATA:")
        print("Total Computation time:", way_point_allocator.total_time)
        print("Total mission completion time:", way_point_allocator.data_total_mission_completion_time)
        np.save(environment.save_path + "total_computation_time", way_point_allocator.total_time)

        plt.plot(total_completion_time, way_point_allocator.area_covered_over_time)
        plt.title("Area Surveyed Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Area Surveyed ($m^2$)")

        print("INDIVIDUAL COMPUTATION TIMES:")
        print("Discretization:", way_point_allocator.time_for_discretization)
        print("Conversions:", way_point_allocator.time_for_conversion)
        print("Conflict Resolution:", way_point_allocator.time_for_conflict_resolution)
        print("Path Planning:", way_point_allocator.time_for_path_planning)
        print("Partitioning:", way_point_allocator.time_for_partitioning)
        np.save(environment.save_path + "discretization_computation_time",
                way_point_allocator.time_for_discretization)
        np.save(environment.save_path + "conversions_computation_time",
                way_point_allocator.time_for_conversion)
        np.save(environment.save_path + "conflict_resolution_computation_time",
                way_point_allocator.time_for_conflict_resolution)
        np.save(environment.save_path + "path_planning_computation_time",
                way_point_allocator.time_for_path_planning)
        np.save(environment.save_path + "partitioning_computation_time",
                way_point_allocator.time_for_partitioning)

        print("INDIVIDUAL COMPUTATION TIMES BY PERCENTAGE:")
        print("Discretization:", way_point_allocator.time_for_discretization / way_point_allocator.total_time)
        print("Conversions:", way_point_allocator.time_for_conversion / way_point_allocator.total_time)
        print("Conflict Resolution:", way_point_allocator.time_for_conflict_resolution / way_point_allocator.total_time)
        print("Path Planning:", way_point_allocator.time_for_path_planning / way_point_allocator.total_time)
        print("Partitioning:", way_point_allocator.time_for_partitioning / way_point_allocator.total_time)
        np.save(environment.save_path + "percent_discretization_computation_time",
                way_point_allocator.time_for_discretization / way_point_allocator.total_time)
        np.save(environment.save_path + "percent_conversions_computation_time",
                way_point_allocator.time_for_conversion / way_point_allocator.total_time)
        np.save(environment.save_path + "percent_conflict_resolution_computation_time",
                way_point_allocator.time_for_conflict_resolution / way_point_allocator.total_time)
        np.save(environment.save_path + "percent_path_planning_computation_time",
                way_point_allocator.time_for_path_planning / way_point_allocator.total_time)
        np.save(environment.save_path + "percent_partitioning_computation_time",
                way_point_allocator.time_for_partitioning / way_point_allocator.total_time)
        print(max(total_completion_time))
        completion_times.append(max(total_completion_time))
    # np.save("100_completion_times_13_robots", completion_times)
    # mng = plt.get_current_fig_manager()
    # mng.resize(*mng.window.maxsize())
    # mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()
    # plt.show()

