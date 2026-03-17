import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns
import pickle
import pandas as pd


computation_vector = np.load("100_completion_times_13_robots.npy")
print(computation_vector)
print(np.median(computation_vector))
print(np.mean(computation_vector))
print(max(computation_vector))
print(min(computation_vector))
print(np.std(computation_vector))
# plt.hist(computation_vector, bins=30)
# plt.show()

mode = "map"
sampling_rate = 1/10

if mode == 1:
    T0 = np.load("Benning/area_coverage0.npy")
    T1 = np.load("Benning/area_coverage1.npy")
    T2 = np.load("Benning/area_coverage2.npy")
    T3 = np.load("Benning/area_coverage3.npy")
    T4 = np.load("Benning/area_coverage4.npy")

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    t2 = np.linspace(0, (len(T2) - 1) * sampling_rate, int(len(T2)))
    t3 = np.linspace(0, (len(T3) - 1) * sampling_rate, int(len(T3)))
    t4 = np.linspace(0, (len(T4) - 1) * sampling_rate, int(len(T4)))
    plt.rc('axes', titlesize=25)    # fontsize of the title
    plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
    plt.rc('legend', fontsize=15)    # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)
    p0, = plt.plot(t0, T0)
    p1, = plt.plot(t1, T1)
    p2, = plt.plot(t2, T2)
    p3, = plt.plot(t3, T3)
    p4, = plt.plot(t4, T4)
    print(max(t0))
    print(max(t1))
    print(max(t2))
    print(max(t3))
    print(max(t4))

    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.legend([p0, p1, p2, p3, p4], ["Case 1", "Case 2", "Case 3", "Case 4", "Case 5"])
    plt.show()


if mode == "height":
    T0 = np.load("LargeLafayetteFLood/Height_Tests/height_T25_area_coverage.npy")
    T1 = np.load("LargeLafayetteFLood/Height_Tests/height_T50_area_coverage.npy")
    T2 = np.load("LargeLafayetteFLood/Height_Tests/height_T75_area_coverage.npy")
    T3 = np.load("LargeLafayetteFLood/Height_Tests/height_T100_area_coverage.npy")

    CT0 = np.load("LargeLafayetteFLood/Height_Tests/height_T25_total_computation_time.npy")
    CT1 = np.load("LargeLafayetteFLood/Height_Tests/height_T50_total_computation_time.npy")
    CT2 = np.load("LargeLafayetteFLood/Height_Tests/height_T75_total_computation_time.npy")
    CT3 = np.load("LargeLafayetteFLood/Height_Tests/height_T100_total_computation_time.npy")

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    t2 = np.linspace(0, (len(T2) - 1) * sampling_rate, int(len(T2)))
    t3 = np.linspace(0, (len(T3) - 1) * sampling_rate, int(len(T3)))

    plt.rc('axes', titlesize=25)    # fontsize of the title
    plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
    plt.rc('legend', fontsize=15)    # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)
    p0, = plt.plot(t0, T0)
    p1, = plt.plot(t1, T1)
    p2, = plt.plot(t2, T2)
    p3, = plt.plot(t3, T3)

    print("Computation Times:")
    print(CT0, CT1, CT2, CT3)

    print("Mission Completion Times:")
    print(max(t0), max(t1), max(t2), max(t3))

    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.grid()
    plt.legend([p0, p1, p2, p3], ["25m operating \n height", "50m operating \n height", "75m operating \n height", "100m operating \n height"])
    plt.savefig("Plot/lafayette_large_height_test_AOT.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()

if mode == "velocity":
    T0 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T2_area_coverage.npy")
    T1 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T4_area_coverage.npy")
    T2 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T6_area_coverage.npy")
    T3 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T8_area_coverage.npy")
    T4 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T10_area_coverage.npy")

    CT0 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T2_total_computation_time.npy")
    CT1 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T4_total_computation_time.npy")
    CT2 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T6_total_computation_time.npy")
    CT3 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T8_total_computation_time.npy")
    CT4 = np.load("LargeLafayetteFLood/Velocity_Tests/velocity_T10_total_computation_time.npy")

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    t2 = np.linspace(0, (len(T2) - 1) * sampling_rate, int(len(T2)))
    t3 = np.linspace(0, (len(T3) - 1) * sampling_rate, int(len(T3)))
    t4 = np.linspace(0, (len(T4) - 1) * sampling_rate, int(len(T4)))

    plt.rc('axes', titlesize=25)    # fontsize of the title
    plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
    plt.rc('legend', fontsize=15)    # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)

    p0, = plt.plot(t0, T0)
    p1, = plt.plot(t1, T1)
    p2, = plt.plot(t2, T2)
    p3, = plt.plot(t3, T3)
    p4, = plt.plot(t4, T4)

    print("Computation Times:")
    print(CT0, CT1, CT2, CT3, CT4)

    print("Mission Completion Times:")
    print(max(t0), max(t1), max(t2), max(t3), max(t4))

    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.grid()
    plt.legend([p0, p1, p2, p3, p4], ["2 m/s velocity", "4 m/s velocity", "6 m/s velocity", "8 m/s velocity", "10 m/s velocity"])
    plt.savefig("Plot/lafayette_large_velocity_test_AOT.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()

if mode == "dispatcher":
    T0 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T1_area_coverage.npy")
    T1 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T2_area_coverage.npy")
    T2 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T3_area_coverage.npy")
    T3 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T4_area_coverage.npy")

    CT0 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T1_total_computation_time.npy")
    CT1 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T2_total_computation_time.npy")
    CT2 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T3_total_computation_time.npy")
    CT3 = np.load("MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T4_total_computation_time.npy")

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    t2 = np.linspace(0, (len(T2) - 1) * sampling_rate, int(len(T2)))
    t3 = np.linspace(0, (len(T3) - 1) * sampling_rate, int(len(T3)))
    plt.rc('axes', titlesize=25)    # fontsize of the title
    plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
    plt.rc('legend', fontsize=15)    # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)

    p0, = plt.plot(t0, T0)
    p1, = plt.plot(t1, T1)
    p2, = plt.plot(t2, T2)
    p3, = plt.plot(t3, T3)

    print("Computation Times:")
    print(CT0, CT1, CT2, CT3)

    print("Mission Completion Times:")
    print(max(t0), max(t1), max(t2), max(t3))
    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.grid()
    plt.legend([p0, p1, p2, p3], ["1 Dispatcher", "2 Dispatchers", "3 Dispatchers", "4 Dispatchers"])
    plt.savefig("Plot/lafayette_medium_dispatcher_test_AOT.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()

if mode == "map":

    titlesize=18  # fontsize of the title
    axeslabelsize=15  # fontsize of the x and y labels
    xticklabelsize=13  # fontsize of the tick labels
    yticklabelsize=13  # fontsize of the tick labels
    legendsize=15  # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)

    plt.rc('axes', titlesize=titlesize)  # fontsize of the title
    plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
    plt.rc('legend', fontsize=legendsize)  # fontsize of the legend

    number_of_robots = [5, 10, 20, 30, 40, 50, 75, 100, 125, 150]
    with open("LargeLafayetteFLood/Map_Comparison/time_per_data.pkl", 'rb') as f:
        time_per_data1 = pickle.load(f)
    with open("LargeLafayetteFLood/Map_Comparison/tasks_data.pkl", 'rb') as f:
        tasks_data1 = pickle.load(f)
    total_completion_time1 = np.load("LargeLafayetteFLood/Map_Comparison/total_mission_completion_time.npy")
    path_planning_time1 = np.load("LargeLafayetteFLood/Map_Comparison/total_computation_time.npy")

    with open("MediumLafayetteFlood/Map_Comparison/time_per_data.pkl", 'rb') as f:
        time_per_data2 = pickle.load(f)
    with open("MediumLafayetteFlood/Map_Comparison/tasks_data.pkl", 'rb') as f:
        tasks_data2 = pickle.load(f)
    total_completion_time2 = np.load("MediumLafayetteFlood/Map_Comparison/total_mission_completion_time.npy")
    path_planning_time2 = np.load("MediumLafayetteFlood/Map_Comparison/total_computation_time.npy")

    # plt.subplot(2, 2, 1)
    # ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data1)
    # plt.title("Mission Completion Time Per Robot \n for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Completion Time \n Per Robot (Seconds)")
    # plt.show()
    # kmn
    # plt.subplot(2, 2, 2)
    # ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data1)
    # plt.title("Number of Tasks Assigned to \n Each Robot for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Number of Tasks Per Robot")

    fig, ax1 = plt.subplots()
    color = 'tab:red'
    ax1.set_xlabel("Number of Robots")
    ax1.set_ylabel("Mission Time (sec)", color=color)
    large_completion, = ax1.plot(number_of_robots, total_completion_time1, color=color, marker="v", linestyle=":")
    medium_completion, = ax1.plot(number_of_robots, total_completion_time2, color=color, marker="o", linestyle=":")
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel("Computing Time (sec)", color=color)  # we already handled the x-label with ax1
    large_computation, = ax2.plot(number_of_robots, path_planning_time1, color=color, marker="v")
    medium_computation, = ax2.plot(number_of_robots, path_planning_time2, color=color, marker="o")
    ax2.tick_params(axis='y', labelcolor=color)

    from matplotlib.lines import Line2D
    custom_lines = [Line2D([0], [0], marker='v', color='w', label='Scatter',
                           markerfacecolor='black', markersize=8),
                    Line2D([0], [0], marker='o', color='w', label='Scatter',
                           markerfacecolor='black', markersize=8)]
    ax1.legend(custom_lines, ["Large Map", "Medium Map"], loc="upper center", bbox_to_anchor=(0.2, 0.57, 1, 0.3))

    fig.set_size_inches(6, 3)
    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    # plt.legend([large_completion, medium_completion, large_computation, medium_computation], ["Large Map Mission Time", "Medium Map Mission Time", "Large Map Computation Time", "Medium Map Computation Time"])

    # plt.subplot(2, 1, 1)
    # p11,  = plt.plot(number_of_robots, total_completion_time1)
    # plt.title("Total Mission Completion Time \n for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Total Mission \n Completion Time (Seconds)")
    #
    # plt.subplot(2, 1, 2)
    # p12,  = plt.plot(number_of_robots, path_planning_time1)
    # plt.title("Computation Time for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Computation Time (Seconds)")
    #
    #
    # # plt.subplot(2, 2, 3)
    # # ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data2)
    # # plt.title("Mission Completion Time Per Robot \n for Various Robot Populations")
    # # plt.xlabel("Robot Population Size")
    # # plt.ylabel("Completion Time \n Per Robot (Seconds)")
    # #
    # # plt.subplot(2, 2, 4)
    # # ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data2)
    # # plt.title("Number of Tasks Assigned to \n Each Robot for Various Robot Populations")
    # # plt.xlabel("Robot Population Size")
    # # plt.ylabel("Number of Tasks Per Robot")
    #
    # plt.subplot(2, 1, 1)
    # p21,  = plt.plot(number_of_robots, total_completion_time2)
    # plt.title("Total Mission Completion Time \n for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Total Mission \n Completion Time (Seconds)")
    # plt.grid()
    # plt.legend([p11, p21], ["Large Map", "Medium Map"])
    #
    # plt.subplot(2, 1, 2)
    # p22,  = plt.plot(number_of_robots, path_planning_time2)
    # plt.title("Computation Time \n for Various Robot Populations")
    # plt.xlabel("Robot Population Size")
    # plt.ylabel("Computation \n Time (Seconds)")
    # plt.grid()
    #
    # plt.legend([p12, p22], ["Large Map", "Medium Map"])
    #
    # left = 0.125  # the left side of the subplots of the figure
    # right = 0.9  # the right side of the subplots of the figure
    # bottom = 0.1  # the bottom of the subplots of the figure
    # top = 1  # the top of the subplots of the figure
    # wspace = 0.2  # the amount of width reserved for space between subplots,
    # # expressed as a fraction of the average axis width
    # hspace = 0.8  # the amount of height reserved for space between subplots,
    # # expressed as a fraction of the average axis height
    # plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
    #
    #
    print("Computation Times:")
    print("Large Map:", path_planning_time1, "\nMedium Map:", path_planning_time2)
    #
    print("Mission Completion Times:")
    print("Large Map:", total_completion_time1, "\nMedium Map:", total_completion_time2)
    # plt.savefig("Plot/2x1_medium_large_map_comparison.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.savefig("Plot/medium_large_map_comparison.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()

if mode == "show_phases":
    with open("SmallLafayetteFlood/rough_partitioning.txt", "rb") as fp:  # Unpickling
        rough_partitioning = pickle.load(fp)
    rough_partitioning_x = rough_partitioning[0]
    rough_partitioning_y = rough_partitioning[1]
    with open("SmallLafayetteFlood/number_of_partitions.txt", "rb") as fp:  # Unpickling
        number_of_partitions = pickle.load(fp)
    with open("SmallLafayetteFlood/cluster_centers.txt", "rb") as fp:  # Unpickling
        cluster_centers = pickle.load(fp)
    with open("SmallLafayetteFlood/partition_colors.txt", "rb") as fp:  # Unpickling
        partition_colors = pickle.load(fp)
    with open("SmallLafayetteFlood/dominated_cells.txt", "rb") as fp:  # Unpickling
        dominated_cells = pickle.load(fp)
    dominated_cells_x = dominated_cells[0]
    dominated_cells_y = dominated_cells[1]
    with open("SmallLafayetteFlood/conflict_cells.txt", "rb") as fp:  # Unpickling
        conflict_cells = pickle.load(fp)
    conflict_cells_x = conflict_cells[0]
    conflict_cells_y = conflict_cells[1]
    with open("SmallLafayetteFlood/final_partitioning.txt", "rb") as fp:  # Unpickling
        final_partitioning = pickle.load(fp)
    final_partitioning_x = final_partitioning[0]
    final_partitioning_y = final_partitioning[1]
    with open("SmallLafayetteFlood/robot_initial_positions_in_cartesian.txt", "rb") as fp:  # Unpickling
        robot_initial_positions_in_cartesian = pickle.load(fp)
    with open("SmallLafayetteFlood/optimal_paths_clone.txt", "rb") as fp:  # Unpickling
        optimal_paths_clone = pickle.load(fp)

    plot_cell_boundary_size = 5
    plot_robot_size = 30
    plot_cell_size = 200
    plot_cell_conflict_boundary_size = 25

    plt.subplot(2, 2, 1)
    for robot_id in range(number_of_partitions):
        plt.scatter(rough_partitioning_x[robot_id], rough_partitioning_y[robot_id], marker="s",
                    s=plot_cell_boundary_size,
                    c=np.ones((len(rough_partitioning_x[robot_id]), 3)) * partition_colors[robot_id])
        plt.scatter(cluster_centers[0], cluster_centers[1], s=plot_robot_size, c='black')
    plt.axis("equal")

    plt.subplot(2, 2, 2)
    for robot_id in range(number_of_partitions):
        plt.scatter(rough_partitioning_x[robot_id], rough_partitioning_y[robot_id], marker="s",
                    s=plot_cell_boundary_size,
                    c=np.ones((len(rough_partitioning_x[robot_id]), 3)) * partition_colors[robot_id])
        plt.scatter(dominated_cells_x[robot_id], dominated_cells_y[robot_id], marker="s",
                    s=plot_cell_size,
                    c=np.ones((len(dominated_cells_x[robot_id]), 3)) * partition_colors[robot_id])
    plt.scatter(conflict_cells_x, conflict_cells_y, marker="s", s=plot_cell_conflict_boundary_size,
                c="black")
    plt.axis("equal")
    count = 0
    for robot_id in range(number_of_partitions):
        if count == 0:
            plt.plot([optimal_paths_clone[robot_id][0, 0], optimal_paths_clone[robot_id][2, 0]], [optimal_paths_clone[robot_id][0, 1], optimal_paths_clone[robot_id][2, 1]],
                 c=partition_colors[robot_id], linewidth=8)

        elif count == 1:
            plt.plot([optimal_paths_clone[robot_id][0, 0], optimal_paths_clone[robot_id][3, 0]], [optimal_paths_clone[robot_id][0, 1], optimal_paths_clone[robot_id][3, 1]],
                 c=partition_colors[robot_id], linewidth=8)

        elif count == 2:
            plt.plot([optimal_paths_clone[robot_id][0, 0], optimal_paths_clone[robot_id][1, 0]], [optimal_paths_clone[robot_id][0, 1], optimal_paths_clone[robot_id][1, 1]],
                 c=partition_colors[robot_id], linewidth=8)

        elif count == 3:
            plt.plot([optimal_paths_clone[robot_id][0, 0], optimal_paths_clone[robot_id][2, 0]], [optimal_paths_clone[robot_id][0, 1], optimal_paths_clone[robot_id][2, 1]],
                 c=partition_colors[robot_id], linewidth=8)

        elif count == 4:
            plt.plot([optimal_paths_clone[robot_id][0, 0], optimal_paths_clone[robot_id][3, 0]], [optimal_paths_clone[robot_id][0, 1], optimal_paths_clone[robot_id][3, 1]],
                 c=partition_colors[robot_id], linewidth=8)
        count += 1

    plt.subplot(2, 2, 3)
    for robot_id in range(number_of_partitions):
        plt.scatter(final_partitioning_x[robot_id], final_partitioning_y[robot_id], marker="s",
                    s=plot_cell_size,
                    c=np.ones((len(final_partitioning_x[robot_id]), 3)) * partition_colors[robot_id])
    plt.scatter(conflict_cells_x, conflict_cells_y, marker="s", s=plot_cell_conflict_boundary_size,
                c="black")
    plt.axis("equal")

    ax4 = plt.subplot(2, 2, 4)
    ax4.scatter(np.transpose(robot_initial_positions_in_cartesian)[0],
                np.transpose(robot_initial_positions_in_cartesian)[1],
                s=plot_robot_size, c="black")
    for robot_id in range(number_of_partitions):
        ax4.scatter(final_partitioning_x[robot_id], final_partitioning_y[robot_id], marker="s",
                    s=plot_cell_size,
                    c=np.ones((len(final_partitioning_x[robot_id]), 3)) * partition_colors[robot_id])
    plt.axis("equal")

    for robot_id in range(number_of_partitions):
        ax4.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
                 c=partition_colors[robot_id])
    plt.show()

if mode == "phase":
    T0 = np.load("SmallLafayetteFlood/no_discontinuities/conflict_resolution/area_coverage.npy")
    T1 = np.load("SmallLafayetteFlood/no_discontinuities/no_conflict_resolution/area_coverage.npy")
    T2 = np.load("SmallLafayetteFlood/no_discontinuities/path_planning/area_coverage.npy")
    T3 = np.load("SmallLafayetteFlood/no_discontinuities/no_path_planning/area_coverage.npy")
    T4 = np.load("SmallLafayetteFlood/discontinuities/conflict_resolution/area_coverage.npy")
    T5 = np.load("SmallLafayetteFlood/discontinuities/no_conflict_resolution/area_coverage.npy")
    T6 = np.load("SmallLafayetteFlood/discontinuities/path_planning/area_coverage.npy")
    T7 = np.load("SmallLafayetteFlood/discontinuities/no_path_planning/area_coverage.npy")

    CT0 = np.load("SmallLafayetteFlood/no_discontinuities/conflict_resolution/total_computation_time.npy")
    CT1 = np.load("SmallLafayetteFlood/no_discontinuities/no_conflict_resolution/total_computation_time.npy")
    CT2 = np.load("SmallLafayetteFlood/no_discontinuities/path_planning/total_computation_time.npy")
    CT3 = np.load("SmallLafayetteFlood/no_discontinuities/no_path_planning/total_computation_time.npy")
    CT4 = np.load("SmallLafayetteFlood/discontinuities/conflict_resolution/total_computation_time.npy")
    CT5 = np.load("SmallLafayetteFlood/discontinuities/no_conflict_resolution/total_computation_time.npy")
    CT6 = np.load("SmallLafayetteFlood/discontinuities/path_planning/total_computation_time.npy")
    CT7 = np.load("SmallLafayetteFlood/discontinuities/no_path_planning/total_computation_time.npy")

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    t2 = np.linspace(0, (len(T2) - 1) * sampling_rate, int(len(T2)))
    t3 = np.linspace(0, (len(T3) - 1) * sampling_rate, int(len(T3)))
    t4 = np.linspace(0, (len(T4) - 1) * sampling_rate, int(len(T4)))
    t5 = np.linspace(0, (len(T5) - 1) * sampling_rate, int(len(T5)))
    t6 = np.linspace(0, (len(T6) - 1) * sampling_rate, int(len(T6)))
    t7 = np.linspace(0, (len(T7) - 1) * sampling_rate, int(len(T7)))

    plt.rc('axes', titlesize=25)    # fontsize of the title
    plt.rc('axes', labelsize=20)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
    plt.rc('legend', fontsize=15)    # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)
    plt.grid()
    p0, = plt.plot(t0, T0)
    p1, = plt.plot(t1, T1)
    p2, = plt.plot(t2, T2)
    p3, = plt.plot(t3, T3)

    print("Computation Times:")
    print(CT0, CT1, CT2, CT3)

    print("Mission Completion Times:")
    print(max(t0), max(t1), max(t2), max(t3))

    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.legend([p0, p1, p2, p3], ["State-biased \n Conflict Resolution", "Random Uniform \n Conflict Resolution", "Nearest Neighbor \n Path Planning", "Random Walk \n Path Planning"])
    plt.savefig("Plot/lafayette_small_phase_test_AOT.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()


if mode == "general_NYC_QLB":
    number_of_robots = [5, 10, 20, 30,40]
    total_computation_time = np.load("Brooklyn_Init_Test/QLB_runs/total_computation_time.npy")
    total_completion_time = np.load("Brooklyn_Init_Test/QLB_runs/total_mission_completion_time.npy")
    tasks_data = pd.read_pickle("Brooklyn_Init_Test/QLB_runs/tasks_data.pkl")
    time_per_data = pd.read_pickle("Brooklyn_Init_Test/QLB_runs/time_per_data.pkl")

    plt.figure(figsize=(8, 6))
    titlesize = 18  # fontsize of the title
    axeslabelsize = 15  # fontsize of the x and y labels
    xticklabelsize = 13  # fontsize of the tick labels
    yticklabelsize = 13  # fontsize of the tick labels
    legendsize = 15  # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)

    plt.rc('axes', titlesize=titlesize)  # fontsize of the title
    plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
    plt.rc('legend', fontsize=legendsize)  # fontsize of the legend

    plt.subplot(2, 2, 1)
    ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data)
    plt.title("Mission Completion \n Time Per Robot \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Completion \n Time Per Robot \n (Seconds)")
    plt.grid()

    plt.subplot(2, 2, 2)
    ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data)
    plt.title("Number of Tasks Assigned to \n Each Robot for \n Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Number of \n Tasks Per Robot")
    plt.grid()

    plt.subplot(2, 2, 3)
    plt.plot(number_of_robots, total_completion_time)
    plt.title("Total Mission Completion Time \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Total Mission \n Completion Time \n (Seconds)")
    plt.grid()

    plt.subplot(2, 2, 4)
    plt.plot(number_of_robots, total_computation_time)
    plt.title("Computation Time \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Computation \n Time (Seconds)")
    plt.grid()

    left = 0.125  # the left side of the subplots of the figure
    right = 0.9  # the right side of the subplots of the figure
    bottom = 0.1  # the bottom of the subplots of the figure
    top = 0.9  # the top of the subplots of the figure
    wspace = 0.5  # the amount of width reserved for space between subplots,
    # expressed as a fraction of the average axis width
    hspace = 0.7  # the amount of height reserved for space between subplots,
    # expressed as a fraction of the average axis height
    plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
    plt.savefig("Plot/NYC_general_test_QLB.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()


if mode == "general_NYC_baseline":
    number_of_robots = [5, 10, 20, 30,40]
    total_computation_time = np.load("Brooklyn_Init_Test/baseline_runs/total_computation_time.npy")
    total_completion_time = np.load("Brooklyn_Init_Test/baseline_runs/total_mission_completion_time.npy")
    tasks_data = pd.read_pickle("Brooklyn_Init_Test/baseline_runs/tasks_data.pkl")
    time_per_data = pd.read_pickle("Brooklyn_Init_Test/baseline_runs/time_per_data.pkl")

    titlesize = 18  # fontsize of the title
    axeslabelsize = 15  # fontsize of the x and y labels
    xticklabelsize = 13  # fontsize of the tick labels
    yticklabelsize = 13  # fontsize of the tick labels
    legendsize = 15  # fontsize of the legend
    # font = {'family': 'serif',
    #         'weight': 'normal',
    #         'size': 12}
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)
    plt.figure(figsize=(8, 6))
    plt.rc('axes', titlesize=titlesize)  # fontsize of the title
    plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
    plt.rc('legend', fontsize=legendsize)  # fontsize of the legend

    plt.subplot(2, 2, 1)
    ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data)
    plt.title("Mission Completion \n Time Per Robot \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Completion \n Time Per Robot \n (Seconds)")
    plt.grid()

    plt.subplot(2, 2, 2)
    ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data)
    plt.title("Number of Tasks Assigned to \n Each Robot for \n Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Number of \n Tasks Per Robot")
    plt.grid()

    plt.subplot(2, 2, 3)
    plt.plot(number_of_robots, total_completion_time)
    plt.title("Total Mission Completion Time \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Total Mission \n Completion Time \n (Seconds)")
    plt.grid()

    plt.subplot(2, 2, 4)
    plt.plot(number_of_robots, total_computation_time)
    plt.title("Computation Time \n for Various Robot Populations")
    plt.xlabel("Robot Population Size")
    plt.ylabel("Computation \n Time (Seconds)")
    plt.grid()

    left = 0.125  # the left side of the subplots of the figure
    right = 0.9  # the right side of the subplots of the figure
    bottom = 0.1  # the bottom of the subplots of the figure
    top = 0.9  # the top of the subplots of the figure
    wspace = 0.5  # the amount of width reserved for space between subplots,
    # expressed as a fraction of the average axis width
    hspace = 0.7  # the amount of height reserved for space between subplots,
    # expressed as a fraction of the average axis height
    plt.subplots_adjust(left, bottom, right, top, wspace, hspace)
    plt.savefig("Plot/NYC_general_test_baseline.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()



if mode == "baseline_environment":
    # # BOXPLOTS FOR WORKLOAD DISTRIBUTION W.R.T. DISTANCE
    # QLBM_distances = np.load("Baseline_Environment/QLB_runs/trip_distances.npy")
    # baseline_distances = loadmat("Baseline_Environment/baseline_runs/trip_distances.mat")["trip_distances"].transpose()[0]
    # print(QLBM_distances)
    # print(baseline_distances)
    # # plt.boxplot()
    # plt.rc('axes', titlesize=25)  # fontsize of the title
    # plt.rc('axes', labelsize=20)  # fontsize of the x and y labels
    # plt.rc('xtick', labelsize=15)  # fontsize of the tick labels
    # plt.rc('ytick', labelsize=15)  # fontsize of the tick labels
    # plt.rc('legend', fontsize=15)  # fontsize of the legend
    # font = {'family': 'Times New Roman',
    #         'weight': 'normal',
    #         'size': 12}
    # matplotlib.rc("font", **font)
    # plt.xlabel("Area Surveillance Method")
    # plt.ylabel("Total Distance to be \nTravelled Amongst Robots")
    # plt.title("Work-load Distribution with Resepect \nto Total Distance Travelled")
    # plt.boxplot([QLBM_distances, baseline_distances], labels=["QLBM", "Guastella"])
    # plt.savefig("Plot/baseline_environment_QLBM_comparison_distance_load_distribution.pdf", format="pdf", dpi=300, bbox_inches='tight')
    # plt.show()

    # BOXPLOTS FOR WORKLOAD DISTRIBUTION W.R.T. TIME
    from scipy.io import loadmat
    QLBM_times = np.load("Baseline_Environment/QLB_runs/trip_times.npy")
    baseline_times = loadmat("Baseline_Environment/baseline_runs/trip_times.mat")["trip_times"].transpose()[
        0]
    print(max(QLBM_times))
    print(baseline_times)
    plt.figure(figsize=(5, 4))
    plt.rc('axes', titlesize=25)  # fontsize of the title
    plt.rc('axes', labelsize=20)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=20)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=20)  # fontsize of the tick labels
    plt.rc('legend', fontsize=15)  # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)
    plt.xlabel("Method")
    plt.ylabel("Completion Time per Robot (sec)")
    plt.boxplot([QLBM_times, baseline_times], labels=["QLBM", "Guastella"])
    plt.savefig("Plot/baseline_environment_QLBM_comparison_time_load_distribution.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()


    # AREA COVERAGE OVER TIME
    T0 = np.load("Baseline_Environment/QLB_runs/area_coverage.npy")
    print(T0)
    CT0 = np.load("Baseline_Environment/QLB_runs/total_computation_time.npy")
    print(CT0)
    baseline_AOT = loadmat("Baseline_Environment/baseline_runs/distance_covered_over_time.mat")["distance_covered_over_time"]
    baseline_t = loadmat("Baseline_Environment/baseline_runs/distance_covered_over_time_time_vector.mat")["area_covered_over_time_time_vector"]
    # baseline_t["distance_covered_over_time_time_vector"]

    t0 = np.linspace(0, (len(T0) - 1) * sampling_rate, int(len(T0)))
    print(max(t0))
    print(baseline_t)
    print(baseline_AOT)
    # t1 = np.linspace(0, (len(T1) - 1) * sampling_rate, int(len(T1)))
    plt.rc('axes', titlesize=25)  # fontsize of the title
    plt.rc('axes', labelsize=20)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=15)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=15)  # fontsize of the tick labels
    plt.rc('legend', fontsize=15)  # fontsize of the legend
    font = {'family': 'Times New Roman',
            'weight': 'normal',
            'size': 12}
    matplotlib.rc("font", **font)

    p0, = plt.plot(t0, T0)
    p1, = plt.plot(baseline_t, baseline_AOT)

    print("Computation Times:")
    # print(CT0, CT1)

    # print("Mission Completion Times:")
    # print(max(t0), max(t1), max(t2), max(t3))
    plt.title("Area Surveyed Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Area Surveyed (%)")
    plt.grid()
    plt.legend([p0, p1], ["QLBM", "Guastella"])
    plt.savefig("Plot/baseline_environment_QLBM_comparison.pdf", format="pdf", dpi=300, bbox_inches='tight')
    plt.show()

    # # COMPUTATION AND MISSION TIME
    # titlesize=18  # fontsize of the title
    # axeslabelsize=15  # fontsize of the x and y labels
    # xticklabelsize=13  # fontsize of the tick labels
    # yticklabelsize=13  # fontsize of the tick labels
    # legendsize=15  # fontsize of the legend
    # font = {'family': 'Times New Roman',
    #         'weight': 'normal',
    #         'size': 12}
    # matplotlib.rc("font", **font)
    #
    # plt.rc('axes', titlesize=titlesize)  # fontsize of the title
    # plt.rc('axes', labelsize=axeslabelsize)  # fontsize of the x and y labels
    # plt.rc('xtick', labelsize=xticklabelsize)  # fontsize of the tick labels
    # plt.rc('ytick', labelsize=yticklabelsize)  # fontsize of the tick labels
    # plt.rc('legend', fontsize=legendsize)  # fontsize of the legend
    #
    # number_of_robots = [5, 10, 20, 30, 40, 50, 75, 100, 125, 150]
    # with open("LargeLafayetteFLood/Map_Comparison/time_per_data.pkl", 'rb') as f:
    #     time_per_data1 = pickle.load(f)
    # with open("LargeLafayetteFLood/Map_Comparison/tasks_data.pkl", 'rb') as f:
    #     tasks_data1 = pickle.load(f)
    # total_completion_time1 = np.load("LargeLafayetteFLood/Map_Comparison/total_mission_completion_time.npy")
    # path_planning_time1 = np.load("LargeLafayetteFLood/Map_Comparison/total_computation_time.npy")
    #
    # with open("MediumLafayetteFlood/Map_Comparison/time_per_data.pkl", 'rb') as f:
    #     time_per_data2 = pickle.load(f)
    # with open("MediumLafayetteFlood/Map_Comparison/tasks_data.pkl", 'rb') as f:
    #     tasks_data2 = pickle.load(f)
    # total_completion_time2 = np.load("MediumLafayetteFlood/Map_Comparison/total_mission_completion_time.npy")
    # path_planning_time2 = np.load("MediumLafayetteFlood/Map_Comparison/total_computation_time.npy")
    #
    #
    # # plt.subplot(2, 2, 1)
    # # ax_time_per_data = sns.lineplot(x="Number of Robots", y="Completion Time Per Robot", data=time_per_data1)
    # # plt.title("Mission Completion Time Per Robot \n for Various Robot Populations")
    # # plt.xlabel("Robot Population Size")
    # # plt.ylabel("Completion Time \n Per Robot (Seconds)")
    # # plt.show()
    # # kmn
    # # plt.subplot(2, 2, 2)
    # # ax_tasks_data = sns.lineplot(x="Number of Robots", y="Tasks Per Robot", data=tasks_data1)
    # # plt.title("Number of Tasks Assigned to \n Each Robot for Various Robot Populations")
    # # plt.xlabel("Robot Population Size")
    # # plt.ylabel("Number of Tasks Per Robot")
    #
    # fig, ax1 = plt.subplots()
    #
    # color = 'tab:red'
    # ax1.set_xlabel("Number of Robots")
    # ax1.set_ylabel("Mission Time (seconds)", color=color)
    # large_completion, = ax1.plot(number_of_robots, total_completion_time1, color=color, marker="v", linestyle=":")
    # medium_completion, = ax1.plot(number_of_robots, total_completion_time2, color=color, marker="o", linestyle=":")
    # ax1.tick_params(axis='y', labelcolor=color)
    #
    # ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    #
    # color = 'tab:blue'
    # ax2.set_ylabel("Computation Time (seconds)", color=color)  # we already handled the x-label with ax1
    # large_computation, = ax2.plot(number_of_robots, path_planning_time1, color=color, marker="v")
    # medium_computation, = ax2.plot(number_of_robots, path_planning_time2, color=color, marker="o")
    # ax2.tick_params(axis='y', labelcolor=color)
    #
    # from matplotlib.lines import Line2D
    # custom_lines = [Line2D([0], [0], marker='v', color='w', label='Scatter',
    #                        markerfacecolor='black', markersize=8),
    #                 Line2D([0], [0], marker='o', color='w', label='Scatter',
    #                        markerfacecolor='black', markersize=8)]
    # ax1.legend(custom_lines, ["Large Map", "Medium Map"], loc="upper center", bbox_to_anchor=(0, 1, 1, 0.3))
    #
    # fig.tight_layout()  # otherwise the right y-label is slightly clipped
