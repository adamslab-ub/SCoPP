import numpy as np
import pickle
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#from Swarm_Surveillance.SCoPP 
import environments as tap
import imageio
from math import sqrt


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

# parameters = tap.SmallLafayetteFLood(mode=("disc", False))
parameters = tap.MediumLafayetteFLood(UAV=0)
save_path = parameters.save_path
sampling_rate = 1/10
with open(save_path + "waypoints", "rb") as f:
    paths = pickle.load(f)
coverage = np.load(save_path + "area_coverage.npy").tolist()
velocity = parameters.robot_velocity
t = np.linspace(0, (len(coverage) - 1) * sampling_rate, int(len(coverage)))
for index, data_point in enumerate(coverage):
    if data_point == max(coverage):
        del coverage[index + 1:]
print("Sampling rate:", sampling_rate)
tol = sampling_rate * 8
rt_path_x = [[] for path in range(len(paths))]
rt_path_y = [[] for path in range(len(paths))]
rt_path_cells = [[] for path in range(len(paths))]
print("time length:", len(t))
max_x = np.NINF
max_y = np.NINF
for path in paths:
    for position in path:
        if position[0] > max_x:
            max_x = position[0]
        if position[1] > max_y:
            max_y = position[1]
# plt.plot(t, coverage)
# plt.show()
coverage_over_time = [0]
# coverage = 0
small_map = np.flipud(plt.imread("SmallLafayetteFlood/SmallLafayetteWithHoles.png"))

medium_map = np.flipud(plt.imread("MediumLafayetteFlood/MediumLafayetteCropped.png"))
# medium_map = Image.open("MediumLafayetteFlood/MediumLafayette.jpg")
# medium_map.transpose(Image.TRANSPOSE)
# print(medium_map.size)
# medium_map.resize((800, 700))
# print(medium_map.size)
# plt.imshow
# plt.show()
# jkhn


for iteration, time in enumerate(t):
    # print(time, iteration)
    # print(len(paths[0:]) == 1)
    # print(len(paths[0:]))
    # if all([len(path) == 1 for p in paths]):
    #     print(paths)
    for agent, path in enumerate(paths):
    # print(path)
    #     if agent == 3:
    #     if len(path) < 2:
    #         print(path)
        if len(path) > 1:
            # if len(path) < 3:
            #     print(path)
            rt_path_x[agent].append(path[0][0])
            rt_path_y[agent].append(path[0][1])
            direction_x = (path[1][0] - path[0][0]) / sqrt((path[1][0] - path[0][0]) ** 2 +
                                                           (path[1][1] - path[0][1]) ** 2)
            direction_y = (path[1][1] - path[0][1]) / sqrt((path[1][0] - path[0][0]) ** 2 +
                                                           (path[1][1] - path[0][1]) ** 2)
            # if agent == 0:
            #     print(direction_x, direction_y)
            paths[agent][0][0] += direction_x * velocity * sampling_rate
            paths[agent][0][1] += direction_y * velocity * sampling_rate
            # print(sqrt((path[1][0] - path[0][0]) ** 2 + (path[1][1] - path[0][1]) ** 2))
            if sqrt((path[1][0] - path[0][0]) ** 2 + (path[1][1] - path[0][1]) ** 2) < tol:
                # print(sqrt((path[1][0] - path[0][0]) ** 2 + (path[1][1] - path[0][1]) ** 2))
                # dftlkjn
                # coverage += 10
                paths[agent].pop(0)
                rt_path_cells[agent].append(1)
            else:
                rt_path_cells[agent].append(0)
                # se5gkl
        else:
            rt_path_x[agent].append(path[0][0])
            rt_path_y[agent].append(path[0][1])
            rt_path_cells[agent].append(0)
    # coverage_over_time.append(coverage)
# coverage_over_time = np.array(coverage_over_time) / max(coverage_over_time) * 100
# coverage_over_time.tolist()
    # print(iteration, len(rt_path_y[0]), len(coverage[0:iteration]), len(t[0:iteration]))
    # print(rt_path_y)
# fgh
scrubbing_speed = 100
fig, ax = plt.subplots(figsize=(10, 8))
# fig.canvas.manager.full_screen_toggle() # toggle fullscreen mode
# fig.set_size_inches(15, 5)
plt.axis([0, max_x + 5, 0, max_y + 5])
plt.axis("off")
# ax1.axis("equal")
# ax2.axis([0, 110, 0, 110])
# plt.title("Agent Positions Over Time")
plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
# ax2.set_title("Total Area Surveyed Over Time")
# ax2.set_xlabel("Time (seconds)")
# ax2.set_ylabel("Total Area Surveyed (square meters)")
image_list = []
# kwargs_write = {'fps':1.0, 'quantizer':'nq'}

# manager = plt.get_current_fig_manager()
# manager.window.showMaximized()

# plt.imshow(small_map)
plt.imshow(medium_map)



# for index, path in enumerate(rt_path_y):
#     path_copy = path
#     path_copy.reverse()
#     rt_path_y[index] = (np.array(path) * -1).tolist()
# print(rt_path_y)
# print(rt_path_x)
#
# print(len(rt_path_y))
# print(len(rt_path_x))
#
# print(len(rt_path_y[0]))
# print(len(rt_path_x[0]))



uav_image = OffsetImage(plt.imread("uav_icon.png"), zoom=0.05)
previous_uav_state = [0 for agent in range(len(paths))]
current_uav_state = [0 for agent in range(len(paths))]
previous_path = [0 for agent in range(len(paths))]
current_path = [0 for agent in range(len(paths))]
for iteration in range(int(len(coverage) / scrubbing_speed)):
    for agent in range(len(paths)):
        x = rt_path_x[agent][iteration * scrubbing_speed]
        y = rt_path_y[agent][iteration * scrubbing_speed]
        if iteration > 0:
            previous_uav_state[agent] = current_uav_state[agent]
            previous_uav_state[agent].remove()
            previous_path[agent] = current_path[agent]
            del previous_path[agent][0]
        # ax.plot(rt_path_x[agent][0:iteration * scrubbing_speed], rt_path_y[agent][0:iteration * scrubbing_speed])
        # if rt_path_cells[agent][iteration * scrubbing_speed] != 0:
        #     plt.scatter(x, y, s=200, marker="s")
        ab = AnnotationBbox(uav_image, (x, y), frameon=False)
        current_uav_state[agent] = ax.add_artist(ab)
        current_path[agent] = ax.plot(rt_path_x[agent][0:iteration * scrubbing_speed], rt_path_y[agent][0:iteration * scrubbing_speed])

    # ax2.plot(t[0:iteration * scrubbing_speed], coverage_over_time[0:iteration * scrubbing_speed], c="green")
    # plt.show()
    plt.show(block=False)
    # plt.pause(0.00001)
    # ax.grid()
    # ax.set(xlabel='X', ylabel='x^{}'.format(power),
    #        title='Powers of x')
    # Used to return the plot as an image array

    fig.canvas.draw()       # draw the canvas, cache the renderer
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    image_list.append(image.reshape(fig.canvas.get_width_height()[::-1] + (3,)))
    # plt.show()
# print(image_list)
imageio.mimsave(save_path + "coverage.gif", image_list, fps=10)
