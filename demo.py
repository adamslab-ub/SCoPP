"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""

#Testing Git to see if this uploads to the online repository

# Import the necessary modules:
#from Swarm_Surveillance.SCoPP 
import monitoring_algorithms
#from Swarm_Surveillance.SCoPP 
import environments as envs
import SCoPP_settings as settings
import pandas as pd
import gc

# Initialize environment class
#environment = envs.SmallLafayetteFLood(0) # Use 5 robots
#environment = envs.soar_small() # Use 20 robots
#environment = envs.Debugger_soar()
environment = envs.MediumLafayetteFLood(0) # Use 75 robots
#environment = envs.Baseline_Envirnonment("baseline") # Use 3 robots
# environment = envs.ShastaBuffaloSmall()

#test_settings = settings.algorithm(planner="nn")
#way_point_allocator = monitoring_algorithms.QLB(environment, 5, priority_points=0, plot="full", algorithm_settings=test_settings)
#waypoints = way_point_allocator.run(info="verbose")
#planner="Elapsed_Priority"Priority_CPP
#test_settings = settings.algorithm(tests=[None])
# Initialize monitoring algorithm instance

sums = []
results_df = pd.DataFrame()

#way_point_allocator = monitoring_algorithms.QLB(environment, 5, priority_points=0, plot="full", algorithm_settings=test_settings)
#waypoints = way_point_allocator.run(info="verbose")
#adasdasdsd
# Main loop
biases = [0.5,0.75,1.0,1.5,2.0]
"""
for bs in biases:
    for i in range(100):
        test_settings = settings.algorithm(bias_factor = bs,planner="nn")
        way_point_allocator = monitoring_algorithms.QLB(environment, 75, priority_points=0, plot="full", algorithm_settings=test_settings)

        # Run the algorithm and gather data
        waypoints = way_point_allocator.run(info="verbose")
        mission_time = waypoints["mission"]["completion_time"]
        lst = waypoints["time_travelled"]

        # Create column names for each robot dynamically
        column_names = []
        for j in range(len(lst)):
            column_name = f'Robot_{j+1}'
            column_names.append(column_name)

        # Create the DataFrame using these column names
        temp_df = pd.DataFrame([lst], columns=column_names)
        temp_df.insert(0, 'Iteration', i)
        temp_df['Completion_Time'] = mission_time  # Add the completion time to the DataFrame

        # Append the results to the main DataFrame
        results_df = pd.concat([results_df, temp_df], ignore_index=True)

#print(["total average is ", sum(sums)/10])
results_df.to_csv('SCoPP_bias_test.csv', index=False)

print("Data saved to 'robot_times.csv'.")
"""
results = []                        # <- collect rows here
import gc, matplotlib.pyplot as plt, torch

for bs in biases:                   # outer loop over bias factors
    for i in range(50):            # inner loop over iterations
        
        test_settings = settings.algorithm(planner="nn")
        
        way_point_allocator = monitoring_algorithms.QLB(
            environment, 75,
            priority_points=0,
            plot="none",
            algorithm_settings=test_settings
        )
        print(i)
        # run the allocator and capture mission completion time
        mission_time = way_point_allocator.run(info="none")["mission"]["completion_time"]
        #print(mission_time)
        # add one record to the list
        results.append({
            "Iteration": i,
            "Bias": bs,
            "Completion_Time": mission_time
        })

        del way_point_allocator
        del test_settings
        del mission_time
        plt.close("all")  
        gc.collect()                      # force a collection cycle

# build the DataFrame in one shot
results_df = pd.DataFrame(results)

# save to disk
results_df.to_csv("SCoPP_bias_test.csv", index=False)