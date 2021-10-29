"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""
# Import the necessary modules:
import monitoring_algorithms
import environments as envs

# Initialize environment class
environment = envs.Debugger()

# Initialize monitoring algorithm instance
way_point_allocator = monitoring_algorithms.QLB(5, environment, plot="full")

# Run the algorithm on the given environment and display all information
paths = way_point_allocator.run(info="verbose")