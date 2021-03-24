# SCoPP
Scalable Coverage Path Planning of Multi-Robot Teams for Monitoring Non-Convex Areas


This repository contains data and code of our SCoPP algorithm that are used to solve multi-robot cooperative path planning problem. For further information on the SCoPP algorithm set, please refer to our paper: Collins, L., Ghassemi, P., Chowdhury, S., Dantu, K., Esfahani, E., and Doermann, D., Scalable Coverage Path Planning of Multi-Robot Teams for Monitoring Non-Convex Areas, 2021 IEEE International Conference on Robotics and Automation (ICRA 2021).

# How to Use the Code 
This section provides further information on the usage of this code:

Swarm_Surveillance is a swarm robot area survey solver
QLB/demo.py can be run and shows how the code can be used.
A monitoring algorithm class must be created in order to use the QLB algorithm. Information on the input parameters for 
this class are given in QLB/monitoring_algorithms.py 


# Dependencies
pyDOE
networkx
mlrose
scikit-learn
shapely
scipy
matplotlib
numpy
seaborn
pandas
