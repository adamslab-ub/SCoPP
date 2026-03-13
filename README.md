# SCoPP
Scalable Coverage Path Planning of Multi-Robot Teams for Monitoring Non-Convex Areas


This repository contains data and code of our SCoPP algorithm that are used to solve multi-robot cooperative path planning problem. For further information on the SCoPP algorithm set, please refer to our paper: 

[1] Collins, L., Ghassemi, P., Chowdhury, S., Dantu, K., Esfahani, E., and Doermann, D., Scalable Coverage Path Planning of Multi-Robot Teams for Monitoring Non-Convex Areas, 2021 IEEE International Conference on Robotics and Automation (ICRA 2021).

[2] KrisshnaKumar, P., Witter, J., Collins, L., Pothuri, Jagadeshwara P.K.V., Ghassemi, P., Esfahani, E., Dantu, K., and Chowdhury, S., Efficient Planning for Scalable and Load-Balanced Area Coverage by Multiple Unmanned Aerial Vehicles, ASME Journal of Computing and Information Science in Engineering, 2026. (accepted)

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
