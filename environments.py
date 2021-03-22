"""This code contains environment parameters to be loaded for use by the SCoPP algorithm. New classes should be created
and stored here for any new environments that the user wishes to use the algorithm on. Simply copy and paste one of the
environments below and alter the values to your liking. Each class follows the same format and require the following
attributes:

    starting_position: list of lists - Starting position (in geographical coordinates) for the robots. If only one
        position is given, all robots start at that position)
    boundary_points: list of lists - Vertices of the polygon which defines the entire survey area (in
        geographical coordinates). The vertices must be in the order which they are connected; either clockwise or
        counterclockwise.
    geo_fencing_holes: list of lists of lists - Vertices of the polygon which defines each discontnuity in the survey
        area (in geographical coordinates). The vertices must be in the order which they are connected; either clockwise
        or counterclockwise.
    robot_FOV: int, float - Downward field of view of the robots in degrees
    robot_operating_height: int, float - Height at which the robots will be flying
    robot_velocity: int, float - Velocity of the robots
    save_path: string - Directory for output data to be sent
**Optional:
    UAV: int - used to store and choose between multiple UAV parameters for a single environment
"""


class Debugger:
    """Robot parameter Class for debugging. This is a simple polygon with a low total area to reduce computation time
    substantially, to make debugging much faster
    """

    def __init__(self):
        self.starting_position = [[40.68251, -73.91134]]
        self.boundary_points = [[40.68251, -73.91134], [40.68250, -73.90935],
                                [40.68173, -73.90935], [40.68176, -73.91138]]
        self.geo_fencing_holes = None
        self.robot_FOV = 150  # degrees
        self.robot_operating_height = 2  # meters
        self.robot_velocity = 10  # meters per second
        self.save_path = "Debug/"


class VeryLargeLafayetteFLood:
    def __init__(self, UAV):
        self.starting_position = [[30.31600, -91.89790], [30.27491, -91.89797], [30.33890, -92.07346]] * 10
        self.boundary_points = [[30.27665, -91.94890],
                                [30.35969, -91.94836],
                                [30.37132, -91.99706],
                                [30.35519, -92.00796],
                                [30.31936, -92.00466],
                                [30.25465, -91.99934]]
        self.geo_fencing_holes = []
        self.save_path = "VeryLargeLafayetteFlood/"
        if UAV == 0:  # Testing
            self.robot_FOV = 105  # degrees
            self.robot_operating_height = 20  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
            self.robot_FOV = 75  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
            self.robot_FOV = 94  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 15  # meters per second
        if UAV == 3:  # Parrot Anafi (max flight range: 4km)
            self.robot_FOV = 84  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 12  # meters per second
        if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
            self.robot_FOV = 110  # degrees
            self.robot_operating_height = 8  # meters
            self.robot_velocity = 8  # meters per second
        if UAV == 5:  # DJI Matrice 300 RTK (max flight range: 15km)
            self.robot_FOV = 14  # degrees
            self.robot_operating_height = 100  # meters
            self.robot_velocity = 10  # meters per second


class SmallLafayetteFLood:
    def __init__(self, UAV=0, mode=False):
        self.boundary_points = [[30.2472, -92.151],
                                [30.247, -92.1426],
                                [30.2464, -92.1427],
                                [30.2418, -92.1472],
                                [30.243, -92.1501],
                                [30.245, -92.1516]]

        self.starting_position = [[30.2436, -92.145]]
        if mode:
            # Optimal Specs
            self.robot_FOV = 14  # degrees
            self.robot_operating_height = 100  # meters
            self.robot_velocity = 10  # meters per second
            # Optimal Specs
            self.robot_FOV = 21  # degrees
            self.robot_operating_height = 100  # meters
            self.robot_velocity = 10  # meters per second

            if mode[0] == "cont":
                self.geo_fencing_holes = []
                if mode[1] == "nopath":
                    self.save_path = "SmallLafayetteFlood/no_discontinuities/no_path_planning/"
                elif mode[1] == "path":
                    self.save_path = "SmallLafayetteFlood/no_discontinuities/path_planning/"
                elif mode[1] == "conres":
                    self.save_path = "SmallLafayetteFlood/no_discontinuities/conflict_resolution/"
                elif mode[1] == "noconres":
                    self.save_path = "SmallLafayetteFlood/no_discontinuities/no_conflict_resolution/"
                else:
                    self.save_path = "SmallLafayetteFlood/no_discontinuities/"


            elif mode[0] == "disc":
                self.geo_fencing_holes = [
                    [[30.2465, -92.1481], [30.2454, -92.1474], [30.2446, -92.1486], [30.2452, -92.1498], [30.2463, -92.1494]]
                ]
                if mode[1] == "nopath":
                    self.save_path = "SmallLafayetteFlood/discontinuities/no_path_planning/"
                elif mode[1] == "path":
                    self.save_path = "SmallLafayetteFlood/discontinuities/path_planning/"
                elif mode[1] == "conres":
                    self.save_path = "SmallLafayetteFlood/discontinuities/conflict_resolution/"
                elif mode[1] == "noconres":
                    self.save_path = "SmallLafayetteFlood/discontinuities/no_conflict_resolution/"
                else:
                    self.save_path = "SmallLafayetteFlood/discontinuities/"
        else:
            self.geo_fencing_holes = []
            self.save_path = "SmallLafayetteFlood/"
            if UAV == 0:  # Map Comparison
                self.robot_FOV = 105  # degrees
                self.robot_operating_height = 12  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
                self.robot_FOV = 75  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
                self.robot_FOV = 94  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 15  # meters per second
            if UAV == 3:  # Parrot Anafi (max flight range: 4km)
                self.robot_FOV = 84  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 12  # meters per second
            if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
                self.robot_FOV = 110  # degrees
                self.robot_operating_height = 8  # meters
                self.robot_velocity = 8  # meters per second
            if UAV == 5:  # DJI Matrice 300 RTK (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 40  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 6:
                # Optimal Specs
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 100  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 7:
                # Testing
                self.robot_FOV = 25  # degrees
                self.robot_operating_height = 100  # meters
                self.robot_velocity = 10  # meters per second

class MediumLafayetteFLood:
    def __init__(self, UAV=0, mode=False):
        self.boundary_points = [[30.24610, -92.03380],
                                [30.24430, -92.04200],
                                [30.23530, -92.04290],
                                [30.23480, -92.03470],
                                [30.24290, -92.03210]]
        self.geo_fencing_holes = []
        if mode:
            if mode == "dispatchers_T1":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]  # North
                self.save_path = "MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T1_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            elif mode == "dispatchers_T2":
                self.starting_position = [[30.24686, -92.03722] for i in range(25)]  # North
                self.starting_position.extend([30.23410, -92.03780] for i in range(25))  # South
                self.save_path = "MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T2_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            elif mode == "dispatchers_T3":
                self.starting_position = [[30.24686, -92.03722] for i in range(16)]  # North
                self.starting_position.extend([30.23410, -92.03780] for i in range(16))  # South
                self.starting_position.extend([30.24104, -92.04399] for i in range(18))  # East
                self.save_path = "MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T3_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            elif mode == "dispatchers_T4":
                self.starting_position = [[30.24686, -92.03722] for i in range(12)]  # North
                self.starting_position.extend([30.23410, -92.03780] for i in range(12))  # South
                self.starting_position.extend([30.24104, -92.04399] for i in range(12))  # East
                self.starting_position.extend([30.24086, -92.03034] for i in range(14))  # West
                self.save_path = "MediumLafayetteFlood/Dispatchers_Tests/dispatchers_T4_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second

        else:
            self.starting_position = [[30.24686, -92.03722]]
            self.save_path = "MediumLafayetteFlood/"

            if UAV == 0:  # Testing
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
                self.robot_FOV = 75  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
                self.robot_FOV = 94  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 15  # meters per second
            if UAV == 3:  # Parrot Anafi (max flight range: 4km)
                self.robot_FOV = 84  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 12  # meters per second
            if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
                self.robot_FOV = 110  # degrees
                self.robot_operating_height = 8  # meters
                self.robot_velocity = 8  # meters per second
            if UAV == 5:  # DJI Matrice 300 RTK (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 40  # meters
                self.robot_velocity = 10  # meters per second


class LargeLafayetteFLood:
    def __init__(self, UAV=0, mode=False):
        self.boundary_points = [[30.27560, -92.12400],
                                [30.28350, -92.11940],
                                [30.28590, -92.12670],
                                [30.28990, -92.12330],
                                [30.29000, -92.13870],
                                [30.28180, -92.14530],
                                [30.27760, -92.13980],
                                [30.27460, -92.13650],
                                [30.27330, -92.13050]]
        self.geo_fencing_holes = []

        if mode:
            if mode == "height_T25":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Height_Tests/height_T25_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 25  # meters
                self.robot_velocity = 10  # meters per second
            elif mode == "height_T50":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Height_Tests/height_T50_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 10  # meters per second
            elif mode == "height_T75":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Height_Tests/height_T75_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 75  # meters
                self.robot_velocity = 10  # meters per second
            elif mode == "height_T100":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Height_Tests/height_T100_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 100  # meters
                self.robot_velocity = 10  # meters per second
            elif mode == "velocity_T2":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Velocity_Tests/velocity_T2_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 2  # meters per second
            elif mode == "velocity_T4":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Velocity_Tests/velocity_T4_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            elif mode == "velocity_T6":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Velocity_Tests/velocity_T6_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 6  # meters per second
            elif mode == "velocity_T8":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Velocity_Tests/velocity_T8_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 8  # meters per second
            elif mode == "velocity_T10":
                self.starting_position = [[30.24686, -92.03722] for i in range(50)]
                self.save_path = "LargeLafayetteFLood/Velocity_Tests/velocity_T10_"
                # DJI Matrice 300 RTK specs (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 10  # meters per second

        else:
            self.starting_position = [[30.24686, -92.03722]]
            self.save_path = "LargeLafayetteFLood/"
            if UAV == 0:  # Testing
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 50  # meters
                self.robot_velocity = 4  # meters per second
            if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
                self.robot_FOV = 75  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 10  # meters per second
            if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
                self.robot_FOV = 94  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 15  # meters per second
            if UAV == 3:  # Parrot Anafi (max flight range: 4km)
                self.robot_FOV = 84  # degrees
                self.robot_operating_height = 10  # meters
                self.robot_velocity = 12  # meters per second
            if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
                self.robot_FOV = 110  # degrees
                self.robot_operating_height = 8  # meters
                self.robot_velocity = 8  # meters per second
            if UAV == 5:  # DJI Matrice 300 RTK (max flight range: 15km)
                self.robot_FOV = 14  # degrees
                self.robot_operating_height = 100  # meters
                self.robot_velocity = 10  # meters per second

# 34.66786, -77.24813
class Lejeune:
    def __init__(self, UAV):
        self.starting_position = [[34.66653, -77.24645]]
        self.boundary_points = [[34.66607, -77.24677],
                                [34.66631, -77.24859],
                                [34.66723, -77.24967],
                                [34.66780, -77.24813],
                                [34.66734, -77.24578]]
        self.geo_fencing_holes = []
        self.save_path = "Lejeune/"
        if UAV == 0:  # Testing
            self.robot_FOV = 105  # degrees
            self.robot_operating_height = 12  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
            self.robot_FOV = 75  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
            self.robot_FOV = 94  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 15  # meters per second
        if UAV == 3:  # Parrot Anafi (max flight range: 4km)
            self.robot_FOV = 84  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 12  # meters per second
        if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
            self.robot_FOV = 100  # degrees
            self.robot_operating_height = 8  # meters
            self.robot_velocity = 8  # meters per second


class Benning:
    def __init__(self, UAV):
        self.starting_position = [[32.38856, -84.81078]]
        self.boundary_points = [[32.38886, -84.81030],
                                [32.39025, -84.81050],
                                [32.39163, -84.81087],
                                [32.39158, -84.81236],
                                [32.38991, -84.81217],
                                [32.38838, -84.81141],
                                [32.38811, -84.81050]]
        self.geo_fencing_holes = [
            [[32.38991, -84.81119], [32.38970, -84.81137], [32.38949, -84.81113], [32.38976, -84.81097]],
            [[32.39132, -84.81172], [32.39105, -84.81164], [32.39114, -84.81123], [32.39142, -84.81134]]
        ]
        self.save_path = "Benning/"
        if UAV == 0:  # Testing
            self.robot_FOV = 105  # degrees
            self.robot_operating_height = 12  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
            self.robot_FOV = 75  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
            self.robot_FOV = 94  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 15  # meters per second
        if UAV == 3:  # Parrot Anafi (max flight range: 4km)
            self.robot_FOV = 84  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 12  # meters per second
        if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
            self.robot_FOV = 110  # degrees
            self.robot_operating_height = 8  # meters
            self.robot_velocity = 8  # meters per second


class HollandNewYorkAgriculture:
    def __init__(self, UAV):
        self.starting_position = [[42.73562, -78.56849]]
        self.boundary_points = [[42.74420, -78.56982],
                                [42.74389, -78.56535],
                                [42.74190, -78.56518],
                                [42.74184, -78.56110],
                                [42.74342, -78.56089],
                                [42.74307, -78.55728],
                                [42.73639, -78.55698],
                                [42.73655, -78.55432],
                                [42.73236, -78.55441],
                                [42.73239, -78.55634],
                                [42.72981, -78.55655],
                                [42.72892, -78.55886],
                                [42.72990, -78.56535],
                                [42.72899, -78.56552],
                                [42.72920, -78.57031]]
        self.geo_fencing_holes = [
            [[42.73690, -78.56894], [42.73694, -78.56673], [42.73501, -78.56781], [42.73499, -78.56939]],
            [[42.73631, -78.56379], [42.73629, -78.56265], [42.73523, -78.56310], [42.73535, -78.56382]],
            [[42.73502, -78.56567], [42.73542, -78.56499], [42.73453, -78.56444], [42.73418, -78.56512]]
        ]
        self.save_path = "Holland_NY/"
        if UAV == 0:  # Testing
            self.robot_FOV = 105  # degrees
            self.robot_operating_height = 12  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 1:  # DJI Phantom 4 Pro (max flight range: 7km)
            self.robot_FOV = 75  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 10  # meters per second
        if UAV == 2:  # Autel Robotics Evo (max flight range: 7km)
            self.robot_FOV = 94  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 15  # meters per second
        if UAV == 3:  # Parrot Anafi (max flight range: 4km)
            self.robot_FOV = 84  # degrees
            self.robot_operating_height = 10  # meters
            self.robot_velocity = 12  # meters per second
        if UAV == 4:  # Yuneec Mantis Q (max flight range: 1.5km)
            self.robot_FOV = 110  # degrees
            self.robot_operating_height = 8  # meters
            self.robot_velocity = 8  # meters per second



class Baseline_Envirnonment:
    def __init__(self, solver):
        self.starting_position = [[37.53607, 15.06927]]
        self.boundary_points = [[37.53685, 15.06921],
                                [37.53682, 15.07013],
                                [37.53599, 15.07011],
                                [37.53601, 15.06954],
                                [37.53615, 15.06926],
                                [37.53616, 15.06905],
                                [37.53667, 15.06905]]
        self.geo_fencing_holes = [
            [[37.53629, 15.06946], [37.53629, 15.06953], [37.53611, 15.06953], [37.53611, 15.06946]],
            [[37.53665, 15.06926], [37.53665, 15.06932], [37.53656, 15.06937], [37.53656, 15.06926]],
            [[37.53683, 15.06952], [37.53674, 15.06969], [37.53665, 15.06968], [37.53665, 15.06957], [37.53656, 15.06957], [37.53656, 15.06950]],
            [[37.53674, 15.06976], [37.53674, 15.06984], [37.53674, 15.06990], [37.53665, 15.06993], [37.53656, 15.06988], [37.53656, 15.06983], [37.53656, 15.06975]]
            ]
        self.robot_FOV = 5 # degrees
        self.robot_operating_height = 40  # meters
        self.robot_velocity = 4  # meters per second
        if solver == "SCoPP":
            self.save_path = "Baseline_Environment/QLB_runs/"
        elif solver == "baseline":
            self.save_path = "Baseline_Environment/baseline_runs/"



class BrooklynInitialTest:
    def __init__(self, solver):
        self.starting_position = [[40.68304, -73.94323]]
        self.boundary_points = [[40.69613, -73.92880],
                                [40.68223, -73.92601],
                                [40.68091, -73.93760],
                                [40.68744, -73.93893],
                                [40.68650, -73.94743],
                                [40.69229, -73.94863],
                                [40.69287, -73.94297]]
        self.geo_fencing_holes = [[[40.69333, -73.93468],
                                   [40.69297, -73.93284],
                                   [40.69200, -73.93155],
                                   [40.69069, -73.93245],
                                   [40.69011, -73.93400],
                                   [40.69040, -73.93567],
                                   [40.69138, -73.93692],
                                   [40.69271, -73.93614]]]
        self.robot_FOV = 105  # degrees
        self.robot_operating_height = 12  # meters
        self.robot_velocity = 10  # meters per second
        if solver == "SCoPP":
            self.save_path = "Brooklyn_Init_Test/QLB_runs/"
        elif solver == "baseline":
            self.save_path = "Brooklyn_Init_Test/baseline_runs/"


class NevadaExploration:
    def __init__(self):
        self.starting_position = [[39.38447, -116.54262]]
        self.boundary_points = [[39.33668, -116.49525],
                                [39.33560, -116.59151],
                                [39.34833, -116.61177],
                                [39.36558, -116.61658],
                                [39.40987, -116.58018],
                                [39.41915, -116.56026],
                                [39.44910, -116.51458],
                                [39.45069, -116.50085]]
        self.geo_fencing_holes = None
        self.robot_FOV = 50  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second
        self.save_path = "NevadaExploration/"


class OntarioWaterRescue:
    def __init__(self):
        self.starting_position = [[44.26976, -76.24346]]
        self.boundary_points = [[44.26204, -76.27156],
                                [44.23279, -76.24362],
                                [44.23144, -76.20018],
                                [44.26818, -76.18695],
                                [44.27679, -76.22847]]
        self.robot_FOV = 50  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second
        self.save_path = "OntarioWaterRescue/"


class SanAntonioFarming:
    def __init__(self):
        self.starting_position = [[29.61902, -98.54841]]
        self.boundary_points = [[29.62933, -98.55423],
                                [29.62933, -98.55100],
                                [29.62697, -98.55099],
                                [29.62689, -98.54249],
                                [29.61861, -98.54207],
                                [29.61857, -98.55438]]
        self.robot_FOV = 50  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second
        self.save_path = "SanAntonioFarming/"
