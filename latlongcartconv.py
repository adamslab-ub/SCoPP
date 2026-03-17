from math import sin, cos, sqrt, atan2, radians, degrees
import numpy as np

from sklearn.linear_model import LinearRegression
import numpy as np


class LLCCV:
    """
    Cartesian to geographical coordinates and vice versa object. Each instance of the class has an origin, and uses this
    origin to perform transformations to convert between cartesian space (meters) and geographical space (latitude,
    longitude).
    """
    def __init__(self, origin):
        """
        Parameters:
        ----------
            origin: list, tuple - location of the origin for the class, in latitude, longitude
        """
        self.R = 6373.0  # approximate radius of earth in km
        print(origin)
        self.lon1 = radians(origin[0])
        self.lat1 = radians(origin[1])
        self.conversion_boundaries = [[], []]
        self.conversion_vector = None

    def get_cartesian(self, point):
        """
        Parameters:
        ----------
            point: list, tuple - point for conversion to cartesian space, in latitude, longitude

        Returns:
        ----------
            x, y: list - point converted to cartesian
        """
        lon2 = radians(point[0])
        lat2 = radians(point[1])
        dlon = lon2 - self.lon1
        dlat = lat2 - self.lat1
        a = sin(dlat / 2) ** 2 + cos(self.lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = self.R * c
        theta = atan2(dlat, dlon)
        x = round(distance * cos(theta) * 1000)
        y = round(distance * sin(theta) * 1000)
        self.conversion_boundaries[0].append([x, y])
        self.conversion_boundaries[1].append([degrees(lon2), degrees(lat2)])
        return [x, y]

    def get_geographic(self, point):
        """
        Parameters:
        ----------
            point: list, tuple - point for conversion to geographical space

        Returns:
        ----------
            x, y: list - point converted to geographical space
        """
        if self.conversion_vector is not None:
            point = list(point)
            point_mat = np.zeros((2, 6))
            point.extend([1])
            point_mat[0, 0:3] = point
            point_mat[1, 3:] = point
            converted = np.matmul(point_mat, self.conversion_vector)
            return converted[1][0], converted[0][0]

        else:
            point = list(point)
            cart_bounds = np.concatenate(
                (np.array(self.conversion_boundaries[0]), np.ones((len(self.conversion_boundaries[0]), 1))), axis=1)
            geo_bounds = np.array(self.conversion_boundaries[1])
            geo_bounds_vec = []
            for item in geo_bounds:
                for value in item:
                    geo_bounds_vec.append([np.transpose(value)])
            cart_bounds_mat = np.zeros((2 * len(self.conversion_boundaries[0]), 6))
            row_count = 0
            for item in cart_bounds:
                cart_bounds_mat[row_count, 0:3] = item
                cart_bounds_mat[row_count + 1, 3:] = item
                row_count += 2
            cart_bounds_mat_inv = np.matmul(np.linalg.inv((np.matmul(np.transpose(cart_bounds_mat), cart_bounds_mat))),
                                            np.transpose(cart_bounds_mat))
            self.conversion_vector = np.matmul(cart_bounds_mat_inv, geo_bounds_vec)
            point_mat = np.zeros((2, 6))
            point.extend([1])
            point_mat[0, 0:3] = point
            point_mat[1, 3:] = point
            converted = np.matmul(point_mat, self.conversion_vector)
        return converted[1][0], converted[0][0]


import numpy as np
from sklearn.linear_model import LinearRegression

class LLCCV1:
    def __init__(self, origin=None):
        self.unreal_coords = np.array([
            (-1738.216, -4941.007),  # Top left
            (-4101.875, -2124.116),  # Bottom left
            (2931.594, -1022.578),   # Top right
            (580.792, 1779.007)      # Bottom right
        ])/100

        self.geo_coords = np.array([
            (42.995842900883865, -78.79717245516603),  # Top left
            (42.995582556201526, -78.79743877606035),  # Bottom left
            (42.99550077425493, -78.79657236672375),   # Top right
            (42.99524373865287, -78.79684321400947)    # Bottom right
        ])

        self.transform_unreal_coords()
        self.conversion_models()

    def conversion_models(self):
        # Use the transformed coordinates for model training
        self.model_x = LinearRegression().fit(self.geo_coords, self.transformed_unreal_coords[:, 0])
        self.model_y = LinearRegression().fit(self.geo_coords, self.transformed_unreal_coords[:, 1])
        self.model_lat = LinearRegression().fit(self.transformed_unreal_coords, self.geo_coords[:, 0])
        self.model_lon = LinearRegression().fit(self.transformed_unreal_coords, self.geo_coords[:, 1])
        
    def get_cartesian(self, point):
        point[0], point[1] = point[1], point[0]
        # Use the trained models to predict x and y using the geolocation
        predicted_x = self.model_x.predict([point])[0]
        predicted_y = self.model_y.predict([point])[0]
        return [predicted_x, predicted_y]
    
    def get_geographic(self, point):
        """
        Parameters:
        ----------
            point: list, tuple - point for conversion to geographical space

        Returns:
        ----------
            x, y: list - point converted to geographical space
        """
        if not isinstance(point, np.ndarray):
            point = np.array(point)
        if point.ndim == 1:
            point = point.reshape(1, -1)  # Reshape to (1, 3) if it's a single point

        #xy_coords = np.array(point)
        # Predict latitude and longitude using the x and y values
        predicted_lat = self.model_lat.predict(point)[0]
        predicted_lon = self.model_lon.predict(point)[0]
    
        # Combine lat and lon into a single array and return
        return [predicted_lat, predicted_lon]

    def transform_unreal_coords(self):
        # Find the minimum values for each axis and add a buffer to ensure all values are above zero
        min_values = np.min(self.unreal_coords)
        buffer = 1.0
        # Translate all coordinates by the absolute value of the found minimums plus the buffer
        self.transformed_unreal_coords = self.unreal_coords + np.abs(min_values) + buffer



