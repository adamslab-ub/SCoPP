from math import sin, cos, sqrt, atan2, radians, degrees
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
