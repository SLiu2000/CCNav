from math import sin, cos, radians, pi, hypot, degrees, atan2
from scipy.optimize import minimize
from itertools import permutations
import numpy as np


def in_range(bound_1: (), bound_2: (), point: ()) -> bool:
    x_bounds, y_bounds = [bound_1[0], bound_2[0]], [bound_1[1], bound_2[1]]
    x_bounds = x_bounds if x_bounds[0] < x_bounds[1] else x_bounds[::-1]
    y_bounds = y_bounds if y_bounds[0] < y_bounds[1] else y_bounds[::-1]
    return (x_bounds[0] < point[0] < x_bounds[1]) and (y_bounds[0] < point[1] < y_bounds[1])


def fix360(x: float) -> float:
    if x > 360:
        return x - 360
    elif x < 0:
        return x + 360
    return x


def point_pos(x0, y0, d, theta):
    theta_rad = pi / 2 - radians(theta)
    return x0 + d * cos(theta_rad), (y0 + d * sin(theta_rad)) * -1


def point_pos_2(d, theta):
    return d * cos(radians(theta)), d * sin(radians(theta))


def get_rotational_relations(landmarks: []) -> float:
    return [fix360(degrees(atan2(landmarks[c[1]][1] - landmarks[c[0]][1], landmarks[c[1]][0] - landmarks[c[0]][0]))) for
            c in [i for i in permutations([i for i in range(len(landmarks))], 2)]]


def avg_rotational_offset(global_relations: [], current_landmarks: []) -> float:
    return sum([fix360(x - global_relations[index]) for index, x in
                enumerate(get_rotational_relations(current_landmarks))]) / len(global_relations)


def find_similar_groups(data: [], landmarks: [], offset: float, tolerance=.3):
    data_len = len(data)
    before_conversion = [[] for _ in range(len(landmarks))]
    new_temp_landmarks = [[] for _ in range(len(landmarks))]
    for i, d in enumerate(data):
        x, y = point_pos(0, 0, d, ((i / data_len) * 360) + offset)
        before_x, before_y = point_pos(0, 0, d, (i / data_len) * 360)
        for landmark_index, landmark in enumerate(landmarks):
            if hypot(landmark[0] - x, landmark[1] - y) < tolerance:
                new_temp_landmarks[landmark_index].append((x, y))
                before_conversion[landmark_index].append((before_x, before_y))
    return [hypot(sum([i[0] for i in x]) / len(x), sum([i[1] for i in x]) / len(x)) for x in new_temp_landmarks], [
        (sum([i[0] for i in x]) / len(x), sum([i[1] for i in x]) / len(x)) for x in before_conversion]


def gps_solve(distances_to_station, stations_coordinates):
    def error(x, c, r):
        return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

    l = len(stations_coordinates)
    S = sum(distances_to_station)
    # compute weight vector for initial guess
    W = [((l - 1) * S) / (S - w) for w in distances_to_station]
    # get initial guess of point _location
    x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
    # optimize distance from signal origin to border of spheres
    return minimize(error, x0, args=(stations_coordinates, distances_to_station), method='Nelder-Mead').x


def match_rotation(landmarks: [], new_data: [], prev_group_counts: [], dist_tolerance=.3, count_tolerance=4,
                   u_turn_tolerance=20):
    # Iterate through rotational offset
    data_len = len(new_data)
    rotated_degrees = []
    match_coefficients = [[0 for _ in range(len(landmarks))] for _ in range(data_len)]
    finalists = []
    for rotational_offset in range(data_len):
        valid_dists = []
        for i, data_point in enumerate(new_data):
            converted_offset = (rotational_offset / data_len) * 360
            deviated_point = point_pos_2(data_point, ((i / data_len) * 360) + converted_offset - 90)
            for landamrk_index, landmark in enumerate(landmarks):
                dist_to_landmark = hypot(landmark[0] - deviated_point[0], landmark[1] - deviated_point[1])
                if dist_to_landmark < dist_tolerance:
                    match_coefficients[rotational_offset][landamrk_index] += 1
                    valid_dists.append(dist_to_landmark)
                    break
        rotated_degrees.append(converted_offset)
        if (not 0 in match_coefficients[rotational_offset]) and abs(
                ((rotational_offset / data_len) * 360) - 180) > u_turn_tolerance:
            temp = 0
            for index, matched_count in enumerate(match_coefficients[rotational_offset]):
                temp += abs(prev_group_counts[index] - matched_count)
            if temp <= count_tolerance:
                finalists.append(sum(valid_dists) / len(valid_dists))
            else:
                finalists.append(999)
        else:
            finalists.append(999)
    return rotated_degrees[finalists.index(min(finalists))], match_coefficients[finalists.index(min(finalists))]


class CCNav:

    def __init__(self, global_position: [()], initial_group_count: [], tolerance=.3):
        self._last_landmarks = global_position
        self._stations = list(np.array([list(i) for i in global_position]))
        self._last_group_counts = initial_group_count
        self._location = [0, 0]
        self._global_rotational_offset = 0
        self._tolerance = tolerance
        self._global_relations = get_rotational_relations(global_position)

    def get_location(self) -> []:
        return self._location

    def get_landmarks(self) -> []:
        return self._last_landmarks

    def get_global_rotational_offset(self):
        return self._global_rotational_offset

    def update(self, lidar_data: []):
        try:
            # Calculate Rotational Offset
            offset_deg, current_group_counts = match_rotation(self._last_landmarks, lidar_data, self._last_group_counts,
                                                              dist_tolerance=self._tolerance)
            # Update group counts
            self._last_group_counts = current_group_counts
        except ZeroDivisionError:
            # Calculate Rotational Offset
            offset_deg, current_group_counts = match_rotation(self._last_landmarks, lidar_data, self._last_group_counts,
                                                              dist_tolerance=self._tolerance, u_turn_tolerance=0)
            # Update group counts
            self._last_group_counts = current_group_counts
            # Inform user of inaccurate positioning
            print(
                "IMPORTANT: NEAR U-TURN MANEUVER HAS BEEN MADE, LOCATION & ROTATIONAL PREDICTIONS MAY NO LONGER BE ACCURATE ")
        # Find current landmarks
        distances, new_landmarks = find_similar_groups(lidar_data, self._last_landmarks, offset_deg,
                                                       tolerance=self._tolerance)
        # Update global rotational offset
        self._global_rotational_offset = avg_rotational_offset(self._global_relations, new_landmarks)
        # Update landmarks
        self._last_landmarks = new_landmarks
        # Calculate position from trilateration
        self._location = gps_solve(distances, self._stations)
