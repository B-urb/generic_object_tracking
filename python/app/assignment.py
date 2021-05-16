import scipy
import numpy as np


def calc_cost_matrix(objects, detections):
    number_of_objects = len(objects)
    cost_matrix = np.zeros((number_of_objects, len(detections)), dtype=float)
    difference = len(objects) - len(detections)

    for i, obj in enumerate(objects):
        for j, detection in enumerate(detections):
            p1 = obj.bounding_box.get_center()
            p2 = detection.bounding_box.get_center()
            distance = scipy.spatial.distance.euclidean(p1, p2)
            cost_matrix[i, j] = distance

    return cost_matrix

def create_assignment(objects, cost_matrix, max_distance):
    # objects are rowqs, deetections are columns
    # check were min then check if detection has no closer distance to other object

    assignment = []
    for i, obj in enumerate(objects):
        detection_index = -1
        min_in_row = np.argmin(cost_matrix[i,:])
        min_in_col = np.argmin(cost_matrix[:,min_in_row])
        min_row_element = cost_matrix[i,min_in_row]
        min_col_element = cost_matrix[min_in_col, min_in_row]
        if min_row_element == min_col_element and cost_matrix[i,min_in_row] < max_distance:
            detection_index = min_in_row
        assignment.append(detection_index)


    return assignment







def do_unbalanced_hungarian_assignment(objects, candidate_objects):
    number_of_objects = len(objects)
    cost_matrix = np.zeros((number_of_objects, number_of_objects), dtype=float)
    difference = len(objects) -len(candidate_objects)

    for i, obj in enumerate(objects):
        for j, cand_obj in enumerate(candidate_objects):
            p1 = obj.bounding_box.get_center()
            p2 = cand_obj.bounding_box.get_center()
            distance = scipy.spatial.distance.euclidean(p1, p2)
            cost_matrix[i, j] = distance
        for j in range(difference, len(objects)):
            cost_matrix[i, j] = 1000.0 #set some high distance
    assignment = scipy.optimize.linear_sum_assignment(cost_matrix)
    return assignment, cost_matrix

def do_hungarian_assignment(objects, candidate_objects):
    number_of_objects = len(objects)
    cost_matrix = np.zeros((number_of_objects, number_of_objects), dtype=float)

    for i, obj in enumerate(objects):
        for j, cand_obj in enumerate(candidate_objects):
            p1 = obj.bounding_box.get_center()
            p2 = cand_obj.bounding_box.get_center()
            distance = scipy.spatial.distance.euclidean(p1,p2)
            cost_matrix[i,j] = distance

    assignment = scipy.optimize.linear_sum_assignment(cost_matrix)
    return assignment, cost_matrix

def do_hungarian_assignment_extending(objects, candidate_objects):
    pass


