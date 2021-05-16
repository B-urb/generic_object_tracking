//
// Created by burban on 02.01.21.
//

#ifndef MASTER_CPP_OBJECTASSIGNMENT_H
#define MASTER_CPP_OBJECTASSIGNMENT_H
#include <vector>
#include <math.h>
#include "TrackingObject.h"
#include <Eigen/Dense>
#include "constants.h"


namespace ObjectAssignment {
    // objects are rows, detections are columns
    // check were min then check if detection has no closeer distance to other object

    inline double getEuclideanDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
        return sqrt(pow(p1(0) - p2(0),2.0) + pow(p1(1) - p2(1),2.0) +pow(p1(2) - p2(2),2.0));

    }
    inline double getIoUDistance(TrackingObject& object, TrackingObject& detection){
        auto corners1 = object.getBoundingBox()->GetBoxPoints();
        auto corners2 = detection.getBoundingBox()->GetBoxPoints();

        double inter_vol = 0;


        double iou = inter_vol / (object.getBoundingBox()->Volume() + detection.getBoundingBox()->Volume() -inter_vol);
        return iou;
    }
    inline Eigen::MatrixXf calcCostMatrix(std::vector<TrackingObject> &objects, std::vector<TrackingObject> &detections) {
        const int number_of_objects = objects.size();
        const int number_of_detections = detections.size();
        Eigen::MatrixXf cost_matrix;
        cost_matrix.resize(number_of_objects, number_of_detections);
        cost_matrix.setZero();
        for(size_t i = 0; i < objects.size(); i++)
        {
            for(size_t j = 0; j < detections.size(); j++) {
                Eigen::Vector3d p1 = objects.at(i).getBoundingBox()->GetCenter();
                Eigen::Vector3d p2 = detections.at(j).getBoundingBox()->GetCenter();
                double distance = ObjectAssignment::getEuclideanDistance(p1, p2);
                cost_matrix(i, j) = distance;
            }
        }
        return cost_matrix;
    }



    inline std::vector<int> createAssignment(std::vector<TrackingObject> &objects, std::vector<TrackingObject> &detections) {
        std::vector<int> assignment;
        assignment.resize(objects.size(), -1);
        if(objects.empty() || detections.empty())

            return assignment;
        Eigen::MatrixXf cost_matrix = ObjectAssignment::calcCostMatrix(objects,detections);


        Eigen::VectorXf row_max = cost_matrix.rowwise().minCoeff();
        Eigen::VectorXf col_max = cost_matrix.colwise().minCoeff();

        for(size_t i = 0; i < row_max.size(); i++)
        {
            int detection_index = -1;
            int row_index, col_index;
            float val = cost_matrix.row(i).minCoeff(&row_index);
            float val2 = cost_matrix.col(row_index).minCoeff(&col_index);
            double volume_difference = abs(objects.at(i).getBoundingBox()->Volume() - detections.at(row_index).getBoundingBox()->Volume());
            if(val == val2 && val < CONSTANTS::TRACKING::ASSIGNMENT_MAXDISTANCE && volume_difference < CONSTANTS::TRACKING::ASSIGNMENT_MAX_VOLUME_DIFF) //
            {
                detection_index = row_index;
            }
            assignment.at(i) = (detection_index);
        }

        return assignment;


//        for(size_t i = 0; i < objects.size(); i++)
//        {
//            int detection_index = -1;
//            auto min_in_row =
//        }
    }


}

#endif //MASTER_CPP_OBJECTASSIGNMENT_H
