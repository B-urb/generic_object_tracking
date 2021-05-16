//
// Created by burban on 01.01.21.
//

#ifndef MASTER_CPP_SPEEDESTIMATION_H
#define MASTER_CPP_SPEEDESTIMATION_H
#include "TrackingObject.h"
#include <vector>
#include "ObjectAssignment.h"
#include <iostream>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "RealSenseCamera.h"
#include "open3d/Open3D.h"
#include "DataGrid.h"
#include "algorithm"
#include "SampleGrid.h"
#include "Sample.h"
#include "Transform3D.h"
#include "constants.h"

typedef  std::unordered_map<std::string,std::shared_ptr<open3d::geometry::Geometry>> GeometryStore;
class SpeedEstimation {
public:
    SpeedEstimation();
    void step(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);
    GeometryStore getGeometriesForRender();
    void drawOnCVImage(cv::Mat &image,const rs2_intrinsics* intrinsics);
    std::vector<Eigen::Vector3d> getClusterColors();


private:
public:
    virtual ~SpeedEstimation();

private:
    void initVoxelGrid(Eigen::Vector3d &min_bound,Eigen::Vector3d &max_bound);
        std::vector<TrackingObject> objects;
    std::chrono::time_point<std::chrono::high_resolution_clock> old_time;
    unsigned int cycle_count = 0;
    std::vector<std::string> generateTextLabels(TrackingObject &object);
    DataGrid<unsigned int> voxel_grid;

    //DataGrid<Eigen::Vector3d> voxel_grid;


    Eigen::Vector3d ComputeMaxBound(
            const std::vector<Eigen::Vector3d>& points) const;
    Eigen::Vector3d ComputeMinBound(
            const std::vector<Eigen::Vector3d>& points) const;
    bool grid_initialized = false;
    SampleGrid sample_grid;
    Eigen::Matrix4d current_floor_transform;
    void processWithOpen3D(std::vector<Eigen::Vector3d> &points);
    std::vector<int> labels;
    void processObjects();
    std::unordered_map<std::string,std::ofstream> log_files;

};


#endif //MASTER_CPP_SPEEDESTIMATION_H
