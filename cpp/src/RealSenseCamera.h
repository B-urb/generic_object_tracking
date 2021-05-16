//
// Created by burban on 01.01.21.
//

#ifndef MASTER_CPP_REALSENSECAMERA_H
#define MASTER_CPP_REALSENSECAMERA_H
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "Eigen/Dense"
#include <iostream>
#include "opencv2/opencv.hpp"

class RealSenseCamera {

public:
    RealSenseCamera(bool use_framequeue = false, bool use_align = false);
    ~RealSenseCamera();
    inline static rs2_intrinsics intrinsics;


    static std::pair<int,int> get2DPixelFromPoint(Eigen::Vector3d& point);

    void warmup(int n_frames);
    void startCameraWithRecord(std::string filename);
    void startCameraFromFile(std::string filename);
    void startCamera();
    bool getNextFrame(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors);
    bool shutdown();

    static const rs2_intrinsics &getIntrinsics();
    const rs2::frame &getCurrentColorframe() const;

    void setCurrentColorframe(const rs2::frame &currentColorframe);

    const rs2::frame &getCurrentDepthframe() const;

    void setCurrentDepthframe(const rs2::frame &currentDepthframe);

private:
    rs2::pipeline pipeline;
    rs2::config config;
    rs2::pipeline_profile profile;
    rs2::align align;
    rs2::frame_queue queue{500,true};
    rs2::decimation_filter dec_filter{4.0};
    rs2::threshold_filter thres_filter{0.31,3.0};
    rs2::temporal_filter temp_filter{0.5,20,1};
    rs2::spatial_filter spat_filter;

    float depth_scale = 1000.0;
    bool use_framequeue = false;
    bool use_align = false;
    rs2::frame current_colorframe;
    rs2::frame current_depthframe;
    bool is_recording = false;

    // Specific matrices to transform from Realsense to Right hand coordinate frame
    // They represent two rotations around first the y and then the z axis // TODO: Check
    Eigen::Matrix3d transformation_matrix_1;
    Eigen::Matrix3d transformation_matrix_2;

    void start();
    void setUpIntrinsicsAndSensor();







};


#endif //MASTER_CPP_REALSENSECAMERA_H
