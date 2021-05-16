#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "open3d/Open3D.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include "src/constants.h"// std::min, std::max
#include "src/Sample.h"
#include "src/RealSenseCamera.h"
#include "src/SpeedEstimation.h"
#include "src/Benchmark.h"
#include "src/helper.h"

int main() {
    open3d::visualization::VisualizerWithKeyCallback vis;
    //vis.PrintVisualizerHelp();
    vis.CreateVisualizerWindow("PCD", CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT);

    open3d::geometry::PointCloud pointcloud;
    auto pc_pointer = std::make_shared<open3d::geometry::PointCloud>(pointcloud);
    auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame();
    std::set<std::string> added_geometries;
    GeometryStore geometry_store;
    geometry_store.insert(std::make_pair("pcd",pc_pointer));
    geometry_store.insert(std::make_pair("mesh",mesh));

    auto param = open3d::camera::PinholeCameraParameters();
    auto cam_param_read_success = open3d::io::ReadIJsonConvertible("view_point.json", param);


    bool geom_added = false;

    auto &benchmark = Benchmark::getInstance();
    benchmark.setLogToConsole(true);
    benchmark.addMetadata("Number of Points", std::to_string(CONSTANTS::CAMERA::IMAGE_WIDTH * CONSTANTS::CAMERA::IMAGE_HEIGHT));
    std::vector<Eigen::Vector3d> points;
    points.resize(CONSTANTS::CAMERA::IMAGE_WIDTH * CONSTANTS::CAMERA::IMAGE_HEIGHT + 1);
    std::vector<Eigen::Vector3d> colors;
    colors.resize(CONSTANTS::CAMERA::IMAGE_WIDTH * CONSTANTS::CAMERA::IMAGE_HEIGHT + 1);
    SpeedEstimation speed_estimation;

    bool record = true;
    int frame_count = 0;
    RealSenseCamera cam(false,true);
   // cam.startCamera();
   //std::string scene_name = "crossing_day";
   //std::string path = "/media/burban/4C9E525B256E4C98/Datasets/own/";
    std::string path = "/home/burban/Documents/";
    //std::string path = "../data/";
    //std::string filename = "../data/realsense";
    std::string scene_name;
    std::cout << "Please, enter Name for recording: ";
   std::getline (std::cin,scene_name);
   std::string filename = path  + scene_name;
//
    //std::string filename = path +"record_" + scene_name + "_" + std::to_string(CONSTANTS::CAMERA::IMAGE_WIDTH) + "x" + std::to_string(CONSTANTS::CAMERA::IMAGE_HEIGHT);

    cam.startCamera();//startCameraFromFile(filename);

//
    cv::VideoWriter video_writer;
    if(record) {
        //cam.startCameraWithRecord(filename);
        video_writer.open(filename + "_rgb.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0,
                          cv::Size(CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT*2));
    }

    cam.warmup(60);
//    auto arrow = Open3DHelperFunctions::getArrow(Eigen::Vector3d{0,0,0}, Eigen::Vector3d{2,4,-2});
//    open3d::visualization::DrawGeometries(std::vector<std::shared_ptr<const open3d::geometry::Geometry>>{arrow, mesh});
    int count = 0;
    for(;;)
    {
        auto cycle_start = std::chrono::high_resolution_clock::now();


       points.resize(CONSTANTS::CAMERA::IMAGE_WIDTH * CONSTANTS::CAMERA::IMAGE_HEIGHT);
       colors.resize(CONSTANTS::CAMERA::IMAGE_WIDTH * CONSTANTS::CAMERA::IMAGE_HEIGHT);
       benchmark.startTimeMeasure("getNextFrame");
       cam.getNextFrame(points,colors);
        benchmark.endTimeMeasure("getNextFrame");

        benchmark.startTimeMeasure("step");
        speed_estimation.step(points, colors);
        benchmark.endTimeMeasure("step");



        //Render PointCloud with Open3D
        //colors = speed_estimation.getClusterColors();

        pc_pointer->points_.resize(points.size());
        pc_pointer->colors_.resize(colors.size());
        pc_pointer->points_ = points;
        pc_pointer->colors_ = colors;


        // Get List bounding boxes of objects with status "active"
        auto geometries = speed_estimation.getGeometriesForRender();
        std::vector<std::string> keys_to_delete;
        for (auto &geom : geometry_store)
        {
            if(geom.first == "mesh" || geom.first == "pcd"){
                continue;
            }
            if (geometries.find(geom.first) == geometries.end()) {
                vis.RemoveGeometry(geom.second,false);
                added_geometries.erase(geom.first);
                keys_to_delete.emplace_back(geom.first);
            }
        }
        for (auto&& key : keys_to_delete)
            geometry_store.erase(key);
        geometry_store.merge(geometries);
        for (auto &geom :geometry_store) {
            if (added_geometries.find(geom.first) == added_geometries.end()) {
                vis.AddGeometry(geom.second,false);
                added_geometries.insert(geom.first);
            }
            else
            {
                vis.UpdateGeometry(geom.second);
            }
        }


        if (!geom_added){
            vis.ResetViewPoint(true);
            if(cam_param_read_success) {
                auto vc = vis.GetViewControl();
                auto view_control = vis.GetViewControl();
                view_control.ConvertFromPinholeCameraParameters(param, true);
            }

            geom_added = true;

        }

        vis.UpdateGeometry();
        for (auto&& key : keys_to_delete)
            geometry_store.erase(key);

        vis.PollEvents();
        vis.UpdateRender();






        cv::Mat image(cv::Size(CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT), CV_8UC3, (void *) cam.getCurrentColorframe().get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(image,image,CV_RGB2BGR);
        //speed_estimation.drawOnCVImage(image,&cam.getIntrinsics());


        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //  Mat image(Size(CAMERA_RESOLUTION_WIDTH, CAMERA_RESOLUTION_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        //Update the window with new data
        //cv::imshow("window_name", image);

       auto scr_image =  vis.CaptureScreenFloatBuffer();
       cv::Mat cv_scr_image (cv::Size(CONSTANTS::CAMERA::IMAGE_WIDTH, CONSTANTS::CAMERA::IMAGE_HEIGHT), CV_32FC3, scr_image->data_.data(), cv::Mat::AUTO_STEP);
       cv::cvtColor(cv_scr_image,cv_scr_image,CV_RGB2BGR);
       cv::Mat out;
       image.convertTo(out, CV_32FC3,1/255.0);
        //memcpy(cv_scr_image.data, scr_image->data_.data(), scr_image->data_.size()*sizeof(uint8_t));
        //
       cv::vconcat(cv_scr_image, out, cv_scr_image);
       cv_scr_image.convertTo(cv_scr_image,CV_8UC3, 255);
        if(record)
       video_writer << cv_scr_image;
       cv::imshow("window_name", cv_scr_image);
        auto key = cv::waitKey(1);

        if(key == 'g' || key == 'G')
        {
            vis.CaptureScreenImage("output_"+std::to_string(count)+".jpg");
        }
        if(key == 'q' || key=='Q')
            break;

//        auto key = cv::waitKey(1);
//        if(key == 103)
//        {
//            auto vc = vis.GetViewControl();
//            auto params = open3d::camera::PinholeCameraParameters();
//            vc.ConvertToPinholeCameraParameters(params);
//            open3d::io::WriteIJsonConvertible("view_point.json", params);
//
//        }


        auto cycle_end = std::chrono::high_resolution_clock::now();
        count++;
    }

    benchmark.printAverageRuntimesToFile("runtimes.txt");
    //cam.shutdown();
    vis.DestroyVisualizerWindow();




}

