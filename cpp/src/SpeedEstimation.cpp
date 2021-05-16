//
// Created by burban on 01.01.21.
//

#include "SpeedEstimation.h"
#include "cstdlib"
#include "Benchmark.h"
#include "constants.h"

auto &benchmark = Benchmark::getInstance();
SpeedEstimation::SpeedEstimation() {
    sample_grid.init();
    //Initialize a 2d Double VOXELGRID, the double values represent the height of
}


void SpeedEstimation::initVoxelGrid(Eigen::Vector3d &min_bound,Eigen::Vector3d &max_bound) {
    //Get Min bound of pointcloud and add a tolerance value, then initialize the grid
    voxel_grid.setDim(CONSTANTS::VOXELGRID::VOXEL_GRID_DIM);
    Eigen::Vector3i nodes_per_dim = ((max_bound - min_bound) / CONSTANTS::VOXELGRID::VOXEL_GRID_CELL_SIZE).cwiseAbs().cast<int>();
    unsigned int * nodes = reinterpret_cast<unsigned int *>(nodes_per_dim.data());
    voxel_grid.setN(nodes);

    Eigen::Vector3d security_factor{1.0,1.0,1.0};
    min_bound -= security_factor;
    max_bound += security_factor;

    voxel_grid.setMin(min_bound.data());
    voxel_grid.setMax(max_bound.data());
    voxel_grid.init();

}

void SpeedEstimation::step(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &colors) {
    if(!grid_initialized)
    {
        Eigen::Vector3d min_bound = ComputeMinBound(points);
        Eigen::Vector3d max_bound = ComputeMaxBound(points);
        initVoxelGrid(min_bound,max_bound);
        grid_initialized = true;
    }
    voxel_grid.clear();

    //Execute the floor removal process
    benchmark.startTimeMeasure("FindingFloorPlane");
    sample_grid.update(points);
    Sample floorPlane = sample_grid.findFloor();
    benchmark.endTimeMeasure("FindingFloorPlane");

    benchmark.startTimeMeasure("TransformPoints");
    current_floor_transform = Transform3D::getTransformationMatrixFromGroundPlane(floorPlane);
    Transform3D::transformPoints(points,current_floor_transform);
    benchmark.endTimeMeasure("TransformPoints");


    benchmark.startTimeMeasure("RemovePoints");
//We remove all points from the cloud that do not exceed the floor threshold, are zero or are not contained by the grid
    bool invert = true;
    std::vector<bool> mask = std::vector<bool>(points.size(), invert);
    size_t pos = 0;
    for (auto & point : points) {
        if (point(2) < CONSTANTS::FLOORDETECTION::FLOOR_HEIGHT || point(0) <= 0 || !(voxel_grid.containsPoint(point.data())) || !point.any()) {
            mask.at(pos) = false;
        }
        ++pos;
    }
    size_t counter = 0;
    for (size_t i = 0; i < points.size(); i++) {
        if (mask[i]) {
            points.at(counter) = points.at(i);
            colors.at(counter) = colors.at(i);
            ++counter;
        }
    }
    points.resize(counter);
    colors.resize(counter);
    std::cout << points.size() << std::endl;
// Set overwrite points at i and keep counter, afterwards resize

    benchmark.endTimeMeasure("RemovePoints");

    benchmark.startTimeMeasure("BuildingGrid");

    std::vector<unsigned int> node_indices;
    node_indices.reserve(points.size());
    for(auto &point : points)
    {
        auto idx = voxel_grid.getNodeFlatIndex(point.data());
        voxel_grid.setAt(idx,1);
        node_indices.push_back(idx);
    }
    benchmark.endTimeMeasure("BuildingGrid");
    std::cout << node_indices.size() << std::endl;
    benchmark.startTimeMeasure("ClusteringRegionGrowing");
    voxel_grid.clusterVoxelGrid();
    benchmark.endTimeMeasure("ClusteringRegionGrowing");


    benchmark.startTimeMeasure("ObtainClusterLabels");
    labels = voxel_grid.returnLabelsForPointCloud(node_indices);
    //node_indices.clear(); //free memory since it is not used anymore
    benchmark.endTimeMeasure("ObtainClusterLabels");

    benchmark.startTimeMeasure("GeneratingDetections");
    int max_label = *std::max_element(std::begin(labels), std::end(labels));
    if (max_label == -1) return;
    std::vector<TrackingObject> detections;
    detections.reserve(max_label +1);
    for (int i = 0; i < max_label + 1; i++) {
        detections.emplace_back(TrackingObject{});

    }
    for (std::size_t i = 0; i < points.size(); ++i) {
        int cluster_id = labels.at(i);
        if (cluster_id != -1)
            detections.at(cluster_id).addPoint(points.at(i));
    }
//    for (int i = detections.size() -1; i >= 0; i--) {
//        TrackingObject & object = detections[i];
//        if(object.getPoints().size() < 60) //|| object.getPoints().size() > 2000)
//        {
//            detections.erase(detections.begin() +i);
//        }
//    }



    for (auto &tracking_object : detections) {
        tracking_object.calcBoundingBox();
        //Initialize member with actual value so first speed measurement is not false
        tracking_object.measure();
    }
    benchmark.endTimeMeasure("GeneratingDetections");


    if (objects.empty()) {
        objects = detections;
    return;}



    benchmark.startTimeMeasure("Assignment");
    auto assignment = ObjectAssignment::createAssignment(objects, detections);
    benchmark.endTimeMeasure("Assignment");

    benchmark.startTimeMeasure("UpdateObjects");
    if (!assignment.empty()) {
        for (int i = 0; i < assignment.size(); i++) {
            TrackingObject &tracking_object = objects.at(i);
            int index = assignment.at(i);
            if (index == -1) {
                tracking_object.incrementStateCounter();
                continue;
            }
            if (tracking_object.getState() ==
                TrackingObjectState::candidate) { tracking_object.resetStateCounter();}
            if (tracking_object.getState() == TrackingObjectState::active) { tracking_object.resetStateCounter(); }
            if (tracking_object.getState() == TrackingObjectState::lost) {
                tracking_object.setState(TrackingObjectState::active);
            }

//           if(abs(tracking_object.getBoundingBox()->Volume() - detections.at(index).getBoundingBox()->Volume()) > 0.3) {
//                tracking_object.setState(TrackingObjectState::lost);
//                continue;
//            }

            tracking_object.setPoints(detections.at(index).getPoints());
            tracking_object.calcBoundingBox();
            tracking_object.measure();
            if(cycle_count % 5 == 0)
                tracking_object.writeMeasurement(&log_files[tracking_object.getId()]);
            tracking_object.updateVelocityVector();


        }
    }
    benchmark.endTimeMeasure("UpdateObjects");
    benchmark.startTimeMeasure("CreateObjects");
    for (size_t i = 0; i < detections.size(); i++) {
        if (std::find(assignment.begin(), assignment.end(), i) == assignment.end())
            objects.emplace_back(detections.at(i));
    }
    benchmark.endTimeMeasure("CreateObjects");

    benchmark.startTimeMeasure("ObjectPostProcessing");
    processObjects();
    benchmark.endTimeMeasure("ObjectPostProcessing");
    cycle_count += 1;
}

void SpeedEstimation::processObjects() {
    for (int i = 0; i < objects.size(); i++) {
        TrackingObject &tracking_object = objects.at(i);
        if(tracking_object.getState() == TrackingObjectState::occluded)
            objects.erase(objects.begin() + i);
        tracking_object.incrementAliveFrames();

        if (tracking_object.getStateCounter() > 10 && tracking_object.getAliveFrames() > 15 &&
            tracking_object.getState() == TrackingObjectState::candidate)
            objects.erase(objects.begin() + i);
        else if (tracking_object.getState() == TrackingObjectState::candidate &&
                 tracking_object.getStateCounter() < 5 && tracking_object.getAliveFrames() > 15) {
            tracking_object.setState(TrackingObjectState::active);
            std::string id{tracking_object.getId()};
            std::ofstream file;
            file.open("../data/" + id + ".txt");
            log_files[id] = std::move(file);
        }

        if (tracking_object.getStateCounter() > 5) {
             if (tracking_object.getState() == TrackingObjectState::active)
                tracking_object.setState(TrackingObjectState::lost);

        }
        if(tracking_object.getStateCounter() > 60)
        {
            if (tracking_object.getState() == TrackingObjectState::lost)
                objects.erase(objects.begin() + i);
        }
    }
}

void SpeedEstimation::processWithOpen3D(std::vector<Eigen::Vector3d> &points)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    open3d::geometry::PointCloud pointCloud{};
    auto pc_pointer = std::make_shared<open3d::geometry::PointCloud>(pointCloud);
    pc_pointer->points_.resize(points.size());
    //pc_pointer->colors_.resize(colors.size());
    pc_pointer->points_ = points;
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "Point Cloud generation: " << duration << " ms" << std::endl;
//    pc_pointer->colors_ = colors;
//    pc_pointer->colors_ = colors;
//    open3d::visualization::DrawGeometries(std::vector<std::shared_ptr<const open3d::geometry::Geometry>>{pc_pointer});

//    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geom;
//    geom.push_back(pc_pointer);
//    open3d::visualization::DrawGeometries(std::vector<std::shared_ptr<const open3d::geometry::Geometry>>{pc_pointer});
    t1 = std::chrono::high_resolution_clock::now();
    auto voxel = pc_pointer->VoxelDownSample(CONSTANTS::VOXELGRID::VOXEL_GRID_CELL_SIZE);

    //open3d::geometry::VOXELGRID::CreateFromPointCloud(points,0.05);
    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "downsampling: " << duration << " ms" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    //auto model = pc_pointer->SegmentPlane(0.04, 3, 200);
    ///pc_pointer = pc_pointer->SelectByIndex(std::get<1>(model),true);
    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "Point segmentation " << duration << " ms" << std::endl;


    t1 = std::chrono::high_resolution_clock::now();
    //open3d::visualization::DrawGeometries(std::vector<std::shared_ptr<const open3d::geometry::Geometry>>{pc_pointer});
    //labels = pc_pointer->ClusterDBSCAN(CONSTANTS::MAX_DISTANCE, CONSTANTS::MIN_PTS, false);

    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "Clustering: " << duration << " ms" << std::endl;
}

SpeedEstimation::~SpeedEstimation() {
for(auto& file: log_files)
    file.second.close();
}



