//
// Created by burban on 16.12.20.
//

#ifndef MASTER_CPP_TRACKINGOBJECT_H
#define MASTER_CPP_TRACKINGOBJECT_H

#include <vector>
#include <iostream>
#include <fstream>
#include "open3d/Open3D.h"
#include <random>
#include <queue>

typedef open3d::geometry::AxisAlignedBoundingBox BoundingBox;
typedef std::pair< std::chrono::time_point<std::chrono::high_resolution_clock>, Eigen::Vector3d> Measurement;


enum class TrackingObjectState {
    candidate, active, lost, occluded
};

class TrackingObject {

public:
    TrackingObject();


    void calcBoundingBox();

    void addPoint(const Eigen::Vector3d &point);

    const std::string &getId() const;

    void incrementStateCounter();

    int getStateCounter() const;

    void resetStateCounter();

    TrackingObjectState getState() const;

    void setState(TrackingObjectState state);

    std::shared_ptr<BoundingBox>  getBoundingBox() const;

    const Eigen::Vector4d &getSpeedVector() const;

    void setPoints(const std::vector<Eigen::Vector3d> &points);

    const std::vector<Eigen::Vector3d> &getPoints() const;

    const Eigen::Vector3d getObjectCenter();
    const std::vector<Eigen::Vector3d> getObjectBoundingPoints();
    void measure();
    void writeMeasurement(std::ofstream* log_file);



    const Eigen::Vector3d &getLastPosition() const;

    void setLastPosition(const Eigen::Vector3d &lastPosition);

    int getAliveFrames() const;
    void updateVelocityVector();
    void incrementAliveFrames();
    const std::chrono::time_point<std::chrono::high_resolution_clock> &getLastMeasurementTimestamp() const;

    void setLastMeasurementTimestamp(
            const std::chrono::time_point<std::chrono::high_resolution_clock> &lastMeasurementTimestamp);
    const std::shared_ptr<open3d::geometry::TriangleMesh> &getSvRepr() const;
private:
//    std::vector<Open3D::Point> points;
    TrackingObjectState state = TrackingObjectState::candidate;
    std::string id;
    std::shared_ptr<BoundingBox> bounding_box = std::make_shared<BoundingBox>(BoundingBox());
    Eigen::Vector4d speed_vector{0.0, 0.0, 0.0,0.0};
    Eigen::Vector3d last_position{0.0, 0.0, 0.0};
    std::deque<Measurement> measurements;
    std::vector<Eigen::Vector3d> points;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_measurement_timestamp;
    int state_counter = 0;
    int alive_frame = 0;
    std::shared_ptr<open3d::geometry::TriangleMesh> sv_repr =std::make_shared<open3d::geometry::TriangleMesh>();
    Eigen::Vector3d calcMedianPoint();
    std::string getUUID();




};


#endif //MASTER_CPP_TRACKINGOBJECT_H
