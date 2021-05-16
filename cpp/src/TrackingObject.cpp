//
// Created by burban on 16.12.20.
//


#include "TrackingObject.h"
#include "ObjectAssignment.h"
#include "helper.h"


TrackingObject::TrackingObject() {

    //status = "candidate"  #uninit, candidate, active, occluded, lost
    //self.id = shortuuid.uuid()[0:4]
    last_measurement_timestamp = std::chrono::high_resolution_clock::now();
    id = getUUID();
}

void TrackingObject::calcBoundingBox() {
    if(bounding_box == nullptr)
    {
        bounding_box = std::make_shared<BoundingBox>();
    }
    else
    {
        *bounding_box = BoundingBox::CreateFromPoints(points);
    }


}

void TrackingObject::addPoint(const Eigen::Vector3d &point) {
    //TODO: Do we need copy ?
points.emplace_back(point);
}

const std::string &TrackingObject::getId() const {
    return id;
}


std::shared_ptr<BoundingBox> TrackingObject::getBoundingBox() const {
    return bounding_box;
}


const Eigen::Vector4d &TrackingObject::getSpeedVector() const {
    return speed_vector;
}


std::string TrackingObject::getUUID() {
    //https://stackoverflow.com/a/58467162/5529396
    static std::random_device dev;
    static std::mt19937 rng(dev());

    std::uniform_int_distribution<int> dist(0, 15);

    const char *v = "0123456789abcdef";
    std::string res;
    for (int i = 0; i < 4; i++) {
        res += v[dist(rng)];
        res += v[dist(rng)];
    }
    return res;
}

TrackingObjectState TrackingObject::getState() const {
    return state;
}

void TrackingObject::setState(TrackingObjectState state) {
    TrackingObject::state = state;
    resetStateCounter();
}

void TrackingObject::incrementStateCounter() {
    state_counter +=1;
}

void TrackingObject::resetStateCounter()  {
    state_counter = 0;
}

int TrackingObject::getStateCounter() const {
    return state_counter;
}

void TrackingObject::setPoints(const std::vector<Eigen::Vector3d> &points) {
    this->points = points;
}

const std::vector<Eigen::Vector3d> &TrackingObject::getPoints() const {
    return points;
}

const Eigen::Vector3d &TrackingObject::getLastPosition() const {
    return last_position;
}

void TrackingObject::setLastPosition(const Eigen::Vector3d &lastPosition) {
    last_position = lastPosition;
}

int TrackingObject::getAliveFrames() const {
    return alive_frame;
}

void TrackingObject::incrementAliveFrames() {
    alive_frame +=1;
}

Eigen::Vector3d TrackingObject::calcMedianPoint()
{
    Eigen::Vector3d median{0.0,0.0,0.0};
    for(Eigen::Vector3d& point : points)
        median += point;
    median /= points.size();
    return median;
}

void TrackingObject::measure() {

    if(measurements.size() <= CONSTANTS::TRACKING::MEASUREMENT_INTERVAL)
    measurements.emplace_back(std::make_pair(std::chrono::high_resolution_clock::now(),bounding_box->GetCenter()));
}

void TrackingObject::updateVelocityVector() {
    Measurement last_measure = measurements.front();
    Measurement current_measure = measurements.back();
    Eigen::Vector3d p1 = last_measure.second;
    Eigen::Vector3d p2 = current_measure.second;
    double distance = ObjectAssignment::getEuclideanDistance(p1, p2);
    auto current_timestamp = std::chrono::high_resolution_clock::now();
    auto time_elapsed = current_measure.first - last_measure.first;
    double time_elapsed_double = std::chrono::duration_cast<std::chrono::milliseconds>(
            time_elapsed).count();

    Eigen::Vector4d speed_vec;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double speed = 0.0;
    if(time_elapsed_double > 0 && distance > 0)
    {   double dx = p2(0) - p1(0);
        double dy = p2(1) - p1(1);
        double dz = p2(2) - p1(2);
        double d = sqrt(dx * dx + dy * dy + dz * dz);
        speed = (distance / time_elapsed_double) * 1000;
        vx = dx / d * speed;
        vy = dy / d * speed;
        vz = dz / d * speed;
    }
    speed_vec(0) = vx;
    speed_vec(1) = vy;
    speed_vec(2) = vz;
    speed_vec(3) = speed;
    Eigen::Vector3d::Index index;
    Eigen::Vector3d half_extent = bounding_box->GetHalfExtent();

    // Get direction with greatest speed
//   speed_vec.head(3).cwiseAbs().maxCoeff(&index);
   Eigen::Vector3d origin = bounding_box->GetCenter();
//   double he = half_extent(index);
//   if(speed_vec(index)>= 0)
//   origin(index) += half_extent(index);
//   else
//       origin(index) -= half_extent(index);
if(speed_vec(3)< 0.03)
    this->speed_vector = speed_vec;
else
    this->speed_vector = 0.7*this->speed_vector + 0.3*speed_vec;
Eigen::Vector3d speed_V = this->speed_vector.head(3);
Eigen::Vector3d end = origin+speed_V;


//speed_vec.normalize();
if(this->speed_vector(3) > 0.015)
*sv_repr = *Open3DHelperFunctions::getArrow(origin,end, speed_V.norm()*3);
else
    *sv_repr = *Open3DHelperFunctions::getArrow(Eigen::Vector3d{0,0,0},Eigen::Vector3d{0,0,0}, 0);


    if(measurements.size() >= CONSTANTS::TRACKING::MEASUREMENT_INTERVAL -1)
measurements.pop_front();
}

const std::chrono::time_point<std::chrono::high_resolution_clock> &TrackingObject::getLastMeasurementTimestamp() const {
    return last_measurement_timestamp;
}

void TrackingObject::setLastMeasurementTimestamp(
        const std::chrono::time_point<std::chrono::high_resolution_clock> &lastMeasurementTimestamp) {
    last_measurement_timestamp = lastMeasurementTimestamp;
}

const Eigen::Vector3d TrackingObject::getObjectCenter() {
    return Eigen::Vector3d();
}

const std::vector<Eigen::Vector3d> TrackingObject::getObjectBoundingPoints() {
    return std::vector<Eigen::Vector3d>();
}

const std::shared_ptr<open3d::geometry::TriangleMesh> &TrackingObject::getSvRepr() const {
    return sv_repr;
}

void TrackingObject::writeMeasurement(std::ofstream* log_file) {
    Eigen::Vector3d point =  bounding_box->GetCenter();
    *log_file << point(0) << "," << point(1) << "," << point(2)  << "," << speed_vector(3) <<"\n";
}


