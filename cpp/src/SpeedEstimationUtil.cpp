//
// Created by burban on 30.01.21.
//
#include "SpeedEstimation.h"
#include "constants.h"

GeometryStore SpeedEstimation::getGeometriesForRender() {
    GeometryStore geometries;
    for (auto &object: objects) {
        if (object.getState() == TrackingObjectState::active) {
            geometries.insert(std::make_pair(object.getId(),object.getBoundingBox()));
            geometries.insert(std::make_pair(object.getId()+"_sv",object.getSvRepr()));
        }
    }
    return geometries;
}

void SpeedEstimation::drawOnCVImage(cv::Mat &image,const rs2_intrinsics *intrinsics) {

    Eigen::Matrix4d inv_floor_transform = current_floor_transform.inverse().eval();

    for (auto &object: objects) {
        if (object.getState() == TrackingObjectState::active || object.getState() == TrackingObjectState::occluded)
        {
            std::vector<std::pair<int, int>> bounding_box_points;

            for (Eigen::Vector3d point : object.getBoundingBox()->GetBoxPoints()) {
                point = (inv_floor_transform * point.homogeneous()).head(3);
                auto pixel = RealSenseCamera::get2DPixelFromPoint(point);
                bounding_box_points.emplace_back(pixel);
            }
            Eigen::Vector3d center = object.getBoundingBox()->GetCenter();
            center = (inv_floor_transform * center.homogeneous()).head(3);
          auto pixel =  RealSenseCamera::get2DPixelFromPoint(center);

            cv::Point p1(bounding_box_points.at(0).first, bounding_box_points.at(0).second);
            cv::Point p2(bounding_box_points.at(4).first, bounding_box_points.at(4).second);

//
//            if (object.getState() == TrackingObjectState::active)
//                cv::rectangle(image, p1, p2, cv::Scalar(0, 255, 0), 8);


            int x = pixel.first;
            int y = pixel.second;
            int dy = 12;
            cv::Point center_p{x,y};
            cv::line(image,center_p,cv::Point{x,y},cv::Scalar(255,255,0),1);
            cv::circle(image, center_p, 3, cv::Scalar(255,255,0),-1);
            for(std::string& line : generateTextLabels(object))
            {

                cv::putText(image,line,cv::Point{x,y},CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar{255,255,0},1);
                y += dy;
            }

            Eigen::Vector3d sv_endpoint = center + object.getSpeedVector().head(3);
            pixel = RealSenseCamera::get2DPixelFromPoint(sv_endpoint);
            x = pixel.first;
            y = pixel.second;
            cv::Point sv_end{x,y};
            //cv::arrowedLine(image, center_p, sv_end,cv::Scalar(0, 255, 0), 2);

        }
    }


}

std::vector<std::string> SpeedEstimation::generateTextLabels(TrackingObject &object) {
    std::vector<std::string> labels;
    labels.emplace_back("ObjId: "+object.getId());
    Eigen::Vector4d speed_vector = object.getSpeedVector();
    labels.emplace_back(std::to_string(speed_vector(0)).substr(0,4)+ + " m/s ");
    labels.emplace_back(std::to_string(speed_vector(1)).substr(0,4)+ " m/s ");
    labels.emplace_back(std::to_string(speed_vector(2)).substr(0,4)+ " m/s ");
    labels.emplace_back(std::to_string(speed_vector(3)).substr(0,4)+ " m/s ");
    return labels;
}

Eigen::Vector3d SpeedEstimation::ComputeMinBound(
        const std::vector<Eigen::Vector3d>& points) const {
    if (points.empty()) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    return std::accumulate(
            points.begin(), points.end(), points[0],
            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.array().min(b.array()).matrix();
            });
}

Eigen::Vector3d SpeedEstimation::ComputeMaxBound(
        const std::vector<Eigen::Vector3d>& points) const {
    if (points.empty()) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    return std::accumulate(
            points.begin(), points.end(), points[0],
            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.array().max(b.array()).matrix();
            });
}


std::vector<Eigen::Vector3d> SpeedEstimation::getClusterColors()
{
    std::vector<Eigen::Vector3d> colors;
    std::vector<Eigen::Vector3d> color_set;
    colors.resize(labels.size());
    if(labels.size() == 0)
        return colors;
    int number_of_clusters = *std::max_element(labels.begin(),labels.end());
    if(number_of_clusters < 0)
    {

    }
    for(int i = 0;i < number_of_clusters;i++)
    {
        double r = ((double) std::rand() / (RAND_MAX));
        double g = ((double) std::rand() / (RAND_MAX));
        double b = ((double) std::rand() / (RAND_MAX));
        color_set.emplace_back(Eigen::Vector3d{r,g,b});
    }
    for(int i = 0; i < labels.size() ;i++)
    {
        int color = labels.at(i)-1;
        if(color < 0)
        {
            colors.at(i) = Eigen::Vector3d{0.0,0.0,0.0};
        }
        else
        {
            colors.at(i) = color_set.at(color);
        }
    }
    return colors;
}



