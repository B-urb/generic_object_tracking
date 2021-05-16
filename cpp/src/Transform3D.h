//
// Created by burban on 23.01.21.
//

#ifndef MASTER_CPP_TRANSFORM3D_H
#define MASTER_CPP_TRANSFORM3D_H
#include <vector>
#include <Eigen/Dense>
#include "Sample.h"

namespace Transform3D {
    inline Eigen::Matrix4d getTransformationMatrixFromGroundPlane(Sample floorPlane){
        Eigen::Matrix4d transform;
        transform.setZero();
        Eigen::Vector3d up{0,0,1};
        auto axis = floorPlane.n.cross(up);
        axis.normalize();
        double angle = std::atan2(floorPlane.n.cross(up).norm(), floorPlane.n.dot(up));
        //double angle = n.angleTo(up);
        double z = floorPlane.n.dot(-floorPlane.p);

        double c = cos(angle);
        double s = sin(angle);
        double t = 1.0 - c;

        transform(0,0) = c + axis(0)*axis(0)*t;
        transform(1,1) = c + axis(1)*axis(1)*t;
        transform(2,2) = c + axis(2)*axis(2)*t;

        double tmp1 = axis(0)*axis(1)*t;
        double tmp2 = axis(2)*s;
        transform(1,0) = tmp1 + tmp2;
        transform(0,1) = tmp1 - tmp2;
        tmp1 = axis(0)*axis(2)*t;
        tmp2 = axis(1)*s;
        transform(2,0) = tmp1 - tmp2;
        transform(0,2) = tmp1 + tmp2;
        tmp1 = axis(1)*axis(2)*t;
        tmp2 = axis(0)*s;
        transform(2,1) = tmp1 + tmp2;
        transform(1,2) = tmp1 - tmp2;
        transform(0,3) = 0;
        transform(1,3) = 0;
        transform(2,3) = z;
        transform(3,0) = 0;
        transform(3,1) = 0;
        transform(3,2) = 0;
        transform(3,3) = 1;
        return transform;
    }
    inline void transformPoints(std::vector<Eigen::Vector3d> &points, Eigen::Matrix4d transform)
    {
        for(auto & point : points)   {
            point = (transform * point.homogeneous()).head(3);
        }
    }
}

#endif //MASTER_CPP_TRANSFORM3D_H
