//
// Created by burban on 07.03.21.
//

#ifndef MASTER_CPP_HELPER_H
#define MASTER_CPP_HELPER_H

#include "open3d/Open3D.h"
#include <math.h>

namespace Open3DHelperFunctions {
    inline std::shared_ptr<open3d::geometry::TriangleMesh> createArrow(double scale) {
        if(scale == 0)
            return std::make_shared<open3d::geometry::TriangleMesh>();
        double cone_height = scale * 0.1;
        double cylinder_height = scale * 0.9;
        double cone_radius = 0.05;
        double cylinder_radius = 0.01;
       auto mesh_frame = open3d::geometry::TriangleMesh::CreateArrow(cylinder_radius,cone_radius,cylinder_height,cone_height);
        return mesh_frame;

    }

    inline std::pair<Eigen::Matrix3d, Eigen::Matrix3d> calculateZYrotationForArrow(Eigen::Vector3d vec) {
        double gamma = atan2(vec(1), vec(0));
        Eigen::Matrix3d Rz{{cos(gamma), -sin(gamma), 0.0},
                           {sin(gamma), cos(gamma),  0.0},
                           {0.0,        0.0,         1.0}};
        vec = Rz * vec;
        double beta = atan2(vec(0), vec(2));
        Eigen::Matrix3d Ry{{cos(beta),  sin(beta), 0.0},
                           {0.0,        1.0,       0.0},
                           {-sin(beta), 0.0,       cos(beta)}};
        return std::make_pair(Rz, Ry);


    }

    inline Eigen::Matrix3d getCrossProd_matrix(Eigen::Vector3d vec) {
        Eigen::Matrix3d qCross_prod_mat{
            {0, -vec(2), vec(1)},
            {vec(2), 0, -vec(0)},
            {-vec(1), vec(0), 0}};
        return qCross_prod_mat;
    }

    inline Eigen::Matrix3d caculateAlignMat(Eigen::Vector3d vec) {
        Eigen::Matrix3d qTrans_Mat;
        double scale = vec.norm();
        vec = vec / scale;
        Eigen::Vector3d z_unit_Arr {0,0,1};
        Eigen::Matrix3d z_mat = getCrossProd_matrix(z_unit_Arr);

        Eigen::Vector3d z_c_vec = z_mat * vec;
        Eigen::Matrix3d z_c_vec_mat = getCrossProd_matrix(z_c_vec);

        if(z_unit_Arr.dot(vec) == -1) {
            qTrans_Mat = -Eigen::Matrix3d::Identity();
        }
        else if(z_unit_Arr.dot(vec) == 1) {
            qTrans_Mat = Eigen::Matrix3d::Identity();
        }
        else {
            qTrans_Mat = Eigen::Matrix3d::Identity() + z_c_vec_mat + (z_c_vec_mat * z_c_vec_mat) / (1 + z_unit_Arr.dot(vec));
        }
        qTrans_Mat *= scale;
        return qTrans_Mat;
    }

    // https://stackoverflow.com/questions/59026581/create-arrows-in-open3d
    inline std::shared_ptr<open3d::geometry::TriangleMesh> getArrow(Eigen::Vector3d origin, Eigen::Vector3d end, double scale) {
        //int scale = 20;
        Eigen::Matrix3d Ry = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
        Eigen::Vector3d vec = end - origin;
        auto rotation_matrices = calculateZYrotationForArrow(vec);
        scale = vec.norm() * 3;
        auto mesh = createArrow(scale);
        auto rotmat = caculateAlignMat(vec);
        //mesh->Rotate(rotation_matrices.second, Eigen::Vector3d{0,0,0});
       //mesh->Rotate(rotation_matrices.first, Eigen::Vector3d{0,0,0});
       mesh->Rotate(rotmat,Eigen::Vector3d{0,0,0});
        mesh->Translate(origin);
        if(scale == 0)
        mesh->PaintUniformColor(Eigen::Vector3d{0,0,0});
        else
            mesh->PaintUniformColor(Eigen::Vector3d{1,0,0});
        return mesh;




    }
}
#endif //MASTER_CPP_HELPER_H
