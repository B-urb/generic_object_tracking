//
// Created by burban on 31.12.20.
//

#ifndef MASTER_CPP_CONSTANTS_H
#define MASTER_CPP_CONSTANTS_H

namespace CONSTANTS
{
    namespace CAMERA {

      //constexpr unsigned int IMAGE_WIDTH = 1280;
    //constexpr unsigned int IMAGE_HEIGHT = 720;
        constexpr unsigned int IMAGE_WIDTH = 848;
        constexpr unsigned int IMAGE_HEIGHT = 480;
        const Eigen::Matrix3d transformation_matrix_1{{0,0, -1}, {0,1, 0}, {1, 0,0}};
        const Eigen::Matrix3d transformation_matrix_2{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
    }
    namespace REGION_GROWING {
        constexpr double MAX_COLOR_DISTANCE = 0.02;
        constexpr bool USE_COLOR = true;
        constexpr int MIN_PTS = 10;
        constexpr int MAX_PTS = 1000;
    }
    namespace FLOORDETECTION {

        constexpr unsigned int SAMPLESX = 64;
        constexpr unsigned int SAMPLESY = 64;
        constexpr double MERGE_THRESHOLD = 0.1;
        constexpr double PRUNE_THRESHOLD = 0.8;
        constexpr double GREED_THRESHOLD = 0.01;
        constexpr double FLOOD_THRESHOLD = 0.01;
        constexpr double FLOOR_HEIGHT= 0.05;
        constexpr double CEILING_HEIGHT= 3.00;
    }

    namespace TRACKING
    {
        constexpr int SPEED_VECTOR_UPDATE_FREQ = 3; // update speed vector each n frames
        constexpr double ASSIGNMENT_MAXDISTANCE = 0.2;
        constexpr double ASSIGNMENT_MAX_VOLUME_DIFF = 0.4;
        constexpr unsigned int MEASUREMENT_INTERVAL = 30;
    }
    namespace VOXELGRID {
        constexpr unsigned int VOXEL_GRID_DIM = 3;
        constexpr double VOXEL_GRID_CELL_SIZE = 0.04;    //in m
    }

    namespace DBSCAN {

        constexpr double MAX_DISTANCE= 0.2;
        constexpr int MIN_PTS = 25;
    }



}
#endif //MASTER_CPP_CONSTANTS_H
