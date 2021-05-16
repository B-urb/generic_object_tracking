//
// Created by burban on 30.12.20.
//

#ifndef MASTER_CPP_SAMPLEGRID_H
#define MASTER_CPP_SAMPLEGRID_H
#include "Sample.h"
#include <vector>
#include <memory>
#include "constants.h"
#include "Eigen/Dense"
#include <Eigen/IterativeLinearSolvers>

typedef struct Sample Sample;

class SampleGrid {


public:
    SampleGrid();
    ~SampleGrid() = default;

    void init();
    void update(const std::vector<Eigen::Vector3d> & points);

    void setUpVector(const Eigen::Vector3d & up);
    Eigen::Vector3d getUpVector() const;

    Sample findFloor();

private:

    std::vector<std::vector<Sample>> samples;
    std::vector<Sample> prunedSamples;

    std::vector<std::vector<Sample>> planes;
    std::vector<Sample> planeAvg;
    std::vector<Sample> floorSegment;
    Sample floorPlane;

    std::vector<Sample> planeCluster;
    Eigen::Vector3d upVector{0,0,0};
    void floodFill(const Eigen::Vector2i &parentIdx);
    void prune();
    bool isIn(const Eigen::Vector2i &gridIdx) const;

};




#endif //MASTER_CPP_SAMPLEGRID_H
