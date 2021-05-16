//
// Created by burban on 30.12.20.
//

#include "SampleGrid.h"
#include "constants.h"

Eigen::Vector3d  Sample::up = Eigen::Vector3d{0,0,0};
SampleGrid::SampleGrid() {
    this->upVector(2) = 1.0;
    this->floorPlane.n = this->upVector;


}

void SampleGrid::init() {

    this->samples.clear();
    for (int k = 0; k < CONSTANTS::FLOORDETECTION::SAMPLESY; k++) {
        std::vector<Sample> V;
        for (int l = 0; l < CONSTANTS::FLOORDETECTION::SAMPLESX; l++) {
            int i = l * (CONSTANTS::CAMERA::IMAGE_WIDTH - 1) / (CONSTANTS::FLOORDETECTION::SAMPLESX - 1);
            int j = CONSTANTS::CAMERA::IMAGE_HEIGHT - 1 - k * (CONSTANTS::CAMERA::IMAGE_HEIGHT - 1) / (
                    CONSTANTS::FLOORDETECTION::SAMPLESY - 1);
            Sample sample;
            sample.gridIdx = Eigen::Vector2i{l, k};

            sample.bufferIdx = i + j * CONSTANTS::CAMERA::IMAGE_WIDTH;
            V.emplace_back(sample);
        }
        samples.emplace_back(V);
    }
}

void SampleGrid::setUpVector(const Eigen::Vector3d &up) {
    this->upVector = up;
    this->upVector.normalize();
    Sample::up = upVector;
}

bool SampleGrid::isIn(const Eigen::Vector2i &gridIdx) const {
    return samples[gridIdx(1)][gridIdx(0)].inGrid;
}

void SampleGrid::update(const std::vector<Eigen::Vector3d> & points) {
    if (upVector.dot(floorPlane.n) > 0.5) {
        setUpVector(floorPlane.n);
    }

    for (int i = 0; i < samples.size(); i++) {
        for (int j = 0; j < samples[i].size(); j++) {
            samples[i][j].p = points.at(samples[i][j].bufferIdx);
            samples[i][j].inGrid = samples[i][j].p.any();
        }
    }

    // Compute all normals.
    Eigen::Vector3d normal;
    for (int i = 0; i < samples.size(); i++) {
        for (int j = 0; j < samples[i].size(); j++) {
            if (!samples[i][j].inGrid)
                continue;

            int upIdx = i + 1;
            if (i == samples.size() - 1 || !samples[upIdx][j].inGrid)
                upIdx = i;

            int downIdx = i - 1;
            if (i == 0 || !samples[downIdx][j].inGrid)
                downIdx = i;

            int rightIdx = j + 1;
            if (j == samples[i].size() - 1 || !samples[i][rightIdx].inGrid)
                rightIdx = j;

            int leftIdx = j - 1;
            if (j == 0 || !samples[i][leftIdx].inGrid)
                leftIdx = j;

            if (upIdx == downIdx || leftIdx == rightIdx)
                continue;

            Sample &up = samples[upIdx][j];
            Sample &down = samples[downIdx][j];
            Sample &right = samples[i][rightIdx];
            Sample &left = samples[i][leftIdx];
            normal =  -((up.p - down.p).cross((right.p - left.p)));
            normal.normalize();
            samples[i][j].n = normal;
        }
    }

}

Sample SampleGrid::findFloor() {
    prune();

    if (prunedSamples.size() < 2)
        return floorPlane;

    // Sort by z coordinate.
    sort(this->prunedSamples.begin(), this->prunedSamples.end(),
         [](const Sample &a, const Sample &b) -> bool {
             return a.p(2) < b.p(2);
         });


    // Reset things.
    planes.clear();
    planeAvg.clear();
    floorSegment.clear();
    floorPlane = prunedSamples[1]; // Always accept the first cluster.

    // Start a flood fill at every point in the sorted pruned set.
    for (int i = 2; i < prunedSamples.size() - 1; i++) {
        if (!isIn(prunedSamples[i].gridIdx))
            continue;

        planeCluster.clear();
        floodFill(prunedSamples[i].gridIdx);
        if (planeCluster.size() == 1)
            continue;


        Sample avg;
        avg.n(2) = 0;
        for (int j = 0; j < planeCluster.size(); j++)
            avg += planeCluster[j];
        avg /= planeCluster.size();

        planeAvg.push_back(avg);
        planes.push_back(planeCluster);


        if (floorPlane.distance(avg) < CONSTANTS::FLOORDETECTION::MERGE_THRESHOLD) {
            floorPlane.p = (floorPlane.p * floorSegment.size() + avg.p * planeCluster.size()) /
                           (floorSegment.size() + planeCluster.size());
            floorPlane.n = (floorPlane.n * floorSegment.size() + avg.n * planeCluster.size()) /
                           (floorSegment.size() + planeCluster.size());
            floorPlane.n.normalize();
            floorSegment.insert(floorSegment.end(), planeCluster.begin(), planeCluster.end());

        }

        // If that didn't work, a huge cluster can replace a very small one.
        if (floorSegment.size() * 20 < planeCluster.size()) {
            floorPlane = avg;
            floorSegment.clear();
            floorSegment.insert(floorSegment.end(), planeCluster.begin(), planeCluster.end());

        }
    }

    if (floorSegment.size() > 2) {
        const int cols = floorSegment.size();
        //const Eigen::Matrix< double , Eigen::Dynamic, Eigen::Dynamic > A(3, cols);
        //Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::ColMajor> A;
        Eigen::MatrixX3d A;
        A.resize(cols,3);
        Eigen::VectorXd b;
        b.resize(cols);
        for (int i = 0; i < floorSegment.size(); i++) {
            A.row(i) = Eigen::Vector3d{floorSegment[i].p(0), floorSegment[i].p(1),1};
            b(i) = floorSegment[i].p(2);
        }

        Eigen::Vector3d solution =A.colPivHouseholderQr().solve(b);
        floorPlane.p(2) = floorPlane.p(0) * solution(0) + floorPlane.p(1) * solution(1) + solution(2);
        Eigen::Vector3d new_normal{-solution(0), -solution(1), 1};
        new_normal.normalize();
        floorPlane.n = new_normal;



    }
    return floorPlane;
}


void SampleGrid::floodFill(const Eigen::Vector2i &parentIdx) {
    Sample &parent = samples[parentIdx(1)][parentIdx(0)];
    if (!parent.inGrid)
        return;

    parent.inGrid = false;
    planeCluster.push_back(parent);


    if (parent.gridIdx(0) > 0) {
        Eigen::Vector2i childIdx = parent.gridIdx - Eigen::Vector2i(1, 0);
        Sample &child = samples[childIdx(1)][childIdx(0)];

        if (parent.distance(child) < CONSTANTS::FLOORDETECTION::FLOOD_THRESHOLD)
            floodFill(childIdx);
    }
    if (parent.gridIdx(0) < CONSTANTS::FLOORDETECTION::SAMPLESX - 1) {
        Eigen::Vector2i childIdx = parent.gridIdx + Eigen::Vector2i(1, 0);
        Sample &child = samples[childIdx(1)][childIdx(0)];
        if (parent.distance(child) < CONSTANTS::FLOORDETECTION::FLOOD_THRESHOLD)
            floodFill(childIdx);
    }
    if (parent.gridIdx(1) > 0) {
        Eigen::Vector2i childIdx = parent.gridIdx - Eigen::Vector2i(0, 1);
        Sample &child = samples[childIdx(1)][childIdx(0)];
        if (parent.distance(child) < CONSTANTS::FLOORDETECTION::FLOOD_THRESHOLD)
            floodFill(childIdx);
    }
    if (parent.gridIdx(1) < CONSTANTS::FLOORDETECTION::SAMPLESY - 1) {
        Eigen::Vector2i childIdx = parent.gridIdx + Eigen::Vector2i(0, 1);
        Sample &child = samples[childIdx(1)][childIdx(0)];

        if (parent.distance(child) < CONSTANTS::FLOORDETECTION::FLOOD_THRESHOLD)
            floodFill(childIdx);
    }
}

void SampleGrid::prune() {
    prunedSamples.clear();
    for (int i = 0; i < samples.size(); i++) {
        for (int j = 0; j < samples[i].size(); j++) {
            if (!samples[i][j].inGrid)
                continue;

            samples[i][j].angle = samples[i][j].n.dot(upVector); // A scalar product-based upright check.
            if (samples[i][j].angle > CONSTANTS::FLOORDETECTION::PRUNE_THRESHOLD) {
                prunedSamples.push_back(samples[i][j]);
            } else {
                samples[i][j].inGrid = false;
            }
        }
    }
}
