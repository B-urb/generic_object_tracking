//
// Created by burban on 30.12.20.
//

#ifndef MASTER_CPP_SAMPLE_H
#define MASTER_CPP_SAMPLE_H
#include "Eigen/Core"
#include <math.h>

struct Sample {
    Eigen::Vector2i gridIdx{0,0};
    int bufferIdx = 0;
    Eigen::Vector3d p{0,0,0};
    Eigen::Vector3d n{0,0,0};
    double angle = 0;
    bool inGrid = true;
    static Eigen::Vector3d up;

    Sample() = default;

    bool operator>(const Sample &o) const {
        return up.dot(p) > up.dot(o.p);
    }

    bool operator<(const Sample &o) const {
        return up.dot(p) < up.dot(o.p);
    }

    bool operator==(const Sample &o) const {
        return (bufferIdx == o.bufferIdx);
    }

    void operator+=(const Sample &o) {
        n += o.n;
        p += o.p;
    }

    void operator/=(double o) {
        n /= o;
        n.normalize();
        p /= o;
    }

    // The plane distance between two samples.
    // The normals have to be normalized!
    double distance(const Sample &o) const {
        double d1 = fabs(n.dot((o.p - p)));
        double d2 = fabs(o.n.dot((o.p - p)));
        return d1 + d2;
    }

    // Distance of v to the plane described by this sample.
    double distance(const Eigen::Vector3d &v) const {
        return n.dot((v - p));
    }
};
#endif //MASTER_CPP_SAMPLE_H
