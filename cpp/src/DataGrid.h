#ifndef DATAGRID_H_
#define DATAGRID_H_

#include "Grid.h"
#include <vector>
#include <stack>
#include "Eigen/Dense"
#include "constants.h"
// The DataGrid extends the Grid with output values at the grid nodes. The type of the
// output is determined by a template parameter. For example: DataGrid<double> dg;

// To use the DataGrid, first set up the grid by providing the grid construction parameters
// such as the number of dimesions, the extents of the grid (min and max) in each dimension,
// and the number of nodes per dimension N. Then, call init() to initialize the data table.
// This will compute a grid that has its first node at min and last node at max and the
// remaining nodes are evenly distributed among them in ever dimension. Each grid node is
// associated with an output value of the type that you provided as a template parameter.
// You can set and read this value using the available setAt() and getAt() methods. Nodes
// are addressed either with a dim index of type VecNu, or a flat index n of type uint.
// DataGrid inherits the coordinate conversion methods from Grid.

//Example:
//
// DataGrid<double> grid;
// grid.setDim(3); // Set the number of dimensions.
// grid.setN(Vec3u(101,201,301)); // Set the numer of nodes per dimension.
// grid.setMin(Vec3(config.xMin, config.yMin, config.zMin)); // Set the minimum values per dimension.
// grid.setMax(Vec3(config.xMax, config.yMax, config.zMax)); // Set the maximum values per dimension.
// grid.init(); // Compute the grid representation.
//
// Then, populate the grid by setting the node outputs.
//
// for (int n = 0; n < grid.getNodeCount(); n++)
//      grid.setAt(n, some value);
//
// You can query the grid at any continuous point within the grid boundaries.
// Vec3 v = grid.getAt(x);
//
// There save(), load(), and draw() function available.
// Please review the function documentations for more information.

template<typename T>
class DataGrid : public Grid {
    std::vector<T> Y; // Ouput values of all DataGrid nodes in a flat list (size: pow(N, DIM)-ish).
    std::vector<int> clusters;
    int cluster_count = 0;
    std::vector<bool> visited;
    int nodes_added = 0;

public:

    DataGrid() {}

    ~DataGrid() {}

    bool isEmpty() const { return Y.empty(); }

    // Initialize the DataGrid. The grid parameters have to have been provided beforehand.
    // Calculates the raster of the grid coordinates. The grid nodes are distributed between the
    // respective min and max values of each dimension such that the first node is located at the
    // min and the last node is located at the max. Dim, N, min, and max must be set before
    // computing the raster. Make sure to set the grid parameters first and then call this method
    // to prepare the data grid before using it.
    void init() {
        // DataGrid overrides Grid::init() so that we can resize Y and p here.
        Grid::init();
        Y.resize(getNodeCount());
    }

    // Resets the data grid to zero output, but does not change the layout of the grid.
    virtual void clear() {
        //Y.fill(0);
        memset((void *) Y.data(), 0, sizeof(T) * Y.size()); // Faster with memset.
    }


    // Sets the output value of the DataGrid node identified by flat index n to y.
    void setAt(unsigned int n, T y) {
        Y[n] = y;
    }

    // Sets the output value of the DataGrid node identified by DIM index idx to y.
    void setAt(const unsigned int *idx, T y) {
        unsigned int n = convertIndex(idx);
        Y[n] = y;
    }

    // Sets the output value of the grid node nearest to the point x.
    // x is truncated to lie inside the boundaries of the DataGrid.
    void setAt(const double *x, T y) {
        unsigned int n = getNodeFlatIndex(x);
        Y[n] = y;
    }


    // Returns the output value of the node identified by flat index n.
    T getAt(unsigned int n) const {
        return Y[n];
    }

    // Returns the output value of the node identified by DIM index idx.
    T getAt(const unsigned int *idx) const {
        unsigned int n = convertIndex(idx);
        return Y[n];
    }

    // Evaluates the DataGrid at point x using the output value of the nearest
    // grid node. x is truncated to lie inside the boundaries of the DataGrid.
    // If no data is loaded, this method returns 0.
    T getAt(const double *x) const {
        unsigned int n = getNodeFlatIndex(x);
        return Y[n];
    }

    void clusterVoxelGrid() {
        clusters.clear();
        cluster_count = 0;
        visited.clear();
        visited.resize(getNodeCount(), false);
        clusters.resize(getNodeCount(), -1);
        for (int i = 0; i < getNodeCount(); i++) {
            if (!visited.at(i) && getAt(i) > 0) {
                regionGrowing(i);
                //Check if points are noise points and if so set cluster to noise label
                if (nodes_added <  CONSTANTS::REGION_GROWING::MIN_PTS || nodes_added > CONSTANTS::REGION_GROWING::MAX_PTS)

                { for (auto &elem : clusters) {
                        if (elem == cluster_count)
                            elem = -1;
                    }
                } else {
                    cluster_count += 1;
                }
                nodes_added = 0;
            } else {
                visited.at(i) = true;
            }
        }
    }

    void regionGrowing(const unsigned int startPoint) {
        std::stack<unsigned int> pointStack;
        pointStack.push(startPoint);
        clusters.at(startPoint) = cluster_count;
        nodes_added += 1;
        bool add_point = false;
        while (!pointStack.empty()) {
            unsigned int topPoint = pointStack.top();
            pointStack.pop();
            auto neighbours = enumerateNeighborHood(topPoint, 1);
            for (auto neighbour : neighbours) {
                const unsigned int *idx = convertIndex(neighbour);
                if (getAt(neighbour) > 0)
                    add_point = true;
                else
                    add_point = false;

                if (add_point  && !visited.at(neighbour)) {
                    clusters.at(neighbour) = cluster_count;
                    nodes_added += 1;
                    visited.at(neighbour) = true;
                    pointStack.push(neighbour);
                } else {
                    visited.at(neighbour) = true;
                }
            }
        }
    }


    std::vector<int> returnLabelsForPointCloud(const std::vector<unsigned int> &node_indices) {
        std::vector<int> labels;
        labels.resize(node_indices.size(), -1);
        std::cout << labels.size() << std::endl;
        for (size_t i = 0; i < node_indices.size(); i++) {
            labels.at(i) = clusters.at(node_indices.at(i));
        }

//        for (int i = 0; i < points.size(); i++) {
//            if(!containsPoint(points[i].data()))
//                continue;
//            int idx = getNodeFlatIndex(points[i].data());
//            labels.at(i) = clusters.at(idx);
//        }
        return labels;
    }
};

#endif
