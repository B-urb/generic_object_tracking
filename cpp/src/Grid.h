#ifndef GRID_H_
#define GRID_H_
#include <vector>

// This is a generic DIM dimensional uniform grid construction class.
// Grid nodes are evenly distributed along each axis of the bounded
// input space such that the first node is located at the minimum and
// the last node is located at the maximum. To use the Grid, first
// provide the grid parameters using setDim(), setN(), setMin(), and
// setMax(). Then, call init()! It is crucial that you call
// init() after all parameters have been provided and before you
// make use of the Grid in any way. Example:
//
// Grid grid; // Construct a grid object.
// grid.setDim(3); // Set the number of dimensions.
// unsigned int N[3] = {101,201,301};
// grid.setN(N); // Set the number of nodes per dimension.
// double min[3] = {config.xMin, config.yMin, config.zMin};
// grid.setMin(min); // Set the minimum values per dimension.
// double max[3] = {config.xMax, config.yMax, config.zMax};
// grid.setMax(max); // Set the maximum values per dimension.
// grid.init(); // Initialize the grid representation.
//
// Now you can access the grid node coordinates using either a DIM
// dimensional unsigned integer index, or a flat unsigned integer index
// that can sequentially enummerate all grid nodes. Example:
//
// for (unsigned int n=0; n < grid.getNodeCount(); n++)
//    Vec3 coordinates = grid.getNodeCoordinates(n);
//
// A number of other nifty methods allow you to convert between the dim
// index and the flat index representations, query the nearest node index
// of any point on the grid, retrieve all eveloping nodes of a query point,
// retreive the neighborhood of a grid node, and to save and load the
// grid to and from a binary file.
//
// As a general rule, points in the grid space are represented as
// QVector<double>, dim indexes are represented as QVector<unsigned int>, and
// flat indexes are represented as unsigned int.
//
// Please note that when setting the number of dimensions with setDim()
// and when calling rasterize(), heap memory needs to be allocated and
// this is not a real time capable operation. If performance is crucial,
// construct the grid offline first. All other methods are as fast as
// can be.

class Grid
{
private:

    // Grid parameters.
    unsigned int DIM; // Number of dimensions.
    std::vector<unsigned int> N; // Number of nodes for each dimension.
    std::vector<double> max; // Input range max values for each dimension.
    std::vector<double> min; // Input range min values for each dimension.

    // Internal structure.
    unsigned int nodeCount; // nodes in total
    std::vector<std::vector<double> >  raster; // Grid coordinates.
    std::vector<unsigned int> cumN; // speeds up index to flat index transformations
    std::vector<double> stride; // speeds up getAt()
    std::vector<double> strideinv; // speeds up evaluateAt()

private:
    mutable std::vector<unsigned int> _idx; // Temporary storage for a dim index.
    mutable std::vector<double> _x; // Temporary storage for a point.

public:

    Grid();
    virtual ~Grid(){}

    // Grid structural methods.
    void setDim(unsigned int d); // Sets DIM, the number of dimensions.
    unsigned int getDim() const; // Returns DIM, the number of dimensions.
    void setMin(double minn); // Sets the lower data range boundaries.
    void setMax(double maxx); // Sets the upper data range boundaries.
    void setMin(const double* minn); // Sets the lower data range boundaries.
    void setMax(const double* maxx); // Sets the upper data range boundaries.
    const double* getMin() const;
    const double* getMax() const;
    void setN(unsigned int N_); // Sets the number of nodes per dimension.
    void setN(const unsigned int* N_); // Sets the number of nodes per dimension.
    const unsigned int* getN() const; // Returns the number of nodes per dimension.
    virtual void init(); // Generates the Grid node coordinates.

    const double* getStride() const; // Returns a reference to the DIM sized vector that contains the strides for each dimension.
    unsigned int getNodeCount() const; // Returns the total number of Grid nodes.
    unsigned int convertIndex(const unsigned int* idx) const; // Converts a DIM dimensional index to a flat index.
    const unsigned int* convertIndex(unsigned int idx) const; // Converts a flat index to a DIM dimensional index.
    const unsigned int* getNodeIndexBl(const double* x) const; // Finds the bottom left node index of the point x.
    const unsigned int* getNodeIndex(const double* x) const; // Finds the index of the point x.
    unsigned int getNodeFlatIndexBl(const double* x) const; // Finds the bottom left flat index of the point x.
    unsigned int getNodeFlatIndex(const double* x) const; // Finds the flat index of the point x.
    const double* getNodeCoordinates(const unsigned int* idx) const; // Returns the Grid coordinates of the node specified by the DIM dimensional index.
    const double* getNodeCoordinates(unsigned int idx) const; // Returns the Grid coordinates of the node specified by the flat index.
    bool containsPoint(const double* x) const;
    bool containsIndex(const unsigned int* idx) const;
    std::vector<unsigned int> getEnvelopingNodes(const double* x, unsigned int radius=0) const; // Returns the flat indices of the nodes surrounding the query point.
    std::vector<unsigned int> enumerateNeighborHood(const unsigned int* idx, unsigned int radius=1) const; // Returns a list of flat indexes of the neighbors of node idx within radius r.
    std::vector<unsigned int> enumerateNeighborHood(unsigned int idx, unsigned int radius=1) const; // Returns a list of flat indexes of the neighbors of node idx within radius r.
    const double* uniformSample() const; // Returns a uniformly sampeled point from the grid space.


};

#endif /* Grid_H_ */
