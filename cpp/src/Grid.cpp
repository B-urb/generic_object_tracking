#include "Grid.h"
#include <algorithm>
#include <cmath>
#include <random>

// This is a generic DIM dimensional uniform grid construction class.
// Grid nodes are evenly distributed along each axis of the bounded
// input space such that the first node is located at the minimum and
// the last node is located at the maximum. To use the Grid, first
// provide the grid parameters using setDim(), setN(), setMin(), and
// setMax(). Then, call rasterize()! It is crucial that you call
// rasterize() after all parameters have been provided and before you
// make use of the Grid in any way. Example:
//
// Grid grid; // Construct a grid object.
// grid.setDim(3); // Set the number of dimensions.
// unsigned int N[3] = {101,201,301};
// grid.setN(N); // Set the numer of nodes per dimension.
// double min[3] = {config.xMin, config.yMin, config.zMin};
// grid.setMin(min); // Set the minimum values per dimension.
// double max[3] = {config.xMax, config.yMax, config.zMax};
// grid.setMax(max); // Set the maximum values per dimension.
// grid.rasterize(); // Compute the grid representation.
//
// Now you can access the grid node coordinates using either a DIM
// dimensional unsigned integer index (dim index), or a one dimensional
// unsigned integer index (flat index) between 0 and nodeCount that
// enummerates all grid nodes. Example:
//
// for (unsigned int n=0; n < grid.getNodeCount(); n++)
// {
//    QVector<double> coordinates = grid.getNodeCoordinates(n);
//    QVector<unsigned int> idx = grid.convertIndex(n);
//    QVector<double> sameCoordinates = grid.getNodeCoordinates(idx);
// }
//
// As a general rule, points in the grid space are represented as
// QVector<double>, dim indexes are represented as QVector<unsigned int>, and
// flat indexes are represented as unsigned int.
//
// A number of other nifty methods allow you to convert between the dim
// and the flat index representations, query the nearest node index of
// any point on the grid, retrieve all eveloping nodes of a query point,
// retrieve the neighborhood of a grid node, and to save and load the
// grid to and from a binary file.
//
// When setting up the grid, heap memory needs to be allocated (setDim()
// and rasterize()) and this is not a real time capable operation. Grid
// construction should happen outside of time critical loops.

Grid::Grid()
{
    nodeCount = 0;
}

// Sets the number of dimensions of the grid.
void Grid::setDim(unsigned int d)
{
    DIM = d;
    N.resize(DIM);
    max.resize(DIM);
    min.resize(DIM);
    raster.resize(DIM);
    cumN.resize(DIM);
    stride.resize(DIM);
    strideinv.resize(DIM);
    _idx.resize(DIM);
    _x.resize(DIM);
}

// Returns DIM, the number of dimensions.
unsigned int Grid::getDim() const
{
    return DIM;
}

// Sets the min boundaries of the DIM dimensional data range.
// The argument double will be set as minimum for all dimensions.
void Grid::setMin(double minn)
{
    for (unsigned int d = 0; d < DIM; d++)
        min[d] = minn;
}

// Sets the max boundaries of the DIM dimensional data range.
// The argument double will be set as maximum for all dimensions.
void Grid::setMax(double maxx)
{
    for (unsigned int d = 0; d < DIM; d++)
        max[d] = maxx;
}

// Sets the min boundaries of the DIM dimensional data range.
// It is your responsibility to make sure the pointer passed as the argument
// points to a valid vector of parameters of size DIM. The util/Vec* classes
// can be used for convenience. setMin(Vec3(m1,m2,m3))
void Grid::setMin(const double *minn)
{
    for (unsigned int d = 0; d < DIM; d++)
        min[d] = minn[d];
}

// Sets the max boundaries of the DIM dimensional data range.
// It is your responsibility to make sure the pointer passed as the argument
// points to a valid vector of parameters of size DIM. The util/Vec* classes
// can be used for convenience. setMax(Vec3(m1,m2,m3))
void Grid::setMax(const double *maxx)
{
    for (unsigned int d = 0; d < DIM; d++)
        max[d] = maxx[d];
}

// Returns the min data range boundaries.
// The returned double* automatically converts to the util/Vec* classes.
// Vec3 min = grid.getMin();
const double* Grid::getMin() const
{
    return min.data();
}

// Returns the max data range boundaries.
// The returned double* automatically converts to the util/Vec* classes.
// Vec3 min = grid.getMin();
const double* Grid::getMax() const
{
    return max.data();
}

// Sets N, the number of nodes per dimension. This method sets N to be the same
// for every dimension and thus creates a uniform grid.
void Grid::setN(unsigned int N_)
{
    unsigned int N__[DIM];
    for (unsigned int d=0; d < DIM; d++)
        N__[d] = N_;
    setN(N__);
}

// Sets N, the number of nodes per dimension. This method sets a different N for
// each individual dimension.
// grid.setN(Vec3u(11,21,31));
void Grid::setN(const unsigned int* N_)
{
    for (unsigned int d=0; d < DIM; d++)
        N[d] = N_[d];
}

// Returns the number of nodes per dimension. The size of the vector is DIM, of course.
// It converts nicely to a util/Vec* class. Vec3u N = grid.getN();
const unsigned int* Grid::getN() const
{
    return N.data();
}

// Calculates the raster of the grid coordinates. The grid nodes are distributed between the
// respective min and max values of each dimension such that the first node is located at the
// min and the last node is located at the max. Dim, N, min, and max must be set before
// computing the raster. Make sure to set the grid parameters first and then call this method
// to prepare the grid before using it. Some of the methods of this class will segfault if init
// has not been called. You have been warned.
void Grid::init()
{
    nodeCount = N[0];
    for (unsigned int d = 1; d < DIM; d++)
        nodeCount *= N[d];

    // Accumulate the number of nodes per dimension to compute a "stride" in index space.
    // This speeds up index coversions.
    cumN[0] = 1;
    for (unsigned int d = 1; d < DIM; d++)
        cumN[d] = cumN[d-1]*N[d-1];

    // Compute the stride and the raster in the grid space.
    for (unsigned int d = 0; d < DIM; d++)
    {
        double l = max[d]-min[d];
        stride[d] = l/(N[d]-1);
        strideinv[d] = 1.0/stride[d];
        raster[d].resize(N[d]);
        for (unsigned int n = 0; n < N[d]; n++)
            raster[d][n] = min[d]+n*stride[d];
    }
}

// Returns a pointer to a DIM sized vector that contains the strides for each dimension.
// The stride is the size of a cell in the respective dimension.
// Vec3 strides = grid.getStride();
const double* Grid::getStride() const
{
    return stride.data();
}

// Returns the total number of Grid nodes.
unsigned int Grid::getNodeCount() const
{
    return nodeCount;
}

// Converts a DIM dimensional index to a flat index.
// unsigned int flatIdx = grid.convertIndex(dimIdx);
unsigned int Grid::convertIndex(const unsigned int *idx) const
{
    unsigned int k = idx[0];
    for (unsigned int d = 1; d < DIM; d++)
        k += idx[d]*cumN[d];
    return k;
}

// Converts a flat index to a DIM dimensional index.
// Invalidates all dim index references returned so far.
// Vec3u dimIdx = grid.convertIndex(flatIdx);
const unsigned int* Grid::convertIndex(unsigned int idx) const
{
    unsigned int v = idx;
    for (unsigned int d = 0; d < DIM; d++)
    {
        _idx[d] = v % N[d];
        v = (unsigned int)(v/N[d]);
    }
    return _idx.data();
}

// Computes the "bottom left" DIM dimensional index of the grid node that contains point x.
// Invalidates all dim index references returned so far.
const unsigned int* Grid::getNodeIndexBl(const double* x) const
{
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = (unsigned int)std::clamp(0, (int)((x[d]-min[d])*strideinv[d]), (int)N[d]-2);
    return _idx.data();
}

// Computes the DIM dimensional index of the grid node closest to the point x.
// Invalidates all dim index references returned so far.
const unsigned int* Grid::getNodeIndex(const double* x) const
{
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = (unsigned int)std::clamp(0, (int)std::round((x[d]-min[d])*strideinv[d]), (int)N[d]-1);
    return _idx.data();
}

// Computes the "bottom left" flat index of the grid node that contains point x.
// Invalidates all dim index references returned so far.
unsigned int Grid::getNodeFlatIndexBl(const double* x) const
{
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = (unsigned int)std::clamp(0, (int)((x[d]-min[d])*strideinv[d]), (int)N[d]-2);
    return convertIndex(_idx.data());
}

// Computes the flat index of the grid node closest to the point x.
// Invalidates all dim index references returned so far.
unsigned int Grid::getNodeFlatIndex(const double* x) const
{
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = (unsigned int)std::clamp(0, (int)std::round((x[d]-min[d])*strideinv[d]), (int)N[d]-1);
    return convertIndex(_idx.data());
}

// Returns the Grid coordinates of the node specified by the DIM dimensional index.
// Invalidates all point references returned so far.
const double* Grid::getNodeCoordinates(const unsigned int* idx) const
{
    for (unsigned int d = 0; d < DIM; d++)
        _x[d] = raster[d][idx[d]];
    return _x.data();
}

// Returns the Grid coordinates of the node specified by the flat index.
// Invalidates all point references returned so far.
const double* Grid::getNodeCoordinates(unsigned int n) const
{
    const unsigned int* idx = convertIndex(n);
    for (unsigned int d = 0; d < DIM; d++)
        _x[d] = raster[d][idx[d]];
    return _x.data();
}

// Enumerates the flat indexes of the nodes in a neighborhood of radius r around
// the node specified by the flat index n. The radius is interpreted as the Manhattan
// distance in index space where directly neighboring nodes have the distance 1.
// When radius = 0, the method returns only the node n itself.
// This method has to allocate heap memory for the returned vector of flat indices
// and is thus not real time capable. Invalidates all index references returned so far.
std::vector<unsigned int> Grid::enumerateNeighborHood(unsigned int n, unsigned int radius) const
{
    return enumerateNeighborHood(convertIndex(n), radius);
}

// Enumerates the flat indexes of the nodes in a neighborhood of radius r around
// the node specified by DIM index idx. The radius is interpreted as the Manhattan
// distance in index space where directly neighboring nodes have the distance 1.
// When radius = 0, the method returns only the node n itself.
// This method has to allocate heap memory for the returned vector of flat indices
// and is thus not real time capable. Invalidates all index references returned so far.
std::vector<unsigned int> Grid::enumerateNeighborHood(const unsigned int *idx, unsigned int radius) const
{
    unsigned int center = convertIndex(idx);

    // Using the radius, determine the min and max boundaries of the enveloping
    // hypercube in index space while respecting the grid boundaries.
    unsigned int bmin[DIM];
    unsigned int bmax[DIM];
    for (unsigned int d = 0; d < DIM; d++)
    {
        bmin[d] = (idx[d] < radius)? (unsigned int)0 : (idx[d] - radius);
        //bmin[d] = qMax(idx[d]-radius, (unsigned int)0);
        bmax[d] = std::min(idx[d]+radius,  N[d]-1);
        //qDebug() << idx[d] << radius << bmin[d] << bmax[d];
    }

    // Count the neighbors.
    unsigned int neighbors = 1;
    for (unsigned int d = 0; d < DIM; d++)
        neighbors *= bmax[d]-bmin[d]+1;
    // Generate the flat coordinates.
    std::vector<unsigned int> neighborhood;
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = bmin[d];

    for (unsigned int i = 0; i < neighbors; i++)
    {
        unsigned int c = convertIndex(_idx.data());
        if (c != center)
            neighborhood.push_back(c);
        unsigned int d = 0;
        while (d < DIM)
        {
            _idx[d]++;
            if (_idx[d] <= bmax[d])
                break;
            _idx[d] = bmin[d];
            d++;
        }
    }

    return neighborhood;
}

// Returns the flat node indixes of the hypercube that contains the given point x.
// If the radius > 0, it expands the enveloping hypercube by radius in index space
// and returns all included node indexes.
// This method has to allocate heap memory for the returned vector of flat indices
// and is thus not real time capable. Invalidates all index references returned so far.
std::vector<unsigned int> Grid::getEnvelopingNodes(const double* x, unsigned int radius) const
{
    // Determine the bl node of the hypercube that contains x.
    const unsigned int* idx = getNodeIndexBl(x);

    // Using the radius, determine the min and max boundaries of the enveloping
    // hypercube in index space while respecting the grid boundaries.
    unsigned int bmin[DIM];
    unsigned int bmax[DIM];
    for (unsigned int d = 0; d < DIM; d++)
    {
        bmin[d] = std::max(idx[d]-radius, (unsigned int)0);
        bmax[d] = std::min(idx[d]+radius+1, N[d]-1);
    }

    // Count the nodes.
    unsigned int neighbors = 1;
    for (unsigned int d = 0; d < DIM; d++)
        neighbors *= bmax[d]-bmin[d]+1;

    // Generate the flat coordinates.
    std::vector<unsigned int> neighborhood;
    for (unsigned int d = 0; d < DIM; d++)
        _idx[d] = bmin[d];
    for (unsigned int i = 0; i < neighbors; i++)
    {
        neighborhood.push_back(convertIndex(_idx.data()));

        unsigned int d = 0;
        while (d < DIM)
        {
            _idx[d]++;
            if (_idx[d] <= bmax[d])
                break;
            _idx[d] = bmin[d];
            d++;
        }
    }

    return neighborhood;
}

// Returns true if the given cartesian point is within the boundaries of the grid.
bool Grid::containsPoint(const double *x) const
{
    for (unsigned int d = 0; d < DIM; d++)
        if (x[d] > max[d] || x[d] < min[d])
            return false;

    return true;
}

// Returns true if the given idx is "in range".
bool Grid::containsIndex(const unsigned int *idx) const
{
    for (unsigned int d = 0; d < DIM; d++)
        if (idx[d] >= N[d])
            return false;
    return true;
}

// Returns a uniformly sampled point from the grid space.
// Invalidates all point references returned so far.
const double *Grid::uniformSample() const
{
    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());

    for (unsigned int d = 0; d < DIM; d++)
        //std::uniform_double_distribution<double>  distr(, range_to);
        //_x[d] = Statistics::uniformSample(min[d], max[d]);

        //_x[d] = Statistics::uniformSample(min[d], max[d]);
    return _x.data();
}



