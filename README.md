# Boundingmesh

Contributors: Andre Gaschler, Quirin Fischer
License: 2-clause BSD, a permissive license, see LICENSE file; see [License](#license) for details of linked libraries

Boundingmesh is a mesh simplification tool aimed at applications performing collision analysis. It reduces mesh complexity while guaranteeing that the mesh will not shrink.

## Table of Contents
*   [Overview](#overview)
    *  [Features](#features)
    *  [Screenshots](#screenshots)
        * [GUI](#gui)
    *  [Example Models](#example-models)
*   [Usage](#usage)
	*  [License](#license)
    *  [Installation](#installation)
        * [Building from Source](#building-from-source)
        * [Binaries](#binaries)
    *  [Command-Line Tool](#command-line-tool)
        * [Arguments](#arguments)
        * [Options](#options)
    *  [GUI Application](#gui-application)
        * [Linux Build](#linux-build)
        * [Windows Build](#windows-build)
    *  [Extras](#extras)
        * [Voxelize](#voxelize)
        * [Approximate Convex Decomposition](#approximate-convex-decomposition-1)
    *  [Static Library](#static-library)
*   [Code-Guide](#code-guide)
    *  [Data Structures](#data-structures)
    *  [Mesh Decimation](#mesh-decimation)
    *  [Convex Decomposition](#convex-decomposition)
*   [Documentation](#documentation)
    *  [Command-line Tool](#command-line-tool-1)
        *  [boundingmesh-bin/main.cpp](#boundingmesh-binmaincpp)
    *  [Graphical User Interface](#graphical-user-interface)
        *  [boundingmesh-gui/gui.h](#boundingmesh-guiguih)
            *  [class MainWindow](#class-mainwindow)
            *  [class CustomLineEdit](#class-customlineedit)
        *  [boundingmesh-gui/ViewerMesh.h](#boundingmesh-guiviewermeshh)
            *  [class ViewerMesh](#class-viewermesh)
    *  [Core Library](#core-library)
        *  [boundingmesh/boundingmesh.h](#boundingmeshboundingmeshh)
        *  [boundingmesh/Primitives.h](#boundingmeshprimitivesh)
            * [class Plane](#class-plane)
            * [class Vertex](#class-vertex)
            * [class Edge](#class-edge)
            * [class Triangle](#class-triangle)
        *  [boundingmesh/Mesh.h](#boundingmeshmeshh)
            * [class VertexPosition](#class-vertexposition)
            * [class VertexPositionSet](#class-vertexpositionset)
            * [class Mesh](#class-mesh)
        *  [boundingmesh/ContractionUtils.h](#boundingmeshcontractionutilsh)
            * [class EdgeContraction](#class-edgecontraction)
            * [class ContractionIndex](#class-contractionindex)
            * [class ContractionQueue](#class-contractionqueue)
        *  [boundingmesh/Decimator.h](#boundingmeshdecimatorh)
            * [class Decimator](#class-decimator)
        *  [boundingmesh/MetricGenerator.h](#boundingmeshmetricgeneratorh)
            * [class MetricGenerator](#class-edgecontraction)

## Overview

### Features

*   Reduce mesh complexity restricted either by vertex count or by mesh error. 
*   Import/Export various standard 3D geometry file formats: `.off`, `.obj`, `.stl`, `.wrl`.
*   Simplification by successive edge collapse.
*   Choose the direction of the changes to the mesh:
    * Only expand the model (grow outward).
    * Only shrink (grow inward).
    * Closest approximation.
*   Supports multiple algorithms to estimate the simplification error.
*   Interactive GUI

### Screenshots

#### GUI
![The GUI interface.](https://github.com/Zera-/bounding-mesh/raw/master/images/ScreenshotGUI.png)
![After some simplification.](https://github.com/Zera-/bounding-mesh/raw/master/images/ScreenshotGUIdecimated.png)

### Example Models

Multiple example meshes can be found in the directory `/examples/`. Simplified output files are named with a suffix `_decimated`.

The model in `/examples/bunny/` originates from [The Stanford 3D Scanning Repository](http://graphics.stanford.edu/data/3Dscanrep/).

## Usage

### License
Boundingmesh itself is under the permissive 2-clause BSD license. However, it may use the following open source software libraries, which have other licenses: [Eigen](http://eigen.tuxfamily.org/), [Coin3D](https://bitbucket.org/Coin3D/coin/wiki/Home),
[Qt](https://www.qt.io/licensing/), [SOLID](http://dtecta.com/licence/index.html), [SoQt](https://bitbucket.org/Coin3D/soqt), [CGAL](http://www.cgal.org/license.html). Please find their source code and licenses on their respective websites.

### Installation

#### Dependencies
We rely on the linear algebra library [Eigen](http://eigen.tuxfamily.org/) for most computations.  
If you perform loading of `.wrl` files, you also need the [Coin3D](https://bitbucket.org/Coin3D/coin/wiki/Home) toolkit.  
The included GUI application also requires __QT4__ and __SoQT4__.  

#### Building from Source
We provide a CMake build file. We recommend building by

    mkdir Release
    cd Release
    cmake ..
    make

### Command-Line Tool
Boundingmesh provides an executable file to be used from the command-line. The interface is as follows:

    ./boundingmesh [options] FilenameIn [FilenameOut]

#### Arguments

*   FileIn (required): The path of the mesh file thats is to be simplified. The filename extension has to indicate one of the supported file formats `.off`, `.obj`, `.stl` or `.wrl`. Loading `.wrl` files requires the Coin library to be available.
*   FileOut (optional): The path where the resulting mesh will be stored. The filename extension determines the written format that has to be one of `.off`, `.obj`, `.stl` or `.wrl`. Defaults to `boundingmesh_[FileIn]`.

#### Options
*   --direction, -d: Direction of the simplification. `Inward`, `Outward` or `Any`.
*   --vertices, -v: The target number of vertices. 
*   --error, -e: The maximum error an edge contraction may introduce.
*   --metric, -m: The metric generation algorithm to be used. Available choices: `QEM_Classic`, `QEM_Modified`, `MinConstant`, `Diagonalization`, `Average`. 
*   --init, -i: The initialization algorithm of merge-based algorithms. Available choices: `Midpoint`, `DistancePrimitives`. 
*   --faceset: If the input format is .wrl, only the faceset with this number is loaded. Supply the index as an 0-indexed integer.
*   --colored: If the output format is .wrl, disconnected submeshes will be assigned distinct colors. No arguments taken.

### GUI Application

The application providing the graphical user interface is built from the files located in`src/boundingmesh-gui/`. 
The filenames of the binaries are respectively 

    ./boundingmesh-gui

or 
    boundingmesh-gui.exe

Building the boundingmesh software including the GUI requires the additional dependencies __Coin3, QT4, and SoQT4__ to be installed.

The GUI enables easy selection of the various decimation configurations. It shows the current mesh detail and allows interactive simplification. Contraction can be started for a single edge or batches of 100 or 5000 edges at a time. Furthermore, simplification down to a certain vertex count or error limit can be started.

#### Linux Build

Most build dependencies are available with the most common distributions.
In Ubuntu, you can install them with the command

    sudo apt-get install build-essentials cmake-curses-gui libcoin60-dev libeigen3-dev libqt4-dev libqt4-opengl-dev libsoqt4-dev libqhull6 libqhull-dev

#### Windows Build

For Windows, we recommend to find the required libraries
on the [Robotics Library Website](http://roboticslibrary.org).
Boundingmesh requires a subset of the dependencies the 
Robotics Library requires, so the build instructions from there
apply to large extent.

### Extras

Besides the main command-line tool, additional exectuables provide access to other capabilities of the boundingmesh library.

### Static Library

Boundingmesh provides a static C++ library that can be used from other software.

We provide a central header:

    #include <boundingmesh.h>

The build process will create the static library `libboundingmesh.a` to be linked against your program.

A minimal example that loads a mesh from an `.off` file, simplifies it until it has 1000 or less vertices, and saves the result:

    #include <boundingmesh.h> 
    int main(int argc, char** argv)
    {
    	boundingmesh::Mesh mesh;
    	mesh.loadOff("mesh.off");
    	boundingmesh::Decimator decimator();
    	decimator.setTargetVertices(1000);
    	boundingmesh::Mesh* result_mesh = decimator.compute(mesh);
    	result_mesh->writeOff("mesh_simplified.off");
    	delete result_mesh;
    	return 0;
    };

The following documentation describes the complete library and gives brief explanations of the provided functionality.

## Code Guide

The following is a rough textual description of the control flow of the programs and algorithms. Hopefully it eases the familiarization with the codebase.

### Data Structures

The primitives to represent 3D geometry are declared in [boundingmesh/Primitives.h](#boundingmeshprimitvesh). Vertices, edges and triangles all store their local neighbourhood. Referencing is always done through __Indices__ that are valid within the mesh.  
For example, assume you want to iterate through the adjacent vertices of one vertex. This is done by iterating over the adjacent triangles (their indices) and retrieving the triangle data to get the vertex indices.  
Meshes can be loaded from a few file formats, which all extract vertex and triangle data encoded in some way. The vertices are added to the `Mesh` object and connected to form triangles. Vertex insertion is done through a proxy class `VertexPositionSet` which unifies vertices with equal coordinates (by checking if a vertex with the same coordinates exists).  
Removal of vertices, triangles... is deferred, marking the deleted objects until a call to `Mesh::cleanAndRenumber()` is made.

### Mesh Decimation

Mesh decimation begins with the configuration of a `Decimator`. Besides setting parameters to select the error metric or the stopping criterion this also includes the selection of the mesh to process. This configuration prepares the decimation queue for the main algorithm by evaluating all edges of the mesh.  
The core decimation algorithm is located in `Decimator::compute()` and just repeatedly picks the best modification.  
The modification of the mesh is done in `Decimator::executeEdgeContraction()`. This method uses information about which parts have to be removed or reconnected collected in `Decimator::collectRemovalData()`.  
The scoring of contractions (and simultaneously the selection of inserted points) is done in `Decimator::computeEdgeContraction()` by minimizing cost functions. A cost function is represented by a matrix, which is computed in the [`MetricGenerator`](#boundingmeshmetricgeneratorh) class depending on the settings.

## Documentation

The whole library is contained in the namespace `boundingmesh`. Add it to your program by including `boundingmesh.h` and linking against `libboundingmesh.a`.  
The source code is written mostly conforming to the [Google C++ Style Guide](http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml).

### Command-line tool
####`boundingmesh-bin/main.cpp`

`int main(int argc, char** argv)`  
Reads the given arguments and adds default arguments if needed. Then the used file formats are determined and the input file loaded. The mesh is simplified and finally the results are written to the specified file.

`FileFormat getFileFormat(std::string filename)`  
Determines the file format by extracting and matching the file extension.

`enum FileFormat`  
Representation of a parsed file format.

`option::ArgStatus checkDirection(const option::Option& option, bool msg)`  
`option::ArgStatus checkInt(const option::Option& option, bool msg)`  
`option::ArgStatus checkFloat(const option::Option& option, bool msg)`  
Callback functions for the option parser.

### Graphical User Interface
#### `boundingmesh-gui/gui.h`

##### `class MainWindow`
The main window of the application, implemented using Qt. Keeps all the stateful GUI fields and 
declares callback functions for active components.

*   `MainWindow(QWidget* parent, QApplication* app)`  
    Sets up all the elements of the GUI and connects the appropriate callbacks.

*   `void loadView()`  
    Used to show a new model. Initializes the library, resets error values and displays the new mesh.

*   `void reload()`  
    Updates current vertex count and error. Also recalculates the recommendation fields and checks button states.

*   `void clickOpen()`  
    `void clickOpenExample(const std::string &file_name)`  
    `void clickOpenExample1()`  
    `void clickOpenExample2()`  
    `void clickOpenExample3()`  
    `void clickSave()`  
    `void clickReset()`  
    `void clickOuter()`  
    `void clickInner()`  
    `void clickAny()`  
    `void clickClassicQEM()`  
    `void clickModifiedQEM()`  
    `void clickMinConstant()`  
    `void clickDiagonalization()`  
    `void clickAverage()`  
    `void clickMidpoint()`  
    `void clickDistancePrimitives()`  
    `void clickDecimate()`  
    `void clickDecimateMany()`  
    `void clickDecimateManyMore()`  
    `void clickDecimateVertices()`  
    `void clickDecimateError()`  
    `void clickDecimatedSolid()`  
    `void clickDecimatedWire()`  
    `void clickDecimatedInvis()`  
    `void clickOriginalSolid()`  
    `void clickOriginalWire()`  
    `void clickOriginalInvis()`  
    Event handlers for all GUI interactions.

##### `class CustomLineEdit`
Modified LineEdit that inserts a default value when empty.

#### `boundingmesh-gui/ViewerMesh.h`
##### `class ViewerMesh`
Converts the data of a `boundingmesh::Mesh` into a Coin scene graph for display.

*   `void replaceByMesh(const boundingmesh::Mesh *mesh)`  
    Swaps the displayed mesh for a new one.

### Core Library
#### `boundingmesh/boundingmesh.h`

Main header file of the library. Collects the other top-level headers.

#### `boundingmesh/Primitives.h`

Definition of the geometric primitives used to construct meshes. The linear algebra library [Eigen](http://eigen.tuxfamily.org/) is used to represent mathematical objects and perform computations.

The geometric primitives provide no writing access to ensure the consistency of mesh data.

`typedef ... Real`  
 Representation of a real number.

`typedef ... Vector3`  
A three dimensional vector.

`typedef ... Vector4`  
A 4D-vector used for homogeneous coordinates.

`typedef ... Matrix44`  
A 4x4 matrix.

`typedef ... Index`  
An index referencing another geometric object. The type of the object and the underlying collection always depend on the context.

##### `class Plane`  
Representation of a plane in 3D space.

*   `Plane()`  
    Create an empty plane. Normal and d are initialized to zero.
*   `Plane(Vector3& normal, Real d)`  
    Define a plane using it's normal vector and the parameter d. 
*   `Plane(Vector3& normal, Vector3& point)`  
    Define a plane using it's normal vector and a point on the plane. 
*   `Plane(Vector3 point1, Vector3 point2, Vector3 point3)`  
    Construct a plane from three points. The orientation of the plane depends on the order of the given points; the normal is computed as the cross product of the positions of point2 and point3 relative to point 1.

*   `Vector3 normal`  
    The plane's normal vector.
*   `Real d`  
    Parameter for the plane equation. Points `x` on the plane satisfy `x*normal + d = 0`.

*   `Real distance(const Vector& point) const`  
    Computes the signed distance from `point` to the plane.
*   `Vector4 vector() const`  
    Returns the representation of the plane as a 4D-Vector.
*   `Matrix44 distanceMatrix() const`  
    Returns a matrix to compute the squared distance to the plane.

##### `class Vertex`
A vertex of a mesh. Contains references to all neighbouring triangles and edges.

*   `Vertex()`  
    Creates an empty vertex. Position initializes to zero.
*   `Vertex(Vector3 position)`  
    Creates a vertex at `position`. 
*   `Vertex(Real x, Real y, Real z)`  
    Creates a vertex from three coordinates.

*   `Vector3 position() const`  
    Getter for the 3D position.

*   `unsigned int nTriangles() const`  
    Number of triangles containing the vertex.
*   `unsigned int nEdges() const`  
    Number of edges containing the vertex.

*   `Index triangle(unsigned int i) const`  
    Returns the ith triangle containing the vertex.
*   `Index edge(unsigned int i) const`  
    Returns the ith edge containing the vertex.

##### `class Edge`
An edge of a mesh. Contains references to it's endpoints and the triangles it is part of. The references to the endpoints are always stored ordered to satisfy `vertex_1 < vertex_2`.

*   `Edge()`  
    Creates an empty edge.
*   `Edge(Index vertex_1, Index vertex_2)`  
    Creates an edge connecting `vertex_1` and `vertex_2`.

*   `unsigned int nTriangles() const`  
    Returns the number of triangles containing the edge.
*   `bool border() const`  
    A flag indicating whether the edge lies on the border of the mesh.

*   `Index vertex(unsigned int i) const`  
    Returns the ith endpoint of the edge. `i` may only be `0` or `1`. 
*   `Index triangle(unsigned int i) const`  
    Returns the ith triangle the edge is part of. `i` may be `0` or `1`. If `border()` is true, `i` may be only `0`.


##### `class Triangle`
A triangle of a mesh. Keeps references of it's vertices and edges. Also provides the plane containing the triangle in space.

*   `Triangle()`  
    Creates an empty triangle.
*   `Triangle(Index vertex_1, Index vertex_2, Index vertex_3)`  
    Creates a triangle with vertices `vertex_1`, `vertex_2`, `vertex_3`.

*   `Index vertex(unsigned int i) const`  
    Returns the ith vertex of the triangle. `i` may be `0`, `1` or `2`.
*   `Index edge(unsigned int i) const`  
    Returns the ith edge of the triangle. `i` may be `0`, `1` or `2`.

*   `Plane plane() const`  
    Returns the plane spanned by the triangles vertices.

#### `boundingmesh/Mesh.h`

##### `class VertexPosition`
Helper class to efficiently find indices of vertices that were added to the mesh.

*   `VertexPosition(Vector3 position)`  
    Creates a vertex without specifying it's index. Used to find the indexed vertex with the same position.
*   `VertexPosition(Index index, Vector3 position)`  
    Creates a new vertex with it's index in the mesh.

*   `Index index() const`  
    Returns the index of the vertex.

*   `friend bool operator<(const VertexPosition & lhs, const VertexPosition & rhs)`  
    `friend bool operator>(const VertexPosition & lhs, const VertexPosition & rhs)`  
    Compares two vertices by their position, component-by-component. 

##### `class VertexPositionSet`
Helper class to add vertices to a mesh by position. Efficiently finds the index if a vertex with the given position already exists.

*   `VertexPositionSet(Mesh& mesh)`  
    Creates an empty set.

*   `Index addVertex(Vector3 position)`  
    Adds a vertex with the given position and returns it's index. If there already exists a vertex on the position, it's index is returned.

##### `class Mesh`
A triangle mesh. Stores it's vertices, edges and triangles. Meshes can be changed by adding and removing vertices or triangles. Load/Save methods for some file formats are provided.

###### Construction

*   `Mesh()`  
    Creates an empty mesh.
*   `Mesh(const std::vector<Vector>& vertices,const std::vector<Index*>& triangles)`  
    Construct a mesh from a list of vertex positions and a list of index 3-tuples defining the triangles.

*   `void loadOff(const std::string filename)`  
    `void loadObj(const std::string filename)`  
    `void loadStl(const std::string filename)`  
    `void loadWrl(const std::string filename)`  
    Loads mesh data from a file. `loadWrl` requires the Coin library.

*   `void writeOff(const std::string filename)`  
    `void writeObj(const std::string filename)`  
    `void writeStl(const std::string filename)`  
    `void writeWrl(const std::string filename)`  
    Saves mesh data to a file.

###### Getter

*   `unsigned int nVertices() const`  
    Returns the total number of vertices.
*   `unsigned int nEdges() const`  
    Returns the total number of Edges.
*   `unsigned int nTriangles() const`  
    Returns the total number of triangles.

*   `const Vertex& vertex(Index i) const`  
    Returns the ith vertex of the mesh.
*   `const Edge& edge(Index i) const`  
    Returns the ith edge of the mesh.
*   `const Triangle& triangle(Index i) const`  
    Returns the ith triangle of the mesh.

*   `bool isDirty()`  
    Can be used to check if the current mesh state is currently dirty because of removal actions. 
    If it is, call `void cleanAndRenumber()` before iterating over geometric objects.

###### Modification

*   `Index addVertex(const Vector3& vertex)`  
    Adds a new vertex to the mesh.
*   `Index addTriangle(Index vertex1, Index vertex2, Index vertex3)`  
    Adds a new triangle to the mesh. Requires the three given indices to be valid.

Removal methods keep the mesh consistent, removing all references to the deleted object and deleting
empty objects (i.e. edges whose triangles were removed). Be careful when accessing the mesh data after removal 
as they leave the mesh in a dirty state.

*   `void removeVertex(Index vertex)`  
    Removes a vertex (and all triangles containing it) from the mesh.
*   `void removeTriangle(Index triangle)`  
    Removes a triangle from the mesh. 

###### Helper Methods (internal)
These methods are only used internally.

*   `void clean()`  
    Resets the mesh to an empty state.
*   `Index registerEdge(Index vertex1, Index vertex2, Index triangle)`  
    Registers a new edge within the mesh.
*   `void cleanAndRenumber()`  
    Perfoms the deferred removal of vertices and triangles. A call to `removeVertex()` or `removeTriangle()` only marks them for later removal and removes all references to the index from the mesh. `cleanAndRenumber()` removes the data and calculates new indices.

*   `std::vector<std::shared_ptr<Mesh> > computeSubmeshes()`  
    Partitions the mesh graph, every connected component is a submesh. 

###### Static methods
*   `void writeMultimeshWrl(std::vector<std::shared_ptr<Mesh> >submeshes, std::string filename, bool colored = false)`  
    Create a VRML file containing multiple meshes.
*   `static Vector3 HSVtoRGB(Vector3 color)`  
    Transformation from HSV to RGB color space.
*   `static std::vector<Vector3> generateColors(int n)`  
    Generate `n` distinct colors.

#### `boundingmesh/ContractionUtils.h`
Data structures used in the edge contraction algorithm.

##### `class EdgeContraction`
Stores information for the contraction of an edge.

*   `EdgeContraction(Index edge, Vector new_point, Real cost, const Matrix& qem)`  
    Stores the contraction data for the edge with index `edge`: The resulting point after contraction, the cost of the introduced error and the QEM matrix of the new point.

*   `Index edge() const`  
    Returns the index of the edge to be contracted.
*   `Vector new_point() const`  
    Returns the position of the new point that will be inserted into the mesh instead. 
*   `Real cost() const`  
    Returns the cost of the mesh change introduced by the contraction. 
*   `const Matrix44& qem() const`  
    Returns the QEM matrix of the new point that will be used to measure mesh error.

*   `friend bool operator<(const EdgeContraction & lhs, const EdgeContraction & rhs)`  
    `friend bool operator>(const EdgeContraction & lhs, const EdgeContraction & rhs)`  
    Compares two contractions by their cost.

##### `class ContractionIndex`
Helper class to access contractions in the `ContractionQueue` by edge index. 

*   `ContractionIndex(Index index)`  
    Creates a contraction index containing only an edge index (but no iterator to a contraction). Used for finding the corresponding contraction.
*   `ContractionIndex(Index index, std::set<EdgeContraction>::iterator contraction)`  
    Creates a new contraction index from an edge index and the iterator of the corresponding contraction.

*   `std::multiset<EdgeContraction>::iterator contraction() const`  
    Returns the iterator of the contraction.

*   `friend bool operator<(const ContractionIndex & lhs, const ContractionIndex & rhs)`  
*   `friend bool operator>(const ContractionIndex & lhs, const ContractionIndex & rhs)`  
    Compares two contraction indices by their edge index.

##### `class ContractionQueue`
Container to efficiently add/remove contractions and retrieve the cheapest contraction.

*   `ContractionQueue()`  
    Creates an empty queue.

*   `unsigned int size()`  
    Returns the current size of the queue.
*   `const EdgeContraction& first()`  
    Returns a reference to the first contraction in the queue (the one with the lowest cost).

*   `void insert(EdgeContraction contraction)`  
    Inserts a contraction into the queue. Also internally creates the contraction index to access the contraction by edge index.
*   `void remove(Index edge)`  
    Removes the contraction of the edge with index `edge` from the queue.

#### `boundingmesh/Decimator.h`
Contains all functionality for mesh simplification through edge contraction.

##### `class Decimator`
Performs mesh simplification by edge decimation.

###### Setup/Configuration

*   `enum DecimationDirection`  
    Options to control the direction of changes to the mesh. Values are `Outward`, `Inward` and `Any`.

*   `Decimator()`  
    Creates a decimator with the default configuration.
*   `Decimator(int target_vertices, Real maximum_error, DecimationDirection direction)`  
    Creates a decimator with a custom configuration. 
*   `void setTargetVertices(int target_vertices)`  
    Changes the targeted number of vertices the output mesh should be made up of.
*   `void setMaximumError(Real maximum_error)`  
    Sets the maximum error by which the output may differ from the input mesh.
*   `void setDirection(DecimationDirection direction)`  
    Sets the allowed direction of changes to the mesh. The mesh may be decimated only outwards, only inwards or in any direction.

###### Execution
*   `Mesh* compute(const Mesh& mesh)`  
    Simplifies the mesh according to the configuration. Returns a pointer to the resulting mesh.
	
###### Submethods (internal)

*   `void initialize()`  
    Initializes the matrices to compute the error function and computes the initially possible contractions.
*   `Matrix44 computeQEM(Index vertex)`  
    Computes the quadratic error matrix used to evaluate the error function near the vertex with index `vertex`.
*   `void executeEdgeContraction(EdgeContraction contraction)`  
    Executes the edge contraction. Removes the referred edge, it's vertices and adjacent triangles. Then inserts the new point and connects it to the mesh.

###### Decimation Computation (internal)

*   `EdgeContraction computeEdgeContraction(Index edge_index)`  
    Computes the new point and it's cost for contracting the edge with index `edge_index`.
*   `unsigned int nSubsets(unsigned int subset_size, unsigned int total_size)`  
    Calculates the number of `subset_size`-sized subsets of a set with `total_size` elements (the binomial coefficient).
*   `void nextSubset(std::vector<unsigned int>& indices_subset,  unsigned int total_size)`  
    Modifies the indices in `indices_subset` to iterate to the next subset of a `total_size`-sized set.

*   `bool solveConstrainedMinimisation(const Matrix44& qem, const std::vector<Plane>& constraints,`  
       ` const std::vector<unsigned int>& subset, Vector& result)`  
    Minimizes the error function given by the matrix `qem` while satisfying the constraints defined by a subset of the plane set. Stores the solution in `result`.
*   `Vector3 minimizeSubspace(const Matrix44& qadratic_cost)`  
*   `Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane)`  
*   `Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane1, Plane plane2)`  
*   `Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane1, Plane plane2, Plane plane3)`  
    Minimizes the function specified by `quadratic_cost` with respect to 0-3 conditions. Solves in the subspace satisfying the constraints, then transforms back.
*   `Vector3 minimizeLagrange(const Matrix44& qadratic_cost)`  
*   `Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane)`  
*   `Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane1, Plane plane2)`  
*   `Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane1, Plane plane2, Plane plane3)`  
    Directly minimizes the function given by `quadratic_cost` with respect to 0-3 conditions using lagrange multipliers.

#### `boundingmesh/MetricGenerator.h`
##### `enum Metric`
List of available metric generation algorithms provided by the `MetricGenerator`. Currently we support the following algorithms:  
* `ClassicQEM`  
* `ModifiedQEM`  
* `MinimizedConstant`  
* `Diagonalization`  
* `MinimalMerge`  

##### `class MetricGenerator`
Produces quadratic error metrices for use by the Decimator. Allows selection of the desired metric.
###### Initialization
*   `void setMetric(Metric metric)`  
    Select a generation algorithm.
*   `void setInitialization(Initialization initialization)`  
    Select the method a merge-based algorithm should use to generate the initial metrices for vertices.
*   `void setMesh(Mesh* mesh)`  
    Set the geometry that is being decimated.

###### Interface to Decimator
*   `Matrix44 getErrorMetric(Index edge_index)`  
    Computes and returns the error metric for contracting a given edge.
*   `void contractEdge(Index edge_index)`  
    Updates metric data to account for the contraction of an edge.
*   `void cleanAndRenumber()`  
    Applies index changes caused by mesh cleanup to the metric data.

###### General Internal Methods
*   `void initialize()`  
    Sets up the active generation algorithm for the current mesh. If a merging-based algorithm is used matrices are generated depending on the selected initialization method.
*   `void shrinkIndexedArray(MatrixArray* array, std::stack<Index> deleted_indices)`  
    Removes all entries corresponding to indices on the stack from an array. Used for cleanup.

###### ClassicQEM
*   `Matrix44 computeQEM(esIndex vertex_index)`  

###### ModifiedQEM
*   `Matrix44 computeModifiedVertexQEM(Index vertex_index)`  
*   `Matrix44 computeModifiedEdgeQEM(Index edge_index)`  
*   `void contractEdgeModifiedQEM(Index edge_index)`  
    Update all metric data according in reaction to a contraction.
*   `void collectRemovalData(...)`  
    Extract information about the neighbourhood of the contracted edge.

###### Merging Based Algorithms
*   `Matrix44 computeInitialMergeMetric(Index vertex_index)`  
    Set up the initial error metric for a vertex
*   `Matrix44 mergeMax(Matrix44 a, Matrix44 b)`
    Merges two matrices so the result returns values greater or equal than the maximum of the evaluation of the input matrices.
*   `Matrix44 mergeMatrices(Matrix44 a, Matrix44 b)`  
    Merges two metrices according to the selected algorithm.
*   `Matrix44 mergeMinConstant(Matrix44 a, Matrix44 b)`  
    Implementation of MinimizedConstant merging. Addition of the matrices, minimization of the constant term.
*   `Matrix44 mergeDiagonalization(Matrix44 a, Matrix44 b)`  
    Implementation of Diagonalization merging. Computes a diagonal bound for each matrix, merges these.
*   `Matrix44 mergeAverage(Matrix44 a, Matrix44 b)`  
    Merges the matrices by computing their average.
