#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/polygon_soup_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/utilities/timing.h"
#include "geometrycentral/surface/exact_geodesics.h"

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<SurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// UI parameters
std::string loadedFilename = "";
bool withGUI = true;
bool iterativeShortenUseIterationCap = false;
int iterativeShortenIterationCap = 1;
bool straightenAtMarked = true;
bool useIterativeShortenLengthLim = false;
float iterativeShortenLengthLim = 0.5;

int nBezierIters = 3;

bool vizAllIntrinsicEdges = false;
float angleEPS = 1e-5;
float splitAngleDeg = 10;
float refineAreaThresh = std::numeric_limits<float>::infinity();
float refineAngleThresh = 25.;
int maxInsertions = -1;

// ====== Path related stuff

void createPathFromPoints() {
  long long int iVStart = psMesh->selectVertex();
  long long int iVEnd = psMesh->selectVertex();

  if (iVStart == -1 || iVEnd == -1) return;

  GeodesicAlgorithmExact mmp(*mesh, *geometry); 
  
  Vertex furthestVertex = Vertex(mesh.get(), iVEnd);
  {
    std::vector<SurfacePoint> sourcePoints;
    sourcePoints.push_back(Vertex(mesh.get(), iVStart));
    mmp.propagate(sourcePoints);
    VertexData<double> distToSource = mmp.getDistanceFunction();
    for (Vertex v : mesh->vertices()) {
      if (distToSource[v] < distToSource[iVEnd] * 2.0) {
        furthestVertex = v;
      } 
    }
  }
  std::vector<SurfacePoint> finalPath;
  {
    std::vector<SurfacePoint> sourcePoints;
    sourcePoints.push_back(Vertex(mesh.get(), iVStart));
    mmp.propagate(sourcePoints);
    std::vector<SurfacePoint> path = mmp.traceBack(furthestVertex);
    finalPath.insert(finalPath.end(), path.begin(), path.end());
  }
  {
    std::vector<SurfacePoint> sourcePoints;
    sourcePoints.push_back(furthestVertex);
    mmp.propagate(sourcePoints);
    std::vector<SurfacePoint> path = mmp.traceBack(Vertex(mesh.get(), iVEnd));
    finalPath.insert(finalPath.end(), path.begin(), path.end());
  }
  {
    std::vector<SurfacePoint> sourcePoints;
    sourcePoints.push_back(Vertex(mesh.get(), iVEnd));
    mmp.propagate(sourcePoints);
    std::vector<SurfacePoint> path = mmp.traceBack(Vertex(mesh.get(), iVStart));
    finalPath.insert(finalPath.end(), path.begin(), path.end());
  }

  psMesh->removeAllQuantities();
  { // Marked vertices
    std::vector<Vector3> cloud;
    for (SurfacePoint p : finalPath) {
      Vector3 p3d = p.interpolate(geometry->inputVertexPositions);
      cloud.push_back(p3d);
    }
    // Visualize balls at marked
    polyscope::PointCloud *psCloud = polyscope::registerPointCloud("marked vertices", cloud);
    psCloud->setPointColor(polyscope::render::RGB_DARKGRAY);
  }
}

// ====== General viz

void clearData() {
  psMesh->removeAllQuantities();
}

// Fancy path construction
bool fancyPathClosed = false;
std::vector<Vertex> fancyPathVerts;
std::vector<std::pair<size_t, int>> fancyPathVertsPs;
VertexData<double> fancyPathVertexNumbers;
bool fancyPathMarkVerts = false;

// A user-defined callback, for creating control panels (etc)
void myCallback() {
  if (ImGui::Button("Construct new shortest local path from endpoints")) {
    clearData();
    createPathFromPoints();
  }
  ImGui::PushItemWidth(150);
  ImGui::PopItemWidth();
}

int main(int argc, char** argv) {

  // Configure the argument parser
  args::ArgumentParser parser("Find geodesic paths.");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

  args::Group output(parser, "ouput");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help&) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  // Set options
  withGUI = true;

  // Initialize polyscope
  if (withGUI) {
    polyscope::init();
    polyscope::state::userCallback = myCallback;
  }

  // Load mesh
  loadedFilename = args::get(inputFilename);
  std::tie(mesh, geometry) = readSurfaceMesh(loadedFilename);

  if (withGUI) {
    // Register the mesh with polyscope
    psMesh = polyscope::registerSurfaceMesh("input mesh", geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));
  }

  // Give control to the gui
  if (withGUI) {
    // Give control to the polyscope gui
    polyscope::show();
  }

  return EXIT_SUCCESS;
}
