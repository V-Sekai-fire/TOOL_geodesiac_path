#include "geometrycentral/surface/edge_length_geometry.h"
#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/mesh_graph_algorithms.h"
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

  std::vector<SurfacePoint> sourcePoints;
  
  SurfacePoint queryPoint2 = Vertex(mesh.get(), iVStart);

  sourcePoints.push_back(queryPoint2);

  mmp.propagate(sourcePoints);

  std::vector<SurfacePoint> path = mmp.traceBack(Vertex(mesh.get(), iVEnd));

  psMesh->removeAllQuantities();
  { // Marked vertices
    std::vector<Vector3> cloud;
    for (SurfacePoint p : path) {
      Vector3 p3d = p.interpolate(geometry->inputVertexPositions);
      cloud.push_back(p3d);
    }
    // Visualize balls at marked
    auto psCloud = polyscope::registerPointCloud("marked vertices", cloud);
    psCloud->setPointColor(polyscope::render::RGB_BLACK);
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
void buildFancyPathUI() {

  auto updateFancyPathViz = [&]() { psMesh->addVertexCountQuantity("fancy path vertices", fancyPathVertsPs); };

  if (ImGui::Button("Push Vertex")) {

    long long int iV = psMesh->selectVertex();
    if (iV != -1) {
      Vertex v = mesh->vertex(iV);
      fancyPathVerts.push_back(v);
      fancyPathVertsPs.emplace_back((size_t)iV, (int)fancyPathVertsPs.size());
      updateFancyPathViz();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Pop Vertex")) {
    if (!fancyPathVerts.empty()) {
      fancyPathVerts.pop_back();
      fancyPathVertsPs.pop_back();
    }
    updateFancyPathViz();
  }
  ImGui::Checkbox("Create Closed Path", &fancyPathClosed);
  ImGui::Checkbox("Mark interior vertices", &fancyPathMarkVerts);
}

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
  args::ArgumentParser parser("Flip edges to find geodesic paths.");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

  args::Group output(parser, "ouput");
  // args::Flag noGUI(output, "noGUI", "exit after processing and do not open the GUI", {"noGUI"});

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
  // withGUI = !noGUI;
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

  // Perform any operations requested via command line arguments

  // Give control to the gui
  if (withGUI) {
    // Give control to the polyscope gui
    polyscope::show();
  }

  return EXIT_SUCCESS;
}
