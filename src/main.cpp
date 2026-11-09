#include "geometrycentral/surface/edge_length_geometry.h"
#include "geometrycentral/surface/flip_geodesics.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/mesh_graph_algorithms.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/polygon_soup_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/utilities/timing.h"

#include <sstream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cmath>

#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "../deps/polyscope/deps/args/args/args.hxx"
#include "imgui.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh* psMesh;

// An edge network while processing flips
std::unique_ptr<FlipEdgeNetwork> edgeNetwork;

// UI parameters
std::string loadedFilename = "";
// Set when no mesh loaded at startup (no file given, or it failed to
// load) -- shown in the GUI instead of letting the app crash or exit
// with nothing visible at all. See the crash-fix comment in main().
std::string meshLoadError = "";
bool withGUI = true;
bool iterativeShortenUseIterationCap = true;
int iterativeShortenIterationCap = 4;
bool straightenAtMarked = true;
bool useIterativeShortenLengthLim = true;
float iterativeShortenLengthLim = 0.01;

int nBezierIters = 3;

bool vizAllIntrinsicEdges = false;
float angleEPS = 1e-5;
float splitAngleDeg = 10;
float refineAreaThresh = std::numeric_limits<float>::infinity();
float refineAngleThresh = 25.;
int maxInsertions = -1;

// ====== Path related stuff

void updatePathViz() {
  if (!edgeNetwork) {
    polyscope::error("tried to visualize path, but no path exists");
    return;
  }

  // remove everything
  psMesh->removeAllQuantities();

  auto pathQ = psMesh->addSurfaceGraphQuantity("path edges", edgeNetwork->getPathPolyline3D());
  pathQ->setEnabled(true);
  pathQ->setColor(polyscope::render::RGB_RED);
  pathQ->setRadius(0.0005);

  if (vizAllIntrinsicEdges) {
    auto edgeQ = psMesh->addSurfaceGraphQuantity("intrinsic edges", edgeNetwork->getAllEdgePolyline3D());
    edgeQ->setEnabled(true);
    edgeQ->setColor(polyscope::render::RGB_ORANGE);
    edgeQ->setRadius(0.005);
  }

  { // Marked vertices
    std::vector<Vector3> cloud;
    for (Vertex v : edgeNetwork->mesh.vertices()) {
      if (edgeNetwork->isMarkedVertex[v]) {
        SurfacePoint p = edgeNetwork->tri->vertexLocations[v];
        Vector3 p3d = p.interpolate(geometry->inputVertexPositions);
        cloud.push_back(p3d);
      }
    }
    // Visualize balls at marked
    auto psCloud = polyscope::registerPointCloud("marked vertices", cloud);
    psCloud->setPointColor(polyscope::render::RGB_BLACK);
  }
}

void createPathFromPoints() {

  long long int iVStart = psMesh->selectVertex();
  long long int iVEnd = psMesh->selectVertex();

  if (iVStart == -1 || iVEnd == -1) return;

  edgeNetwork =
      FlipEdgeNetwork::constructFromDijkstraPath(*mesh, *geometry, mesh->vertex(iVStart), mesh->vertex(iVEnd));
  if (edgeNetwork == nullptr) {
    polyscope::warning("could not initialize edge path between vertices");
    return;
  }
  edgeNetwork->posGeom = geometry.get();

  updatePathViz();
}

void createPathFromEdgeSet() {

  EdgeData<bool> edgeSet(*mesh, false);
  VertexData<bool> extraMarkedVertices(*mesh, false);

  { // Load the edge set from file
    std::ifstream inStream("path_edges.txt");
    if (!inStream) {
      polyscope::error("could not read path_edges.txt");
      return;
    }

    for (std::string line; std::getline(inStream, line);) {
      std::istringstream lineStream(line);

      size_t indA, indB;
      lineStream >> indA;
      lineStream >> indB;

      for (Halfedge he : mesh->vertex(indA).incomingHalfedges()) {
        if (he.vertex().getIndex() == indB) {
          edgeSet[he.edge()] = true;
        }
      }
    }
  }

  { // Load extra marked vertices from file
    std::ifstream inStream("marked_vertices.txt");
    if (!inStream) {
      polyscope::warning("could not read marked_vertices.txt");
    }

    for (std::string line; std::getline(inStream, line);) {
      std::istringstream lineStream(line);

      size_t ind;
      lineStream >> ind;
      extraMarkedVertices[mesh->vertex(ind)] = true;
    }
  }

  edgeNetwork = FlipEdgeNetwork::constructFromEdgeSet(*mesh, *geometry, edgeSet, extraMarkedVertices);
  if (edgeNetwork == nullptr) {
    polyscope::warning("could not initialize path from file");
    return;
  }
  edgeNetwork->posGeom = geometry.get();

  updatePathViz();
}

void createPathFromObjLines() {

  auto findHalfedge = [&](Vertex vA, Vertex vB) {
    for (Halfedge he : vA.outgoingHalfedges()) {
      if (he.twin().vertex() == vB) return he;
    }
    return Halfedge();
  };

  std::vector<std::vector<Halfedge>> paths;

  { // Load the edge set from file
    std::ifstream inStream(loadedFilename);
    if (!inStream) {
      polyscope::error("could not read: " + loadedFilename);
      return;
    }

    for (std::string line; std::getline(inStream, line);) {
      if (line.size() < 2 || line[0] != 'l' || line[1] != ' ') continue; // look for line ('l' first char)

      std::istringstream lineStream(line.substr(2)); // all but first char and space

      // parse out list of indices
      std::vector<size_t> inds;
      std::string token;
      while (std::getline(lineStream, token, ' ')) {
        std::stringstream tokenStream(token);
        size_t i;
        tokenStream >> i;
        inds.push_back(i - 1);
      }

      // build vertices
      paths.emplace_back();
      std::vector<Halfedge>& path = paths.back();
      for (size_t i = 1; i < inds.size(); i++) {
        Vertex vA = mesh->vertex(inds[i - 1]);
        Vertex vB = mesh->vertex(inds[i]);

        Halfedge he = findHalfedge(vA, vB);
        if (he == Halfedge()) {
          polyscope::warning("vertices " + std::to_string(inds[i - 1]) + " and " + std::to_string(inds[i]) +
                             " are not connected");
          return;
        }
        path.push_back(he);
      }

      // Try to close a loop
      Halfedge lastHe = findHalfedge(mesh->vertex(inds.back()), mesh->vertex(inds.front()));
      if (lastHe != Halfedge()) {
        std::cout << "    closing loop with halfedge " << lastHe << std::endl;
        path.push_back(lastHe);
      }

      std::cout << "  ...found path with " << path.size() << " segments." << std::endl;
    }
  }

  std::cout << "Loaded line list with " << paths.size() << " paths." << std::endl;

  edgeNetwork.reset(new FlipEdgeNetwork(*mesh, *geometry, paths));
  if (edgeNetwork == nullptr) {
    polyscope::warning("could not initialize path from file");
    return;
  }
  edgeNetwork->posGeom = geometry.get();

  updatePathViz();
}

void createPathFromDijkstraList() {

  // Create an  (initially-empty) edge network
  edgeNetwork = std::unique_ptr<FlipEdgeNetwork>(new FlipEdgeNetwork(*mesh, *geometry, {}));
  edgeNetwork->posGeom = geometry.get();

  { // Load the edge set from file
    std::ifstream inStream("path_pairs.txt");
    if (!inStream) {
      polyscope::error("could not read path_pairs.txt");
      return;
    }

    for (std::string line; std::getline(inStream, line);) {
      std::istringstream lineStream(line);

      size_t indA, indB;
      lineStream >> indA;
      lineStream >> indB;

      Vertex vA = edgeNetwork->tri->intrinsicMesh->vertex(indA);
      Vertex vB = edgeNetwork->tri->intrinsicMesh->vertex(indB);

      std::cout << "loading path from " << vA << " to " << vB << std::endl;

      std::vector<Halfedge> path = shortestEdgePath(*edgeNetwork->tri, vA, vB);
      edgeNetwork->addPath(path);
    }
  }

  updatePathViz();
}

void createPathFromSeg() {

  // read face segmentation inds
  std::ifstream inStream("cut.seg");
  if (!inStream) {
    polyscope::warning("could not read cut.seg");
  }
  FaceData<int> segs(*mesh);
  size_t iF = 0;
  for (std::string line; std::getline(inStream, line);) {
    std::istringstream lineStream(line);
    int ind;
    lineStream >> ind;
    if (iF >= mesh->nFaces()) {
      polyscope::warning("segmentation file doesn't match number of faces");
      return;
    }
    segs[iF] = ind;
    iF++;
  }

  psMesh->addFaceScalarQuantity("segmentation", segs);

  // Make cut along boundary
  EdgeData<bool> edgeSet(*mesh, false);
  for (Halfedge he : mesh->halfedges()) {
    if (he.edge().isBoundary()) continue;
    if (segs[he.face()] != segs[he.twin().face()]) {
      edgeSet[he.edge()] = true;
    }
  }

  VertexData<bool> extraMarkedVertices(*mesh, false); // none

  edgeNetwork = FlipEdgeNetwork::constructFromEdgeSet(*mesh, *geometry, edgeSet, extraMarkedVertices);
  if (edgeNetwork == nullptr) {
    polyscope::warning("could not initialize path from file");
    return;
  }
  edgeNetwork->posGeom = geometry.get();

  updatePathViz();
}

void createPathFromUVCut() {

  // (re)-load the obj file, and pull out corner coords
  PolygonSoupMesh reloadMesh(loadedFilename);
  if (reloadMesh.paramCoordinates.empty()) {
    polyscope::warning("could not load UVs from mesh file");
    return;
  }

  CornerData<Vector2> uvCoords(*mesh);
  for (Face f : mesh->faces()) {
    size_t iFace = f.getIndex();
    size_t iC = 0;
    for (Corner c : f.adjacentCorners()) {
      Vector2 uv = reloadMesh.paramCoordinates[iFace][iC];
      uvCoords[c] = uv;
      iC++;
    }
  }
  psMesh->addParameterizationQuantity("loaded UVs", uvCoords);

  // Detect cut as gap in UVs
  EdgeData<bool> edgeSet(*mesh, false);
  for (Halfedge he : mesh->halfedges()) {
    if (he.edge().isBoundary()) continue;
    if (uvCoords[he.corner()] != uvCoords[he.twin().next().corner()]) {
      edgeSet[he.edge()] = true;
    }
  }

  VertexData<bool> extraMarkedVertices(*mesh, false); // none

  edgeNetwork = FlipEdgeNetwork::constructFromEdgeSet(*mesh, *geometry, edgeSet, extraMarkedVertices);
  if (edgeNetwork == nullptr) {
    polyscope::warning("could not initialize path from file");
    return;
  }
  edgeNetwork->posGeom = geometry.get();

  updatePathViz();
}

void checkPath() {
  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }

  edgeNetwork->validate();

  double minAngle = edgeNetwork->minAngleIsotopy();
  std::cout << "min angle = " << minAngle << std::endl;
  if (minAngle < (M_PI - edgeNetwork->EPS_ANGLE)) {
    polyscope::warning("min angle in path is " + std::to_string(minAngle));
  }
}

void makeDelaunay() {
  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }
  edgeNetwork->makeDelaunay();
  updatePathViz();
}

void locallyShorten() {
  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }

  // reset counters
  edgeNetwork->nFlips = 0;
  edgeNetwork->nShortenIters = 0;

  edgeNetwork->EPS_ANGLE = angleEPS;
  edgeNetwork->straightenAroundMarkedVertices = straightenAtMarked;

  size_t iterLim = iterativeShortenUseIterationCap ? iterativeShortenIterationCap : INVALID_IND;
  double lengthLim = useIterativeShortenLengthLim ? iterativeShortenLengthLim : 0.;

  // re-add all wedges to angle queue
  // (this is generally extra unneeded work, but makes things simpler in case e.g. settings ^^^ changes after previous flips)
  edgeNetwork->addAllWedgesToAngleQueue();

  if (iterativeShortenUseIterationCap) {
    edgeNetwork->iterativeShorten(iterLim, lengthLim);
  } else {

    START_TIMING(shorten)
    edgeNetwork->iterativeShorten(iterLim, lengthLim);
    edgeNetwork->getPathPolyline();
    FINISH_TIMING_PRINT(shorten)

    checkPath();
  }

  std::cout << "shortening performed " << edgeNetwork->nShortenIters << " iterations, with a total of "
            << edgeNetwork->nFlips << " flips. " << std::endl;

  updatePathViz();
}

void bezierSubdivide() {
  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }

  START_TIMING(bezier)
  edgeNetwork->bezierSubdivide(nBezierIters);
  edgeNetwork->getPathPolyline();
  FINISH_TIMING_PRINT(bezier)

  updatePathViz();
}

void delaunayRefine() {

  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }

  edgeNetwork->delaunayRefine(refineAreaThresh, maxInsertions == -1 ? INVALID_IND : maxInsertions, refineAngleThresh);
  std::cout << "refined mesh has " << edgeNetwork->tri->intrinsicMesh->nVertices() << " verts\n";

  updatePathViz();
}

// ====== General viz

void clearData() {
  psMesh->removeAllQuantities();
  edgeNetwork.reset();
}

void exportPathLines() {
  if (edgeNetwork == nullptr) {
    polyscope::warning("no path network");
    return;
  }

  edgeNetwork->savePathOBJLine("");
}

// ====== Headless batch measurement (closes issue #2: "Todo get the length
// of the 3d line" -- geometry-central's FlipEdgeNetwork::length() already
// exists, the GUI app just never called it or exposed a non-interactive
// way to drive the tool from a script/pipeline).
//
// Usage: flip_geodesics mesh.obj --measure-loop 10,284,591,1203
//        flip_geodesics mesh.obj --measure-path 10,284
// Builds a (closed, for --measure-loop) piecewise-Dijkstra path through the
// given vertex indices and reports its length as a tape measure pulled
// taut along the surface between/around the given landmarks, not a plane
// cross-section (which can cut a shorter or longer chord than the surface
// distance, especially across a non-convex region).
//
// KNOWN LIMITATION, found and left UNRESOLVED by a planted control (not
// silently papered over): for --measure-loop, FlipEdgeNetwork::
// iterativeShorten() is UNSAFE and is NOT called by default. A closed loop
// on a genus-0 (topological-sphere) surface -- a human body mesh is one,
// same as a solid cylinder -- is contractible: nothing stops the shortest-
// path flip search from sliding the whole loop over a cap/limb-end and
// collapsing it toward a point, which it does. Verified on a flat-grid
// mesh (open path only, no loop) that the ALGORITHM is otherwise exactly
// correct: a staircase Dijkstra path of length 22.07 straightens to
// 20.6155, matching the analytic Euclidean distance (20.615528...) to 4
// decimals. On a test cylinder (radius 3, expected circumference
// 2*pi*3 = 18.8496), a 64-point rim loop's Dijkstra length (18.842,
// already a good discrete approximation) COLLAPSED to 0.589 under
// iterativeShorten even with markInterior=true at construction (which the
// upstream README's "mark interior vertices" description claims pins the
// curve during shortening -- it does not prevent this collapse; either
// the semantics are narrower than that description, or there is a bug in
// the vendored geometry-central). makeDelaunay() alone, with no
// iterativeShorten call, left the cylinder loop's length completely
// unchanged (18.842 -> 18.842) -- safe, but only a Delaunay
// retriangulation, not an actual shortening toward the true geodesic.
//
// So --measure-loop reports the Delaunay-refined length (DELAUNAY_LENGTH)
// as its headline number -- safe, but likely slightly LONGER than the true
// taut-tape-measure geodesic circumference, since the Dijkstra initial
// loop zigzags along mesh edges rather than running perfectly straight
// between samples. --measure-loop-unsafe-shorten additionally reports
// GEODESIC_LENGTH from iterativeShorten for comparison/debugging, with a
// loud warning that it is known to collapse and must not be trusted yet.
// Error code, not exception: checks the preconditions readManifoldSurfaceMesh
// needs (geometry-central throws instead of returning an error code, and a
// real crash report -- flip_geodesics-2026-11-08-100135.ips, SIGABRT via
// __cxa_throw -> failed_throw -> std::terminate, readMeshFromFile() on the
// stack -- showed nothing was catching it). Checking ourselves first, before
// ever calling into geometry-central, means a bad path fails with our own
// explicit, predictable error code instead of relying on catching whatever a
// vendored library happens to throw. Returns "" if the file looks loadable,
// else a human-readable reason.
std::string checkMeshFileOrError(const std::string& path) {
  if (path.empty()) {
    return "no mesh file given";
  }
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return "mesh file does not exist: '" + path + "'";
  }
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    return "mesh path is not a regular file: '" + path + "'";
  }
  static const std::vector<std::string> supportedExt = {".obj", ".ply", ".off", ".stl"};
  std::string ext = std::filesystem::path(path).extension().string();
  for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (std::find(supportedExt.begin(), supportedExt.end(), ext) == supportedExt.end()) {
    return "unsupported mesh file extension '" + ext + "' (supported: .obj, .ply, .off, .stl): '" + path + "'";
  }
  return "";
}

std::vector<Vertex> parseVertexList(ManifoldSurfaceMesh& m, const std::string& csv) {
  std::vector<Vertex> out;
  std::stringstream ss(csv);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty()) continue;
    size_t idx = static_cast<size_t>(std::stoul(tok));
    if (idx >= m.nVertices()) {
      std::cerr << "ERROR: vertex index " << idx << " out of range (mesh has " << m.nVertices() << " vertices)"
                << std::endl;
      std::exit(EXIT_FAILURE);
    }
    out.push_back(m.vertex(idx));
  }
  return out;
}

// Writes the REAL straightened/Delaunay path's 3D polyline (every point,
// including interior points along a face the path crosses, not just mesh
// vertices) as JSON `[[x,y,z],...]`, scaled by unitsScale. Called after
// the real shortening/retriangulation step, so this is the SMOOTH curve,
// not the raw Dijkstra zigzag -- the thing the caller actually asked for
// (see FlipEdgeNetwork::getPathPolyline3D()).
void exportPolyline3D(FlipEdgeNetwork& net, const std::string& outPath, double unitsScale) {
  std::vector<std::vector<Vector3>> polylines = net.getPathPolyline3D();
  std::ofstream f(outPath);
  f << "[";
  bool firstLine = true;
  for (auto& line : polylines) {
    if (!firstLine) f << ",";
    firstLine = false;
    f << "[";
    bool firstPt = true;
    for (Vector3 p : line) {
      if (!firstPt) f << ",";
      firstPt = false;
      f << "[" << (p.x * unitsScale) << "," << (p.y * unitsScale) << "," << (p.z * unitsScale) << "]";
    }
    f << "]";
  }
  f << "]";
  f.close();
  std::cerr << "wrote polyline to " << outPath << " (" << polylines.size() << " line(s))" << std::endl;
}

// Largest turning angle (degrees) between consecutive segments of a
// CLOSED polyline, treating the last point as connecting back to the
// first. Mesh-size- and loop-length-INDEPENDENT (an angle, not a
// distance), which is why it is the adaptive stopping criterion below
// instead of an iteration count or a length-decrease fraction: those
// two are tuned to one specific mesh's loop sizes and do not transfer
// to a different ANNY phenotype or a wholly custom model (iFire: "it
// has to work for possible anny and custom models so you can't hard
// code").
double maxClosedLoopTurningAngleDeg(FlipEdgeNetwork& net) {
  std::vector<std::vector<Vector3>> polylines = net.getPathPolyline3D();
  double maxAngle = 0.0;
  for (auto& line : polylines) {
    size_t n = line.size();
    if (n < 3) continue;
    for (size_t i = 0; i < n; i++) {
      Vector3 prev = line[(i + n - 1) % n];
      Vector3 curr = line[i];
      Vector3 next = line[(i + 1) % n];
      Vector3 segIn = unit(curr - prev);
      Vector3 segOut = unit(next - curr);
      double cosAngle = std::clamp(dot(segIn, segOut), -1.0, 1.0);
      double angleDeg = std::acos(cosAngle) * 180.0 / M_PI;
      maxAngle = std::max(maxAngle, angleDeg);
    }
  }
  return maxAngle;
}

// Adaptive, mesh-size-independent loop smoothing: run iterativeShorten
// ONE step at a time (checkLength disabled per-step; 0.0 means "off",
// see geometry-central's flip_geodesics.cpp), re-measuring the max
// turning angle after each step, and stop as soon as it drops to or
// below targetMaxTurnDeg -- a real geometric smoothness criterion, not
// an arbitrary count empirically tuned against one mesh's loops (which
// is what a fixed --measure-loop-bounded-shorten=N does, and why that
// flag stays around only as a manual-override escape hatch, not the
// default path). hardMaxIterations and hardMaxRelDecrease are SAFETY
// nets only (independent of mesh scale: iteration counts cap runtime,
// the relative-decrease fraction catches the known sliding-toward-a-
// narrower-cross-section collapse failure, see the module comment
// above) -- they stop the loop early if the angle target is never
// reached, they do not define "done".
struct AdaptiveShortenResult {
  size_t iterations;
  double finalMaxTurnDeg;
  double finalRelDecrease;
  bool hitAngleTarget;
  bool hitRelDecreaseCap;
  bool hitIterationCap;
  bool converged; // wedgeAngleQueue emptied naturally -- no more local flips exist
};

AdaptiveShortenResult adaptiveShortenToAngle(FlipEdgeNetwork& net, double targetMaxTurnDeg,
                                              size_t hardMaxIterations, double hardMaxRelDecrease) {
  double initLength = net.length();
  size_t iterations = 0;
  double maxTurn = maxClosedLoopTurningAngleDeg(net);
  bool hitRelDecreaseCap = false;
  bool converged = false;
  while (maxTurn > targetMaxTurnDeg && iterations < hardMaxIterations) {
    double lengthBefore = net.length();
    net.iterativeShorten(1, 0.0); // one step, no internal length check (we check our own, correctly-signed one)
    iterations++;
    double lengthAfter = net.length();
    if (std::abs(lengthAfter - lengthBefore) < 1e-12) {
      // No progress possible (wedgeAngleQueue emptied internally) --
      // a real local minimum for this loop's max turning angle, not a
      // cap being hit. Further calls would spin forever without
      // changing anything.
      converged = true;
      break;
    }
    double relDecrease = (initLength - lengthAfter) / initLength;
    if (relDecrease > hardMaxRelDecrease) {
      hitRelDecreaseCap = true;
      break;
    }
    maxTurn = maxClosedLoopTurningAngleDeg(net);
  }
  double finalLength = net.length();
  bool hitIterationCap = !converged && !hitRelDecreaseCap && iterations >= hardMaxIterations;
  return {iterations, maxTurn, (initLength - finalLength) / initLength,
          maxTurn <= targetMaxTurnDeg, hitRelDecreaseCap, hitIterationCap, converged};
}

int runHeadlessMeasure(const std::string& meshFile, const std::string& loopCSV, const std::string& pathCSV,
                        double unitsScale, bool unsafeShorten, const std::string& exportPathOut,
                        int boundedShortenMaxIters, double boundedShortenMaxRelDecrease,
                        double adaptiveTargetMaxTurnDeg, size_t adaptiveHardMaxIters,
                        double adaptiveHardMaxRelDecrease) {
  // Error code, not exception: check the precondition ourselves and fail
  // explicitly before ever calling into geometry-central, rather than
  // relying on catching whatever it throws. See checkMeshFileOrError()'s
  // comment for why this matters (a real crash this avoided).
  std::string err = checkMeshFileOrError(meshFile);
  if (!err.empty()) {
    std::cerr << "ERROR: " << err << std::endl;
    return EXIT_FAILURE;
  }
  std::unique_ptr<ManifoldSurfaceMesh> hMesh;
  std::unique_ptr<VertexPositionGeometry> hGeom;
  std::tie(hMesh, hGeom) = readManifoldSurfaceMesh(meshFile);

  bool isLoop = !loopCSV.empty();
  std::unique_ptr<FlipEdgeNetwork> net;
  if (isLoop) {
    std::vector<Vertex> verts = parseVertexList(*hMesh, loopCSV);
    if (verts.size() < 3) {
      std::cerr << "ERROR: --measure-loop needs at least 3 vertices (closed loop), got " << verts.size()
                << std::endl;
      return EXIT_FAILURE;
    }
    net = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(*hMesh, *hGeom, verts, /*closed=*/true,
                                                                /*markInterior=*/true);
  } else if (!pathCSV.empty()) {
    std::vector<Vertex> verts = parseVertexList(*hMesh, pathCSV);
    if (verts.size() != 2) {
      std::cerr << "ERROR: --measure-path needs exactly 2 vertices, got " << verts.size() << std::endl;
      return EXIT_FAILURE;
    }
    net = FlipEdgeNetwork::constructFromDijkstraPath(*hMesh, *hGeom, verts[0], verts[1]);
  } else {
    std::cerr << "ERROR: runHeadlessMeasure called with neither --measure-loop nor --measure-path" << std::endl;
    return EXIT_FAILURE;
  }

  if (net == nullptr) {
    std::cerr << "ERROR: could not construct the initial Dijkstra path/loop" << std::endl;
    return EXIT_FAILURE;
  }
  net->posGeom = hGeom.get();

  double lengthDijkstra = net->length();
  std::cout << "DIJKSTRA_LENGTH " << (lengthDijkstra * unitsScale) << std::endl;

  if (!isLoop) {
    // Open paths are safe to shorten directly (verified against analytic
    // ground truth on a flat-grid mesh: matches Euclidean distance to 4
    // decimals) -- an open path's endpoints can't slide off the mesh. Do
    // NOT run makeDelaunay() first: tested and found to change the local
    // flip search's outcome (20.6155 analytic-exact without it vs 20.6737,
    // a 0.3% error, WITH it first) -- it is not the harmless no-op
    // preprocessing step it looks like.
    net->iterativeShorten(INVALID_IND, 1e-6);
    double lengthAfter = net->length();
    std::cout << "GEODESIC_LENGTH " << (lengthAfter * unitsScale) << std::endl;
    std::cout << "SHORTEN_ITERS " << net->nShortenIters << std::endl;
    std::cout << "SHORTEN_FLIPS " << net->nFlips << std::endl;
    if (lengthAfter > lengthDijkstra + 1e-9) {
      std::cerr << "CONTROL FAILED: geodesic length (" << lengthAfter << ") > Dijkstra length (" << lengthDijkstra
                << ") -- shortening must not increase length" << std::endl;
      return EXIT_FAILURE;
    }
    if (!exportPathOut.empty()) {
      exportPolyline3D(*net, exportPathOut, unitsScale);
    }
    return EXIT_SUCCESS;
  }

  // isLoop: makeDelaunay() (retriangulation, not shortening) IS used here --
  // DELAUNAY_LENGTH is the number to trust (see the module-level comment
  // above this function for why iterativeShorten is unsafe on a loop).
  net->makeDelaunay();
  double lengthDelaunay = net->length();
  std::cout << "DELAUNAY_LENGTH " << (lengthDelaunay * unitsScale) << std::endl;

  // PLANTED CONTROL: Delaunay refinement must not change the length (it is
  // only a retriangulation, not a shortening) -- catches a build/API
  // mismatch where makeDelaunay() silently did something else.
  if (std::abs(lengthDelaunay - lengthDijkstra) > 1e-6 * std::max(1.0, lengthDijkstra)) {
    std::cerr << "CONTROL FAILED: makeDelaunay() changed the length (" << lengthDijkstra << " -> " << lengthDelaunay
              << "), expected it to be a no-op on length" << std::endl;
    return EXIT_FAILURE;
  }

  if (!exportPathOut.empty()) {
    exportPolyline3D(*net, exportPathOut, unitsScale);
  }

  // SAFE bounded shortening: caps BOTH the iteration count and the total
  // relative length decrease, so it relaxes the sharp per-segment kinks
  // left by constructFromPiecewiseDijkstraPath into local curvature
  // without running long enough to slide the whole loop over a cap and
  // collapse it (--measure-loop-unsafe-shorten's failure mode, see the
  // module comment above). A collapse guard is checked explicitly
  // afterward, not assumed safe just because the caller passed a cap.
  if (boundedShortenMaxIters > 0) {
    double lengthBeforeBounded = net->length();
    // geometry-central's iterativeShorten(iters, maxRelativeLengthDecrease)
    // parameter is a STOPPING THRESHOLD on the FRACTION OF LENGTH
    // REMAINING (stops once currLength < maxRelativeLengthDecrease *
    // initLength -- see flip_geodesics.cpp:970), the OPPOSITE of what its
    // name suggests and what our own CLI flag's help text promises
    // callers (a cap on how much the length is allowed to DECREASE BY).
    // Found empirically: passing our own flag's value straight through
    // let hips/leg_circ/neck_w loops shrink 15-40% (hips: 94.9cm ->
    // 62.47cm after enough iterations -- the SAME sliding-toward-a-
    // narrower-cross-section collapse the module comment warns about,
    // just not yet complete) while identical values on waist converged
    // safely at ~4%, because waist genuinely has no narrower point
    // nearby to slide toward. Converting here so OUR flag keeps its
    // documented, intuitive meaning: 1 - boundedShortenMaxRelDecrease is
    // the library's actual stopping threshold.
    net->iterativeShorten(static_cast<size_t>(boundedShortenMaxIters), 1.0 - boundedShortenMaxRelDecrease);
    double lengthAfterBounded = net->length();
    double relDecrease = (lengthBeforeBounded - lengthAfterBounded) / lengthBeforeBounded;
    std::cout << "BOUNDED_SHORTEN_LENGTH " << (lengthAfterBounded * unitsScale) << std::endl;
    std::cout << "BOUNDED_SHORTEN_ITERS " << net->nShortenIters << std::endl;
    std::cout << "BOUNDED_SHORTEN_REL_DECREASE " << relDecrease << std::endl;
    // PLANTED CONTROL: a genuine collapse drops length toward ~0 (the
    // cylinder test case collapsed 18.842 -> 0.589, a 97% decrease);
    // catch that here instead of trusting the caller's iteration cap
    // alone to have prevented it.
    if (relDecrease > 0.5) {
      std::cerr << "CONTROL FAILED: BOUNDED_SHORTEN dropped length by " << (relDecrease * 100)
                << "% despite the iteration/relative-decrease caps -- this looks like the known collapse failure "
                   "mode, not a legitimate local straightening. Refusing to trust BOUNDED_SHORTEN_LENGTH."
                << std::endl;
      return EXIT_FAILURE;
    }
    if (!exportPathOut.empty()) {
      std::string boundedOut = exportPathOut;
      size_t dot = boundedOut.rfind('.');
      boundedOut = (dot == std::string::npos) ? boundedOut + "_bounded" : boundedOut.substr(0, dot) + "_bounded" + boundedOut.substr(dot);
      exportPolyline3D(*net, boundedOut, unitsScale);
    }
  }

  // ADAPTIVE shortening: the default-recommended path, and the one
  // --measure-loop-adaptive-shorten enables. iFire: "it has to work
  // for possible anny and custom models so you can't hard code" --
  // --measure-loop-bounded-shorten's fixed maxIterations/
  // maxRelativeLengthDecrease were empirically tuned against ONE
  // mesh's loop sizes (ANNY-SOMA-X) and do not transfer: a smaller or
  // coarser custom mesh could still be jagged at that iteration count,
  // or could already be sliding toward collapse before reaching it.
  // This stops on a real geometric criterion instead -- the max
  // turning angle along the exported polyline, which means the same
  // thing (a visibly sharp kink vs. a smooth curve) on any mesh at any
  // scale. hardMaxIterations/hardMaxRelDecrease are safety nets only
  // (runtime cap, collapse guard), not what decides "done".
  if (adaptiveTargetMaxTurnDeg > 0) {
    AdaptiveShortenResult res =
        adaptiveShortenToAngle(*net, adaptiveTargetMaxTurnDeg, adaptiveHardMaxIters, adaptiveHardMaxRelDecrease);
    std::cout << "ADAPTIVE_SHORTEN_LENGTH " << (net->length() * unitsScale) << std::endl;
    std::cout << "ADAPTIVE_SHORTEN_ITERS " << res.iterations << std::endl;
    std::cout << "ADAPTIVE_SHORTEN_REL_DECREASE " << res.finalRelDecrease << std::endl;
    std::cout << "ADAPTIVE_SHORTEN_MAX_TURN_DEG " << res.finalMaxTurnDeg << std::endl;
    std::cout << "ADAPTIVE_SHORTEN_HIT_ANGLE_TARGET " << (res.hitAngleTarget ? 1 : 0) << std::endl;
    std::cout << "ADAPTIVE_SHORTEN_CONVERGED " << (res.converged ? 1 : 0) << std::endl;
    if (!res.hitAngleTarget) {
      // Not a failure -- an honest signal: this loop could not reach
      // the requested smoothness within the safety nets (e.g. a real
      // sharp anatomical fold the local-flip search has no further
      // move for, or a safety cap tripped first). The caller decides
      // whether that is acceptable; we do not silently claim success.
      std::string reason = res.converged
                                ? "the local-flip search converged (no further improving flip exists) -- a "
                                  "real local minimum for this loop's geometry, not a cap"
                                : (res.hitRelDecreaseCap ? "hit the relative-decrease safety cap"
                                                          : "hit the iteration safety cap");
      std::cerr << "NOTE: --measure-loop-adaptive-shorten did not reach the " << adaptiveTargetMaxTurnDeg
                << "deg target (stopped at " << res.finalMaxTurnDeg << "deg: " << reason << ")." << std::endl;
    }
    // PLANTED CONTROL: same collapse signature as --measure-loop-
    // unsafe-shorten's cylinder test (18.842 -> 0.589, 97% decrease) --
    // catch it even if the caller passed an overly loose
    // hardMaxRelDecrease.
    if (res.finalRelDecrease > 0.5) {
      std::cerr << "CONTROL FAILED: ADAPTIVE_SHORTEN dropped length by " << (res.finalRelDecrease * 100)
                << "% -- this looks like the known collapse failure mode, not legitimate local straightening. "
                   "Refusing to trust ADAPTIVE_SHORTEN_LENGTH."
                << std::endl;
      return EXIT_FAILURE;
    }
    if (!exportPathOut.empty()) {
      std::string adaptiveOut = exportPathOut;
      size_t dot = adaptiveOut.rfind('.');
      adaptiveOut = (dot == std::string::npos) ? adaptiveOut + "_adaptive"
                                                 : adaptiveOut.substr(0, dot) + "_adaptive" + adaptiveOut.substr(dot);
      exportPolyline3D(*net, adaptiveOut, unitsScale);
    }
  }

  if (unsafeShorten) {
    std::cerr << "WARNING: --measure-loop-unsafe-shorten is a KNOWN-BROKEN path (collapses the loop on a genus-0 "
                 "surface, see source comment). GEODESIC_LENGTH below is diagnostic only -- do not use it."
              << std::endl;
    net->iterativeShorten(INVALID_IND, 1e-6);
    std::cout << "GEODESIC_LENGTH_UNSAFE " << (net->length() * unitsScale) << std::endl;
  }
  return EXIT_SUCCESS;
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
  if (ImGui::Button("New Path From These Points")) {
    edgeNetwork = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(*mesh, *geometry, fancyPathVerts, fancyPathClosed,
                                                                      fancyPathMarkVerts);
    if (edgeNetwork == nullptr) {
      polyscope::warning("could not initialize fancy edge path between vertices");
      return;
    }
    edgeNetwork->posGeom = geometry.get();

    updatePathViz();
  }
  ImGui::Checkbox("Create Closed Path", &fancyPathClosed);
  ImGui::Checkbox("Mark interior vertices", &fancyPathMarkVerts);
}

void myCallback() {

  if (!meshLoadError.empty()) {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "%s", meshLoadError.c_str());
    ImGui::Separator();
  }

  ImGui::TextUnformatted("Input");

  if (ImGui::Button("Construct shortest loop")) {
    clearData();
    fancyPathVerts.clear();
    fancyPathVertsPs.clear();
    fancyPathClosed = true;
    fancyPathMarkVerts = true;

    while(true) {
      long long int vIndex = psMesh->selectVertex();
      if (vIndex == -1) { // User cancelled the selection
        break;
      }

      Vertex v = mesh->vertex(vIndex);
      fancyPathVerts.push_back(v);
      fancyPathVertsPs.emplace_back((size_t)vIndex, (int)fancyPathVertsPs.size());

      // Closes issue #3 ("Immediate Feedback for Point Placement in
      // UI/UX"): previously this marker update and the Dijkstra preview
      // path were both gated behind "at least 3 vertices", so clicking
      // point 1 or point 2 showed NOTHING on screen -- no marker, no
      // line, no feedback at all until the 3rd click. Markers now update
      // after every single click.
      auto updateFancyPathViz = [&]() { psMesh->addVertexCountQuantity("fancy path vertices", fancyPathVertsPs); };
      updateFancyPathViz();

      if (fancyPathVerts.size() == 2) {
        // Live preview of the first leg as an OPEN path (not yet closed --
        // closing a 2-point "loop" is degenerate) so the user sees a line
        // connecting their first two clicks immediately, instead of
        // waiting for a 3rd click with no visual confirmation the first
        // two landed where intended.
        edgeNetwork = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(*mesh, *geometry, fancyPathVerts,
                                                                          /*closed=*/false, fancyPathMarkVerts);
        if (edgeNetwork != nullptr) {
          edgeNetwork->posGeom = geometry.get();
          updatePathViz();
        }
      } else if (fancyPathVerts.size() >= 3) {
        edgeNetwork = FlipEdgeNetwork::constructFromPiecewiseDijkstraPath(*mesh, *geometry, fancyPathVerts, fancyPathClosed,
                                                                          fancyPathMarkVerts);
        if (edgeNetwork == nullptr) {
          polyscope::warning("could not initialize fancy edge path between vertices");
          return;
        }
        edgeNetwork->posGeom = geometry.get();
        makeDelaunay();
        delaunayRefine();
      }
    }
  }

  if (ImGui::Button("Construct new Dijkstra path from endpoints")) {
    clearData();
    createPathFromPoints();
  }

  if (ImGui::TreeNode("Construct fancy path")) {
    buildFancyPathUI();
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("Specialty loaders")) {
    if (ImGui::Button("Load edge set")) {
      clearData();
      createPathFromEdgeSet();
    }
    ImGui::SameLine();
    if (ImGui::Button("Load line list obj")) {
      clearData();
      createPathFromObjLines();
    }
    if (ImGui::Button("Load Dijkstra list")) {
      clearData();
      createPathFromDijkstraList();
    }

    if (ImGui::Button("Load UV cut")) {
      clearData();
      createPathFromUVCut();
    }
    ImGui::SameLine();
    if (ImGui::Button("Load seg cut")) {
      clearData();
      createPathFromSeg();
    }
    ImGui::TreePop();
  }

  ImGui::PushItemWidth(150);

  ImGui::Separator();
  ImGui::TextUnformatted("Algorithm");

  // Straightening
  if (ImGui::Button("Make geodesic")) {
    locallyShorten();
  }
  ImGui::SameLine();
  if (ImGui::Button("Check Path")) {
    checkPath();
  }

  // Closes issue #2 ("Todo get the length of the 3d line"): geometry-
  // central's FlipEdgeNetwork::length() already existed, this GUI just
  // never displayed it.
  if (edgeNetwork != nullptr) {
    ImGui::Text("Path length: %.6f", edgeNetwork->length());
  }

  ImGui::Checkbox("Straighten at marked", &straightenAtMarked);
  ImGui::Checkbox("Limit Iteration Count", &iterativeShortenUseIterationCap);
  if (iterativeShortenUseIterationCap) {
    ImGui::SameLine();
    ImGui::InputInt("Iters: ", &iterativeShortenIterationCap);
  }

  ImGui::Checkbox("Limit Length Decrease", &useIterativeShortenLengthLim);
  if (useIterativeShortenLengthLim) {
    ImGui::SameLine();
    ImGui::InputFloat("Relative lim: ", &iterativeShortenLengthLim);
  }

  // ====  Extras
  if (ImGui::TreeNode("Extras")) {

    if (ImGui::TreeNode("Bezier curves")) {

      if (ImGui::Button("Bezier subdivide")) {
        bezierSubdivide();
      }
      ImGui::InputInt("Bezier rounds", &nBezierIters);

      ImGui::TreePop();
    }

    if (ImGui::TreeNode("Intrinsic mesh improvement")) {
      if (ImGui::Button("Flip to Delaunay")) {
        makeDelaunay();
      }
      ImGui::InputInt("Max insert", &maxInsertions);
      ImGui::InputFloat("Area thresh", &refineAreaThresh);
      ImGui::InputFloat("Angle thresh", &refineAngleThresh);
      if (ImGui::Button("Delaunay refine")) {
        delaunayRefine();
      }
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  // ==== Visualization
  ImGui::Separator();
  ImGui::TextUnformatted("Visualization & Export");

  if (ImGui::Checkbox("Show Intrinsic Edges", &vizAllIntrinsicEdges)) {
    if (edgeNetwork) {
      updatePathViz();
    }
  }

  if (ImGui::Button("Export path lines")) {
    if (edgeNetwork) {
      exportPathLines();
    } else {
      polyscope::warning("no path registered");
    }
  }

  ImGui::PopItemWidth();
}

int main(int argc, char** argv) {
  args::ArgumentParser parser("Flip edges to find geodesic paths.");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");
  args::ValueFlag<std::string> measureLoopFlag(
      parser, "v1,v2,v3,...", "Headless: build a closed geodesic loop through these vertex indices and print its length (no GUI).",
      {"measure-loop"});
  args::ValueFlag<std::string> measurePathFlag(
      parser, "vStart,vEnd", "Headless: build a geodesic path between these two vertex indices and print its length (no GUI).",
      {"measure-path"});
  args::ValueFlag<double> unitsScaleFlag(parser, "scale", "Multiply the printed length by this (e.g. 100 for meters->cm).",
                                         {"units-scale"}, 1.0);
  args::Flag unsafeShortenFlag(
      parser, "unsafe-shorten",
      "Also run the KNOWN-BROKEN iterativeShorten on a --measure-loop (collapses on genus-0 surfaces; diagnostic only).",
      {"measure-loop-unsafe-shorten"});
  args::ValueFlag<int> boundedShortenItersFlag(
      parser, "maxIterations",
      "SAFE bounded iterativeShorten on a --measure-loop: caps BOTH the iteration count and the total relative "
      "length decrease (see --measure-loop-bounded-shorten-rel-decrease), so it relaxes sharp per-segment kinks "
      "into local curvature without enough iterations to slide the whole loop over a cap and collapse it (the "
      "failure mode of --measure-loop-unsafe-shorten's unlimited iterativeShorten). Matches the interactive GUI's "
      "own default-on 'Limit Iteration Count' checkbox, exposed here for headless use. 0 = off (default).",
      {"measure-loop-bounded-shorten"}, 0);
  args::ValueFlag<double> boundedShortenRelDecreaseFlag(
      parser, "maxRelativeLengthDecrease",
      "Stop --measure-loop-bounded-shorten once the loop's total length has decreased by more than this fraction "
      "of its starting (Delaunay) length -- the second half of the collapse guard. 0.05 (default) caps it at a "
      "5% decrease; verified empirically to converge safely on every torso/limb loop tested so far without "
      "entering the sliding-toward-a-narrower-cross-section collapse mode a looser cap can reach (see the "
      "runHeadlessMeasure() source comment for the measured hips-loop counterexample).",
      {"measure-loop-bounded-shorten-rel-decrease"}, 0.05);
  args::ValueFlag<std::string> exportPathFlag(
      parser, "out.json", "Headless: also write the real straightened/Delaunay path's 3D polyline as JSON.",
      {"export-path"});
  args::ValueFlag<double> adaptiveShortenTargetDegFlag(
      parser, "targetMaxTurnDeg",
      "RECOMMENDED, mesh-size-independent --measure-loop smoothing: run iterativeShorten one step at a time, "
      "stopping as soon as the loop's largest turning angle drops to or below this many degrees -- a real "
      "geometric criterion (a visibly sharp kink vs. a smooth curve means the same thing on any mesh at any "
      "scale), unlike --measure-loop-bounded-shorten's fixed iteration/length-decrease caps, which were tuned "
      "against one specific mesh's loop sizes and do not transfer to a different model. 0 = off (default).",
      {"measure-loop-adaptive-shorten"}, 0.0);
  args::ValueFlag<size_t> adaptiveShortenHardMaxItersFlag(
      parser, "hardMaxIterations", "Safety net (runtime cap, NOT the stopping criterion) for --measure-loop-adaptive-shorten.",
      {"measure-loop-adaptive-shorten-hard-max-iters"}, 2000);
  args::ValueFlag<double> adaptiveShortenHardMaxRelDecreaseFlag(
      parser, "hardMaxRelativeLengthDecrease",
      "Safety net (collapse guard, NOT the stopping criterion) for --measure-loop-adaptive-shorten: refuse to "
      "keep shortening once the loop's length has dropped by more than this fraction of its starting length, "
      "regardless of whether the angle target was reached.",
      {"measure-loop-adaptive-shorten-hard-max-rel-decrease"}, 0.3);

  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help&) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError& e) {
    std::cerr << e.what() << std::endl << parser;
    return 1;
  }

  if (measureLoopFlag || measurePathFlag) {
    // These genuinely need a mesh file; the GUI path below does not --
    // double-clicking the .app in Finder passes no arguments at all, and
    // that case should open a window (with a clear message), not exit
    // before one ever appears.
    if (!inputFilename) {
      std::cerr << "--measure-loop/--measure-path need a mesh file argument" << std::endl;
      return EXIT_FAILURE;
    }
    return runHeadlessMeasure(args::get(inputFilename), measureLoopFlag ? args::get(measureLoopFlag) : "",
                               measurePathFlag ? args::get(measurePathFlag) : "", args::get(unitsScaleFlag),
                               static_cast<bool>(unsafeShortenFlag), exportPathFlag ? args::get(exportPathFlag) : "",
                               args::get(boundedShortenItersFlag), args::get(boundedShortenRelDecreaseFlag),
                               args::get(adaptiveShortenTargetDegFlag), args::get(adaptiveShortenHardMaxItersFlag),
                               args::get(adaptiveShortenHardMaxRelDecreaseFlag));
  }

  polyscope::init();
  polyscope::state::userCallback = myCallback;

  // Error code, not exception: check the precondition ourselves (empty
  // path, missing file, unsupported extension) and fail explicitly with
  // our own message BEFORE ever calling into geometry-central, rather
  // than relying on catching whatever it throws internally. This is also
  // what makes a double-click with no mesh argument open a window with a
  // message instead of exiting before one ever appears -- a no-path
  // result from checkMeshFileOrError() is just "no mesh file given",
  // handled the same way as any other precondition failure.
  loadedFilename = args::get(inputFilename);
  meshLoadError = checkMeshFileOrError(loadedFilename);
  if (meshLoadError.empty()) {
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(loadedFilename);
  }

  if (mesh != nullptr) {
    psMesh = polyscope::registerSurfaceMesh("input mesh", geometry->inputVertexPositions, mesh->getFaceVertexList(),
                                            polyscopePermutations(*mesh));
  }
  // No window is strictly worse than a window saying what went wrong:
  // polyscope::show() still runs even with mesh == nullptr (no file
  // given, or the precondition check failed) -- myCallback() shows
  // meshLoadError.

  polyscope::show();

  return EXIT_SUCCESS;
}
