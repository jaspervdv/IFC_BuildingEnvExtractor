#define USE_IFC4

#ifdef USE_IFC2x3
#define IfcSchema Ifc2x3
#define buildVersion "IFC2X3"
#define SCHEMA_VERSIONS (2x3)
#define SCHEMA_SEQ (2x3)

#elif defined(USE_IFC4)
#define IfcSchema Ifc4
#define buildVersion "IFC4"
#define SCHEMA_VERSIONS (4)
#define SCHEMA_SEQ (4)

#elif defined(USE_IFC4x3)
#define IfcSchema Ifc4x3
#define buildVersion "IFC4X3"
#define SCHEMA_VERSIONS (4x3)
#define SCHEMA_SEQ (4x3)

#else
#error "No IFC version defined"
#endif // USE_IFC

#include "surfaceCollection.h"

// IfcOpenShell includes
#include <ifcparse/IfcFile.h>
#include <ifcgeom_schema_agnostic/Kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>
#include <ifcparse/IfcHierarchyHelper.h>

// OpenCascade includes
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>
#include <TopoDS.hxx>

#include <CJT.h>
#include <CJToKernel.h>

#include <chrono>
#include <unordered_set>
#include <memory>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> BoostPoint3D;
typedef std::pair<bg::model::box<BoostPoint3D>, int> Value;

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

// helper functions that can be utilised everywhere
struct helperFunctions{
	
	/// point managing functions
	
	/// Convert OpenCascade point to Boost point
	static BoostPoint3D Point3DOTB(const gp_Pnt& oP);
	/// Conver Boost point to OpenCascade point
	static gp_Pnt Point3DBTO(const BoostPoint3D& oP);
	/// get a list of unique points from a pointlist
	static std::vector<gp_Pnt> getUniquePoints(const std::vector<gp_Pnt>& pointList);
	/// get a list of unique points from a shape
	static std::vector<gp_Pnt> getUniquePoints(const TopoDS_Shape& inputShape);
	/// get a list of points from a shape
	static std::vector<gp_Pnt> getPoints(const TopoDS_Shape& shape);
	/// get the total count of unique points 
	static int getPointCount(const TopoDS_Shape& inputShape);
	///	Rotate OpenCascade point around 0,0,0
	static gp_Pnt rotatePointWorld(const gp_Pnt& p, double angle);
	///	Rotate Boost point around 0,0,0
	static BoostPoint3D rotatePointWorld(const BoostPoint3D& p, double angle);
	/// Rotate OpenCascade point p around point anchorP
	static gp_Pnt rotatePointPoint(const gp_Pnt& p, const gp_Pnt& anchorP, const double& angle);
	// get a grid of points that are placed over a surface
	static std::vector<gp_Pnt> getPointGridOnSurface(const TopoDS_Face& theface);
	// get a grid of points that are placed alongside the wire on a face
	static std::vector<gp_Pnt> getPointGridOnWire(const TopoDS_Face& theface);

	/// bounding box creating code

	/// get the lllpoint and urr point of a list of TopoDS shape
	/// the rotation values will rotate the shape before creating the points of the box, afterwards the box needs to be rotated back to represent that actual bounding box
	template<typename T>
	static void bBoxDiagonal(const std::vector<T>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer = 0, const double angle = 0, const double secondAngle = 0);
	/// get the lllpoint and urr point of an TopoDS shape
	/// the rotation values will rotate the shape before creating the points of the box, afterwards the box needs to be rotated back to represent that actual bounding box
	template<typename T>
	static void bBoxDiagonal(const T& theShape, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer = 0, const double angle = 0, const double secondAngle = 0);
	/// get the lllpoint and urr point of list of points
	/// the rotation values will rotate the shape before creating the points of the box, afterwards the box needs to be rotated back to represent that actual bounding box
	static bool bBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer = 0, const double angle = 0, const double secondAngle = 0);
	/// construct the smallest orientated bounding box
	static void bBoxOrientated(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, double* rotationAngle, const double buffer = 0);
	/// construct a bbox from a shape 
	static bg::model::box <BoostPoint3D> createBBox(const TopoDS_Shape& shape, double buffer = 0.05);
	/// construct a bbox from a list of shapes
	static bg::model::box <BoostPoint3D> createBBox(const std::vector<TopoDS_Shape>& shape, double buffer = 0.05);
	/// construct a bbox from a list of points
	static bg::model::box <BoostPoint3D> createBBox(const std::vector<gp_Pnt>& pointList, double buffer = 0.05);
	/// construct a bbox from the urr and lll points
	static bg::model::box <BoostPoint3D> createBBox(const gp_Pnt& p1, const gp_Pnt& p2, double buffer = 0.05);
	/// construct a OCCTbbox from the urr and lll points
	static TopoDS_Shape createBBOXOCCT(const gp_Pnt& p1, const gp_Pnt& p2, double buffer = 0.0, double horizontalAngle = 0.0, double verticalAngle = 0.0);
	/// construct a Boostbbox from the urr and lll points (can not be rotated)
	/// simplefies shape by creating a smallest bbox around it that is fully orientated
	static TopoDS_Shape boxSimplefyShape(const TopoDS_Shape& shape);
	/// applies the buffer values to the lll and urr point
	static void applyBuffer(gp_Pnt* lllPoint, gp_Pnt* urrPoint, double buffer = 0.0);


	/// Height (z) computing code

	/// Get the lowest Z value of a shape. If areaFilter = true the lowest point of the largest face is taken
	template<typename T>
	static double getLowestZ(const T& shape);
	/// Get the highest Z value of a shape. If areaFilter = true the lowest point of the largest face is taken
	template<typename T>
	static double getHighestZ(const T& shape);
	/// Get the highest Z value of a list of shapes. If areaFilter = true the lowest point of the largest face is taken
	template<typename T>
	static double getHighestZ(const std::vector<T>& faceList);
	/// get the average height of a shape, computed by taking the average height of all the object's vertices
	template<typename T>
	static double getAverageZ(const T& shape);

	/// point on shape code

	/// get the middle of a triangle
	static gp_Pnt getTriangleCenter(const opencascade::handle<Poly_Triangulation>& mesh, const Poly_Triangle& theTriangle, const TopLoc_Location& loc);
	/// get the middlepoint of the face located in the first triangle of its triangulation
	static std::optional<gp_Pnt> getPointOnFace(const TopoDS_Face& theFace);
	/// get the middlepoint list of the face triangulation
	static std::vector<gp_Pnt> getPointListOnFace(const TopoDS_Face& theFace);
	/// get first point on shape (used for wires and edges)
	static gp_Pnt getFirstPointShape(const TopoDS_Shape& shape);
	/// get last point on shape (used for wires and edges)
	static gp_Pnt getLastPointShape(const TopoDS_Shape& shape);
	
	/// direction and angle code

	/// compute the edge dir 
	static gp_Vec computeEdgeDir(const TopoDS_Edge& theEdge);
	/// compute the face normal 
	static gp_Vec computeFaceNormal(const TopoDS_Face& theFace);
	/// make a reversed copy of the input wire
	static TopoDS_Wire reversedWire(const TopoDS_Wire& mainWire);
	/// compute the largest angle of the edges, returns 0 if not found
	static double computeLargestAngle(const TopoDS_Face& theFace);
	/// compute the horizontal dir based on vector count
	static gp_Vec getShapedir(const std::vector<gp_Pnt>& pointList, bool isHorizontal);

	/// overlapping object code

	/// checks if two faces share and edge (if they are resting against eachother)
	static bool shareEdge(const TopoDS_Face& theFace, const TopoDS_Face& theotherFace);
	/// check if edges overlap by checking the endpoints triangular distance
	static bool edgeEdgeOVerlapping(const TopoDS_Edge& currentEdge, const TopoDS_Edge& otherEdge);
	/// check if upperface overlaps the lower face by checking the edges
	static bool faceFaceOverlapping(const TopoDS_Face& upperFace, const TopoDS_Face& lowerFace);
	/// Check if evaluation surface is completely overlapped by other face
	static bool isOverlappingCompletely(const ROSCollection& evalFace, const ROSCollection& otherFace); //TODO: roscollection unique code?
	/// Check if evaluation surface is completely overlapped by other face list
	template<typename T>
	static bool isOverlappingCompletely(const ROSCollection& evalFace, const std::vector<ROSCollection>& facePool, const T& shapeIdx); //TODO: roscollection unique code?

	/// line surface intersection related code

	/// get the signed volume
	static double tVolume(const gp_Pnt& p, const std::vector<gp_Pnt>& vertices);
	/// check if line intersects triangle
	static bool triangleIntersecting(const std::vector<gp_Pnt>& line, const std::vector<gp_Pnt>& triangle);
	/// check if two values have the same sign
	static bool hasSameSign(const double& leftDouble, const double& rightDouble);

	/// line line intersection related code

	/// get the intersection between two linear lines, returns 0 if not intersection
	static std::optional<gp_Pnt> linearLineIntersection(const gp_Pnt& sP1, const gp_Pnt& eP1, const gp_Pnt& sP2, const gp_Pnt& eP2, bool projected, double buffer = 0.01);
	/// get the intersection between two linear lines, returns 0 if not intersection
	static std::optional<gp_Pnt> linearLineIntersection(const Edge& edge1, const Edge& edge2, bool projected, double buffer = 0.01);
	/// get the intersection between two linear lines, returns 0 if not intersection
	static std::optional<gp_Pnt> linearLineIntersection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, bool projected, double buffer = 0.01);

	// surface and wire merging code

	/// merges the input wires in the correct order
	static TopoDS_Wire mergeWireOrientated(const TopoDS_Wire& baseWire, const TopoDS_Wire& mergingWire);
	/// merge faces that rest against eachother
	static std::vector<TopoDS_Face> mergeFaces(const std::vector<TopoDS_Face>& theFaceList);
	/// merge coplanar surfaces
	static std::vector<TopoDS_Face> mergeCoFaces(const std::vector<TopoDS_Face>& theFaceList);
	/// attempts to close an open wire
	static TopoDS_Wire closeWireOrientated(const TopoDS_Wire& baseWire);

	// face creation code

	/// creates face with middlepont 0,0,0 ranging from -x to x and -y to y at z
	static TopoDS_Face createHorizontalFace(double x, double y, double z);
	/// creates a planar face between lll and urr with a rotation
	static TopoDS_Face createHorizontalFace(const gp_Pnt& lll, const gp_Pnt& urr, double rotationAngle);
	/// creates a planar face by connecting the 4 points, make sure the 4 points are on a single plane
	static TopoDS_Face createPlanarFace(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3 = {});
	/// creates a planar copy of input face at input height
	static TopoDS_Face projectFaceFlat(const TopoDS_Face& theFace, double height);

	/// @brief get the footprint shapes from the collection of outer edges
	static std::vector<TopoDS_Face> outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief grows wires from unordered exterior edges
	static std::vector<TopoDS_Wire> growWires(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief cleans the wires (removes redundant vertex)
	static std::vector<TopoDS_Wire> cleanWires(const std::vector<TopoDS_Wire>& wireList);
	static TopoDS_Wire cleanWire(const TopoDS_Wire& wire);
	static std::vector<TopoDS_Face> wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList);

	/// IFC related code

	/// collects the non-standard property data in the ifc file of an object 
	static std::vector<nlohmann::json> collectPropertyValues(std::string objectId, IfcParse::IfcFile* ifcFile);
	/// get the z value stored in the IfcObjectPlacement 
	static double getObjectZOffset(IfcSchema::IfcObjectPlacement* objectPlacement, bool deepOnly);


	/// other code
 
	/// compute the area of a face
	static double computeArea(const TopoDS_Face& theFace);
	/// get a nested list represeting the triangulation of an object
	static void triangulateShape(const TopoDS_Shape& shape);

};
#endif // HELPER_HELPER_H