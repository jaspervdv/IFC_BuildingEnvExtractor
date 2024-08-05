#define USE_IFC4x3

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
	/// Output shape to step file (for development only)
	static void WriteToSTEP(const TopoDS_Solid& shape, const std::string& addition);
	static void WriteToSTEP(const TopoDS_Shape& shape, const std::string& addition);
	static void WriteToSTEP(const std::vector<TopoDS_Face>& shapeList, const std::string& addition);
	static void WriteToSTEP(const std::vector<TopoDS_Shape>& shapeList, const std::string& addition);

	/// Ouptu shape to txt file (for development only)
	static void WriteToTxt(const std::vector<TopoDS_Face>& shapeList, const std::string& addition);
	static void WriteToTxt(const std::vector<TopoDS_Solid>& shapeList, const std::string& addition);

	/// Print point to console (for development only)
	static void printPoint(const gp_Pnt& p);
	static void printPoint(const gp_Pnt2d& p);
	static void printPoint(const BoostPoint3D& p);
	static void printPoint(const gp_Vec& p);
	static void printPoint(const gp_Vec2d& p);

	/// Print points of the faces to console (for development only)
	static void printFaces(const TopoDS_Shape& shape);

	/// get a list of unique points from a pointlist
	static std::vector<gp_Pnt> getUniquePoints(const std::vector<gp_Pnt>& pointList);
	/// get a list of unique points from a shape
	static std::vector<gp_Pnt> getUniquePoints(const TopoDS_Shape& inputShape);
	static int getPointCount(const TopoDS_Shape& inputShape);

	///	Rotate OpenCascade point around 0,0,0
	static gp_Pnt rotatePointWorld(const gp_Pnt& p, double angle);
	///	Rotate Boost point around 0,0,0
	static BoostPoint3D rotatePointWorld(const BoostPoint3D& p, double angle);

	/// Rotate OpenCascade point p around point anchorP
	static gp_Pnt rotatePointPoint(const gp_Pnt& p, const gp_Pnt& anchorP, double angle);

	static bool bBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, double buffer = 0);
	static bool rotatedBBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, double angle, double secondAngle = 0);

	/// Convert OpenCascade point to Boost point
	static BoostPoint3D Point3DOTB(const gp_Pnt& oP);

	/// Conver Boost point to OpenCascade point
	static gp_Pnt Point3DBTO(const BoostPoint3D& oP);

	/// Get the lowest point of a shape. If areaFilter = true the lowest point of the largest face is taken
	static gp_Pnt getLowestPoint(const TopoDS_Shape& shape, bool areaFilter);
	/// Get the highest point of a shape
	static gp_Pnt getHighestPoint(const TopoDS_Shape& shape);
	/// get the middlepoint of the face located in the first triangle of its triangulation
	static std::optional<gp_Pnt> getPointOnFace(const TopoDS_Face& theFace);
	static std::vector<gp_Pnt> getPointListOnFace(const TopoDS_Face& theFace);
	/// make a reversed copy of the input wire
	static TopoDS_Wire reversedWire(const TopoDS_Wire& mainWire);
	/// get first point on shape (used for wires and edges)
	static gp_Pnt getFirstPointShape(const TopoDS_Shape& shape);
	/// get last point on shape (used for wires and edges)
	static gp_Pnt getLastPointShape(const TopoDS_Shape& shape);
	/// compute the face normal 
	static gp_Vec computeFaceNormal(const TopoDS_Face& theFace);

	/// compute the largest angle of the edges, returns 0 if not found
	static double computeLargestAngle(const TopoDS_Face& theFace);

	/// check if edges overlap by checking the endpoints
	static bool edgeEdgeOVerlapping(const TopoDS_Edge& currentEdge, const TopoDS_Edge& otherEdge);

	static std::vector<TopoDS_Face> shape2FaceList(const TopoDS_Shape& shape);
	static std::vector<gp_Pnt> shape2PointList(const TopoDS_Shape& shape);

	/// get the products nested in this object
	static std::vector<IfcSchema::IfcProduct*> getNestedProducts(IfcSchema::IfcProduct* product);

	/// translates objects back to original positions, transforms to coply with georeferencing, and translates objects back to corrected position
	static void geoTransform(TopoDS_Shape* shape, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans);
	static void geoTransform(TopoDS_Face* shape, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans);
	static void geoTransform(gp_Pnt* point, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans);

	/// get a nested list represeting the triangulation of an object
	static void triangulateShape(const TopoDS_Shape& shape);

	/// get the signed volume
	static double tVolume(const gp_Pnt& p, const std::vector<gp_Pnt>& vertices);

	/// check if line intersects triangle
	static bool triangleIntersecting(const std::vector<gp_Pnt>& line, const std::vector<gp_Pnt>& triangle);

	/// check is boost box has a volume
	static bool hasVolume(const bg::model::box <BoostPoint3D>& box);

	/// compute the area of a face
	static double computeArea(const TopoDS_Face& theFace);

	/// check if value is in vector T1
	template<typename T1, typename T2>
	static bool isInList(const T1& list, const T2& value);

	/// get the average height of a shape, computed by taking the average height of all the object's vertices
	static double getAvFaceHeight(const TopoDS_Face& face);

	/// get the height of the heighest vertex
	static double getTopFaceHeight(const TopoDS_Face& face);

	/// gets the direction that the edge is orentated towards
	static gp_Vec getDirEdge(const TopoDS_Edge& edge);

	/// merges the input wires in the correct order
	static TopoDS_Wire mergeWireOrientated(const TopoDS_Wire& baseWire, const TopoDS_Wire& mergingWire);

	static std::vector<gp_Pnt> getPointGridOnSurface(const TopoDS_Face& theface);
	static std::vector<gp_Pnt> getPointGridOnWire(const TopoDS_Face& theface);

	/// attempts to close an open wire
	static TopoDS_Wire closeWireOrientated(const TopoDS_Wire& baseWire);

	/// inverts the dirInx for neighbourSearching
	static int invertDir(int dirIndx);

	/// construct a bbox from a shape or list of shapes
	static bg::model::box <BoostPoint3D> createBBox(const TopoDS_Shape& shape);
	static bg::model::box <BoostPoint3D> createBBox(const std::vector<TopoDS_Shape>& shape);
	static bg::model::box <BoostPoint3D> createBBox(const gp_Pnt& p1, const gp_Pnt& p2, double buffer = 0.0);

	static TopoDS_Shape createBBOXOCCT(const gp_Pnt& p1, const gp_Pnt& p2, double buffer = 0.0);

	/// creates face with middlepont 0,0,0 ranging from -x to x and -y to y at z
	static TopoDS_Face createHorizontalFace(double x, double y, double z);

	/// creates a planar face between lll and urr with a rotation
	static TopoDS_Face createHorizontalFace(const gp_Pnt& lll, const gp_Pnt& urr, double rotationAngle);

	/// creates a planar face by connecting the 4 points, make sure the 4 points are on a single plane
	static TopoDS_Face createPlanarFace(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3 = {});

	/// creates a planar copy of input face at input height
	static TopoDS_Face projectFaceFlat(const TopoDS_Face& theFace, double height);

	/// get the intersection between two linear lines, returns 0 if not intersection
	static std::optional<gp_Pnt> linearLineIntersection(const gp_Pnt& sP1, const gp_Pnt& eP1, const gp_Pnt& sP2, const gp_Pnt& eP2, bool projected, double buffer = 0.01);
	static std::optional<gp_Pnt> linearLineIntersection(const Edge& edge1, const Edge& edge2, bool projected, double buffer = 0.01);
	static std::optional<gp_Pnt> linearLineIntersection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, bool projected, double buffer = 0.01);

	/// Check if surface is completely overlapped
	static bool isOverlappingCompletely(const SurfaceGroup& evalFace, const SurfaceGroup& otherFace);

	template<typename T>
	static bool isOverlappingCompletely(const SurfaceGroup& evalFace, const std::vector<SurfaceGroup>& facePool, const T& shapeIdx)
	{
		std::vector<Value> qResult;
		shapeIdx.query(bgi::intersects(
			bg::model::box <BoostPoint3D>(
				createBBox(evalFace.getFaces()[0])
				)), std::back_inserter(qResult));
		for (size_t i = 0; i < qResult.size(); i++)
		{
			if (isOverlappingCompletely(evalFace, facePool[qResult[i].second])) { return true; }
		}
		return false;
	}

};
#endif // HELPER_HELPER_H