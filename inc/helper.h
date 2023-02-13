#define USE_IFC4

#ifdef USE_IFC4
#define IfcSchema Ifc4
#else
#define IfcSchema Ifc2x3
#endif // USE_IFC4

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>

#include <GProp_GProps.hxx>
#include <BOPAlgo_Splitter.hxx>

#include <BRepGProp.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <IntTools_EdgeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepSweep_Prism.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

#include <CJT.h>
#include <CJToKernel.h>

#include <memory>

#include <gce_MakeLin2d.hxx>

// Forward Decleration helper class
class helper;
struct roomObject;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> BoostPoint3D;
typedef std::pair<bg::model::box<BoostPoint3D>, int> Value;
typedef std::tuple<IfcSchema::IfcProduct*, std::vector<std::vector<gp_Pnt>>, TopoDS_Shape , bool, TopoDS_Shape> LookupValue;
typedef std::tuple<IfcSchema::IfcProduct*,std::vector<gp_Pnt>, std::vector<roomObject*>*> ConnectLookupValue;
typedef std::tuple<IfcSchema::IfcSpace*, TopoDS_Shape> roomLookupValue;

#ifndef HELPER_HELPER_H
#define HELPER_HELPER_H

// helper functions that can be utilised everywhere
gp_Pnt rotatePointWorld(gp_Pnt p, double angle);
BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle);

void WriteToSTEP(TopoDS_Solid shape, std::string addition);
void WriteToSTEP(TopoDS_Shape shape, std::string addition);

void printPoint(gp_Pnt p);
void printPoint(gp_Pnt2d p);
void printPoint(BoostPoint3D p);
void printPoint(gp_Vec p);
void printPoint(gp_Vec2d p);

void printFaces(TopoDS_Shape shape);

BoostPoint3D Point3DOTB(gp_Pnt oP);

gp_Pnt Point3DBTO(BoostPoint3D oP);

gp_Pnt getLowestPoint(TopoDS_Shape shape, bool areaFilter);
gp_Pnt getHighestPoint(TopoDS_Shape shape);

std::vector<TopoDS_Face> getRoomFootprint(TopoDS_Shape shape);

std::vector<IfcSchema::IfcProduct*> getNestedProducts(IfcSchema::IfcProduct* product);

bool testSolid(TopoDS_Shape shape);

std::vector<std::vector<gp_Pnt>> triangulateShape(TopoDS_Shape* shape);

double tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices);
bool triangleIntersecting(const std::vector<gp_Pnt> line, const std::vector<gp_Pnt> triangle);

std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> checkConnection(TopoDS_Shape roomShape, IfcSchema::IfcSpace* room, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList);
std::vector<IfcSchema::IfcRelSpaceBoundary*> makeSpaceBoundary(IfcSchema::IfcSpace* room, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList);

// finds the ifc schema that is used in the supplied file
bool findSchema(std::string path, bool quiet = false);

class roomObject {
private:
	IfcSchema::IfcSpace* self_;
	std::vector<roomObject*> connections_;
	int doorCount_;
	gp_Pnt point_;
	int indexNum_;
	int sectionNum_ = -1;
	bool isInside_ = true;
	double area_ = 0;

public:

	roomObject(IfcSchema::IfcSpace* s, int i) { self_ = s; point_ = gp_Pnt(i, 0, 0); indexNum_ = i; }

	void setSelf(IfcSchema::IfcSpace* s) { self_ = s; }

	void setPoint(int i) { point_ = gp_Pnt(i, 0, 0); }

	void setIndx(int i) { indexNum_ = i; }

	void setIsOutSide() { isInside_ = false; }

	void setSNum(int i) { sectionNum_ = i; }

	void setArea(double i) { area_ = i; }

	void setDoorCount(int i) { doorCount_ = i; }

	IfcSchema::IfcSpace* getSelf() { return self_; }

	const std::vector<roomObject*> getConnections() { return connections_; }

	void addConnection(roomObject* product) { connections_.emplace_back(product); }

	gp_Pnt getPoint() { return point_; }

	int getIdx() { return indexNum_; }

	int getSNum() { return sectionNum_; }

	bool isInside() { return isInside_; }

	double getArea() { return area_; }

	int getDoorCount() { return doorCount_; }

};

class helperCluster
{
private:
	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;
	double originRot_;

	bool hasBbox_ = false;

	std::vector<helper*> helperList;
	int size_ = 0;

	double hallwayNum_ = 5;
	double minRoom_ = 2;
	double minArea_ = 32;

	std::vector<std::string> objectList_;

public:
	std::vector<helper*> getHelper() const { return helperList; }

	gp_Pnt getLllPoint() const { return lllPoint_; }
	gp_Pnt getUrrPoint() const { return urrPoint_; }
	double getDirection() const { return originRot_; }

	int getSize() const { return size_; }

	void internaliseData();

	bool hasBbox() const { return hasBbox_; }

	void appendHelper(std::string path);
	void appendHelper(helper* data);

	void makeBbox();

	helper* getHelper(int i) { return helperList[i]; }
	std::vector<helper*> getHelpers() { return helperList; }

	// updates the room data of every connectivity object
	void updateConnections(TopoDS_Shape room, roomObject* rObject, boost::geometry::model::box<BoostPoint3D> qBox, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> connectedObjects);
	std::vector<roomObject*> createGraphData();
	std::vector<roomObject*> createGraph(std::vector<roomObject*> rObjectList);
	void writeGraph(std::string path, std::vector<roomObject*> rObjectList);
	void updateRoomCData(std::vector<roomObject*> rObjectList);

	void determineRoomBoundaries();

	void setUseProxy(bool b = true);

	void setApRules(int hallway, int minRooms, double minArea) { hallwayNum_ = hallway; minRoom_ = minRooms; minArea_ = minArea; }

	int getHallwayNum() { return hallwayNum_; }
	int getMinRoomNum() { return minRoom_; }
	int getMinArea() { return minArea_; }

	std::list<std::string> getObjectList();

};

class helper
{
private:

	// The unit multipliers found
	double length_ = 0;
	double area_ = 0;
	double volume_ = 0;

	double objectCount = 0;

	bool hasFloors = false;
	bool isConstruct = false; 
	bool isPartial = false;
	bool hasGeo = false;
	bool hasRooms = false;

	double maxProxyP = 0.3;
	double proxyCount = 0;
	bool hasProxy = false;
	bool hasLotProxy = false;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// The needed rotation for the model to be aligned to the world axis!
	double originRot_;

	std::string path_;
	std::string fileName_;

	IfcParse::IfcFile* file_;
	IfcGeom::Kernel* kernel_;

	static const int treeDepth = 25;
	bgi::rtree<Value, bgi::rstar<treeDepth>> index_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> cIndex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> rIndex_;
	std::vector<LookupValue> productLookup_;
	std::vector<ConnectLookupValue> connectivityLookup_;
	std::vector<roomLookupValue> roomLookup_;
	std::vector<gp_Pnt> roomCenterPoints_;

	bool hasIndex_ = false;
	bool hasCIndex_ = false;
	bool hasRIndex_ = false;

	std::map < int, TopoDS_Shape > shapeLookup_;
	std::map < int, TopoDS_Shape > adjustedshapeLookup_;

	bool useProxy = false;
	std::list<std::string>* roomBoundingObjects_ = {};
	bool useCustom = false;
	bool useCustomFull = false;

	// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

	// returns a list of all the points present in a model
	std::vector<gp_Pnt> getAllPoints(IfcSchema::IfcProduct::list::ptr products);

	// returns a bbox of a ifcproduct that functions with boost
	bg::model::box <BoostPoint3D> makeObjectBox(IfcSchema::IfcProduct* product);
	bg::model::box <BoostPoint3D> makeObjectBox(std::vector<IfcSchema::IfcProduct*> products);
	TopoDS_Solid makeSolidBox(gp_Pnt lll, gp_Pnt urr, double angle);

	template <typename T>
	void addObjectToIndex(T object);

	template <typename T>
	void addObjectToCIndex(T object);

	template <typename T>
	void addObjectToRIndex(T object);

public:
	
	/* 
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper(std::string path);

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	// internalises the geometry while approximating a smallest bbox around the geometry
	void internalizeGeo();

	// internalises the geometry while creating a bbox with one axis along the give angle
	void internalizeGeo(double angle);

	// makes a spatial index for the geometry
	void indexGeo();
	void indexRooms();
	void indexConnectiveShapes();

	// corrects room classification
	void correctRooms();

	std::map<std::string, std::string> getProjectInformation();
	std::map<std::string, std::string> getBuildingInformation();
	std::string getBuildingName();
	std::string getBuildingLongName();

	std::list<std::string> getObjectTypes();

	// returns a vector with length, area and volume multipliers
	std::vector<double> getUnits() const { return { length_, area_, volume_ }; }

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	// returns the area multiplier
	double getAreaMultiplier() const { return area_; }

	// returns the volume multiplier
	double getVolumeMultiplier() const { return volume_; }

	std::string getName() const { return fileName_; }

	std::string getPath() const { return path_; }

	// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile() const { return file_; }

	// returns a pointer to the kernel
	IfcGeom::Kernel* getKernel() const { return kernel_; }

	// returns a pointer to the owner(s)
	IfcSchema::IfcOwnerHistory* getHistory();

	bool getDepending() { return isPartial; }

	bool getIsConstruct() { return isConstruct; }

	bool getHasGeo() { return hasGeo; }

	bool getHasRoom() { return hasRooms; }

	double getProxyNum() { return proxyCount; }

	double getObjectCount() { return objectCount; }

	bool getHasProxy() { return hasProxy; }

	bool getHasLotProxy() { return hasLotProxy; }

	gp_Pnt getLllPoint() { return lllPoint_; }

	gp_Pnt getUrrPoint() { return urrPoint_; }

	double getRotation() { return originRot_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getConnectivityIndexPointer() { return &cIndex_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getRoomIndexPointer() { return &rIndex_; }

	void setRoomBoundingObjects(std::list<std::string>* objectList, bool custom, bool customFull) { roomBoundingObjects_ = objectList; useCustom = custom; useCustomFull = customFull; };

	bool hasClookup() { return hasCIndex_; }

	bool hasIndex() { return hasIndex_; }

	auto getLookup(int i) { return productLookup_[i]; }
	auto updateLookupTriangle(std::vector<std::vector<gp_Pnt>> triangleMeshList, int i) { std::get<1>(productLookup_[i]) = triangleMeshList; }
	
	auto getCLookup(int i) { return connectivityLookup_[i]; }

	auto getRLookup(int i) { return roomLookup_[i]; }

	auto getFullClookup() { return connectivityLookup_; }

	auto getFullRLookup() { return roomLookup_; }

	auto getRoomCenters() { return roomCenterPoints_; }

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool sortEdges = false, bool simple = false);
	std::vector<gp_Pnt> getObjectPoints(TopoDS_Shape shape, bool sortEdges = false);

	std::vector<TopoDS_Face> getObjectFaces(IfcSchema::IfcProduct* product, bool simple = false);

	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false);
	void updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted = false);
	void updateIndex(IfcSchema::IfcProduct* product, TopoDS_Shape shape);
	void applyVoids();

	std::vector<std::vector<gp_Pnt>> triangulateProduct(IfcSchema::IfcProduct* product);

	template <typename T>
	void voidShapeAdjust(T products);

	void setIsConstruct(bool b) { isConstruct = b; }
	
	void setPath(std::string path) { path_ = path; }

	void setName(std::string name) { fileName_ = name; }

	void setHasRooms() { hasRooms = true; }

	//TODO implement
	void whipeObject(IfcSchema::IfcProduct* product);
	
	// add bounding box items for the present objects in the data
	void createBounds(helper* data);

	// deletes all dependencies of an object and the object itself
	static void wipeObject(helper* data, int id);

	void writeToFile(std::string path);

	void setDepending(bool i) { isPartial = i; }

	void setUseProxy(bool b) { useProxy = b; }

	~helper() {};

};

#endif // HELPER_HELPER_H
