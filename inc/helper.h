#define USE_IFC4

#ifdef USE_IFC4
#define IfcSchema Ifc4
#else
#define IfcSchema Ifc2x3
#endif // USE_IFC4

// Boost includes
#include <boost/algorithm/string.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

// IfcOpenShell includes
#include <ifcparse/IfcFile.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>
#include <ifcparse/IfcHierarchyHelper.h>

// OpenCascade includes
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>
#include <TopoDS.hxx>

#include <unordered_set>

#include <CJT.h>
#include <CJToKernel.h>

#include <chrono>

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

void addTimeToJSON(nlohmann::json* j, std::string valueName, std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime);

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
gp_Pnt getPointOnFace(TopoDS_Face theFace);
TopoDS_Wire reversedWire(const TopoDS_Wire& mainWire);
gp_Pnt& getFirstPointShape(const TopoDS_Shape& shape);
gp_Pnt& getLastPointShape(const TopoDS_Shape& shape);
gp_Vec computeFaceNormal(const TopoDS_Face& theFace);

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


class helperCluster
{
private:
	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;
	double originRot_;
	gp_Trsf objectTranslation_;

	bool hasBbox_ = false;

	std::vector<helper> helperList;
	int size_ = 0;

	double hallwayNum_ = 5;
	double minRoom_ = 2;
	double minArea_ = 32;

	std::vector<std::string> objectList_;

public:
	//std::vector<helper*> getHelper() const { return helperList; }

	gp_Pnt getLllPoint() const { return lllPoint_; }
	gp_Pnt getUrrPoint() const { return urrPoint_; }
	double getDirection() const { return originRot_; }

	int getSize() const { return size_; }

	void internaliseData();

	bool hasBbox() const { return hasBbox_; }

	void appendHelper(std::string path);
	void appendHelper(helper data);

	void makeBbox();

	helper* getHelper(int i) { return &helperList[i]; }
	//std::vector<helper*> getHelpers() { return helperList; }

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

	double footprintEvalLvl_ = -0.15;

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
	gp_Trsf objectTranslation_;

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
	TopoDS_Solid makeSolidBox(gp_Pnt lll, gp_Pnt urr, double angle, double extraAngle = 0);

	template <typename T>
	void addObjectToIndex(T object);

	template <typename T>
	void addObjectToCIndex(T object);

	template <typename T>
	void addObjectToRIndex(T object);

	template <typename T>
	std::vector<gp_Pnt> getAllTypePoints(T &typePtr);

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
	void internalizeGeo(double angle, gp_Trsf objectTranslation);

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
	std::string getProjectName();

	std::list<std::string> getObjectTypes();

	// returns a vector with length, area and volume multipliers
	std::vector<double> getUnits() const { return { length_, area_, volume_ }; }

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	// returns the area multiplier
	double getAreaMultiplier() const { return area_; }

	// returns the volume multiplier
	double getVolumeMultiplier() const { return volume_; }

	// returns the floor evalLvl
	double getfootprintEvalLvl() { return footprintEvalLvl_; }

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

	gp_Trsf getObjectTranslation() { return objectTranslation_; }

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

	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false, bool memorize = true);
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

class IOManager {
private:

	// if true no comminucation is pushed to console
	bool isSilent_ = false;

	// if programm is instructed by a json file = true
	bool isJsonInput_ = false;

	std::vector<std::string> inputPathList_ = {};
	std::string outputFolderPath_ = "";

	// sets which LoD envelopes are attampted to be created
	bool make00_ = true;
	bool make02_ = true;
	bool make10_ = true;
	bool make12_ = true;
	bool make13_ = true;
	bool make22_ = true;
	bool make32_ = true;

	bool writeReport_ = true;

	// variables set the deviding objects
	bool useDefaultDiv_ = true;
	bool useProxy_ = false;

	std::unordered_set<std::string> divObjects_ = { // Only used for output purposes
		"IFCSLAB",
		"IFCROOF",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCCOVERING",
		"IFCCOLUMN",
		"IFCBEAM",
		"IFCCURTAINWALL",
		"IFCPLATE",
		"IFCMEMBER",
		"IFCDOOR",
		"IFCWINDOW"
	};

	std::unordered_set<std::string> addDivObjects_ = {
	};

	std::unordered_set<std::string> DevObjectsOptions_ = {
		"IFC2DCOMPOSITECURVE",
		"IFCACTIONREQUEST",
		"IFCACTOR",
		"IFCACTORROLE",
		"IFCACTUATORTYPE",
		"IFCADDRESS",
		"IFCAIRTERMINALBOXTYPE",
		"IFCAIRTERMINALTYPE",
		"IFCAIRTOAIRHEATRECOVERYTYPE",
		"IFCALARMTYPE",
		"IFCANGULARDIMENSION",
		"IFCANNOTATION",
		"IFCANNOTATIONCURVEOCCURRENCE",
		"IFCANNOTATIONFILLAREA",
		"IFCANNOTATIONFILLAREAOCCURRENCE",
		"IFCANNOTATIONOCCURRENCE",
		"IFCANNOTATIONSURFACE",
		"IFCANNOTATIONSURFACEOCCURRENCE",
		"IFCANNOTATIONSYMBOLOCCURRENCE",
		"IFCANNOTATIONTEXTOCCURRENCE",
		"IFCAPPLICATION",
		"IFCAPPLIEDVALUE",
		"IFCAPPLIEDVALUERELATIONSHIP",
		"IFCAPPROVAL",
		"IFCAPPROVALACTORRELATIONSHIP",
		"IFCAPPROVALPROPERTYRELATIONSHIP",
		"IFCAPPROVALRELATIONSHIP",
		"IFCARBITRARYCLOSEDPROFILEDEF",
		"IFCARBITRARYOPENPROFILEDEF",
		"IFCARBITRARYPROFILEDEFWITHVOIDS",
		"IFCASSET",
		"IFCASYMMETRICISHAPEPROFILEDEF",
		"IFCAXIS1PLACEMENT",
		"IFCAXIS2PLACEMENT2D",
		"IFCAXIS2PLACEMENT3D",
		"IFCBSPLINECURVE",
		"IFCBEAM",
		"IFCBEAMTYPE",
		"IFCBEZIERCURVE",
		"IFCBLOBTEXTURE",
		"IFCBLOCK",
		"IFCBOILERTYPE",
		"IFCBOOLEANCLIPPINGRESULT",
		"IFCBOOLEANRESULT",
		"IFCBOUNDARYCONDITION",
		"IFCBOUNDARYEDGECONDITION",
		"IFCBOUNDARYFACECONDITION",
		"IFCBOUNDARYNODECONDITION",
		"IFCBOUNDARYNODECONDITIONWARPING",
		"IFCBOUNDEDCURVE",
		"IFCBOUNDEDSURFACE",
		"IFCBOUNDINGBOX",
		"IFCBOXEDHALFSPACE",
		"IFCBUILDING",
		"IFCBUILDINGELEMENT",
		"IFCBUILDINGELEMENTCOMPONENT",
		"IFCBUILDINGELEMENTPART",
		"IFCBUILDINGELEMENTPROXY",
		"IFCBUILDINGELEMENTPROXYTYPE",
		"IFCBUILDINGELEMENTTYPE",
		"IFCBUILDINGSTOREY",
		"IFCCSHAPEPROFILEDEF",
		"IFCCABLECARRIERFITTINGTYPE",
		"IFCCABLECARRIERSEGMENTTYPE",
		"IFCCABLESEGMENTTYPE",
		"IFCCALENDARDATE",
		"IFCCARTESIANPOINT",
		"IFCCARTESIANTRANSFORMATIONOPERATOR",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM",
		"IFCCENTERLINEPROFILEDEF",
		"IFCCHAMFEREDGEFEATURE",
		"IFCCHILLERTYPE",
		"IFCCIRCLE",
		"IFCCIRCLEHOLLOWPROFILEDEF",
		"IFCCIRCLEPROFILEDEF",
		"IFCCLASSIFICATION",
		"IFCCLASSIFICATIONITEM",
		"IFCCLASSIFICATIONITEMRELATIONSHIP",
		"IFCCLASSIFICATIONNOTATION",
		"IFCCLASSIFICATIONNOTATIONFACET",
		"IFCCLASSIFICATIONREFERENCE",
		"IFCCLOSEDSHELL",
		"IFCCOILTYPE",
		"IFCCOLOURRGB",
		"IFCCOLOURSPECIFICATION",
		"IFCCOLUMN",
		"IFCCOLUMNTYPE",
		"IFCCOMPLEXPROPERTY",
		"IFCCOMPOSITECURVE",
		"IFCCOMPOSITECURVESEGMENT",
		"IFCCOMPOSITEPROFILEDEF",
		"IFCCOMPRESSORTYPE",
		"IFCCONDENSERTYPE",
		"IFCCONDITION",
		"IFCCONDITIONCRITERION",
		"IFCCONIC",
		"IFCCONNECTEDFACESET",
		"IFCCONNECTIONCURVEGEOMETRY",
		"IFCCONNECTIONGEOMETRY",
		"IFCCONNECTIONPOINTECCENTRICITY",
		"IFCCONNECTIONPOINTGEOMETRY",
		"IFCCONNECTIONPORTGEOMETRY",
		"IFCCONNECTIONSURFACEGEOMETRY",
		"IFCCONSTRAINT",
		"IFCCONSTRAINTAGGREGATIONRELATIONSHIP",
		"IFCCONSTRAINTCLASSIFICATIONRELATIONSHIP",
		"IFCCONSTRAINTRELATIONSHIP",
		"IFCCONSTRUCTIONEQUIPMENTRESOURCE",
		"IFCCONSTRUCTIONMATERIALRESOURCE",
		"IFCCONSTRUCTIONPRODUCTRESOURCE",
		"IFCCONSTRUCTIONRESOURCE",
		"IFCCONTEXTDEPENDENTUNIT",
		"IFCCONTROL",
		"IFCCONTROLLERTYPE",
		"IFCCONVERSIONBASEDUNIT",
		"IFCCOOLEDBEAMTYPE",
		"IFCCOOLINGTOWERTYPE",
		"IFCCOORDINATEDUNIVERSALTIMEOFFSET",
		"IFCCOSTITEM",
		"IFCCOSTSCHEDULE",
		"IFCCOSTVALUE",
		"IFCCOVERING",
		"IFCCOVERINGTYPE",
		"IFCCRANERAILASHAPEPROFILEDEF",
		"IFCCRANERAILFSHAPEPROFILEDEF",
		"IFCCREWRESOURCE",
		"IFCCSGPRIMITIVE3D",
		"IFCCSGSOLID",
		"IFCCURRENCYRELATIONSHIP",
		"IFCCURTAINWALL",
		"IFCCURTAINWALLTYPE",
		"IFCCURVE",
		"IFCCURVEBOUNDEDPLANE",
		"IFCCURVESTYLE",
		"IFCCURVESTYLEFONT",
		"IFCCURVESTYLEFONTANDSCALING",
		"IFCCURVESTYLEFONTPATTERN",
		"IFCDAMPERTYPE",
		"IFCDATEANDTIME",
		"IFCDEFINEDSYMBOL",
		"IFCDERIVEDPROFILEDEF",
		"IFCDERIVEDUNIT",
		"IFCDERIVEDUNITELEMENT",
		"IFCDIAMETERDIMENSION",
		"IFCDIMENSIONCALLOUTRELATIONSHIP",
		"IFCDIMENSIONCURVE",
		"IFCDIMENSIONCURVEDIRECTEDCALLOUT",
		"IFCDIMENSIONCURVETERMINATOR",
		"IFCDIMENSIONPAIR",
		"IFCDIMENSIONALEXPONENTS",
		"IFCDIRECTION",
		"IFCDISCRETEACCESSORY",
		"IFCDISCRETEACCESSORYTYPE",
		"IFCDISTRIBUTIONCHAMBERELEMENT",
		"IFCDISTRIBUTIONCHAMBERELEMENTTYPE",
		"IFCDISTRIBUTIONCONTROLELEMENT",
		"IFCDISTRIBUTIONCONTROLELEMENTTYPE",
		"IFCDISTRIBUTIONELEMENT",
		"IFCDISTRIBUTIONELEMENTTYPE",
		"IFCDISTRIBUTIONFLOWELEMENT",
		"IFCDISTRIBUTIONFLOWELEMENTTYPE",
		"IFCDISTRIBUTIONPORT",
		"IFCDOCUMENTELECTRONICFORMAT",
		"IFCDOCUMENTINFORMATION",
		"IFCDOCUMENTINFORMATIONRELATIONSHIP",
		"IFCDOCUMENTREFERENCE",
		"IFCDOOR",
		"IFCDOORLININGPROPERTIES",
		"IFCDOORPANELPROPERTIES",
		"IFCDOORSTYLE",
		"IFCDRAUGHTINGCALLOUT",
		"IFCDRAUGHTINGCALLOUTRELATIONSHIP",
		"IFCDRAUGHTINGPREDEFINEDCOLOUR",
		"IFCDRAUGHTINGPREDEFINEDCURVEFONT",
		"IFCDRAUGHTINGPREDEFINEDTEXTFONT",
		"IFCDUCTFITTINGTYPE",
		"IFCDUCTSEGMENTTYPE",
		"IFCDUCTSILENCERTYPE",
		"IFCEDGE",
		"IFCEDGECURVE",
		"IFCEDGEFEATURE",
		"IFCEDGELOOP",
		"IFCELECTRICAPPLIANCETYPE",
		"IFCELECTRICDISTRIBUTIONPOINT",
		"IFCELECTRICFLOWSTORAGEDEVICETYPE",
		"IFCELECTRICGENERATORTYPE",
		"IFCELECTRICHEATERTYPE",
		"IFCELECTRICMOTORTYPE",
		"IFCELECTRICTIMECONTROLTYPE",
		"IFCELECTRICALBASEPROPERTIES",
		"IFCELECTRICALCIRCUIT",
		"IFCELECTRICALELEMENT",
		"IFCELEMENT",
		"IFCELEMENTASSEMBLY",
		"IFCELEMENTCOMPONENT",
		"IFCELEMENTCOMPONENTTYPE",
		"IFCELEMENTQUANTITY",
		"IFCELEMENTTYPE",
		"IFCELEMENTARYSURFACE",
		"IFCELLIPSE",
		"IFCELLIPSEPROFILEDEF",
		"IFCENERGYCONVERSIONDEVICE",
		"IFCENERGYCONVERSIONDEVICETYPE",
		"IFCENERGYPROPERTIES",
		"IFCENVIRONMENTALIMPACTVALUE",
		"IFCEQUIPMENTELEMENT",
		"IFCEQUIPMENTSTANDARD",
		"IFCEVAPORATIVECOOLERTYPE",
		"IFCEVAPORATORTYPE",
		"IFCEXTENDEDMATERIALPROPERTIES",
		"IFCEXTERNALREFERENCE",
		"IFCEXTERNALLYDEFINEDHATCHSTYLE",
		"IFCEXTERNALLYDEFINEDSURFACESTYLE",
		"IFCEXTERNALLYDEFINEDSYMBOL",
		"IFCEXTERNALLYDEFINEDTEXTFONT",
		"IFCEXTRUDEDAREASOLID",
		"IFCFACE",
		"IFCFACEBASEDSURFACEMODEL",
		"IFCFACEBOUND",
		"IFCFACEOUTERBOUND",
		"IFCFACESURFACE",
		"IFCFACETEDBREP",
		"IFCFACETEDBREPWITHVOIDS",
		"IFCFAILURECONNECTIONCONDITION",
		"IFCFANTYPE",
		"IFCFASTENER",
		"IFCFASTENERTYPE",
		"IFCFEATUREELEMENT",
		"IFCFEATUREELEMENTADDITION",
		"IFCFEATUREELEMENTSUBTRACTION",
		"IFCFILLAREASTYLE",
		"IFCFILLAREASTYLEHATCHING",
		"IFCFILLAREASTYLETILESYMBOLWITHSTYLE",
		"IFCFILLAREASTYLETILES",
		"IFCFILTERTYPE",
		"IFCFIRESUPPRESSIONTERMINALTYPE",
		"IFCFLOWCONTROLLER",
		"IFCFLOWCONTROLLERTYPE",
		"IFCFLOWFITTING",
		"IFCFLOWFITTINGTYPE",
		"IFCFLOWINSTRUMENTTYPE",
		"IFCFLOWMETERTYPE",
		"IFCFLOWMOVINGDEVICE",
		"IFCFLOWMOVINGDEVICETYPE",
		"IFCFLOWSEGMENT",
		"IFCFLOWSEGMENTTYPE",
		"IFCFLOWSTORAGEDEVICE",
		"IFCFLOWSTORAGEDEVICETYPE",
		"IFCFLOWTERMINAL",
		"IFCFLOWTERMINALTYPE",
		"IFCFLOWTREATMENTDEVICE",
		"IFCFLOWTREATMENTDEVICETYPE",
		"IFCFLUIDFLOWPROPERTIES",
		"IFCFOOTING",
		"IFCFUELPROPERTIES",
		"IFCFURNISHINGELEMENT",
		"IFCFURNISHINGELEMENTTYPE",
		"IFCFURNITURESTANDARD",
		"IFCFURNITURETYPE",
		"IFCGASTERMINALTYPE",
		"IFCGENERALMATERIALPROPERTIES",
		"IFCGENERALPROFILEPROPERTIES",
		"IFCGEOMETRICCURVESET",
		"IFCGEOMETRICREPRESENTATIONCONTEXT",
		"IFCGEOMETRICREPRESENTATIONITEM",
		"IFCGEOMETRICREPRESENTATIONSUBCONTEXT",
		"IFCGEOMETRICSET",
		"IFCGRID",
		"IFCGRIDAXIS",
		"IFCGRIDPLACEMENT",
		"IFCGROUP",
		"IFCHALFSPACESOLID",
		"IFCHEATEXCHANGERTYPE",
		"IFCHYGROMATERIALPROPERTIES",
		"IFCSHAPEPROFILEDEF",
		"IFCIMAGE",
		"IFCINVENTORY",
		"IFCIRREGULARTIMESERIES",
		"IFCIRREGULARTIMESERIESVALUE",
		"IFCJUNCTIONBOXTYPE",
		"IFCLSHAPEPROFILEDEF",
		"IFCLABORRESOURCE",
		"IFCLAMPTYPE",
		"IFCLIBRARYINFORMATION",
		"IFCLIBRARYREFERENCE",
		"IFCLIGHTDISTRIBUTIONDATA",
		"IFCLIGHTFIXTURETYPE",
		"IFCLIGHTINTENSITYDISTRIBUTION",
		"IFCLIGHTSOURCE",
		"IFCLIGHTSOURCEAMBIENT",
		"IFCLIGHTSOURCEDIRECTIONAL",
		"IFCLIGHTSOURCEGONIOMETRIC",
		"IFCLIGHTSOURCEPOSITIONAL",
		"IFCLIGHTSOURCESPOT",
		"IFCLINE",
		"IFCLINEARDIMENSION",
		"IFCLOCALPLACEMENT",
		"IFCLOCALTIME",
		"IFCLOOP",
		"IFCMANIFOLDSOLIDBREP",
		"IFCMAPPEDITEM",
		"IFCMATERIAL",
		"IFCMATERIALCLASSIFICATIONRELATIONSHIP",
		"IFCMATERIALDEFINITIONREPRESENTATION",
		"IFCMATERIALLAYER",
		"IFCMATERIALLAYERSET",
		"IFCMATERIALLAYERSETUSAGE",
		"IFCMATERIALLIST",
		"IFCMATERIALPROPERTIES",
		"IFCMEASUREWITHUNIT",
		"IFCMECHANICALCONCRETEMATERIALPROPERTIES",
		"IFCMECHANICALFASTENER",
		"IFCMECHANICALFASTENERTYPE",
		"IFCMECHANICALMATERIALPROPERTIES",
		"IFCMECHANICALSTEELMATERIALPROPERTIES",
		"IFCMEMBER",
		"IFCMEMBERTYPE",
		"IFCMETRIC",
		"IFCMONETARYUNIT",
		"IFCMOTORCONNECTIONTYPE",
		"IFCMOVE",
		"IFCNAMEDUNIT",
		"IFCOBJECT",
		"IFCOBJECTDEFINITION",
		"IFCOBJECTPLACEMENT",
		"IFCOBJECTIVE",
		"IFCOCCUPANT",
		"IFCOFFSETCURVE2D",
		"IFCOFFSETCURVE3D",
		"IFCONEDIRECTIONREPEATFACTOR",
		"IFCOPENSH",
		"IFCOPENINGELEMENT",
		"IFCOPTICALMATERIALPROPERTIES",
		"IFCORDERACTION",
		"IFCORGANIZATION",
		"IFCORGANIZATIONRELATIONSHIP",
		"IFCORIENTEDEDGE",
		"IFCOUTLETTYPE",
		"IFCOWNERHISTORY",
		"IFCPARAMETERIZEDPROFILEDEF",
		"IFCPATH",
		"IFCPERFORMANCEHISTORY",
		"IFCPERMEABLECOVERINGPROPERTIES",
		"IFCPERMIT",
		"IFCPERSON",
		"IFCPERSONANDORGANIZATION",
		"IFCPHYSICALCOMPLEXQUANTITY",
		"IFCPHYSICALQUANTITY",
		"IFCPHYSICALSIMPLEQUANTITY",
		"IFCPILE",
		"IFCPIPEFITTINGTYPE",
		"IFCPIPESEGMENTTYPE",
		"IFCPIXELTEXTURE",
		"IFCPLACEMENT",
		"IFCPLANARBOX",
		"IFCPLANAREXTENT",
		"IFCPLANE",
		"IFCPLATE",
		"IFCPLATETYPE",
		"IFCPOINT",
		"IFCPOINTONCURVE",
		"IFCPOINTONSURFACE",
		"IFCPOLYLOOP",
		"IFCPOLYGONALBOUNDEDHALFSPACE",
		"IFCPOLYLINE",
		"IFCPORT",
		"IFCPOSTALADDRESS",
		"IFCPREDEFINEDCOLOUR",
		"IFCPREDEFINEDCURVEFONT",
		"IFCPREDEFINEDDIMENSIONSYMBOL",
		"IFCPREDEFINEDITEM",
		"IFCPREDEFINEDPOINTMARKERSYMBOL",
		"IFCPREDEFINEDSYMBOL",
		"IFCPREDEFINEDTERMINATORSYMBOL",
		"IFCPREDEFINEDTEXTFONT",
		"IFCPRESENTATIONLAYERASSIGNMENT",
		"IFCPRESENTATIONLAYERWITHSTYLE",
		"IFCPRESENTATIONSTYLE",
		"IFCPRESENTATIONSTYLEASSIGNMENT",
		"IFCPROCEDURE",
		"IFCPROCESS",
		"IFCPRODUCT",
		"IFCPRODUCTDEFINITIONSHAPE",
		"IFCPRODUCTREPRESENTATION",
		"IFCPRODUCTSOFCOMBUSTIONPROPERTIES",
		"IFCPROFILEDEF",
		"IFCPROFILEPROPERTIES",
		"IFCPROJECT",
		"IFCPROJECTORDER",
		"IFCPROJECTORDERRECORD",
		"IFCPROJECTIONCURVE",
		"IFCPROJECTIONELEMENT",
		"IFCPROPERTY",
		"IFCPROPERTYBOUNDEDVALUE",
		"IFCPROPERTYCONSTRAINTRELATIONSHIP",
		"IFCPROPERTYDEFINITION",
		"IFCPROPERTYDEPENDENCYRELATIONSHIP",
		"IFCPROPERTYENUMERATEDVALUE",
		"IFCPROPERTYENUMERATION",
		"IFCPROPERTYLISTVALUE",
		"IFCPROPERTYREFERENCEVALUE",
		"IFCPROPERTYSET",
		"IFCPROPERTYSETDEFINITION",
		"IFCPROPERTYSINGLEVALUE",
		"IFCPROPERTYTABLEVALUE",
		"IFCPROTECTIVEDEVICETYPE",
		"IFCPROXY",
		"IFCPUMPTYPE",
		"IFCQUANTITYAREA",
		"IFCQUANTITYCOUNT",
		"IFCQUANTITYLENGTH",
		"IFCQUANTITYTIME",
		"IFCQUANTITYVOLUME",
		"IFCQUANTITYWEIGHT",
		"IFCRADIUSDIMENSION",
		"IFCRAILING",
		"IFCRAILINGTYPE",
		"IFCRAMP",
		"IFCRAMPFLIGHT",
		"IFCRAMPFLIGHTTYPE",
		"IFCSTRUCTURALACTION",
		"IFCSTRUCTURALACTIVITY",
		"IFCSTRUCTURALANALYSISMODEL",
		"IFCSTRUCTURALCONNECTION",
		"IFCSTRUCTURALCONNECTIONCONDITION",
		"IFCSTRUCTURALCURVECONNECTION",
		"IFCSTRUCTURALCURVEMEMBER",
		"IFCSTRUCTURALCURVEMEMBERVARYING",
		"IFCSTRUCTURALITEM",
		"IFCSTRUCTURALLINEARACTION",
		"IFCSTRUCTURALLINEARACTIONVARYING",
		"IFCSTRUCTURALLOAD",
		"IFCSTRUCTURALLOADGROUP",
		"IFCSTRUCTURALLOADLINEARFORCE",
		"IFCSTRUCTURALLOADPLANARFORCE",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENT",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENTDISTORTION",
		"IFCSTRUCTURALLOADSINGLEFORCE",
		"IFCSTRUCTURALLOADSINGLEFORCEWARPING",
		"IFCSTRUCTURALLOADTEMPERATURE",
		"IFCSTRUCTURALMEMBER",
		"IFCSTRUCTURALPLANARACTION",
		"IFCSTRUCTURALPOINTACTION",
		"IFCSTRUCTURALPOINTCONNECTION",
		"IFCSTRUCTURALPOINTREACTION",
		"IFCSTRUCTURALREACTION",
		"IFCSTRUCTURALRESULTGROUP",
		"IFCSTRUCTURALSTEELPROFILEPROPERTIES",
		"IFCSTRUCTURALSURFACECONNECTION",
		"IFCSTRUCTURALSURFACEMEMBER",
		"IFCSTRUCTURALSURFACEMEMBERVARYING",
		"IFCSTRUCTUREDDIMENSIONCALLOUT",
		"IFCSUBCONTRACTRESOURCE",
		"IFCSUBEDGE",
		"IFCSURFACE",
		"IFCSURFACECURVESWEPTAREASOLID",
		"IFCSURFACEOFLINEAREXTRUSION",
		"IFCSURFACEOFREVOLUTION",
		"IFCSURFACESTYLE",
		"IFCSURFACESTYLELIGHTING",
		"IFCSURFACESTYLEREFRACTION",
		"IFCSURFACESTYLERENDERING",
		"IFCSURFACESTYLESHADING",
		"IFCSURFACESTYLEWITHTEXTURES",
		"IFCSWEPTAREASOLID",
		"IFCSWITCHINGDEVICETYPE",
		"IFCSYMBOLSTYLE",
		"IFCSYMBOLSTYLESELECT",
		"IFCSYSTEM",
		"IFCSYSTEMFURNITUREELEMENTTYPE",
		"IFCTSHAPEPROFILEDEF",
		"IFCTABLE",
		"IFCTABLEROW",
		"IFCTANKTYPE",
		"IFCTASK",
		"IFCTASKTIME",
		"IFCTAXONOMICCLASSIFICATION",
		"IFCTAXONOMICCLASSIFICATIONRELATIONSHIP",
		"IFCTEXTSTYLE",
		"IFCTHEATRE",
		"IFCTHEATRETYPE",
		"IFCTHERMALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALPROPERTYSET",
		"IFCTHERMOPHYSICALPROPERTYSETUSAGE",
		"IFCTHERMOPHYSICALSIMPLEPROPERTY",
		"IFCTIMEPERIOD",
		"IFCTIMESERIES",
		"IFCTIMESERIESGROUP",
		"IFCTIMESERIESREFERENCE",
		"IFCTIMESERIESSCHEDULE",
		"IFCTIMESERIESVALUE",
		"IFCTOPOLOGICALREPRESENTATIONITEM",
		"IFCTRANSFORMERRESOURCE",
		"IFCTRANSPORTELEMENT",
		"IFCTRANSPORTELEMENTTYPE",
		"IFCTRANSPORTELEMENTSTATICDEFLECTION",
		"IFCTRANSPORTELEMENTSTATICREACTION",
		"IFCTRAPEZIUMPROFILEDEF",
		"IFCTRIMMEDCURVE",
		"IFCTUBEBUNDLETYPE",
		"IFCTWIRLGENERATORTYPE",
		"IFCUNITASSIGNMENT",
		"IFCUNITARYCONTROLELEMENT",
		"IFCUNITARYCONTROLELEMENTTYPE",
		"IFCUNITARYEQUIPMENT",
		"IFCUNITARYEQUIPMENTTYPE",
		"IFCUNITARYPRODUCT",
		"IFCUNITARYPRODUCTTYPE",
		"IFCVALVE",
		"IFCVALVETYPE",
		"IFCVECTOR",
		"IFCVERTEX",
		"IFCVERTEXLOOP",
		"IFCVERTEXPOINT",
		"IFCVIBRATORYPE",
		"IFCVIRTUALELEMENT",
		"IFCVIRTUALELEMENTTYPE",
		"IFCVOIDINGFEATURE",
		"IFCVOIDINGFEATURETYPE",
		"IFCVOLUMEBEAM",
		"IFCVOLUMEBEAMTYPE",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCWALLTYPE",
		"IFCWARPINGCONSTANTMEASURE",
		"IFCWARPINGMOMENTOFINERTIA_MEASURE",
		"IFCWINDOW",
		"IFCWINDOWLININGPROPERTIES",
		"IFCWINDOWPANELPROPERTIES",
		"IFCWINDOWSTYLE",
		"IFCZSHAPEPROFILEDEF"
	};

	double voxelSize_ = 0.5;

	double footprintElevation_ = 0.0;

	// how many proxy objects are present in the input
	int proxyCount_ = 0;

	helperCluster hCluster_;

	// time summary for the output
	double timeInternalizing_ = -1;
	double timeInternalizingIsS_ = false;

	double timeLoD00_ = -1;
	bool timeLoD00IsS_ = false;
	double timeLoD02_ = -1;
	bool timeLoD02IsS_ = false;
	double timeLoD10_ = -1;
	bool timeLoD10IsS_ = false;
	double timeLoD12_ = -1;
	bool timeLoD12IsS_ = false;
	double timeLoD13_ = -1;
	bool timeLoD13IsS_ = false;
	double timeLoD22_ = -1;
	bool timeLoD22IsS_ = false;
	double timeLoD32_ = -1;
	bool timeLoD32IsS_ = false;

	double timeProcess = -1;
	double timeTotal = -1;

	// question askers
	bool yesNoQuestion();
	int numQuestion(int n, bool lower = true);

	bool getTargetPathList();
	bool getOutputPathList();
	std::string getFileName(const std::string& stringPath);

	bool getUseDefaultSettings();

	bool getDesiredLoD();

	bool getBoudingRules();

	bool getVoxelSize();

	bool getFootprintElev();

	// attempts to ask the user for settings
	bool getUserValues();

	// attempts to get the settings from json file
	bool getJSONValues();

	// checks if the string has the extension that is supplied
	bool hasExtension(const std::vector<std::string>& stringList, const std::string& ext);
	bool hasExtension(const std::string& string, const std::string& ext);

	// console outputs the settings that are utilized
	void printSummary();

	std::string getLoDEnabled();

public:
	bool init(const std::vector<std::string>& inputPathList, bool silent = false);

	bool run();

	// temp data

	double voxelSize() { return voxelSize_; }
	bool makeReport() { return writeReport_; }

	std::string getOutputPath() { return outputFolderPath_; }

	helperCluster helpCluster() { return hCluster_; }
	
	bool makeLoD00() { return make00_; }
	bool makeLoD02() { return make02_; }
	bool makeLoD10() { return make10_; }
	bool makeLoD12() { return make12_; }
	bool makeLoD13() { return make13_; }
	bool makeLoD22() { return make22_; }
	bool makeLoD32() { return make32_; }

};



#endif // HELPER_HELPER_H
