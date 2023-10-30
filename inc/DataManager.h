#include "helper.h"

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

#ifndef DATAMANAGER_DATAMANAGER_H
#define DATAMANAGER_DATAMANAGER_H

// lookup for the major spatial index used in the code (indexing all the objects in the ifc file)
class lookupValue
{
private:
	IfcSchema::IfcProduct* productPtr_;
	std::vector<std::vector<gp_Pnt>> triangulatedShape_;
	TopoDS_Shape cBox_;

public:
	lookupValue(IfcSchema::IfcProduct* productPtr, const std::vector<std::vector<gp_Pnt>>& triangulatedShape, const TopoDS_Shape& cBox);

	IfcSchema::IfcProduct* getProductPtr() { return productPtr_; }

	bool hasTraingulatedShape();

	std::vector<std::vector<gp_Pnt>> getTriangluatedShape() { return triangulatedShape_; }

	bool hasCBox() { return !cBox_.IsNull(); }

	TopoDS_Shape getCBox() { return cBox_; }
};

class helper
{
private:
	// The unit multipliers found
	double length_ = 0;
	double area_ = 0;
	double volume_ = 0;

	double objectCount_ = 0;

	double footprintEvalLvl_ = -0.15;

	bool hasGeo_ = false;

	double maxProxyP_ = 0.3;
	double proxyCount_ = 0;
	bool hasProxy_ = false;
	bool hasLotProxy_ = false;

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
	std::vector<lookupValue> productLookup_;

	bool hasIndex_ = false;

	std::map < int, TopoDS_Shape > shapeLookup_;
	std::map < int, TopoDS_Shape > adjustedshapeLookup_;

	bool useProxy_ = false;
	std::list<std::string>* roomBoundingObjects_ = {};
	bool useCustom_ = false;
	bool useCustomFull_ = false;

	std::unordered_set<std::string> openingObjects_  = { "IfcWall", "IfcWallStandardCase", "IfcRoof", "IfcSlab" }; 
	std::unordered_set<std::string> cuttingObjects_  = { "IfcWindow", "IfcDoor", "IfcColumn"}; 

	/// finds the ifc schema that is used in the supplied file
	bool findSchema(const std::string& path, bool quiet = false);

	/// sets the unit multipliers to allow for the use of other units than metres
	void setUnits(IfcParse::IfcFile* file);

	/// count the elements in the file and set the related bools
	void elementCountSummary(bool* hasProxy, bool* hasLotProxy);

	/// compute the lll point, urr point and the rotation related to the apporximated smallest bbox
	void computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint, double* originRot);

	/// compute vector from the lll corner to the originpoint
	void computeObjectTranslation(gp_Vec* vec);

	/// returns a bbox of a ifcproduct that functions with boost
	bg::model::box <BoostPoint3D> makeObjectBox(IfcSchema::IfcProduct* product);
	bg::model::box <BoostPoint3D> makeObjectBox(const std::vector<IfcSchema::IfcProduct*>& products);
	TopoDS_Solid makeSolidBox(const gp_Pnt& lll, const gp_Pnt& urr, double angle, double extraAngle = 0);

	/// check if shape is inside of a wall, floor or roof
	bool isInWall(const bg::model::box <BoostPoint3D>& bbox);

	/// create orientated bbox representing a simplefied shape of the input
	TopoDS_Shape boxSimplefy(const TopoDS_Shape& shape);

	/// adds all instances of an objectype to the spatial index
	template <typename T>
	void addObjectToIndex(const T& object);

	/// get all the points of all the instances of an objecttype
	template <typename T>
	std::vector<gp_Pnt> getAllTypePoints(const T& typePtr);

public:

	/*
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper() {};
	explicit helper(std::string path);

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	// internalises the geometry while approximating a smallest bbox around the geometry
	void internalizeGeo();

	// makes a spatial index for the geometry
	void indexGeo();

	/// gets the generic building information
	std::map<std::string, std::string> getBuildingInformation();
	/// gets the building name
	std::string getBuildingName();
	/// gets the long building name
	std::string getBuildingLongName();
	/// gets the project name
	std::string getProjectName();

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

	std::string getFileName() const { return fileName_; }

	std::string getPath() const { return path_; }

	// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile() const { return file_; }

	// returns a pointer to the kernel
	IfcGeom::Kernel* getKernel() const { return kernel_; }

	bool getHasGeo() { return hasGeo_; }

	double getProxyNum() { return proxyCount_; }

	double getObjectCount() { return objectCount_; }

	bool getHasProxy() { return hasProxy_; }

	bool getHasLotProxy() { return hasLotProxy_; }

	gp_Pnt getLllPoint() { return lllPoint_; }

	gp_Pnt getUrrPoint() { return urrPoint_; }

	double getRotation() { return originRot_; }

	gp_Trsf getObjectTranslation() { return objectTranslation_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }

	void setRoomBoundingObjects(std::list<std::string>* objectList, bool custom, bool customFull) { roomBoundingObjects_ = objectList; useCustom_ = custom; useCustomFull_ = customFull; };

	bool hasIndex() { return hasIndex_; }

	auto getLookup(int i) { return productLookup_[i]; }
	auto updateLookupTriangle(const std::vector<std::vector<gp_Pnt>>& triangleMeshList, int i) { productLookup_[i].getTriangluatedShape() = triangleMeshList; }

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool simple = false);

	std::vector<TopoDS_Face> getObjectFaces(IfcSchema::IfcProduct* product, bool simple = false);

	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false, bool memorize = true);
	void updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted = false);
	void updateIndex(IfcSchema::IfcProduct* product, TopoDS_Shape shape);
	void applyVoids();

	std::vector<std::vector<gp_Pnt>> triangulateProduct(IfcSchema::IfcProduct* product);
	std::vector<std::vector<gp_Pnt>> triangulateShape(const TopoDS_Shape& shape);

	template <typename T>
	void voidShapeAdjust(T products);

	void setPath(const std::string& path) { path_ = path; }

	void setName(const std::string& name) { fileName_ = name; }

	void setUseProxy(bool b) { useProxy_ = b; }

	void setfootprintLvl(double lvl) { footprintEvalLvl_ = lvl; }

	~helper() {};

};

#endif // DATAMANAGER_DATAMANAGER_H