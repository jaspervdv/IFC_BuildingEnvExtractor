#include "helper.h"
#include "settingsCollection.h"

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

#include <mutex>

#ifndef DATAMANAGER_DATAMANAGER_H
#define DATAMANAGER_DATAMANAGER_H

// lookup for the major spatial index used in the code (indexing all the objects in the ifc file)
class lookupValue
{
private:
	std::unique_ptr<IfcSchema::IfcProduct> productPtr_;
	TopoDS_Shape productShape_;
	std::vector<gp_Pnt> productPointList_;
	TopoDS_Shape cBox_;


public:
	lookupValue(
		IfcSchema::IfcProduct* productPtr, 
		const TopoDS_Shape& productShape,
		const TopoDS_Shape& cBox);

	~lookupValue() {
	}

	IfcSchema::IfcProduct* getProductPtr() { return productPtr_.get(); }

	const TopoDS_Shape& getProductShape() { return productShape_; }
	const std::vector<gp_Pnt>& getProductPoints() { return productPointList_; }

	bool hasCBox() { return !cBox_.IsNull(); }

	const TopoDS_Shape& getCBox() { return cBox_; }
};


class fileKernelCollection 
{
private:
	IfcParse::IfcFile* file_; //TODO: find out why memory needs to be leaked
	std::unique_ptr<IfcGeom::Kernel> kernel_;

	bool good_ = false;

	// The unit multipliers found
	double length_ = 0;
	double area_ = 0;
	double volume_ = 0;

public:
	fileKernelCollection(const std::string& file);

	~fileKernelCollection() {
	}

	/// returns the pointer to the file object
	IfcParse::IfcFile* getFilePtr()  { return file_; }

	/// returns the pointer to the kernel object
	IfcGeom::Kernel* getKernelPtr() { return kernel_.get(); }

	/// internalizes the units that are stored in the file
	void setUnits();

	// returns a vector with length, area and volume multipliers
	std::vector<double> getUnits() const { return { length_, area_, volume_ }; }

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	// returns the area multiplier
	double getAreaMultiplier() const { return area_; }

	// returns the volume multiplier
	double getVolumeMultiplier() const { return volume_; }

	bool isGood() { return good_; }
};


class helper
{
private:
	std::shared_ptr<SettingsCollection> sudoSettings_;

	double objectCount_ = 0;

	bool hasGeo_ = false;
	bool isPopulated_ = false;

	double maxProxyP_ = 0.3;
	double proxyCount_ = 0;
	bool hasProxy_ = false;
	bool hasLotProxy_ = false;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// translation needed for accuracy if object is very far away from origin 
	gp_Trsf objectTranslation_;

	// The needed rotation for the model to be aligned to the world axis!
	double originRot_;

	std::vector<std::unique_ptr<fileKernelCollection>> datacollection_;
	int dataCollectionSize_ = 0;

	static const int treeDepth = 5;
	bgi::rtree<Value, bgi::rstar<treeDepth>> index_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> SpaceIndex_;
	std::vector<lookupValue*> productLookup_;
	std::vector<lookupValue*> SpaceLookup_;

	bool hasIndex_ = false;

	std::map <std::string, std::unordered_map < std::string, TopoDS_Shape>> shapeLookup_;
	std::map <std::string, std::unordered_map < std::string, TopoDS_Shape>> adjustedshapeLookup_;

	std::list<std::string>* roomBoundingObjects_ = {};
	bool useCustom_ = false;
	bool useCustomFull_ = false;

	std::unordered_set<std::string> openingObjects_  = { "IfcWall", "IfcWallStandardCase", "IfcRoof", "IfcSlab" };  // read only!
	std::unordered_set<std::string> cuttingObjects_  = { "IfcWindow", "IfcDoor", "IfcColumn"}; // read only!

	/// finds the ifc schema that is used in the supplied file
	bool findSchema(const std::string& path, bool quiet = false);

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
	void addObjectToIndex(const T& object, bool addToRoomIndx = false);

	/// get all the points of all the instances of an objecttype
	template <typename T>
	std::vector<gp_Pnt> getAllTypePoints(const T& typePtr, bool simple = false);

	template<typename T>
	void getAllTypePointsPtr(const T& typePtr, std::vector<gp_Pnt>* pointList, bool simple);

	/// gets shapes from memory without checking for correct adjusted boolean
	TopoDS_Shape getObjectShapeFromMemEmpty(IfcSchema::IfcProduct* product, bool adjusted);

public:
	/*
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit helper() {};
	explicit helper(const std::vector<std::string>& path, std::shared_ptr<SettingsCollection> settings);

	~helper() {
		for (size_t i = 0; i < productLookup_.size(); i++)
		{
			delete productLookup_[i];
		}
	}

	// returns true if helper is well populated
	bool isPopulated() { return isPopulated_; }

	// returns true when length, area and volume multiplier are not 0
	bool hasSetUnits();

	// internalises the geometry while approximating a smallest bbox around the geometry
	void internalizeGeo();

	// makes a spatial index for the geometry
	void indexGeo();

	/// gets the projectiodata if present
	void getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData, gp_Trsf* trsf);
	/// gets the generic building information
	std::map<std::string, std::string> getBuildingInformation();
	/// gets the building name
	std::string getBuildingName();
	/// gets the long building name
	std::string getBuildingLongName();
	/// gets the project name
	std::string getProjectName();

	// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile(int i) const { return datacollection_[i].get()->getFilePtr(); }
	int getSourceFileCount() { return dataCollectionSize_; }

	// returns a pointer to the kernel
	IfcGeom::Kernel* getKernel(int i) const { return datacollection_[i].get()->getKernelPtr(); }

	double getScaler(int i) const { return datacollection_[i].get()->getLengthMultiplier(); }

	bool getHasGeo() { return hasGeo_; }

	double getProxyNum() { return proxyCount_; }

	double getObjectCount() { return objectCount_; }

	bool getHasProxy() { return hasProxy_; }

	bool getHasLotProxy() { return hasLotProxy_; }

	const gp_Pnt& getLllPoint() { return lllPoint_; }

	const gp_Pnt& getUrrPoint() { return urrPoint_; }

	double getRotation() { return originRot_; }

	const gp_Trsf& getObjectTranslation() { return objectTranslation_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getSpaceIndexPointer() { return &SpaceIndex_; }

	void setRoomBoundingObjects(std::list<std::string>* objectList, bool custom, bool customFull) { roomBoundingObjects_ = objectList; useCustom_ = custom; useCustomFull_ = customFull; };

	bool hasIndex() { return hasIndex_; }

	lookupValue* getLookup(int i) { return productLookup_.at(i); }

	lookupValue* getSpaceLookup(int i) { return SpaceLookup_.at(i); }

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool simple = false);

	std::vector<TopoDS_Face> getObjectFaces(IfcSchema::IfcProduct* product, bool simple = false);

	TopoDS_Shape getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted);
	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false, bool memorize = true);
	void updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted = false);
	void applyVoids();

	std::map<std::string, std::string> getProductPropertySet(const std::string& productGui, int fileNum);

	template <typename T>
	void voidShapeAdjust(T products);
};

#endif // DATAMANAGER_DATAMANAGER_H