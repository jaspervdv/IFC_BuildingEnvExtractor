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
#include <ifcgeom_schema_agnostic/Kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>
#include <ifcparse/IfcHierarchyHelper.h>

// OpenCascade includes
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>
#include <TopoDS.hxx>

#include <shared_mutex> 
#include <mutex> 

#ifndef DATAMANAGER_DATAMANAGER_H
#define DATAMANAGER_DATAMANAGER_H

// lookup for the major spatial index used in the code (indexing all the objects in the ifc file)
class lookupValue
{
private:
	std::unique_ptr<IfcSchema::IfcProduct> productPtr_;
	TopoDS_Shape productShape_;
	TopoDS_Shape simpleShape_;

	bgi::rtree<Value, bgi::rstar<25>> spatialIndex_;
	std::vector<std::vector<gp_Pnt>> productTrianglePoints_;

	std::vector<gp_Pnt> productPointList_;
	TopoDS_Shape cBox_;

public:
	lookupValue(
		IfcSchema::IfcProduct* productPtr, 
		const TopoDS_Shape& productShape,
		const TopoDS_Shape& simpleShape,
		const TopoDS_Shape& cBox);

	~lookupValue() {
	}

	IfcSchema::IfcProduct* getProductPtr() { return productPtr_.get(); }

	const TopoDS_Shape& getProductShape() { return productShape_; }
	const TopoDS_Shape& getSimpleShape() { return simpleShape_; }
	const std::vector<gp_Pnt>& getProductPoints() { return productPointList_; }

	bgi::rtree<Value, bgi::rstar<25>>* getIndxPointer() { return &spatialIndex_; }
	const std::vector<std::vector<gp_Pnt>> & getProductTriangleList() { return productTrianglePoints_; }
	const std::vector<gp_Pnt>& getProductTriangle(int i) { return productTrianglePoints_[i]; }

	bool hasCBox() { return !cBox_.IsNull(); }

	const TopoDS_Shape& getCBox() { return cBox_; }

	void setSimpleShape(const TopoDS_Shape& newShape) { simpleShape_ = newShape; }
};


class fileKernelCollection 
{
private:
	IfcParse::IfcFile* file_; //TODO: find out why memory needs to be leaked
	std::unique_ptr<IfcGeom::Kernel> kernel_;

	bool good_ = false;

	// The unit multipliers found
	double length_ = 0;

public:
	fileKernelCollection(const std::string& file);

	~fileKernelCollection() {
	}

	double getSiPrefixValue(const IfcSchema::IfcSIUnit& unitItem);
	double getSiScaleValue(const IfcSchema::IfcSIUnit& unitItem);

	/// returns the pointer to the file object
	IfcParse::IfcFile* getFilePtr()  { return file_; }

	/// returns the pointer to the kernel object
	IfcGeom::Kernel* getKernelPtr() { return kernel_.get(); }

	/// internalizes the units that are stored in the file
	void setUnits();

	// returns the length multiplier
	double getLengthMultiplier() const { return length_; }

	bool isGood() { return good_; }
};

/// <summary>
/// Manages the IFC file collection
/// </summary>
class DataManager
{
private:
	bool hasGeo_ = false;
	bool isPopulated_ = false;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// translation needed for accuracy if object is very far away from origin 
	gp_Trsf objectTranslation_;

	std::vector<std::unique_ptr<fileKernelCollection>> datacollection_;
	int dataCollectionSize_ = 0;

	static const int treeDepth = 5;
	
	std::shared_mutex indexMutex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> index_;
	std::vector<std::shared_ptr<lookupValue>> productLookup_;

	std::mutex spaceIndexMutex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> spaceIndex_;
	std::vector<std::shared_ptr<lookupValue>> SpaceLookup_;

	std::mutex convertMutex_;

	bool hasIndex_ = false;
	std::map <std::string, std::unordered_map < std::string, int >> productIndxLookup_;

	/// finds the ifc schema that is used in the supplied file
	bool findSchema(const std::string& path, bool quiet = false);

	/// count the elements in the file and set the related bools
	void elementCountSummary();

	/// compute the inital lll point, urr point and the rotation related to the apporximated smallest bbox around ino type of object
	void computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint);

	/// compute vector from the lll corner to the originpoint based on the first slab in the ifc slab list
	void computeObjectTranslation(gp_Vec* vec);

	/// returns a bbox of a ifcproduct that functions with boost
	bg::model::box <BoostPoint3D> makeObjectBox(const TopoDS_Shape& productShape, const double& rotationAngle);

	/// check if shape is inside of a wall, floor or roof
	bool isInWall(const bg::model::box <BoostPoint3D>& bbox);

	/// create orientated bbox representing a simplefied shape of the input
	TopoDS_Shape boxSimplefy(const TopoDS_Shape& shape);

	/// adds all instances of an objectype to the spatial index
	template <typename T>
	void addObjectListToIndex(const T& objectList, bool addToRoomIndx = false);
	
	template <typename T>
	void addObjectToIndex(const T& productList, bool addToRoomIndx = false);
	void addObjectToIndex(IfcSchema::IfcProduct* product, bool addToRoomIndx = false);

	/// get all the points of all the instances of an objecttype
	template <typename T>
	std::vector<gp_Pnt> getAllTypePoints(const T& typePtr, bool simple = false);

	/// gets shapes from memory without checking for correct adjusted boolean
	int getObjectShapeLocation(IfcSchema::IfcProduct* product);

	// update the lll and urr point
	void updateBoudingData(const bg::model::box <BoostPoint3D>& box);

public:
	/*
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit DataManager() {};
	explicit DataManager(const std::vector<std::string>& path);

	// returns true if helper is well populated
	bool isPopulated() { return isPopulated_; }

	// returns true when length, area and volume multiplier of the internal geo ifc kernels are not 0
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

	const gp_Pnt& getLllPoint() { return lllPoint_; }

	const gp_Pnt& getUrrPoint() { return urrPoint_; }

	const gp_Trsf& getObjectTranslation() { return objectTranslation_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }

	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getSpaceIndexPointer() { return &spaceIndex_; }

	bool hasIndex() { return hasIndex_; }

	std::shared_ptr<lookupValue> getLookup(int i) { return productLookup_.at(i); }

	std::shared_ptr<lookupValue> getSpaceLookup(int i) { return SpaceLookup_.at(i); }

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool simple = false);

	TopoDS_Shape getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted);
	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false, int memoryLocation = -1);
	void updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape);
	void applyVoids();

	std::map<std::string, std::string> getProductPropertySet(const std::string& productGui, int fileNum);

	template <typename T>
	void voidShapeAdjust(T products);
};

#endif // DATAMANAGER_DATAMANAGER_H