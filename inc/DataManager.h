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

class MeshTriangle
{
private:
	std::vector<gp_Pnt> points_ = {};
public:
	MeshTriangle(const std::vector<gp_Pnt> points) { points_ = points; }
	std::vector<gp_Pnt> getPoints() { return points_; }
};

// lookup for the major spatial index used in the code (indexing all the objects in the ifc file)
class lookupValue
{
private:
	// unique pointer to the product and its data
	std::unique_ptr<IfcSchema::IfcProduct> productPtr_;
	// full shape of the product
	TopoDS_Shape productShape_;
	// simplefied shape of the product (can be empty)
	TopoDS_Shape simpleShape_;

	// spatial index of the triangles constructing the mesh of the object
	bgi::rtree<Value, bgi::rstar<25>> triangleIndex_;
	// vector containing the triangle points related to the index
	std::vector<MeshTriangle> productTrianglePoints_;

	// boolean that signifies if an object is detailed in nature, such as windows and doors
	bool isDetailed_ = false;

public:
	lookupValue(
		IfcSchema::IfcProduct* productPtr,
		const TopoDS_Shape& productShape,
		const TopoDS_Shape& simpleShape);

	~lookupValue() {
	}

	IfcSchema::IfcProduct* getProductPtr() { return productPtr_.get(); }

	const TopoDS_Shape& getProductShape() { return productShape_; }

	void setSimpleShape(const TopoDS_Shape& newShape) { simpleShape_ = newShape; }
	const TopoDS_Shape& getSimpleShape() { return simpleShape_; }

	bgi::rtree<Value, bgi::rstar<25>>* getIndxPointer() { return &triangleIndex_; }
	const std::vector<MeshTriangle>& getProductTriangleList() { return productTrianglePoints_; }

	bool hasSimpleShape() { return !simpleShape_.IsNull(); }
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

	/// create orientated bbox representing a simplefied shape of the input
	TopoDS_Shape boxSimplefy(const TopoDS_Shape& shape);

	/// adds all instances of the template type to the index and reports to user
	template <typename IfcType>
	void timedAddObjectListToIndex(const std::string& typeName, bool addToRoomIndx = false);

	/// adds all instances of the string type to the index and reports to user
	void timedAddObjectListToIndex(const std::string& typeName);

	/// splits the object list over the available threads and adds all instances of an objectype to the spatial index
	template <typename T>
	void addObjectListToIndex(const T& objectList, bool addToRoomIndx = false);

	/// adds all instances of an objecttype to the spatial index
	void addObjectToIndex(IfcSchema::IfcProduct::list::ptr productList, bool addToRoomIndx = false);

	/// adds the product to the spatial index
	void addObjectToIndex(IfcSchema::IfcProduct* product, bool addToRoomIndx = false);

	/// gets shapes from memory without checking for correct adjusted boolean
	int getObjectShapeLocation(IfcSchema::IfcProduct* product);

	/// get the product representation from the object from the kernel
	IfcSchema::IfcRepresentation* getProductRepPtr(IfcSchema::IfcProduct* product);

	// update the lll and urr point
	void updateBoudingData(const bg::model::box <BoostPoint3D>& box);

	/// get object shapes from the objects that are grouped in the current product
	TopoDS_Shape getNestedObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false);

	/// get the kernel which contains the product with the supplied product guid
	IfcGeom::Kernel* getKernelObject(const std::string& productGuid);

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

	template<typename T>
	std::vector<gp_Pnt> getObjectListPoints(const T& productList, bool simple = false);

	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool simple = false);

	TopoDS_Shape getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted);
	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool adjusted = false);
	void updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape);
	void applyVoids();

	std::map<std::string, std::string> getProductPropertySet(const std::string& productGui, int fileNum);

	template <typename T>
	void timedVoidShapeAdjust(const std::string& typeName);

	template <typename T>
	void voidShapeAdjust(T productList);
};

#endif // DATAMANAGER_DATAMANAGER_H