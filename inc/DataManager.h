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

// container for mesh triangles
class MeshTriangle
{
private:
	/// the three points of the triangle (do not store any other larger vector)
	std::vector<gp_Pnt> points_ = {};
public:
	MeshTriangle(const std::vector<gp_Pnt> points) { points_ = points; }
	/// return the three points of the triangle
	const std::vector<gp_Pnt> getPoints() { return points_; }
};

// lookup for the major spatial index used in the code (indexing all the objects in the ifc file)
class IfcProductSpatialData
{
private:
	// unique pointer to the product and its data
	std::unique_ptr<IfcSchema::IfcProduct> productPtr_;
	// full shape of the product
	TopoDS_Shape productShape_;

	// spatial index of the triangles constructing the mesh of the object
	bgi::rtree<Value, bgi::rstar<25>> triangleIndex_;
	// vector containing the triangle points related to the index
	std::vector<MeshTriangle> productTrianglePoints_;

	// boolean that signifies if an object is detailed in nature, such as windows and doors
	bool isDetailed_ = false;

public:
	IfcProductSpatialData(
		IfcSchema::IfcProduct* productPtr,
		const TopoDS_Shape& productShape);

	~IfcProductSpatialData() {
	}
	/// returns the pointer of the product 
	IfcSchema::IfcProduct* getProductPtr() { return productPtr_.get(); }
	/// returns the shape of the product
	const TopoDS_Shape& getProductShape() { return productShape_;  }
	/// replaces or sets the stored shape of the product
	void setProductShape(const TopoDS_Shape& newShape) { productShape_ = newShape; }
	/// returns the index for the triangulated shape
	bgi::rtree<Value, bgi::rstar<25>>* getIndxPointer() { return &triangleIndex_; }
	/// returns the triangulated shape vector that coorperates with the index
	const std::vector<MeshTriangle>& getProductTriangleList() { return productTrianglePoints_; }
};

class fileKernelCollection 
{
private:
	IfcParse::IfcFile* file_; //TODO: find out why memory needs to be leaked
	std::unique_ptr<IfcGeom::Kernel> kernel_;

	// The unit multipliers found
	double length_ = 0;

	double getSiPrefixValue(const IfcSchema::IfcSIUnit& unitItem);
	double getSiScaleValue(const IfcSchema::IfcSIUnit& unitItem);

public:
	fileKernelCollection(const std::string& file);

	~fileKernelCollection() {
	}

	/// returns the pointer to the file object
	IfcParse::IfcFile* getFilePtr()  { return file_; }
	/// returns the pointer to the kernel object
	IfcGeom::Kernel* getKernelPtr() { return kernel_.get(); }
	/// returns the length multiplier
	double getLengthMultiplier() const { return length_; }
	/// returns if the file object is good (functioning)
	bool isGood() { return file_->good(); }

	/// internalizes the units that are stored in the file
	void setUnits();
};

/// <summary>
/// Manages the IFC file collection
/// </summary>
class DataManager
{
private:
	bool isPopulated_ = false;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// translation needed for accuracy if object is very far away from origin 
	gp_Trsf objectTranslation_;
	// translation that is adding the object translation to the ifc georef translation
	gp_Trsf objectIfcTranslation_;

	std::vector<std::unique_ptr<fileKernelCollection>> datacollection_;
	int dataCollectionSize_ = 0;

	static const int treeDepth = 5;
	
	std::shared_mutex indexMutex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> index_;
	std::vector<std::shared_ptr<IfcProductSpatialData>> productLookup_;

	std::mutex spaceIndexMutex_;
	bgi::rtree<Value, bgi::rstar<treeDepth>> spaceIndex_;
	std::vector<std::shared_ptr<IfcProductSpatialData>> SpaceLookup_;

	std::mutex convertMutex_;

	std::map <std::string, std::unordered_map < std::string, int >> productIndxLookup_;

	std::unordered_map<std::string, std::vector<IfcSchema::IfcPropertySet*>> attributeLookup_;

	/// finds the ifc schema that is used in the supplied file
	bool findSchema(const std::string& path, bool quiet = false);
	/// count the elements in the file and set the related bools
	void elementCountSummary();
	/// compute the inital lll point, urr point and the rotation related to the apporximated smallest bbox around ino type of object
	void computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint);
	/// compute vector from the lll corner to the originpoint based on the first object it can find (prefers slab objects)
	gp_Vec computeObjectTranslation();
	/// compute vector from the lll corner to the originpoint based on the first object of the input type
	gp_Vec computeObjectTranslation(const std::string& objectType);

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
	
	/// get the kernel which contains the product with the supplied product guid
	IfcGeom::Kernel* getKernelObject(const std::string& productGuid);
	/// gets shapes from memory without checking for correct adjusted boolean
	int getObjectShapeLocation(IfcSchema::IfcProduct* product);

	/// get the product representation from the object from the kernel
	IfcSchema::IfcRepresentation* getProductRepPtr(IfcSchema::IfcProduct* product);
	/// get the products that are nested inside of the main product
	IfcSchema::IfcProduct::list::ptr getNestedProductList(IfcSchema::IfcProduct* product);
	/// get flat pointlist of the input product list
	template<typename T>
	std::vector<gp_Pnt> getObjectListPoints(bool simple = false);
	/// get flat pointlist of the input product 
	std::vector<gp_Pnt> getObjectPoints(IfcSchema::IfcProduct* product, bool simple = false);
	/// replace the simple shape of a spatial data object in memory
	void updateShapeMemory(IfcSchema::IfcProduct* product, TopoDS_Shape shape);
	
	/// apply voids to opening objects
	void applyVoids();
	/// applies the voids to all of the template type to the index and reports to user
	template <typename T>
	void timedVoidShapeAdjust(const std::string& typeName);
	/// applies the voids to all of the template type to the index
	template <typename T>
	void voidShapeAdjust(T productList);
	/// returns the voids that have no objects placed inside of it
	std::vector<TopoDS_Shape> computeEmptyVoids(IfcSchema::IfcRelVoidsElement::list::ptr voidElementList);
	/// returns a shape which is the voidobjectlist voids applied to the untrimmed input shape
	TopoDS_Shape applyVoidtoShape(const TopoDS_Shape& untrimmedShape, std::vector<TopoDS_Shape>& voidObjectList);

	// update the lll and urr point
	void updateBoudingData(const bg::model::box <BoostPoint3D>& box);

	// check if the site contains all the required data for georeferencing according to IfcGRef (ifc2x3) only
	bool validateProjectionData(const nlohmann::json& sitePropertySetData);

	// populate a map that has all the guid related propertysets 
	void populateAttributeLookup();

public:
	/*
	construct and populate a helper
	creates and stores SI unit mulitpliers for length, area and volume
	creates and stores the file and kernel for quick acess
	*/
	explicit DataManager() {};
	explicit DataManager(const std::vector<std::string>& path);

	/// returns true if helper is well populated
	bool isPopulated() { return isPopulated_; }
	/// returns true when length, area and volume multiplier of the internal geo ifc kernels are not 0
	bool hasSetUnits();
	/// returns a pointer to the sourcefile
	IfcParse::IfcFile* getSourceFile(int i) const { return datacollection_[i].get()->getFilePtr(); }
	/// returns a vector of pointers to the sourcefiles
	std::vector<IfcParse::IfcFile*> getSourceFiles() const;
	/// get the total amount of items in the datacollection
	int getSourceFileCount() { return dataCollectionSize_; }
	/// get the length multiplier of a sourcefile
	double getScaler(int i) const { return datacollection_[i].get()->getLengthMultiplier(); }
	/// get the lll point of the ifc bbox
	const gp_Pnt& getLllPoint() { return lllPoint_; }
	/// get the urr point of the ifc bbox
	const gp_Pnt& getUrrPoint() { return urrPoint_; }
	/// get the translation of the ifc model
	const gp_Trsf& getObjectTranslation() { return objectTranslation_; }
	/// get the translation of the ifc + georeference model
	const gp_Trsf& getObjectNormalizedTranslation() { return objectIfcTranslation_; }
	/// get the pointer to the space dividing objects index
	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getIndexPointer() { return &index_; }
	/// get the pointer to the space objects index
	const bgi::rtree<Value, bgi::rstar<treeDepth>>* getSpaceIndexPointer() { return &spaceIndex_; }
	/// get the spatial/product data related to the space dividing objects index
	std::shared_ptr<IfcProductSpatialData> getLookup(int i) { return productLookup_.at(i); }
	/// get the spatial/product data  related to the space objects index
	std::shared_ptr<IfcProductSpatialData> getSpaceLookup(int i) { return SpaceLookup_.at(i); }

	// internalises the geometry while approximating a smallest bbox around the geometry
	void internalizeGeo();
	// makes a spatial index for the geometry
	void indexGeo();

	/// gets the projectiodata if present
	gp_Trsf getProjectionTransformation();
	/// gets the projectiodata if present
	void getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData);
	/// gets the generic building information
	nlohmann::json getBuildingInformation();
	/// gets the object list name or long name
	template <typename T>
	std::string getIfcObjectName(const std::string& objectTypeName, bool isLong);
	/// gets the object name or long name
	template <typename T>
	std::string getIfcObjectName(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong);

	/// collects the non-standard property data in the ifc file of an object 
	nlohmann::json collectPropertyValues(const std::string& objectId, const std::string& psetName = "");
	/// collects the non-standard property data in the ifc file of an object 
	nlohmann::json collectPropertyValues(const std::string& objectId, int location, const std::string& psetName = "");
	/// collects the non-standard property data in the ifc file of an object 
	nlohmann::json collectPropertyValues(const std::string& objectId, IfcParse::IfcFile* ifcFile, const std::string& psetName = "");

	/// search the object shape from memory only
	TopoDS_Shape getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool isSimple);

	/// get the shape of an ifcproduct
	TopoDS_Shape getObjectShape(IfcSchema::IfcProduct* product, bool getNested = false, bool isSimple = false, bool fromMemOnly = false);

	/// get the shape of an ifcproduct

};

#endif // DATAMANAGER_DATAMANAGER_H