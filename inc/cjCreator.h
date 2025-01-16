#include "helper.h"
#include "DataManager.h"
#include "settingsCollection.h"
#include "voxel.h"
#include "voxelGrid.h"

#include <chrono>
#include <CJToKernel.h>

#include <TopoDS.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>


#ifndef CJGEOCREATOR_CJGEOCREATOR_H
#define CJGEOCREATOR_CJGEOCREATOR_H

class FloorOutlineObject {
private:
	std::vector<TopoDS_Face> outlineList_;
	std::map<std::string, std::string> semanticInformation_;
	std::string ifc_guid;
public:
	FloorOutlineObject(const std::vector<TopoDS_Face>& outlineList, const std::map<std::string, std::string>& semanticInformation, const std::string& guid);
	FloorOutlineObject(const TopoDS_Face& outlineList, const std::map<std::string, std::string>& semanticInformationconst, const std::string& guid );

	std::vector<TopoDS_Face> getOutlines() { return outlineList_; }
	std::map<std::string, std::string> getSemanticInfo() { return semanticInformation_; }
};


class CJGeoCreator {
private:
	typedef std::pair<BoostBox3D, TopoDS_Face> BoxFacePair;

	// default spatial index tree depth
	static const int treeDepth_ = 25;
	std::shared_ptr<VoxelGrid> voxelGrid_ = nullptr;

	// container for surface group data
	std::vector<std::vector<RCollection>> faceList_;
	std::mutex faceListMutex_;

	// flags representing the eval state of the creator
	bool hasTopFaces_ = false;
	bool hasFootprints_ = false;
	bool hasStoreyPrints_ = false;
	bool useRoofprints_ = false;
	bool hasGeoBase_ = false;

	// if true the roofoutlines are used to create the geometry
	bool useRoofOutline_ = true;

	/// mutex that keeps the injection of objects into the rtree in check
	std::mutex indexInjectMutex_;

	// list containing all the roof outlines
	std::vector<TopoDS_Face> roofOutlineList_; 

	// list containing all the footprints
	std::vector<TopoDS_Face> footprintList_;

	// list containing all the floor outlines;
	std::vector<FloorOutlineObject> storeyPrintList_; //TODO: remove

	// returns true if the area and normal z dir are significant for the evaluation
	bool useFace(const TopoDS_Face& face, gp_Pnt* centerPoint = nullptr);

	/// get top objects by projecting the top voxel grid in beams downwards
	std::vector<TopoDS_Shape> beamProjection(DataManager* h);

	// culls dub shapes based on mass and area
	std::vector<TopoDS_Shape> getUniqueShapedObjects (const std::vector<TopoDS_Shape>& topObjectList);

	/// @brief get the top geometry objects of the model
	std::vector<TopoDS_Shape> getTopObjects(DataManager* h);

	std::vector<TopoDS_Face> createRoofOutline();

	/// creates face collection that represent the merged input shapes
	std::vector<TopoDS_Face> planarFaces2Outline(const std::vector<TopoDS_Face>& planarFaces, const TopoDS_Face& boundingFace);

	/// @brief reduce the surfaces of an object for roof extraction by z-ray casting on itself
	void reduceSurfaces(
		const std::vector<TopoDS_Shape>& inputShapes, 
		bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, 
		std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList
	);
	void reduceSurface(
		const std::vector<TopoDS_Shape>& inputShapes, 
		bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, 
		std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList
	);

	/// @brief reduce the surfaces in the facelist for roof extraction by z-ray casting on itself and others
	void FinefilterSurfaces(
		const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList, 
		std::vector<std::shared_ptr<SurfaceGridPair>>* fineFilteredShapeList
	);
	void FinefilterSurface(
		const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList, 
		const std::vector<std::shared_ptr<SurfaceGridPair>>& otherShapeList, 
		std::vector<std::shared_ptr<SurfaceGridPair>>* fineFilteredShapeList
	);

	/// @brief get the surfaces that are not covered by other surfaces within the objects 
	std::vector<std::shared_ptr<SurfaceGridPair>> getObjectTopSurfaces(const TopoDS_Shape& shape);

	/// get the unique edges from in a shape or a collection of edges
	std::vector<Edge> getUniqueEdges(const TopoDS_Shape& flattenedEdges);
	std::vector<Edge> getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges);

	/// get faces that are uniquely in a list dublicates are ingnored
	std::vector<TopoDS_Face> getUniqueFaces(const std::vector<TopoDS_Face>& faceList);

	/// get all the faces that have all vertices shared with one or more other faces from the list
	std::vector<TopoDS_Face> getEncompassedFaces(const std::vector<TopoDS_Face>& faceList);

	/// check if edge is in list of edge objects
	bool isInList(const TopoDS_Edge& currentEdge, const std::vector<Edge>& edgeList);

	/// @bried merges all the overlapping edges that have the same direction
	std::vector<Edge> mergeOverlappingEdges(const std::vector<Edge>& uniqueEdges, bool project=true);

	/// @brief merges flat faces together in a singular shape
	std::vector<TopoDS_Face> simplefyProjection(const std::vector<TopoDS_Face> inputFaceList);

	/// @brief check if edge is part of outer envelope
	bool isOuterEdge(Edge currentEdge, const std::vector<TopoDS_Face>& flatFaceList, const bgi::rtree<Value, bgi::rstar<treeDepth_>>& spatialIndex);

	/// @brief get all the edges that enclose the projected faces
	std::vector<TopoDS_Edge> getOuterEdges(const std::vector<Edge>& edgeList, const std::vector<RCollection>& faceList);

	/// @brief get all the outer edges based on a voxelplate
	std::vector<TopoDS_Edge> getOuterEdges(
		const std::vector<Edge>& edgeList,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<std::shared_ptr<voxel>>& originVoxels,
		double floorlvl);

	// divides the projected footprints over the seperate buildings
	void sortRoofStructures();

	// merges faces that are near eachother
	void mergeRoofSurfaces(std::vector<std::shared_ptr<SurfaceGridPair>>& Collection);


	// create list of edges by cutting objects at the floor lvl
	std::vector<TopoDS_Face> section2Faces(const std::vector<Value>& productLookupValues, DataManager* h, double cutlvl);

	// extrudes shape downwards and caps it on the splitting face
	TopoDS_Solid extrudeFace(const TopoDS_Face& evalFace, bool downwards,  double splittingFaceHeight = 0);
	TopoDS_Solid extrudeFace(const std::vector<TopoDS_Face>& faceList, const std::vector<TopoDS_Wire>& wireList, bool downwards,  double splittingFaceHeight = 0);
	/// splits the surfaces with extruded solid copies and returns the ones visible from the top
	std::vector<TopoDS_Face> getSplitTopFaces(
		const std::vector<TopoDS_Face>& inputFaceList,
		double lowestZ, 
		const TopoDS_Face& bufferSurface = {}
	);
	/// splits the surfaces with extruded solid copies
	std::vector<TopoDS_Face> getSplitFaces(const std::vector<TopoDS_Face>& inputFaceList,
		const std::vector<TopoDS_Solid>& ExtrudedShapes,
		const bgi::rtree<Value, bgi::rstar<treeDepth_>>& spatialIndex
	);
	/// returns visible surfaces from the top
	std::vector<TopoDS_Face> getVisTopSurfaces(const std::vector<TopoDS_Face>& faceIdx, double lowestZ, const TopoDS_Face& bufferSurface = {});

	/// create a solid extrusion from the projected roofoutline
	std::vector<TopoDS_Shape> computePrisms(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ, bool preFilter = true, const TopoDS_Face& bufferSurface = {});

	/// remove redundant edges from a solid shape
	TopoDS_Shape simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap = false);

	/// attempts to merge faces into one big face
	TopoDS_Face mergeFaces(const std::vector<TopoDS_Face>& mergeFaces);

	/// outputs the time delta between the start and end time
	void printTime(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime);

	/// checks if surface is encapsulated by another shape
	bool surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape);

	/// checks if ray intersects shape
	bool checkShapeIntersection(const TopoDS_Edge& ray, const TopoDS_Shape& shape);
	bool checksurfaceIntersection(const TopoDS_Edge& ray, const TopoDS_Face& face);

	/// create spatial index for voxels and lookup
	void populateVoxelIndex(
		bgi::rtree<Value, bgi::rstar<25>>* voxelIndex, 
		std::vector<std::shared_ptr<voxel>>* originVoxels,
		const std::vector<std::shared_ptr<voxel>> exteriorVoxels
	);

	/// remove objects that are completely encapsulated by other objects
	void filterEncapsulatedObjects(std::vector<Value>* productLookupValues, bgi::rtree<Value, bgi::rstar<25>>* exteriorProductIndex, DataManager* h);

	/// remove dublicate values from valueList
	std::vector<Value> makeUniqueValueList(const std::vector<Value>& valueList);

	//gets the unqiue products that intersect with the voxelList
	std::vector<Value> getUniqueProductValues(std::vector<std::shared_ptr<voxel>> voxelList);

	// generates default CJ object that represents a room, used for voxel rooms if no room objects are present
	std::shared_ptr<CJT::CityObject> createDefaultRoomObject(std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int roomNum, double lowestZ);

	void createSemanticData(CJT::GeoObject* geoObject, const TopoDS_Shape& geometryShape, bool isExterior = true);

	void populateSurfaceData(CJT::GeoObject* geoObject, const std::vector<int>& SurfaceIndxDataList, bool isExterior = true);

public:
	explicit CJGeoCreator(DataManager* h, double vSize);

	/// computes and internalizes the data that is required to do the basic city scale output
	void initializeBasic(DataManager* h);

	/// computes and internalizes the data that is required to do footprint related city scale output
	void makeFootprint(DataManager* h);

	void makeFloorSectionCollection(DataManager* h);
	std::vector<TopoDS_Face> makeFloorSection(DataManager* h, double sectionHeight);

	/// store the roofoutline data to LoD 02
	void useRoofPrint(bool b) { useRoofprints_ = b; }

	/// generates the empty storey objects for the ifc storeys in a file
	std::vector<std::shared_ptr<CJT::CityObject>> makeStoreyObjects(DataManager* h);

	/// generates the empty room objects for the ifc storeys in a file
	std::vector<std::shared_ptr<CJT::CityObject>> makeRoomObjects(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& cityStoreyObjects);

	/// generates an LoD0.0 object
	CJT::GeoObject makeLoD00(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.2 objects
	std::vector< CJT::GeoObject>  makeLoD02(DataManager* h, CJT::Kernel* kernel, int unitScale);
	void makeLoD02Storeys(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale);
	void makeSimpleLodRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	/// generates a list of LoD0.3 objects
	std::vector<std::vector<TopoDS_Face>> makeLoD03Faces(DataManager* h, CJT::Kernel* kernel, int unitScale, bool footprintBased = false);
	std::vector< CJT::GeoObject> makeLoD03(DataManager* h, std::vector<std::vector<TopoDS_Face>>* lod03FaceList, CJT::Kernel* kernel, int unitScale);
	/// generates an LoD1.0 object
	CJT::GeoObject makeLoD10(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.2 objects
	std::vector< CJT::GeoObject> makeLoD12(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.3 objects
	std::vector< CJT::GeoObject> makeLoD13(DataManager* h, const std::vector<std::vector<TopoDS_Face>>& roofList03, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD2.2 objects
	std::vector< CJT::GeoObject> makeLoD22(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.2 objects
	std::vector< CJT::GeoObject> makeLoD32(DataManager* h, CJT::Kernel* kernel, int unitScale);
	void makeComplexLoDRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	/// generates a list of voxelized objects
	std::vector< CJT::GeoObject> makeV(DataManager* h, CJT::Kernel* kernel, int unitScale);
	void makeVRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	/// generates a list of the site and its outline
	std::vector<CJT::CityObject> makeSite(DataManager* h, CJT::Kernel* kernel, int unitScale);

	// creates roomshapes 
	TopoDS_Shape voxels2Shape(int roomNum);
	// creates exterior shapes
	TopoDS_Shape voxels2ExtriorShape(int buildingNum);

	// fetches the room related data from the ifc model
	std::vector < std::shared_ptr<CJT::CityObject >> fetchRoomObject(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int roomNum);

	// approximate the area of a room base on the voxelshape (Only works with full voxelization)
	double approximateRoomArea(int roomNum);
	void processDirectionalFaces(int direction, int roomNum, std::vector<TopoDS_Face>& collectionList);

	/// computes data related to the voxel shape such as volume and shell area
	void extractOuterVoxelSummary(CJT::CityObject* shellObject, DataManager* h, double footprintHeight, double geoRot);
	void extractInnerVoxelSummary(CJT::CityObject* shellObject, DataManager* h);

	void addFullSurfaceDict(CJT::GeoObject* geoObject);

};
#endif // CJGEOCREATOR_CJGEOCREATOR_H

