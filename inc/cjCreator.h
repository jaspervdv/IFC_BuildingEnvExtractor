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

class CJGeoCreator {
private:
	typedef std::pair<BoostBox3D, TopoDS_Face> BoxFacePair;

	// container with source data required for geo creation of a single building
	class BuildingSurfaceCollection {
	private:
		// roof collection list
		std::vector<RCollection> roofFacesRCollection_;
		// the roof outline in a single surface at z=0
		TopoDS_Face roofOutline_;
		// the footprint in a single surface at z=0
		TopoDS_Face footPrint_;

	public:
		void addRoof(RCollection theRoof) { roofFacesRCollection_.emplace_back(theRoof); }
		const std::vector<RCollection> getRoof() const { return roofFacesRCollection_; }
		void setRoof(std::vector<RCollection> theRoofList) { roofFacesRCollection_ = theRoofList; }
		void setRoofOutline(TopoDS_Face theOutline) { roofOutline_ = theOutline; }
		const TopoDS_Face getRoofOutline() const { return roofOutline_; }
		void setFootPrint(TopoDS_Face theFootPrint) { footPrint_ = theFootPrint; }
		const TopoDS_Face getFootPrint() const { return footPrint_; }
	};

	// building geo data per building
	std::vector<BuildingSurfaceCollection> buildingSurfaceDataList_;

	// default spatial index tree depth
	static const int treeDepth_ = 25;
	std::shared_ptr<VoxelGrid> voxelGrid_ = nullptr;

	// flags representing the eval state of the creator
	bool hasTopFaces_ = false;
	bool hasFootprints_ = false;
	bool hasGeoBase_ = false;

	// surface collecting code

	// list collects the faces from the LoD03 creation to base LoD13 output on
	std::vector<std::vector<TopoDS_Face>> LoD03RoofFaces_;
	// list collects the faces from the LoD04 creation to base LoD22 output on
	std::vector<std::vector<TopoDS_Face>> LoD04RoofFaces_;
	// list collects the lod02 plates per lvl TODO: make grouped
	std::map<double, std::vector<TopoDS_Face>> LoD02Plates_;
	// check if the surfaces that are stored can be discarded.
	void garbageCollection();

	// grouping original geo code

	/// fetches all the footprint surfaces from the buildingSurfaceDataList
	std::vector<TopoDS_Face> getFootPrintList();
	/// fetches all the roofOutline surfaces from the buildingSurfaceDataList
	std::vector<TopoDS_Face> getFootRoofOutlineList();
	/// checks if all the buildingSurfaceDataList entries have a valid footprint
	bool hasFootprints();
	/// checks if all the buildingSurfaceDataList entries have a valid roofoutlines
	bool hasRoofOutlines();

	// group surfaces that are placed next or on top of eachother
	std::vector<FaceComplex> groupFaces(const std::vector<TopoDS_Face>& inputFaceList);
	// divides the projected footprints over the seperate buildings
	std::vector<BuildingSurfaceCollection> sortRoofStructures(const std::vector<TopoDS_Face>& roofOutlines, const std::vector<RCollection>& rCollectionList);

	// returns true if the area and normal z dir are significant for the evaluation
	bool useFace(const TopoDS_Face& face, gp_Pnt* centerPoint = nullptr);

	// roof surface detection and cleaning code

	/// @brief get the top geometry objects of the model
	std::vector<TopoDS_Shape> getTopObjects(DataManager* h);
	/// get top objects by projecting the top voxel grid in beams downwards
	std::vector<TopoDS_Shape> beamProjection(DataManager* h);
	// culls dub shapes based on mass and area
	std::vector<TopoDS_Shape> getUniqueShapedObjects(const std::vector<TopoDS_Shape>& topObjectList); //TODO: move to helper?
	/// @brief reduce the surfaces of an object for roof extraction by z-ray casting on itself
	void reduceSurfaces(
		const std::vector<TopoDS_Shape>& inputShapes,
		bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx,
		std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList
	);
	/// @brief reduce the surfaces of an object for roof extraction by z-ray casting on itself (sublist)
	void reduceSurface(
		const std::vector<TopoDS_Shape>& inputShapes,
		std::mutex& processMutex,
		bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx,
		std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList
	);
	/// @brief reduce the surfaces in the facelist for roof extraction by z-ray casting on itself and others
	std::vector<std::shared_ptr<SurfaceGridPair>> FinefilterSurfaces(const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList);

	void FinefilterSurface(
		const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList,
		const std::vector<std::shared_ptr<SurfaceGridPair>>& otherShapeList,
		std::mutex& processMutex,
		std::vector<std::shared_ptr<SurfaceGridPair>>* fineFilteredShapeList
	);
	/// @brief get the surfaces that are not covered by other surfaces within the objects 
	std::vector<std::shared_ptr<SurfaceGridPair>> getObjectTopSurfaces(const TopoDS_Shape& shape);
	// merges roof representations that are near eachother
	std::vector<RCollection> mergeRoofSurfaces(std::vector<std::shared_ptr<SurfaceGridPair>>& Collection);
	/// returns visible surfaces from the top
	std::vector<TopoDS_Face> getVisTopSurfaces(const std::vector<TopoDS_Face>& faceIdx, double lowestZ, const TopoDS_Face& bufferSurface = {});
	// trims the roof surfaces to the underlying footprint
	std::vector<TopoDS_Face> trimFacesToFootprint(const std::vector<TopoDS_Face>& roofFaces, const TopoDS_Face& footprintFace);
	// splits the roof surfaces to the underlying footprint
	void splitFacesToFootprint(std::vector<TopoDS_Face>& outRoofFaces, std::vector<TopoDS_Face>& outOverhangFaces, const std::vector<TopoDS_Face>& roofFaces, const TopoDS_Face& footprintFace);

	// storey generation code

	/// adds a storey object to the city object
	void make2DStorey(std::mutex& storeyMutex, DataManager* h, CJT::Kernel* kernel, const std::shared_ptr<CJT::CityObject>& storeyCityObject, std::map<std::string, int>& progressMap, int unitScale, bool is03);
	/// give user updates about the state of storey creation
	void monitorStoreys(std::mutex& storeyMutex, std::map<std::string, int>& progressMap, int totalProcesses);

	/// create the roof outline at z=0 based on the flattened roof surface representations
	std::vector<TopoDS_Face> createRoofOutline(const std::vector<RCollection>& rCollectionList);

	void makeFloorSection(std::vector<TopoDS_Face>& facesOut, DataManager* h, double sectionHeight);

	void makeFloorSectionComplex(std::vector<TopoDS_Face>& intFacesOut, std::vector<TopoDS_Face>& extFacesOut, DataManager* h, double sectionHeight, const std::vector<IfcSchema::IfcBuildingStorey*>& buildingStoreyObjectList);

	// create list of faces by cutting objects at the floor lvl
	std::vector<TopoDS_Face> section2Faces(const std::vector<Value>& productLookupValues, DataManager* h, double cutlvl);
	// create list of faces by cutting objects at the floor lvl
	template <typename T>
	std::vector<TopoDS_Face> section2Faces(const std::vector<T>& shapes, double cutlvl);
	// split the input faces list over interior and exterior horizontal faces based on the voxels
	void SplitInAndOuterHFaces(const std::vector<TopoDS_Face>& inputFaces, std::vector<TopoDS_Face>& innerFaces, std::vector<TopoDS_Face>& outerFaces);
	// split the input shape over interior and exterior horizontal faces based on the voxels
	void SplitInAndOuterHFaces(const TopoDS_Shape& inputFaces, std::vector<TopoDS_Face>& innerFaces, std::vector<TopoDS_Face>& outerFaces);
	
	// planar simplification code

	/// creates face collection for the interior and exterior that represent the merged input shapes
	void planarFaces2OutlineComplex(std::vector<TopoDS_Face>& intFacesOut, std::vector<TopoDS_Face>& extFacesOut, const std::vector<TopoDS_Face>& planarFaces, const TopoDS_Face& boundingFace, bool filterExternal = false);

	// face extruding code
	
	/// create a solid extrusion from the projected roofoutline
	std::vector<TopoDS_Shape> computePrisms(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ, bool preFilter = true, const TopoDS_Face& bufferSurface = {});
	// extrudes shape downwards and caps it on the splitting face
	TopoDS_Solid extrudeFace(const TopoDS_Face& evalFace, bool downwards,  double splittingFaceHeight = 0);
	/// splits the surfaces with extruded solid copies and returns the ones visible from the top
	std::vector<TopoDS_Face> getSplitTopFaces(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ, const TopoDS_Face& bufferSurface = {});
	/// splits the surfaces with extruded solid copies
	std::vector<TopoDS_Face> getSplitFaces(
		const std::vector<TopoDS_Face>& inputFaceList,
		bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx
	);


	// generic shape simplification code

	/// remove redundant edges and faces from a solid shape
	TopoDS_Shape simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap = false);
	/// remove redundant edges and faces from a group of faces
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap = false);
	/// remove redundant edges and faces from a group of faces assisted with the normal direction of the faces
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap = false);
	/// creates an index populated with unique faces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>> indexUniqueFaces(const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>>& faceIndx);
	
	/// attempts to merge faces into one big face
	TopoDS_Face mergeFaces(const std::vector<TopoDS_Face>& mergeFaces);

	/// outputs the time delta between the start and end time
	void printTime(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime);

	/// checks if surface is encapsulated by another shape
	bool surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape);

	/// create spatial index for voxels and lookup
	void populateVoxelIndex(
		bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>* voxelIndex,
		const std::vector<std::shared_ptr<voxel>> exteriorVoxels
	);

	// voxel shape related code

	// creates shape represnatation based on the voxels
	TopoDS_Shape voxels2Shape(int roomNum);
	// approximate the area of a room base on the voxelshape (Only works with full voxelization)
	void processDirectionalFaces(int direction, int roomNum, std::vector<TopoDS_Face>& collectionList);

	// LoD32 ray casting related code

	// get the outer surface by raycasting against exterior voxels
	void getOuterRaySurfaces(
		std::vector<std::pair<TopoDS_Face, std::string>>& outerSurfacePairList, 
		const std::vector<Value>& valueObjectList,
		std::mutex& listmutex,
		DataManager* h,
		const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIdx,
		const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>& voxelIndex);

	// query result cleaning code

	/// remove dublicate values from valueList
	std::vector<Value> makeUniqueValueList(const std::vector<Value>& valueList);
	//gets the unqiue products that intersect with the voxelList
	std::vector<Value> getUniqueProductValues(std::vector<std::shared_ptr<voxel>> voxelList);

	// CityJSON object related code

	// creates and binds the semantic surface data for the input geo data shape and cjt geo object
	void createSemanticData(CJT::GeoObject* geoObject, const TopoDS_Shape& geometryShape, bool isExterior = true);
	// binds the semantic surface data for the input geo data shape and cjt geo object
	void populateSurfaceData(CJT::GeoObject* geoObject, const std::vector<int>& SurfaceIndxDataList, bool isExterior = true);
	// generates default CJ object that represents a room, used for voxel rooms if no room objects are present
	std::shared_ptr<CJT::CityObject> createDefaultRoomObject(std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int roomNum, double lowestZ);

	// other code

	std::vector<IfcSchema::IfcBuildingStorey*> fetchStoreyObjects(DataManager* h, const std::vector<std::string>& storeyGuidList);

	// fetches the room related data from the ifc model
	std::vector < std::shared_ptr<CJT::CityObject >> fetchRoomObject(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int roomNum);

public:
	explicit CJGeoCreator(DataManager* h, double vSize);

	/// computes and internalizes the data that is required to do the basic city scale output
	void initializeBasic(DataManager* h);

	/// computes and internalizes the data that is required to do footprint related city scale output
	void makeFootprint(DataManager* h);

	/// generates the empty storey objects for the ifc storeys in a file
	std::vector<std::shared_ptr<CJT::CityObject>> makeStoreyObjects(DataManager* h);

	/// generates the empty room objects for the ifc storeys in a file
	std::vector<std::shared_ptr<CJT::CityObject>> makeRoomObjects(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& cityStoreyObjects);
	/// generates a list of cityobjects with LoD0.2/1.2/2.2 faces if required by the input settings 
	void makeSimpleLodRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	
	/// adds a storey object to the city object
	void make2DStoreys(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale, bool is03);

	/// generates an LoD0.0 object
	CJT::GeoObject makeLoD00(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.2 objects
	std::vector< CJT::GeoObject>  makeLoD02(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.3 roof faces
	std::vector<std::vector<TopoDS_Face>> makeRoofFaces(DataManager* h, CJT::Kernel* kernel, int unitScale, bool useFlatFaces, bool footprintBased = false);
	/// generates a list of LoD0.3 objects
	std::vector< CJT::GeoObject> makeLoD03(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.4 objects
	std::vector< CJT::GeoObject> makeLoD04(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates an LoD1.0 object
	CJT::GeoObject makeLoD10(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.2 objects
	std::vector< CJT::GeoObject> makeLoD12(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.3 objects
	std::vector< CJT::GeoObject> makeLoD13(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD2.2 objects
	std::vector< CJT::GeoObject> makeLoD22(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.0 objects
	std::vector< CJT::GeoObject> makeLoD30(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.0 objects
	std::vector< CJT::GeoObject> makeLoD31(DataManager* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.2 objects
	std::vector< CJT::GeoObject> makeLoD32(DataManager* h, CJT::Kernel* kernel, int unitScale);
	void makeComplexLoDRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	/// generates a list of voxelized objects
	std::vector< CJT::GeoObject> makeV(DataManager* h, CJT::Kernel* kernel, int unitScale);
	void makeVRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale);
	/// generates a list of the site and its outline
	std::vector<CJT::CityObject> makeSite(DataManager* h, CJT::Kernel* kernel, int unitScale);

	/// computes data related to the voxel shape such as volume and shell area of the outer shell
	void extractOuterVoxelSummary(CJT::CityObject* shellObject, DataManager* h, double footprintHeight, double geoRot);
	/// computes data related to the voxel shape such as volume and shell area of the inner shell
	void extractInnerVoxelSummary(CJT::CityObject* shellObject, DataManager* h);

};
#endif // CJGEOCREATOR_CJGEOCREATOR_H

