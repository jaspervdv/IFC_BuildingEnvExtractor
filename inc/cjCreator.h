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
	// default spatial index tree depth
	static const int treeDepth_ = 25;
	std::shared_ptr<VoxelGrid> voxelGrid_ = nullptr;

	// rotation of the voxelgrid (or inverse rotation of the objects)
	gp_Trsf geoRefRotation_;

	// container for surface group data
	std::vector<std::vector<SurfaceGroup>> faceList_;
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

	// create a group of voxels representing a rough room
	std::vector<int> growExterior(int startIndx, int roomnum, helper* h);

	void markVoxelBuilding(int startIndx, int roomnum);

	/// @brief get the top geometry objects of the model
	std::vector<TopoDS_Shape> getTopObjects(helper* h);

	/// @brief reduce the surfaces of an object for roof extraction by z-ray casting on itself
	void reduceSurfaces(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<SurfaceGroup>* shapeList);
	void reduceSurface(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<SurfaceGroup>* shapeList);

	/// @brief reduces the surfaces by merging neigbouring surfaces that have parallel normals 
	void mergeSurfaces(const std::vector<SurfaceGroup>& shapeList);

	/// @brief reduce the surfaces in the facelist for roof extraction by z-ray casting on itself and others
	void FinefilterSurfaces(const std::vector<SurfaceGroup>& shapeList);
	void FinefilterSurface(const std::vector<SurfaceGroup>& shapeList, const std::vector<SurfaceGroup>& otherShapeList);

	/// @brief get the surfaces that have an area when flattened
	std::vector<SurfaceGroup> getXYFaces(const TopoDS_Shape& shape);

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

	/// splits all the edges so that no edges overlap but all rest against eachother with their start and endpoints
	std::vector<Edge> splitIntersectingEdges(const std::vector<Edge>& edges, bool project = true);

	/// @brief get a clean list of all the edges the projected shapes
	std::vector<Edge> makeJumbledGround();

	/// @brief check if edge is part of outer envelope
	bool isOuterEdge(Edge currentEdge, const std::vector<TopoDS_Face>& flatFaceList, const bgi::rtree<Value, bgi::rstar<treeDepth_>>& spatialIndex);

	/// @brief get all the edges that enclose the projected faces
	std::vector<TopoDS_Edge> getOuterEdges(const std::vector<Edge>& edgeList, const std::vector<SurfaceGroup>& faceList);

	/// @brief get all the outer edges based on a voxelplate
	std::vector<TopoDS_Edge> getOuterEdges(
		const std::vector<Edge>& edgeList,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<std::shared_ptr<voxel>>& originVoxels,
		double floorlvl);

	/// @brief get the footprint shapes from the collection of outer edges
	std::vector<TopoDS_Face> outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief grows wires from unordered exterior edges
	std::vector<TopoDS_Wire> growWires(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief cleans the wires (removes redundant vertex)
	std::vector<TopoDS_Wire> cleanWires(const std::vector<TopoDS_Wire>& wireList);
	TopoDS_Wire cleanWire(const TopoDS_Wire& wire);

	std::vector<TopoDS_Face> wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList);

	// divides the projected footprints over the seperate buildings
	void sortRoofStructures();

	// merges faces that are near eachother
	void mergeRoofSurfaces();

	// get a full xy section of voxels that is 1 voxel thick at the desired plate level
	std::vector<int> getVoxelPlate(double platelvl);

	// create list of edges by cutting objects at the floor lvl
	std::vector<TopoDS_Edge> section2edges(const std::vector<Value>& productLookupValues, helper* h, double cutlvl);

	// extrudes shape downwards and caps it on the splitting face
	TopoDS_Solid extrudeFaceDW(const TopoDS_Face& evalFace, double splittingFaceHeight = 0);

	/// create a solid extrusion from the projected roofoutline
	std::vector<TopoDS_Shape> computePrisms(bool isFlat, helper* h);

	/// remove redundant edges from a solid shape
	TopoDS_Shape simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap = false);

	/// attempts to merge faces into one big face
	TopoDS_Face mergeFaces(const std::vector<TopoDS_Face>& mergeFaces);

	/// generate the indx related to the type data for json output: 0 = GroundSurface, 1 = WallSurface, 2 = RoofSurface
	std::vector<int> getTypeValuesBySample(const TopoDS_Shape& prism, bool flat);

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
		std::vector<Value>* productLookupValues, 
		const std::vector<std::shared_ptr<voxel>> exteriorVoxels
	);

	/// remove objects that are completely encapsulated by other objects
	void filterEncapsulatedObjects(std::vector<Value>* productLookupValues, bgi::rtree<Value, bgi::rstar<25>>* exteriorProductIndex, helper* h);

	/// remove dublicate values from valueList
	std::vector<Value> makeUniqueValueList(const std::vector<Value>& valueList);

	/// checks if the surface is visible utilizing a pointgrid
	bool isSurfaceVisible(
		helper* h,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<std::shared_ptr<voxel>>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		double gridDistance,
		double buffer
	);

	/// checks if the edge is visible utilizing a pointgrid
	bool isWireVisible(
		helper* h,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<std::shared_ptr<voxel>>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		double gridDistance,
		double buffer
	);

	/// checks if the point is visible utilizing a pointgrid
	bool pointIsVisible(helper* h,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<std::shared_ptr<voxel>>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		const gp_Pnt& point,
		const double& buffer);

public:
	explicit CJGeoCreator(helper* h, double vSize);

	/// computes and internalizes the data that is required to do the basic city scale output
	void initializeBasic(helper* h);

	/// computes and internalizes the data that is required to do footprint related city scale output
	void makeFootprint(helper* h);

	void makeFloorSectionCollection(helper* h);
	std::vector<TopoDS_Face> makeFloorSection(helper* h, double sectionHeight);

	/// store the roofoutline data to LoD 02
	void useroofprint0() { useRoofprints_ = true; }

	/// generates the storey objects for the ifc storeys in a file
	std::vector<std::shared_ptr<CJT::CityObject>> makeStoreyObjects(helper* h);

	/// generates an LoD0.0 object
	CJT::GeoObject makeLoD00(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.2 objects
	std::vector< CJT::GeoObject>  makeLoD02(helper* h, CJT::Kernel* kernel, int unitScale);
	void makeLoD02Storeys(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale);
	std::vector < CJT::CityObject> makeLoD02Apartments(helper* h, CJT::Kernel* kernel, std::string guid, int unitScale);
	/// generates a list of LoD0.3 objects
	// TODO: implement
	std::vector< CJT::GeoObject*> makeLoD03(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates an LoD1.0 object
	CJT::GeoObject makeLoD10(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.2 objects
	std::vector< CJT::GeoObject> makeLoD12(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.3 objects
	std::vector< CJT::GeoObject> makeLoD13(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD2.2 objects
	std::vector< CJT::GeoObject> makeLoD22(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.2 objects
	std::vector< CJT::GeoObject> makeLoD32(helper* h, CJT::Kernel* kernel, int unitScale);
	/// generates a list of voxelized objects
	std::vector< CJT::GeoObject> makeV(helper* h, CJT::Kernel* kernel, int unitScale);
	std::vector<CJT::CityObject>  makeVRooms(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale);
	/// generates a list of the site and its outline
	std::vector<CJT::CityObject> makeSite(helper* h, CJT::Kernel* kernel, int unitScale);


	void setRefRotation(const gp_Trsf& trsf) { geoRefRotation_ = trsf; }
	gp_Trsf getRefRotation() { return geoRefRotation_; }

	TopoDS_Shape voxels2Shape(int roomNum);
	// approximate the area of a room base on the voxelshape (Only works with full voxelization)
	double approximateRoomArea(int roomNum);
	void processDirectionalFaces(int direction, int roomNum, std::vector<TopoDS_Face>& collectionList);

	/// computes data related to the voxel shape such as volume and shell area
	void extractOuterVoxelSummary(CJT::CityObject* shellObject, helper* h, double footprintHeight, double geoRot);
	void extractInnerVoxelSummary(CJT::CityObject* shellObject, helper* h);
};
#endif // CJGEOCREATOR_CJGEOCREATOR_H

