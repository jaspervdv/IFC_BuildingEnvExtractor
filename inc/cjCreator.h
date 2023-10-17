#include "helper.h"
#include "DataManager.h"
#include "voxel.h"

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
	// default spatial index tree depth
	static const int treeDepth_ = 25;

	// world space data
	gp_Pnt anchor_;

	// relative space data related to the voxel grid
	int xRelRange_;
	int yRelRange_;
	int zRelRange_;

	int totalVoxels_;

	// x y z size of the voxels in the grid
	double voxelSize_;
	double voxelSizeZ_;

	// rotation of the voxelgrid (or inverse rotation of the objects)
	double planeRotation_ = 0;

	// assignment of the voxels (should be removed) -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment_;
	// exterior voxels;
	std::vector<int> exteriorVoxelsIdx_;
	// exterior voxel lookup map
	std::map<int, voxel*> VoxelLookup_;

	// container for surface group data
	std::vector<std::vector<SurfaceGroup*>> faceList_;

	// flags representing the eval state of the creator
	bool hasTopFaces_ = false;
	bool hasFootprints_ = false;
	bool useFootprints_ = false;
	bool hasGeoBase_ = false;

	// if true the roofoutlines are used to create the geometry
	bool useRoofOutline_ = true;

	// list containing all the roof outlines
	std::vector<TopoDS_Face> roofOutlineList_; 

	// list containing all the footprints
	std::vector<TopoDS_Face> footprintList_;

	/// get a list of idx representing the neighbours of the input voxel indx
	std::vector<int> getNeighbours(int voxelIndx, bool connect6 = false);

	// transform coordinates
	template<typename T>
	T linearToRelative(int i);
	BoostPoint3D relPointToWorld(const BoostPoint3D& p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);

	// create a group of voxels representing a rough room
	std::vector<int> growExterior(int startIndx, int roomnum, helper* h);

	void markVoxelBuilding(int startIndx, int roomnum);

	/// @brief creates and adds a voxel object + checks with which products from the cluster it intersects
	void addVoxel(int indx);

	/// @brief get the top geometry objects of the model
	std::vector<TopoDS_Shape> getTopObjects(helper* h);

	/// @brief get the top layer of voxels
	std::vector<int> getTopBoxelIndx();

	/// @brief get the surfaces that have an area when flattened
	std::vector<SurfaceGroup*> getXYFaces(const TopoDS_Shape& shape);

	/// get the unique edges from in a shape or a collection of edges
	std::vector<Edge*> getUniqueEdges(const TopoDS_Shape& flattenedEdges);
	std::vector<Edge*> getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges);

	/// check if edge is in list of edge objects
	bool isInList(const TopoDS_Edge& currentEdge, const std::vector<Edge*>& edgeList);

	/// @bried merges all the overlapping edges that have the same direction
	std::vector<Edge*> mergeOverlappingEdges(std::vector<Edge*>& uniqueEdges, bool project=true);

	/// splits all the edges so that no edges overlap but all rest against eachother with their start and endpoints
	std::vector<Edge*> splitIntersectingEdges(std::vector<Edge*>& edges, bool project = true);

	/// @brief get a clean list of all the edges the projected shapes
	std::vector<Edge*> makeJumbledGround();

	/// @brief check if edge is part of outer envelope
	bool isOuterEdge(Edge* currentEdge, const std::vector<TopoDS_Face*>& flatFaceList, const bgi::rtree<Value, bgi::rstar<treeDepth_>>& spatialIndex);

	/// @brief get all the edges that enclose the projected faces
	std::vector<TopoDS_Edge> getOuterEdges(const std::vector<Edge*>& edgeList, const std::vector<SurfaceGroup*>& faceList);

	/// @brief get the footprint shapes from the collection of outer edges
	std::vector<TopoDS_Face> outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief grows wires from unordered exterior edges
	std::vector<TopoDS_Wire> growWires(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief cleans the wires (removes redundant vertex)
	std::vector<TopoDS_Wire> cleanWires(const std::vector<TopoDS_Wire>& wireList);
	TopoDS_Wire cleanWire(const TopoDS_Wire& wire);


	std::vector<TopoDS_Face> wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList);

	/// create a solid extrusion from the projected roofoutline
	std::vector<TopoDS_Shape> computePrisms(bool isFlat = false);

	/// remove redundant edges from a solid shape
	TopoDS_Shape simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap = false);

	/// attempts to merge faces into one big face
	TopoDS_Face mergeFaces(const std::vector<TopoDS_Face>& mergeFaces);

	/// generate the indx related to the type data for json output: 0 = GroundSurface, 1 = WallSurface, 2 = RoofSurface
	std::vector<int> getTypeValuesBySample(const TopoDS_Shape& prism, int prismNum, bool flat);

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
		std::vector<int>* originVoxels, 
		std::vector<Value>* productLookupValues, 
		const std::vector<int>& exteriorVoxels
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
		const std::vector<int>& originVoxels,
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
		const std::vector<int>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		double gridDistance,
		double buffer
	);

	/// checks if the point is visible utilizing a pointgrid
	bool pointIsVisible(helper* h,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<int>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		const gp_Pnt& point,
		const double& buffer);

public:
	explicit CJGeoCreator(helper* h, double vSize);

	/// computes and internalizes the data that is required to do the basic city scale output
	void initializeBasic(helper* h);

	/// computes and internalizes the data that is required to do footprint related city scale output
	void makeFootprint(helper* h);

	/// generates an LoD0.0 object
	CJT::GeoObject* makeLoD00(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.2 objects
	std::vector< CJT::GeoObject*> makeLoD02(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD0.3 objects
	// TODO: implement
	std::vector< CJT::GeoObject*> makeLoD03(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates an LoD1.0 object
	CJT::GeoObject* makeLoD10(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.2 objects
	std::vector< CJT::GeoObject*> makeLoD12(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD1.3 objects
	std::vector< CJT::GeoObject*> makeLoD13(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD2.2 objects
	std::vector< CJT::GeoObject*> makeLoD22(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of LoD3.2 objects
	std::vector< CJT::GeoObject*> makeLoD32(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	/// generates a list of voxelized objects
	std::vector< CJT::GeoObject*> makeV(helper* h, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
};
#endif // CJGEOCREATOR_CJGEOCREATOR_H

