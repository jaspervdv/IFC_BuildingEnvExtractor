#include "helper.h"

#include <chrono>
#include <CJToKernel.h>

#include <TopoDS.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>

#ifndef EVALUATIONPOINT_EVALUATIONPOINT_H
#define EVALUATIONPOINT_EVALUATIONPOINT_H

class EvaluationPoint {
private:
	gp_Pnt thePoint_;
	TopoDS_Edge evalEdge_;
	gp_Lin evalLin_;
	bool isVisible_ = true;

public:
	EvaluationPoint(gp_Pnt p);
	bool isVisible() { return isVisible_; }
	void setInvisible() { isVisible_ = false; }

	const gp_Pnt getPoint() { return thePoint_; }
	const TopoDS_Edge& getEvalEdge() { return evalEdge_; }
	const gp_Lin& getEvalLine() { return evalLin_; }
};
#endif // EVALUATIONPOINT_EVALUATIONPOINT_H


#ifndef SURFACEGROUP_SURFACEGROUP_H
#define SURFACEGROUP_SURFACEGROUP_H

class SurfaceGroup {
private:
	TopoDS_Face theFace_;
	TopoDS_Face theFlatFace_;
	TopoDS_Face theProjectedFace_;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;
	
	gp_Pnt2d llPoint_;
	gp_Pnt2d urPoint_;

	double topHeight_;
	double avHeight_;

	bool visibility_ = true;
	bool isSmall_ = false;
	int vertCount_;

	std::vector<EvaluationPoint*> pointGrid_;
	bool overlap(SurfaceGroup* other);

public:

	SurfaceGroup(TopoDS_Face aFace);

	const TopoDS_Face& getFace() { return theFace_; }
	const TopoDS_Face& getFlatFace() { return theFlatFace_; }
	const TopoDS_Face& getProjectedFace() { return theProjectedFace_; }
	TopoDS_Face* getProjectedFacePtr() { return &theProjectedFace_; }

	const gp_Pnt getLLLPoint() { return lllPoint_; }
	const gp_Pnt getURRPoint() { return urrPoint_; }

	const gp_Pnt2d getLLPoint() { return llPoint_; }
	const gp_Pnt2d getURPoint() { return urPoint_; }

	double getAvHeight() { return avHeight_; }
	double getTopHeight() { return topHeight_; }

	std::vector<EvaluationPoint*>& getPointGrid() { return pointGrid_; }
	bool isVisible() { return visibility_; }
	bool testIsVisable(std::vector<SurfaceGroup*> otherSurfaces, bool preFilter = false);
	int getVertCount() { return vertCount_; }

	void setIsHidden() { visibility_ = false; }
	void computeFlatFace();
	void projectFace();

	void populateGrid(double distance);
};
#endif // SURFACEGROUP_SURFACEGROUP_H


#ifndef EDGE_EDGE_H
#define EDGE_EDGE_H

class Edge {
private:
	TopoDS_Edge* theEdge_;
	gp_Pnt startPoint_;
	gp_Pnt endPoint_;

	bool hasEdgeEval_ = false;
	bool isOuter_ = false;

public:
	explicit Edge(TopoDS_Edge edge)
	{
		theEdge_ = new TopoDS_Edge(edge);

		TopExp_Explorer vertexExplorer(edge, TopAbs_VERTEX);

		// Step 2: Get the start and end vertices of the edge.
		gp_Pnt startVertex, endVertex;
		if (vertexExplorer.More()) {
			startPoint_ = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
			vertexExplorer.Next();
		}
		if (vertexExplorer.More()) {
			endPoint_ = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
		}


	}

	TopoDS_Edge* getEdge() { return theEdge_; }

	gp_Pnt getStart() { return startPoint_; }

	gp_Pnt getEnd() { return endPoint_; }

	gp_Pnt getProjectedStart() { return gp_Pnt(startPoint_.X(), startPoint_.Y(), 0); }

	gp_Pnt getProjectedEnd() { return gp_Pnt(endPoint_.X(), endPoint_.Y(), 0); }
};
#endif // EDGE_EDGE_H


#ifndef VOXEL_VOXEL_H
#define VOXEL_VOXEL_H

class voxel {
private:

	bool isIntersecting_ = false;
	bool hasEvalIntt = false;
	std::vector<Value> internalProducts;

	bool isInside = true;
	std::vector<int> roomnums_; 
	BoostPoint3D center_;
	double sizeXY_; // TODO: remove redundant
	double sizeZ_;


public:

	// greates an axis aligned voxel
	explicit voxel(BoostPoint3D center, double sizeXY, double sizeZ);

	// returns the lll and urr point of a voxel in axis aligned space
	bg::model::box<BoostPoint3D> getVoxelGeo();

	// return the cornerpoints of a voxel based on the angle
	std::vector<gp_Pnt> getCornerPoints(double angle);

	// returns integers with that comply with the getCornerPoints output
	std::vector<std::vector<int>> getVoxelTriangles();
	std::vector<std::vector<int>> getVoxelFaces();
	std::vector<std::vector<int>> getVoxelEdges();

	void addRoomNumber(int num) { roomnums_.emplace_back(num); }

	void setOutside() { isInside = false; }

	std::vector<int> getRoomNumbers() { return roomnums_; }

	bool getIsInside() { return isInside; }

	BoostPoint3D getCenterPoint() { return center_; }
	gp_Pnt getOCCTCenterPoint() { return gp_Pnt(center_.get<0>(), center_.get<1>(), center_.get<2>()); }

	BoostPoint3D getCenterPoint(double angle) { return rotatePointWorld(center_, -angle); }

	// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(LookupValue lookup, const std::vector<gp_Pnt> voxelPoints, helper* h);

	bool linearEqIntersection(std::vector<gp_Pnt> productPoints, std::vector<gp_Pnt> voxelPoints);

	bool getIsIntersecting() { return isIntersecting_; }

	void isIntersecting() { isIntersecting_ = true; }

	void addInternalProduct(Value prodValue) { internalProducts.emplace_back(prodValue); }

	bool getHasEvalIntt() { return hasEvalIntt; }

	std::vector<Value> getInternalProductList() { return internalProducts; }

	double getZ() { return sizeZ_; }
};
#endif // VOXEL_VOXEL_H


#ifndef CJGEOCREATOR_CJGEOCREATOR_H
#define CJGEOCREATOR_CJGEOCREATOR_H

class CJGeoCreator {

private:

	static const int treeDepth_ = 25;

	//world space data
	gp_Pnt anchor_;

	// relative space data
	int xRelRange_;
	int yRelRange_;
	int zRelRange_;

	int totalVoxels_;

	// x y z size of the voxel
	double voxelSize_; // xy plane size
	double voxelSizeZ_; // z plane size if flat roofed

	bool isFlat_ = true;

	double planeRotation_ = 0;

	double hallwayNum_ = 5;
	double minRoom_ = 2;
	double minArea_ = 32;

	// -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment_;
	std::map<int, voxel*> VoxelLookup_;
	std::vector<int> exteriorVoxelsIdx_;

	// higher LoD data collection
	bool hasSortedFaces_ = false;

	std::vector<std::vector<SurfaceGroup*>> faceList_;
	bool hasTopFaces_ = false;
	bool hasFootprints_ = false;
	bool useFootprints_ = false;

	std::vector<TopoDS_Face> roofOutlineList_; // list containing all the roof outlines
	std::vector<TopoDS_Face> footprintList_; // list containing all the footprints
	bool hasGeoBase_ = false;
	bool useRoofOutline_ = true; // if true the roofoutlines are used to create the geometry

	std::vector<TopoDS_Face>& getGeoBase_();

	std::vector<int> getNeighbours(int voxelIndx, bool connect6 = false);

	// transform coordinates 
	template<typename T>
	T linearToRelative(int i);
	BoostPoint3D relPointToWorld(BoostPoint3D p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);
	BoostPoint3D WorldPointToRel(BoostPoint3D p);

	// create a group of voxels representing a rough room
	std::vector<int> growExterior(int startIndx, int roomnum, helperCluster* cluster);

	// generates the faces of the voxel that are needed to create a rough room shape
	std::vector<TopoDS_Face> getPartialFaces(std::vector<int> roomIndx, int voxelIndx);

	/// @brief get the lowest face of a shape
	TopoDS_Face getLowestFace(TopoDS_Shape shape);

	/// @brief creates and adds a voxel object + checks with which products from the cluster it intersects
	void addVoxel(int indx, helperCluster* cluster);

	void outputFieldToFile();

	/// @brief get the top geometry objects of the model
	std::vector<TopoDS_Shape> getTopObjects(helperCluster* cluster);

	/// @brief get the top layer of voxels
	std::vector<int> getTopBoxelIndx();

	/// @brief get the surfaces that have an area when flattened
	std::vector<SurfaceGroup*> getXYFaces(TopoDS_Shape shape);

	std::vector<Edge*> getUniqueEdges(const TopoDS_Shape& flattenedEdges);
	std::vector<Edge*> getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges);

	bool isInList(TopoDS_Edge currentEdge, std::vector<Edge*> edgeList);

	/// @bried merges all the overlapping edges that have the same direction
	std::vector<Edge*> mergeOverlappingEdges(std::vector<Edge*>& uniqueEdges, bool project=true);

	std::vector<Edge*> splitIntersectingEdges(std::vector<Edge*>& edges, bool project = true);

	/// @brief get 2d projection of shape at z=0
	TopoDS_Face getFlatFace(TopoDS_Face face);

	/// @brief get a clean list of all the edges the projected shapes
	std::vector<Edge*> makeJumbledGround();

	/// @brief check if edge is part of outer envelope
	bool isOuterEdge(Edge* currentEdge, std::vector<TopoDS_Face*> flatFaceList, bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex);

	/// @brief get all the edges that enclose the projected faces
	std::vector<TopoDS_Edge> getOuterEdges(std::vector<Edge*> edgeList, std::vector<SurfaceGroup*> faceList);

	/// @brief get the footprint shapes from the collection of outer edges
	std::vector<TopoDS_Face> outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief grows wires from unordered exterior edges
	std::vector<TopoDS_Wire> growWires(const std::vector<TopoDS_Edge>& edgeList);

	/// @brief cleans the wires (removes redundant vertex)
	std::vector<TopoDS_Wire> cleanWires(const std::vector<TopoDS_Wire>& wireList);
	TopoDS_Wire cleanWire(const TopoDS_Wire& wire);

	std::vector<TopoDS_Face> wireCluster2Faces(std::vector<TopoDS_Wire> wireList);

	std::vector<TopoDS_Shape> computePrisms(bool isFlat = false);
	TopoDS_Shape simplefySolid(TopoDS_Shape solidShape, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap = false);
	std::vector<TopoDS_Face> simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap = false);
	TopoDS_Face mergeFaces(std::vector<TopoDS_Face> mergeFaces);

	std::vector<int> getTypeValuesBySample(TopoDS_Shape prism, int prismNum, bool flat);

	void printTime(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime);

	// checks if surface is encapsulated by another shape
	bool surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape);

	// checks if ray intersects shape
	bool checkShapeIntersection(const TopoDS_Edge& ray, const TopoDS_Shape& shape);
	bool checksurfaceIntersection(const TopoDS_Edge& ray, const TopoDS_Face& face);


	// create spatial index for voxels and lookup
	void populateVoxelIndex(
		bgi::rtree<Value, bgi::rstar<25>>* voxelIndex, 
		std::vector<int>* originVoxels, 
		std::vector<Value>* productLookupValues, 
		const std::vector<int>& exteriorVoxels
	);

	// remove objects that are completely encapsulated by other objects
	void filterEncapsulatedObjects(std::vector<Value>* productLookupValues, bgi::rtree<Value, bgi::rstar<25>>* exteriorProductIndex, helperCluster* cluster);

	// remove dublicate values from valueList
	std::vector<Value> makeUniqueValueList(const std::vector<Value>& valueList);

	bool isSurfaceVisible(
		helperCluster* cluster,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<int>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		double gridDistance,
		double buffer
	);

	bool isWireVisible(
		helperCluster* cluster,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<int>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		double gridDistance,
		double buffer
	);

	bool pointIsVisible(helperCluster* cluster,
		const TopoDS_Shape& currentShape,
		const TopoDS_Face& currentFace,
		const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
		const std::vector<int>& originVoxels,
		const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
		const gp_Pnt& point,
		const double& buffer);

public:
	explicit CJGeoCreator(helperCluster* cluster, double vSize);

	void initializeBasic(helperCluster* cluster);
	void makeFootprint(helperCluster* cluster);

	CJT::GeoObject* makeLoD00(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD02(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	CJT::GeoObject* makeLoD03(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	CJT::GeoObject* makeLoD10(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD12(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD13(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD22(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD32(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeV(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
};
#endif // CJGEOCREATOR_CJGEOCREATOR_H

