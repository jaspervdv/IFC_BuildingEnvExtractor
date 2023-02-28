#include "helper.h"
#include "floorProcessor.h"

#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <gce_MakeLin.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>

#include <GeomLProp_SLProps.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <BOPAlgo_BuilderFace.hxx>


#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>


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

		TopExp_Explorer expl;

		int counter = 0;
		for (expl.Init(edge, TopAbs_VERTEX); expl.More(); expl.Next()) {
			TopoDS_Vertex currentVertex = TopoDS::Vertex(expl.Current());
			gp_Pnt currentPoint = BRep_Tool::Pnt(currentVertex);
			if (counter == 0)
			{
				startPoint_ = currentPoint;
				counter++;
				continue;
			}
			if (counter == 1)
			{
				endPoint_ = currentPoint;
				break;
			}
		}
	}

	TopoDS_Edge* getEdge() { return theEdge_; }

	gp_Pnt getStart() { return startPoint_; }

	gp_Pnt getEnd() { return endPoint_; }

	gp_Pnt getProjectedStart() { return gp_Pnt(startPoint_.X(), startPoint_.Y(), 0); }

	gp_Pnt getProjectedEnd() { return gp_Pnt(endPoint_.X(), endPoint_.Y(), 0); }
};


class voxel {
private:

	bool isIntersecting_ = false;
	bool hasEvalIntt = false;
	std::vector<int> internalProducts;

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

	BoostPoint3D getCenterPoint(double angle) { return rotatePointWorld(center_, -angle); }

	// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(LookupValue lookup, const std::vector<gp_Pnt> voxelPoints, helper* h);

	bool linearEqIntersection(std::vector<gp_Pnt> productPoints, std::vector<gp_Pnt> voxelPoints);

	bool getIsIntersecting() { return isIntersecting_; }

	void isIntersecting() { isIntersecting_ = true; }

	void addInternalProduct(int prodValue) { internalProducts.emplace_back(prodValue); }

	bool getHasEvalIntt() { return hasEvalIntt; }

	std::vector<int> getInternalProductList() { return internalProducts; }

	double getZ() { return sizeZ_; }
};


class CJGeoCreator {

private:
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

	// higher LoD data collection
	bool hasSortedFaces_ = false;

	std::vector<std::vector<SurfaceGroup*>> faceList_;
	bool hasTopFaces_ = false;

	std::vector<TopoDS_Face> footPrintList_;
	bool hasFootPrint_ = false;

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

	std::vector<Edge*> getUniqueEdges(TopoDS_Shape& flattenedEdges);

	/// @bried merges all the overlapping edges that have the same direction
	std::vector<Edge*> mergeOverlappingEdges(std::vector<Edge*>& uniqueEdges);

	std::vector<Edge*> splitIntersectingEdges(std::vector<Edge*>& edges);

	/// @brief get 2d projection of shape at z=0
	TopoDS_Face getFlatFace(TopoDS_Face face);

	/// @brief get a clean list of all the edges the projected shapes
	std::vector<Edge*> makeJumbledGround();

	/// @brief check if edge is part of outer envelope
	bool isOuterEdge(Edge* currentEdge, std::vector<TopoDS_Face*> flatFaceList);

	/// @brief get all the edges that enclose the projected faces
	std::vector<TopoDS_Edge> getOuterEdges(std::vector<Edge*> edgeList, std::vector<SurfaceGroup*> faceList);

	/// @brief get the footprint shapes from the collection of outer edges
	std::vector<TopoDS_Face> outerEdges2Shapes(std::vector<TopoDS_Edge> edgeList);

	/// @brief grows wires from unordered exterior edges
	std::vector<TopoDS_Wire> growWires(std::vector<TopoDS_Edge> edgeList);

	/// @brief cleans the wires (removes redundant vertex)
	std::vector<TopoDS_Wire> cleanWires(std::vector<TopoDS_Wire> wireList);
	TopoDS_Wire cleanWire(TopoDS_Wire wire);

	std::vector<TopoDS_Face> wireCluster2Faces(std::vector<TopoDS_Wire> wireList);

	void initializeBasic(helperCluster* cluster);

	std::vector<TopoDS_Shape> computePrisms(bool isFlat = false);
	TopoDS_Shape simplefySolid(TopoDS_Solid solidShape);
	TopoDS_Face mergeFaces(std::vector<TopoDS_Face> mergeFaces);

	std::vector<int> getTypeValuesBySample(TopoDS_Shape prism, int prismNum, bool flat);

public:
	explicit CJGeoCreator(helperCluster* cluster, bool isFlat = true);

	CJT::GeoObject* makeLoD00(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD02(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	CJT::GeoObject* makeLoD03(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	CJT::GeoObject* makeLoD10(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD12(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD13(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	std::vector< CJT::GeoObject*> makeLoD22(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);
	CJT::GeoObject* makeLoD32(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale);

};
