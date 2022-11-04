#include "helper.h"
#include "floorProcessor.h"

#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <gce_MakeLin.hxx>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <vector>
#include <tuple>


class voxel {
private:
	bool isIntersecting_ = false;
	bool isInside = true;
	std::vector<int> roomnums_;
	BoostPoint3D center_;
	double sizeXY_;
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

	double getZ() { return sizeZ_; }
};


class voxelfield {

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

	std::vector<roomObject*> roomObjectList_;
	std::vector<double> roomAreaList_;
	bool hasGraphData_;
	
	std::vector<int> getNeighbours(int voxelIndx);

	// transform coordinates 
	template<typename T>
	T linearToRelative(int i);
	BoostPoint3D relPointToWorld(BoostPoint3D p);
	BoostPoint3D relPointToWorld(int px, int py, int pz);
	BoostPoint3D WorldPointToRel(BoostPoint3D p);

	// create a group of voxels representing a rough room
	std::vector<int> growRoom(int startIndx, int roomnum);

	// generates the faces of the voxel that are needed to create a rough room shape
	std::vector<TopoDS_Face> getPartialFaces(std::vector<int> roomIndx, int voxelIndx);

	TopoDS_Face getLowestFace(TopoDS_Shape shape);

	// creates and adds a voxel object + checks with which products from the cluster it intersects
	void addVoxel(int indx, helperCluster* cluster);

	void outputFieldToFile();

	void createGraph(helperCluster* cluster);

	// returns vector with: semanticName, semanticLongName, semanticDescription
	std::vector<std::string> getSemanticMatch(std::vector< IfcSchema::IfcSpace*> semanticSources, int roomNum);

public:

	explicit voxelfield(helperCluster* cluster, bool isFlat = true);

	void makeRooms(helperCluster* cluster);

	void writeGraph(std::string path);

};
