#include "helper.h"
#include "settingsCollection.h"
#include "voxel.h"

#ifndef VOXELGRID_VOXELGRID_H
#define VOXELGRID_VOXELGRID_H

class VoxelGrid {
private:
	// world space data
	gp_Pnt anchor_;
	double planeRotation_ = 0;

	// relative space data related to the voxel grid
	int xRelRange_;
	int yRelRange_;
	int zRelRange_;

	int totalVoxels_;
	std::mutex voxelGrowthMutex;

	std::map<int, std::shared_ptr<voxel>> VoxelLookup_;
	std::mutex voxelLookupMutex;

	// exterior voxels;
	std::vector<int> exteriorVoxelsIdx_;

	int roomSize_ = 0;

	bool hasSemanticSurfaces_ = false;

	/// create the voxelgrid and find intersections of voxels
	void populatedVoxelGrid(helper* h);

	/// @brief creates and adds a voxel object + checks with which products from the cluster it intersects
	bool addVoxel(int indx, helper* h, bool checkIfInt = true); /// returns true if intersects
	void addVoxelColumn(int beginIindx, int endIdx, helper* h, int* voxelGrowthCount = nullptr);
	void countVoxels(const int* voxelGrowthCount);

	// transform coordinates
	template<typename T>
	T linearToRelative(int i);
	int relativeToLinear(const BoostPoint3D& i);

	BoostPoint3D relPointToWorld(const BoostPoint3D& p);
	BoostPoint3D worldToRelPoint(BoostPoint3D p);

	/// grow the exterior voxelized shape
	void growExterior(int startIndx, int roomnum, helper* h);

	/// mark the voxels to which building they are part off
	void markVoxelBuilding(int startIndx, int buildnum);

	/// get a list of idx representing the neighbours of the input voxel indx
	std::vector<int> getNeighbours(int voxelIndx, bool connect6 = false);

	/// get a list of idx representing the dir of the neighbor in -/+ x, y, z dir 
	std::vector<int> getDirNeighbours(int voxelIndx);

	std::vector<TopoDS_Edge> getTransitionalEdges(int dirIndx, int voxelIndx);

	// returns true if beam casted from voxel intersects with a facade opening
	bool voxelBeamWindowIntersection(helper* h, std::shared_ptr<voxel> currentVoxel, int indxDir);


public:
	VoxelGrid(helper* h);

	void computeSurfaceSemantics(helper* h);

	/// get a list of idx representing the neighbours of the input voxel
	std::vector<int> getNeighbours(std::shared_ptr<voxel> boxel, bool connect6 = false);
	int getNeighbour(std::shared_ptr<voxel> boxel, int dir);

	int getLowerNeighbour(int voxelIndx, bool connect6 = false);

	/// @brief get the top layer of voxels
	std::vector<int> getTopBoxelIndx();

	const voxel& getVoxel(int i) { return *VoxelLookup_[i]; }
	std::shared_ptr<voxel> getVoxelPtr(int i) { return VoxelLookup_[i]; }

	// returns a plate in full x an y but 1 z the closes at the input platelvl 
	std::vector<std::shared_ptr<voxel>> getVoxelPlate(double platelvl);
	std::vector<std::shared_ptr<voxel>> getIntersectingVoxels();
	std::vector<std::shared_ptr<voxel>> getExternalVoxels();
	std::vector<std::shared_ptr<voxel>> getInternalVoxels();
	std::vector<std::shared_ptr<voxel>> getVoxels();

	gp_Pnt getAnchor() { return anchor_; }
	double getRotation() { return planeRotation_; }

	int getRoomSize() { return roomSize_; }
	double getRoomArea(int roomNum);

	std::vector<std::vector<TopoDS_Edge>> getDirectionalFaces(int dirIndx, double angle, int roomNum);

	gp_Pnt getPointInRoom(int roomNum);

};

#endif // VOXEL_VOXEL_H
