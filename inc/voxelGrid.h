#include "helper.h"
#include "settingsCollection.h"
#include "voxel.h"

#ifndef VOXELGRID_VOXELGRID_H
#define VOXELGRID_VOXELGRID_H

class VoxelGrid {
private:
	// world space data
	gp_Pnt anchor_ = gp_Pnt(0,0,0);
	double planeRotation_ = 0;

	// relative space data related to the voxel grid
	int xRelRange_ = 0;
	int yRelRange_ = 0;
	int zRelRange_ = 0;

	int totalVoxels_ = 0;;
	std::mutex voxelGrowthMutex_;
	int activeThreads_ = 0;

	std::map<int, std::shared_ptr<voxel>> VoxelLookup_;
	std::mutex voxelLookupMutex;

	int roomSize_ = 0;

	bool hasSemanticSurfaces_ = false;

	/// set the values required for voxelisation
	void init(DataManager* h);

	/// create the voxelgrid and find intersections of voxels
	void populateVoxelGrid(DataManager* h);
	/// counts the progress of the voxel population process
	void countVoxels(const int* voxelGrowthCount);

	/// creates and adds a voxel object + checks with which products from the cluster it intersects
	bool addVoxel(int indx, DataManager* h);
	/// voxelises the building by running downward columns from the top plate of the voxel grid
	void addVoxelColumn(int beginIindx, int endIdx, DataManager* h, int* voxelGrowthCount = nullptr);

	/// compute the "score" of each column by doing a querry of the ifc objects and storing the result count
	std::vector<int> computeColumnScore(DataManager* h);

	/// increment active thread count
	void addActiveThread();
	/// decrement active thread count
	void removeActiveThread();
	/// coompute the end index based on the clomn score
	int computeEndColumnIdx(const std::vector<int>& columScoreList, const int threadScore, const int beginIdx);

	/// transform coordinate from 1D to 3D voxelgrid space
	template<typename T>
	T linearToRelative(int i);
	/// transform coordinate from 3D to 1D voxelgrid space
	int relativeToLinear(const BoostPoint3D& i);
	/// transform coordinate from 3D voxelgrid space to 3D processing space
	BoostPoint3D relPointToWorld(const BoostPoint3D& p);
	/// transform coordinate from 3D processing space to 3D voxelgrid space
	BoostPoint3D worldToRelPoint(BoostPoint3D p);

	/// get a list of idx representing the dir of the neighbor in -/+ x, y, z dir 
	std::array<int, 6> getDirNeighbours(int voxelIndx);

	/// grow a void and mark it
	void growVoid(int startIndx, int roomnum, DataManager* h);
	/// grow the exterior voxelized shape
	void growExterior(DataManager* h);
	/// grow the interior voxelized shapes
	void growInterior(DataManager* h);

	/// group voxels into buildings
	void pairVoxels();
	/// mark the voxels to which building they are part off
	void markVoxelBuilding(int startIndx, int buildnum);


	// returns true if beam casted from voxel intersects with a facade opening
	bool voxelBeamWindowIntersection(DataManager* h, std::shared_ptr<voxel> currentVoxel, int indxDir);

	/// inverts the dirInx for neighbourSearching
	static int invertDir(int dirIndx);

public:
	VoxelGrid(DataManager* h);

	void computeSurfaceSemantics(DataManager* h);

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
