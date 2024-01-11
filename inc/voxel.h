#include "helper.h"
#include "dataManager.h"

#include <gp_Pnt.hxx>

#ifndef VOXEL_VOXEL_H
#define VOXEL_VOXEL_H

class voxel {
private:
	BoostPoint3D center_;

	double sizeXY_; // TODO: let function with non uniform voxels
	double sizeZ_;

	bool isIntersecting_ = false;
	bool hasEvalIntt_ = false;
	bool isInside_ = true;
	bool isShell_ = false;
	int buildingNum_ = -1;
	int roomNum_ = -1;

	std::vector<Value> internalProducts_;

	// transitional faces (faces between different types of voxels)
	bool hasFace0 = false;
	bool hasFace1 = false;
	bool hasFace2 = false;
	bool hasFace3 = false;
	bool hasFace4 = false;
	bool hasFace5 = false;

public:
	/// greates an axis aligned voxel
	explicit voxel(const BoostPoint3D& center, double sizeXY, double sizeZ);

	/// returns the lll and urr point of a voxel in axis aligned space
	bg::model::box<BoostPoint3D> getVoxelGeo();

	/// return the cornerpoints of a voxel based on the angle
	std::vector<gp_Pnt> getCornerPoints(double angle);

	/// returns integers with that comply with the getCornerPoints output
	std::vector<std::vector<int>> getVoxelTriangles();
	std::vector<std::vector<int>> getVoxelFaces();
	std::vector<std::vector<int>> getVoxelEdges();

	/// sets number representing to which building the voxel belongs
	void setBuildingNum(int num) { buildingNum_ = num; }

	/// sets flag signifying if voxel is part of the exterior shell
	void setIsShell() { isShell_ = true; }

	bool getIsShell() { return isShell_; }

	/// set flag notifying if the voxel is part of the outside of a building 
	void setOutside() { isInside_ = false; }

	/// returns flag representing if the voxel is part of the interior of a building
	const bool getIsInside() { return isInside_; }

	/// returns the centerpoint of a voxel at its virtual location
	BoostPoint3D getCenterPoint() { return center_; }
	gp_Pnt getOCCTCenterPoint() { return gp_Pnt(center_.get<0>(), center_.get<1>(), center_.get<2>()); }

	/// returns the centerpoint of a voxel at its real location
	BoostPoint3D getCenterPoint(double angle) { return helperFunctions::rotatePointWorld(center_, -angle); }

	/// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(lookupValue& lookup, const std::vector<gp_Pnt>& voxelPoints, helper* h);

	/// check if any cornerpoints fall inside voxel
	bool linearEqIntersection(const std::vector<gp_Pnt>& productPoints, const std::vector<gp_Pnt>& voxelPoints);

	/// returns the intersecting flag
	bool getIsIntersecting() { return isIntersecting_; }

	/// sets the intersecting flag
	void isIntersecting() { isIntersecting_ = true; }

	/// returns the building number
	int getBuildingNum() { return buildingNum_; }

	/// internalize an object, used to internalize the objects the voxel intersects with
	void addInternalProduct(const Value& prodValue) { internalProducts_.emplace_back(prodValue); }

	/// return has intersection evaluated flag 
	bool getHasEvalIntt() { return hasEvalIntt_; }

	/// returns the list with internalized products
	std::vector<Value> getInternalProductList() { return internalProducts_; }

	/// returns boolean if the face in that direction is present, if no number input returns if any face is present
	bool hasFace(const int* dirNum = nullptr);

	/// sets a transitionalface
	void setTransFace(const int& dirNum);
};
#endif // VOXEL_VOXEL_H

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

	// x y z size of the voxels in the grid
	double voxelSize_;
	double voxelSizeZ_;

	std::map<int, voxel*> VoxelLookup_;
	std::mutex voxelLookupMutex;

	// exterior voxels;
	std::vector<int> exteriorVoxelsIdx_;

	// assignment of the voxels (should be removed) -1 is intersected 0 is not assigned 1..n is room assignement;
	std::vector<int> Assignment_;

	/// @brief creates and adds a voxel object + checks with which products from the cluster it intersects
	void addVoxel(int indx, helper* h);
	void addVoxelPool(int beginIindx, int endIdx, helper* h, int* voxelGrowthCount = nullptr);
	void countVoxels(const int* voxelGrowthCount);

	// transform coordinates
	template<typename T>
	T linearToRelative(int i);

	int relativeToLinear(const BoostPoint3D& i);

	BoostPoint3D relPointToWorld(const BoostPoint3D& p);

	BoostPoint3D worldToRelPoint(BoostPoint3D p);

public:
	VoxelGrid();

	VoxelGrid(helper* h, double voxelSize );

	void populatedVoxelGrid(helper* h);

	/// get a list of idx representing the neighbours of the input voxel indx
	std::vector<int> getNeighbours(int voxelIndx, bool connect6 = false);
	std::vector<int> getNeighbours(voxel* boxel, bool connect6 = false);

	/// @brief get the top layer of voxels
	std::vector<int> getTopBoxelIndx();

	double getVoxelSize() { return voxelSize_; }
	const voxel& getVoxel(int i) { return *VoxelLookup_[i]; }
	//const voxel* getVoxelPtr(int i) { return VoxelLookup_[i]; }

	// returns a plate in full x an y but 1 z the closes at the input platelvl 
	std::vector<voxel*> getVoxelPlate(double platelvl);
	std::vector<voxel*> getIntersectingVoxels();

	gp_Pnt getAnchor() { return anchor_; }

	std::vector<int> growExterior(int startIndx, int roomnum, helper* h);
	void markVoxelBuilding(int startIndx, int buildnum);

};

#endif // VOXEL_VOXEL_H
