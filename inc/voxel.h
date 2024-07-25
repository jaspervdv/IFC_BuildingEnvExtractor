#include "helper.h"
#include "DataManager.h"

#include <gp_Pnt.hxx>

#ifndef VOXEL_VOXEL_H
#define VOXEL_VOXEL_H

class voxelFace {
private:
	bool isWall_ = true;
	bool isWindow_ = false;
	bool isRoof_ = false;

public:
	void setIsWall();
	void setIsWindow();
	void setIsRoof();

	bool isWall() { return isWall_; }
	bool isWindow() { return isWindow_; }
	bool isRoof() { return isRoof_; }
};


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
	// voids hold their own containers, intersecting voxels hold the outer shell
	bool hasFace0_ = false; // -x
	bool hasFace1_ = false; // +x
	bool hasFace2_ = false; // -y
	bool hasFace3_ = false; // +y
	bool hasFace4_ = false; // -z
	bool hasFace5_ = false; // +z

	// map containing the semantic voxel faces, the int signifies the dir
	std::map<int, voxelFace> faceMap_ = {}; 

public:
	/// greates an axis aligned voxel
	explicit voxel(const BoostPoint3D& center, double sizeXY, double sizeZ);

	/// returns the lll and urr point of a voxel in axis aligned space
	bg::model::box<BoostPoint3D> getVoxelGeo();

	/// return the cornerpoints of a voxel based on the angle
	std::vector<gp_Pnt> getCornerPoints();
	/// return the points represeinging the three intersection planes of a voxel based on the angle
	std::vector<gp_Pnt> getPlanePoints();

	/// returns integers with that comply with the getCornerPoints output
	std::vector<std::vector<int>> getVoxelTriangles();
	std::vector<std::vector<int>> getVoxelFaces();
	std::vector<std::vector<int>> getVoxelEdges();

	/// returns integers with that comply with the getPlanePoints output
	std::vector<std::vector<int>> getplaneTriangles();
	std::vector<std::vector<int>> getPlaneEdges();

	/// sets number representing to which building the voxel belongs
	void setBuildingNum(int num) { buildingNum_ = num; }

	/// sets flag signifying if voxel is part of the exterior shell
	void setIsShell() { isShell_ = true; }

	bool getIsShell() { return isShell_; }

	/// set flag notifying if the voxel is part of the outside of a building 
	void setOutside() { isInside_ = false; }
	void setOutside(bool b) { isInside_ = !b; }

	/// returns flag representing if the voxel is part of the interior of a building
	const bool getIsInside() { return isInside_; }

	/// returns the centerpoint of a voxel at its virtual location
	BoostPoint3D getCenterPoint() { return center_; }
	gp_Pnt getOCCTCenterPoint() { return gp_Pnt(center_.get<0>(), center_.get<1>(), center_.get<2>()); }

	/// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(lookupValue& lookup, const std::vector<gp_Pnt>& voxelPoints, const gp_Pnt& centerPoint, helper* h, int intersectionLogic = 4);

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
	/// 0 = -X
	/// 1 = +X
	/// 2 = -Y
	/// 3 = +Y
	/// 4 = +Z
	/// 5 = -Z
	bool hasFace(int dirNum = -1);

	/// returns the number of transfaces
	int numberOfFaces();

	/// sets a transitionalface
	void setTransFace(int dirNum);

	void setRoomNum(int roomNum) { roomNum_ = roomNum; }
	int getRoomNum() { return roomNum_; }

	void addRoofSemantic(int indx);
	void addWindowSemantic(int indx);

	bool hasWindow(int dirNum = -1);
};
#endif // VOXEL_VOXEL_H
