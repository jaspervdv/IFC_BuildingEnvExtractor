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
	/// the voxel center point in processing space (not world space)
	BoostPoint3D center_;

	/// the size of the voxel in the xy plane
	double sizeXY_;
	/// the size of the voxel in the z direction 
	double sizeZ_;

	/// flag signifying if the voxel intersects with ifc product geo
	bool isIntersecting_ = false;
	/// flag signifying if the voxel is inside of the building
	bool isInside_ = true;
	/// flag signifying if the voxel is part of the outer ring of intersecting voxels
	bool isShell_ = false;
	/// int grouping the voxel to a building
	int buildingNum_ = -1;
	/// int grouping the voxel to a room
	int roomNum_ = -1;

	/// collection of the ifc product the voxel intersects with
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

	/// get the points of the product mesh
	std::vector<gp_Pnt> getPotentialMeshObjectPoints(const std::vector<gp_Pnt>& voxelPoints, IfcProductSpatialData& lookup);	
	/// check if any cornerpoints fall inside voxel
	bool linearEqIntersection(const std::vector<gp_Pnt>& productPoints, const std::vector<gp_Pnt>& voxelPoints);

	/// check if the centerpoint of the voxel falls within a product shape
	bool voxelCoreIsInShape(const gp_Pnt& centerPoint, IfcProductSpatialData& lookup);
	/// check if any edge of the product mesh intersects with the voxel
	bool productEdgeIntersectsVoxel(const std::vector<gp_Pnt>& voxelPoints, const std::vector<gp_Pnt>& productPoints, int intersectionLogic);
	/// check if any edge of the voxel mesh intersects with the prodcut
	bool voxelEdgeIntersectsProduct(const std::vector<gp_Pnt>& voxelPoints, const std::vector<gp_Pnt>& productPoints, int intersectionLogic);

public:
	/// greates an axis aligned voxel
	explicit voxel(const BoostPoint3D& center, double sizeXY, double sizeZ);

	/// returns the lll and urr point of a voxel in axis aligned space
	bg::model::box<BoostPoint3D> getVoxelGeo();
	/// return the cornerpoints of a voxel based on the angle
	std::vector<gp_Pnt> getCornerPoints();
	/// return the points represeinging the three intersection planes of a voxel based on the angle
	std::vector<gp_Pnt> getPlanePoints();

	/// check the intersection of a triangluted product and a voxel
	bool checkIntersecting(IfcProductSpatialData& lookup, const std::vector<gp_Pnt>& voxelPoints, const gp_Pnt& centerPoint, int intersectionLogic = 4);

	/// returns integers representing the voxel mesh that comply with the getCornerPoints output
	static const std::vector<std::vector<int>>& getVoxelTriangles();
	/// returns integers representing the voxel faces that comply with the getCornerPoints output
	static const std::vector<std::vector<int>>& getVoxelFaces();
	/// returns integers representing the voxel edges that comply with the getCornerPoints output
	static const std::vector<std::vector<int>>& getVoxelEdges();

	/// returns integers of the voxel planes' mesh that comply with the getPlanePoints output
	static const std::vector<std::vector<int>>& getplaneTriangles();
	/// returns integers of the voxel planes' edges that comply with the getPlanePoints output
	static const std::vector<std::vector<int>>& getPlaneEdges();

	/// sets number representing to which building the voxel belongs
	void setBuildingNum(int num) { buildingNum_ = num; }

	/// sets flag signifying if voxel is part of the exterior shell
	void setIsShell() { isShell_ = true; }
	/// get flag signifying if voxel is part of the exterior shell
	bool getIsShell() { return isShell_; }

	/// set flag notifying if the voxel is part of the outside of a building 
	void setOutside(bool b) { isInside_ = !b; }
	/// returns flag representing if the voxel is part of the interior of a building
	const bool getIsInside() { return isInside_; }

	/// returns the centerpoint of a voxel at its virtual location
	BoostPoint3D getCenterPoint() { return center_; }
	/// returns the centerpoint of a voxel at its virtual location in OCCT format
	gp_Pnt getOCCTCenterPoint() { return gp_Pnt(center_.get<0>(), center_.get<1>(), center_.get<2>()); }

	/// returns the intersecting flag
	bool getIsIntersecting() { return isIntersecting_; }
	/// sets the intersecting flag
	void isIntersecting() { isIntersecting_ = true; }

	/// returns the building number
	int getBuildingNum() { return buildingNum_; }

	/// internalize an object, used to internalize the objects the voxel intersects with
	void addInternalProduct(const Value& prodValue) { internalProducts_.emplace_back(prodValue); }
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

	/// sets the roomnum the voxel is related to
	void setRoomNum(int roomNum) { roomNum_ = roomNum; }
	/// get the roomnum the voxel is related to
	int getRoomNum() { return roomNum_; }

	/// set voxel face to roof
	void addRoofSemantic(int indx);
	/// set voxel face to window
	void addWindowSemantic(int indx);

	/// check if voxel has face with window semantic flag
	bool hasWindow(int dirNum = -1);
};
#endif // VOXEL_VOXEL_H
