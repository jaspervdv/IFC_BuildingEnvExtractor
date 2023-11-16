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
	int buildingNum_ = -1;
	int roomNum_ = -1;

	std::vector<Value> internalProducts_;

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

	/// set flag notifying if the voxel is part of the outside of a building 
	void setOutside() { isInside_ = false; }

	/// returns flag representing if the voxel is part of the interior of a building
	bool getIsInside() { return isInside_; }

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
};
#endif // VOXEL_VOXEL_H
