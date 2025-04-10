#include "helper.h"
#include "voxel.h"
#include "IOManager.h"

#include <gp_Pnt.hxx>

#include <ifcparse/IfcFile.h>
#include <ifcgeom_schema_agnostic/Kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>
#include <ifcparse/IfcHierarchyHelper.h>

#include <BRepClass3d_SolidClassifier.hxx>

#include <thread>   

std::vector<gp_Pnt> voxel::getPotentialMeshObjectPoints(const std::vector<gp_Pnt>& voxelPoints, IfcProductSpatialData& lookup)
{
	std::vector<Value> qResultList;
	lookup.getIndxPointer()->query(bgi::intersects(helperFunctions::createBBox(voxelPoints)), std::back_inserter(qResultList));
	std::vector<gp_Pnt> productPoints;
	for (const Value& qResult : qResultList)
	{
		MeshTriangle triangle = lookup.getProductTriangleList()[qResult.second];
		const std::vector<gp_Pnt> trianglePoints = triangle.getPoints();
		for (const gp_Pnt& currentPoint : trianglePoints)
		{
			productPoints.emplace_back(currentPoint);
		}
	}
	return productPoints;
}

bool voxel::linearEqIntersection(const std::vector<gp_Pnt>& productPoints, const std::vector<gp_Pnt>& voxelPoints)
{
	gp_Pnt p1 = voxelPoints[0];
	gp_Pnt p2 = voxelPoints[1];
	gp_Pnt p4 = voxelPoints[4];

	for (const gp_Pnt& currentPP : productPoints)
	{
		if (currentPP.X() < p1.X() || currentPP.X() > p4.X()) { continue; }
		if (currentPP.Y() < p1.Y() || currentPP.Y() > p4.Y()) { continue; }
		if (currentPP.Z() < p1.Z() || currentPP.Z() > p4.Z()) { continue; }

		return true;
	}
	return false;
}


bool voxel::voxelCoreIsInShape(const gp_Pnt& centerPoint, IfcProductSpatialData& lookup)
{
	// add check if voxel falls completely in the shape
	gp_Pnt offsetPoint = gp_Pnt(centerPoint.X(), centerPoint.Y(), centerPoint.Z() + 100);
	int counter = 0;

	std::vector<Value> qResultList;
	lookup.getIndxPointer()->query(bgi::intersects(helperFunctions::createBBox(centerPoint, offsetPoint)), std::back_inserter(qResultList));

	for (const Value& qResult : qResultList)
	{
		MeshTriangle triangle = lookup.getProductTriangleList()[qResult.second];
		const std::vector<gp_Pnt> trianglePoints = triangle.getPoints();
		if (helperFunctions::triangleIntersecting({ centerPoint,  offsetPoint }, trianglePoints))
		{
			counter++;
		}
	}
	if (counter % 2 == 1)
	{
		isIntersecting_ = true;
		return true;
	}
	return false;
}

bool voxel::productEdgeIntersectsVoxel(const std::vector<gp_Pnt>& voxelPoints, const std::vector<gp_Pnt>& productPoints, int intersectionLogic)
{
	// check if any object edges itersect with the voxel
	std::vector<std::vector<int>> triangleVoxels;
	if (intersectionLogic == 2) { triangleVoxels = getplaneTriangles(); }
	else if (intersectionLogic == 3) { triangleVoxels = getVoxelTriangles(); }

	double voxelLowZ = voxelPoints[0].Z();
	double voxelTopZ = voxelPoints[4].Z();

	double voxelLowX = voxelPoints[0].X();
	double voxelMaxX = voxelPoints[4].X();

	double voxelLowY = voxelPoints[0].Y();
	double voxelMaxY = voxelPoints[4].Y();

	for (const auto& boxel : triangleVoxels)
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[boxel[0]], voxelPoints[boxel[1]], voxelPoints[boxel[2]] };

		for (size_t i = 0; i < productPoints.size(); i += 3)
		{
			// first check if voxel falls within the bounding box of the edge
			gp_Pnt p1 = productPoints[i + 0];
			gp_Pnt p2 = productPoints[i + 1];
			gp_Pnt p3 = productPoints[i + 2];
			double p1z = p1.Z();
			double p2z = p2.Z();
			double p3z = p3.Z();

			double maxZ = std::max({ p1z, p2z, p3z });
			double minZ = std::min({ p1z, p2z, p3z });

			if (minZ > voxelTopZ || maxZ < voxelLowZ) { continue; }

			double p1x = p1.X();
			double p2x = p2.X();
			double p3x = p3.X();

			double maxX = std::max({ p1x, p2x, p3x });
			double minX = std::min({ p1x, p2x, p3x });

			if (minX > voxelMaxX || maxX < voxelLowX) { continue; }

			double p1y = p1.Y();
			double p2y = p2.Y();
			double p3y = p3.Y();

			double maxY = std::max({ p1y, p2y, p3y });
			double minY = std::min({ p1y, p2y, p3y });

			if (minY > voxelMaxY || maxY < voxelLowY) { continue; }

			std::vector<std::vector<gp_Pnt>> lineList = {
				{p1, p2},
				{p2, p3},
				{p3, p1}
			};

			for (std::vector<gp_Pnt> line : lineList)
			{
				if (helperFunctions::triangleIntersecting(line, voxelTriangle))
				{
					isIntersecting_ = true;
					return true;
				}
			}
		}
	}
	return false;
}

bool voxel::voxelEdgeIntersectsProduct(const std::vector<gp_Pnt>& voxelPoints, const std::vector<gp_Pnt>& productPoints, int intersectionLogic)
{
	// check with triangulated object
	std::vector<std::vector<int>> vets;
	if (intersectionLogic == 2) { vets = getPlaneEdges(); }
	else if (intersectionLogic == 3) { vets = getVoxelEdges(); }

	for (size_t i = 0; i < productPoints.size(); i += 3)
	{
		for (size_t j = 0; j < vets.size(); j++)
		{
			gp_Pnt p1 = productPoints[i + 0];
			gp_Pnt p2 = productPoints[i + 1];
			gp_Pnt p3 = productPoints[i + 2];
			if (helperFunctions::triangleIntersecting(
				{ voxelPoints[vets[j][0]], voxelPoints[vets[j][1]] },
				{ p1, p2, p3 })
				)
			{
				isIntersecting_ = true;
				return true;
			}
		}
	}
	return false;
}


voxel::voxel(const BoostPoint3D& center, double sizeXY, double sizeZ)
{
	sizeXY_ = sizeXY;
	sizeZ_ = sizeZ;
	center_ = center;

	double offsetXY = sizeXY / 2;
	double offsetZ = sizeZ / 2;

	gp_Pnt minPoint(bg::get<0>(center) - offsetXY, bg::get<1>(center) - offsetXY, bg::get<2>(center) - offsetZ);
	gp_Pnt maxPoint(bg::get<0>(center) + offsetXY, bg::get<1>(center) + offsetXY, bg::get<2>(center) + offsetZ);
}

bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	double offsetXY = sizeXY_ / 2;
	double offsetZ = sizeZ_ / 2;

	BoostPoint3D lll(bg::get<0>(center_) - offsetXY, bg::get<1>(center_) - offsetXY, bg::get<2>(center_) - offsetZ);
	BoostPoint3D urr(bg::get<0>(center_) + offsetXY, bg::get<1>(center_) + offsetXY, bg::get<2>(center_) + offsetZ);

	return bg::model::box<BoostPoint3D>(lll, urr);
}

std::vector<gp_Pnt> voxel::getCornerPoints()
{
	auto boxelGeo = getVoxelGeo();

	auto minPoint = helperFunctions::Point3DBTO(boxelGeo.min_corner());
	auto maxPoint = helperFunctions::Point3DBTO(boxelGeo.max_corner());

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	std::vector<gp_Pnt> pointList;
	pointList.emplace_back(minPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), maxPoint.Z());

	return pointList;
}

std::vector<gp_Pnt> voxel::getPlanePoints()
{
	auto boxelGeo = getVoxelGeo();

	auto minPoint = helperFunctions::Point3DBTO(boxelGeo.min_corner());
	auto maxPoint = helperFunctions::Point3DBTO(boxelGeo.max_corner());

	std::vector<gp_Pnt> pointList;
	double offset = sizeXY_ / 2;
	// x plane
	pointList.emplace_back(minPoint.X() + offset, minPoint.Y(), minPoint.Z());
	pointList.emplace_back(minPoint.X() + offset, minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X() + offset, maxPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X() + offset, maxPoint.Y(), minPoint.Z());
	// y plane
	pointList.emplace_back(minPoint.X(), minPoint.Y() + offset, minPoint.Z());
	pointList.emplace_back(maxPoint.X(), minPoint.Y() + offset, minPoint.Z());
	pointList.emplace_back(maxPoint.X(), minPoint.Y() + offset, maxPoint.Z());
	pointList.emplace_back(minPoint.X(), minPoint.Y() + offset, maxPoint.Z());
	// z plane
	pointList.emplace_back(minPoint.X(), minPoint.Y(), minPoint.Z() + offset);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z() + offset);
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z() + offset);
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z() + offset);

	return pointList;
}


bool voxel::checkIntersecting(IfcProductSpatialData& lookup, const std::vector<gp_Pnt>& voxelPoints, const gp_Pnt& centerPoint, int intersectionLogic)
{
	if (!voxelPoints.size()) { return false; }

	// add check if voxel falls completely in the shape
	if (voxelCoreIsInShape(centerPoint, lookup))
	{
		return true;
	}

	std::vector<gp_Pnt> productPoints = getPotentialMeshObjectPoints(voxelPoints, lookup);
	if (productPoints.size() == 0)
	{
		return false;
	}

	// check if any product triangle point falls within the voxel
	if (intersectionLogic == 3) // not required if the intersection is not volumetric
	{
		if (linearEqIntersection(productPoints, voxelPoints))
		{
			isIntersecting_ = true;
			return true;
		}
	}

	if (productEdgeIntersectsVoxel(voxelPoints, productPoints, intersectionLogic))
	{
		return true;
	}

	if (voxelEdgeIntersectsProduct(voxelPoints, productPoints, intersectionLogic))
	{
		return true;
	}

	return false;
}


const std::vector<std::vector<int>>& voxel::getVoxelTriangles()
{
	static const std::vector<std::vector<int>> voxelTriangles = {
	{ 0, 1, 5 }, // side    
	{ 0, 5, 6 },
	{ 1, 2, 4 },
	{ 1, 4, 5 },
	{ 2, 3, 7 },
	{ 2, 7, 4 },
	{ 3, 0, 6 },
	{ 3, 6, 7 },
	{ 6, 5, 4 }, // top
	{ 6, 4, 7 },
	{ 0, 3, 2 }, // bottom
	{ 0, 2, 1 }
	};
	return voxelTriangles;
}

const std::vector<std::vector<int>>& voxel::getVoxelFaces()
{
	static const std::vector<std::vector<int>> voxelFaces = {
		{ 1, 2, 4, 5 },
		{ 3, 0, 6, 7 },
		{ 2, 3, 7, 4 },
		{ 0, 1, 5, 6 },
		{ 6, 5, 4, 7 }, // top
		{ 0, 3, 2, 1 }
	};
	return voxelFaces;
}

const std::vector<std::vector<int>>& voxel::getVoxelEdges()
{
	static const std::vector<std::vector<int>> voxelEdges = {
		{ 0, 1},
		{ 1, 2},
		{ 2, 3},
		{ 3, 0},
		{ 4, 5},
		{ 5, 6},
		{ 6, 7},
		{ 7, 4},
		{ 1, 5},
		{ 2, 4},
		{ 3, 7},
		{ 0, 6}
	};
	return voxelEdges;
}


const std::vector<std::vector<int>>& voxel::getplaneTriangles()
{
	static const std::vector<std::vector<int>> planeTriangles = {
		{0, 1, 3},
		{1, 2, 3},
		{4, 5, 7},
		{5, 6, 7},
		{8, 9, 11},
		{9, 10, 11}
	};
	return planeTriangles;
}

const std::vector<std::vector<int>>& voxel::getPlaneEdges()
{
	static const std::vector<std::vector<int>> planeEdges {
		{ 0, 1},
		{ 1, 2},
		{ 2, 3},
		{ 3, 0},
		{ 4, 5},
		{ 5, 6},
		{ 6, 7},
		{ 7, 4},
		{ 8, 9},
		{ 9, 10},
		{ 10, 11},
		{ 11, 8}
	};
	return planeEdges;
}


bool voxel::hasFace(int dirNum)
{
	if (dirNum == -1) // check if there is any face
	{
		if (!hasFace0_ && !hasFace1_ && !hasFace2_ && !hasFace3_ && !hasFace4_ && !hasFace5_) { return false; }
		return true;
	}

	if (dirNum == 0 && !hasFace0_) { return false; }
	if (dirNum == 1 && !hasFace1_) { return false; }
	if (dirNum == 2 && !hasFace2_) { return false; }
	if (dirNum == 3 && !hasFace3_) { return false; }
	if (dirNum == 4 && !hasFace4_) { return false; }
	if (dirNum == 5 && !hasFace5_) { return false; }

	return true;
}

int voxel::numberOfFaces()
{
	int numFaces = 0;

	if (hasFace0_) { numFaces++; }
	if (hasFace1_) { numFaces++; }
	if (hasFace2_) { numFaces++; }
	if (hasFace3_) { numFaces++; }
	if (hasFace4_) { numFaces++; }
	if (hasFace5_) { numFaces++; }

	return numFaces;
}


void voxel::setTransFace(int dirNum)
{
	if (dirNum < 0 || dirNum > 6)
	{
		throw std::invalid_argument("dirNum arguments must be a value of 0 to 6");
	}

	if (dirNum == 0) { hasFace0_ = true; }
	if (dirNum == 1) { hasFace1_ = true; }
	if (dirNum == 2) { hasFace2_ = true; }
	if (dirNum == 3) { hasFace3_ = true; }
	if (dirNum == 4) { hasFace4_ = true; }
	if (dirNum == 5) { hasFace5_ = true; }
}

void voxel::addRoofSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsRoof();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsRoof();
	faceMap_[indx] = newFace;
	return;
}

void voxel::addOuterCeilingSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsExternalCeiling();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsExternalCeiling();
	faceMap_[indx] = newFace;
	return;
}

void voxel::addWallSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsWall();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsWall();
	faceMap_[indx] = newFace;
	return;
}

void voxel::addWindowSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsWindow();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsWindow();
	faceMap_[indx] = newFace;
	return;
}

void voxel::addDoorSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsDoor();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsDoor();
	faceMap_[indx] = newFace;
	return;
}

void voxel::addGroundSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsGround();
		faceMap_[indx] = currentFace;
		return;
	}
	voxelFace newFace;
	newFace.setIsGround();
	faceMap_[indx] = newFace;
	return;
}

CJObjectID voxel::faceType(int dirNum)
{
	if (faceMap_.size() == 0) { return CJObjectID::CJTypeNone; }

	if (dirNum < 0 || dirNum > 5)
	{
		return CJObjectID::CJTypeNone;
	}
	else {
		if (faceMap_.count(dirNum))
		{
			return faceMap_[dirNum].getType();
		}
		return CJObjectID::CJTypeNone;
	}
}

void voxelFace::setIsDoor()
{
	voxelType_ = CJObjectID::CJTypeDoor;
}

void voxelFace::setIsWall()
{
	voxelType_ = CJObjectID::CJTypeWallSurface;
}

void voxelFace::setIsWindow()
{
	voxelType_ = CJObjectID::CJTypeWindow;
}

void voxelFace::setIsRoof()
{
	voxelType_ = CJObjectID::CJTypeRoofSurface;
}

void voxelFace::setIsExternalCeiling()
{
	voxelType_ = CJObjectID::CJTTypeOuterCeilingSurface;
}

void voxelFace::setIsGround()
{
	voxelType_ = CJObjectID::CJTypeGroundSurface;
}

bool voxelFace::isDoor()
{
	if (voxelType_ == CJObjectID::CJTypeDoor)
	{
		return true;
	}
	return false;
}

bool voxelFace::isWall()
{
	if (voxelType_ == CJObjectID::CJTypeWallSurface)
	{
		return true;
	}
	return false;
}

bool voxelFace::isWindow()
{
	if (voxelType_ == CJObjectID::CJTypeWindow)
	{
		return true;
	}
	return false;
}

bool voxelFace::isRoof()
{
	if (voxelType_ == CJObjectID::CJTypeRoofSurface)
	{
		return true;
	}
	return false;
}

bool voxelFace::isGround()
{
	if (voxelType_ == CJObjectID::CJTypeGroundSurface)
	{
		return true;
	}
	return false;
}

CJObjectID voxelFace::getType()
{
	return voxelType_;
}
