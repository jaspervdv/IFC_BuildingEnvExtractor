#include "helper.h"
#include "voxel.h"
#include "ioManager.h"

#include <gp_Pnt.hxx>

#include <ifcparse/IfcFile.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>
#include <ifcgeom_schema_agnostic/Serialization.h>
#include <ifcparse/IfcHierarchyHelper.h>

#include <BRepClass3d_SolidClassifier.hxx>

voxel::voxel(const BoostPoint3D& center, double sizeXY, double sizeZ)
{
	sizeXY_ = sizeXY;
	sizeZ_ = sizeZ;
	center_ = center;

	gp_Pnt minPoint(bg::get<0>(center) - 1 / 2 * sizeXY, bg::get<1>(center) - 1 / 2 * sizeXY, bg::get<2>(center) - 1 / 2 * sizeZ);
	gp_Pnt maxPoint(bg::get<0>(center) + 1 / 2 * sizeXY, bg::get<1>(center) + 1 / 2 * sizeXY, bg::get<2>(center) + 1 / 2 * sizeZ);
}


bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	double offsetXY = sizeXY_ / 2;
	double offsetZ = sizeZ_ / 2;

	BoostPoint3D lll(bg::get<0>(center_) - offsetXY, bg::get<1>(center_) - offsetXY, bg::get<2>(center_) - offsetZ);
	BoostPoint3D urr(bg::get<0>(center_) + offsetXY, bg::get<1>(center_) + offsetXY, bg::get<2>(center_) + offsetZ);

	return bg::model::box<BoostPoint3D>(lll, urr);
}


std::vector<gp_Pnt> voxel::getCornerPoints(double angle)
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

	for (size_t i = 0; i < pointList.size(); i++)
	{
		pointList[i] = helperFunctions::rotatePointWorld(pointList[i], -angle);
	}
	return pointList;
}

std::vector<gp_Pnt> voxel::getPlanePoints(double angle)
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
	pointList.emplace_back(minPoint.X(), minPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z()+ offset);

	for (size_t i = 0; i < pointList.size(); i++)
	{
		pointList[i] = helperFunctions::rotatePointWorld(pointList[i], -angle);
	}
	return pointList;
}


std::vector<std::vector<int>> voxel::getVoxelTriangles()
{
	return {
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
	{ 0, 3, 2 }, // buttom
	{ 0, 2, 1 }
	};
}

std::vector<std::vector<int>> voxel::getplaneTriangles()
{
	return {
		{0, 1, 3},
		{1, 2, 3},
		{4, 5, 7},
		{5, 6, 7},
		{8, 9, 11},
		{9, 10, 11}
	};
}


std::vector<std::vector<int>> voxel::getVoxelFaces()
{
	return {
		{ 1, 2, 4, 5 },
		{ 3, 0, 6, 7 },
		{ 2, 3, 7, 4 },
		{ 0, 1, 5, 6 },
		{ 6, 5, 4, 7 }, // top
		{ 0, 3, 2, 1 }
	};
}


std::vector<std::vector<int>> voxel::getVoxelEdges()
{
	return {
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
}

std::vector<std::vector<int>> voxel::getPlaneEdges()
{
	return {
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
}


bool voxel::checkIntersecting(lookupValue& lookup, const std::vector<gp_Pnt>& voxelPoints, const gp_Pnt& centerPoint, helper* h, bool planeIntersection)
{
	hasEvalIntt_ = true;
	// get the product
	IfcSchema::IfcProduct* product = lookup.getProductPtr();
	std::string productType = product->data().type()->name();
	if (productType == "IfcDoor" || productType == "IfcWindow") // only use simplefied opening geo
	{
		if (!lookup.hasCBox()) { return false; }
	}

	// check if any cornerpoints fall inside voxel
	std::vector<gp_Pnt> productPoints = h->getObjectPoints(product, true);
	if (!planeIntersection) // plane intersection is not volumetric, so not required if used
	{
		if (linearEqIntersection(productPoints, voxelPoints))
		{
			isIntersecting_ = true;
			return true;
		}
	}

	// check if any object edges itersect with the voxel
	std::vector<std::vector<int>> triangleVoxels;
	if (planeIntersection) { triangleVoxels = getplaneTriangles(); }
	else { triangleVoxels = getVoxelTriangles(); }


	for (size_t i = 0; i < triangleVoxels.size(); i++)
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[triangleVoxels[i][0]], voxelPoints[triangleVoxels[i][1]], voxelPoints[triangleVoxels[i][2]] };

		for (size_t k = 0; k < productPoints.size(); k += 2)
		{
			if (helperFunctions::triangleIntersecting({ productPoints[k], productPoints[k + 1] }, voxelTriangle))
			{
				isIntersecting_ = true;
				return true;
			}
		}
	}

	// check with triangulated object
	std::vector<std::vector<int>> vets;
	if (planeIntersection) { vets = getPlaneEdges(); }
	else { vets = getVoxelEdges(); }
	
	std::vector<std::vector<gp_Pnt>>* triangleMesh = lookup.getTriangluatedShape();
	for (size_t i = 0; i < triangleMesh->size(); i++)
	{
		std::vector<gp_Pnt> triangle = triangleMesh->at(i);
		for (size_t k = 0; k < vets.size(); k++)
		{
			if (helperFunctions::triangleIntersecting({ voxelPoints[vets[k][0]], voxelPoints[vets[k][1]] }, triangle))
			{
				isIntersecting_ = true;
				return true;
			}
		}
	}

	// check if voxel is completely inside of object
	gp_Pnt offsetPoint = gp_Pnt(centerPoint.X(), centerPoint.Y(), centerPoint.Z() + 1000);
	int counter = 0;

	for (size_t i = 0; i < triangleMesh->size(); i++)
	{
		std::vector<gp_Pnt> triangle = triangleMesh->at(i);
		if (helperFunctions::triangleIntersecting({ centerPoint,  offsetPoint }, triangle))
		{
			counter++; 
		}
	}
	if (counter%2 == 1)
	{
		isIntersecting_ = true;
		return true;
	}
	return false;
}


bool voxel::linearEqIntersection(const std::vector<gp_Pnt>& productPoints, const std::vector<gp_Pnt>& voxelPoints)
{
	gp_Pnt p1 = voxelPoints[0];
	gp_Pnt p2 = voxelPoints[1];
	gp_Pnt p3 = voxelPoints[3];

	if (p2.Y() - p1.Y() == 0)
	{
		for (size_t i = 0; i < productPoints.size(); i++)
		{
			gp_Pnt currentPP = productPoints[i];

			if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) {
				continue;
			}
			if (currentPP.X() < p1.X() && currentPP.X() < voxelPoints[4].X() ||
				currentPP.X() > p1.X() && currentPP.X() > voxelPoints[4].X()) {
				continue;
			}
			if (currentPP.Y() < p1.Y() && currentPP.Y() < voxelPoints[4].Y() ||
				currentPP.Y() > p1.Y() && currentPP.Y() > voxelPoints[4].Y()) {
				continue;
			}
			return true;
		}
		return false;
	}

	double a1 = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
	double b11 = p2.Y() - a1 * p2.X();
	double b12 = p3.Y() - a1 * p3.X();

	double a2 = -1 / a1;
	double b21 = p3.Y() - a2 * p3.X();
	double b22 = p2.Y() - a2 * p2.X();

	for (size_t i = 0; i < productPoints.size(); i++)
	{
		gp_Pnt currentPP = productPoints[i];

		// check if point is in z range
		if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) { continue; }

		// check if point is in voxel
		double x = currentPP.X();

		double y11 = a1 * x + b11;
		double y12 = a1 * x + b12;

		if (currentPP.Y() < y11 && currentPP.Y() < y12 ||
			currentPP.Y() > y11 && currentPP.Y() > y12) {
			continue;
		}

		double y21 = a2 * x + b21;
		double y22 = a2 * x + b22;

		if (currentPP.Y() < y21 && currentPP.Y() > y22 ||
			currentPP.Y() > y21 && currentPP.Y() < y22)
		{
			return true;
		}
	}
	return false;
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


