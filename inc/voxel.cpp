#include "helper.h"
#include "voxel.h"
#include "ioManager.h"

#include <gp_Pnt.hxx>

#include <ifcparse/IfcFile.h>
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
	pointList.emplace_back(minPoint.X(), minPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z()+ offset);
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z()+ offset);

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
	TopoDS_Shape productShape = lookup.getProductShape();

	if (productType == "IfcDoor" || productType == "IfcWindow") // only use simplefied opening geo
	{
		if (!lookup.hasCBox()) { return false; }
		productShape = lookup.getCBox();
	}

	if (!lookup.getSimpleShape().IsNull())
	{
		productShape = lookup.getSimpleShape();
	}

	// check if any cornerpoints fall inside voxel

	std::vector<gp_Pnt> productPoints = lookup.getProductPoints();

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

	double voxelLowZ = voxelPoints[0].Z();
	double voxelTopZ = voxelPoints[4].Z();

	double voxelLowX = voxelPoints[0].X();
	double voxelMaxX = voxelPoints[4].X();

	double voxelLowY = voxelPoints[0].Z();
	double voxelMaxY = voxelPoints[4].Z();

	for (const auto& boxel : triangleVoxels) 
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[boxel[0]], voxelPoints[boxel[1]], voxelPoints[boxel[2]] };

		for (size_t k = 0; k < productPoints.size(); k += 2)
		{
			// first check if voxel falls within the bounding box of the line

			gp_Pnt p1 = productPoints[k]; // line point 1
			gp_Pnt p2 = productPoints[k + 1]; // and 2

			double p1z = p1.Z();
			double p2z = p2.Z();

			double maxZ = std::max(p1z, p2z);
			double minZ = std::min(p1z, p2z);

			if (minZ > voxelTopZ || maxZ < voxelLowZ) { continue; }

			double p1x = p1.X();
			double p2x = p2.X();

			double maxX = std::max(p1x, p2x);
			double minX = std::min(p1x, p2x);

			if (minX > voxelMaxX || maxX < voxelLowX) { continue; }

			double p1y = p1.Y();
			double p2y = p2.Y();

			double maxY = std::max(p1y, p2y);
			double minY = std::min(p1y, p2y);

			if (minY > voxelMaxY || maxY < voxelLowY) { continue; }

			if (helperFunctions::triangleIntersecting({ p1, p2 }, voxelTriangle))
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

	// add check if voxel falls completely in the shape
	gp_Pnt offsetPoint = gp_Pnt(centerPoint.X(), centerPoint.Y(), centerPoint.Z() + 10);
	int counter = 0;

	for (TopExp_Explorer expl(productShape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face productFace = TopoDS::Face(expl.Current());

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(productFace, loc);

		if (mesh.IsNull()) { continue; }

		for (size_t i = 1; i <= mesh.get()->NbTriangles(); i++) //TODO: find out if there is use to keep the opencascade structure
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

			std::vector<gp_Pnt> trianglePoints{
				mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
			};

			for (size_t k = 0; k < vets.size(); k++)
			{
				if (helperFunctions::triangleIntersecting({ voxelPoints[vets[k][0]], voxelPoints[vets[k][1]] }, trianglePoints))
				{
					isIntersecting_ = true;
					return true;
				}
			}

			// check if inside of shape
			if (helperFunctions::triangleIntersecting({ centerPoint,  offsetPoint }, trianglePoints))
			{

				counter++;
			}
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
	gp_Pnt p4 = voxelPoints[4];

	for (size_t i = 0; i < productPoints.size(); i++)
	{
		// check if falls in z domain
		gp_Pnt currentPP = productPoints[i];

		if (currentPP.X() < p1.X() || currentPP.X() > p4.X()) { continue; }
		if (currentPP.Y() < p1.Y() || currentPP.Y() > p4.Y()) { continue; }
		if (currentPP.Z() < p1.Z() || currentPP.Z() > p4.Z()) { continue; }

		return true;
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

void voxel::addRoofSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsRoof();
		faceMap_[indx] = currentFace; //TODO: pointers
		return;
	}
	voxelFace newFace;
	newFace.setIsRoof();
	faceMap_[indx] = newFace; //TODO: pointers
	return;
}

void voxel::addWindowSemantic(int indx)
{
	if (faceMap_.count(indx))
	{
		voxelFace currentFace = faceMap_[indx];
		currentFace.setIsWindow();
		faceMap_[indx] = currentFace; //TODO: pointers
		return;
	}
	voxelFace newFace;
	newFace.setIsWindow();
	faceMap_[indx] = newFace; //TODO: pointers
	return;
}

bool voxel::hasWindow(int dirNum)
{
	if (faceMap_.size() == 0) { return false; }

	if (dirNum == -1)
	{
		for (auto entry = faceMap_.begin(); entry != faceMap_.end(); entry++)
		{
			if (entry->second.isWindow())
			{
				return true;
			}
		}
		return false;
	}
	else {
		if (faceMap_.count(dirNum))
		{
			if (faceMap_[dirNum].isWindow())
			{
				return true;
			}
		}
		return false;
	}



}

void voxelFace::setIsWall()
{
	isWall_ = true;
	isWindow_ = false;
	isRoof_ = false;
}

void voxelFace::setIsWindow()
{
	isWall_ = false;
	isWindow_ = true;
	isRoof_ = false;
}

void voxelFace::setIsRoof()
{
	isWall_ = false;
	isWindow_ = false;
	isRoof_ = true;
}
