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


bool voxel::checkIntersecting(lookupValue& lookup, const std::vector<gp_Pnt>& voxelPoints, const gp_Pnt& centerPoint, helper* h)
{
	hasEvalIntt_ = true;
	std::vector<std::vector<int>> vets = getVoxelEdges();

	IfcSchema::IfcProduct* product = lookup.getProductPtr();

	std::string productType = product->data().type()->name();

	if (productType == "IfcDoor" || productType == "IfcWindow")
	{
		if (!lookup.hasCBox()) { return false; }
	}

	std::vector<gp_Pnt> productPoints = h->getObjectPoints(product, true);

	// check if any cornerpoints fall inside voxel
	if (linearEqIntersection(productPoints, voxelPoints))
	{
		isIntersecting_ = true;
		return true;
	}

	std::vector<std::vector<int>> triangleVoxels = getVoxelTriangles();

	// check if any object edges itersect with the voxel
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


void VoxelGrid::addVoxel(int indx, helper* h)
{
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	voxel* boxel = new voxel(midPoint, voxelSize_, voxelSize_);

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	auto boxelGeo = boxel->getVoxelGeo();
	std::vector<gp_Pnt> pointList = boxel->getCornerPoints(planeRotation_);

	std::vector<Value> qResult;

	qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
	h->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));

	for (size_t k = 0; k < qResult.size(); k++)
	{
		lookupValue* lookup = h->getLookup(qResult[k].second);
		if (boxel->checkIntersecting(*lookup, pointList, helperFunctions::Point3DBTO(midPoint), h))
		{
			boxel->addInternalProduct(qResult[k]);
		}
	}

	std::unique_lock<std::mutex> voxelWriteLock(voxelLookupMutex);
	VoxelLookup_.emplace(indx, boxel);
	voxelWriteLock.unlock();
	return;
}

void VoxelGrid::addVoxelPool(int beginIindx, int endIdx, helper* h, int* voxelGrowthCount)
{
	std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex);
	voxelCountLock.unlock();

	for (int i = beginIindx; i < endIdx; i++) {
		addVoxel(i, h);
		std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex);
		(*voxelGrowthCount)++;
		voxelCountLock.unlock();
	}
}


VoxelGrid::VoxelGrid(helper* h, double voxelSize)
{
	voxelSize_ = voxelSize;
	voxelSizeZ_ = voxelSize;
	anchor_ = h->getLllPoint();
	gp_Pnt urrPoints = h->getUrrPoint();

	// resize to allow full voxel encapsulation
	anchor_.SetX(anchor_.X() - (voxelSize_ * 2));
	anchor_.SetY(anchor_.Y() - (voxelSize_ * 2));
	anchor_.SetZ(anchor_.Z() - (voxelSize_ * 2));

	urrPoints.SetX(urrPoints.X() + (voxelSize_ * 2));
	urrPoints.SetY(urrPoints.Y() + (voxelSize_ * 2));
	urrPoints.SetZ(urrPoints.Z() + (voxelSize_ * 2));

	// set range
	double xRange = urrPoints.X() - anchor_.X();
	double yRange = urrPoints.Y() - anchor_.Y();
	double zRange = urrPoints.Z() - anchor_.Z();

	xRelRange_ = (int)ceil(xRange / voxelSize_) + 1;
	yRelRange_ = (int)ceil(yRange / voxelSize_) + 1;
	zRelRange_ = (int)ceil(zRange / voxelSize_) + 1;

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment_ = std::vector<int>(totalVoxels_, 0);

	planeRotation_ = h->getRotation();

	if (false)
	{
		std::cout << "cluster debug:" << std::endl;

		std::cout << anchor_.X() << std::endl;
		std::cout << anchor_.Y() << std::endl;
		std::cout << anchor_.Z() << std::endl;

		std::cout << xRange << std::endl;
		std::cout << yRange << std::endl;
		std::cout << zRange << std::endl;

		std::cout << xRelRange_ << std::endl;
		std::cout << yRelRange_ << std::endl;
		std::cout << zRelRange_ << std::endl;

		std::cout << totalVoxels_ << std::endl;
	}

	std::cout << "- Populate Grid" << std::endl;
	populatedVoxelGrid(h);

	std::cout << "- Exterior space growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting()) //TODO: improve this
		{
			exteriorVoxelsIdx_ = growExterior(i, 0, h);
			break;
		}
	}

	std::cout << std::endl;
	if (exteriorVoxelsIdx_.size() == 0)
	{
		std::cout << "No exterior space has been found" << std::endl;
	}
	std::cout << "\tExterior space succesfully grown" << std::endl;

	std::cout << "- Pair voxels" << std::endl;
	int buildingNum = 0;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (VoxelLookup_[i]->getIsIntersecting() && VoxelLookup_[i]->getBuildingNum() == -1)
		{
			markVoxelBuilding(i, buildingNum);
			buildingNum++;
		}
	}
	std::cout << "\tVoxel pairing succesful" << std::endl;
	std::cout << "\t" << buildingNum << " buildings(s) found" << std::endl << std::endl;
}

void VoxelGrid::populatedVoxelGrid(helper* h)
{
	// split the range over cores
	int coreCount = std::thread::hardware_concurrency();
	int coreUse = coreCount - 1;
	int splitListSize = floor(totalVoxels_ / coreUse);
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;

	for (size_t i = 0; i < coreUse; i++)
	{
		int beginIdx = i * splitListSize;
		int endIdx = (i + 1) * splitListSize;

		if (i == coreUse - 1) { endIdx = totalVoxels_; }

		threadList.emplace_back([=, &voxelsGrown]() {
			addVoxelPool(beginIdx, endIdx, h, &voxelsGrown);
			});
	}

	std::thread countThread([&]() { countVoxels(&voxelsGrown); });

	for (size_t i = 0; i < threadList.size(); i++) {
		if (threadList[i].joinable()) { threadList[i].join(); }
	}
	if (countThread.joinable()) { countThread.join(); }

	std::cout << std::endl;
}

void VoxelGrid::countVoxels(const int* voxelGrowthCount)
{
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
		std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex);
		int count = *voxelGrowthCount;
		voxelCountLock.unlock();

		std::cout.flush();
		std::cout << "\t" << count << " of " << totalVoxels_ << "\r";

		if (count == totalVoxels_) { return; }
	}
}


std::vector<voxel*> VoxelGrid::getIntersectingVoxels()
{
	std::vector<voxel*> intersectingVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		voxel* currentVoxel = i->second;

		if (!currentVoxel->getIsIntersecting()) { continue; }
		if (!currentVoxel->getBuildingNum() == -1) { continue; }

		intersectingVoxels.emplace_back(currentVoxel);
	}
	return intersectingVoxels;
}


std::vector<voxel*> VoxelGrid::getExternalVoxels()
{
	std::vector<voxel*> externalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		voxel* currentVoxel = i->second;

		if (currentVoxel->getIsInside()) { continue; }

		externalVoxels.emplace_back(currentVoxel);
	}
	return externalVoxels;
}

std::vector<voxel*> VoxelGrid::getInternalVoxels()
{
	std::vector<voxel*> internalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		voxel* currentVoxel = i->second;

		if (!currentVoxel->getIsInside()) { continue; }

		internalVoxels.emplace_back(currentVoxel);
	}
	return internalVoxels;
}


std::vector<voxel*> VoxelGrid::getVoxels()
{
	std::vector<voxel*> externalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		voxel* currentVoxel = i->second;
		externalVoxels.emplace_back(currentVoxel);
	}
	return externalVoxels;
}

std::vector<std::vector<TopoDS_Edge>> VoxelGrid::getDirectionalFaces(int dirIndx, double angle)
{
	std::vector<std::vector<TopoDS_Edge>> clusteredEdges = {};

	std::vector<int> evaluated(VoxelLookup_.size());
	std::vector<int> allowedNeighbourDir = {};

	if (dirIndx == 0 || dirIndx == 1) { allowedNeighbourDir = { 2,3,4,5 }; }
	if (dirIndx == 2 || dirIndx == 3) { allowedNeighbourDir = { 0,1,4,5 }; }
	if (dirIndx == 4 || dirIndx == 5) { allowedNeighbourDir = { 0,1,2,3 }; }

	while (true)
	{
		// find the start of the face growth
		std::vector<int> buffer = {};

		for (size_t i = 0; i < VoxelLookup_.size(); i++)
		{
			voxel* potentialVoxel = VoxelLookup_[i];

			if (evaluated[i] == 1) { continue; }
			if (!potentialVoxel->getIsIntersecting()) { continue; }
			if (!potentialVoxel->hasFace(dirIndx)) { continue; }

			buffer.emplace_back(i);
			evaluated[i] = 1;
			break;
		}

		if (!buffer.size()) { break; }

		// find edges for faces
		std::vector<TopoDS_Edge> edgeList = {};

		while (buffer.size())
		{
			// growth
			std::vector<int> tempBuffer = {};
			for (auto bufferIT = buffer.begin(); bufferIT != buffer.end(); bufferIT++)
			{
				int bufferIndx = *bufferIT;
				voxel* currentVoxel = VoxelLookup_[bufferIndx];

				bool isEdge = false;

				std::vector<int> neighbourIndxList = getDirNeighbours(bufferIndx);
				for (size_t i = 0; i < 4; i++)
				{
					int neighbourIndx = neighbourIndxList[allowedNeighbourDir[i]];
					if (neighbourIndx == -1) { continue; }
					voxel* neighbourVoxel = VoxelLookup_[neighbourIndx];

					// find neighbour to grow into 
					if (neighbourVoxel->hasFace(dirIndx)) {
						if (evaluated[neighbourIndx] == 1) { continue; }
						evaluated[neighbourIndx] = 1;
						tempBuffer.emplace_back(neighbourIndx);
						continue;
					}

					std::vector<gp_Pnt> voxelPoints = currentVoxel->getCornerPoints(planeRotation_);
					// create edge based on the normal dir
					if (dirIndx == 0)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[5], voxelPoints[1]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[2], voxelPoints[4]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[1], voxelPoints[2]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[4], voxelPoints[5]));
						}
					}
					else if (dirIndx == 1)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[0], voxelPoints[6]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[7], voxelPoints[3]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[3], voxelPoints[0]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[6], voxelPoints[7]));
						}
					}
					else if (dirIndx == 2)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[3], voxelPoints[7]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[4], voxelPoints[2]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[2], voxelPoints[3]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[7], voxelPoints[4]));
						}
					}
					else if (dirIndx == 3)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[6], voxelPoints[0]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[1], voxelPoints[5]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[0], voxelPoints[1]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[5], voxelPoints[6]));
						}
					}
					else if (dirIndx == 4)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[7], voxelPoints[6]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[5], voxelPoints[4]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[5], voxelPoints[6]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[4], voxelPoints[7]));
						}
					}
					else if (dirIndx == 5)
					{
						if (i == 0)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[0], voxelPoints[3]));
						}
						else if (i == 1)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[1], voxelPoints[2]));
						}
						else if (i == 2)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[0], voxelPoints[1]));
						}
						else if (i == 3)
						{
							edgeList.emplace_back(BRepBuilderAPI_MakeEdge(voxelPoints[2], voxelPoints[3]));
						}
					}
				}
			}
			buffer = tempBuffer;
		}
		clusteredEdges.emplace_back(edgeList);
	}
	return clusteredEdges;
}


std::vector<int> VoxelGrid::getTopBoxelIndx() {

	std::vector<int> voxelIndx;

	for (size_t i = xRelRange_ * yRelRange_ * zRelRange_ - xRelRange_ * yRelRange_; 
		i < xRelRange_ * yRelRange_ * zRelRange_ - 1; 
		i++)
	{
		voxelIndx.emplace_back(i);
	}
	return voxelIndx;
}


std::vector<voxel*> VoxelGrid::getVoxelPlate(double platelvl) {
	double voxelCount = VoxelLookup_.size();
	double zlvls = voxelCount / (xRelRange_ * yRelRange_);
	double smallestDistanceToLvl = 999999;

	int plateVoxelLvl;

	for (size_t i = 0; i < zlvls; i++)
	{
		voxel v = *VoxelLookup_[i * xRelRange_ * yRelRange_];

		double coreHeight = v.getCenterPoint().get<2>();
		double distanceToLvl = abs(platelvl - coreHeight);

		if (distanceToLvl < smallestDistanceToLvl)
		{
			smallestDistanceToLvl = distanceToLvl;
			plateVoxelLvl = i;
			continue;
		}
		break;
	}

	int lvl = plateVoxelLvl * xRelRange_ * yRelRange_;
	int topLvL = (plateVoxelLvl + 1) * xRelRange_ * yRelRange_ - 1;

	std::vector<voxel*> plateVoxels;

	for (size_t i = 0; i < VoxelLookup_.size(); i++)
	{
		int currentVoxelIdx = i;
		if (currentVoxelIdx < lvl || currentVoxelIdx > topLvL) { continue; }
		plateVoxels.emplace_back(VoxelLookup_[currentVoxelIdx]);
	}
	return plateVoxels;
}


template<typename T>
T VoxelGrid::linearToRelative(int i) {
	int x = i % xRelRange_;
	int y = floor((i / xRelRange_) % yRelRange_);
	int z = floor(i / (xRelRange_ * yRelRange_));

	return T(x, y, z);
}


int VoxelGrid::relativeToLinear(const BoostPoint3D& p) {
	int x = static_cast<int>(std::round(p.get<0>()));
	int y = static_cast<int>(std::round(p.get<1>()));
	int z = static_cast<int>(std::round(p.get<2>()));

	int i = z * xRelRange_ * yRelRange_ +  y * xRelRange_ + x;

	return i;
}


std::vector<int> VoxelGrid::getNeighbours(int voxelIndx, bool connect6)
{
	std::vector<int> neightbours;
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool xSmall = loc3D.X() - 1 >= 0;
	bool xBig = loc3D.X() + 1 < xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 < yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 < zRelRange_;

	// connectivity
	if (xSmall)
	{
		neightbours.emplace_back(voxelIndx - 1);

		if (!connect6)
		{
			if (ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
			if (yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ - 1); }
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - 1); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - 1); }
		}

	}
	if (xBig)
	{
		neightbours.emplace_back(voxelIndx + 1);

		if (!connect6)
		{
			if (ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ + 1); }
			if (yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ + 1); }
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + 1); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + 1); }
		}
	}
	if (ySmall)
	{
		neightbours.emplace_back(voxelIndx - xRelRange_);
		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - xRelRange_); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - xRelRange_); }
		}
	}
	if (yBig)
	{
		neightbours.emplace_back(voxelIndx + xRelRange_);
		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + xRelRange_); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + xRelRange_); }
		}
	}

	if (zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)); }
	if (zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)); }

	if (connect6)
	{
		return neightbours;
	}

	if (xSmall && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
	if (xBig && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ + 1); }

	if (xBig && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ + 1); }
	if (xSmall && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ - 1); }

	if (xSmall && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)-xRelRange_ - 1); }
	if (xBig && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)-xRelRange_ + 1); }

	if (xSmall && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)+xRelRange_ - 1); }
	if (xBig && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)+xRelRange_ + 1); }

	if (xSmall && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)+xRelRange_ - 1); }
	if (xBig && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)+xRelRange_ + 1); }

	if (xSmall && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)-xRelRange_ - 1); }
	if (xBig && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)-xRelRange_ + 1); }

	return neightbours;
}

std::vector<int> VoxelGrid::getDirNeighbours(int voxelIndx)
{
	std::vector<int> neightbours;
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool xSmall = loc3D.X() - 1 >= 0;
	bool xBig = loc3D.X() + 1 < xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 < yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 < zRelRange_;

	// connectivity
	if (xSmall) { neightbours.emplace_back(voxelIndx - 1); }
	else { neightbours.emplace_back(-1); }
	if (xBig) { neightbours.emplace_back(voxelIndx + 1); }
	else { neightbours.emplace_back(-1); }
	if (ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_); }
	else { neightbours.emplace_back(-1); }
	if (yBig) { neightbours.emplace_back(voxelIndx + xRelRange_); }
	else { neightbours.emplace_back(-1); }
	if (zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)); }
	else { neightbours.emplace_back(-1); }
	if (zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_)); }
	else { neightbours.emplace_back(-1); }

	return neightbours;

}

std::vector<TopoDS_Edge> VoxelGrid::getTransitionalEdges(int dirIndx, int voxelIndx)
{

	return std::vector<TopoDS_Edge>();
}

std::vector<int> VoxelGrid::getNeighbours(voxel* boxel, bool connect6)
{
	BoostPoint3D middlePoint = boxel->getCenterPoint();
	BoostPoint3D relativePoint = worldToRelPoint(middlePoint);
	int voxelInt = relativeToLinear(relativePoint);
	return getNeighbours(voxelInt, connect6);
}

int VoxelGrid::getLowerNeighbour(int voxelIndx, bool connect6)
{
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool zSmall = loc3D.Z() - 1 >= 0;

	if (!zSmall) { return -1; }

	return voxelIndx - (xRelRange_) * (yRelRange_);

}

BoostPoint3D VoxelGrid::relPointToWorld(const BoostPoint3D& p)
{
	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize_) + voxelSize_ / 2;
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize_) + voxelSize_ / 2;
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSize_) + voxelSize_ / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}


BoostPoint3D VoxelGrid::worldToRelPoint(BoostPoint3D p)
{
	double xCoord = (bg::get<0>(p) - anchor_.X() - voxelSize_ / 2) / voxelSize_;
	double yCoord = (bg::get<1>(p) - anchor_.Y() - voxelSize_ / 2) / voxelSize_;
	double zCoord = (bg::get<2>(p) - anchor_.Z() - voxelSize_ / 2) / voxelSize_;

	return BoostPoint3D(xCoord, yCoord, zCoord );
}


std::vector<int> VoxelGrid::growExterior(int startIndx, int roomnum, helper* h)
{
	std::vector<int> buffer = { startIndx };
	std::vector<int> totalRoom = { startIndx };
	Assignment_[startIndx] = 1;

	bool isOutSide = false;

	while (buffer.size() > 0)
	{
		std::vector<int> tempBuffer;
		for (size_t j = 0; j < buffer.size(); j++)
		{
			if (totalRoom.size() % 1000 == 0)
			{
				std::cout.flush();
				std::cout << "\tSize: " << totalRoom.size() << "\r";
			}

			int currentIdx = buffer[j];

			// find neighbours
			std::vector<int> neighbourIndxList = getDirNeighbours(currentIdx);

			for (size_t k = 0; k < neighbourIndxList.size(); k++)
			{
				int neighbourIdx = neighbourIndxList[k];

				if (neighbourIdx == -1) 
				{
					isOutSide = true;
					continue; 
				}

				voxel* neighbourVoxel = VoxelLookup_[neighbourIdx];

				if (neighbourVoxel->getIsIntersecting())
				{
					neighbourVoxel->setIsShell();
					neighbourVoxel->setTransFace(k);
					continue;
				}

				// exlude if already assigned
				if (Assignment_[neighbourIdx] == 0) {
					bool dupli = false;
					for (size_t l = 0; l < tempBuffer.size(); l++)
					{
						// exlude if already in buffer
						if (neighbourIdx == tempBuffer[l])
						{
							dupli = true;
							break;
						}
					}
					if (!dupli)
					{
						tempBuffer.emplace_back(neighbourIdx);
						totalRoom.emplace_back(neighbourIdx);
						Assignment_[neighbourIdx] = 1;
					}
				}
				else if (Assignment_[neighbourIdx] == -1) {
					bool dupli = false;

					for (size_t l = 0; l < totalRoom.size(); l++)
					{
						if (neighbourIdx == totalRoom[l]) {
							dupli = true;
						}
					}
					if (!dupli)
					{
						totalRoom.emplace_back(neighbourIdx);
						tempBuffer.emplace_back(neighbourIdx);
					}
				}
			}
		}
		buffer.clear();
		buffer = tempBuffer;
	}
	if (isOutSide) //TODO: this should be smarter
	{
		std::vector<int> exterior;

		for (size_t k = 0; k < totalRoom.size(); k++)
		{
			int currentIdx = totalRoom[k];
			voxel* currentBoxel = VoxelLookup_[currentIdx];
			if (!currentBoxel->getIsIntersecting())
			{
				currentBoxel->setOutside();
				exterior.emplace_back(currentIdx);
			}
		}
		return exterior;
	}
	return {};
}

void VoxelGrid::markVoxelBuilding(int startIndx, int buildnum) {

	VoxelLookup_[startIndx]->setBuildingNum(buildnum);
	std::vector<int> buffer = { startIndx };
	std::vector<int> potentialBuildingVoxels;

	while (true)
	{
		for (size_t i = 0; i < buffer.size(); i++)
		{
			int currentIdx = buffer[i];
			voxel* currentVoxel = VoxelLookup_[currentIdx];

			std::vector<int> neighbours = getNeighbours(currentIdx);

			for (size_t j = 0; j < neighbours.size(); j++)
			{
				int otherIdx = neighbours[j];
				voxel* otherVoxel = VoxelLookup_[otherIdx];

				if (!otherVoxel->getIsIntersecting()) { continue; }
				if (otherVoxel->getBuildingNum() != -1) { continue; }

				otherVoxel->setBuildingNum(buildnum);
				potentialBuildingVoxels.emplace_back(otherIdx);
			}
		}

		if (potentialBuildingVoxels.size() == 0)
		{
			break;
		}
		buffer = potentialBuildingVoxels;
		potentialBuildingVoxels.clear();
	}
	return;
}

