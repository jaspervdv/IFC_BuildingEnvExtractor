#include "helper.h"
#include "voxelGrid.h"
#include "voxel.h"
#include "stringManager.h"

#include <algorithm>
#include <execution>
#include <thread>   

bool VoxelGrid::addVoxel(int indx, helper* h, bool checkIfInt)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	gp_Pnt midPointOCCT = helperFunctions::Point3DBTO(midPoint);

	std::shared_ptr<voxel> boxel = std::make_shared<voxel>(voxel(midPoint, settingsCollection.voxelSize(), settingsCollection.voxelSize()));

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	auto boxelGeo = boxel->getVoxelGeo();

	std::vector<gp_Pnt> pointList;
	if (settingsCollection.intersectionLogic() == 2) { pointList = boxel->getPlanePoints(); }
	else if (settingsCollection.intersectionLogic() == 3) { pointList = boxel->getCornerPoints(); }

	bool isIntersecting = false;

	if (checkIfInt)
	{
		std::unordered_set<std::string> productTypesList; // list with types already intersected with
		std::vector<Value> qResult;
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		h->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));
		for (size_t k = 0; k < qResult.size(); k++)
		{
			std::shared_ptr<lookupValue> lookup = h->getLookup(qResult[k].second);
			std::string productType = lookup->getProductPtr()->data().type()->name();

			//if (productTypesList.find(productType) != productTypesList.end()) { continue; }

			if (boxel->checkIntersecting(*lookup, pointList, midPointOCCT, h, settingsCollection.intersectionLogic()))
			{
				productTypesList.insert(productType); //TODO: make this work
				boxel->addInternalProduct(qResult[k]);
				isIntersecting = true;
			}
		}
	}

	std::unique_lock<std::mutex> voxelWriteLock(voxelLookupMutex);
	VoxelLookup_.emplace(indx, boxel);
	voxelWriteLock.unlock();
	return isIntersecting;
}


void VoxelGrid::addVoxelColumn(int beginIindx, int endIdx, helper* h, int* voxelGrowthCount)
{
	std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex);
	voxelCountLock.unlock();

	int plate = (zRelRange_ -1) * xRelRange_  * yRelRange_ ;

	for (int i = beginIindx; i < endIdx; i++)
	{
		int voxelIndx = plate + i;
		bool intersect = true;
		while (true)
		{
			if (
				addVoxel(voxelIndx, h, intersect) &&
				!SettingsCollection::getInstance().requireFullVoxels()
				)
			{
				intersect = false;
			}

			std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex);
			(*voxelGrowthCount)++;
			voxelCountLock.unlock();

			voxelIndx = getLowerNeighbour(voxelIndx);

			if (voxelIndx == -1) { break; }
		}
	}
}

VoxelGrid::VoxelGrid(helper* h)
{
	anchor_ = h->getLllPoint();

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	double voxelSize = settingsCollection.voxelSize();

	gp_Pnt urrPoints = h->getUrrPoint();

	// resize to allow full voxel encapsulation
	anchor_.SetX(anchor_.X() - (voxelSize * 2));
	anchor_.SetY(anchor_.Y() - (voxelSize * 2));
	anchor_.SetZ(anchor_.Z() - (voxelSize * 2));

	urrPoints.SetX(urrPoints.X() + (voxelSize * 2));
	urrPoints.SetY(urrPoints.Y() + (voxelSize * 2));
	urrPoints.SetZ(urrPoints.Z() + (voxelSize * 2));

	// set range
	double xRange = urrPoints.X() - anchor_.X();
	double yRange = urrPoints.Y() - anchor_.Y();
	double zRange = urrPoints.Z() - anchor_.Z();

	xRelRange_ = static_cast<int>(ceil(xRange / voxelSize) + 1);
	yRelRange_ = static_cast<int>(ceil(yRange / voxelSize) + 1);
	zRelRange_ = static_cast<int>((int)ceil(zRange / voxelSize) + 1);

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	planeRotation_ = settingsCollection.gridRotation();

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

	if (!settingsCollection.requireVoxels()) { 
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoNoVoxelizationReq) << std::endl;
		return; 
	} // no voxels needed for lod0.0 and 1.0 only

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoPopulateGrid) << std::endl;
	populatedVoxelGrid(h);

	if (!settingsCollection.requireFullVoxels()) { 
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoNocompleteVoxelizationReq) << std::endl;
		return; 
	}

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoExteriorSpaceGrowing) << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting())
		{
			growExterior(i, 0, h);
			break;
		}
	}

	std::cout << std::endl;
	if (exteriorVoxelsIdx_.size() == 0)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentNoExteriorSpace) << std::endl;
	}
	else
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentExteriorSpaceGrown) << std::endl;
	}


	if (settingsCollection.makeInterior())
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoInterioSpacesGrowing) << std::endl;
		roomSize_ = 1;
		for (int i = 0; i < totalVoxels_; i++)
		{
			if (!VoxelLookup_[i]->getIsIntersecting() && VoxelLookup_[i]->getRoomNum() == -1)
			{
				growExterior(i, roomSize_, h);
				roomSize_++;
			}
		}
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentInteriorSpaceGrown) << std::endl;
	}

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoPairVoxels) << std::endl;
	int buildingNum = 0;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (VoxelLookup_[i]->getIsIntersecting() && VoxelLookup_[i]->getBuildingNum() == -1)
		{
			markVoxelBuilding(i, buildingNum);
			buildingNum++;
		}
	}
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentPairedVoxels) << std::endl;
	std::cout << "\t" << buildingNum << " buildings(s) found" << std::endl << std::endl;

	return;
}

void VoxelGrid::computeSurfaceSemantics(helper* h)
{
	if (hasSemanticSurfaces_) { return; } // if already processed no reprocessing is needed
	hasSemanticSurfaces_ = true;

	// compute external surfaces
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = getIntersectingVoxels();

	for (size_t i = 0; i < intersectingVoxels.size(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = intersectingVoxels[i];

		// find the roofs
		if (currentVoxel->hasFace(4))
		{
			currentVoxel->addRoofSemantic(4);	
		}

		// find the windows
		for (int indxdir = 0; indxdir < 6; indxdir++)
		{
			if (voxelBeamWindowIntersection(h, currentVoxel, indxdir)) //TODO: improve the logic
			{
				currentVoxel->addWindowSemantic(indxdir);

				int cidxDir = helperFunctions::invertDir(indxdir);

				int boundNeighbourIndx =  getNeighbour(currentVoxel, cidxDir);

				if (boundNeighbourIndx == -1) { continue; }

				std::shared_ptr<voxel> boundNeighbour = VoxelLookup_[boundNeighbourIndx];
				boundNeighbour->addWindowSemantic(cidxDir);

			}
		}	
	}
}

void VoxelGrid::populatedVoxelGrid(helper* h)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// split the range over cores
	int coreUse = settingsCollection.threadcount() - 1;
	int loopRange = xRelRange_ * yRelRange_;
	int plateIndx = (zRelRange_ - 1) * xRelRange_ * yRelRange_;

	// compute column scores
	int voxelSize = settingsCollection.voxelSize();

	std::vector<int> columScoreList;
	int columSumScore = 0;
	for (int i = 0; i < loopRange; i++)
	{
		int plate = plateIndx + i;

		BoostPoint3D coneCenter = relPointToWorld(linearToRelative<BoostPoint3D>(plate));
		BoostPoint3D lll = BoostPoint3D( 
			coneCenter.get<0>() - voxelSize /2,
			coneCenter.get<1>() - voxelSize /2,
			-100
		);

		BoostPoint3D urr = BoostPoint3D(
			coneCenter.get<0>() + voxelSize /2,
			coneCenter.get<1>() + voxelSize /2,
			coneCenter.get<2>() + voxelSize /2
		);

		std::vector<Value> qResult;
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		h->getIndexPointer()->query(
			bgi::intersects(
				bg::model::box<BoostPoint3D>(lll, urr)
			), std::back_inserter(qResult)
		);

		int columnScore = static_cast<int>(qResult.size());
		columScoreList.emplace_back(columnScore);
		columSumScore += columnScore;
	}

	int threadScore = columSumScore / coreUse;
	int voxelsGrown = 0;
	int beginIdx = 0;

	std::vector<std::thread> threadList;
	for (int i = 0; i < coreUse; i++)
	{
		int score = 0;
		int endIdx = 0;
		if (i == coreUse - 1) { endIdx = xRelRange_ * yRelRange_; }
		else
		{
			for (int j = beginIdx; j < loopRange; j++)
			{
				score += columScoreList[j];

				if (score > threadScore)
				{
					endIdx = j;
					break;
				}
			}
		}

		threadList.emplace_back([=, &voxelsGrown]() 
			{
			addVoxelColumn(beginIdx, endIdx, h, &voxelsGrown);
			});

		beginIdx = endIdx ;
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


std::vector<std::shared_ptr<voxel>> VoxelGrid::getIntersectingVoxels()
{
	std::vector<std::shared_ptr<voxel>> intersectingVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = i->second;
		if (!currentVoxel->getIsIntersecting()) { continue; }
		if (currentVoxel->getBuildingNum() == -1) { continue; }

		intersectingVoxels.emplace_back(currentVoxel);
	}
	return intersectingVoxels;
}


std::vector<std::shared_ptr<voxel>> VoxelGrid::getExternalVoxels()
{
	std::vector<std::shared_ptr<voxel>> externalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = i->second;

		if (currentVoxel->getIsInside()) { continue; }

		externalVoxels.emplace_back(currentVoxel);
	}
	return externalVoxels;
}

std::vector<std::shared_ptr<voxel>> VoxelGrid::getInternalVoxels()
{
	std::vector<std::shared_ptr<voxel>> internalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = i->second;

		if (!currentVoxel->getIsInside()) { continue; }

		internalVoxels.emplace_back(currentVoxel);
	}
	return internalVoxels;
}


std::vector<std::shared_ptr<voxel>> VoxelGrid::getVoxels()
{
	std::vector<std::shared_ptr<voxel>> externalVoxels;
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = i->second;
		externalVoxels.emplace_back(currentVoxel);
	}
	return externalVoxels;
}

double VoxelGrid::getRoomArea(int roomNum)
{
	double voxelSize = SettingsCollection::getInstance().voxelSize();
	double voxelarea = voxelSize * voxelSize;

	double roomArea = 0;
	for (std::pair<int, std::shared_ptr<voxel>> voxelPair : VoxelLookup_)
	{
		std::shared_ptr<voxel> currentVoxel = voxelPair.second;
		if (currentVoxel->getRoomNum() != roomNum) { continue; }
		if (!currentVoxel->hasFace(5)) { continue; }
		roomArea += voxelarea;
	}
	return roomArea;
}

std::vector<std::vector<TopoDS_Edge>> VoxelGrid::getDirectionalFaces(int dirIndx, double angle, int roomNum)
{
	std::vector<std::vector<TopoDS_Edge>> clusteredEdges = {};

	std::vector<int> evaluated(VoxelLookup_.size());
	std::vector<int> evaluatedGrowth(VoxelLookup_.size());
	std::vector<int> allowedNeighbourDir = {};

	if (dirIndx == 0 || dirIndx == 1) { allowedNeighbourDir = { 2,3,4,5 }; }
	if (dirIndx == 2 || dirIndx == 3) { allowedNeighbourDir = { 0,1,4,5 }; }
	if (dirIndx == 4 || dirIndx == 5) { allowedNeighbourDir = { 0,1,2,3 }; }

	int searchStartIdx = 0;
	while (true)
	{
		// find the start of the face growth
		std::vector<int> buffer = {};
		bool searchWindow = false;

		for (int i = searchStartIdx; i < VoxelLookup_.size(); i++)
		{
			std::shared_ptr<voxel> potentialVoxel = VoxelLookup_[i];

			if (evaluated[i] == 1) { continue; }
			if (potentialVoxel->getRoomNum() != roomNum) { continue; }
			if (!potentialVoxel->hasFace(dirIndx)) { continue; }
			//searchWindow = potentialVoxel->hasWindow(dirIndx);
			buffer.emplace_back(i);
			evaluated[i] = 1;
			evaluatedGrowth[i] = 1;
			searchStartIdx = i;
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
				std::shared_ptr<voxel> currentVoxel = VoxelLookup_[bufferIndx];

				bool isEdge = false;

				std::vector<int> neighbourIndxList = getDirNeighbours(bufferIndx);
				for (size_t i = 0; i < 4; i++)
				{
					int neighbourIndx = neighbourIndxList[allowedNeighbourDir[i]];
					if (neighbourIndx == -1) { continue; }
					std::shared_ptr<voxel> neighbourVoxel = VoxelLookup_[neighbourIndx];

					// find neighbour to grow into 
					if (neighbourVoxel->hasFace(dirIndx)) {
						if (evaluatedGrowth[neighbourIndx] == 1) { continue; }

						//if (neighbourVoxel->hasWindow(dirIndx) == searchWindow)
						//{
							evaluatedGrowth[neighbourIndx] = 1;
							tempBuffer.emplace_back(neighbourIndx);
							continue;
						//}
					}

					std::vector<gp_Pnt> voxelPoints = currentVoxel->getCornerPoints();
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

		for (size_t i = 0; i < evaluatedGrowth.size(); i++)
		{
			if (evaluated[i] == 0 && evaluatedGrowth[i] == 1) { evaluated[i] = 1; }
		}

		if (edgeList.size())
		{
			clusteredEdges.emplace_back(edgeList);
		}
	}
	return clusteredEdges;
}

gp_Pnt VoxelGrid::getPointInRoom(int roomNum)
{
	for (auto i = VoxelLookup_.begin(); i != VoxelLookup_.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = i->second;

		if (currentVoxel->getRoomNum() == roomNum)
		{
			return currentVoxel->getOCCTCenterPoint();
		}
	}
	return gp_Pnt();
}


std::vector<int> VoxelGrid::getTopBoxelIndx() {

	std::vector<int> voxelIndx;

	for (int i = xRelRange_ * yRelRange_ * zRelRange_ - xRelRange_ * yRelRange_;
		i < xRelRange_ * yRelRange_ * zRelRange_ - 1;
		i++)
	{
		voxelIndx.emplace_back(i);
	}
	return voxelIndx;
}


std::vector<std::shared_ptr<voxel>> VoxelGrid::getVoxelPlate(double platelvl) {
	double voxelCount = (double) VoxelLookup_.size();
	double zlvls = voxelCount / (static_cast<double>(xRelRange_) * static_cast<double>(yRelRange_));
	double smallestDistanceToLvl = 999999;

	int plateVoxelLvl;

	for (int i = 0; i < zlvls; i++)
	{
		std::shared_ptr<voxel> v = VoxelLookup_[i * xRelRange_ * yRelRange_];

		double coreHeight = v->getCenterPoint().get<2>();
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

	std::vector<std::shared_ptr<voxel>> plateVoxels;

	for (int i = 0; i < VoxelLookup_.size(); i++)
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
	int y = static_cast<int>(floor((i / xRelRange_) % yRelRange_));
	int z = static_cast<int>(floor(i / (xRelRange_ * yRelRange_)));

	return T(x, y, z);
}


int VoxelGrid::relativeToLinear(const BoostPoint3D& p) {
	int x = static_cast<int>(std::round(p.get<0>()));
	int y = static_cast<int>(std::round(p.get<1>()));
	int z = static_cast<int>(std::round(p.get<2>()));

	int i = z * xRelRange_ * yRelRange_ + y * xRelRange_ + x;

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

bool VoxelGrid::voxelBeamWindowIntersection(helper* h, std::shared_ptr<voxel> currentVoxel, int indxDir)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	double windowSearchDepth = 0.3;
	double windowArea = 0;

	// get a beam
	double voxelJump = settingsCollection.voxelSize();
	std::vector<std::shared_ptr<voxel>> voxelBeam;

	std::shared_ptr<voxel> loopingCurrentVoxel = currentVoxel;
	voxelBeam.emplace_back(currentVoxel);

	bool windowFound = false;

	if (!loopingCurrentVoxel->hasFace(indxDir))
	{
		return false;
	}
	while (true)
	{
		int loopingCurrentIndx = getNeighbour(loopingCurrentVoxel, indxDir);
		if (loopingCurrentIndx == -1) { break; }

		loopingCurrentVoxel = getVoxelPtr(loopingCurrentIndx);
		if (!loopingCurrentVoxel->getIsIntersecting()) { break; }

		voxelBeam.emplace_back(loopingCurrentVoxel);
		voxelJump += settingsCollection.voxelSize();
		if (windowSearchDepth < voxelJump) { break; }
	}

	for (size_t j = 0; j < voxelBeam.size(); j++)
	{
		std::vector<Value> intersectingValues = voxelBeam[j]->getInternalProductList();
		for (auto valueIt = intersectingValues.begin(); valueIt != intersectingValues.end(); ++valueIt)
		{
			std::string productTypeName = h->getLookup(valueIt->second)->getProductPtr()->data().type()->name();

			if (productTypeName == "IfcDoor" || productTypeName == "IfcWindow")
			{
				return true;
			}
		}
	}
	return false;
}

std::vector<int> VoxelGrid::getNeighbours(std::shared_ptr<voxel> boxel, bool connect6)
{
	BoostPoint3D middlePoint = boxel->getCenterPoint();
	BoostPoint3D relativePoint = worldToRelPoint(middlePoint);
	int voxelInt = relativeToLinear(relativePoint);
	return getNeighbours(voxelInt, connect6);
}

int VoxelGrid::getNeighbour(std::shared_ptr<voxel> boxel, int dir)
{
	BoostPoint3D middlePoint = boxel->getCenterPoint();
	BoostPoint3D relativePoint = worldToRelPoint(middlePoint);
	int voxelInt = relativeToLinear(relativePoint);
	std::vector<int> neighbours = getDirNeighbours(voxelInt);

	if (neighbours.size() == 0)
	{
		return -1;
	}

	return neighbours[dir];
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
	double voxelSize = SettingsCollection::getInstance().voxelSize();

	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize) + voxelSize / 2;
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize) + voxelSize / 2;
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSize) + voxelSize / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}


BoostPoint3D VoxelGrid::worldToRelPoint(BoostPoint3D p)
{
	double voxelSize = SettingsCollection::getInstance().voxelSize();

	double xCoord = (bg::get<0>(p) - anchor_.X() - voxelSize / 2) / voxelSize;
	double yCoord = (bg::get<1>(p) - anchor_.Y() - voxelSize / 2) / voxelSize;
	double zCoord = (bg::get<2>(p) - anchor_.Z() - voxelSize / 2) / voxelSize;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}


void VoxelGrid::growExterior(int startIndx, int roomnum, helper* h)
{
	// set up starting data
	std::vector<int> buffer = { startIndx };
	std::vector<int> totalRoom = { startIndx };
	VoxelLookup_[startIndx]->setRoomNum(roomnum);

	// setting up extracting exterior data
	std::vector<int> exterior;
	bool isOutSide = false;

	while (buffer.size() > 0)
	{
		std::vector<int> tempBuffer;
		for (int currentIdx : buffer)
		{
			// only output process if outside is processed
			if (roomnum == 0)
			{
				if (totalRoom.size() % 10000 == 0)
				{
					std::cout.flush();
					std::cout << "\tSize: " << totalRoom.size() << "\r";
				}
			}

			std::shared_ptr<voxel> currentVoxel = VoxelLookup_[currentIdx];
			currentVoxel->setOutside(isOutSide);

			// find neighbours
			std::vector<int> neighbourIndxList = getDirNeighbours(currentIdx);

			for (int i = 0; i < neighbourIndxList.size(); i++)
			{
				int neighbourIdx = neighbourIndxList[i];

				bool isOutVoxel = false;
				if (neighbourIdx == -1) { isOutVoxel = true; }
				if (isOutVoxel && isOutSide) { continue; }
				if (isOutVoxel) // update exterior flag backwards
				{
					isOutSide = true;
					for (int roomVoxel : totalRoom)
					{
						VoxelLookup_[roomVoxel]->setOutside(isOutSide);
						exteriorVoxelsIdx_.emplace_back(roomVoxel);
					}
					continue;
				}

				std::shared_ptr<voxel> neighbourVoxel = VoxelLookup_[neighbourIdx];
				// bypass if neighbour already part of a room
				if (neighbourVoxel->getRoomNum() != -1) { continue; }

				// set trans faces if neighbour is intersecting
				if (neighbourVoxel->getIsIntersecting())
				{
					if (roomnum == 0)
					{
						neighbourVoxel->setTransFace(i);
						neighbourVoxel->setIsShell();
					}

					if (i % 2 == 1) { currentVoxel->setTransFace(i - 1); }
					else { currentVoxel->setTransFace(i + 1); }
					continue;
				}

				// exlude neighbour if already in buffer
				bool dupli = false;
				for (size_t l = 0; l < tempBuffer.size(); l++)
				{
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
					VoxelLookup_[neighbourIdx]->setRoomNum(roomnum);
				}
			}
		}
		buffer.clear();
		buffer = tempBuffer;
	}
	return;
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
			std::shared_ptr<voxel> currentVoxel = VoxelLookup_[currentIdx];

			std::vector<int> neighbours = getNeighbours(currentIdx);

			for (size_t j = 0; j < neighbours.size(); j++)
			{
				int otherIdx = neighbours[j];
				std::shared_ptr<voxel> otherVoxel = VoxelLookup_[otherIdx];

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

