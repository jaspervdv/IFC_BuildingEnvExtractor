#include "helper.h"
#include "voxelGrid.h"
#include "voxel.h"
#include "stringManager.h"
#include "DebugUtils.h"

#include <algorithm>
#include <execution>
#include <thread>  

void VoxelGrid::init(DataManager* h)
{
	anchor_ = h->getLllPoint();

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	double voxelSize = settingsCollection.voxelSize();

	gp_Pnt urrPoint = h->getUrrPoint();

	// resize to allow full voxel encapsulation
	anchor_.SetX(anchor_.X() - (voxelSize * 2));
	anchor_.SetY(anchor_.Y() - (voxelSize * 2));
	anchor_.SetZ(anchor_.Z() - (voxelSize * 2));

	urrPoint.SetX(urrPoint.X() + (voxelSize * 2));
	urrPoint.SetY(urrPoint.Y() + (voxelSize * 2));
	urrPoint.SetZ(urrPoint.Z() + (voxelSize * 2));

	// set range
	double xRange = urrPoint.X() - anchor_.X();
	double yRange = urrPoint.Y() - anchor_.Y();
	double zRange = urrPoint.Z() - anchor_.Z();

	xRelRange_ = static_cast<int>(ceil(xRange / voxelSize) + 1);
	yRelRange_ = static_cast<int>(ceil(yRange / voxelSize) + 1);
	zRelRange_ = static_cast<int>((int)ceil(zRange / voxelSize) + 1);

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	planeRotation_ = settingsCollection.gridRotation();
	return;
}


void VoxelGrid::populateVoxelGrid(DataManager* h)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoPopulateGrid) << std::endl;

	// compute column scores
	std::vector<int> columScoreList = computeColumnScore(h);
	int columSumScore = std::accumulate(columScoreList.begin(), columScoreList.end(), 0);

	// split the range over cores
	int coreUse = SettingsCollection::getInstance().threadcount() - 1;
	int threadScore = columSumScore / coreUse;
	int voxelsGrown = 0;
	int beginIdx = 0;

	std::vector<std::thread> threadList;
	for (int i = 0; i < coreUse; i++)
	{
		int endIdx = 0;
		// compute the core processing list size
		if (i == coreUse - 1)
		{
			endIdx = xRelRange_ * yRelRange_;
		}
		else
		{
			endIdx = computeEndColumnIdx(columScoreList, threadScore, beginIdx);
		}

		threadList.emplace_back([=, &voxelsGrown]()
			{
				addVoxelColumn(beginIdx, endIdx, h, &voxelsGrown);
			});

		// set begin of the next step to the end of the current step
		beginIdx = endIdx;
	}

	// count the voxels that have been generated
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
		std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex_);
		int count = *voxelGrowthCount;
		voxelCountLock.unlock();

		std::cout.flush();
		std::cout << "\t" << count << " of " << totalVoxels_ << "\r";

		if (count == totalVoxels_) { return; }
		if (activeThreads_ == 0) { return; }
	}
}


bool VoxelGrid::addVoxel(int indx, DataManager* h)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	std::shared_ptr<voxel> boxel = std::make_shared<voxel>(voxel(midPoint, settingsCollection.voxelSize(), settingsCollection.voxelSize()));
	
	std::vector<gp_Pnt> pointList;
	if (settingsCollection.intersectionLogic() == 2) { pointList = boxel->getPlanePoints(); }
	else if (settingsCollection.intersectionLogic() == 3) { pointList = boxel->getCornerPoints(); }

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	auto boxelGeo = boxel->getVoxelGeo();
	gp_Pnt midPointOCCT = helperFunctions::Point3DBTO(midPoint);

	std::vector<Value> qResult;
	qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
	h->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));
	for (const Value& resultItem : qResult)
	{
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(resultItem.second);
		std::string productType = lookup->getProductPtr()->data().type()->name();
		if (boxel->checkIntersecting(*lookup, pointList, midPointOCCT, settingsCollection.intersectionLogic()))
		{
			boxel->addInternalProduct(resultItem);
		}
	}

	std::unique_lock<std::mutex> voxelWriteLock(voxelLookupMutex);
	VoxelLookup_.emplace(indx, boxel);
	voxelWriteLock.unlock();
	return boxel->getIsIntersecting();
}

void VoxelGrid::addVoxelColumn(int beginIindx, int endIdx, DataManager* h, int* voxelGrowthCount)
{
	addActiveThread();
	bool fullVoxelReq = SettingsCollection::getInstance().requireFullVoxels();
	// get top plate begin to start growing voxels down from
	int topPlateBeginIndx = (zRelRange_ -1) * xRelRange_  * yRelRange_ ;

	for (int i = beginIindx; i < endIdx; i++)
	{
		int voxelIndx = topPlateBeginIndx + i;
		while (true)
		{
			if ( addVoxel(voxelIndx, h) && !fullVoxelReq) { break; }

			// add to the voxel counter after adding
			std::unique_lock<std::mutex> voxelCountLock(voxelGrowthMutex_);
			(*voxelGrowthCount)++;
			voxelCountLock.unlock();

			voxelIndx = getLowerNeighbour(voxelIndx);
			if (voxelIndx == -1) { break; }
		}
	}
	removeActiveThread();
}


std::vector<int> VoxelGrid::computeColumnScore(DataManager* h)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double voxelSize = settingsCollection.voxelSize();
	int loopRange = xRelRange_ * yRelRange_;
	int plateIndx = (zRelRange_ - 1) * xRelRange_ * yRelRange_;

	std::vector<int> columScoreList;
	for (int i = 0; i < loopRange; i++)
	{
		int plate = plateIndx + i;

		BoostPoint3D coneCenter = relPointToWorld(linearToRelative<BoostPoint3D>(plate));
		BoostPoint3D lll = BoostPoint3D(
			coneCenter.get<0>() - voxelSize / 2,
			coneCenter.get<1>() - voxelSize / 2,
			-100
		);

		BoostPoint3D urr = BoostPoint3D(
			coneCenter.get<0>() + voxelSize / 2,
			coneCenter.get<1>() + voxelSize / 2,
			coneCenter.get<2>() + voxelSize / 2
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
	}
	return columScoreList;
}


void VoxelGrid::addActiveThread()
{
	std::unique_lock<std::mutex> threadCountLock(voxelGrowthMutex_);
	activeThreads_++;
	threadCountLock.unlock();
}

void VoxelGrid::removeActiveThread()
{
	std::unique_lock<std::mutex> threadCountLock(voxelGrowthMutex_);
	activeThreads_--;
	threadCountLock.unlock();
}

int VoxelGrid::computeEndColumnIdx(const std::vector<int>& columScoreList, const int threadScore, const int beginIdx)
{
	int score = 0;
	int endIdx = 0;
	int loopRange = xRelRange_ * yRelRange_;

	for (int j = beginIdx; j < loopRange; j++)
	{
		score += columScoreList[j];
		if (score > threadScore)
		{
			return j;
		}
	}
	return -1;
}


template<typename T>
T VoxelGrid::linearToRelative(int i)
{
	int x = i % xRelRange_;
	int y = static_cast<int>(floor((i / xRelRange_) % yRelRange_));
	int z = static_cast<int>(floor(i / (xRelRange_ * yRelRange_)));

	return T(x, y, z);
}

int VoxelGrid::relativeToLinear(const BoostPoint3D& p)
{
	int x = static_cast<int>(std::round(p.get<0>()));
	int y = static_cast<int>(std::round(p.get<1>()));
	int z = static_cast<int>(std::round(p.get<2>()));

	int i = z * xRelRange_ * yRelRange_ + y * xRelRange_ + x;

	return i;
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

std::array<int, 6> VoxelGrid::getDirNeighbours(int voxelIndx)
{
	std::array<int, 6> neightbours;
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool xSmall = loc3D.X() - 1 >= 0;
	bool xBig = loc3D.X() + 1 < xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 < yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 < zRelRange_;

	// connectivity
	if (xSmall) { neightbours[0] = voxelIndx - 1; }
	else { neightbours[0] = -1; }
	if (xBig) { neightbours[1] = voxelIndx + 1; }
	else { neightbours[1] = -1; }
	if (ySmall) { neightbours[2] = voxelIndx - xRelRange_; }
	else { neightbours[2] = -1; }
	if (yBig) { neightbours[3] = voxelIndx + xRelRange_; }
	else { neightbours[3] = -1; }
	if (zSmall) { neightbours[4] = voxelIndx - xRelRange_ * yRelRange_; }
	else { neightbours[4] = -1; }
	if (zBig) { neightbours[5] = voxelIndx + xRelRange_ * yRelRange_; }
	else { neightbours[5] = -1; }

	return neightbours;
}


void VoxelGrid::growVoid(int startIndx, int roomnum, DataManager* h)
{
	// set up starting data
	std::unordered_set<int> buffer = { startIndx };
	int totalRoomSize = 1;
	VoxelLookup_[startIndx]->setRoomNum(roomnum);

	bool isOutSide = false;
	if (roomnum == 0) { isOutSide = true; }

	while (buffer.size() > 0)
	{
		std::unordered_set<int> tempBuffer;
		for (const int currentIdx : buffer)
		{
			// only output process if outside is processed
			if (roomnum == 0)
			{
				if (totalRoomSize % 10000 == 0)
				{
					std::cout.flush();
					std::cout << "\tSize: " << totalRoomSize << "\r";
				}
			}

			std::shared_ptr<voxel> externalVoxel = VoxelLookup_[currentIdx];
			externalVoxel->setOutside(isOutSide);

			// find neighbours
			std::array<int, 6> neighbourIndxList = getDirNeighbours(currentIdx);

			for (int i = 0; i < neighbourIndxList.size(); i++)
			{
				int neighbourIdx = neighbourIndxList[i];
				if (neighbourIdx == -1) { continue; }

				std::shared_ptr<voxel> potentialIntVoxel = VoxelLookup_[neighbourIdx];
				// bypass if neighbour already part of a room
				if (potentialIntVoxel->getRoomNum() != -1) { continue; }

				// set trans faces if neighbour is intersecting
				if (potentialIntVoxel->getIsIntersecting())
				{
					potentialIntVoxel->setTransFace(i ^ 1);
					externalVoxel->setTransFace(i);
					if (roomnum == 0)
					{					
						potentialIntVoxel->setIsShell();
					}

					continue;
				}

				// exclude neighbour if already in buffer
				if (tempBuffer.find(neighbourIdx) != tempBuffer.end()) {
					continue;
				}
				tempBuffer.emplace(neighbourIdx);
				VoxelLookup_[neighbourIdx]->setRoomNum(roomnum);
				totalRoomSize++;
			}
		}
		buffer.clear();
		buffer = tempBuffer;
	}
	return;
}

void VoxelGrid::growExterior(DataManager* h)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoExteriorSpaceGrowing) << std::endl;

	if (VoxelLookup_[0]->getIsIntersecting())
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentNoExteriorSpace) << std::endl;
		return;
	}

	growVoid(0, 0, h); //TODO: multithread
	std::cout << std::endl;
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentExteriorSpaceGrown) << std::endl;
	return;
}

void VoxelGrid::growInterior(DataManager* h)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoInterioSpacesGrowing) << std::endl;
	roomSize_ = 1;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting() && VoxelLookup_[i]->getRoomNum() == -1)
		{
			growVoid(i, roomSize_, h);
			roomSize_++;
		}
	}
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentInteriorSpaceGrown) << std::endl;
}


void VoxelGrid::pairVoxels()
{
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
}

void VoxelGrid::markVoxelBuilding(int startIndx, int buildnum) {

	VoxelLookup_[startIndx]->setBuildingNum(buildnum);
	std::vector<int> buffer = { startIndx };
	std::vector<int> potentialBuildingVoxels;

	while (true)
	{
		for (const int currentIdx : buffer)
		{
			std::shared_ptr<voxel> currentVoxel = VoxelLookup_[currentIdx];

			std::array<int, 6> neighbours = getDirNeighbours(currentIdx);

			for (const int otherIdx : neighbours)
			{
				if (otherIdx == - 1) { continue; }

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


bool VoxelGrid::voxelBeamWindowIntersection(DataManager* h, std::shared_ptr<voxel> currentVoxel, int indxDir)
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


int VoxelGrid::invertDir(int dirIndx) { //TODO: this seems voxel related code?
	if (dirIndx % 2 == 1) { return dirIndx - 1; }
	else { return dirIndx + 1; }
}

void VoxelGrid::setSemanticVoxelFace(DataManager* h, std::shared_ptr<voxel> voxel , int dirIndx, const std::vector<Value>& intersectingValues)
{
	if (intersectingValues.size() == 0) { return; }

	bool hasWindowSurface = false;
	bool hasDoorSurface = false;
	bool hasWallSurface = false;
	bool hasRoofSurface = false;

	for (auto valueIt = intersectingValues.begin(); valueIt != intersectingValues.end(); ++valueIt)
	{
		IfcSchema::IfcProduct* intersectedProduct = h->getLookup(valueIt->second)->getProductPtr();
		std::string productTypeName = intersectedProduct->data().type()->name();

		if (productTypeName == "IfcWindow")
		{
			hasWindowSurface = true;
			continue;
		}
		if (productTypeName == "IfcDoor")
		{
			hasDoorSurface = true;
			continue;
		}
		if (productTypeName == "IfcPlate")
		{
			if (helperFunctions::hasGlassMaterial(intersectedProduct))
			{
				hasWindowSurface = true;
				continue;
			}
		}
		if (productTypeName == "IfcWall" || productTypeName == "IfcWallStandardCase" || productTypeName == "IfcColumn")
		{
			hasWallSurface = true;
			continue;
		}
		if (productTypeName == "IfcRoof" || productTypeName == "IfcSlab")
		{
			hasRoofSurface = true;
			continue;
		}

	}

	if (hasWindowSurface)
	{
		voxel->addWindowSemantic(dirIndx);
		return;
	}

	if (hasDoorSurface)
	{
		voxel->addDoorSemantic(dirIndx);
		return;
	}

	if (hasWallSurface)
	{
		if (dirIndx == 4)
		{
			voxel->addRoofSemantic(dirIndx);
			return;
		}
		if (dirIndx == 5)
		{
			voxel->addOuterCeilingSemantic(dirIndx);
			return;
		}

		voxel->addWallSemantic(dirIndx);
		return;
	}

	if (hasRoofSurface)
	{
		if (voxel->getOCCTCenterPoint().Z() < SettingsCollection::getInstance().footprintElevation())
		{
			voxel->addGroundSemantic(dirIndx);
		}
		else if (dirIndx == 5)
		{
			voxel->addOuterCeilingSemantic(dirIndx); 
		}
		else
		{
			voxel->addRoofSemantic(dirIndx); //TODO: add check to see if roof or not
		}

		return;
	}
	return;
}


VoxelGrid::VoxelGrid(DataManager* h)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	if (!settingsCollection.requireVoxels()) { 
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoNoVoxelizationReq) << std::endl;
		return; 
	} // no voxels needed for lod0.0 and 1.0 only
	
	// init the basic data
	init(h);
	populateVoxelGrid(h);

	if (!settingsCollection.requireFullVoxels()) { 
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoNocompleteVoxelizationReq) << std::endl;
		std::cout << std::endl;
		return; 
	}
	growExterior(h);

	if (settingsCollection.makeInterior())
	{
		growInterior(h);
	}
	pairVoxels();

	return;
}

void VoxelGrid::computeSurfaceSemantics(DataManager* h)
{
	if (hasSemanticSurfaces_) { return; } // if already processed no reprocessing is needed
	hasSemanticSurfaces_ = true;

	// compute external surfaces
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = getIntersectingVoxels();

	for (size_t i = 0; i < intersectingVoxels.size(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = intersectingVoxels[i];

		// find the types
		for (int indxdir = 0; indxdir < 6; indxdir++)
		{
			int neighbourIndx = getNeighbour(currentVoxel, indxdir);
			std::shared_ptr<voxel> neighbourVoxel = VoxelLookup_[neighbourIndx];
			int invertDir = indxdir ^ 1;
			if (neighbourVoxel->getIsIntersecting()) { continue; }
			if (neighbourVoxel->getRoomNum() == -1) { continue; }
			if (!neighbourVoxel->hasFace(invertDir)) { continue; }

			std::vector<Value> intersectingValues = currentVoxel->getInternalProductList();
			setSemanticVoxelFace(h, neighbourVoxel, invertDir, intersectingValues);
		}	
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

std::vector<std::pair<std::vector<TopoDS_Edge>, CJObjectID>> VoxelGrid::getDirectionalFaces(int dirIndx, double angle, int roomNum)
{
	std::vector<std::pair<std::vector<TopoDS_Edge>, CJObjectID>> clusteredEdges = {};

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
		CJObjectID search4Type = CJObjectID::CJTypeNone;
		for (int i = searchStartIdx; i < VoxelLookup_.size(); i++)
		{
			std::shared_ptr<voxel> potentialVoxel = VoxelLookup_[i];
			if (evaluated[i] == 1) { continue; }
			if (potentialVoxel->getRoomNum() != roomNum) { continue; }
			if (!potentialVoxel->hasFace(dirIndx)) { continue; }
			search4Type = potentialVoxel->faceType(dirIndx);
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

				std::array<int, 6> neighbourIndxList = getDirNeighbours(bufferIndx);
				for (size_t i = 0; i < 4; i++)
				{
					int neighbourIndx = neighbourIndxList[allowedNeighbourDir[i]];
					if (neighbourIndx == -1) { continue; }
					std::shared_ptr<voxel> neighbourVoxel = VoxelLookup_[neighbourIndx];
					// find neighbour to grow into 
					if (neighbourVoxel->hasFace(dirIndx) && neighbourVoxel->faceType(dirIndx) == search4Type) {
						if (evaluatedGrowth[neighbourIndx] == 1) { continue; }
						evaluatedGrowth[neighbourIndx] = 1;
						tempBuffer.emplace_back(neighbourIndx);
						continue;
					}

					std::vector<gp_Pnt> voxelPoints = currentVoxel->getCornerPoints();
					// create edge based on the normal dir
					if (dirIndx == 1)
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
					else if (dirIndx == 0)
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
					else if (dirIndx == 3)
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
					else if (dirIndx == 2)
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
					else if (dirIndx == 5)
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
					else if (dirIndx == 4)
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
			evaluatedGrowth[i] = 0;
		}

		if (edgeList.size())
		{
			clusteredEdges.emplace_back(std::make_pair(edgeList, search4Type));
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


std::vector<int> VoxelGrid::getTopBoxelIndx() 
{
	std::vector<int> voxelIndx;

	for (int i = xRelRange_ * yRelRange_ * zRelRange_ - xRelRange_ * yRelRange_;
		i < xRelRange_ * yRelRange_ * zRelRange_ - 1;
		i++)
	{
		voxelIndx.emplace_back(i);
	}
	return voxelIndx;
}


std::vector<std::shared_ptr<voxel>> VoxelGrid::getVoxelPlate(double platelvl) 
{
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

int VoxelGrid::getNeighbour(std::shared_ptr<voxel> boxel, int dir)
{
	BoostPoint3D middlePoint = boxel->getCenterPoint();
	BoostPoint3D relativePoint = worldToRelPoint(middlePoint);
	int voxelInt = relativeToLinear(relativePoint);
	std::array<int, 6> neighbours = getDirNeighbours(voxelInt);

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

int VoxelGrid::getCloseByVoxel(const gp_Pnt& targetPoint)
{
	BoostPoint3D relPoint = worldToRelPoint(helperFunctions::Point3DOTB(targetPoint));
	return relativeToLinear(relPoint);
}
