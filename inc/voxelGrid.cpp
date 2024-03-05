#include "helper.h"
#include "voxelGrid.h"
#include "voxel.h"

bool VoxelGrid::addVoxel(int indx, helper* h, bool checkIfInt)
{
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	gp_Pnt midPointOCCT = helperFunctions::Point3DBTO(midPoint);

	//helperFunctions::printPoint(midPointOCCT);

	voxel* boxel = new voxel(midPoint, sudoSettings_->voxelSize_, sudoSettings_->voxelSize_);

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	auto boxelGeo = boxel->getVoxelGeo();

	std::vector<gp_Pnt> pointList;
	if (sudoSettings_->planeIntersection_) { pointList = boxel->getPlanePoints(planeRotation_); }
	else { pointList = boxel->getCornerPoints(planeRotation_); }

	bool isIntersecting = false;

	if (checkIfInt)
	{
		std::vector<Value> qResult;
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		h->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));
		for (size_t k = 0; k < qResult.size(); k++)
		{
			lookupValue* lookup = h->getLookup(qResult[k].second);
			if (boxel->checkIntersecting(*lookup, pointList, midPointOCCT, h, sudoSettings_->planeIntersection_))
			{
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
				!sudoSettings_->requireFullVoxels_
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

VoxelGrid::VoxelGrid(helper* h, std::shared_ptr<SettingsCollection> settings)
{
	sudoSettings_ = settings;
	anchor_ = h->getLllPoint();

	double voxelSize = sudoSettings_->voxelSize_;

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

	xRelRange_ = (int)ceil(xRange / voxelSize) + 1;
	yRelRange_ = (int)ceil(yRange / voxelSize) + 1;
	zRelRange_ = (int)ceil(zRange / voxelSize) + 1;

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment_ = std::vector<int>(totalVoxels_, 0);

	planeRotation_ = h->getRotation(); //tODO: make this centralized as well 

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

	if (!sudoSettings_->requireVoxels_) { 
		std::cout << "[INFO] No voxelization required "  << std::endl;
		return; 
	} // no voxels needed for lod0.0 and 1.0 only

	std::cout << "- Populate Grid" << std::endl;
	populatedVoxelGrid(h);

	if (!sudoSettings_->requireFullVoxels_) { 
		std::cout << "[INFO] No complete voxelization required " << std::endl;
		return; 
	}

	std::cout << "- Exterior space growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting())
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

	if (sudoSettings_->makeInterior_)
	{
		std::cout << "- Interior spaces growing" << std::endl;
		roomSize_ = 1;
		for (int i = 0; i < totalVoxels_; i++)
		{
			if (!VoxelLookup_[i]->getIsIntersecting() && VoxelLookup_[i]->getRoomNum() == -1)
			{
				growExterior(i, roomSize_, h);
				roomSize_++;
			}
		}
		std::cout << "\tInterior spaces succesfully grown" << std::endl;
	}

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

	return;
}

void VoxelGrid::populatedVoxelGrid(helper* h)
{
	// split the range over cores
	int coreCount = std::thread::hardware_concurrency();
	int coreUse = coreCount - 1;
	int splitListSize = floor(xRelRange_* yRelRange_ / coreUse);
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;

	for (size_t i = 0; i < coreUse; i++)
	{
		int beginIdx = i * splitListSize;
		int endIdx = (i + 1) * splitListSize;

		if (i == coreUse - 1) { endIdx = xRelRange_ * yRelRange_; }

		threadList.emplace_back([=, &voxelsGrown]() {
			addVoxelColumn(beginIdx, endIdx, h, &voxelsGrown);
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

std::vector<std::vector<TopoDS_Edge>> VoxelGrid::getDirectionalFaces(int dirIndx, double angle, int roomNum)
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
			if (potentialVoxel->getRoomNum() != roomNum) { continue; }
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

std::vector<int> VoxelGrid::getNeighbours(voxel* boxel, bool connect6)
{
	BoostPoint3D middlePoint = boxel->getCenterPoint();
	BoostPoint3D relativePoint = worldToRelPoint(middlePoint);
	int voxelInt = relativeToLinear(relativePoint);
	return getNeighbours(voxelInt, connect6);
}

int VoxelGrid::getNeighbour(voxel* boxel, int dir)
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
	double voxelSize = sudoSettings_->voxelSize_;

	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize) + voxelSize / 2;
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize) + voxelSize / 2;
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSize) + voxelSize / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}


BoostPoint3D VoxelGrid::worldToRelPoint(BoostPoint3D p)
{
	double voxelSize = sudoSettings_->voxelSize_;

	double xCoord = (bg::get<0>(p) - anchor_.X() - voxelSize / 2) / voxelSize;
	double yCoord = (bg::get<1>(p) - anchor_.Y() - voxelSize / 2) / voxelSize;
	double zCoord = (bg::get<2>(p) - anchor_.Z() - voxelSize / 2) / voxelSize;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}


std::vector<int> VoxelGrid::growExterior(int startIndx, int roomnum, helper* h)
{
	std::vector<int> buffer = { startIndx };
	std::vector<int> totalRoom = { startIndx };
	VoxelLookup_[startIndx]->setRoomNum(roomnum);
	Assignment_[startIndx] = 1;

	bool isOutSide = false;
	while (buffer.size() > 0)
	{
		std::vector<int> tempBuffer;
		for (size_t j = 0; j < buffer.size(); j++)
		{
			if (roomnum == 0)
			{
				if (totalRoom.size() % 1000 == 0)
				{
					std::cout.flush();
					std::cout << "\tSize: " << totalRoom.size() << "\r";
				}
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
					if (roomnum == 0)
					{
						VoxelLookup_[neighbourIdx]->setTransFace(k);
						VoxelLookup_[neighbourIdx]->setIsShell();
					}

					if (k % 2 == 1) { VoxelLookup_[currentIdx]->setTransFace(k - 1); }
					else { VoxelLookup_[currentIdx]->setTransFace(k + 1); }
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
						VoxelLookup_[neighbourIdx]->setRoomNum(roomnum);
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
						VoxelLookup_[neighbourIdx]->setRoomNum(roomnum);
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

