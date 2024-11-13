#include "DataManager.h"
#include "helper.h"
#include "stringManager.h"
#include "errorCollection.h"

#include <BOPAlgo_Splitter.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>

#include <thread>
#include <shared_mutex> 
#include <mutex> 


helper::helper(const std::vector<std::string>& pathList) {
	for (size_t i = 0; i < pathList.size(); i++)
	{
		std::string path = pathList[i];
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoParsingFile) << path << std::endl;
		if (!findSchema(path)) { 
			continue;
		}
		std::unique_ptr<fileKernelCollection> dataCollection = std::make_unique<fileKernelCollection>(path);
		if (!dataCollection.get()->isGood())
		{
			std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcUnableToParse) << " " << path << std::endl;
			continue;
		}

		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentValidIFCFound) << std::endl;
		std::cout << std::endl;

		datacollection_.emplace_back(std::move(dataCollection));
		dataCollectionSize_++;
		isPopulated_ = true;
	}
	return;
}

bool helper::findSchema(const std::string& path, bool quiet) {
	std::ifstream infile(path);
	std::string line;
	int linecount = 0;
	std::unordered_set<std::string> ifcVersionList = SettingsCollection::getInstance().getSupportedIfcVersionList();

	while (linecount < 100 && std::getline(infile, line))
	{
		if (line[0] == '#')
		{
			if (!quiet)
			{
				std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoSchema) << path << std::endl;
				ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSchema, path);
			}
			infile.close();
			return false;
		}
		if (line.find("FILE_SCHEMA") == std::string::npos)
		{
			linecount++;
			continue;
		}

		for (std::string ifcVersion : ifcVersionList)
		{
			if (line.find(ifcVersion) == std::string::npos) {
				continue;
			}

			if (buildVersion != ifcVersion)
			{
				if (!quiet)
				{
					std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcIncomp) + ifcVersion << std::endl;
					ErrorCollection::getInstance().addError(ErrorID::warningIfcIncomp, path);
				}
				infile.close();
				return false;
			}
			else {
				if (!quiet)
				{
					std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentcompIFCFound) + ifcVersion << std::endl;
				}
				infile.close();
				return true;
			}			
		}
	}
	infile.close();
	std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoSchema) << path << std::endl;
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSchema, path);
	return false;
}


void helper::elementCountSummary(bool* hasProxy, bool* hasLotProxy)
{
	// count the proxy amount
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		IfcSchema::IfcProduct::list::ptr products = fileObject->instances_by_type<IfcSchema::IfcProduct>();
		IfcSchema::IfcBuildingElementProxy::list::ptr proxyProducts = fileObject->instances_by_type<IfcSchema::IfcBuildingElementProxy>();

		objectCount_ += products->size();
		proxyCount_ += proxyProducts->size();
	}

	std::cout << "\t" << objectCount_ << " objects found" << std::endl;
	std::cout << "\t" << proxyCount_ << " IfcBuildingElementProxy objects found" << std::endl;

	if (proxyCount_ > 0) { *hasProxy = true; }
	if (proxyCount_ / objectCount_ >= maxProxyP_) { *hasLotProxy = true; }
	return;
}


void helper::computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();


	std::vector<gp_Pnt> pointList;
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		IfcSchema::IfcSlab::list::ptr slabList = fileObject->instances_by_type<IfcSchema::IfcSlab>();
		std::vector<gp_Pnt> pointLisSlab;
		if (slabList->size())
		{
			pointLisSlab = getAllTypePoints<IfcSchema::IfcSlab::list::ptr>(slabList);
		}
		pointList.insert(pointList.end(), pointLisSlab.begin(), pointLisSlab.end());
	}

	if (!pointList.size())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoPoints);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoPoints));
	}

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	double rotation = settingsCollection.desiredRotation(); // if not pre set by the user the default grid rotation = 0
	int maxIt = 15;

	// set data for a bbox
	helperFunctions::rotatedBBoxDiagonal(pointList, lllPoint, urrPoint, rotation);
	if (!settingsCollection.autoRotateGrid()) {
		settingsCollection.setGridRotation(settingsCollection.desiredRotation());
		return;
	} //bypass rotation comp if not required

	double smallestDistance = lllPoint->Distance(*urrPoint);

	for (size_t i = 0; i < maxIt; i++)
	{
		std::tuple<gp_Pnt, gp_Pnt, double> left;
		std::tuple<gp_Pnt, gp_Pnt, double> right;

		gp_Pnt leftLllPoint;
		gp_Pnt leftUrrPoint;
		gp_Pnt rghtLllPoint;
		gp_Pnt rghtUrrPoint;

		helperFunctions::rotatedBBoxDiagonal(pointList, &leftLllPoint, &leftUrrPoint, rotation - angle);
		helperFunctions::rotatedBBoxDiagonal(pointList, &rghtLllPoint, &rghtUrrPoint, rotation + angle);

		double leftDistance = leftLllPoint.Distance(leftUrrPoint);
		double rghtDistance = rghtLllPoint.Distance(rghtUrrPoint);

		if (leftDistance > rghtDistance && smallestDistance > rghtDistance)
		{
			rotation = rotation + angle;
			smallestDistance = rghtDistance;
			*lllPoint = rghtLllPoint;
			*urrPoint = rghtUrrPoint;
		}
		else if (smallestDistance > leftDistance)
		{
			rotation = rotation - angle;
			smallestDistance = leftDistance;
			*lllPoint = leftLllPoint;
			*urrPoint = leftUrrPoint;
		}
		angle = angle / 2;
	}
	settingsCollection.setGridRotation(rotation);
	return;
}


void helper::computeObjectTranslation(gp_Vec* vec)
{
	// get a point to translate the model to
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcSchema::IfcSlab::list::ptr slabList = datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>();

		if (!slabList.get()->size()) { continue; }
		IfcSchema::IfcSlab* slab = *slabList->begin();

		gp_Pnt lllPoint;
		gp_Pnt urrPoint;
		TopoDS_Shape siteShape = getObjectShape(slab, false, false);
		helperFunctions::rotatedBBoxDiagonal(helperFunctions::shape2PointList(siteShape), &lllPoint, &urrPoint, 0);
		*vec = gp_Vec(-lllPoint.X(), -lllPoint.Y(), 0);
		return;
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSlab);
	throw std::string(errorWarningStringEnum::getString(ErrorID::warningIfcNoSlab));
	return;
}


template<typename T>
std::vector<gp_Pnt> helper::getAllTypePoints(const T& typePtr, bool simple)
{
	std::vector<gp_Pnt> pointList;
	for (auto it = typePtr->begin(); it != typePtr->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;
		std::vector<gp_Pnt> temp = getObjectPoints(product, simple);

		for (const auto& point : temp) {
			pointList.emplace_back(point);
		}
	}
	return std::vector<gp_Pnt>(pointList);
}


bool helper::hasSetUnits() {
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		if (!datacollection_[i]->getAreaMultiplier() || 
			!datacollection_[i]->getLengthMultiplier() || 
			!datacollection_[i]->getVolumeMultiplier()) { return false; }
	}
	{ return true; }
}


void helper::internalizeGeo()
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoInternalizingGeo) << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	gp_Vec accuracyObjectTranslation;
	computeObjectTranslation(&accuracyObjectTranslation);
	objectTranslation_.SetTranslationPart(accuracyObjectTranslation);
	elementCountSummary(&hasProxy_, &hasLotProxy_);

	try
	{
		computeBoundingData(&lllPoint_, &urrPoint_);
	}
	catch (const std::string& exceptionString)
	{
		throw exceptionString;
	}
	std::cout << 
		CommunicationStringEnum::getString(CommunicationStringID::indentSuccesFinished) <<
		std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << 
		UnitStringEnum::getString(UnitStringID::seconds) << "\n" << std::endl;

	hasGeo_ = true;
}


void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (hasIndex_) { return; }
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCreateSpatialIndex) << std::endl;
	auto t = datacollection_[0]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>();

	if (settingsCollection.useDefaultDiv())
	{
		// add the floorslabs to the rtree
		auto startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcSlab::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>());
		std::cout << "\tIfcSlab objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcRoof::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcRoof>());
		std::cout << "\tIfcRoof objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		// add the walls to the rtree
		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcWall::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcWall>());
		std::cout << "\tIfcWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcCovering::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcCovering>());
		std::cout << "\tIfcCovering objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		// add the columns to the rtree TODO sweeps
		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcColumn::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcColumn>());
		std::cout << "\tIfcColumn objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		// add the beams to the rtree
		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcBeam::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcBeam>());
		std::cout << "\tIfcBeam objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		// add the curtain walls to the rtree 
		//TODO: check this
		/*startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcCurtainWall>());
		std::cout << "\tIfcCurtainWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;*/

		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcPlate::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcPlate>());
		std::cout << "\tIfcPlate objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcMember::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcMember>());
		std::cout << "\tIfcMember objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
		// add doors to the rtree (for the appartment detection)
		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcDoor::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcDoor>());
		std::cout << "\tIfcDoor objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcWindow::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcWindow>());
		std::cout << "\tIfcWindow objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;

		if (settingsCollection.useProxy())
		{
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++)
				addObjectListToIndex<IfcSchema::IfcBuildingElementProxy::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcBuildingElementProxy>());
			std::cout << "\tIfcBuildingElementProxy objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
		}
	}
	else // add custom set div objects
	{
		std::vector<std::string> customDivTypeList = settingsCollection.getCustomDivList();

		for (const std::string& customDivType : customDivTypeList)
		{
			IfcSchema::IfcProduct::list::ptr selectedlist = boost::make_shared<IfcSchema::IfcProduct::list>();
			for (size_t i = 0; i < dataCollectionSize_; i++)
			{
				IfcSchema::IfcProduct::list::ptr productList = datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcProduct>();
				for (auto et = productList->begin(); et != productList->end(); ++et)
				{
					IfcSchema::IfcProduct* product = *et;
					if (customDivType != boost::to_upper_copy<std::string>(product->data().type()->name())) { continue; }
					selectedlist.get()->push(product);
				}
			}

			auto startTime = std::chrono::high_resolution_clock::now();
			addObjectListToIndex<IfcSchema::IfcProduct::list::ptr>(selectedlist);
			std::cout << "\t" + customDivType + " objects finished in : " <<
				std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << 
				UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
		}
	}

	if (settingsCollection.makeInterior())
	{
		auto startTime = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < dataCollectionSize_; i++)
			addObjectListToIndex<IfcSchema::IfcSpace::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcSpace>(), true);
		std::cout << "\tIfcRoom objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
	}
	std::cout << std::endl;

	// find valid voids
	//applyVoids(); 
	hasIndex_ = true;
}


bg::model::box < BoostPoint3D > helper::makeObjectBox(const TopoDS_Shape& productShape, const double& rotationAngle)
{
	std::vector<gp_Pnt> productVert = helperFunctions::shape2PointList(productShape);
	if (productVert.size() <= 1) { throw ErrorID::warningFailedObjectSimplefication; }

	// only outputs 2 corners of the three needed corners!
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	helperFunctions::rotatedBBoxDiagonal(productVert, &lllPoint, &urrPoint, rotationAngle);

	BoostPoint3D boostlllpoint = helperFunctions::Point3DOTB(lllPoint);
	BoostPoint3D boosturrpoint = helperFunctions::Point3DOTB(urrPoint);

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}


bool helper::isInWall(const bg::model::box<BoostPoint3D>& bbox)
{
	gp_Pnt lll = helperFunctions::Point3DBTO(bbox.min_corner());
	gp_Pnt urr = helperFunctions::Point3DBTO(bbox.max_corner());
	gp_Pnt mP = gp_Pnt((lll.X() + urr.X()) / 2, (lll.Y() + urr.Y()) / 2, (lll.Z() + urr.Z()) / 2);

	BRepExtrema_DistShapeShape distanceMeasurer;
	distanceMeasurer.LoadS1(BRepBuilderAPI_MakeVertex(mP));

	// get potential nesting objects
	std::vector<Value> qResult;
	qResult.clear();
	index_.query(bgi::intersects(bbox), std::back_inserter(qResult));

	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	for (size_t i = 0; i < qResult.size(); i++)
	{
		indexMutex_.lock_shared();
		std::shared_ptr<lookupValue> lookup = productLookup_.at(qResult[i].second);
		indexMutex_.unlock_shared();

		IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();
		std::string qProductType = qProduct->data().type()->name();

		if (openingObjects.find(qProductType) == openingObjects.end()) { continue; }

		std::string objectType = qProduct->data().type()->name();

		indexMutex_.lock_shared();
		auto typeSearch = productIndxLookup_.find(objectType);
		if (typeSearch == productIndxLookup_.end()) 
		{ 
			indexMutex_.unlock_shared();
			continue; 
		}
		indexMutex_.unlock_shared();

		auto objectSearch = typeSearch->second.find(qProduct->GlobalId());
		if (objectSearch == typeSearch->second.end()) { continue; }
		
		indexMutex_.lock_shared();
		TopoDS_Shape qUntrimmedShape = productLookup_[objectSearch->second]->getProductShape();
		indexMutex_.unlock_shared();
		TopExp_Explorer expl;
		for (expl.Init(qUntrimmedShape, TopAbs_SOLID); expl.More(); expl.Next()) {
			// get distance to isolated actual object
			distanceMeasurer.LoadS2(expl.Current());
			distanceMeasurer.Perform();

			if (!distanceMeasurer.InnerSolution()) { continue; }
			if (distanceMeasurer.Value() > 0.2) { continue; }

			return true;
		}
	}
	return false;
}


TopoDS_Shape helper::boxSimplefy(const TopoDS_Shape& shape) //TODO: go through this
{
	// find most occuring horizontal and vertical edge
	std::vector<gp_Pnt> pointList;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		pointList.emplace_back(p);
	}

	std::vector<std::pair<gp_Vec, int>> HorizontalVertPair;
	std::vector<std::pair<gp_Vec, int>> VerticalVertPair;

	for (size_t i = 0; i < pointList.size(); i += 2)
	{
		gp_Pnt p1 = pointList[i];
		gp_Pnt p2 = pointList[i + 1];

		gp_Vec edgeVec = gp_Vec(p1, p2);

		if (p1.Distance(p2) < 0.01)
		{
			continue;
		}

		p1.SetZ(0);
		p2.SetZ(0);

		if (p1.Distance(p2) != 0)
		{
			gp_Vec projectedVec = gp_Vec(p1, p2);


			bool hFound = false;
			for (auto& vecPair : HorizontalVertPair)
			{
				if (vecPair.first.IsParallel(projectedVec, 0.001))
				{
					vecPair.second += 1;
					hFound = true;
					break;
				}
			}

			if (!hFound)
			{
				HorizontalVertPair.emplace_back(std::pair<gp_Vec, int>(projectedVec, 1));
			}
		}

		if (abs(edgeVec.Z()) < 0.001 ) { 
			continue; 
		}

		bool vFound = false;
		for (auto& vecPair : VerticalVertPair)
		{
			if (vecPair.first.IsParallel(edgeVec, 0.001))
			{
				vecPair.second += 1;
				vFound = true;
				break;
			}
		}

		if (!vFound)
		{
			VerticalVertPair.emplace_back(std::pair<gp_Vec, int>(edgeVec, 1));
		}

	}

	if (!HorizontalVertPair.size() || !VerticalVertPair.size())
	{
		return TopoDS_Shape();
	}

	std::pair<gp_Vec, int> hRotationVec = HorizontalVertPair[0];
	std::pair<gp_Vec, int> vRotationVec = VerticalVertPair[0];

	for (auto& vecPair : HorizontalVertPair)
	{
		if (hRotationVec.second < vecPair.second)
		{
			hRotationVec = vecPair;
		}
	}

	for (auto& vecPair : VerticalVertPair)
	{
		if (vRotationVec.second < vecPair.second)
		{
			vRotationVec = vecPair;
		}
	}

	// compute horizontal rotaion
	gp_Pnt p1 = gp_Pnt(0,0,0);
	gp_Pnt p2 = p1.Translated(hRotationVec.first);

	double angleFlat = 0;

	if (abs(p1.Y() - p2.Y()) > 0.00001)
	{
		double os = abs(p1.Y() - p2.Y()) / p1.Distance(p2);
		angleFlat = asin(os);

		gp_Pnt tempP = helperFunctions::rotatePointPoint(p2, p1, angleFlat);

		if (Abs(p1.X() - tempP.X()) > 0.01 && Abs(p1.Y() - tempP.Y()) > 0.01)
		{
			angleFlat = -angleFlat;
		}
	}

	// compute vertical rotation
	gp_Pnt p3 = gp_Pnt(0, 0, 0);
	gp_Pnt p4 = helperFunctions::rotatePointPoint(p3.Translated(vRotationVec.first), p3, angleFlat);

	bool isRotated = false;
	if (abs(p3.X() - p4.X()) < 0.01)
	{
		p3 = helperFunctions::rotatePointWorld(p3, M_PI / 2.0);
		p4 = helperFunctions::rotatePointWorld(p4, M_PI / 2.0);
		angleFlat += M_PI / 2.0;
		isRotated = true;
	}

	double angleVert = acos(abs(p4.Z() - p3.Z()) / p3.Distance(p4));

	gp_Pnt lllPoint1;
	gp_Pnt lllPoint2;
	gp_Pnt urrPoint1;
	gp_Pnt urrPoint2;
	helperFunctions::rotatedBBoxDiagonal(pointList, &lllPoint1, &urrPoint1, angleFlat, angleVert);
	helperFunctions::rotatedBBoxDiagonal(pointList, &lllPoint2, &urrPoint2, angleFlat, -angleVert);
	if (lllPoint1.IsEqual(urrPoint1, SettingsCollection::getInstance().precision())) { std::cout << "false" << std::endl; return TopoDS_Shape(); }

	if (lllPoint1.Distance(urrPoint1) < lllPoint2.Distance(urrPoint2))
	{
		return helperFunctions::createBBOXOCCT(lllPoint1, urrPoint1, 0.0, angleFlat, angleVert);
	}
	return  helperFunctions::createBBOXOCCT(lllPoint2, urrPoint2, 0.0, angleFlat, -angleVert);
}


void helper::getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData, gp_Trsf* trsf)
{
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();

#if defined(USE_IFC4) || defined(USE_IFC4x3)
	IfcSchema::IfcMapConversion::list::ptr mapList = fileObejct->instances_by_type<IfcSchema::IfcMapConversion>();
	if (mapList->size() == 0) { return; }
	if (mapList->size() > 1) { 
		ErrorCollection::getInstance().addError(ErrorID::warningIfcMultipleProjections);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcMultipleProjections) << std::endl;
	}
	
	IfcSchema::IfcMapConversion* mapConversion = *(mapList->begin());

	metaData->setReferenceSystem(mapConversion->TargetCRS()->Name());

	if (mapConversion->Scale().has_value())
	{
		std::array<double, 3> scaleCity = transformation->getScale();
		double scaleIfc = mapConversion->Scale().get();

		for (size_t i = 0; i < scaleCity.size(); i++)
		{
			scaleCity[i] = scaleCity[i] * scaleIfc;
		}
		transformation->setScale(scaleCity);
	}

	if (!mapConversion->XAxisAbscissa().has_value() || !mapConversion->XAxisOrdinate().has_value()) { return; }

	double XAO = mapConversion->XAxisOrdinate().get();
	double XAA = mapConversion->XAxisAbscissa().get();

	trsf->SetValues(
		XAA, -XAO, 0, 0,
		XAO, XAA, 0, 0,
		0, 0, 1, 0
	);

	gp_XYZ invertedObjectTrsf = objectTranslation_.Inverted().TranslationPart();

	transformation->setTranslation(
		mapConversion->Eastings() + invertedObjectTrsf.X(),
		mapConversion->Northings() + invertedObjectTrsf.Y(),
		mapConversion->OrthogonalHeight()
	);
#else
	IfcSchema::IfcSite::list::ptr ifcSiteList = fileObejct->instances_by_type<IfcSchema::IfcSite>();

	if (ifcSiteList->size() == 0) { return; }
	if (ifcSiteList->size() > 1) { std::cout << "[WARNING] multiple sites detected" << std::endl; }

	IfcSchema::IfcSite* ifcSite = *ifcSiteList->begin();
	IfcSchema::IfcRelDefines::list::ptr relDefinesList = ifcSite->IsDefinedBy();

	double Eastings = 0;
	double Northings = 0;
	double OrthogonalHeight = 0;

	double XAO = 0;
	double XAA = 0;

	int mapConvCounter = 0;
	for (auto relIt = relDefinesList->begin(); relIt != relDefinesList->end(); ++relIt)
	{
		IfcSchema::IfcRelDefines* relDefinesObject = *relIt;
		int targetId = std::stoi(relDefinesObject->get("RelatingPropertyDefinition")->toString().erase(0, 1));
		IfcSchema::IfcPropertySet* propertySet = fileObejct->instance_by_id(targetId)->as<IfcSchema::IfcPropertySet>();

		if (propertySet->Name().get() != "ePSet_MapConversion") { continue; }

		IfcSchema::IfcProperty::list::ptr propertyList = propertySet->HasProperties();
		for (auto propIt = propertyList->begin(); propIt != propertyList->end(); ++propIt)
		{
			IfcSchema::IfcPropertySingleValue* propertyObject = (*propIt)->as<IfcSchema::IfcPropertySingleValue>();
			std::string propertyName = propertyObject->Name();

			if (propertyName == "TargetCRS")
			{
				IfcSchema::IfcLabel* crsLabel = propertyObject->NominalValue()->as<IfcSchema::IfcLabel>();
				std::string stringcrs = crsLabel->data().getArgument(0)->toString();
				metaData->setReferenceSystem(stringcrs.substr(1, stringcrs.size() - 2));
				mapConvCounter++;
			}
			else if (propertyName == "Scale")
			{
				//TODO: fix
				IfcSchema::IfcReal* scaleReal = propertyObject->NominalValue()->as<IfcSchema::IfcReal>();
				//transformation->setScale(*transformation->getScale() * double(*scaleReal->data().getArgument(0))); 
				mapConvCounter++;
			}
			else if (propertyName == "Eastings")
			{
				IfcSchema::IfcLengthMeasure* lengthValue = propertyObject->NominalValue()->as<IfcSchema::IfcLengthMeasure>();
				Eastings = double(*lengthValue->data().getArgument(0));
				mapConvCounter++;
			}
			else if (propertyName == "Northings")
			{
				IfcSchema::IfcLengthMeasure* lengthValue = propertyObject->NominalValue()->as<IfcSchema::IfcLengthMeasure>();
				Northings = double(*lengthValue->data().getArgument(0));
				mapConvCounter++;
			}
			else if (propertyName == "OrthogonalHeight")
			{
				IfcSchema::IfcLengthMeasure* lengthValue = propertyObject->NominalValue()->as<IfcSchema::IfcLengthMeasure>();
				OrthogonalHeight = double(*lengthValue->data().getArgument(0));
				mapConvCounter++;
			}
			else if (propertyName == "XAxisAbscissa")
			{
				IfcSchema::IfcReal* abReal = propertyObject->NominalValue()->as<IfcSchema::IfcReal>();
				XAA = double(*abReal->data().getArgument(0));
				mapConvCounter++;
			}
			else if (propertyName == "XAxisOrdinate")
			{
				IfcSchema::IfcReal* sordReal = propertyObject->NominalValue()->as<IfcSchema::IfcReal>();
				XAO = double(*sordReal->data().getArgument(0));
				mapConvCounter++;
			}
		}
	}

	if (mapConvCounter != 7)
	{
		std::cout << "[WARNING] no valid georeferencing found" << std::endl;
		return;
	}

	trsf->SetValues(
		XAA, -XAO, 0, 0,
		XAO, XAA, 0, 0,
		0, 0, 1, 0
	);

	transformation->setTranslation(
		Eastings,
		Northings,
		OrthogonalHeight
	);
#endif // !USE_IFC4
	return;
}

std::map<std::string, std::string> helper::getBuildingInformation() //TODO: go through this
{
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();

	std::map<std::string, std::string> dictionary;
	IfcSchema::IfcBuilding::list::ptr buildingList = fileObejct->instances_by_type<IfcSchema::IfcBuilding>();

	for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
		IfcSchema::IfcBuilding* building = *it;

		if (building->Description().has_value()) { dictionary.emplace(CJObjectEnum::getString(CJObjectID::ifcDescription), building->Description().get()); }
		if (building->ObjectType().has_value()) { dictionary.emplace(CJObjectEnum::getString(CJObjectID::ifcObjectType), building->ObjectType().get()); }
		if (building->Name().has_value()) { dictionary.emplace(CJObjectEnum::getString(CJObjectID::ifcName), building->Name().get()); }
		if (building->LongName().has_value()) { dictionary.emplace(CJObjectEnum::getString(CJObjectID::ifcLongName), building->LongName().get()); }
	}

	IfcSchema::IfcRelDefinesByProperties::list::ptr propteriesRels = fileObejct->instances_by_type<IfcSchema::IfcRelDefinesByProperties>();

	for (auto it = propteriesRels->begin(); it != propteriesRels->end(); ++it) {
		IfcSchema::IfcRelDefinesByProperties* propteriesRel = *it;

		auto relatedObjects = propteriesRel->RelatedObjects();
		bool isBuilding = false;

		for (auto et = relatedObjects->begin(); et != relatedObjects->end(); ++et) {

			auto* relatedObject = *et;
			if (std::string(relatedObject->data().type()->name().c_str()) != "IfcBuilding") { break; }

			isBuilding = true;
			break;
		}

		if (isBuilding)
		{
			IfcSchema::IfcPropertySet* te = propteriesRel->RelatingPropertyDefinition()->as<IfcSchema::IfcPropertySet>();
			if (propteriesRel->RelatingPropertyDefinition()->data().type()->name() != "IfcPropertySet") { continue; }
			//TODO: improve the datamanagement
			// check if the function will work 
			// 
			//auto relAssociations = propteriesRel->RelatingPropertyDefinition()->data().getArgument(4)->operator IfcEntityList::ptr();

			//for (auto et = relAssociations->begin(); et != relAssociations->end(); ++et) {
			//	auto* relatedObject = *et;
			//	IfcSchema::IfcPropertySingleValue* propertyValue = relatedObject->as<IfcSchema::IfcPropertySingleValue>();

			//	int dataType = propertyValue->NominalValue()->data().getArgument(0)->type();
			//	if (dataType == 3) // If Bool
			//	{
			//		if (propertyValue->NominalValue()->data().getArgument(0)->toString() == ".T.")
			//		{
			//			dictionary.emplace(propertyValue->Name().c_str(), "True");
			//		}
			//		else {
			//			dictionary.emplace(propertyValue->Name().c_str(), "False");
			//		}
			//	}
			//	else if (dataType == 4) // If Area
			//	{
			//		std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();

			//		if (stringValue[stringValue.size() - 1] == '.') { stringValue += "0"; }

			//		if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue + " m^2"); } // TODO: get unit

			//	}
			//	else if (dataType == 5) // If String
			//	{
			//		std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();
			//		stringValue = stringValue.substr(1, stringValue.size() - 2);
			//		if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue); }
			//		continue;
			//	}

			//	if (propertyValue->NominalValue()->data().getArgument(0)->toString().size() != 0)
			//	{
			//		dictionary.emplace(propertyValue->Name().c_str(), propertyValue->NominalValue()->data().getArgument(0)->toString());
			//	}
			//}
		}
	}
	return dictionary;
}


std::string helper::getBuildingName()
{
	IfcParse::IfcFile* fileObject = datacollection_[0]->getFilePtr();

	IfcSchema::IfcBuilding::list::ptr buildingList = fileObject->instances_by_type<IfcSchema::IfcBuilding>();
	
	if (buildingList->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcMultipleBuildingObjects); //TODO: add file path to error
	}
	
	for (IfcSchema::IfcBuilding* building : *buildingList)
	{
		if (building->Name().has_value())
		{
			return building->Name().get();
		}
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNobuildingName);

	return "";
}


std::string helper::getBuildingLongName() //TODO: implement
{
	IfcParse::IfcFile* fileObject = datacollection_[0]->getFilePtr();

	IfcSchema::IfcBuilding::list::ptr buildingList = fileObject->instances_by_type<IfcSchema::IfcBuilding>();

	if (buildingList->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcMultipleBuildingObjects);
	}

	for (IfcSchema::IfcBuilding* building : *buildingList)
	{
		if (building->LongName().has_value())
		{
			return building->LongName().get();
		}
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNobuildingNameLong);

	return "";
}


std::string helper::getProjectName()
{
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();
	IfcSchema::IfcProject::list::ptr projectList = fileObejct->instances_by_type<IfcSchema::IfcProject>();

	if (projectList->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::WarningIfcMultipleProjects);
	}

	for (IfcSchema::IfcProject* project : *projectList)
	{
		if (project->Name().has_value())
		{
			return project->Name().get();
		}
	}
	ErrorCollection::getInstance().addError(ErrorID::WarningIfcNoProjectsName);
	return "";
}


template <typename T>
void helper::addObjectListToIndex(const T& objectList, bool addToRoomIndx) {

	int coreCount = SettingsCollection::getInstance().threadcount();
	int coreUse = coreCount;
	int splitListSize = static_cast<int>(floor(objectList->size() / coreUse));
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;

	if (splitListSize == 0) { coreUse = 1; } //if less objects than threads are present just use 1 thread

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = objectList->begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? objectList->end() : startIdx + splitListSize;

		IfcSchema::IfcProduct::list::ptr subList = boost::make_shared<IfcSchema::IfcProduct::list>();

		for (auto subIterator = startIdx; subIterator != endIdx; ++subIterator)
		{
			subList->push(*subIterator);
		}
		threadList.emplace_back([=]() { addObjectToIndex<IfcSchema::IfcProduct::list::ptr>(subList, addToRoomIndx); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
}

template <typename T>
void helper::addObjectToIndex(const T& objectList, bool addToRoomIndx) {
	for (auto it = objectList->begin(); it != objectList->end(); ++it)
	{
		addObjectToIndex(*it, addToRoomIndx);
	}
}


void helper::addObjectToIndex(IfcSchema::IfcProduct* product, bool addToRoomIndx)
{
	std::string productType = product->data().type()->name();

	// pass over if dub
	bool dub = false;
	indexMutex_.lock_shared();
	for (size_t i = 0; i < productLookup_.size(); i++)
	{
		if (productLookup_.at(i)->getProductPtr()->data().id() == product->data().id()) //TODO: optimize this
		{
			dub = true;
			break;
		}
	}
	indexMutex_.unlock_shared();
	if (dub) { return; }

	TopoDS_Shape shape = getObjectShape(product); //TODO: rewrite to function for family related objects

	bg::model::box <BoostPoint3D> box;
	try
	{
		box = makeObjectBox(shape, 0);
	}
	catch (const ErrorID&)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
		return;
	}

	TopoDS_Shape simpleShape;
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();
	if (openingObjects.find(productType) != openingObjects.end())
	{
		simpleShape = getObjectShape(product, true);
	}

	TopoDS_Shape cbbox;
	if (productType == "IfcDoor" || productType == "IfcWindow")
	{
		cbbox = boxSimplefy(shape);
		if (cbbox.IsNull())
		{
			ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
			return;
		}
	}
	std::shared_ptr<lookupValue> lookup = std::make_shared<lookupValue>(product, shape, simpleShape, cbbox);
	if (addToRoomIndx)
	{
		std::lock_guard<std::mutex> spaceLock(spaceIndexMutex_);
		spaceIndex_.insert(std::make_pair(box, (int)spaceIndex_.size()));
		SpaceLookup_.emplace_back(lookup);
		return;
	}

	indexMutex_.lock();
	int locationIdx = (int)index_.size();
	index_.insert(std::make_pair(box, locationIdx));
	productLookup_.emplace_back(lookup);

	updateBoudingData(box);

	auto typeSearch = productIndxLookup_.find(productType);
	if (typeSearch == productIndxLookup_.end())
	{
		productIndxLookup_.insert({ productType, std::unordered_map < std::string, int >() });
	}
	productIndxLookup_[productType].insert({ product->GlobalId(), locationIdx });
	indexMutex_.unlock();
	return;
}


std::vector<gp_Pnt> helper::getObjectPoints(IfcSchema::IfcProduct* product, bool simple)
{
	TopoDS_Shape productShape = getObjectShape(product, simple, false);
	std::vector<gp_Pnt> pointList = helperFunctions::shape2PointList(productShape);
	return pointList;
}


int helper::getObjectShapeLocation(IfcSchema::IfcProduct* product)
{
	std::string objectType = product->data().type()->name();
	std::shared_lock<std::shared_mutex> lock(indexMutex_);
	auto typeSearch = productIndxLookup_.find(objectType);
	if (typeSearch == productIndxLookup_.end()) { return -1; }

	auto objectSearch = typeSearch->second.find(product->GlobalId());
	if (objectSearch == typeSearch->second.end()) { return -1; }
	return objectSearch->second;
}


void helper::updateBoudingData(const bg::model::box<BoostPoint3D>& box)
{
	if (lllPoint_.X() > box.min_corner().get<0>()) { lllPoint_.SetX(box.min_corner().get<0>()); }
	if (lllPoint_.Y() > box.min_corner().get<1>()) { lllPoint_.SetY(box.min_corner().get<1>()); }
	if (lllPoint_.Z() > box.min_corner().get<2>()) { lllPoint_.SetZ(box.min_corner().get<2>()); }
	if (urrPoint_.X() < box.max_corner().get<0>()) { urrPoint_.SetX(box.max_corner().get<0>()); }
	if (urrPoint_.Y() < box.max_corner().get<1>()) { urrPoint_.SetY(box.max_corner().get<1>()); }
	if (urrPoint_.Z() < box.max_corner().get<2>()) { urrPoint_.SetZ(box.max_corner().get<2>()); }
	return;
}


TopoDS_Shape helper::getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted)
{
	// filter with lookup
	std::string objectType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	if (openingObjects.find(objectType) == openingObjects.end()) { adjusted = false; }

	int obbjectShapeLocation = getObjectShapeLocation(product);

	if (obbjectShapeLocation == -1) { return {}; }
	std::shared_lock<std::shared_mutex> lock(indexMutex_);
	if (adjusted)
	{
		return productLookup_[obbjectShapeLocation]->getSimpleShape();
	}
	return productLookup_[obbjectShapeLocation]->getProductShape();
}


TopoDS_Shape helper::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted, int memoryLocation)
{
	bool memorize = false;
	if (memoryLocation != -1) { memorize = true; }
	std::string objectType = product->data().type()->name();
	if (objectType == "IfcFastener") { return {}; } //TODO: check why this does what it does

	// filter with lookup
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();
	if (openingObjects.find(objectType) == openingObjects.end()) { adjusted = false; }

	TopoDS_Shape potentialShape = getObjectShapeFromMem(product, adjusted); 
	if (!potentialShape.IsNull()) { return potentialShape; }

	IfcSchema::IfcRepresentation* ifc_representation = nullptr;
	if (product->Representation())
	{
		IfcSchema::IfcProductRepresentation* prodrep = product->Representation();
		IfcSchema::IfcRepresentation::list::ptr reps = prodrep->Representations();

		for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
			IfcSchema::IfcRepresentation* rep = *it;
			if (rep->RepresentationIdentifier().get() == "Body") {
				ifc_representation = rep;
				break;
			}
		}
	}

	bool hasHoles = false; //TODO: check this
	if (openingObjects.find(objectType) != openingObjects.end()) { hasHoles = true; }
	if (ifc_representation == nullptr)
	{
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() > 0)
		{
			BRep_Builder builder;
			TopoDS_Compound collection;
			builder.MakeCompound(collection);

			for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
#if defined(USE_IFC4) || defined(USE_IFC4x3)
				IfcSchema::IfcRelAggregates* aggregates = *et;
#else
				IfcSchema::IfcRelDecomposes* aggregates = *et;
#endif // USE_IFC4
				IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

				for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
					IfcSchema::IfcObjectDefinition* aggDef = *rt;

					IfcSchema::IfcProduct* addprod = aggDef->as<IfcSchema::IfcProduct>();

					if (adjusted)
					{
						TopoDS_Shape addshapeSimple = getObjectShape(addprod, true, false);
						builder.Add(collection, addshapeSimple);
					}
					else
					{
						TopoDS_Shape addshape = getObjectShape(addprod, false, false);
						builder.Add(collection, addshape);
					}
				}
			}
			return collection;
		}
		return {};
	}

	IfcGeom::Kernel* kernelObject = nullptr;
	if (dataCollectionSize_ == 1)
	{
		kernelObject = datacollection_[0]->getKernelPtr();
	}
	else {
		for (size_t i = 0; i < dataCollectionSize_; i++)
		{
			try { datacollection_[i]->getFilePtr()->instance_by_guid(product->GlobalId())->data().toString(); }
			catch (const std::exception&) { continue; }

			kernelObject = datacollection_[i]->getKernelPtr();
		}
	}
	if (kernelObject == nullptr)
	{
		return {};
	}
	gp_Trsf trsf;

	convertMutex_.lock();
	kernelObject->convert_placement(product->ObjectPlacement(), trsf);
	convertMutex_.unlock();

	IfcGeom::BRepElement* brep = nullptr;

	if (hasHoles && adjusted) {
		IfcGeom::IteratorSettings iteratorSettings = SettingsCollection::getInstance().simpleIteratorSettings();
		std::lock_guard<std::mutex> lock(convertMutex_);
		brep = kernelObject->convert(iteratorSettings, ifc_representation, product);
	}
	else
	{
		IfcGeom::IteratorSettings iteratorSettings = SettingsCollection::getInstance().iteratorSettings();
		std::lock_guard<std::mutex> lock(convertMutex_);
		brep = kernelObject->convert(iteratorSettings, ifc_representation, product);
	}
	if (brep == nullptr) { return {}; }

	TopoDS_Compound comp;
	gp_Trsf placement;
	gp_Trsf trs;
	trs.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), SettingsCollection::getInstance().gridRotation());
	kernelObject->convert_placement(ifc_representation, placement);
	comp = brep->geometry().as_compound();
	comp.Move(trsf * placement); // location in global space
	comp.Move(objectTranslation_);
	comp.Move(trs);
	helperFunctions::triangulateShape(comp);
	return comp;
}


void helper::updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape)
{
	if (!product->Representation()) { return; }

	helperFunctions::triangulateShape(shape);

	std::string objectType = product->data().type()->name();


	// filter with lookup
	std::shared_lock<std::shared_mutex> lock(indexMutex_);
	if (productIndxLookup_.find(objectType) == productIndxLookup_.end())
	{
		return;
	}

	if (productIndxLookup_[objectType].find(product->GlobalId()) == productIndxLookup_[objectType].end())
	{
		return;
	}

	std::shared_ptr<lookupValue> currentLookupvalue = productLookup_[productIndxLookup_[objectType][product->GlobalId()]];
	currentLookupvalue->setSimpleShape(shape);
}


void helper::applyVoids()
{
	for (size_t i = 0; i < dataCollectionSize_; i++) //TODO: multithread
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		voidShapeAdjust<IfcSchema::IfcWall::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcWall>());
	}
}


std::map<std::string, std::string> helper::getProductPropertySet(const std::string& productGui, int fileNum)
{
	std::map<std::string, std::string> productPoperties;
	IfcSchema::IfcRelDefinesByProperties::list::ptr ODevList = datacollection_[fileNum]->getFilePtr()->instances_by_type<IfcSchema::IfcRelDefinesByProperties>();

	for (auto pSetIt = ODevList->begin(); pSetIt != ODevList->end(); ++pSetIt)
	{
		bool defFloor = false;
		IfcSchema::IfcRelDefinesByProperties* dProp = *pSetIt;

#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcObjectDefinition::list::ptr oDefList = dProp->RelatedObjects();
#else
		IfcSchema::IfcObject::list::ptr oDefList = dProp->RelatedObjects();
#endif // USE_IFC

		for (auto oDefIt = oDefList->begin(); oDefIt != oDefList->end(); ++oDefIt)
		{

#if defined(USE_IFC4) || defined(USE_IFC4x3)
			IfcSchema::IfcObjectDefinition* oDef = *oDefIt;
#else
			IfcSchema::IfcObject* oDef = *oDefIt;
#endif // USE_IFC4

			if (oDef->GlobalId() == productGui) 
			{
				defFloor = true; 
				break;
			}
		}

		if (!defFloor) { continue; }

#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcPropertySetDefinitionSelect* pDef = dProp->RelatingPropertyDefinition();
#else
		IfcSchema::IfcPropertySetDefinition* pDef = dProp->RelatingPropertyDefinition();
#endif // USE_IFC4

		if (pDef->data().type()->name() == "IfcElementQuantity")
		{
			IfcSchema::IfcElementQuantity* eQantCollection = pDef->as<IfcSchema::IfcElementQuantity>();
			auto physicsQuantities = eQantCollection->Quantities();
			
			for (auto pQuanIt = physicsQuantities->begin(); pQuanIt != physicsQuantities->end(); ++pQuanIt)
			{
				IfcSchema::IfcPhysicalQuantity* pQuan = *pQuanIt;

				auto valueList = pQuan->data().attributes();
				try
				{
					std::string attributName = pQuan->Name().c_str();
					std::string attributeValue = pQuan->data().attributes()[3]->toString();
					productPoperties.emplace(attributName, attributeValue);
				}
				catch (const std::exception&)
				{
					continue;
				}
			}
		}
	}
	return productPoperties;
}


template <typename T>
void helper::voidShapeAdjust(T products)
{
	std::unordered_set<std::string> cuttingObjects = SettingsCollection::getInstance().getCuttingObjectsList();
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* wallProduct = *it;
		TopoDS_Shape untrimmedWallShape = getObjectShape(wallProduct, true);
		helperFunctions::printFaces(untrimmedWallShape);

		// get the voids
		IfcSchema::IfcElement* wallElement = wallProduct->as<IfcSchema::IfcElement>();
		IfcSchema::IfcRelVoidsElement::list::ptr voidElementList = wallElement->HasOpenings();
		std::vector<TopoDS_Shape> validVoidShapes;
		double voidCount = voidElementList->size();

		// find if the voids are filled or not
		for (auto et = voidElementList->begin(); et != voidElementList->end(); ++et) {
			IfcSchema::IfcRelVoidsElement* voidElement = *et;
			IfcSchema::IfcFeatureElementSubtraction* openingElement = voidElement->RelatedOpeningElement();
			TopoDS_Shape substractionShape = getObjectShape(openingElement);

			gp_Pnt lllPoint;
			gp_Pnt urrPoint;
			helperFunctions::rotatedBBoxDiagonal(getObjectPoints(openingElement), &lllPoint, &urrPoint, 0);

			std::vector<Value> qResult;
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(helperFunctions::Point3DOTB(lllPoint), helperFunctions::Point3DOTB(urrPoint));

			index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

			if (qResult.size() == 0)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}

			BRepClass3d_SolidClassifier insideChecker;
			insideChecker.Load(substractionShape);

			bool inter = false;

			for (size_t i = 0; i < qResult.size(); i++)
			{
				std::shared_ptr<lookupValue> lookup = getLookup(qResult[i].second);
				IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();

				if (cuttingObjects.find(qProduct->data().type()->name()) == cuttingObjects.end()) { continue; }

				TopoDS_Shape qShape = getObjectShape(qProduct);

				for (TopExp_Explorer expl(qShape, TopAbs_VERTEX); expl.More(); expl.Next()) {

					insideChecker.Perform(BRep_Tool::Pnt(TopoDS::Vertex(expl.Current())), 0.01);

					if (insideChecker.State() || insideChecker.IsOnAFace()) {
						inter = true;
						break;
					}
				}

				if (inter)
				{
					break;
				}
			}

			if (inter == false)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}
		}

		if (validVoidShapes.size() == 0)
		{
			continue;
		}
		else if (validVoidShapes.size() == voidCount)
		{
			TopoDS_Shape finalShape = getObjectShape(wallProduct, false);
			updateShapeLookup(wallProduct, finalShape);
		}

		else if (validVoidShapes.size() > 0 && voidElementList->size() > 0) 
		{
			// get a basepoint of the wall
			std::vector<gp_Pnt> pList = helperFunctions::shape2PointList(untrimmedWallShape);

			// bool out voidShape
			BOPAlgo_Splitter aSplitter;
			TopTools_ListOfShape aLSObjects;
			aLSObjects.Append(untrimmedWallShape);
			TopTools_ListOfShape aLSTools;

			for (size_t i = 0; i < validVoidShapes.size(); i++)
			{
				aLSTools.Append(validVoidShapes[i]);
			}

			aLSTools.Reverse();
			aSplitter.SetArguments(aLSObjects);
			aSplitter.SetTools(aLSTools);
			aSplitter.SetRunParallel(Standard_True);
			aSplitter.SetNonDestructive(Standard_True);
			aSplitter.Perform();

			const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

			TopoDS_Shape finalShape;
			std::vector<TopoDS_Solid> solids;
			for (TopExp_Explorer expl(aResult, TopAbs_SOLID); expl.More(); expl.Next()) {
				solids.emplace_back(TopoDS::Solid(expl.Current()));
			}

			for (size_t i = 0; i < pList.size(); i++)
			{
				int count = 0;

				for (size_t j = 0; j < solids.size(); j++)
				{
					for (TopExp_Explorer expl(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
						gp_Pnt evalPoint = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
						if (pList[i].X() == evalPoint.X() &&
							pList[i].Y() == evalPoint.Y() &&
							pList[i].Z() == evalPoint.Z())
						{
							finalShape = solids[j];
							count++;
							break;
						}
					}
				}
				if (count == 1)
				{
					break;
				}
			}
			updateShapeLookup(wallProduct, finalShape);
		}
	}
}


lookupValue::lookupValue(IfcSchema::IfcProduct* productPtr, const TopoDS_Shape& productShape,  const TopoDS_Shape& simpleShape, const TopoDS_Shape& cBox)
{
	productPtr_ = std::make_unique<IfcSchema::IfcProduct>(*productPtr);
	productShape_ = productShape;
	simpleShape_ = simpleShape;
	cBox_ = cBox;

	helperFunctions::triangulateShape(cBox_);

	if (!cBox_.IsNull())
	{
		productPointList_ = helperFunctions::shape2PointList(cBox_);
		
	}
	if (!simpleShape.IsNull())
	{
		productPointList_ = helperFunctions::shape2PointList(simpleShape);
	}
	else
	{
		productPointList_ = helperFunctions::shape2PointList(productShape);
	}

	for (TopExp_Explorer expl(productShape_, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face productFace = TopoDS::Face(expl.Current());

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(productFace, loc);

		if (mesh.IsNull()) { continue; }

		for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

			std::vector<gp_Pnt> trianglePoints{
				mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
			};

			auto box = helperFunctions::createBBox(trianglePoints);
			spatialIndex_.insert(std::make_pair(helperFunctions::createBBox(trianglePoints), productTrianglePoints_.size()));
			productTrianglePoints_.emplace_back(trianglePoints);
		}
	}
}


fileKernelCollection::fileKernelCollection(const std::string& filePath)
{
	file_ = new IfcParse::IfcFile(filePath);
	if (!file_->good()) { return; }
	good_ = true;
	kernel_ = std::make_unique<IfcGeom::Kernel>(file_);;
	IfcGeom::Kernel* kernelObject = kernel_.get();
	kernel_.get()->setValue(kernelObject->GV_PRECISION, SettingsCollection::getInstance().precision());
	setUnits();
}


double fileKernelCollection::getScaleValue(const IfcSchema::IfcSIUnit& unitItem) {
	IfcSchema::IfcSIUnitName::Value unitType = unitItem.Name();

	boost::optional<IfcSchema::IfcSIPrefix::Value> prefixOption = unitItem.Prefix();
	if (!prefixOption) { return 1; }

	if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_METRE)
	{
		if (prefixOption == IfcSchema::IfcSIPrefix::IfcSIPrefix_MILLI) { return 1e-3; }
	}
	else if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_SQUARE_METRE)
	{
		if (prefixOption == IfcSchema::IfcSIPrefix::IfcSIPrefix_MILLI) { return 1e-6; }
	}
	else if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_SQUARE_METRE)
	{
		if (prefixOption == IfcSchema::IfcSIPrefix::IfcSIPrefix_MILLI) { return 1e-9; }
	}
	return 0;
}


void fileKernelCollection::setUnits()
{
	double length = 0;
	double area = 0;
	double volume = 0;

	// unit names if the unit is not standard
	std::string lengthUnitName = "";
	std::string areaUnitName = "";
	std::string volumeUnitName = "";

	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = file_->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		ErrorCollection::getInstance().addError(ErrorID::errorNoUnits);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorNoUnits) << std::endl;
		return;
	}
	else if (presentUnits.get()->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorMultipleUnits);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorMultipleUnits) << std::endl;
		return;
	}

	for (IfcSchema::IfcUnitAssignment::list::it it = presentUnits->begin(); it != presentUnits->end(); ++it)
	{
		const IfcSchema::IfcUnitAssignment* itUnits = *it;
		auto units = itUnits->Units();

		for (auto et = units.get()->begin(); et != units.get()->end(); et++) {
			auto unit = *et;

			int typeInt = unit->declaration().type();
			if (typeInt == 902 || typeInt == 765 ) // select the IfcSIUnit
			{
				IfcSchema::IfcSIUnit* siUnit = unit->as<IfcSchema::IfcSIUnit>();

				if (siUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_LENGTHUNIT) { length = getScaleValue(*siUnit); }
				else if(siUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_AREAUNIT) { area = getScaleValue(*siUnit); }
				if (siUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_VOLUMEUNIT) { volume = getScaleValue(*siUnit); }
			}

			if (typeInt == 233) // select the conversions
			{
				IfcSchema::IfcConversionBasedUnit* conversionUnit = unit->as<IfcSchema::IfcConversionBasedUnit>();
				if (conversionUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_LENGTHUNIT)
				{
					double conversionScale = conversionUnit->ConversionFactor()->ValueComponent()->data().getArgument(0)->operator double();
					length = conversionScale * getScaleValue(*conversionUnit->ConversionFactor()->UnitComponent()->as<IfcSchema::IfcSIUnit>());
					lengthUnitName = conversionUnit->Name();
				}
				else if (conversionUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_AREAUNIT)
				{
					double conversionScale = conversionUnit->ConversionFactor()->ValueComponent()->data().getArgument(0)->operator double();
					area = conversionScale * getScaleValue(*conversionUnit->ConversionFactor()->UnitComponent()->as<IfcSchema::IfcSIUnit>());
					areaUnitName = conversionUnit->Name();
				}
				if (conversionUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_VOLUMEUNIT)
				{
					double conversionScale = conversionUnit->ConversionFactor()->ValueComponent()->data().getArgument(0)->operator double();
					volume = conversionScale * getScaleValue(*conversionUnit->ConversionFactor()->UnitComponent()->as<IfcSchema::IfcSIUnit>());
					volumeUnitName = conversionUnit->Name();
				}
			}
		}
	}

	// check if units have been found
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoFoundUnits) << std::endl;

	std::string lenghtOutputString = "\tLength in ";
	if (!length)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoLengthUnit);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorNoLengthUnit) << std::endl;
		return;
	}
	else if (length == 1) { std::cout << lenghtOutputString << UnitStringEnum::getString(UnitStringID::meterFull) << std::endl; }
	else if (length == 0.001) { std::cout << lenghtOutputString << UnitStringEnum::getString(UnitStringID::millimeterFull) << std::endl; }
	else { std::cout << lenghtOutputString << boost::algorithm::to_lower_copy(lengthUnitName) << std::endl; }

	std::string areaOutputString = "\tArea in square ";
	if (!area)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoAreaUnit);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorNoAreaUnit) << std::endl;
		return;
	}
	else if (area == 1) { std::cout << areaOutputString << UnitStringEnum::getString(UnitStringID::meterFull) << std::endl; }
	else if (area == 0.000001) { std::cout << areaOutputString << UnitStringEnum::getString(UnitStringID::millimeterFull) << std::endl; }
	else { std::cout << areaOutputString << boost::algorithm::to_lower_copy(areaUnitName) << std::endl;}

	std::string volumeOutputString = "\tVolume in cubic ";
	if (!volume)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcNoVolumeUnit);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoVolumeUnit) << std::endl;
		return;
		//std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoDefaultVolumeUnit) << std::endl;
		//volume = 1;
	}
	else if (volume == 1) { std::cout << volumeOutputString << UnitStringEnum::getString(UnitStringID::meterFull) << std::endl; }
	else if (volume == 0.000000001) { std::cout << volumeOutputString << UnitStringEnum::getString(UnitStringID::millimeterFull) << std::endl; }
	else { std::cout << volumeOutputString << boost::algorithm::to_lower_copy(volumeUnitName) << std::endl; }

	std::cout << std::endl;

	//internalize the data
	length_ = length;
	area_ = area;
	volume_ = volume;

	return;
}
