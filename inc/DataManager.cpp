#include "DataManager.h"
#include "helper.h"

#include <BOPAlgo_Splitter.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

#include <thread>


helper::helper(const std::vector<std::string>& pathList) {
	for (size_t i = 0; i < pathList.size(); i++)
	{
		std::string path = pathList[i];
		std::cout << "[INFO] Parsing file " << path << std::endl;
		if (!findSchema(pathList[i])) { continue; }
		std::unique_ptr<fileKernelCollection> dataCollection = std::make_unique<fileKernelCollection>(path);
		if (!dataCollection.get()->isGood())
		{
			std::cout << "Unable to parse .ifc file" << std::endl;
			return;
		}

		std::cout << "- Valid IFC file found" << std::endl;
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

	while (linecount < 100 && std::getline(infile, line))
	{
		if (line[0] == '#') 
		{
			if (!quiet)
			{
				std::cout << "[Warning] No valid ifc scheme found" << std::endl;
			}
			infile.close();
			return false;
		}
		if (line.find("FILE_SCHEMA") == std::string::npos)
		{
			linecount++;
			continue;
		}

		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			if (buildVersion != "Ifc4") 
			{ 
				if (!quiet)
				{
					std::cout << "- Incompatible scheme found: IFC4" << std::endl;
				}
				return false; 
			}
			else {
				if (!quiet)
				{
					std::cout << "- Valid scheme found: IFC4" << std::endl;
				}
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			if (buildVersion != "Ifc2x3") 
			{ 
				if (!quiet)
				{
					std::cout << "- Incompatible scheme found: IFC2X3" << std::endl;
				}
				return false; 
			}
			else {
				if (!quiet)
				{
					std::cout << "- Valid scheme found: IFC2X3" << std::endl;
				}
			}
			break;
		}
		else if (line.find("FILE_SCHEMA (('IFC2X3'))") != std::string::npos) {
			if (buildVersion != "Ifc2x3")
			{
				if (!quiet)
				{
					std::cout << "- Incompatible scheme found: IFC2X3" << std::endl;
				}
				return false;
			}
			else
			{
				if (!quiet)
				{
					std::cout << "- Valid scheme found: IFC2X3" << std::endl;
				}
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x1'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x1" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x2'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x2" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x3'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x3" << std::endl;
			}
			break;
		}
	}
	infile.close();
	return true;
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


void helper::computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint, double* originRot)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::vector<gp_Pnt> pointList;
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();

		std::vector<gp_Pnt> pointListWall = getAllTypePoints<IfcSchema::IfcWall::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcWall>());
		std::vector<gp_Pnt> pointListWallSt = getAllTypePoints<IfcSchema::IfcWallStandardCase::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcWallStandardCase>());
		std::vector<gp_Pnt> pointListRoof = getAllTypePoints<IfcSchema::IfcRoof::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcRoof>());
		std::vector<gp_Pnt> pointLisSlab = getAllTypePoints<IfcSchema::IfcSlab::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcSlab>());
		std::vector<gp_Pnt> pointListWindow = getAllTypePoints<IfcSchema::IfcWindow::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcWindow>());

		pointList.reserve(pointListWall.size() + pointListWallSt.size() + pointListRoof.size() + pointLisSlab.size() + pointListWindow.size());

		pointList.insert(pointList.end(), pointListWall.begin(), pointListWall.end());
		pointList.insert(pointList.end(), pointListWallSt.begin(), pointListWallSt.end());
		pointList.insert(pointList.end(), pointListRoof.begin(), pointListRoof.end());
		pointList.insert(pointList.end(), pointLisSlab.begin(), pointLisSlab.end());
		pointList.insert(pointList.end(), pointListWindow.begin(), pointListWindow.end());
	}

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	double rotation = 0;
	int maxIt = 15;

	// set data for a bbox
	helperFunctions::rotatedBBoxDiagonal(pointList, lllPoint, urrPoint, rotation);
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
	*originRot = rotation;
	return;
}


void helper::computeObjectTranslation(gp_Vec* vec)
{
	// get a point to translate the model to
	IfcSchema::IfcSlab::list::ptr slabList = datacollection_[0]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>();

	if (!slabList.get()->size()) {
		std::cout << "[WARNING] no slab objects were found!" << std::endl;
		return; 
	}
	IfcSchema::IfcSlab* slab = *slabList->begin();

	gp_Pnt lllPoint;
	gp_Pnt urrPoint;
	TopoDS_Shape siteShape = getObjectShape(slab, false, false);
	helperFunctions::rotatedBBoxDiagonal(helperFunctions::shape2PointList(siteShape), &lllPoint, &urrPoint, 0);
	*vec = gp_Vec(-lllPoint.X(), -lllPoint.Y(), 0);
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


template<typename T>
void helper::getAllTypePointsPtr(const T& typePtr, std::vector<gp_Pnt>* pointList, bool simple)
{
	for (auto it = typePtr->begin(); it != typePtr->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		std::vector<gp_Pnt> temp = getObjectPoints(product, simple);

		for (const auto& point : temp) {
			pointList->emplace_back(point);
		}
	}
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
	std::cout << "[INFO] Internalizing Geometry of Construction Model" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();
	gp_Vec accuracyObjectTranslation;
	computeObjectTranslation(&accuracyObjectTranslation);
	objectTranslation_.SetTranslationPart(accuracyObjectTranslation);
	elementCountSummary(&hasProxy_, &hasLotProxy_);
	computeBoundingData(&lllPoint_, &urrPoint_, &originRot_);

	std::cout << "\tSuccessfully finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s\n" << std::endl;

	hasGeo_ = true;
}


void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location
	if (!hasIndex_)
	{
		std::cout << "- Create Spatial Index" << std::endl;
		if (!useCustomFull_)
		{
			// add the floorslabs to the rtree
			auto startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++)
				addObjectToIndex<IfcSchema::IfcSlab::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>());
			std::cout << "\tIfcSlab objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++)
				addObjectToIndex<IfcSchema::IfcRoof::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcRoof>());
			std::cout << "\tIfcRoof objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the walls to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++)
				addObjectToIndex<IfcSchema::IfcWall::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcWall>());
			std::cout << "\tIfcWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++)
				addObjectToIndex<IfcSchema::IfcWallStandardCase::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcWallStandardCase>());
			std::cout << "\tIfcWallStandardCase objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcCovering::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcCovering>());
			std::cout << "\tIfcCovering objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the columns to the rtree TODO sweeps
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcColumn::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcColumn>());
			std::cout << "\tIfcColumn objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the beams to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcBeam::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcBeam>());
			std::cout << "\tIfcBeam objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the curtain walls to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcCurtainWall>());
			std::cout << "\tIfcCurtainWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcPlate::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcPlate>());
			std::cout << "\tIfcPlate objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcMember::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcMember>());
			std::cout << "\tIfcMember objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add doors to the rtree (for the appartment detection)
			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcDoor::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcDoor>());
			std::cout << "\tIfcDoor objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			for (size_t i = 0; i < dataCollectionSize_; i++) 
				addObjectToIndex<IfcSchema::IfcWindow::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcWindow>());
			std::cout << "\tIfcWindow objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			if (useProxy_)
			{
				startTime = std::chrono::high_resolution_clock::now();
				for (size_t i = 0; i < dataCollectionSize_; i++) 
					addObjectToIndex<IfcSchema::IfcBuildingElementProxy::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcBuildingElementProxy>());
				std::cout << "\tIfcBuildingElementProxy objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
			}
		}
		if (useCustom_)
		{
			for (size_t i = 0; i < dataCollectionSize_; i++)
			{
				IfcSchema::IfcProduct::list::ptr productList = datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcProduct>();

				for (auto it = roomBoundingObjects_->begin(); it != roomBoundingObjects_->end(); ++it) {
					auto startTime = std::chrono::high_resolution_clock::now();

					IfcSchema::IfcProduct::list::ptr selectedlist(new IfcSchema::IfcProduct::list);
					for (auto et = productList->begin(); et != productList->end(); ++et)
					{
						IfcSchema::IfcProduct* product = *et;
						if (*it == boost::to_upper_copy<std::string>(product->data().type()->name()))
						{
							selectedlist.get()->push(product);
						}
					}

					std::cout << "\t" + *it + " objects finished in : " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

					addObjectToIndex<IfcSchema::IfcProduct::list::ptr>(selectedlist);
				}
			}
		}
		std::cout << std::endl;

		// find valid voids
		applyVoids();
		hasIndex_ = true;
	}
}


bg::model::box < BoostPoint3D > helper::makeObjectBox(IfcSchema::IfcProduct* product)
{
	std::vector<gp_Pnt> productVert = getObjectPoints(product);
	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({ 0,0,0 }, { 0,0,0 }); }

	// only outputs 2 corners of the three needed corners!
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	helperFunctions::rotatedBBoxDiagonal(productVert, &lllPoint, &urrPoint, originRot_);

	BoostPoint3D boostlllpoint = helperFunctions::Point3DOTB(lllPoint);
	BoostPoint3D boosturrpoint = helperFunctions::Point3DOTB(urrPoint);

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}


bg::model::box < BoostPoint3D > helper::makeObjectBox(const std::vector<IfcSchema::IfcProduct*>& products)
{
	std::vector<gp_Pnt> productVert;

	for (size_t i = 0; i < products.size(); i++)
	{
		std::vector<gp_Pnt> singleVerts = getObjectPoints(products[i]);

		for (size_t j = 0; j < singleVerts.size(); j++)
		{
			productVert.emplace_back(singleVerts[j]);
		}
	}

	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({ 0,0,0 }, { 0,0,0 }); }

	// only outputs 2 corners of the three needed corners!
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	helperFunctions::rotatedBBoxDiagonal(productVert, &lllPoint, &urrPoint, originRot_);

	BoostPoint3D boostlllpoint = helperFunctions::Point3DOTB(lllPoint);
	BoostPoint3D boosturrpoint = helperFunctions::Point3DOTB(urrPoint);

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}


TopoDS_Solid helper::makeSolidBox(const gp_Pnt& lll, const gp_Pnt& urr, double angle, double extraAngle)
{
	gp_Ax1 vertRotation(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));

	BRep_Builder brepBuilder;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidbox;
	brepBuilder.MakeSolid(solidbox);

	gp_Pnt p0(helperFunctions::rotatePointWorld(lll.Rotated(vertRotation, extraAngle), -angle));
	gp_Pnt p1 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p2 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p3 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);

	gp_Pnt p4(helperFunctions::rotatePointWorld(urr.Rotated(vertRotation, extraAngle), -angle));
	gp_Pnt p5 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p6 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p7 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);

	bool sameXCoordinate = true;
	bool sameYCoordinate = true;
	bool sameZCoordinate = true;

	double xCoord = p0.X();
	double yCoord = p0.Y();
	double zCoord = p0.Z();
	for (const auto& p : { p1, p2, p3, p4, p5, p6, p7 }) {
		if (abs(p.X() - xCoord) > 0.01) { sameXCoordinate = false; }
		if (abs(p.Y() - yCoord) > 0.01) { sameYCoordinate = false; }
		if (abs(p.Z() - zCoord) > 0.01) { sameZCoordinate = false; }

		if (!sameXCoordinate && !sameYCoordinate && !sameZCoordinate) { break; }
	}

	if (sameXCoordinate || sameYCoordinate || sameZCoordinate) { return TopoDS_Solid(); }

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Edge edge4 = BRepBuilderAPI_MakeEdge(p4, p5);
	TopoDS_Edge edge5 = BRepBuilderAPI_MakeEdge(p5, p6);
	TopoDS_Edge edge6 = BRepBuilderAPI_MakeEdge(p6, p7);
	TopoDS_Edge edge7 = BRepBuilderAPI_MakeEdge(p7, p4);

	TopoDS_Edge edge8 = BRepBuilderAPI_MakeEdge(p0, p6);
	TopoDS_Edge edge9 = BRepBuilderAPI_MakeEdge(p3, p7);
	TopoDS_Edge edge10 = BRepBuilderAPI_MakeEdge(p2, p4);
	TopoDS_Edge edge11 = BRepBuilderAPI_MakeEdge(p1, p5);

	std::vector<TopoDS_Face> faceList;

	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));

	BRepBuilderAPI_Sewing brepSewer;
	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }
	brepSewer.Perform();
	brepBuilder.Add(solidbox, brepSewer.SewedShape());

	return solidbox;
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

	for (size_t i = 0; i < qResult.size(); i++)
	{
		lookupValue* lookup = productLookup_.at(qResult[i].second);
		IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();
		std::string qProductType = qProduct->data().type()->name();

		if (openingObjects_.find(qProductType) == openingObjects_.end()) { continue; }

		auto search = adjustedshapeLookup_.find(qProduct->GlobalId());
		if (search == adjustedshapeLookup_.end()) { continue; }
		TopoDS_Shape qUntrimmedShape = search->second;
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


TopoDS_Shape helper::boxSimplefy(const TopoDS_Shape& shape)
{
	// find longest horizontal and vertical edge
	std::vector<gp_Pnt> pointList;
	std::vector<gp_Pnt> horizontalMaxEdge;
	std::vector<gp_Pnt> verticalMaxEdge;

	double maxHDistance = 0;
	double maxVDistance = 0;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		pointList.emplace_back(p);
	}
	for (size_t i = 0; i < pointList.size(); i += 2)
	{
		gp_Pnt p1 = pointList[i];
		gp_Pnt p2 = pointList[i + 1];

		double vDistance = abs(p1.Z() - p2.Z());

		if (maxVDistance < vDistance)
		{
			maxVDistance = vDistance;
			verticalMaxEdge = { p1, p2 };
		}

		p1.SetZ(0);
		p2.SetZ(0);

		double hDistance = p1.Distance(p2);

		if (hDistance > maxHDistance)
		{
			maxHDistance = hDistance;
			horizontalMaxEdge = { p1, p2 };
		}
	}

	// compute horizontal rotaion
	gp_Pnt p1 = horizontalMaxEdge[0];
	gp_Pnt p2 = horizontalMaxEdge[1];
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
	gp_Pnt p3 = verticalMaxEdge[0];
	gp_Pnt p4 = helperFunctions::rotatePointPoint(verticalMaxEdge[1], p3, angleFlat);

	bool isRotated = false;
	if (abs(p3.X() - p4.X()) < 0.01)
	{
		p3 = helperFunctions::rotatePointWorld(p3, M_PI / 2.0);
		p4 = helperFunctions::rotatePointWorld(p4, M_PI / 2.0);
		angleFlat += M_PI / 2.0;
		isRotated = true;
	}

	double angleVert = acos(abs(p4.Z() - p3.Z()) / p3.Distance(p4));
	gp_Pnt tempPoint = p4.Rotated(gp_Ax1(p3, gp_Dir(0, 1, 0)), angleVert);
	if (Abs(p3.X() - tempPoint.X()) > 0.01 && Abs(p3.Z() - tempPoint.Z()) > 0.01)
	{
		angleVert = -angleVert;
	}

	gp_Pnt lllPoint;
	gp_Pnt urrPoint;
	helperFunctions::rotatedBBoxDiagonal(pointList, &lllPoint, &urrPoint, angleFlat, angleVert);

	if (lllPoint.IsEqual(urrPoint, 1e-6)) { return TopoDS_Shape(); }
	return makeSolidBox(lllPoint, urrPoint, angleFlat, angleVert);
}

void helper::getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData, gp_Trsf* trsf)
{
#ifdef USE_IFC4
	std::map<std::string, std::string> projectionMapMap;


	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();
	IfcSchema::IfcMapConversion::list::ptr mapList = fileObejct->instances_by_type<IfcSchema::IfcMapConversion>();

	if (mapList->size() == 0)
	{
		return;
	}

	if (mapList->size() > 1)
	{
		std::cout << "[WARNING] multiple map projections detected" << std::endl;
	}

	IfcSchema::IfcMapConversion* mapConversion = *(mapList->begin());
	transformation->setTranslation(
		mapConversion->Eastings(),
		mapConversion->Northings(),
		mapConversion->OrthogonalHeight()
	);
	metaData->setReferenceSystem(mapConversion->TargetCRS()->Name());
	transformation->setScale(*transformation->getScale() * mapConversion->Scale());

	double XAO = mapConversion->XAxisOrdinate();
	double XAA = mapConversion->XAxisAbscissa();

	trsf->SetValues(
		XAA, -XAO, 0, 0,
		XAO, XAA, 0, 0,
		0, 0, 1, 0
	);

	return;
#endif // !USE_IFC4
}

std::map<std::string, std::string> helper::getBuildingInformation()
{
	//TODO: improve the datamanagement
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();

	std::map<std::string, std::string> dictionary;
	IfcSchema::IfcBuilding::list::ptr buildingList = fileObejct->instances_by_type<IfcSchema::IfcBuilding>();

	for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
		IfcSchema::IfcBuilding* building = *it;

		if (building->hasDescription()) { dictionary.emplace("Description", building->Description()); }
		if (building->hasObjectType()) { dictionary.emplace("ObjectType", building->ObjectType()); }
		if (building->hasName()) { dictionary.emplace("Name", building->Name()); }
		if (building->hasLongName()) { dictionary.emplace("Long Name", building->LongName()); }
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
			auto relAssociations = propteriesRel->RelatingPropertyDefinition()->data().getArgument(4)->operator IfcEntityList::ptr();

			for (auto et = relAssociations->begin(); et != relAssociations->end(); ++et) {
				auto* relatedObject = *et;
				IfcSchema::IfcPropertySingleValue* propertyValue = relatedObject->as<IfcSchema::IfcPropertySingleValue>();

				int dataType = propertyValue->NominalValue()->data().getArgument(0)->type();
				if (dataType == 3) // If Bool
				{
					if (propertyValue->NominalValue()->data().getArgument(0)->toString() == ".T.")
					{
						dictionary.emplace(propertyValue->Name().c_str(), "True");
					}
					else {
						dictionary.emplace(propertyValue->Name().c_str(), "False");
					}
				}
				else if (dataType == 4) // If Area
				{
					std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();

					if (stringValue[stringValue.size() - 1] == '.') { stringValue += "0"; }

					if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue + " m^2"); } // TODO: get unit

				}
				else if (dataType == 5) // If String
				{
					std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();
					stringValue = stringValue.substr(1, stringValue.size() - 2);
					if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue); }
					continue;
				}

				if (propertyValue->NominalValue()->data().getArgument(0)->toString().size() != 0)
				{
					dictionary.emplace(propertyValue->Name().c_str(), propertyValue->NominalValue()->data().getArgument(0)->toString());
				}
			}
		}
	}
	return dictionary;
}


std::string helper::getBuildingName()
{
	//TODO: improve the datamanagement
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();

	IfcSchema::IfcBuilding::list::ptr buildingList = fileObejct->instances_by_type<IfcSchema::IfcBuilding>();

	IfcSchema::IfcBuilding* building = *buildingList->begin();
	if (building->hasName()) { return building->Name(); }
	return "";
}


std::string helper::getBuildingLongName()
{
	//TODO: improve the datamanagement
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();
	IfcSchema::IfcBuilding::list::ptr buildingList = fileObejct->instances_by_type<IfcSchema::IfcBuilding>();

	IfcSchema::IfcBuilding* building = *buildingList->begin();
	if (building->hasLongName()) { return building->LongName(); }
	return "";
}


std::string helper::getProjectName()
{
	//TODO: improve the datamanagement
	IfcParse::IfcFile* fileObejct = datacollection_[0]->getFilePtr();
	IfcSchema::IfcProject::list::ptr projectList = fileObejct->instances_by_type<IfcSchema::IfcProject>();

	IfcSchema::IfcProject* project = *projectList->begin();
	if (project->hasName()) { return project->Name(); }
	return "";
}


std::vector<std::vector<gp_Pnt>> helper::triangulateProduct(IfcSchema::IfcProduct* product)
{
	return triangulateShape(getObjectShapeFromMem(product, true));
}


std::vector<std::vector<gp_Pnt>> helper::triangulateShape(const TopoDS_Shape& shape)
{
	std::vector<TopoDS_Face> faceList = helperFunctions::shape2FaceList(shape);
	std::vector<std::vector<gp_Pnt>> triangleMeshList;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		BRepMesh_IncrementalMesh faceMesh = BRepMesh_IncrementalMesh(faceList[i], 0.004);

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(faceList[i], loc);
		const gp_Trsf& trsf = loc.Transformation();

		if (mesh.IsNull()) //TODO warning
		{
			continue;
		}

		for (size_t j = 1; j <= mesh.get()->NbTriangles(); j++) //TODO: find out if there is use to keep the opencascade structure
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

			std::vector<gp_Pnt> trianglePoints;
			for (size_t k = 1; k <= 3; k++)
			{
				gp_Pnt p = mesh->Nodes().Value(theTriangle(k)).Transformed(trsf);
				trianglePoints.emplace_back(p);
			}
			triangleMeshList.emplace_back(trianglePoints);
		}
	}
	return triangleMeshList;
}


template <typename T>
void helper::addObjectToIndex(const T& object) {
	for (auto it = object->begin(); it != object->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		bg::model::box <BoostPoint3D> box = makeObjectBox(product);
		if (!helperFunctions::hasVolume(box))
		{
			std::cout << "Failed: " + product->data().toString() << std::endl;
			continue;
		}

		bool dub = false;
		for (size_t i = 0; i < productLookup_.size(); i++)
		{
			if (productLookup_.at(i)->getProductPtr()->data().id() == product->data().id())
			{
				dub = true;
				break;
			}
		}

		if (dub) { continue; }

		TopoDS_Shape shape = getObjectShape(product);
		TopoDS_Shape cbbox;
		bool hasCBBox = false;
		bool matchFound = false;

		std::string productType = product->data().type()->name();

		if (productType == "IfcDoor" || productType == "IfcWindow")
		{
			//if (!isInWall(box)) { cbbox = boxSimplefy(shape); }
			cbbox = boxSimplefy(shape);
		}

		index_.insert(std::make_pair(box, (int)index_.size()));
		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);
		lookupValue* lookup = new lookupValue(product, triangleMeshList, cbbox);
		productLookup_.emplace_back(lookup);
	}
}


std::vector<gp_Pnt> helper::getObjectPoints(IfcSchema::IfcProduct* product, bool simple)
{
	TopoDS_Shape productShape = getObjectShape(product, simple);
	std::vector<gp_Pnt> pointList = helperFunctions::shape2PointList(productShape);
	return pointList;
}


std::vector<TopoDS_Face> helper::getObjectFaces(IfcSchema::IfcProduct* product, bool simple)
{
	if (!product->hasRepresentation()) { return {}; }

	TopoDS_Shape rShape = getObjectShape(product, simple);
	std::vector<TopoDS_Face> faceList = helperFunctions::shape2FaceList(rShape);
	return faceList;
}


TopoDS_Shape helper::getObjectShapeFromMemEmpty(IfcSchema::IfcProduct* product, bool adjusted)
{
	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->GlobalId());
		if (search != shapeLookup_.end()) { return search->second; }
	}
	else if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->GlobalId());
		if (search != adjustedshapeLookup_.end()) { return search->second; }
	}
	return {};
}


TopoDS_Shape helper::getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted)
{
	// filter with lookup
	std::string objectType = product->data().type()->name();
	if (openingObjects_.find(objectType) == openingObjects_.end()) { adjusted = false; }
	return getObjectShapeFromMemEmpty(product, adjusted);
}


TopoDS_Shape helper::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted, bool memorize)
{
	std::string objectType = product->data().type()->name();

	//TODO: find issue with this object with Bologna model
	//TODO: make threadsave

	if (objectType == "IfcFastener") { return {}; }

	// filter with lookup
	if (openingObjects_.find(objectType) == openingObjects_.end()) { adjusted = false; }
	TopoDS_Shape potentialShape = getObjectShapeFromMemEmpty(product, adjusted);

	if (!potentialShape.IsNull()) { return potentialShape; }

	IfcSchema::IfcRepresentation* ifc_representation = 0;
	bool hasbody = false;

	if (product->hasRepresentation())
	{
		IfcSchema::IfcProductRepresentation* prodrep = product->Representation();
		IfcSchema::IfcRepresentation::list::ptr reps = prodrep->Representations();

		for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
			IfcSchema::IfcRepresentation* rep = *it;
			if (rep->RepresentationIdentifier() == "Body") {
				ifc_representation = rep;
				hasbody = true;
				break;
			}
		}
	}

	bool hasHoles = false;
	if (openingObjects_.find(objectType) != openingObjects_.end()) { hasHoles = true; }

	if (!hasbody)
	{
#ifdef USE_IFC4
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() > 0)
		{
			if (decomposedProducts->size() == 0) { return { }; }

			BRep_Builder builder;
			TopoDS_Compound collection;
			builder.MakeCompound(collection);

			BRep_Builder builderSimple;
			TopoDS_Compound collectionSimple;
			builderSimple.MakeCompound(collectionSimple);

			for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
#ifdef USE_IFC4
				IfcSchema::IfcRelAggregates* aggregates = *et;
#else
				IfcSchema::IfcRelDecomposes* aggregates = *et;
#endif // USE_IFC4
				IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

				for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
					IfcSchema::IfcObjectDefinition* aggDef = *rt;

					IfcSchema::IfcProduct* addprod = aggDef->as<IfcSchema::IfcProduct>();

					TopoDS_Shape addshape = getObjectShape(addprod, false, false);
					TopoDS_Shape addshapeSimple = getObjectShape(addprod, true, false);
					builder.Add(collection, addshape);
					builderSimple.Add(collectionSimple, addshapeSimple);
				}
			}

			if (memorize) { shapeLookup_[product->GlobalId()] = collection; }

			if (hasHoles)
			{
				if (memorize) { adjustedshapeLookup_[product->GlobalId()] = collectionSimple; }
			}

			if (adjusted) { return collectionSimple; }
			return collection;
		}
		return {};
	}

	IfcGeom::Kernel* kernelObject = nullptr;
	if (dataCollectionSize_ == 1)
	{
		kernelObject = datacollection_[0]->getKernelPtr();
	}
	else { // TODO: makes this smarter and quicker
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
	TopoDS_Compound comp;
	TopoDS_Compound simpleComp;
	gp_Trsf placement;
	gp_Trsf trsf;

	kernelObject->convert_placement(product->ObjectPlacement(), trsf);
	IfcGeom::IteratorSettings settings;

	IfcGeom::BRepElement<double, double>* brep = nullptr;
	try { brep = kernelObject->convert(settings, ifc_representation, product); }
	catch (const std::exception&) { return {}; }

	if (brep == nullptr) {

		return {}; 
	} //TODO: find manner to aquire data in another manner

	kernelObject->convert_placement(ifc_representation, placement);

	comp = brep->geometry().as_compound();
	comp.Move(trsf * placement); // location in global space
	comp.Move(objectTranslation_);

	if (hasHoles)
	{
		settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true);
		brep = kernelObject->convert(settings, ifc_representation, product);
		kernelObject->convert_placement(ifc_representation, placement);

		simpleComp = brep->geometry().as_compound();
		simpleComp.Move(trsf * placement); // location in global space
		simpleComp.Move(objectTranslation_);
	}

	if (memorize) { shapeLookup_[product->GlobalId()] = comp; }

	if (hasHoles)
	{
		if (memorize)
		{ adjustedshapeLookup_[product->GlobalId()] = simpleComp; }
	}
	if (adjusted) { return simpleComp; }
	return comp;
}


void helper::updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted)
{
	if (!product->hasRepresentation()) { return; }

	// filter with lookup
	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->GlobalId());
		if (search == shapeLookup_.end())
		{
			return;
		}
		shapeLookup_[product->GlobalId()] = shape;
	}

	if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->GlobalId());
		if (search == adjustedshapeLookup_.end())
		{
			return;
		}
		adjustedshapeLookup_[product->GlobalId()] = shape;
	}
}


void helper::updateIndex(IfcSchema::IfcProduct* product, TopoDS_Shape shape) {
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;
	auto base = helperFunctions::rotatedBBoxDiagonal(getObjectPoints(product), &lllPoint, &urrPoint, 0);

	std::vector<Value> qResult;
	boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(helperFunctions::Point3DOTB(lllPoint), helperFunctions::Point3DOTB(urrPoint));

	index_.query(bgi::intersects(qBox), std::back_inserter(qResult));
	for (size_t i = 0; i < qResult.size(); i++)
	{
		lookupValue* lookup = getLookup(qResult[i].second);
		IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();

		if (qProduct->data().id() != product->data().id()) { continue; }

		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);
		updateLookupTriangle(triangleMeshList, qResult[i].second);
	}
}


void helper::applyVoids()
{
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		// TODO: make this work
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		voidShapeAdjust<IfcSchema::IfcWallStandardCase::list::ptr>(fileObject->instances_by_type<IfcSchema::IfcWallStandardCase>());
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

#ifdef USE_IFC4
		IfcSchema::IfcObjectDefinition::list::ptr oDefList = dProp->RelatedObjects();
#else
		IfcSchema::IfcObject::list::ptr oDefList = dProp->RelatedObjects();
#endif // USE_IFC4

		for (auto oDefIt = oDefList->begin(); oDefIt != oDefList->end(); ++oDefIt)
		{

#ifdef USE_IFC4
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

#ifdef USE_IFC4
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
					productPoperties[pQuan->Name().c_str()] = pQuan->data().attributes()[3]->toString();
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
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* wallProduct = *it;
		TopoDS_Shape untrimmedWallShape = getObjectShape(wallProduct, true);

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
			helperFunctions::rotatedBBoxDiagonal(getObjectPoints(openingElement), &lllPoint, &urrPoint, originRot_);

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
				lookupValue* lookup = getLookup(qResult[i].second);
				IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();

				if (cuttingObjects_.find(qProduct->data().type()->name()) == cuttingObjects_.end()) { continue; }

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
			updateShapeLookup(wallProduct, finalShape, true);
			updateIndex(wallProduct, finalShape);
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
			updateShapeLookup(wallProduct, finalShape, true);
			updateIndex(wallProduct, finalShape);
		}
	}
}


lookupValue::lookupValue(IfcSchema::IfcProduct* productPtr, const std::vector<std::vector<gp_Pnt>>& triangulatedShape, const TopoDS_Shape& cBox)
{
	productPtr_ = std::make_unique<IfcSchema::IfcProduct>(*productPtr);
	setTriangluatedShape(triangulatedShape);
	cBox_ = cBox;
}


bool lookupValue::hasTraingulatedShape()
{
	if (triangulatedShape_.get()->size() > 0) { return true; }
	return false;
}


fileKernelCollection::fileKernelCollection(const std::string& filePath)
{
	file_ = new IfcParse::IfcFile(filePath);
	if (!file_->good()) { return; }
	good_ = true;
	kernel_ = std::make_unique<IfcGeom::Kernel>(file_);;
	IfcGeom::Kernel* kernelObject = kernel_.get();
	kernel_.get()->setValue(kernelObject->GV_PRECISION, 1e-6);
	setUnits();
}


void fileKernelCollection::setUnits()
{
	double length = 0;
	double area = 0;
	double volume = 0;

	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = file_->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		std::cout << "[Error] No unit assignment has been found" << std::endl;
		return;
	}
	else if (presentUnits.get()->size() > 1)
	{
		std::cout << "[Error] Multiple unit assignments have been found" << std::endl;
		return;
	}


	for (IfcSchema::IfcUnitAssignment::list::it it = presentUnits->begin(); it != presentUnits->end(); ++it)
	{
		const IfcSchema::IfcUnitAssignment* itUnits = *it;
		auto units = itUnits->Units();

		for (auto et = units.get()->begin(); et != units.get()->end(); et++) {
			auto unit = *et;

			if (unit->declaration().type() == 902 || unit->declaration().type() == 765) // select the IfcSIUnit
			{
				std::string unitType = unit->data().getArgument(1)->toString();
				std::string SiUnitBase = unit->data().getArgument(3)->toString();
				std::string SiUnitAdd = unit->data().getArgument(2)->toString();

				if (unitType == ".LENGTHUNIT.")
				{
					if (SiUnitBase == ".METRE.") { length = 1; }
					if (SiUnitAdd == ".MILLI.") { length = length / 1000; }
				}
				else if (unitType == ".AREAUNIT.")
				{
					if (SiUnitBase == ".SQUARE_METRE.") { area = 1; }
					if (SiUnitAdd == ".MILLI.") { area = area / pow(1000, 2); }
				}
				if (unitType == ".VOLUMEUNIT.")
				{
					if (SiUnitBase == ".CUBIC_METRE.") { volume = 1; }
					if (SiUnitAdd == ".MILLI.") { volume = volume / pow(1000, 3); }
				}
			}
		}
	}

	// check if units have been found
	std::cout << "[INFO] found units:" << std::endl;
	if (!length)
	{
		std::cout << "[Error] SI unit for length cannot be found!" << std::endl;
		return;
	}
	else if (length == 1) { std::cout << "- Lenght in metre" << std::endl; }
	else if (length == 0.001) { std::cout << "- Lenght in millimetre" << std::endl; }

	if (!area)
	{
		std::cout << "[Error] SI unit for area cannot be found!" << std::endl;
		return;
	}
	else if (area == 1) { std::cout << "- Area in square metre" << std::endl; }
	else if (area == 0.000001) { std::cout << "- Area in square millimetre" << std::endl; }

	if (!volume)
	{
		std::cout << "[Warning] SI unit for volume cannot be found!" << std::endl;
		std::cout << "[Warning] SI unit for volume is set to cubic metre!" << std::endl;
		volume = 1;
	}
	else if (volume == 1) { std::cout << "- Volume in cubic metre" << std::endl; }
	else if (volume == 0.000000001) { std::cout << "- Volume in cubic millimetre" << std::endl; }

	std::cout << std::endl;

	//internalize the data
	length_ = length;
	area_ = area;
	volume_ = volume;

	return;
}
