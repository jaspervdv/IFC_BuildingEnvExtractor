#include "DataManager.h"
#include "helper.h"
#include "stringManager.h"
#include "errorCollection.h"
#include "DebugUtils.h"

#include <BOPAlgo_Splitter.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>

#include <thread>
#include <shared_mutex> 
#include <mutex> 

template std::string DataManager::getIfcObjectName<IfcSchema::IfcBuilding>(const std::string& objectTypeName, bool isLong);
template std::string DataManager::getIfcObjectName<IfcSchema::IfcSite>(const std::string& objectTypeName, bool isLong);

template std::string DataManager::getIfcObjectName<IfcSchema::IfcBuilding>(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong);
template std::string DataManager::getIfcObjectName<IfcSchema::IfcSite>(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong);

IfcProductSpatialData::IfcProductSpatialData(IfcSchema::IfcProduct* productPtr, const TopoDS_Shape& productShape)
{
	ErrorCollection& errorCollection = ErrorCollection::getInstance();

	productPtr_ = std::make_unique<IfcSchema::IfcProduct>(*productPtr);
	productShape_ = productShape;
	std::string objectType = productPtr->data().type()->name();

	for (TopExp_Explorer expl(productShape_, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face productFace = TopoDS::Face(expl.Current());

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(productFace, loc);

		if (mesh.IsNull()) { 
			errorCollection.addError(ErrorID::warningUnableToMesh, productPtr->GlobalId());
			continue; 
		}

		for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

			std::vector<gp_Pnt> trianglePoints{
				mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
				mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
			};

			auto box = helperFunctions::createBBox(trianglePoints);
			triangleIndex_.insert(std::make_pair(helperFunctions::createBBox(trianglePoints), productTrianglePoints_.size()));
			productTrianglePoints_.emplace_back(MeshTriangle(trianglePoints));
		}
	}

	std::string productType = productPtr_->data().type()->name();
	if (productType == "IfcDoor" || productType == "IfcWindow")
	{
		isDetailed_ = true;
	}
}

double fileKernelCollection::getSiPrefixValue(const IfcSchema::IfcSIUnit& unitItem) {
	boost::optional<IfcSchema::IfcSIPrefix::Value> prefixOption = unitItem.Prefix();
	if (!prefixOption) { return 1; }

	switch (*prefixOption) {
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_EXA:   return 1e18;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_PETA:  return 1e15;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_TERA:  return 1e12;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_GIGA:  return 1e9;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_MEGA:  return 1e6;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_KILO:  return 1e3;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_HECTO: return 1e2;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_DECA:  return 10;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_DECI:  return 1e-1;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_CENTI: return 1e-2;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_MILLI: return 1e-3;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_MICRO: return 1e-6;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_NANO:  return 1e-9;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_PICO:  return 1e-12;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_FEMTO: return 1e-15;
	case IfcSchema::IfcSIPrefix::IfcSIPrefix_ATTO:	return 1e-18;
	default: return 0;
	}
}

double fileKernelCollection::getSiScaleValue(const IfcSchema::IfcSIUnit& unitItem) {

	double prefixValue = getSiPrefixValue(unitItem);
	if (!prefixValue) { return 0; }

	IfcSchema::IfcSIUnitName::Value unitType = unitItem.Name();
	if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_METRE)
	{
		return prefixValue;
	}
	else if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_SQUARE_METRE)
	{
		return prefixValue * prefixValue;

	}
	else if (unitType == IfcSchema::IfcSIUnitName::IfcSIUnitName_SQUARE_METRE)
	{
		return prefixValue * prefixValue * prefixValue;
	}
	return 0;
}

fileKernelCollection::fileKernelCollection(const std::string& filePath)
{
	file_ = new IfcParse::IfcFile(filePath);
	if (!file_->good()) { return; }
	kernel_ = std::make_unique<IfcGeom::Kernel>(file_);;
	IfcGeom::Kernel* kernelObject = kernel_.get();
	kernel_.get()->setValue(kernelObject->GV_PRECISION, SettingsCollection::getInstance().spatialTolerance());
	setUnits();
}

void fileKernelCollection::setUnits()
{
	double length = 0;
	double area = 0;
	double volume = 0;

	IfcSchema::IfcUnitAssignment::list::ptr assignedUnitListObjects = file_->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (assignedUnitListObjects.get()->size() == 0) {
		ErrorCollection::getInstance().addError(ErrorID::errorNoUnits);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorNoUnits) << std::endl;
		return;
	}
	else if (assignedUnitListObjects.get()->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningMultipleUnits);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningMultipleUnits) << std::endl;
	}

	IfcSchema::IfcUnitAssignment* assignedUnitListObject = *assignedUnitListObjects->begin();
	IfcSchema::IfcUnit::list::ptr assignedUnitList = assignedUnitListObject->Units();

	for (IfcSchema::IfcUnit::list::it unitIterator = assignedUnitList->begin(); unitIterator != assignedUnitList->end(); ++unitIterator)
	{
		IfcSchema::IfcUnit* currentUnit = *unitIterator;
		if (currentUnit->declaration().name() == "IfcSIUnit") //comput SI units
		{
			IfcSchema::IfcSIUnit* currentSiUnit = currentUnit->as<IfcSchema::IfcSIUnit>();
			if (currentSiUnit->UnitType() == IfcSchema::IfcUnitEnum::IfcUnit_LENGTHUNIT)
			{
				length = getSiScaleValue(*currentSiUnit);
				continue;
			}
		}
		if (length != 0)
		{
			break;
		}
	}

	//internalize the data
	if (!length)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoLengthUnit);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorNoLengthUnit) << std::endl;
		return;
	}

	length_ = length;

	// print found data to user
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoFoundUnits) << std::endl;
	std::cout << "\tLength multiplier = " << length_ << std::endl << std::endl;
	return;
}


DataManager::DataManager(const std::vector<std::string>& pathList) {
	for (const std::string& path : pathList)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoParsingFile) << path << std::endl;
		if (!findSchema(path)) { 
			continue;
		}

		// make new collection
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

bool DataManager::findSchema(const std::string& path, bool quiet) {
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


void DataManager::elementCountSummary()
{
	// count the proxy amount
	int proxyCount = 0;
	int objectCount = 0;

	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		IfcSchema::IfcProduct::list::ptr products = fileObject->instances_by_type<IfcSchema::IfcProduct>();
		IfcSchema::IfcBuildingElementProxy::list::ptr proxyProducts = fileObject->instances_by_type<IfcSchema::IfcBuildingElementProxy>();

		objectCount += products->size();
		proxyCount += proxyProducts->size();
	}

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << 
		objectCount << " objects found" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << 
		proxyCount << " IfcBuildingElementProxy objects found" << std::endl;

	SettingsCollection::getInstance().setProxyCount(proxyCount);
	SettingsCollection::getInstance().setObjectCount(proxyCount);
	return;
}

void DataManager::computeBoundingData(gp_Pnt* lllPoint, gp_Pnt* urrPoint)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// get the slab pointlist to base the inital bbox on
	std::vector<gp_Pnt> pointList = getObjectListPoints <IfcSchema::IfcSlab> (true);
	if (!pointList.size())
	{
		pointList = getObjectListPoints<IfcSchema::IfcRoof>(true);
	}
	if (!pointList.size())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoPoints);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoPoints));
	}

	// if custom roration is required use that for bbox creation
	double rotation = settingsCollection.desiredRotation() - objectTranslation_.GetRotation().GetRotationAngle();
	if (!settingsCollection.autoRotateGrid()) 
	{
		helperFunctions::bBoxDiagonal(pointList, lllPoint, urrPoint, 0, rotation, 0);
		settingsCollection.setGridRotation(rotation);
		return;
	}
	// compute the smallest orientated bbox
	helperFunctions::bBoxDiagonal(pointList, lllPoint, urrPoint, 0); // compute initial values
	helperFunctions::bBoxOrientated(pointList, lllPoint, urrPoint, &rotation, 0); // compute optimal values
	//TODO: let rotation start on the georef rotation
	settingsCollection.setGridRotation(rotation);
	return;
}

gp_Vec DataManager::computeObjectTranslation()
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	gp_Vec translationVec = computeObjectTranslation("IfcSlab");
	if (translationVec.Magnitude() > precision) { return translationVec; }
	translationVec = computeObjectTranslation("IfcRoof");
	if (translationVec.Magnitude() > precision) { return translationVec; }

	if (SettingsCollection::getInstance().useDefaultDiv())
	{
		for (const std::string& currentType : SettingsCollection::getInstance().getDefaultDivList())
		{
			if (currentType == "IfcSlab" || currentType == "IfcRoof") { continue; }
			translationVec = computeObjectTranslation(currentType);
			if (translationVec.Magnitude() > precision) { return translationVec; }
		}
	}
	if (SettingsCollection::getInstance().useProxy())
	{
		translationVec = computeObjectTranslation("IfcBuildingElementProxy");
		if (translationVec.Magnitude() > precision) { return translationVec; }
	}

	for (const std::string& currentType : SettingsCollection::getInstance().getCustomDivList())	
	{
		if (currentType == "IfcSlab" || currentType == "IfcRoof") { continue; }
		translationVec = computeObjectTranslation(currentType);
		if (translationVec.Magnitude() > precision) { return translationVec; }
	}
	return gp_Vec();
}

gp_Vec DataManager::computeObjectTranslation(const std::string& objectType)
{
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		aggregate_of_instance::ptr productList = datacollection_[i]->getFilePtr()->instances_by_type(objectType);
		if (productList == nullptr) { continue; }
		if (!productList->size()) { continue; }

		for (auto et = productList->begin(); et != productList->end(); ++et)
		{
			IfcUtil::IfcBaseClass* test = *et;
			IfcSchema::IfcProduct* product = (*et)->as<IfcSchema::IfcProduct>();

			gp_Pnt lllPoint;
			gp_Pnt urrPoint;
			TopoDS_Shape slabShape = getObjectShape(product, true);

			if (slabShape.IsNull()) { continue; }
			helperFunctions::bBoxDiagonal(helperFunctions::getPoints(slabShape), &lllPoint, &urrPoint, 0);
			return gp_Vec(-lllPoint.X(), -lllPoint.Y(), -lllPoint.Z());
		}
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSlab);
	std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoSlab) << std::endl;
	return gp_Vec();


}


template<typename IfcType>
void DataManager::timedAddObjectListToIndex(const std::string& typeName, bool addToRoomIndx)
{
	std::cout << "\t" + typeName + " objects ";
	auto startTime = std::chrono::high_resolution_clock::now();
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		typename IfcType::list::ptr objectList = datacollection_[i]->getFilePtr()->instances_by_type<IfcType>();
		addObjectListToIndex<typename IfcType::list::ptr>(objectList, addToRoomIndx);
	}

	std::cout << "finished in: " <<
		std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() <<
		UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
}

void DataManager::timedAddObjectListToIndex(const std::string& typeName)
{
	std::cout << "\t" + typeName + " objects ";
	IfcSchema::IfcProduct::list::ptr selectedlist = boost::make_shared<IfcSchema::IfcProduct::list>();
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		aggregate_of_instance::ptr productList = datacollection_[i]->getFilePtr()->instances_by_type(typeName);

		if (productList == nullptr) { continue; }
		if (!productList->size()) { continue; }

		for (auto et = productList->begin(); et != productList->end(); ++et)
		{
			IfcUtil::IfcBaseClass* test = *et;
			IfcSchema::IfcProduct* product = (*et)->as<IfcSchema::IfcProduct>();
			selectedlist.get()->push(product);
		}
	}

	auto startTime = std::chrono::high_resolution_clock::now();
	addObjectListToIndex<IfcSchema::IfcProduct::list::ptr>(selectedlist);
	std::cout << "finished in: " <<
		std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() <<
		UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
}

template <typename T>
void DataManager::addObjectListToIndex(const T& objectList, bool addToRoomIndx)
{
	if (!objectList->size()) { return; }

	int coreCount = SettingsCollection::getInstance().threadcount();
	int coreUse = coreCount;
	int splitListSize = static_cast<int>(floor(objectList->size() / coreUse));

	std::vector<std::thread> threadList;

	if (splitListSize == 0) { coreUse = 1; } //if less objects than threads are present just use 1 thread
	coreUse = 1;

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = objectList->begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? objectList->end() : startIdx + splitListSize;

		IfcSchema::IfcProduct::list::ptr subList = boost::make_shared<IfcSchema::IfcProduct::list>();

		for (auto subIterator = startIdx; subIterator != endIdx; ++subIterator)
		{
			subList->push(*subIterator);
		}
		threadList.emplace_back([=]() { addObjectToIndex(subList, addToRoomIndx); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
}

void DataManager::addObjectToIndex(const IfcSchema::IfcProduct::list::ptr objectList, bool addToRoomIndx) {
	for (auto it = objectList->begin(); it != objectList->end(); ++it)
	{
		addObjectToIndex(*it, addToRoomIndx);
	}
}

void DataManager::addObjectToIndex(IfcSchema::IfcProduct* product, bool addToRoomIndx)
{
	// pass over if dub
	if (getObjectShapeLocation(product) != -1) { return; }
	std::string productType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	TopoDS_Shape shape;
	if (openingObjects.find(productType) != openingObjects.end())
	{
		shape = getObjectShape(product, true); //TODO: fix bool, rn vague
	}
	else
	{
		shape = getObjectShape(product);
	}

	if (shape.IsNull())
	{
		ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectConversion, product->GlobalId());
		return;
	}

	if (SettingsCollection::getInstance().simplefyGeo())
	{
		if (productType == "IfcDoor" || productType == "IfcWindow")
		{
			const std::vector<std::string>& ignoreList = SettingsCollection::getInstance().getIgnoreSimplificationList();
			if (std::find(ignoreList.begin(), ignoreList.end(), product->GlobalId()) == ignoreList.end())
			{
				shape = helperFunctions::boxSimplefyShape(shape);
				if (shape.IsNull())
				{
					ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
					return;
				}
			}
		}
	}

	bg::model::box <BoostPoint3D> box;
	try
	{
		box = helperFunctions::createBBox(shape, 0);
	}
	catch (const ErrorID&)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
		return;
	}
	if (!helperFunctions::hasVolume(box))
	{
		ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
		return;
	}

	std::shared_ptr<IfcProductSpatialData> lookup = std::make_shared<IfcProductSpatialData>(product, shape);
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


IfcGeom::Kernel* DataManager::getKernelObject(const std::string& productGuid)
{
	if (dataCollectionSize_ == 1)
	{
		return datacollection_[0]->getKernelPtr();
	}
	else {
		for (size_t i = 0; i < dataCollectionSize_; i++)
		{
			try { datacollection_[i]->getFilePtr()->instance_by_guid(productGuid)->data().toString(); }
			catch (const std::exception&) { continue; }

			return datacollection_[i]->getKernelPtr();
		}
	}
	return nullptr;
}

int DataManager::getObjectShapeLocation(IfcSchema::IfcProduct* product)
{
	std::string objectType = product->data().type()->name();
	std::shared_lock<std::shared_mutex> lock(indexMutex_);
	auto typeSearch = productIndxLookup_.find(objectType);
	if (typeSearch == productIndxLookup_.end()) { return -1; }

	auto objectSearch = typeSearch->second.find(product->GlobalId());
	if (objectSearch == typeSearch->second.end()) { return -1; }
	return objectSearch->second;
}


IfcSchema::IfcRepresentation* DataManager::getProductRepPtr(IfcSchema::IfcProduct* product)
{
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
	return ifc_representation;
}

TopoDS_Shape DataManager::getNestedObjectShape(IfcSchema::IfcProduct* product, bool isSimple)
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

				if (getObjectShapeLocation(addprod) != -1) { continue; } // if already internalized ignore

				TopoDS_Shape addshapeSimple = getObjectShape(addprod, isSimple);
				builder.Add(collection, addshapeSimple);
			}
		}
		return collection;
	}
	return {};
}

template<typename T>
std::vector<gp_Pnt> DataManager::getObjectListPoints(bool simple)
{
	std::vector<gp_Pnt> pointList;
	for (const auto& fileObject : datacollection_)
	{
		typename T::list::ptr objectList = fileObject->getFilePtr()->instances_by_type<T>();
		for (auto it = objectList->begin(); it != objectList->end(); ++it) {
			IfcSchema::IfcProduct* product = *it;
			std::vector<gp_Pnt> temp = getObjectPoints(product, simple);

			for (const auto& point : temp) {
				pointList.emplace_back(point);
			}
		}
	}
	return std::vector<gp_Pnt>(pointList);
}

std::vector<gp_Pnt> DataManager::getObjectPoints(IfcSchema::IfcProduct* product, bool simple)
{
	TopoDS_Shape productShape = getObjectShape(product, simple);
 	std::vector<gp_Pnt> pointList = helperFunctions::getPoints(productShape);
	return pointList;
}

void DataManager::updateShapeMemory(IfcSchema::IfcProduct* product, TopoDS_Shape shape)
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

	std::shared_ptr<IfcProductSpatialData> currentLookupvalue = productLookup_[productIndxLookup_[objectType][product->GlobalId()]];
	currentLookupvalue->setProductShape(shape);
}


void DataManager::applyVoids()
{
	//TODO: make this work with the opening objects list 
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoApplyVoids) << std::endl;
	timedVoidShapeAdjust<IfcSchema::IfcWall>("IfcWall");
	timedVoidShapeAdjust<IfcSchema::IfcSlab>("IfcSlab");
	timedVoidShapeAdjust<IfcSchema::IfcRoof>("IfcRoof");
	std::cout << "\n";
}

template <typename T>
void DataManager::timedVoidShapeAdjust(const std::string& typeName)
{
	std::cout << "\t" + typeName + " objects ";
	auto startTime = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < dataCollectionSize_; i++) //TODO: multithread
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		voidShapeAdjust(datacollection_[i]->getFilePtr()->instances_by_type<T>());

		std::cout << "finished in: " <<
			std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() <<
			UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
		return;
	}
}

template <typename T>
void DataManager::voidShapeAdjust(T productList)
{
	for (auto it = productList->begin(); it != productList->end(); ++it)
	{
		IfcSchema::IfcProduct* wallProduct = *it;

		// get the voids
		IfcSchema::IfcElement* objectElement = wallProduct->as<IfcSchema::IfcElement>();
		IfcSchema::IfcRelVoidsElement::list::ptr voidElementList = objectElement->HasOpenings();
		std::vector<TopoDS_Shape> validVoidShapes = computeEmptyVoids(voidElementList);

		if (validVoidShapes.size() == 0)
		{
			continue;
		}
		else if (validVoidShapes.size() == voidElementList->size())
		{
			TopoDS_Shape finalShape = getObjectShape(wallProduct, false);
			updateShapeMemory(wallProduct, finalShape);
		}

		TopoDS_Shape untrimmedWallShape = getObjectShape(wallProduct, true);
		TopoDS_Shape finalShape = applyVoidtoShape(untrimmedWallShape, validVoidShapes);

		if (finalShape.IsNull()) { continue; }
		updateShapeMemory(wallProduct, finalShape);
	}
}

std::vector<TopoDS_Shape> DataManager::computeEmptyVoids(IfcSchema::IfcRelVoidsElement::list::ptr voidElementList)
{
	// find if the voids are filled or not
	std::vector<TopoDS_Shape> emptyVoidShapeList;
	std::unordered_set<std::string> cuttingObjects = SettingsCollection::getInstance().getCuttingObjectsList();

	for (auto et = voidElementList->begin(); et != voidElementList->end(); ++et)
	{
		IfcSchema::IfcRelVoidsElement* voidElement = *et;
		IfcSchema::IfcFeatureElementSubtraction* openingElement = voidElement->RelatedOpeningElement();
		TopoDS_Shape substractionShape = getObjectShape(openingElement);

		// find void occupying objects
		gp_Pnt lllPoint;
		gp_Pnt urrPoint;
		helperFunctions::bBoxDiagonal(getObjectPoints(openingElement), &lllPoint, &urrPoint, 0);

		std::vector<Value> qResult;
		bg::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(
			helperFunctions::Point3DOTB(lllPoint),
			helperFunctions::Point3DOTB(urrPoint)
			);
		index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

		// if no occupying objects found void can be applied
		if (qResult.size() == 0)
		{
			emptyVoidShapeList.emplace_back(substractionShape);
			continue;
		}

		BRepClass3d_SolidClassifier insideChecker;
		insideChecker.Load(substractionShape);

		bool intersects = false;
		for (size_t i = 0; i < qResult.size(); i++)
		{
			std::shared_ptr<IfcProductSpatialData> lookup = getLookup(qResult[i].second);
			IfcSchema::IfcProduct* qProduct = lookup->getProductPtr();
			if (cuttingObjects.find(qProduct->data().type()->name()) == cuttingObjects.end()) { continue; }

			TopoDS_Shape qShape = getObjectShape(qProduct);
			for (TopExp_Explorer expl(qShape, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				insideChecker.Perform(BRep_Tool::Pnt(TopoDS::Vertex(expl.Current())), 0.01);
				if (insideChecker.State() || insideChecker.IsOnAFace())
				{
					intersects = true;
					break;
				}
			}
			if (intersects)
			{
				break;
			}
		}

		if (!intersects)
		{
			emptyVoidShapeList.emplace_back(substractionShape);
			continue;
		}
	}
	return emptyVoidShapeList;
}

TopoDS_Shape DataManager::applyVoidtoShape(const TopoDS_Shape& untrimmedShape, std::vector<TopoDS_Shape>& voidObjectList)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	// bool out voidShape
	BOPAlgo_Splitter aSplitter;
	TopTools_ListOfShape aLSObjects;
	aLSObjects.Append(untrimmedShape);
	TopTools_ListOfShape aLSTools;

	for (size_t i = 0; i < voidObjectList.size(); i++)
	{
		aLSTools.Append(voidObjectList[i]);
	}

	aLSTools.Reverse();
	aSplitter.SetArguments(aLSObjects);
	aSplitter.SetTools(aLSTools);
	aSplitter.SetRunParallel(Standard_True);
	aSplitter.SetNonDestructive(Standard_True);
	aSplitter.Perform();

	const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

	TopoDS_Shape finalShape;
	// get a basepoint of the wall
	gp_Pnt anchorPoint = helperFunctions::getFirstPointShape(untrimmedShape);
	for (TopExp_Explorer solidExpl(aResult, TopAbs_SOLID); solidExpl.More(); solidExpl.Next())
	{
		TopoDS_Solid currentSolid = TopoDS::Solid(solidExpl.Current());
		for (TopExp_Explorer expl(currentSolid, TopAbs_VERTEX); expl.More(); expl.Next()) {
			gp_Pnt evalPoint = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
			if (anchorPoint.IsEqual(evalPoint, precision))
			{
				return currentSolid;
			}
		}
	}
	return TopoDS_Shape();
}


void DataManager::updateBoudingData(const bg::model::box<BoostPoint3D>& box)
{
	if (lllPoint_.X() > box.min_corner().get<0>()) { lllPoint_.SetX(box.min_corner().get<0>()); }
	if (lllPoint_.Y() > box.min_corner().get<1>()) { lllPoint_.SetY(box.min_corner().get<1>()); }
	if (lllPoint_.Z() > box.min_corner().get<2>()) { lllPoint_.SetZ(box.min_corner().get<2>()); }
	if (urrPoint_.X() < box.max_corner().get<0>()) { urrPoint_.SetX(box.max_corner().get<0>()); }
	if (urrPoint_.Y() < box.max_corner().get<1>()) { urrPoint_.SetY(box.max_corner().get<1>()); }
	if (urrPoint_.Z() < box.max_corner().get<2>()) { urrPoint_.SetZ(box.max_corner().get<2>()); }
	return;
}


bool DataManager::validateProjectionData(const nlohmann::json& sitePropertySetData)
{
	std::vector<std::string> missingObjects;

	if (!sitePropertySetData.contains("TargetCRS"))
	{
		missingObjects.emplace_back("TargetCRS");
	}
	else if (!sitePropertySetData["TargetCRS"].is_string())
	{
		missingObjects.emplace_back("TargetCRS");
	}

	if (!sitePropertySetData.count("Scale"))
	{
		missingObjects.emplace_back("Scale");
	}
	else if (!sitePropertySetData["Scale"].is_number())
	{
		missingObjects.emplace_back("Scale");
	}

	if (!sitePropertySetData.contains("Eastings"))
	{
		missingObjects.emplace_back("Eastings");
	}
	else if (!sitePropertySetData["Eastings"].contains("value"))
	{
		missingObjects.emplace_back("Eastings");
	}
	else if (!sitePropertySetData["Eastings"]["value"].is_number())
	{
		missingObjects.emplace_back("Eastings");
	}

	if (!sitePropertySetData.contains("Northings"))
	{
		missingObjects.emplace_back("Northings");
	}
	else if (!sitePropertySetData["Northings"].contains("value"))
	{
		missingObjects.emplace_back("Northings");
	}
	else if (!sitePropertySetData["Northings"]["value"].is_number())
	{
		missingObjects.emplace_back("Northings");
	}

	if (!sitePropertySetData.contains("OrthogonalHeight"))
	{
		missingObjects.emplace_back("OrthogonalHeight");
	}
	else if (!sitePropertySetData["OrthogonalHeight"].contains("value"))
	{
		missingObjects.emplace_back("OrthogonalHeight");
	}
	else if (!sitePropertySetData["OrthogonalHeight"]["value"].is_number())
	{
		missingObjects.emplace_back("OrthogonalHeight");
	}

	if (!sitePropertySetData.count("XAxisAbscissa"))
	{
		missingObjects.emplace_back("XAxisAbscissa");
	}
	else if (!sitePropertySetData["XAxisAbscissa"].is_number())
	{
		missingObjects.emplace_back("XAxisAbscissa");
	}


	if (!sitePropertySetData.count("XAxisOrdinate"))
	{
		missingObjects.emplace_back("XAxisOrdinate");
	}
	else if (!sitePropertySetData["XAxisOrdinate"].is_number())
	{
		missingObjects.emplace_back("XAxisOrdinate");
	}

	if (!missingObjects.empty())
	{
		return true;
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcMissingGeoreference, missingObjects);
	return false;
}

void DataManager::populateAttributeLookup()
{
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcSchema::IfcRelDefinesByProperties::list::ptr relDefList = datacollection_[i]->getFilePtr()->instances_by_type <IfcSchema::IfcRelDefinesByProperties>();
		for (auto reldefIt = relDefList->begin(); reldefIt != relDefList->end(); reldefIt++)
		{
			IfcSchema::IfcRelDefinesByProperties* relDefItem = *reldefIt;
#if defined(USE_IFC4) || defined(USE_IFC4x3)
			IfcSchema::IfcObjectDefinition::list::ptr relatedObjectList = relDefItem->RelatedObjects();
#else
			IfcSchema::IfcObject::list::ptr relatedObjectList = relDefItem->RelatedObjects();
#endif

			std::vector<std::string> GuidList;
			GuidList.reserve(relatedObjectList->size());
			for (auto objectIt = relatedObjectList->begin(); objectIt != relatedObjectList->end(); objectIt++)
			{
				GuidList.emplace_back((*objectIt)->GlobalId());
			}

#if defined(USE_IFC4) || defined(USE_IFC4x3)
			IfcSchema::IfcPropertySetDefinitionSelect* propertyDef = relDefItem->RelatingPropertyDefinition();
#else
			IfcSchema::IfcPropertySetDefinition* propertyDef = relDefItem->RelatingPropertyDefinition();
#endif
			if (propertyDef == nullptr) { continue; }
			if (propertyDef->data().type()->name() != "IfcPropertySet") { continue; }
			IfcSchema::IfcPropertySet* propertySet = relDefItem->RelatingPropertyDefinition()->as<IfcSchema::IfcPropertySet>();

			for (const std::string currentGuid : GuidList)
			{
				if (attributeLookup_.find(currentGuid) != attributeLookup_.end())
				{
					attributeLookup_[currentGuid].emplace_back(propertySet);
					continue;
				}

				std::vector< IfcSchema::IfcPropertySet*> propertySetList = { propertySet };
				attributeLookup_.emplace(std::make_pair(currentGuid, propertySetList));
			}
		}
	}
	return;
}


bool DataManager::hasSetUnits() {
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		if (!datacollection_[i]->getLengthMultiplier()) { return false; }
	}
	return true; 
}

std::vector<IfcParse::IfcFile*> DataManager::getSourceFiles() const
{
	std::vector<IfcParse::IfcFile*> ptrList;
	ptrList.reserve(dataCollectionSize_);
	for (int i = 0; 1 < dataCollectionSize_; i ++) { ptrList.emplace_back(datacollection_[i].get()->getFilePtr()); }
	return ptrList;
}


void DataManager::internalizeGeo()
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoInternalizingGeo) << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();
	//combine the georef transformation from the ifc file with the local origin offset
	gp_Trsf geoTrsf = getProjectionTransformation();

	objectTranslation_.SetRotation(geoTrsf.GetRotation()); //set the objectranslation to the rotation only
	gp_Vec ifcTrsf(0,0,0);
	if (SettingsCollection::getInstance().correctPlacement())
	{
		ifcTrsf = computeObjectTranslation();
		objectTranslation_.SetTranslationPart(ifcTrsf);
	}

	objectIfcTranslation_.SetTranslationPart(-ifcTrsf + geoTrsf.TranslationPart());
	elementCountSummary();
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
}

void DataManager::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (index_.size() > 0) { return; }
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCreateSpatialIndex) << std::endl;

	if (settingsCollection.useDefaultDiv())
	{
		bool addToRoomIndex = false;
		timedAddObjectListToIndex<IfcSchema::IfcSlab>("IfcSlab", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcRoof>("IfcRoof", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcWall>("IfcWall", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcCovering>("IfcCovering", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcBeam>("IfcBeam", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcColumn>("IfcColumn", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcPlate>("IfcPlate", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcMember>("IfcMember", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcWindow>("IfcWindow", addToRoomIndex);
		timedAddObjectListToIndex<IfcSchema::IfcDoor>("IfcDoor", addToRoomIndex);
		//addObjectListToIndex<IfcSchema::IfcCurtainWall>("IfcCurtainWall", addToRoomIndex);

		if (settingsCollection.useProxy())
		{
			bool addToRoomIndex = false;
			timedAddObjectListToIndex<IfcSchema::IfcBuildingElementProxy>("IfcBuildingElementProxy", addToRoomIndex);
		}
	}
	else // add custom set div objects
	{
		std::vector<std::string> customDivTypeList = settingsCollection.getCustomDivList();
		for (const std::string& customDivType : customDivTypeList)
		{
			timedAddObjectListToIndex(customDivType);
		}
	}

	if (settingsCollection.makeInterior())
	{
		bool addToRoomIndex = true;
		timedAddObjectListToIndex<IfcSchema::IfcSpace>("IfcSpace", addToRoomIndex);
	}
	std::cout << std::endl;
	// find valid voids
	if (settingsCollection.applyVoidGrade() == 1)
	{
		applyVoids();
	}
	return;
}


gp_Trsf DataManager::getProjectionTransformation()
{
	IfcParse::IfcFile* fileObject = datacollection_[0]->getFilePtr();
#if defined(USE_IFC4) || defined(USE_IFC4x3)
	IfcSchema::IfcMapConversion::list::ptr mapList = fileObject->instances_by_type<IfcSchema::IfcMapConversion>();
	if (mapList->size() == 0) { return gp_Trsf(); }
	if (mapList->size() > 1) {
		ErrorCollection::getInstance().addError(ErrorID::warningIfcMultipleProjections);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcMultipleProjections) << std::endl;
	}

	gp_Trsf trsf;
	IfcSchema::IfcMapConversion* mapConversion = *(mapList->begin());

	if (!mapConversion->XAxisAbscissa().has_value() || !mapConversion->XAxisOrdinate().has_value()) { return gp_Trsf(); }
	double XAO = mapConversion->XAxisOrdinate().get();
	double XAA = mapConversion->XAxisAbscissa().get();
	trsf.SetValues(
		XAA, -XAO, 0, 0,
		XAO, XAA, 0, 0,
		0, 0, 1, 0
	);

	trsf.SetTranslationPart(gp_Vec(mapConversion->Eastings(), mapConversion->Northings(), mapConversion->OrthogonalHeight()));
#else
	IfcSchema::IfcSite::list::ptr ifcSiteList = fileObject->instances_by_type<IfcSchema::IfcSite>();

	if (ifcSiteList->size() == 0) { return gp_Trsf(); }
	if (ifcSiteList->size() > 1) { std::cout << "[WARNING] multiple sites detected" << std::endl; }

	IfcSchema::IfcSite* ifcSite = *ifcSiteList->begin();
	IfcSchema::IfcRelDefines::list::ptr relDefinesList = ifcSite->IsDefinedBy();

	nlohmann::json sitePropertySetData = collectPropertyValues(ifcSite->GlobalId(), "ePSet_MapConversion");
	if (sitePropertySetData.empty()) { return gp_Trsf(); }
	if (validateProjectionData(sitePropertySetData)) { return gp_Trsf(); }

	double Eastings = sitePropertySetData["Eastings"]["value"];
	double Northings = sitePropertySetData["Northings"]["value"];
	double OrthogonalHeight = sitePropertySetData["OrthogonalHeight"]["value"];
	double XAA = sitePropertySetData["XAxisAbscissa"];
	double XAO = sitePropertySetData["XAxisOrdinate"];

	gp_Trsf trsf;
	trsf.SetValues(
		XAA, -XAO, 0, 0,
		XAO, XAA, 0, 0,
		0, 0, 1, 0
	);
	trsf.SetTranslationPart(gp_Vec(Eastings, Northings, OrthogonalHeight));

#endif // !USE_IFC4
	return trsf;
}


void DataManager::getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData)
{
	IfcParse::IfcFile* fileObject = datacollection_[0]->getFilePtr();
#if defined(USE_IFC4) || defined(USE_IFC4x3)
	IfcSchema::IfcMapConversion::list::ptr mapList = fileObject->instances_by_type<IfcSchema::IfcMapConversion>();
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
	gp_XYZ invertedObjectTrsf = objectIfcTranslation_.TranslationPart();

	transformation->setTranslation(
		invertedObjectTrsf.X(),
		invertedObjectTrsf.Y(),
		invertedObjectTrsf.Z()
	);
#else
	IfcSchema::IfcSite::list::ptr ifcSiteList = fileObject->instances_by_type<IfcSchema::IfcSite>();

	if (ifcSiteList->size() != 0) {
		if (ifcSiteList->size() > 1) { std::cout << "[WARNING] multiple sites detected" << std::endl; }

		IfcSchema::IfcSite* ifcSite = *ifcSiteList->begin();
		IfcSchema::IfcRelDefines::list::ptr relDefinesList = ifcSite->IsDefinedBy();

		nlohmann::json sitePropertySetData = collectPropertyValues(ifcSite->GlobalId(), "ePSet_MapConversion");
		if (sitePropertySetData.empty()) { return; }

		if (sitePropertySetData.contains("TargetCRS"))
		{
			if (sitePropertySetData["TargetCRS"].is_string())
			{
				metaData->setReferenceSystem(sitePropertySetData["TargetCRS"]);
			}
		}
		if (sitePropertySetData.contains("Scale"))
		{
			if (sitePropertySetData["Scale"].is_number())
			{
				transformation->setScale(transformation->getScale()[0] * sitePropertySetData["Scale"].get<float>());
			}	
		}
	}
	
	gp_XYZ invertedObjectTrsf = objectIfcTranslation_.TranslationPart();
	transformation->setTranslation(
		invertedObjectTrsf.X(),
		invertedObjectTrsf.Y(),
		invertedObjectTrsf.Z()
	);
#endif // !USE_IFC4
	return;
}

nlohmann::json DataManager::getBuildingInformation()
{
	nlohmann::json dictionary;

	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();

		IfcSchema::IfcBuilding::list::ptr buildingList = fileObject->instances_by_type<IfcSchema::IfcBuilding>();

		for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
			IfcSchema::IfcBuilding* building = *it;

			if (building->Description().has_value()) { dictionary[CJObjectEnum::getString(CJObjectID::ifcDescription)] = building->Description().get(); }
			if (building->ObjectType().has_value()) { dictionary[CJObjectEnum::getString(CJObjectID::ifcObjectType)] = building->ObjectType().get(); }
			if (building->Name().has_value()) { dictionary[CJObjectEnum::getString(CJObjectID::ifcName)] = building->Name().get(); }
			if (building->LongName().has_value()) { dictionary[CJObjectEnum::getString(CJObjectID::ifcLongName)] = building->LongName().get(); }

			nlohmann::json psetMapList = collectPropertyValues(building->GlobalId());

			for (auto jsonObIt = psetMapList.begin(); jsonObIt != psetMapList.end(); ++jsonObIt) {
				dictionary[sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + jsonObIt.key()] = jsonObIt.value();
			}
		}
	}
	return dictionary;
}

template <typename T>
std::string DataManager::getIfcObjectName(const std::string& objectTypeName, bool isLong)
{
	std::vector<std::string> stringList;
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		std::string nameString = getIfcObjectName<T>(objectTypeName, fileObject, isLong);
		if (nameString == "") { continue; }
		stringList.emplace_back(nameString);
	}

	if (stringList.size() == 0) { return ""; }
	std::string baseString = stringList[0];
	for (size_t i = 1; i < dataCollectionSize_; i++)
	{
		if (baseString != stringList[i])
		{
			ErrorCollection::getInstance().addError(ErrorID::warningIfcObjectDifferentName, objectTypeName);
			break;
		}
	}
	return baseString;
}

template <typename T>
std::string DataManager::getIfcObjectName(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong)
{
	typename T::list::ptr objectList = filePtr->instances_by_type<T>();

	if (objectList->size() > 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcMultipleUniqueObjects, objectTypeName);
		return "";
	}

	for (T* object : *objectList)
	{
		if (isLong)
		{
			if (object->LongName().has_value())
			{
				return object->LongName().get();
			}
		}
		else
		{
			if (object->Name().has_value())
			{
				return object->Name().get();
			}
		}
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNoObjectName, objectTypeName);
	return "";
}

nlohmann::json DataManager::collectPropertyValues(const std::string& objectId, const std::string& psetName)
{
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		try
		{
			datacollection_[i]->getFilePtr()->instance_by_guid(objectId);
			return collectPropertyValues(objectId, getSourceFile(i), psetName);
		}
		catch (const std::exception&)
		{
			continue;
		}
	}
	return {};
}

nlohmann::json DataManager::collectPropertyValues(const std::string& objectId, int location, const std::string& psetName)
{
	return collectPropertyValues(objectId, getSourceFile(location), psetName);
}

nlohmann::json DataManager::collectPropertyValues(const std::string& objectId, IfcParse::IfcFile* ifcFile, const std::string& psetName)
{
	if (attributeLookup_.empty()) { populateAttributeLookup(); }

	bool searchName = true;
	if (psetName == "") { searchName = false; }

	nlohmann::json attributesList;
	if (attributeLookup_.find(objectId) == attributeLookup_.end()) { return attributesList; }

	std::vector<IfcSchema::IfcPropertySet*> relatedProperties = attributeLookup_[objectId];
	for (const IfcSchema::IfcPropertySet* currentProperty :  relatedProperties)
	{
		IfcSchema::IfcProperty::list::ptr propertyList = currentProperty->HasProperties();

		if (searchName)
		{
			if (!currentProperty->Name()) { continue; }
			if (*currentProperty->Name() != psetName) { continue; }
		}

		for (auto propertyIt = propertyList->begin(); propertyIt != propertyList->end(); propertyIt++)
		{
			if (*propertyIt == nullptr) { continue; }

			if ((*propertyIt)->data().type()->name() != "IfcPropertySingleValue") //TODO: implement IfcPropertyEnumeratedValue
			{
				continue;
			}

			IfcSchema::IfcPropertySingleValue* propertyItem = (*propertyIt)->as<IfcSchema::IfcPropertySingleValue>();
			IfcSchema::IfcValue* ifcValue = ifcValue = propertyItem->NominalValue();
			if (ifcValue == nullptr) { continue; }

			std::string propertyIdName = ifcValue->data().type()->name();

			if (propertyIdName == "IfcIdentifier")
			{
				IfcSchema::IfcIdentifier* propertyValueContainer = ifcValue->as<IfcSchema::IfcIdentifier>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator std::string();
			}
			else if (propertyIdName == "IfcText")
			{
				IfcSchema::IfcText* propertyValueContainer = ifcValue->as<IfcSchema::IfcText>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator std::string();
			}
			else if (propertyIdName == "IfcLabel")
			{
				IfcSchema::IfcLabel* propertyValueContainer = ifcValue->as<IfcSchema::IfcLabel>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator std::string();
			}
			else if (propertyIdName == "IfcLengthMeasure")
			{
				IfcSchema::IfcLengthMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcLengthMeasure>();
				attributesList[propertyItem->Name()] = {
					{CJObjectEnum::getString(CJObjectID::jsonValue), propertyValueContainer->operator double() },
					{CJObjectEnum::getString(CJObjectID::jsonUom) , UnitStringEnum::getString(UnitStringID::meter) } //TODO: update to unit?
				};
			}
			else if (propertyIdName == "IfcAreaMeasure")
			{
				IfcSchema::IfcAreaMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcAreaMeasure>();
				attributesList[propertyItem->Name()] = {
					{CJObjectEnum::getString(CJObjectID::jsonValue), propertyValueContainer->operator double() },
					{CJObjectEnum::getString(CJObjectID::jsonUom) , UnitStringEnum::getString(UnitStringID::sqrMeter) } //TODO: update to set unit?
				};
			}
			else if (propertyIdName == "IfcReal")
			{
				IfcSchema::IfcReal* propertyValueContainer = ifcValue->as<IfcSchema::IfcReal>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator double();
			}
			else if (propertyIdName == "IfcInteger")
			{
				IfcSchema::IfcInteger* propertyValueContainer = ifcValue->as<IfcSchema::IfcInteger>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator int();
			}
			else if (propertyIdName == "IfcPowerMeasure")
			{
				IfcSchema::IfcPowerMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcPowerMeasure>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator double();
			}
			else if (propertyIdName == "IfcThermalTransmittanceMeasure")
			{
				IfcSchema::IfcThermalTransmittanceMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcThermalTransmittanceMeasure>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator double();
			}
			else if (propertyIdName == "IfcBoolean")
			{
				IfcSchema::IfcBoolean* propertyValueContainer = ifcValue->as<IfcSchema::IfcBoolean>();
				attributesList[propertyItem->Name()] = propertyValueContainer->operator bool();
			}
			else if (propertyIdName == "IfcLogical")
			{
				IfcSchema::IfcLogical* propertyValueContainer = ifcValue->as<IfcSchema::IfcLogical>();
				boost::logic::tribool tribool = propertyValueContainer->operator boost::logic::tribool();

				if (tribool.value == tribool.false_value) { attributesList[propertyItem->Name()] = "FALSE"; }
				else if (tribool.value == tribool.true_value) { attributesList[propertyItem->Name()] = "TRUE"; }
				else { attributesList[propertyItem->Name()] = "UNKNOWN"; }
			}
			else
			{
				ErrorCollection::getInstance().addError(ErrorID::propertyNotImplemented, propertyIdName);
			}
		}
	}
	return attributesList;
}

TopoDS_Shape DataManager::getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool isSimple)
{
	// filter with lookup
	std::string objectType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	if (openingObjects.find(objectType) == openingObjects.end() &&
		objectType != "IfcDoor" && objectType != "IfcWindow"
		) { isSimple = false; }

	int obbjectShapeLocation = getObjectShapeLocation(product);

	if (obbjectShapeLocation == -1) { return {}; }
	std::shared_ptr<IfcProductSpatialData> currentProductData = productLookup_[obbjectShapeLocation];

	std::shared_lock<std::shared_mutex> lock(indexMutex_);
	return currentProductData->getProductShape();
}


TopoDS_Shape DataManager::getObjectShape(IfcSchema::IfcProduct* product, bool isSimple)
{
	// filter with lookup
	std::string objectType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	int simplefyGeoGrade = SettingsCollection::getInstance().applyVoidGrade();

	if (simplefyGeoGrade == 0) { isSimple = false; }
	else if (simplefyGeoGrade == 2) { isSimple = true; }
	else if (openingObjects.find(objectType) == openingObjects.end()) { isSimple = false; }

	// get the object from memory if available
	TopoDS_Shape potentialShape = getObjectShapeFromMem(product, isSimple);
	if (!potentialShape.IsNull()) { return potentialShape; }

	IfcSchema::IfcRepresentation* ifc_representation = getProductRepPtr(product);

	if (ifc_representation == nullptr)
	{
		return getNestedObjectShape(product, isSimple);
	}	

	IfcGeom::Kernel* kernelObject = getKernelObject(product->GlobalId());
	if (kernelObject == nullptr) {
		//TODO: add error
		return {}; 
	}

	gp_Trsf trsf;
	convertMutex_.lock();
	kernelObject->convert_placement(product->ObjectPlacement(), trsf);
	convertMutex_.unlock();
	IfcGeom::IteratorSettings iteratorSettings = SettingsCollection::getInstance().iteratorSettings(isSimple);

	convertMutex_.lock();
	IfcGeom::BRepElement* brep = nullptr;
	try
	{
		brep = kernelObject->convert(iteratorSettings, ifc_representation, product);
	}
	catch (const std::exception&)
	{
		//TODO: add error
	}

	convertMutex_.unlock();
	if (brep == nullptr) { 
		//TODO: add error
		return {}; 
	}

	TopoDS_Compound comp;
	gp_Trsf placement;
	gp_Trsf trs;
	trs.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), SettingsCollection::getInstance().gridRotation());
	
	convertMutex_.lock();
	kernelObject->convert_placement(ifc_representation, placement);
	convertMutex_.unlock();

	comp = brep->geometry().as_compound();
	comp.Move(trsf * placement); // location in global space
	comp.Move(objectTranslation_);
	comp.Move(trs);
	helperFunctions::triangulateShape(comp);

	return comp;
}
