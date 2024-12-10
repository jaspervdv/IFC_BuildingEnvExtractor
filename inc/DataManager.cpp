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

template std::string DataManager::getObjectName<IfcSchema::IfcBuilding>(const std::string& objectTypeName, bool isLong);
template std::string DataManager::getObjectName<IfcSchema::IfcSite>(const std::string& objectTypeName, bool isLong);

template std::string DataManager::getObjectName<IfcSchema::IfcBuilding>(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong);
template std::string DataManager::getObjectName<IfcSchema::IfcSite>(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong);

lookupValue::lookupValue(IfcSchema::IfcProduct* productPtr, const TopoDS_Shape& productShape, const TopoDS_Shape& simpleShape)
{
	productPtr_ = std::make_unique<IfcSchema::IfcProduct>(*productPtr);
	productShape_ = productShape;
	simpleShape_ = simpleShape;

	ErrorCollection& errorCollection = ErrorCollection::getInstance();

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
	std::vector<gp_Pnt> pointList;
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		IfcSchema::IfcSlab::list::ptr slabList = fileObject->instances_by_type<IfcSchema::IfcSlab>();

		std::vector<gp_Pnt> pointLisSlab;
		if (slabList->size())
		{
			pointLisSlab = getObjectListPoints<IfcSchema::IfcSlab::list::ptr>(slabList);
		}
		pointList.insert(pointList.end(), pointLisSlab.begin(), pointLisSlab.end());
	}

	if (!pointList.size())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorNoPoints);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoPoints));
	}

	// if custom roration is required use that for bbox creation
	double rotation = settingsCollection.desiredRotation();
	if (!settingsCollection.autoRotateGrid()) 
	{
		helperFunctions::bBoxDiagonal(pointList, lllPoint, urrPoint, 0, rotation);
		settingsCollection.setGridRotation(settingsCollection.desiredRotation());
		return;
	}

	// compute the smallest orientated bbox
	helperFunctions::bBoxOrientated(pointList, lllPoint, urrPoint, &rotation, 0);
	settingsCollection.setGridRotation(rotation);
	return;
}


void DataManager::computeObjectTranslation(gp_Vec* vec)
{
	// get a point to translate the model to
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcSchema::IfcSlab::list::ptr slabList = datacollection_[i]->getFilePtr()->instances_by_type<IfcSchema::IfcSlab>();

		if (!slabList.get()->size()) { continue; }
		IfcSchema::IfcSlab* slab = *slabList->begin();

		gp_Pnt lllPoint;
		gp_Pnt urrPoint;
		TopoDS_Shape slabShape = getObjectShape(slab, true);
		helperFunctions::bBoxDiagonal(helperFunctions::getPoints(slabShape), &lllPoint, &urrPoint, 0);
		*vec = gp_Vec(-lllPoint.X(), -lllPoint.Y(), 0);
		return;
	}
	ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSlab);
	throw std::string(errorWarningStringEnum::getString(ErrorID::warningIfcNoSlab));
	return;
}

bool DataManager::hasSetUnits() {
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		if (!datacollection_[i]->getLengthMultiplier()) { return false; }
	}
	return true; 
}


void DataManager::internalizeGeo()
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoInternalizingGeo) << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	gp_Vec accuracyObjectTranslation;
	computeObjectTranslation(&accuracyObjectTranslation);
	objectTranslation_.SetTranslationPart(accuracyObjectTranslation);
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

	hasGeo_ = true;
}


void DataManager::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (hasIndex_) { return; }
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCreateSpatialIndex) << std::endl;

	if (settingsCollection.useDefaultDiv())
	{
		timedAddObjectListToIndex<IfcSchema::IfcSlab>("IfcSlab", false);
		timedAddObjectListToIndex<IfcSchema::IfcRoof>("IfcRoof", false);
		timedAddObjectListToIndex<IfcSchema::IfcWall>("IfcWall", false);
		timedAddObjectListToIndex<IfcSchema::IfcCovering>("IfcCovering", false);
		timedAddObjectListToIndex<IfcSchema::IfcColumn>("IfcColumn", false);
		timedAddObjectListToIndex<IfcSchema::IfcPlate>("IfcPlate", false);
		timedAddObjectListToIndex<IfcSchema::IfcMember>("IfcMember", false);
		timedAddObjectListToIndex<IfcSchema::IfcWindow>("IfcWindow", false);
		timedAddObjectListToIndex<IfcSchema::IfcDoor>("IfcDoor", false);
		//addObjectListToIndex<IfcSchema::IfcCurtainWall>("IfcCurtainWall", false);

		if (settingsCollection.useProxy())
		{
			timedAddObjectListToIndex<IfcSchema::IfcBuildingElementProxy>("IfcBuildingElementProxy", false);
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
		timedAddObjectListToIndex<IfcSchema::IfcSpace>("IfcSpace", true);
	}
	std::cout << std::endl;

	// find valid voids
	if (settingsCollection.simplefyGeoGrade() == 1)
	{
		applyVoids();
	}
	hasIndex_ = true;
}


void DataManager::getProjectionData(CJT::ObjectTransformation* transformation, CJT::metaDataObject* metaData, gp_Trsf* trsf)
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

std::map<std::string, std::string> DataManager::getBuildingInformation() //TODO: go through this
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

template <typename T>
std::string DataManager::getObjectName(const std::string& objectTypeName, bool isLong)
{
	std::vector<std::string> stringList;
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		IfcParse::IfcFile* fileObject = datacollection_[i]->getFilePtr();
		std::string nameString = getObjectName<T>(objectTypeName, fileObject, isLong);
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
std::string DataManager::getObjectName(const std::string& objectTypeName, IfcParse::IfcFile* filePtr, bool isLong)
{
	T::list::ptr objectList = filePtr->instances_by_type<T>();

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


template <typename T>
void DataManager::addObjectListToIndex(const T& objectList, bool addToRoomIndx)
{
	if (!objectList->size()) { return; }

	int coreCount = SettingsCollection::getInstance().threadcount();
	int coreUse = coreCount;
	int splitListSize = static_cast<int>(floor(objectList->size() / coreUse));

	std::vector<std::thread> threadList;

	if (splitListSize == 0) { coreUse = 1; } //if less objects than threads are present just use 1 thread
	coreUse = 1; //TODO: fix this

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

template<typename IfcType>
void DataManager::timedAddObjectListToIndex(const std::string& typeName, bool addToRoomIndx)
{
	std::cout << "\t" + typeName + " objects ";
	auto startTime = std::chrono::high_resolution_clock::now();
	for (size_t i = 0; i < dataCollectionSize_; i++)
	{
		addObjectListToIndex<IfcType::list::ptr>(datacollection_[i]->getFilePtr()->instances_by_type<IfcType>());
	}
	
	std::cout << "finished in: " <<
		std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << 
		UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
}

void DataManager::addObjectToIndex(const IfcSchema::IfcProduct::list::ptr objectList, bool addToRoomIndx) {
	for (auto it = objectList->begin(); it != objectList->end(); ++it)
	{
		addObjectToIndex(*it, addToRoomIndx);
	}
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

void DataManager::addObjectToIndex(IfcSchema::IfcProduct* product, bool addToRoomIndx)
{
	// pass over if dub
	if (getObjectShapeLocation(product) != -1) { return; }
	TopoDS_Shape shape = getObjectShape(product); //TODO: rewrite to function for family related objects

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

	TopoDS_Shape simpleShape;
	std::string productType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();
	if (openingObjects.find(productType) != openingObjects.end())
	{
		simpleShape = getObjectShape(product, true);
	}

	if (productType == "IfcDoor" || productType == "IfcWindow")
	{
		simpleShape = helperFunctions::boxSimplefyShape(shape);
		if (simpleShape.IsNull())
		{
			ErrorCollection::getInstance().addError(ErrorID::warningFailedObjectSimplefication, product->GlobalId());
			return;
		}
	}
	std::shared_ptr<lookupValue> lookup = std::make_shared<lookupValue>(product, shape, simpleShape);
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

std::vector<gp_Pnt> DataManager::getObjectPoints(IfcSchema::IfcProduct* product, bool simple)
{
	TopoDS_Shape productShape = getObjectShape(product, simple);
	std::vector<gp_Pnt> pointList = helperFunctions::getPoints(productShape);
	return pointList;
}

template<typename T>
std::vector<gp_Pnt> DataManager::getObjectListPoints(const T& productList, bool simple)
{
	std::vector<gp_Pnt> pointList;
	for (auto it = productList->begin(); it != productList->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;
		std::vector<gp_Pnt> temp = getObjectPoints(product, simple);

		for (const auto& point : temp) {
			pointList.emplace_back(point);
		}
	}
	return std::vector<gp_Pnt>(pointList);
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

TopoDS_Shape DataManager::getNestedObjectShape(IfcSchema::IfcProduct* product, bool adjusted)
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

				TopoDS_Shape addshapeSimple = getObjectShape(addprod, adjusted);
				builder.Add(collection, addshapeSimple);
			}
		}
		return collection;
	}
	return {};
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

TopoDS_Shape DataManager::getObjectShapeFromMem(IfcSchema::IfcProduct* product, bool adjusted)
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


TopoDS_Shape DataManager::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted)
{
	// filter with lookup
	std::string objectType = product->data().type()->name();
	std::unordered_set<std::string> openingObjects = SettingsCollection::getInstance().getOpeningObjectsList();

	if (SettingsCollection::getInstance().simplefyGeoGrade() == 0) { adjusted = false; }
	else if (SettingsCollection::getInstance().simplefyGeoGrade() == 2) { adjusted = true; }
	else if (openingObjects.find(objectType) == openingObjects.end()) { adjusted = false; }


	// get the object from memory if available
	TopoDS_Shape potentialShape = getObjectShapeFromMem(product, adjusted);
	if (!potentialShape.IsNull()) { return potentialShape; }

	IfcSchema::IfcRepresentation* ifc_representation = getProductRepPtr(product);
	if (ifc_representation == nullptr)
	{
		return getNestedObjectShape(product, adjusted);
	}	

	IfcGeom::Kernel* kernelObject = getKernelObject(product->GlobalId());
	if (kernelObject == nullptr) { return {}; }

	gp_Trsf trsf;
	convertMutex_.lock();
	kernelObject->convert_placement(product->ObjectPlacement(), trsf);
	convertMutex_.unlock();

	IfcGeom::IteratorSettings iteratorSettings = SettingsCollection::getInstance().iteratorSettings(adjusted);
	convertMutex_.lock();
	IfcGeom::BRepElement* brep = kernelObject->convert(iteratorSettings, ifc_representation, product);
	convertMutex_.unlock();
	if (brep == nullptr) { return {}; }

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


void DataManager::updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape)
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


void DataManager::applyVoids()
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoApplyVoids) << std::endl;
	timedVoidShapeAdjust<IfcSchema::IfcWall>("IfcWall");
	timedVoidShapeAdjust<IfcSchema::IfcSlab>("IfcSlab");
	timedVoidShapeAdjust<IfcSchema::IfcRoof>("IfcRoof");
	std::cout << "\n";
}


std::map<std::string, std::string> DataManager::getProductPropertySet(const std::string& productGui, int fileNum)
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
	std::unordered_set<std::string> cuttingObjects = SettingsCollection::getInstance().getCuttingObjectsList();
	for (auto it = productList->begin(); it != productList->end(); ++it) {
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
			helperFunctions::bBoxDiagonal(getObjectPoints(openingElement), &lllPoint, &urrPoint, 0);

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
			std::vector<gp_Pnt> pList = helperFunctions::getPoints(untrimmedWallShape);

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
		ErrorCollection::getInstance().addError(ErrorID::errorMultipleUnits);
		std::cout << errorWarningStringEnum::getString(ErrorID::errorMultipleUnits) << std::endl;
		return;
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
	std::cout << "\tLength multiplier = " << length_ << std::endl;
	std::cout << std::endl;

	return;
}
