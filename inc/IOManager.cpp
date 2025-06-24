#include "helper.h"
#include "IOManager.h"
#include "cjCreator.h"
#include "stringManager.h"
#include "errorCollection.h"

#include <unordered_set>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <boost/filesystem.hpp>


void addTimeToJSON(nlohmann::json* j, const std::string& valueName, const std::chrono::steady_clock::time_point& startTime, const std::chrono::steady_clock::time_point& endTime)
{
	long long duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5) { (*j)[valueName + UnitStringEnum::getString(UnitStringID::milliseconds)] = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count(); }
	else { (*j)[valueName + UnitStringEnum::getString(UnitStringID::seconds)] = duration; }
}

template<typename T>
void addTimeToJSON(nlohmann::json* j, const std::string& valueName, T duration)
{
	std::string timeUnitString = "Unit";
	std::string timeDurationString = "Duration";
	nlohmann::json timeSet;
	if (duration == 0) { return; }
	else if (duration < 5000)
	{
		timeSet[timeDurationString] = duration;
		timeSet[timeUnitString] = UnitStringEnum::getString(UnitStringID::milliseconds);
	}
	else
	{
		timeSet[timeDurationString] = duration / 1000;
		timeSet[timeUnitString] = UnitStringEnum::getString(UnitStringID::seconds);
	}
	(*j)[valueName] = timeSet;
	return;
}


bool IOManager::hasExtension(const std::string& string, const std::string& ext)
{
	std::string substring = boost::to_lower_copy<std::string>(string.substr(string.find_last_of(".") + 1));
	if (substring == ext) { return true; }
	return false;
}

bool IOManager::isValidPath(const std::string& path)
{
	struct stat info;
	if (stat(path.c_str(), &info) != 0) { return false; }
	return true;
}

std::string IOManager::getTargetPath()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// preload communcation strings
	std::string stringJSONRequest = CommunicationStringEnum::getString(CommunicationStringID::infoJsonRequest);
	std::string stringNoFilePath = CommunicationStringEnum::getString(CommunicationStringID::infoNoFilePath);
	std::string stringNoValFilePath = CommunicationStringEnum::getString(CommunicationStringID::infoNoValFilePath);

	std::cout << stringJSONRequest << std::endl;

	while (true)
	{
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		if (singlepath.size() == 0 && settingsCollection.getIfcPathList().size() == 0)
		{
			std::cout << stringNoFilePath << std::endl;
			std::cout << stringJSONRequest << std::endl;
			continue;
		}
		if (singlepath.front() == '"' && singlepath.back() == '"')
		{
			singlepath = singlepath.substr(1, singlepath.size() - 2);
		}
		if (!hasExtension(singlepath, "json"))
		{
			std::cout << stringNoValFilePath << std::endl;
			std::cout << stringJSONRequest << std::endl;
			continue;
		}
		if (!isValidPath(singlepath))
		{
			std::cout << stringNoValFilePath << std::endl;
			std::cout << stringJSONRequest << std::endl;
			continue;
		}
		return singlepath;
	}
}

bool IOManager::getJSONValues(const std::string& inputPath)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// test if input configuration path is valid
	try { settingsCollection.setInputJSONPath(inputPath, true); }
	catch (const std::string& errorString) { throw errorString; }

	// read config file
	std::ifstream f(settingsCollection.getInputJSONPath());
	nlohmann::json json = nlohmann::json::parse(f);

	// in and output related settings
	// get filepath object
	try { settingsCollection.setIOPaths(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set report output toggle
	try { settingsCollection.setWriteReport(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set the threadcount
	try { settingsCollection.setThreadcount(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set lod output values
	try { settingsCollection.setLoD(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set json related values 
	try { settingsCollection.setJSONRelatedSettings(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set json related values 
	try { settingsCollection.setVoxelRelatedSettings(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set IFC related values 
	try { settingsCollection.setIFCRelatedSettings(json); }
	catch (const std::string& errorString) { throw errorString; }

	try { settingsCollection.setFormatRelatedSettings(json); }
	catch (const std::string& errorString) { throw errorString; }

	// set IFC related values 
	try { settingsCollection.generateGeneralSettings(); }
	catch (const std::string& errorString) { throw errorString; }

	return true;
}


void IOManager::printSummary()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << "\n\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "Used settings:\n\n";

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "I/O settings\n";
	std::cout << "- Configuration file:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getInputJSONPath() << "\n";
	std::cout << "- Input File(s):\n";
	for (const std::string& inputPath : settingsCollection.getIfcPathList()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << inputPath << "\n"; }
	std::cout << "- Output File:\n";
	if (settingsCollection.createJSON())
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getOutputJSONPath() << "\n";
	}
	if (settingsCollection.createSTEP())
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getOutputSTEPPath() << "\n";
	}
	if (settingsCollection.createOBJ())
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getOutputOBJPath() << "\n";
	}

	std::cout << "- Create Report:\n";
	std::cout << boolToString(settingsCollection.writeReport()) << "\n";
	std::cout << "- Report File:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getOutputReportPath() << "\n";
	std::cout << "- LoD export enabled:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << getLoDEnabled() << "\n";
	std::cout << "- Output format:\n";
	std::string formatString = "";
	if (settingsCollection.createJSON()) { formatString += "JSON, "; }
	if (settingsCollection.createSTEP()) { formatString += "STEP, "; }
	if (settingsCollection.createOBJ()) { formatString += "OBJ, "; }
	formatString.erase(formatString.size() - 2);
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << formatString << "\n\n";

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "IFC settings\n";
	std::cout << "- Model rotation:\n";
	if (settingsCollection.autoRotateGrid()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "automatic\n"; } //TODO: add true north?
	else
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.desiredRotation() << "\n";
	}
	std::cout << "- Simplify geometry grade:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.simplefyGeoGrade() << "\n";
	std::cout << "- Space dividing objects:\n";
	if (settingsCollection.useDefaultDiv())
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << boost::to_upper_copy(*it) << "\n";; }
	}
	if (settingsCollection.useProxy())
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "IFCBUILDINGELEMENTPROXY\n";
	}
	std::vector<std::string> addDivObjects = settingsCollection.getCustomDivList();
	for (auto it = addDivObjects.begin(); it != addDivObjects.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << "\n"; }
	std::cout << "\n";

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "voxel settings\n";
	std::cout << "- Voxel size:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.voxelSize() << "\n";
	std::cout << "- Voxel logic:\n";
	if (settingsCollection.intersectionLogic() == 2) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "plane\n"; }
	if (settingsCollection.intersectionLogic() == 3) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "solid\n"; }
	std::cout << "- Store voxel summary/approximation:\n";
	std::cout << boolToString(settingsCollection.summaryVoxels()) << "\n\n";

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "json settings\n";
	std::cout << "- Footprint Elevation:\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.footprintElevation() << "\n";;
	std::cout << "- Footrint based extraction:\n";
	std::cout << boolToString(settingsCollection.footPrintBased()) << "\n";
	std::cout << "- horizontal section offset\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.horizontalSectionOffset() << "\n";;
	if (settingsCollection.make02())
	{
		std::cout << "- Create footprint:\n";
		std::cout << boolToString(settingsCollection.makeFootPrint()) << "\n";

		std::cout << "- Store Lod0.2 roof outline:\n";
		std::cout << boolToString(settingsCollection.makeRoofPrint()) << "\n";
	}
	std::cout << "- Generate interior:\n";
	std::cout << boolToString(settingsCollection.makeInterior()) << "\n";
	std::cout << "- Generate exterior:\n";
	std::cout << boolToString(settingsCollection.makeExterior()) << "\n";
	std::cout << "- Output site:\n";
	std::cout << boolToString(settingsCollection.makeSite()) << "\n";
	std::cout << "- Georeference:\n";
	std::cout << boolToString(settingsCollection.geoReference()) << "\n";
	std::cout << "- Merge semantic objects:\n";
	std::cout << boolToString(settingsCollection.mergeSemantics()) << "\n\n";

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "other settings:\n";
	std::cout << "- Max thread count\n";
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.threadcount() << "\n\n";

	if (settingsCollection.simplefyGeoGrade() != 0 && settingsCollection.summaryVoxels() ||
		settingsCollection.simplefyGeoGrade() != 0 && settingsCollection.makeV() ||
		settingsCollection.simplefyGeoGrade() != 0 && settingsCollection.make32())
	{
		std::cout << errorWarningStringEnum::getString(ErrorID::warningSimplefication) << "\n\n";
		ErrorCollection::getInstance().addError(ErrorID::warningSimplefication);
	}

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << "\n\n";
}

void IOManager::printErrors()
{
	ErrorCollection& errorCol = ErrorCollection::getInstance();

	std::cout << "[INFO] Warnings/Errors:\n";
	if (!errorCol.hasError())
	{
		std::cout << "\tCode 0\n";
		return;
	}

	for (const auto& error : errorCol.getErrorCollection())
	{
		ErrorObject currentError = error.second;
		std::cout << "\tCode " << currentError.errorCode_ << " : " << currentError.errorDescript_ << "\n";
	}
	return;
}

std::string IOManager::boolToString(const bool boolValue)
{
	if (boolValue) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "yes"; }
	else { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "no"; }
}


std::string IOManager::getLoDEnabled()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::string summaryString = "";

	if (settingsCollection.make00()) { summaryString += ", 0.0"; }
	if (settingsCollection.make02()) { summaryString += ", 0.2"; }
	if (settingsCollection.make03()) { summaryString += ", 0.3"; }
	if (settingsCollection.make04()) { summaryString += ", 0.4"; }
	if (settingsCollection.make10()) { summaryString += ", 1.0"; }
	if (settingsCollection.make12()) { summaryString += ", 1.2"; }
	if (settingsCollection.make13()) { summaryString += ", 1.3"; }
	if (settingsCollection.make22()) { summaryString += ", 2.2"; }
	if (settingsCollection.makeb0()) { summaryString += ", b.0"; }
	if (settingsCollection.makec1()) { summaryString += ", c.1"; }
	if (settingsCollection.makec2()) { summaryString += ", c.2"; }
	if (settingsCollection.maked1()) { summaryString += ", d.1"; }
	if (settingsCollection.maked2()) { summaryString += ", d.2"; }
	if (settingsCollection.makee1()) { summaryString += ", e.1"; }
	if (settingsCollection.make32()) { summaryString += ", 3.2"; }
	if (settingsCollection.makeV()) { summaryString += ", 5.0 (V)"; }

	summaryString.erase(0, 2);

	return summaryString;
}


nlohmann::json IOManager::settingsToJSON()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	nlohmann::json settingsJSON; // overal jsonFile

	// store the filepath data
	nlohmann::json ioJSON;
	std::string filePathsOName = JsonObjectInEnum::getString(JsonObjectInID::filePaths);
	std::string inputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathsInput);
	std::string outputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathOutput);

	ioJSON[inputOName] = settingsCollection.getIfcPathList();
	ioJSON[outputOName] = settingsCollection.getOutputJSONPath();
	settingsJSON[filePathsOName] = ioJSON;

	// store the report data
	settingsJSON[JsonObjectInEnum::getString(JsonObjectInID::outputReport)] = "true";

	// store the thread data
	settingsJSON[JsonObjectInEnum::getString(JsonObjectInID::maxThread)] = settingsCollection.threadcount();

	// store the LoD output data
	std::vector<std::string> LoDList;
	if (settingsCollection.make00()) { LoDList.emplace_back("0.0"); }
	if (settingsCollection.make02()) { LoDList.emplace_back("0.2"); }
	if (settingsCollection.make03()) { LoDList.emplace_back("0.3"); }
	if (settingsCollection.make04()) { LoDList.emplace_back("0.4"); }
	if (settingsCollection.make10()) { LoDList.emplace_back("1.0"); }
	if (settingsCollection.make12()) { LoDList.emplace_back("1.2"); }
	if (settingsCollection.make13()) { LoDList.emplace_back("1.3"); }
	if (settingsCollection.make22()) { LoDList.emplace_back("2.2"); }
	if (settingsCollection.makeb0()) { LoDList.emplace_back("b.0"); }
	if (settingsCollection.makec1()) { LoDList.emplace_back("c.1"); }
	if (settingsCollection.makec2()) { LoDList.emplace_back("c.2"); }
	if (settingsCollection.maked1()) { LoDList.emplace_back("d.1"); }
	if (settingsCollection.maked2()) { LoDList.emplace_back("d.2"); }
	if (settingsCollection.makee1()) { LoDList.emplace_back("e.1"); }
	if (settingsCollection.make32()) { LoDList.emplace_back("3.2"); }
	if (settingsCollection.makeV()) { LoDList.emplace_back("5.0"); }

	settingsJSON[JsonObjectInEnum::getString(JsonObjectInID::lodOutput)] = LoDList;

	// store the voxel data
	nlohmann::json voxelJSON;
	std::string voxelOName = JsonObjectInEnum::getString(JsonObjectInID::voxel);
	std::string voxelSizOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSize);
	std::string voxelSummarizeOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSummarize);
	std::string voxelIntersectionOName = JsonObjectInEnum::getString(JsonObjectInID::voxelIntersection);

	voxelJSON[voxelSizOName] = settingsCollection.voxelSize();
	voxelJSON[voxelSummarizeOName] = settingsCollection.summaryVoxels();
	voxelJSON[voxelIntersectionOName] = settingsCollection.intersectionLogic();
	settingsJSON[voxelOName] = voxelJSON;

	//store the ifc data
	nlohmann::json ifcJSON;
	std::string ifcOName = JsonObjectInEnum::getString(JsonObjectInID::IFC);
	std::string ifcRotationOName = JsonObjectInEnum::getString(JsonObjectInID::IFCRotationAngle);
	std::string ifcDivOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDivObject);
	std::string ifcSimpleOName = JsonObjectInEnum::getString(JsonObjectInID::IFCsimplefyGeo);
	
	ifcJSON[ifcRotationOName] = settingsCollection.gridRotation();
	std::vector<std::string> DivList;
	if (settingsCollection.useDefaultDiv())
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it)
		{
			DivList.emplace_back(boost::to_upper_copy(*it));
		}
	}
	if (settingsCollection.useProxy()) { DivList.emplace_back("IFCBUILDINGELEMENTPROXY"); }
	ifcJSON[ifcDivOName] = settingsCollection.getCustomDivList();
	ifcJSON[ifcSimpleOName] = settingsCollection.simplefyGeoGrade();
	settingsJSON[ifcOName] = ifcJSON;

	// store the json data
	nlohmann::json jsonJSON;
	std::string jsonOName = JsonObjectInEnum::getString(JsonObjectInID::JSON);
	std::string jsonFootprintElevOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintElev);
	std::string jsonFootprintBSOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintBShape);
	std::string jsonSecOffsetOName = JsonObjectInEnum::getString(JsonObjectInID::JSONSecOffset);
	std::string jsonGenFootprintOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenFootPrint);
	std::string jsonGenRootprintOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenRoofOutline);
	std::string jsonGenInteriorOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenInterior);
	std::string jsonGenExteriorOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenExterior);
	std::string jsonGensiteOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenSite);
	std::string jsonGeoreferenceOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGeoreference);
	std::string jsonMergeSemanticsOName = JsonObjectInEnum::getString(JsonObjectInID::JSONMergeSemantics);

	jsonJSON[jsonFootprintElevOName] = settingsCollection.footprintElevation();
	jsonJSON[jsonFootprintBSOName] = settingsCollection.footPrintBased();
	jsonJSON[jsonSecOffsetOName] = settingsCollection.horizontalSectionOffset();
	jsonJSON[jsonGenFootprintOName] = settingsCollection.makeFootPrint();
	if (settingsCollection.make02())
	{ 
		jsonJSON[jsonGenFootprintOName] = settingsCollection.makeFootPrint();
		jsonJSON[jsonGenRootprintOName] = settingsCollection.makeRoofPrint();
	}
	jsonJSON[jsonGenInteriorOName] = settingsCollection.makeInterior();
	jsonJSON[jsonGenExteriorOName] = settingsCollection.makeExterior();
	jsonJSON[jsonGensiteOName] = settingsCollection.makeSite();
	jsonJSON[jsonGeoreferenceOName] = settingsCollection.geoReference();
	jsonJSON[jsonMergeSemanticsOName] = settingsCollection.mergeSemantics();
	settingsJSON[jsonOName] = jsonJSON;

	return settingsJSON;
}

void IOManager::internalizeGeo()
{
	// Time Collection Starts
	auto internalizingTime = std::chrono::high_resolution_clock::now();

	// internalize the helper data
	try
	{
		internalDataManager_->internalizeGeo();
		internalDataManager_->indexGeo();
	}
	catch (const std::string& exceptionString)
	{
		throw exceptionString;
	}
	timeInternalizing_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - internalizingTime).count();
	return;
}

void IOManager::setMetaData(std::shared_ptr<CJT::CityCollection> collection)
{
	CJT::ObjectTransformation transformation(0.001);
	CJT::metaDataObject metaData;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	metaData.setTitle(CJObjectEnum::getString(CJObjectID::metaDataTitle));
	if (settingsCollection.geoReference())
	{
		internalDataManager_.get()->getProjectionData(&transformation, &metaData);
	}
	transformation.setScale(transformation.getScale()[0]);

	// compute the extends
	gp_Pnt lll = internalDataManager_.get()->getLllPoint();
	gp_Pnt urr = internalDataManager_.get()->getUrrPoint();

	gp_Trsf gridRotation;
	gridRotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	TopoDS_Shape bboxGeo = helperFunctions::createBBOXOCCT(lll, urr).Moved(gridRotation);
	bboxGeo.Move(internalDataManager_->getObjectNormalizedTranslation());
	bg::model::box <BoostPoint3D> extents = helperFunctions::createBBox(bboxGeo);

	metaData.setExtend(
		CJT::CJTPoint(extents.min_corner().get<0>(), extents.min_corner().get<1>(), extents.min_corner().get<2>()),
		CJT::CJTPoint(extents.max_corner().get<0>(), extents.max_corner().get<1>(), extents.max_corner().get<2>())
	);

	collection->setTransformation(transformation); // set transformation early to avoid geo compression
	collection->setVersion(CJObjectEnum::getString(CJObjectID::v11));
	collection->setMetaData(metaData);
	return;
}

void IOManager::setDefaultSemantic(CJT::CityObject& cityBuildingObject, CJT::CityObject& cityOuterShellObject, CJT::CityObject& cityInnerShellObject)
{
	// Set up objects and their relationships
	std::string BuildingName = internalDataManager_.get()->getIfcObjectName<IfcSchema::IfcBuilding>("IfcBuilding", false);
	if (BuildingName == "") { BuildingName = internalDataManager_.get()->getIfcObjectName<IfcSchema::IfcSite>("IfcSite", false); }
	nlohmann::json buildingAttributes = internalDataManager_.get()->getBuildingInformation();
	cityBuildingObject.addAttributes(buildingAttributes);

	cityBuildingObject.setName(BuildingName);
	cityBuildingObject.setType(CJT::Building_Type::Building);

	cityOuterShellObject.setName(CJObjectEnum::getString(CJObjectID::outerShell));
	cityOuterShellObject.setType(CJT::Building_Type::BuildingPart);

	cityInnerShellObject.setName(CJObjectEnum::getString(CJObjectID::innerShell));
	cityInnerShellObject.setType(CJT::Building_Type::BuildingPart);

	cityBuildingObject.addChild(&cityOuterShellObject);
	cityBuildingObject.addChild(&cityInnerShellObject);

	return;
}

void IOManager::setComputedSemantic(CJGeoCreator* geoCreator, CJT::CityObject& cityBuildingObject, CJT::CityObject& cityOuterShellObject, CJT::CityObject& cityInnerShellObject)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	cityOuterShellObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "footprint elevation", settingsCollection.footprintElevation());
	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "buildingHeight", internalDataManager_.get()->getUrrPoint().Z() - settingsCollection.footprintElevation());

	int storeyFloors = 0;
	int basementFloors = 0;
	ComputeStoreysAboveGround(&storeyFloors, &basementFloors);
	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "storeysAboveGround", storeyFloors);
	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "storeysBelowGround", basementFloors);

	// find location of main entrance (CHEK related code)
	std::unique_ptr<gp_Pnt> entranceCoord = nullptr;
	entranceCoord = FetchMainEntranceLocation(std::move(entranceCoord)); //TODO: make this triggerable
	if (entranceCoord != nullptr)
	{
		cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "Main Entrance coordinate", 
			std::to_string(entranceCoord->X()) + ", " + 
			std::to_string(entranceCoord->Y()) + ", " +
			std::to_string(entranceCoord->Z()));
	}

	if (settingsCollection.summaryVoxels())
	{
		geoCreator->extractOuterVoxelSummary(
			&cityOuterShellObject,
			internalDataManager_.get(),
			settingsCollection.footprintElevation(),
			-settingsCollection.gridRotation()
		);

		geoCreator->extractInnerVoxelSummary(
			&cityInnerShellObject,
			internalDataManager_.get()
		);
	}
	return;
}

void IOManager::ComputeStoreysAboveGround(int* storeysAboveGround, int* storeysBelowGround)
{
	IfcSchema::IfcBuildingStorey::list::ptr storeyList = internalDataManager_.get()->getSourceFile(0)->instances_by_type<IfcSchema::IfcBuildingStorey>();

	double groundHeight = SettingsCollection::getInstance().footprintElevation();
	double smallestDistance = 1000000;
	double groundFloorHeight = 0;
	std::vector<double> storeyHeights;
	for (auto it = storeyList->begin(); it != storeyList->end(); ++it)
	{
		IfcSchema::IfcBuildingStorey* storeyObject = *it;
		double floorHeight = storeyObject->Elevation().get() * internalDataManager_.get()->getScaler(0);
		storeyHeights.emplace_back(floorHeight);

		double distanceToGround = abs(floorHeight - groundHeight);

		if (distanceToGround < smallestDistance)
		{
			smallestDistance = distanceToGround;
			groundFloorHeight = floorHeight;
		}
	}

	int storeyFloors = 0;
	int basementFloors = 0;
	for (double floorHeight : storeyHeights)
	{
		if (floorHeight >= groundFloorHeight) { storeyFloors++; }
		else { basementFloors++; }
	}
	*storeysAboveGround = storeyFloors;
	*storeysBelowGround = basementFloors;

	return;
}

std::unique_ptr<gp_Pnt> IOManager::FetchMainEntranceLocation(std::unique_ptr<gp_Pnt> location)
{
	for (size_t i = 0; i < internalDataManager_->getSourceFileCount(); i++)
	{
		IfcParse::IfcFile* currentFile = internalDataManager_->getSourceFile(i);
		IfcSchema::IfcDoor::list::ptr doorlist  = currentFile->instances_by_type<IfcSchema::IfcDoor>();

		for (auto it = doorlist->begin(); it != doorlist->end(); ++it)
		{
			IfcSchema::IfcDoor* currentDoor = *it;

			nlohmann::json attributeList = helperFunctions::collectPropertyValues(currentDoor->GlobalId(), currentFile);

			if (!attributeList.contains("CHEK_IsMainEntrance"))
			{
				continue;
			}

			// if is mainentrance
			TopoDS_Shape currentShape = internalDataManager_->getObjectShape(currentDoor, true);
			gp_Pnt lll;
			gp_Pnt urr;
			helperFunctions::bBoxDiagonal(currentShape, &lll, &urr);

			gp_Pnt centerPoint = gp_Pnt(
				(lll.X() - urr.X()) / 2,
				(lll.Y() - urr.Y()) / 2,
				lll.Z());

			location = std::make_unique<gp_Pnt>(centerPoint);
			return location;
		}
	}
	return nullptr;
}


void IOManager::processExternalLoD(CJGeoCreator* geoCreator, CJT::CityObject& cityOuterShellObject, CJT::Kernel* kernel)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingExterior) << std::endl;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	
	if (settingsCollection.make02() && settingsCollection.makeFootPrint() || 
		settingsCollection.make03() && settingsCollection.makeFootPrint() ||
		settingsCollection.make04() && settingsCollection.makeFootPrint() ||
		settingsCollection.footPrintBased() || 
		settingsCollection.makeb0()
		)
	{
		try
		{
			geoCreator->makeFootprint(internalDataManager_.get());
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(ErrorID::errorFootprintFailed);
			succesfullExit_ = 0;
		}
	}

	// if not interior is created for the suitable storey object it has to be created, self checks is required
	geoCreator->store2DStoreyData(internalDataManager_.get(), kernel);

	if (settingsCollection.make00())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD00(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD00, timeLoD00_);
	}
	if (settingsCollection.make02())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD02(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD02, timeLoD02_);
	}
	if (settingsCollection.make03())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD03(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD03, timeLoD03_);
	}
	if (settingsCollection.make04())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD04(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD04, timeLoD04_);
	}
	if (settingsCollection.make10())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD10(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD10, timeLoD10_);
	}
	if (settingsCollection.make12())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD12(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD12, timeLoD12_);
	}
	if (settingsCollection.make13())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD13(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD13, timeLoD13_);
	}
	if (settingsCollection.make22())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD22(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD22, timeLoD22_);
	}
	if (settingsCollection.makeb0())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoDb0(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoDb0, timeLoDb0_);
	}
	if (settingsCollection.makec1())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoDc1(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoDc1, timeLoDc1_);
	}
	if (settingsCollection.makec2())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoDc2(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoDc2, timeLoDc2_);
	}
	if (settingsCollection.maked1())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoDd1(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoDd1, timeLoDd1_);
	}
	if (settingsCollection.maked2())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeLoDd2(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoDd2, timeLoDd2_);
	}
	if (settingsCollection.make32() || settingsCollection.makee1())
	{
		processExternalLoD([&]() { //TODO: split e1 and 32
			return std::vector<CJT::GeoObject>{geoCreator->makeLoD32(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD32, timeLoD32_);
	}
	if (settingsCollection.makeV())
	{
		processExternalLoD([&]() {
			return std::vector<CJT::GeoObject>{geoCreator->makeV(internalDataManager_.get(), kernel, 1)};
			}, cityOuterShellObject, ErrorID::failedLoD50, timeV_);
	}
	std::cout << "\n";
	return;
}

void IOManager::processExternalLoD(
	const std::function<std::vector<CJT::GeoObject>()>& lodCreationFunc,
	CJT::CityObject& cityOuterShellObject,
	ErrorID errorID,
	long long& timeRecord
)
{
	auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
	try
	{
		std::vector<CJT::GeoObject> geoObjects = lodCreationFunc();
		for (size_t i = 0; i < geoObjects.size(); i++) { cityOuterShellObject.addGeoObject(geoObjects[i]); }
	}
	catch (const std::exception&)
	{
		ErrorCollection::getInstance().addError(errorID);
		succesfullExit_ = 0;
	}
	timeRecord = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
	return;
}


void IOManager::processInteriorLod(CJGeoCreator* geoCreator, std::shared_ptr<CJT::CityCollection> collection, CJT::CityObject* cityInnerShellObject, CJT::Kernel* kernel)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingInterior) << std::endl;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// get storey semantic objects
	std::vector<std::shared_ptr<CJT::CityObject>> storeyObjects = geoCreator->makeStoreyObjects(internalDataManager_.get());
	std::vector<std::shared_ptr<CJT::CityObject>> roomObjects = geoCreator->makeRoomObjects(internalDataManager_.get(), storeyObjects);

	// storeys
	if (settingsCollection.make02())
	{
		geoCreator->make2DStoreys(internalDataManager_.get(), kernel, storeyObjects, 1, false);
	}
	if (settingsCollection.make03())
	{
		geoCreator->make2DStoreys(internalDataManager_.get(), kernel, storeyObjects, 1, true);
	}
	if (settingsCollection.make02() || settingsCollection.make12() || settingsCollection.make22())
	{
		geoCreator->makeSimpleLodRooms(internalDataManager_.get(), kernel, roomObjects, 1);
	}

	if (settingsCollection.make32())
	{
		geoCreator->makeComplexLoDRooms(internalDataManager_.get(), kernel, roomObjects, 1);
	}

	// rooms
	if (settingsCollection.makeV())
	{
		geoCreator->makeVRooms(internalDataManager_.get(), kernel, storeyObjects, roomObjects, 1);
	}

	for (size_t i = 0; i < storeyObjects.size(); i++)
	{
		storeyObjects[i]->addParent(cityInnerShellObject);
		collection->addCityObject(*storeyObjects[i].get());
	}

	for (size_t i = 0; i < roomObjects.size(); i++)
	{
		collection->addCityObject(*roomObjects[i].get());
	}
	std::cout << "\n";
	return;
}

void IOManager::processSitelod(CJGeoCreator* geoCreator, std::shared_ptr<CJT::CityCollection> collection, CJT::CityObject* cityBuildingObject, CJT::Kernel* kernel)
{
	auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
	std::vector<CJT::CityObject> siteObjects = geoCreator->makeSite(internalDataManager_.get(), kernel, 1);
	for (size_t i = 0; i < siteObjects.size(); i++)
	{
		CJT::CityObject currentSiteObject = siteObjects[i];
		currentSiteObject.addParent(cityBuildingObject);
		collection->addCityObject(currentSiteObject);
	}
	return;
}


bool IOManager::init(const std::vector<std::string>& inputPathList)
{
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << std::endl;
	std::cout << "		IFC_BuildingEnvExtractor " << buildVersion << std::endl;
	std::cout << "    Experimental building shell extractor/approximation\n" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << std::endl;
	std::cout << std::endl;

	std::string inputPath = "";
	if (inputPathList.size() == 0) { inputPath = getTargetPath(); }
	else if (inputPathList.size() > 1) { return false; } // too many args
	else { inputPath = inputPathList[0]; }

	try { getJSONValues(inputPath); }
	catch (const std::string& exceptionString)
	{
		throw exceptionString;
	}
	std::cout << std::endl;
	printSummary();
	
	internalDataManager_ = std::make_unique<DataManager>(SettingsCollection::getInstance().getIfcPathList());
	DataManager* internalManagerPtr = internalDataManager_.get();
	if (!internalManagerPtr->isPopulated()) { return false; }
	if (!internalManagerPtr->hasSetUnits()) { return false; }

	return true;
}

bool IOManager::run()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// internalize the helper data
	try { internalizeGeo(); }
	catch (const std::string& exceptionString) { throw exceptionString; }

	// create the cjt objects
	std::shared_ptr<CJT::CityCollection> collection = std::make_shared<CJT::CityCollection>();
	CJT::CityObject cityBuildingObject; //the overarching city object
	CJT::CityObject cityOuterShellObject; //container of outer shell geo objects
	CJT::CityObject cityInnerShellObject; //container of storey objects

	setDefaultSemantic(cityBuildingObject, cityOuterShellObject, cityInnerShellObject);
	setMetaData(collection);

	// make the geometrycreator and voxelgrid
	auto voxelTime = std::chrono::high_resolution_clock::now();
	CJGeoCreator geoCreator(internalDataManager_.get(), settingsCollection.voxelSize());
	timeVoxel_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - voxelTime).count();

	try
	{
		geoCreator.initializeBasic(internalDataManager_.get());
	}
	catch (const std::exception&)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorFailedInit);
		return false;
	}

	// create the actual geometry
	CJT::Kernel kernel = CJT::Kernel(collection);
	if (settingsCollection.makeInterior())
	{
		processInteriorLod(&geoCreator, collection, &cityInnerShellObject, &kernel);
	}
	if (settingsCollection.makeExterior())
	{
		processExternalLoD(&geoCreator, cityOuterShellObject, &kernel);
	}
	if (settingsCollection.makeSite())
	{
		processSitelod(&geoCreator, collection, &cityBuildingObject, &kernel);
	}

	setComputedSemantic(&geoCreator, cityBuildingObject, cityOuterShellObject, cityInnerShellObject);

	collection->addCityObject(cityBuildingObject);
	collection->addCityObject(cityOuterShellObject);
	collection->addCityObject(cityInnerShellObject);
	collection->cullDuplicatedVerices();
	cityCollection_ = collection;

	printErrors();

	return succesfullExit_;
}

bool IOManager::write(bool reportOnly)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (settingsCollection.getOutputJSONPath() == "") {
		//TODO: add cout for no report file made
		return true; 
	} // no output path set yet, cannot write to unknown location

	if (!reportOnly)
	{
		cityCollection_->dumpJson(settingsCollection.getOutputJSONPath());
	}
	if (!settingsCollection.writeReport()) { return true; }
	nlohmann::json report;
	report["Input settings"] = settingsToJSON();

	nlohmann::json timeReport;
	addTimeToJSON(&timeReport, "Internalizing", timeInternalizing_);
	addTimeToJSON(&timeReport, "Voxel creation", timeVoxel_);
	addTimeToJSON(&timeReport, "LoD0.0 generation", timeLoD00_);
	addTimeToJSON(&timeReport, "LoD0.2 generation", timeLoD02_);
	addTimeToJSON(&timeReport, "LoD0.3 generation", timeLoD03_);
	addTimeToJSON(&timeReport, "LoD0.4 generation", timeLoD04_);
	addTimeToJSON(&timeReport, "LoD1.0 generation", timeLoD10_);
	addTimeToJSON(&timeReport, "LoD1.2 generation", timeLoD12_);
	addTimeToJSON(&timeReport, "LoD1.3 generation", timeLoD13_);
	addTimeToJSON(&timeReport, "LoD2.2 generation", timeLoD22_);
	addTimeToJSON(&timeReport, "LoDe.1/3.2 generation", timeLoD32_);
	addTimeToJSON(&timeReport, "LoD5.0 (V) generation", timeV_);
	addTimeToJSON(&timeReport, "LoDb.0 generation", timeLoDb0_);
	addTimeToJSON(&timeReport, "LoDc.1 generation", timeLoDc1_);
	addTimeToJSON(&timeReport, "LoDc.2 generation", timeLoDc2_);
	addTimeToJSON(&timeReport, "LoDd.1 generation", timeLoDd1_);
	addTimeToJSON(&timeReport, "LoDd.2 generation", timeLoDd2_);
	addTimeToJSON(&timeReport, "Total Processing",
		timeInternalizing_ +
		timeVoxel_ +
		timeLoD00_ +
		timeLoD02_ +
		timeLoD03_ +
		timeLoD04_ +
		timeLoD10_ +
		timeLoD12_ +
		timeLoD13_ +
		timeLoD22_ +
		timeLoDb0_ +
		timeLoDc1_ +
		timeLoDc2_ +
		timeLoDd1_ +
		timeLoDd2_ +
		timeLoD32_ +
		timeV_
	);

	report["Duration"] = timeReport;
	report["Errors"] = ErrorCollection::getInstance().toJson();

	std::ofstream reportFile(settingsCollection.getOutputReportPath());
	reportFile << report;
	reportFile.close();
	return true;
}
