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


bool IOManager::getTargetPathList()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// preload communcation strings
	std::string stringJSONRequest = CommunicationStringEnum::getString(CommunicationStringID::infoJsonRequest);
	std::string stringNoFilePath = CommunicationStringEnum::getString(CommunicationStringID::infoNoFilePath);
	std::string stringNoValFilePath = CommunicationStringEnum::getString(CommunicationStringID::infoNoFilePath);

	std::cout << stringJSONRequest << std::endl;

	while (true)
	{
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		if (singlepath.size() == 0 && settingsCollection.getInputPathList().size() == 0)
		{
			std::cout << stringNoFilePath << std::endl;
			std::cout << stringJSONRequest << std::endl;
			continue;
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

		if (hasExtension(singlepath, "json"))
		{
			if (settingsCollection.getInputPathList().size() > 1)
			{
				return false;
			}
		}
		settingsCollection.addToInputPathList(singlepath);
		break;
	}
	if (settingsCollection.getInputPathList().size() > 0) { return true; }
	return false;
}


std::string IOManager::getFileName(const std::string& stringPath)
{
	std::vector<std::string> segments;
	boost::split(segments, stringPath, boost::is_any_of("/, \\"));
	std::string filePath = segments[segments.size() - 1];
	return filePath.substr(0, filePath.size() - 4);
}


bool IOManager::getJSONValues()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::ifstream f(settingsCollection.getInputPathList()[0]);
	nlohmann::json json = nlohmann::json::parse(f);

	// in and output related settings
	settingsCollection.clearInputPathList();
	std::string outputReportOName = JsonObjectInEnum::getString(JsonObjectInID::outputReport);
	if (json.contains(outputReportOName))
	{
		if (json[outputReportOName].type() != nlohmann::json::value_t::number_integer && json[outputReportOName].type() != nlohmann::json::value_t::number_unsigned)
		{
			
			throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONReportPath));
		}

		if (json[outputReportOName] == 0) { settingsCollection.setWriteReport(false); }
	}

	std::string threadMaxOName = JsonObjectInEnum::getString(JsonObjectInID::maxThread);
	if (json.contains(threadMaxOName))
	{
		if (json[threadMaxOName].type() != nlohmann::json::value_t::number_integer && json[threadMaxOName].type() != nlohmann::json::value_t::number_unsigned)
		{
			throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONThreadNum));
		}
		if (json[threadMaxOName] > 0) { settingsCollection.setThreadcount(static_cast<int>(json[threadMaxOName])); }
	}

	std::string filePathsOName = JsonObjectInEnum::getString(JsonObjectInID::filePaths);
	if (!json.contains(filePathsOName))
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONFilePath));
	}
	nlohmann::json filePaths = json[filePathsOName];

	std::string inputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathsInput);
	if (!filePaths.contains(inputOName))
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONInputPath));
	}

	nlohmann::json inputPaths = filePaths[inputOName];
	if (inputPaths.type() != nlohmann::json::value_t::array )
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONInvalInputPathFormat));
	}

	for (size_t i = 0; i < inputPaths.size(); i++) {
		if (inputPaths[i].type() != nlohmann::json::value_t::string)
		{
			throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONNoValInputPath));
		}

		std::string inputPath = inputPaths[i];

		if (hasExtension(inputPath, "ifc") && isValidPath(inputPath))
		{
			settingsCollection.addToInputPathList(inputPaths[i]);
		}
		else
		{
			throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONInvalInputPath) + inputPath);
		}	
	}

	std::string outputOName = JsonObjectInEnum::getString(JsonObjectInID::filePatsOutput);
	if (!filePaths.contains(outputOName))
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONOutputPath));
	}

	if (filePaths[outputOName].type() != nlohmann::json::value_t::string)
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONOutputPath));
	}

	settingsCollection.setOutputPath(filePaths[outputOName]);

	if (!hasExtension(settingsCollection.getOutputPath(), "json"))
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONInvalOuputFormat));
	}

	boost::filesystem::path outputFolderPath = boost::filesystem::path(std::string(settingsCollection.getOutputPath())).parent_path();

	if (!boost::filesystem::exists(outputFolderPath))
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONInvalOutputFolder));
	}

	//JSOn related output and processing settings
	std::string lodOutputOName = JsonObjectInEnum::getString(JsonObjectInID::lodOutput);
	std::string jsonOname = JsonObjectInEnum::getString(JsonObjectInID::JSON);
	
	nlohmann::json outputDataJson = {};
	if (json.contains(jsonOname)) { outputDataJson = json[jsonOname];}

	if (json.contains(lodOutputOName))
	{
		nlohmann::json lodList = json[lodOutputOName];
		settingsCollection.setMake00(false);
		settingsCollection.setMake02(false);
		settingsCollection.setMake03(false);
		settingsCollection.setMake10(false);
		settingsCollection.setMake12(false);
		settingsCollection.setMake13(false);
		settingsCollection.setMake22(false);
		settingsCollection.setMake32(false);
		settingsCollection.setMakeV(false);

		std::string generateInteriorOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenInterior);
		std::string generatefootprOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenFootPrint);
		std::string generateRoofOlineOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenRoofOutline);
		std::string footprintElevOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintElev);
		std::string hSectionOffsetOName = JsonObjectInEnum::getString(JsonObjectInID::JSONSecOffset);

		for (size_t i = 0; i < lodList.size(); i++) // check if interior generation is required
		{
			std::unordered_set<double> LoDWInterior = settingsCollection.getLoDWInterior();
			if (LoDWInterior.find(lodList[i]) == LoDWInterior.end()) { continue; }

			if (outputDataJson.contains(generateInteriorOName))
			{
				settingsCollection.setMakeInterior((int)outputDataJson[generateInteriorOName]);
			}
			break;
		}

		if (outputDataJson.contains(footprintElevOName))
		{
			settingsCollection.setFootprintElevation(outputDataJson[footprintElevOName]);
		}

		if (outputDataJson.contains(hSectionOffsetOName))
		{
			settingsCollection.setHorizontalSectionOffset(outputDataJson[hSectionOffsetOName]);
		}
		for (size_t i = 0; i < lodList.size(); i++)
		{
			if (lodList[i] == 0.0) { settingsCollection.setMake00(true); }
			else if (lodList[i] == 0.2) 
			{ 
				settingsCollection.setMake02(true);

				if (outputDataJson.contains(generatefootprOName))
				{
					settingsCollection.setMakeFootPrint((int)outputDataJson[generatefootprOName]);
				}
				if (outputDataJson.contains(generateRoofOlineOName))
				{
					settingsCollection.setMakeRoofPrint((int)outputDataJson[generateRoofOlineOName]);
					if (settingsCollection.makeRoofPrint()) { settingsCollection.setMakeOutlines(true); }
				}
			}
			else if (lodList[i] == 0.3)
			{
				settingsCollection.setMake03(true);
			}
			else if (lodList[i] == 1.0) 
			{ 
				settingsCollection.setMake10(true);
			}
			else if (lodList[i] == 1.2) 
			{ 
				settingsCollection.setMake12(true);
				settingsCollection.setMakeOutlines(true);
			}
			else if (lodList[i] == 1.3) 
			{ 
				settingsCollection.setMake13(true);
				settingsCollection.setMakeOutlines(true);
			}
			else if (lodList[i] == 2.2) 
			{ 
				settingsCollection.setMake22(true);
				settingsCollection.setMakeOutlines(true);
			}
			else if (lodList[i] == 3.2) 
			{ 
				settingsCollection.setMake32(true);
			}
			else if (lodList[i] == 5.0) 
			{ 
				settingsCollection.setMakeV(true);
			}
		}
	}

	std::string georeferenceOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGeoreference);
	if (outputDataJson.contains(georeferenceOName))
	{
		if (outputDataJson[georeferenceOName] == false || outputDataJson[georeferenceOName] == 0)
		{
			settingsCollection.setGeoReference(false);
		}		
	}
	 
	// Voxel related output and processing settings
	std::string voxelOName = JsonObjectInEnum::getString(JsonObjectInID::voxel);
	std::string voxelSizOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSize);
	std::string voxelSummarizeOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSummarize);
	std::string voxelIntersectionOName = JsonObjectInEnum::getString(JsonObjectInID::voxelIntersection);
	if (json.contains(voxelOName))
	{
		nlohmann::json voxelData = json[voxelOName];

		if (voxelData.contains(voxelSizOName))			
		{
			settingsCollection.setVoxelSize(voxelData[voxelSizOName]);
		}
		if (voxelData.contains(voxelSummarizeOName))
		{ 
			if (voxelData[voxelSummarizeOName] == 1) { settingsCollection.setSummaryVoxels(true); }
		}
		if (voxelData.contains(voxelIntersectionOName)) 
		{
			if (voxelData[voxelIntersectionOName] == 2) { settingsCollection.setIntersectionLogic(2); }
			if (voxelData[voxelIntersectionOName] == 3) { settingsCollection.setIntersectionLogic(3); }
		}
	}

	//IFC related processing settings
	std::string ifcOName = JsonObjectInEnum::getString(JsonObjectInID::IFC);
	std::string rotationOName = JsonObjectInEnum::getString(JsonObjectInID::IFCRotation);
	std::string defaultDivOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDefaultDiv);
	std::string ignoreProxyOName = JsonObjectInEnum::getString(JsonObjectInID::IFCIgnoreProxy);
	std::string divObjectsOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDivObject);

	nlohmann::json ifcInputJson = {};
	if (json.contains(ifcOName)) { ifcInputJson = json[ifcOName]; }

	if (ifcInputJson.contains(rotationOName))
	{
		nlohmann::json rotationData = ifcInputJson[rotationOName];
		if (rotationData.type() != nlohmann::json::value_t::boolean)
		{
			settingsCollection.setAutoRotateGrid(false);
			settingsCollection.setDesiredRotation(static_cast<double>(rotationData)* (M_PI / 180));
		}
	}

	if (ifcInputJson.contains(defaultDivOName))
	{
		if (ifcInputJson[defaultDivOName] == 0 || ifcInputJson[defaultDivOName] == false)
		{
			settingsCollection.setUseDefaultDiv(false);
		}
	}

	if (ifcInputJson.contains(ignoreProxyOName))
	{
		if (ifcInputJson[ignoreProxyOName] == 0 || ifcInputJson[ignoreProxyOName] == false)
		{
			settingsCollection.setUseProxy(true);
		}
	}

	if (ifcInputJson.contains(divObjectsOName))
	{
		std::vector<std::string> stringDivList = ifcInputJson[divObjectsOName];

		for (size_t i = 0; i < stringDivList.size(); i++)
		{
			std::string potentialType = stringDivList[i];
			std::transform(potentialType.begin(), potentialType.end(), potentialType.begin(), ::toupper);

			if (DevObjectsOptions_.find(potentialType) == DevObjectsOptions_.end())
			{
				continue;
			}

			if (addDivObjects_.find(potentialType) != addDivObjects_.end())
			{
				continue;
			}
			addDivObjects_.insert(potentialType);
			settingsCollection.addToCustomDivList(potentialType);
		}
	}

	if (!settingsCollection.getCustomDivList().size() && !settingsCollection.useDefaultDiv())
	{
		throw std::string(CommunicationStringEnum::getString(CommunicationStringID::errorJSONNoDivObjects));
	}

	// set generated settings
	if ( settingsCollection.make00() || settingsCollection.make10())
	{
		if (!settingsCollection.make02() && !settingsCollection.make12() &&
			!settingsCollection.make13() && !settingsCollection.make22() &&
			!settingsCollection.make32() && !settingsCollection.summaryVoxels())
		{
			settingsCollection.setRequireVoxels(false);
		}
	}

	if (!settingsCollection.make32() && !settingsCollection.makeV() && !settingsCollection.summaryVoxels())
	{
		if (!settingsCollection.makeFootPrint() && !settingsCollection.makeInterior())
		{
			settingsCollection.setRequireFullVoxels(false);
		}
	}

	// set ifcGeomsettings
	IfcGeom::IteratorSettings iteratorSettings;
	IfcGeom::IteratorSettings simpleIteratorSettings;
	iteratorSettings.set(simpleIteratorSettings.DISABLE_OPENING_SUBTRACTIONS, true);

	settingsCollection.setIterator(iteratorSettings);
	settingsCollection.setSimpleIterator(simpleIteratorSettings);

	return true;
}

bool IOManager::hasExtension(const std::vector<std::string>& stringList, const std::string& ext)
{
	for (size_t i = 0; i < stringList.size(); i++)
	{
		if (!hasExtension(stringList[i], ext)) { return false; }
	}
	return true;
}

bool IOManager::hasExtension(const std::string& string, const std::string& ext)
{
	std::string substring = boost::to_lower_copy<std::string>(string.substr(string.find_last_of(".") + 1));
	if (substring == ext) { return true; }
	return false;
}


bool IOManager::isValidPath(const std::vector<std::string>& path)
{
	for (size_t i = 0; i < path.size(); i++)
	{
		if (!isValidPath(path[i])) { return false; }
	}
	return true;
}


bool IOManager::isValidPath(const std::string& path)
{
	struct stat info;
	if (stat(path.c_str(), &info) != 0) { return false; }
	return true;
}


void addTimeToJSON(nlohmann::json* j, const std::string& valueName, const std::chrono::steady_clock::time_point& startTime, const std::chrono::steady_clock::time_point& endTime)
{
	long long duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5) { (*j)[valueName + UnitStringEnum::getString(UnitStringID::milliseconds)] = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count(); }
	else { (*j)[valueName + UnitStringEnum::getString(UnitStringID::seconds)] = duration; }
}


void addTimeToJSON(nlohmann::json* j, const std::string& valueName, double duration)
{
	std::string timeUnitString = "Unit";
	std::string timeDurationString = "Duration";
	nlohmann::json timeSet;
	if (duration == 0)  { return; }
	else if (duration < 5000) 
	{
		timeSet[timeDurationString] = duration;
		timeSet[timeUnitString] = UnitStringEnum::getString(UnitStringID::milliseconds);
	}
	else 
	{
		timeSet[timeDurationString] = duration/1000;
		timeSet[timeUnitString] = UnitStringEnum::getString(UnitStringID::seconds);
	}
	(*j)[valueName] = timeSet;
	return;
}


void IOManager::printSummary()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::cout << "=============================================================" << std::endl;
	std::cout << "[INFO] Used settings: " << std::endl;

	std::cout << "- Input File(s):" << std::endl;
	for (const std::string& inputPath : settingsCollection.getInputPathList()) { std::cout << "    " << inputPath << std::endl; }
	std::cout << "- Output File:" << std::endl;
	std::cout << "    " << settingsCollection.getOutputPath() << std::endl;
	std::cout << "- Create Report:" << std::endl;
	if (settingsCollection.writeReport()) { std::cout << "    yes" << std::endl; }
	else { std::cout << "    no" << std::endl; }
	std::cout << "- LoD export enabled:" << std::endl;
	std::cout << "    " << getLoDEnabled() << std::endl;
	std::cout << "- Space dividing objects: " << std::endl;
	if (settingsCollection.useDefaultDiv())
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	}
	if (settingsCollection.useProxy())
	{
		std::cout << "    IFCBUILDINGELEMENTPROXY" << std::endl;
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	std::cout << "- Voxel size:" << std::endl;
	std::cout << "    " << settingsCollection.voxelSize() << std::endl;
	std::cout << "- Voxel logic:" << std::endl;
	if (settingsCollection.intersectionLogic() == 0) { std::cout << "    point" << std::endl; }
	if (settingsCollection.intersectionLogic() == 1) { std::cout << "    line" << std::endl; }
	if (settingsCollection.intersectionLogic() == 2) { std::cout << "    plane" << std::endl; }
	if (settingsCollection.intersectionLogic() == 3) { std::cout << "    solid" << std::endl; }



	if (settingsCollection.make02())
	{
		std::cout << "- Create footprint:" << std::endl;
		if (settingsCollection.makeFootPrint()) { std::cout << "    Yes" << std::endl; }
		else { std::cout << "    No" << std::endl; }

		std::cout << "- Store Lod0.2 roof outline:" << std::endl;
		if (settingsCollection.makeRoofPrint()) { std::cout << "    Yes" << std::endl; }
		else { std::cout << "    No" << std::endl; }
	}

	std::cout << "- Footprint Elevation:" << std::endl;
	std::cout << "    " << settingsCollection.footprintElevation() << std::endl;

	std::cout << "- Max thread count" << std::endl;
	std::cout << "    " <<  settingsCollection.threadcount() << std::endl;

	std::cout << "=============================================================" << std::endl;
}


std::string IOManager::getLoDEnabled()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::string summaryString = "";

	if (settingsCollection.make00()) { summaryString += ", 0.0"; }
	if (settingsCollection.make02()) { summaryString += ", 0.2"; }
	if (settingsCollection.make03()) { summaryString += ", 0.3"; }
	if (settingsCollection.make10()) { summaryString += ", 1.0"; }
	if (settingsCollection.make12()) { summaryString += ", 1.2"; }
	if (settingsCollection.make13()) { summaryString += ", 1.3"; }
	if (settingsCollection.make22()) { summaryString += ", 2.2"; }
	if (settingsCollection.make32()) { summaryString += ", 3.2"; }
	if (settingsCollection.makeV()) { summaryString += ", 5.0 (V)"; }

	summaryString.erase(0, 2);

	return summaryString;
}


nlohmann::json IOManager::settingsToJSON()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	nlohmann::json settingsJSON;

	settingsJSON["Input IFC file"] = settingsCollection.getInputPathList();
	settingsJSON["Output CityJSON file"] = settingsCollection.getOutputPath();
	settingsJSON["Create report"] = "true";

	std::vector<std::string> DivList;

	if (settingsCollection.useDefaultDiv())
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { DivList.emplace_back(boost::to_upper_copy(*it)); }
	}
	if (settingsCollection.useProxy())
	{
		DivList.emplace_back("IFCBUILDINGELEMENTPROXY");
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { DivList.emplace_back(boost::to_upper_copy(*it)); }
	settingsJSON["Space bounding objects"] = DivList;
	settingsJSON["Voxel size"] = settingsCollection.voxelSize();

	if (settingsCollection.make02())
	{ 
		settingsJSON["Footprint elevation"] = settingsCollection.footprintElevation();
		settingsJSON["Generate footprint"] = settingsCollection.makeFootPrint();
		settingsJSON["Generate roof outline"] = settingsCollection.makeRoofPrint();
	}
	
	std::vector<std::string> LoDList;
	if (settingsCollection.make00()) { LoDList.emplace_back("0.0"); }
	if (settingsCollection.make02()) { LoDList.emplace_back("0.2"); }
	if (settingsCollection.make10()) { LoDList.emplace_back("1.0"); }
	if (settingsCollection.make12()) { LoDList.emplace_back("1.2"); }
	if (settingsCollection.make13()) { LoDList.emplace_back("1.3"); }
	if (settingsCollection.make22()) { LoDList.emplace_back("2.2"); }
	if (settingsCollection.make32()) { LoDList.emplace_back("3.2"); }
	if (settingsCollection.makeV()) { LoDList.emplace_back("3.2"); }

	settingsJSON["Desired LoD output"] = LoDList;

	return settingsJSON;
}


bool IOManager::init(const std::vector<std::string>& inputPathList, bool silent)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	timeTotal = 0;
	bool isSilent_ = silent;

	// find if input has IFC format
	bool isIFC = true;

	if (!isSilent_)
	{
		std::wcout << "============================================================= \n" << std::endl;
		std::cout << "		IFC_BuildingEnvExtractor " << buildVersion << std::endl;
		std::cout << "    Experimental building shell extractor/approximation\n" << std::endl;
		std::wcout << "=============================================================" << std::endl;
		std::cout << std::endl;
	}

	bool hasDirectInterface = false;
	bool pathIsValid = isValidPath(inputPathList);

	if (inputPathList.size() == 0)
	{
		getTargetPathList();
		hasDirectInterface = true;
	}
	else if (!isValidPath(inputPathList))
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::errorNoValFilePaths) << std::endl;
		return false;
	}
	else if (hasExtension(inputPathList, "json") && isValidPath(inputPathList)) {
		settingsCollection.setInputPathList(inputPathList);
	}

	if (inputPathList.size() > 1) { return false; }

	try { getJSONValues(); }
	catch (const std::string& exceptionString)
	{
		throw exceptionString;
	}

	if (!isSilent_)
	{
		std::cout << std::endl;
		printSummary();
	}
	auto internalizingTime = std::chrono::high_resolution_clock::now(); // Time Collection Starts
	
	internalHelper_ = std::make_unique<helper>(settingsCollection.getInputPathList());
	helper* internalHelperPtr = internalHelper_.get();
	if (!internalHelperPtr->isPopulated()) { return 0; }
	if (!internalHelperPtr->hasSetUnits()) { return 0; }

	return true;
}

bool IOManager::run()
{
	// Time Collection Starts
	auto internalizingTime = std::chrono::high_resolution_clock::now();
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	
	int succesfullExit = 1;

	// internalize the helper data
	try
	{
		internalHelper_->internalizeGeo();
	}
	catch (const std::string& exceptionString)
	{
		throw exceptionString;
	}

	internalHelper_->indexGeo();
	timeInternalizing_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - internalizingTime).count();
	
	// create the cjt objects
	std::shared_ptr<CJT::CityCollection> collection = std::make_shared<CJT::CityCollection>();
	gp_Trsf geoRefRotation;
	CJT::ObjectTransformation transformation(0.001);
	CJT::metaDataObject metaData;
	metaData.setTitle(CJObjectEnum::getString(CJObjectID::metaDataTitle));
	if (settingsCollection.geoReference())
	{
		internalHelper_.get()->getProjectionData(&transformation, &metaData, &geoRefRotation);
	}
	transformation.setScale(transformation.getScale()[0]);
	collection->setTransformation(transformation);
	collection->setVersion(CJObjectEnum::getString(CJObjectID::v11));

	// Set up objects and their relationships
	CJT::CityObject cityBuildingObject;
	CJT::CityObject cityShellObject;
	CJT::CityObject cityInnerShellObject;

	std::string BuildingName = internalHelper_.get()->getBuildingName();
	if (BuildingName == "") { BuildingName = internalHelper_.get()->getProjectName(); }

	cityBuildingObject.setName(BuildingName);
	cityBuildingObject.setType(CJT::Building_Type::Building);

	cityShellObject.setName(CJObjectEnum::getString(CJObjectID::outerShell));
	cityShellObject.setType(CJT::Building_Type::BuildingPart);

	cityInnerShellObject.setName(CJObjectEnum::getString(CJObjectID::innerShell));
	cityInnerShellObject.setType(CJT::Building_Type::BuildingPart);

	cityBuildingObject.addChild(&cityShellObject);
	cityBuildingObject.addChild(&cityInnerShellObject);

	CJT::Kernel kernel = CJT::Kernel(collection);

	std::map<std::string, std::string> buildingAttributes = internalHelper_.get()->getBuildingInformation();
	for (std::map<std::string, std::string>::iterator iter = buildingAttributes.begin(); iter != buildingAttributes.end(); ++iter) { cityBuildingObject.addAttribute(iter->first, iter->second); }

	// make the geometrycreator and voxelgrid
	auto voxelTime = std::chrono::high_resolution_clock::now();
	CJGeoCreator geoCreator(internalHelper_.get(), settingsCollection.voxelSize());
	timeVoxel_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - voxelTime).count();

	if (settingsCollection.makeOutlines())
	{
		try
		{
			geoCreator.initializeBasic(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(errorID::failedInit);
			return false;
		}
	}

	if (settingsCollection.make02() && settingsCollection.makeFootPrint())
	{
		try
		{
			geoCreator.makeFootprint(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(errorID::failedFootprint);
			succesfullExit = 0;
		}
	}

	if (settingsCollection.makeRoofPrint())
	{
		geoCreator.useroofprint0();
	}

	geoCreator.setRefRotation(geoRefRotation);

	if (settingsCollection.make00())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			CJT::GeoObject geo00 = geoCreator.makeLoD00(internalHelper_.get(), &kernel, 1);
			cityShellObject.addGeoObject(geo00);
			timeLoD00_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD00);
			succesfullExit = 0;
		}

	}
	if (settingsCollection.make02())
	{
		auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
		try
		{
			std::vector<CJT::GeoObject> geo02 = geoCreator.makeLoD02(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo02.size(); i++) { cityShellObject.addGeoObject(geo02[i]); }
			
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD02);
			succesfullExit = 0;
		}
		timeLoD02_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();

	}
	if (settingsCollection.make03()) //TODO: make binding
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo03 = geoCreator.makeLoD03(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo03.size(); i++) { cityShellObject.addGeoObject(geo03[i]); }
			timeLoD10_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(errorID::failedLoD10);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make10())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			CJT::GeoObject geo10 = geoCreator.makeLoD10(internalHelper_.get(), &kernel, 1);
			cityShellObject.addGeoObject(geo10);
			timeLoD10_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD10);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make12())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo12 = geoCreator.makeLoD12(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo12.size(); i++) { cityShellObject.addGeoObject(geo12[i]); }
			timeLoD12_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&)
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD12);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make13())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo13 = geoCreator.makeLoD13(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo13.size(); i++) { cityShellObject.addGeoObject(geo13[i]); }
			timeLoD13_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD13);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make22())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo22 = geoCreator.makeLoD22(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo22.size(); i++) { cityShellObject.addGeoObject(geo22[i]); }
			timeLoD22_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD22);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make32())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo32 = geoCreator.makeLoD32(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo32.size(); i++) { cityShellObject.addGeoObject(geo32[i]); }
			timeLoD32_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(errorID::failedLoD32);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.makeV())
	{
		auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
		std::vector<CJT::GeoObject> geoV = geoCreator.makeV(internalHelper_.get(), &kernel, 1);
		for (size_t i = 0; i < geoV.size(); i++) { cityShellObject.addGeoObject(geoV[i]); }
		timeV_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();	
	}

	if (settingsCollection.makeInterior())
	{
		// get storey semantic objects
		std::vector<std::shared_ptr<CJT::CityObject>> storeyObjects = geoCreator.makeStoreyObjects(internalHelper_.get());
		std::vector<std::shared_ptr<CJT::CityObject>> roomObjects = geoCreator.makeRoomObjects(internalHelper_.get(), storeyObjects);

		// storeys
		if (settingsCollection.make02())
		{
			try
			{
				geoCreator.makeFloorSectionCollection(internalHelper_.get());
			}
			catch (const std::exception&)
			{
				ErrorCollection::getInstance().addError(errorID::failedStorey);
				succesfullExit = 0;
			}
			geoCreator.makeLoD02Storeys(internalHelper_.get(), &kernel, storeyObjects, 1);
		}

		if (settingsCollection.make02() || settingsCollection.make12() ||settingsCollection.make22())
		{
			geoCreator.makeSimpleLodRooms(internalHelper_.get(), &kernel, roomObjects, 1);
		}

		if (settingsCollection.make32())
		{
			geoCreator.makeComplexLoDRooms(internalHelper_.get(), &kernel, roomObjects, 1);
		}

		// rooms
		if (settingsCollection.makeV())
		{
			geoCreator.makeVRooms(internalHelper_.get(), &kernel, storeyObjects, roomObjects, 1);		
		}

		for (size_t i = 0; i < storeyObjects.size(); i++)
		{
			storeyObjects[i]->addParent(&cityInnerShellObject);
			collection->addCityObject(*storeyObjects[i].get());
		}

		for (size_t i = 0; i < roomObjects.size(); i++)
		{
			collection->addCityObject(*roomObjects[i].get());
		}
	}

	if (false) // store the site
	{
		auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
		std::vector<CJT::CityObject> siteObjects = geoCreator.makeSite(internalHelper_.get(), &kernel, 1);
		for (size_t i = 0; i < siteObjects.size(); i++)
		{
			CJT::CityObject currentSiteObject = siteObjects[i];
			currentSiteObject.addParent(&cityBuildingObject);
			collection->addCityObject(currentSiteObject);
		}
	}

	// compute the extends
	gp_Pnt lll = internalHelper_.get()->getLllPoint();
	gp_Pnt urr = internalHelper_.get()->getUrrPoint();

	gp_Trsf originRotation;
	originRotation.SetRotation(gp_Quaternion(gp_Vec(0, 0, 1), -settingsCollection.gridRotation()));

	gp_Trsf originTranslation = internalHelper_->getObjectTranslation().Inverted();

	TopoDS_Shape bboxGeo = helperFunctions::createBBOXOCCT(lll, urr).Moved(originRotation).Moved(originTranslation);

	gp_Trsf trs;
	trs.SetRotation(geoCreator.getRefRotation().GetRotation());
	trs.SetTranslationPart(
		gp_Vec(transformation.getTranslation()[0], transformation.getTranslation()[1], transformation.getTranslation()[2])
		 - originTranslation.TranslationPart()
	);

	bboxGeo = bboxGeo.Moved(trs);
	bg::model::box <BoostPoint3D> extents = helperFunctions::createBBox(bboxGeo);

	metaData.setExtend(
		CJT::CJTPoint(extents.min_corner().get<0>(), extents.min_corner().get<1>(), extents.min_corner().get<2>()),
		CJT::CJTPoint(extents.max_corner().get<0>(), extents.max_corner().get<1>(), extents.max_corner().get<2>())
	);

	collection->setMetaData(metaData);

	cityShellObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "footprint elevation", settingsCollection.footprintElevation());
	cityShellObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "buildingHeight", internalHelper_.get()->getUrrPoint().Z() - settingsCollection.footprintElevation());

	if (settingsCollection.summaryVoxels())
	{
		geoCreator.extractOuterVoxelSummary(
			&cityShellObject,
			internalHelper_.get(),
			settingsCollection.footprintElevation(),
			geoRefRotation.GetRotation().GetRotationAngle()
		);

		geoCreator.extractInnerVoxelSummary(
			&cityInnerShellObject,
			internalHelper_.get()
		);
	}

	collection->addCityObject(cityBuildingObject);
	collection->addCityObject(cityShellObject);
	collection->addCityObject(cityInnerShellObject);
	collection->cullDuplicatedVerices();
	cityCollection_ = collection;
	return succesfullExit;
}

bool IOManager::write()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();


	cityCollection_->dumpJson(settingsCollection.getOutputPath());

	if (!settingsCollection.writeReport()) { return true; }
	nlohmann::json report;
	report["Settings"] = settingsToJSON();

	nlohmann::json timeReport;
	addTimeToJSON(&timeReport, "Internalizing", timeInternalizing_);
	addTimeToJSON(&timeReport, "Voxel creation", timeVoxel_);
	addTimeToJSON(&timeReport, "LoD0.0 generation", timeLoD00_);
	addTimeToJSON(&timeReport, "LoD0.2 generation", timeLoD02_);
	addTimeToJSON(&timeReport, "LoD1.0 generation", timeLoD10_);
	addTimeToJSON(&timeReport, "LoD1.2 generation", timeLoD12_);
	addTimeToJSON(&timeReport, "LoD1.3 generation", timeLoD13_);
	addTimeToJSON(&timeReport, "LoD2.2 generation", timeLoD22_);
	addTimeToJSON(&timeReport, "LoD3.2 generation", timeLoD32_);
	addTimeToJSON(&timeReport, "LoD5.0 (V) generation", timeV_);
	addTimeToJSON(&timeReport, "Total Processing",
		timeInternalizing_ +
		timeVoxel_ +
		timeLoD00_ +
		timeLoD02_ +
		timeLoD10_ +
		timeLoD12_ +
		timeLoD13_ +
		timeLoD22_ +
		timeLoD32_ +
		timeV_
	);

	report["Duration"] = timeReport;
	report["Errors"] = ErrorCollection::getInstance().toJson();

	//addTimeToJSON(&report, "Total running time", startTime, endTime);
	const std::string extension1 = ".json";

	boost::filesystem::path filePath(settingsCollection.getOutputPath());
	filePath.replace_extension("");
	if (hasExtension(filePath.string(), "city")) { filePath.replace_extension(""); }

	boost::filesystem::path filePathWithoutExtension = boost::filesystem::path(settingsCollection.getOutputPath()).stem();
	std::ofstream reportFile(filePath.string() + "_report.json");
	reportFile << report;
	reportFile.close();
	return true;
}
