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


bool IOManager::getJsonBoolValue(const nlohmann::json& jsonBoolValue)
{
	if (jsonBoolValue.is_boolean())
	{
		return static_cast<bool>(jsonBoolValue);
	}

	if (!jsonBoolValue.is_number_integer() &&
		!jsonBoolValue.is_number_unsigned())
	{
		throw ErrorID::errorJsonInvalBool;
	}

	int jsonBoolInt = static_cast<int>(jsonBoolValue);
	if (jsonBoolInt == 0) { return false; }
	if (jsonBoolInt == 1) { return true; }

	throw ErrorID::errorJsonInvalBool; //if not 0 or 1 invalid
}


int IOManager::getJsonInt(const nlohmann::json& jsonIntValue, bool requiredPositive) 
{
	if (!jsonIntValue.is_number_integer() && 
		!jsonIntValue.is_number_unsigned())
	{
		throw ErrorID::errorJsonInvalInt;
	}

	if (!requiredPositive) { return static_cast<int>(jsonIntValue); }

	int intValue = static_cast<int>(jsonIntValue);
	if (intValue < 0)
	{
		throw ErrorID::errorJsonInvalNegInt;
	}
	return intValue;
}

double IOManager::getJsonDouble(const nlohmann::json& jsonDouleValue) 
{
	if (!jsonDouleValue.is_number_integer() &&
		!jsonDouleValue.is_number_unsigned() &&
		!jsonDouleValue.is_number_float())
	{
		throw ErrorID::errorJsonInvalNum;
	}
	return static_cast<double>(jsonDouleValue);
}

std::string IOManager::getJsonString(const nlohmann::json& jsonStringValue)
{
	if (!jsonStringValue.is_string())
	{
		throw ErrorID::errorJsonInvalString;
	}
	return static_cast<std::string>(jsonStringValue);
}

std::string IOManager::getJsonPath(const nlohmann::json& jsonStringValue, bool valFolder, const std::string& fileExtension)
{
	std::string jsonPathPtr = "";
	try
	{
		jsonPathPtr = getJsonString(jsonStringValue);
	}
	catch (const std::string& exception)
	{
		throw exception;
	}

	if (!hasExtension(jsonPathPtr, fileExtension))
	{
		throw ErrorID::errorJsonInvalPath;
	}

	if (valFolder)
	{
		std::filesystem::path folderPath(jsonPathPtr);
		if (!isValidPath(folderPath.parent_path().string()))
		{
			throw ErrorID::errorJsonNoRealPath;
		}
		return jsonPathPtr;
	}

	if (!isValidPath(jsonPathPtr))
	{
		throw ErrorID::errorJsonNoRealPath;
	}

	return jsonPathPtr;
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
	std::string stringNoValFilePath = CommunicationStringEnum::getString(CommunicationStringID::infoNoFilePath);

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

	// test if input path to config json is valid
	if (!isValidPath(inputPath))
	{
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoValFilePaths));
	}
	else if (!hasExtension(inputPath, "json")) {
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoValFilePaths));
	}
	settingsCollection.setInputJSONPath(inputPath);
	
	std::ifstream f(settingsCollection.getInputJSONPath());
	nlohmann::json json = nlohmann::json::parse(f);

	// in and output related settings
	// get filepath object
	std::string filePathsOName = JsonObjectInEnum::getString(JsonObjectInID::filePaths);
	if (!json.contains(filePathsOName))
	{
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + filePathsOName);
	}
	nlohmann::json filePaths = json[filePathsOName];

	// get output name
	std::string outputOName = JsonObjectInEnum::getString(JsonObjectInID::filePatsOutput);
	if (!filePaths.contains(outputOName))
	{
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + outputOName);
	}

	try { settingsCollection.setOutputPath(getJsonPath(filePaths[outputOName], true, "json")); }
	catch (const ErrorID& exceptionId) { throw std::string(errorWarningStringEnum::getString(exceptionId) + outputOName); }

	// set report output toggle
	std::string outputReportOName = JsonObjectInEnum::getString(JsonObjectInID::outputReport);
	if (json.contains(outputReportOName))
	{
		try
		{
			bool reportBool = getJsonBoolValue(json[outputReportOName]);
			settingsCollection.setWriteReport(reportBool);
		}
		catch (const ErrorID& exceptionId)
		{
			throw std::string(errorWarningStringEnum::getString(exceptionId) + outputReportOName);
		}
	}
	// from now on errors can be stored
	
	// get ifc input path array
	std::string inputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathsInput);
	if (!filePaths.contains(inputOName))
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonMissingEntry, inputOName);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + inputOName);
	}
	nlohmann::json inputPaths = filePaths[inputOName];
	if (inputPaths.type() != nlohmann::json::value_t::array)
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalArray, inputOName);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalArray) + inputOName);
	}

	// store input ifc paths
	for (size_t i = 0; i < inputPaths.size(); i++) {
		try { settingsCollection.addToIfcPathList(getJsonPath(inputPaths[i], false, "ifc")); }
		catch (const ErrorID& exceptionId) 
		{
			ErrorCollection::getInstance().addError(exceptionId, inputOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + inputOName);
		}
	}

	// set the threadcount
	bool threadFound = false;
	std::string threadMaxOName = JsonObjectInEnum::getString(JsonObjectInID::maxThread);
	if (json.contains(threadMaxOName))
	{
		try
		{
			int desiredThreadCount = getJsonInt(json[threadMaxOName], true);
			if (desiredThreadCount > 0)
			{
				settingsCollection.setThreadcount(desiredThreadCount);
				threadFound = true;
			}
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, threadMaxOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + threadMaxOName);
		}
	}
	if (!threadFound) // if entry not found set to max availble - 2
	{
		int availableThreads = std::thread::hardware_concurrency();
		if (availableThreads - 2 > 0) { settingsCollection.setThreadcount(availableThreads - 2); }
		else { settingsCollection.setThreadcount(availableThreads); }
	}

	//JSOn related output and processing settings
	std::string lodOutputOName = JsonObjectInEnum::getString(JsonObjectInID::lodOutput);
	std::string jsonOname = JsonObjectInEnum::getString(JsonObjectInID::JSON);

	nlohmann::json outputDataJson = {};
	if (json.contains(jsonOname)) { outputDataJson = json[jsonOname]; }

	// check for LoD output names
	if (!json.contains(lodOutputOName))
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonMissingEntry, lodOutputOName);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + lodOutputOName);
	}

	// reset the internal LoD values
	nlohmann::json lodList = json[lodOutputOName];

	// check if interior generation is required
	std::string generateInteriorOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenInterior);
	std::unordered_set<double> LoDWInterior = settingsCollection.getLoDWInterior();

	for (size_t i = 0; i < lodList.size(); i++) 
	{
		if (LoDWInterior.find(lodList[i]) == LoDWInterior.end()) { continue; }

		if (outputDataJson.contains(generateInteriorOName))
		{
			try
			{
				bool interiorBool = getJsonBoolValue(outputDataJson[generateInteriorOName]);
				settingsCollection.setMakeInterior(interiorBool);
			}
			catch (const ErrorID& exceptionId)
			{
				ErrorCollection::getInstance().addError(exceptionId, generateInteriorOName);
				throw std::string(errorWarningStringEnum::getString(exceptionId) + generateInteriorOName);
			}
		}
		break;
	}

	// check for footprint height
	const std::string& footprintElevOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintElev);
	if (outputDataJson.contains(footprintElevOName))
	{
		try
		{
			double footprintElev = getJsonDouble(outputDataJson[footprintElevOName]);
			settingsCollection.setFootprintElevation(footprintElev);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, footprintElevOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + footprintElevOName);
		}	
	}

	// check for horizontal offset
	std::string hSectionOffsetOName = JsonObjectInEnum::getString(JsonObjectInID::JSONSecOffset);
	if (outputDataJson.contains(hSectionOffsetOName))
	{
		try
		{
			double sectionOffset = getJsonDouble(outputDataJson[hSectionOffsetOName]);
			settingsCollection.setHorizontalSectionOffset(sectionOffset);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, hSectionOffsetOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + hSectionOffsetOName);
		}
	}

	// check if roof outline and footprint have to generated and which LoD are to be created
	std::string generatefootprOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenFootPrint); // check if footprint has to be output in LoD0.2
	std::string FootrpintBSOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintBShape); // check if footprint based output is desired (LoD1.2, 1.3 and 2.2)
	std::string generateRoofOlineOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenRoofOutline); // check if roofprint has to be output in LoD0.2

	bool generateFootprintBool = settingsCollection.makeFootPrint();
	bool footprintBasedBool = settingsCollection.footPrintBased();
	bool generateRoofprintBool = settingsCollection.makeRoofPrint();

	if (outputDataJson.contains(generatefootprOName))
	{
		try { generateFootprintBool = getJsonBoolValue(outputDataJson[generatefootprOName]); }
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, generatefootprOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + generatefootprOName);
		}
	}
	if (outputDataJson.contains(FootrpintBSOName))
	{
		try { footprintBasedBool = getJsonBoolValue(outputDataJson[FootrpintBSOName]); }
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, FootrpintBSOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + FootrpintBSOName);
		}
	}
	if (outputDataJson.contains(generateRoofOlineOName))
	{
		try { generateRoofprintBool = getJsonBoolValue(outputDataJson[generateRoofOlineOName]); }
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, generateRoofOlineOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + generateRoofOlineOName);
		}
	}

	for (const double& currentLoD: lodList)
	{
		if (currentLoD == 0.0)
		{ 
			settingsCollection.setMake00(true);
			continue;
		}
		if (currentLoD == 0.2)
		{
			settingsCollection.setMake02(true);
			settingsCollection.setMakeFootPrint(generateFootprintBool);
			settingsCollection.setMakeRoofPrint(generateRoofprintBool);
			settingsCollection.setMakeOutlines(generateRoofprintBool);
			continue;
		}
		if (currentLoD == 0.3)
		{
			settingsCollection.setMake03(true);
			continue;
		}
		if (currentLoD == 1.0)
		{
			settingsCollection.setMake10(true);
			continue;
		}
		if (currentLoD == 1.2)
		{
			settingsCollection.setMake12(true);
			settingsCollection.setMakeOutlines(true);
			settingsCollection.setFootPrintBased(footprintBasedBool);
			continue;
		}
		if (currentLoD == 1.3)
		{
			settingsCollection.setMake13(true);
			settingsCollection.setMakeOutlines(true);
			settingsCollection.setFootPrintBased(footprintBasedBool);
			continue;
		}
		if (currentLoD == 2.2)
		{
			settingsCollection.setMake22(true);
			settingsCollection.setMakeOutlines(true);
			settingsCollection.setFootPrintBased(footprintBasedBool);
			continue;
		}
		if (currentLoD == 3.2)
		{
			settingsCollection.setMake32(true);
			continue;
		}
		if (currentLoD == 5.0)
		{
			settingsCollection.setMakeV(true);
			continue;
		}

		//if this is reached an unexpected value has been encountered
		std::stringstream lodStringStream;
		lodStringStream << std::fixed << std::setprecision(1) << currentLoD;
		std::string formattedLoD = lodStringStream.str();

		ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalidLod, formattedLoD);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLod) + formattedLoD);
	}

	std::string georeferenceOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGeoreference);
	if (outputDataJson.contains(georeferenceOName))
	{
		try
		{
			bool georeferenceBool = getJsonBoolValue(outputDataJson[georeferenceOName]);
			settingsCollection.setGeoReference(georeferenceBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, georeferenceOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + georeferenceOName);
		}
	}

	std::string mergeSemantics = JsonObjectInEnum::getString(JsonObjectInID::JSONMergeSemantics);
	if (outputDataJson.contains(mergeSemantics))
	{
		try
		{
			bool mergeSemanticBool = getJsonBoolValue(outputDataJson[mergeSemantics]);
			settingsCollection.setmergeSemantics(mergeSemanticBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, mergeSemantics);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + mergeSemantics);
		}
	}

	// Voxel related output and processing settings
	std::string voxelOName = JsonObjectInEnum::getString(JsonObjectInID::voxel);
	if (json.contains(voxelOName))
	{
		nlohmann::json voxelData = json[voxelOName];

		std::string voxelSizOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSize);
		std::string voxelSummarizeOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSummarize);
		std::string voxelIntersectionOName = JsonObjectInEnum::getString(JsonObjectInID::voxelIntersection);
		if (voxelData.contains(voxelSizOName))			
		{
			try
			{
				double voxelSizeDouble = getJsonDouble(voxelData[voxelSizOName]);
				settingsCollection.setVoxelSize(voxelSizeDouble);
			}
			catch (const ErrorID& exceptionId)
			{
				ErrorCollection::getInstance().addError(exceptionId, voxelSizOName);
				throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelSizOName);
			}
		}
		if (voxelData.contains(voxelSummarizeOName))
		{
			try
			{
				bool summarizeVoxelBool = getJsonBoolValue(voxelData[voxelSummarizeOName]);
				settingsCollection.setSummaryVoxels(summarizeVoxelBool);
			}
			catch (const ErrorID& exceptionId)
			{
				ErrorCollection::getInstance().addError(exceptionId, voxelSummarizeOName);
				throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelSummarizeOName);
			}
		}
		if (voxelData.contains(voxelIntersectionOName)) 
		{
			int voxelLogicInt = 0;
			try
			{
				voxelLogicInt = getJsonInt(voxelData[voxelIntersectionOName], false);
			}
			catch (const ErrorID& exceptionId)
			{
				ErrorCollection::getInstance().addError(exceptionId, voxelIntersectionOName);
				throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelIntersectionOName);
			}
			if (voxelLogicInt == 2) { settingsCollection.setIntersectionLogic(2); }
			else if (voxelLogicInt == 3) { settingsCollection.setIntersectionLogic(3); }
			else
			{
				ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalidLogic);
				throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLogic) + voxelIntersectionOName);
			}
		}
	}

	//IFC related processing settings
	std::string ifcOName = JsonObjectInEnum::getString(JsonObjectInID::IFC);

	nlohmann::json ifcInputJson = {};
	if (json.contains(ifcOName)) { ifcInputJson = json[ifcOName]; }

	std::string rotationOName = JsonObjectInEnum::getString(JsonObjectInID::IFCRotation);
	if (ifcInputJson.contains(rotationOName))
	{
		nlohmann::json rotationData = ifcInputJson[rotationOName];
		if (rotationData.is_boolean())
		{
			bool roationBool = static_cast<bool>(rotationData);
			settingsCollection.setAutoRotateGrid(!roationBool);
		}
		else if (rotationData.is_number())
		{
			settingsCollection.setAutoRotateGrid(false);
			settingsCollection.setDesiredRotation(static_cast<double>(rotationData)* (M_PI / 180));
		}
		else
		{
			ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalEntry, rotationOName);
			throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalEntry) + rotationOName);
		}
	}

	std::string defaultDivOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDefaultDiv);
	if (ifcInputJson.contains(defaultDivOName))
	{
		try
		{
			bool defaultDivBool = getJsonBoolValue(ifcInputJson[defaultDivOName]);
			settingsCollection.setUseDefaultDiv(defaultDivBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, defaultDivOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + defaultDivOName);
		}
	}

	std::string ignoreProxyOName = JsonObjectInEnum::getString(JsonObjectInID::IFCIgnoreProxy);
	if (ifcInputJson.contains(ignoreProxyOName))
	{
		try
		{
			bool ignoreProxyBool = getJsonBoolValue(ifcInputJson[ignoreProxyOName]);
			settingsCollection.setUseProxy(!ignoreProxyBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, ignoreProxyOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + ignoreProxyOName);
		}
	}

	std::string divObjectsOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDivObject);
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
		ErrorCollection::getInstance().addError(ErrorID::errorJsonNoDivObjects);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonNoDivObjects));
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
	simpleIteratorSettings.set(simpleIteratorSettings.DISABLE_OPENING_SUBTRACTIONS, true);

	settingsCollection.setIterator(iteratorSettings);
	settingsCollection.setSimpleIterator(simpleIteratorSettings);

	return true;
}

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

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << std::endl;
	std::cout << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "Used settings : " << std::endl;
	std::cout << std::endl;

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "I/O settings" << std::endl;
	std::cout << "- Configuration file:" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getInputJSONPath() << std::endl;
	std::cout << "- Input File(s):" << std::endl;
	for (const std::string& inputPath : settingsCollection.getIfcPathList()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << inputPath << std::endl; }
	std::cout << "- Output File:" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.getOutputPath() << std::endl;
	std::cout << "- Create Report:" << std::endl;
	if (settingsCollection.writeReport()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }
	std::cout << "- LoD export enabled:" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << getLoDEnabled() << std::endl;
	std::cout << std::endl;

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "IFC settings" << std::endl;
	std::cout << "- Model rotation:" << std::endl;
	if (settingsCollection.autoRotateGrid()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "automatic" << std::endl; } //TODO: add true north?
	else
	{ std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.desiredRotation() << std::endl;
	}
	std::cout << "- Space dividing objects:" << std::endl;
	if (settingsCollection.useDefaultDiv())
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << boost::to_upper_copy(*it) << std::endl; }
	}
	if (settingsCollection.useProxy())
	{
		std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "IFCBUILDINGELEMENTPROXY" << std::endl;
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	std::cout << std::endl;

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "voxel settings" << std::endl;
	std::cout << "- Voxel size:" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.voxelSize() << std::endl;
	std::cout << "- Voxel logic:" << std::endl;
	if (settingsCollection.intersectionLogic() == 2) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "plane" << std::endl; }
	if (settingsCollection.intersectionLogic() == 3) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "solid" << std::endl; }
	std::cout << "- Store voxel summary/approximation:" << std::endl;
	if (settingsCollection.summaryVoxels()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) <<"no" << std::endl; }
	std::cout << std::endl;

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "json settings" << std::endl;
	std::cout << "- Footprint Elevation:" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.footprintElevation() << std::endl;
	std::cout << "- Footrint based extraction:" << std::endl;
	if (settingsCollection.footPrintBased()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }
	std::cout << "- horizontal section offset" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << settingsCollection.horizontalSectionOffset() << std::endl;
	if (settingsCollection.make02())
	{
		std::cout << "- Create footprint:" << std::endl;
		if (settingsCollection.makeFootPrint()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
		else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }

		std::cout << "- Store Lod0.2 roof outline:" << std::endl;
		if (settingsCollection.makeRoofPrint()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
		else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }
	}
	std::cout << "- Generate interior:" << std::endl;
	if (settingsCollection.makeInterior()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }
	std::cout << "- Georeference:" << std::endl;
	if (settingsCollection.geoReference()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }
	std::cout << "- Merge semantic objects" << std::endl;
	if (settingsCollection.mergeSemantics()) { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "yes" << std::endl; }
	else { std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) << "no" << std::endl; }

	std::cout << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) << "other settings:" << std::endl;
	std::cout << "- Max thread count" << std::endl;
	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) <<  settingsCollection.threadcount() << std::endl;
	std::cout << std::endl;

	std::cout << CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::seperator) << std::endl;
	std::cout << std::endl;
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

	nlohmann::json settingsJSON; // overal jsonFile

	// store the filepath data
	nlohmann::json ioJSON;
	std::string filePathsOName = JsonObjectInEnum::getString(JsonObjectInID::filePaths);
	std::string inputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathsInput);
	std::string outputOName = JsonObjectInEnum::getString(JsonObjectInID::filePatsOutput);

	ioJSON[inputOName] = settingsCollection.getIfcPathList();
	ioJSON[outputOName] = settingsCollection.getOutputPath();
	settingsJSON[filePathsOName] = ioJSON;

	// store the report data
	settingsJSON[JsonObjectInEnum::getString(JsonObjectInID::outputReport)] = "true";

	// store the thread data
	settingsJSON[JsonObjectInEnum::getString(JsonObjectInID::maxThread)] = settingsCollection.threadcount();

	// store the LoD output data
	std::vector<std::string> LoDList;
	if (settingsCollection.make00()) { LoDList.emplace_back("0.0"); }
	if (settingsCollection.make02()) { LoDList.emplace_back("0.2"); }
	if (settingsCollection.make10()) { LoDList.emplace_back("1.0"); }
	if (settingsCollection.make12()) { LoDList.emplace_back("1.2"); }
	if (settingsCollection.make13()) { LoDList.emplace_back("1.3"); }
	if (settingsCollection.make22()) { LoDList.emplace_back("2.2"); }
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
	std::string ifcRotationOName = JsonObjectInEnum::getString(JsonObjectInID::IFCRotation);
	std::string ifcDivOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDivObject);
	
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
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { DivList.emplace_back(boost::to_upper_copy(*it)); }
	ifcJSON[ifcDivOName] = DivList;
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
	jsonJSON[jsonGeoreferenceOName] = settingsCollection.geoReference();
	jsonJSON[jsonMergeSemanticsOName] = settingsCollection.mergeSemantics();
	settingsJSON[jsonOName] = jsonJSON;

	return settingsJSON;
}


bool IOManager::init(const std::vector<std::string>& inputPathList)
{
	//TODO: check if silent how to deal with this
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
	
	internalHelper_ = std::make_unique<helper>(SettingsCollection::getInstance().getIfcPathList());
	helper* internalHelperPtr = internalHelper_.get();
	if (!internalHelperPtr->isPopulated()) { return false; }
	if (!internalHelperPtr->hasSetUnits()) { return false; }

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
			ErrorCollection::getInstance().addError(ErrorID::errorFailedInit);
			return false;
		}
	}

	if (settingsCollection.make02() && settingsCollection.makeFootPrint() || settingsCollection.footPrintBased())
	{
		try
		{
			geoCreator.makeFootprint(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(ErrorID::errorFootprintFailed);
			succesfullExit = 0;
		}
	}

	if (settingsCollection.makeRoofPrint())
	{
		geoCreator.useroofprint0();
	}

	geoCreator.setRefRotation(geoRefRotation);

	// list collects the faces from the LoD03 creation to base LoD13 output on
	std::vector<std::vector<TopoDS_Face>> LoD03Faces;

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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD00);
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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD02);
			succesfullExit = 0;
		}
		timeLoD02_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();

	}
	if (settingsCollection.make03()) //TODO: make binding
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo03 = geoCreator.makeLoD03(internalHelper_.get(), &LoD03Faces, &kernel, 1);
			for (size_t i = 0; i < geo03.size(); i++) { cityShellObject.addGeoObject(geo03[i]); }
			timeLoD03_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&)
		{
			ErrorCollection::getInstance().addError(ErrorID::failedLoD03);
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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD10);
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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD12);
			succesfullExit = 0;
		}
	}
	if (settingsCollection.make13())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject> geo13 = geoCreator.makeLoD13(internalHelper_.get(), LoD03Faces, &kernel, 1);
			for (size_t i = 0; i < geo13.size(); i++) { cityShellObject.addGeoObject(geo13[i]); }
			timeLoD13_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) 
		{ 
			ErrorCollection::getInstance().addError(ErrorID::failedLoD13);
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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD22);
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
			ErrorCollection::getInstance().addError(ErrorID::failedLoD32);
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

	if (settingsCollection.makeInterior()) //TODO: make this a function
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
	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "buildingHeight", internalHelper_.get()->getUrrPoint().Z() - settingsCollection.footprintElevation());

	//TODO: get storey data
	IfcSchema::IfcBuildingStorey::list::ptr storeyList = internalHelper_.get()->getSourceFile(0)->instances_by_type<IfcSchema::IfcBuildingStorey>();

	double groundHeight = settingsCollection.footprintElevation();
	double smallestDistance = 1000000;
	double groundFloorHeight = 0;
	std::vector<double> storeyHeights;
	for (auto it = storeyList->begin(); it != storeyList->end(); ++it)
	{
		IfcSchema::IfcBuildingStorey* storeyObject = *it;
		double floorHeight = storeyObject->Elevation().get() * internalHelper_.get()->getScaler(0);
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

	//TODO: find closest storey to ground level
	//TODO: count storeys above this value + 1
	//TODO: count storeys below this value

	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "storeysAboveGround", storeyFloors);
	cityBuildingObject.addAttribute(sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "storeysBelowGround", basementFloors);

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

bool IOManager::write(bool reportOnly)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (settingsCollection.getOutputPath() == "") {
		//TODO: add cout for no report file made
		return true; 
	} // no output path set yet, cannot write to unknown location

	if (!reportOnly)
	{
		cityCollection_->dumpJson(settingsCollection.getOutputPath());
	}
	if (!settingsCollection.writeReport()) { return true; }
	nlohmann::json report;
	report["Input settings"] = settingsToJSON();

	nlohmann::json timeReport;
	addTimeToJSON(&timeReport, "Internalizing", timeInternalizing_);
	addTimeToJSON(&timeReport, "Voxel creation", timeVoxel_);
	addTimeToJSON(&timeReport, "LoD0.0 generation", timeLoD00_);
	addTimeToJSON(&timeReport, "LoD0.2 generation", timeLoD02_);
	addTimeToJSON(&timeReport, "LoD0.2 generation", timeLoD03_);
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
