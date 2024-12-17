#include "settingsCollection.h"
#include "errorCollection.h"
#include "stringManager.h"

#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <sys/stat.h>

#include <algorithm>
#include <vector>

std::unordered_set<std::string> getIfcTypeList();

bool hasExtension(const std::string& string, const std::string& ext)
{
	std::string substring = boost::to_lower_copy<std::string>(string.substr(string.find_last_of(".") + 1));
	if (substring == ext) { return true; }
	return false;
}

bool isValidPath(const std::string& path)
{
	return boost::filesystem::exists(path);
}

bool getJsonBoolValue(const nlohmann::json& jsonBoolValue)
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

int getJsonInt(const nlohmann::json& jsonIntValue, bool requiredPositive, bool requiredNonZero)
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

	if (!requiredNonZero) { return intValue; }
	if (intValue == 0) { throw ErrorID::errorJsonInvalZeroInt; }
	return intValue;
}

double getJsonDouble(const nlohmann::json& jsonDouleValue)
{
	if (!jsonDouleValue.is_number_integer() &&
		!jsonDouleValue.is_number_unsigned() &&
		!jsonDouleValue.is_number_float())
	{
		throw ErrorID::errorJsonInvalNum;
	}
	return static_cast<double>(jsonDouleValue);
}

std::string getJsonString(const nlohmann::json& jsonStringValue)
{
	if (!jsonStringValue.is_string())
	{
		throw ErrorID::errorJsonInvalString;
	}
	return static_cast<std::string>(jsonStringValue);
}

std::string getJsonPath(const nlohmann::json& jsonStringValue, bool valFolder, const std::string& fileExtension)
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


void SettingsCollection::setJSONRelatedSettings(const nlohmann::json& json)
{
	std::string jsonOname = JsonObjectInEnum::getString(JsonObjectInID::JSON);
	if (!json.contains(jsonOname)) { return; }

	nlohmann::json jsonDataJson = json[jsonOname];

	try
	{
		setMakeRoofPrint(jsonDataJson);
		setMakeFootPrint(jsonDataJson);
		setMakeOutlines(jsonDataJson);

		setMakeInterior(jsonDataJson);
		setFootprintElevation(jsonDataJson);
		setHorizontalSectionOffset(jsonDataJson);

		setGeoReference(jsonDataJson);
		setmergeSemantics(jsonDataJson);
	}
	catch (const std::string& errorString)
	{
		throw errorString;
	}

	return;
}

void SettingsCollection::setVoxelRelatedSettings(const nlohmann::json& json)
{
	// Voxel related output and processing settings
	std::string voxelOName = JsonObjectInEnum::getString(JsonObjectInID::voxel);
	if (!json.contains(voxelOName)) { return; }
	nlohmann::json voxelDataJson = json[voxelOName];

	try
	{
		setVoxelSize(voxelDataJson);
		setSummaryVoxels(voxelDataJson);
		setIntersectionLogic(voxelDataJson);
	}
	catch (const std::string& errorString)
	{
		throw errorString;
	}
	return;
}

void SettingsCollection::setIFCRelatedSettings(const nlohmann::json& json)
{
	//IFC related processing settings
	std::string ifcOName = JsonObjectInEnum::getString(JsonObjectInID::IFC);
	if (!json.contains(ifcOName)) { return; }
	nlohmann::json ifcDataJson = json[ifcOName];

	try
	{
		setRotation(ifcDataJson);
		setUseDefaultDiv(ifcDataJson);
		setUseProxy(ifcDataJson);
		setCustomDivList(ifcDataJson);
		setSimpleGeoGrade(ifcDataJson);
	}
	catch (const std::string& errorString)
	{
		throw errorString;
	}
	return;
}

void SettingsCollection::generateGeneralSettings()
{
	// set generated settings
	if (make00() || make10())
	{
		if (!make02() &&
			!make03() &&
			!make12() &&
			!make13() &&
			!make22() &&
			!make32() &&
			!summaryVoxels())
		{
			setRequireVoxels(false);
		}
	}

	if (!make32() && !makeV() && !summaryVoxels())
	{
		if (!makeFootPrint() && !makeInterior())
		{
			setRequireFullVoxels(false);
		}
	}

	// set ifcGeomsettings
	IfcGeom::IteratorSettings iteratorSettings;

	IfcGeom::IteratorSettings simpleIteratorSettings;
	simpleIteratorSettings.set(simpleIteratorSettings.DISABLE_OPENING_SUBTRACTIONS, true);

	setIterator(iteratorSettings);
	setSimpleIterator(simpleIteratorSettings);
	return;
}

void SettingsCollection::setInputJSONPath(const std::string& inputString, bool validate)
{
	if (validate)
	{
		if (!isValidPath(inputString))
		{
			throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoValFilePaths));
		}
		else if (!hasExtension(inputString, "json")) {
			throw std::string(errorWarningStringEnum::getString(ErrorID::errorNoValFilePaths));
		}
	}
	InputJsonPath_ = inputString;
	return;
}

void SettingsCollection::setIOPaths(const nlohmann::json& json)
{
	// get filepath object
	std::string filePathsOName = JsonObjectInEnum::getString(JsonObjectInID::filePaths);
	if (!json.contains(filePathsOName))
	{
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + filePathsOName);
	}
	nlohmann::json filePaths = json[filePathsOName];

	// get the output city file path
	std::string outputOName = JsonObjectInEnum::getString(JsonObjectInID::filePathOutput);
	if (filePaths.contains(outputOName))
	{
		try
		{
			setOutputIFCPath(getJsonPath(filePaths[outputOName], true, "json"));
		}
		catch (const ErrorID& exceptionId)
		{
			throw std::string(errorWarningStringEnum::getString(exceptionId) + outputOName);
		}	
	}
	else
	{
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + outputOName);
	}

	// get the output report path
	std::string outputReportPathOName = JsonObjectInEnum::getString(JsonObjectInID::filePathReport);
	if (filePaths.contains(outputReportPathOName))
	{
		try
		{
			setOutputReportPath(getJsonPath(filePaths[outputReportPathOName], true, "json"));
		}
		catch (const ErrorID& exceptionId)
		{
			throw std::string(errorWarningStringEnum::getString(exceptionId) + outputReportPathOName);
		}
	}
	else
	{
		boost::filesystem::path filePath(getOutputIFCPath());
		boost::filesystem::path parentFolder = filePath.parent_path();
		boost::filesystem::path fileNameStem = filePath.stem();
		std::string reportPath = (parentFolder / (fileNameStem.string() + "_report.json")).string();
		setOutputReportPath(reportPath);
	}
	
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
		try { addToIfcPathList(getJsonPath(inputPaths[i], false, "ifc")); }
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, inputOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + inputOName);
		}
	}
	return;
}

void SettingsCollection::setLoD(const nlohmann::json& json)
{
	std::string lodOutputOName = JsonObjectInEnum::getString(JsonObjectInID::lodOutput);
	if (!json.contains(lodOutputOName))
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonMissingEntry, lodOutputOName);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonMissingEntry) + lodOutputOName);
	}

	nlohmann::json LoDContainer = json[lodOutputOName];

	if (!LoDContainer.is_array())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalArray, lodOutputOName);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalArray) + lodOutputOName);
	}

	for (const double& currentLoD : LoDContainer)
	{
		if (currentLoD == 0.0)
		{
			setMake00(true);
			continue;
		}
		if (currentLoD == 0.2)
		{
			setMake02(true);
			continue;
		}
		if (currentLoD == 0.3)
		{
			setMake03(true);
			continue;
		}
		if (currentLoD == 1.0)
		{
			setMake10(true);
			continue;
		}
		if (currentLoD == 1.2)
		{
			setMake12(true);
			continue;
		}
		if (currentLoD == 1.3)
		{
			setMake13(true);
			continue;
		}
		if (currentLoD == 2.2)
		{
			setMake22(true);
			continue;
		}
		if (currentLoD == 3.2)
		{
			setMake32(true);
			continue;
		}
		if (currentLoD == 5.0)
		{
			setMakeV(true);
			continue;
		}

		//if this is reached an unexpected value has been encountered
		std::stringstream lodStringStream;
		lodStringStream << std::fixed << std::setprecision(1) << currentLoD;
		std::string formattedLoD = lodStringStream.str();

		ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalidLod, formattedLoD);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLod) + formattedLoD);
	}
	return;
}

void SettingsCollection::setMakeOutlines(const nlohmann::json& json)
{
	if (!make02() &&
		!make03() &&
		!make12() &&
		!make13() &&
		!make22() 
		) //TODO: check this list
	{
		return;
	}

	setMakeOutlines(true);
	return;

}

void SettingsCollection::setMakeFootPrint(const nlohmann::json& json)
{
	if (!make02())
	{
		setMakeFootPrint(false);
		return;
	}

	// check for footprint height
	const std::string& footprintOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenFootPrint);
	if (json.contains(footprintOName))
	{
		try
		{
			bool makeFootprint = getJsonBoolValue(json[footprintOName]);
			setMakeFootPrint(makeFootprint);

		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, footprintOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + footprintOName);
		}
	}
	return;
}

void SettingsCollection::setMakeRoofPrint(const nlohmann::json& json)
{
	if (!make02())
	{
		setMakeRoofPrint(false);
		return;
	}

	std::string generateRoofOlineOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenRoofOutline); // check if roofprint has to be output in LoD0.2
	if (json.contains(generateRoofOlineOName))
	{
		try 
		{ 
			bool generateRoofprintBool = getJsonBoolValue(json[generateRoofOlineOName]); 
			setMakeRoofPrint(generateRoofprintBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, generateRoofOlineOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + generateRoofOlineOName);
		}
	}
	return;
}

void SettingsCollection::setMakeInterior(const nlohmann::json& json)
{
	if (!make02() && 
		!make12() && 
		!make22() && 
		!make32() && 
		!makeV()
		)
	{
		return;
	}


	std::string generateInteriorOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGenInterior);
	if (json.contains(generateInteriorOName))
	{
		try
		{
			bool interiorBool = getJsonBoolValue(json[generateInteriorOName]);
			setMakeInterior(interiorBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, generateInteriorOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + generateInteriorOName);
		}
	}
	return;
}

void SettingsCollection::setFootPrintBased(const nlohmann::json& json)
{
	if (!make12() && 
		!make13() && 
		!make22())
	{
		return;
	}

	std::string FootrpintBSOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintBShape); // check if footprint based output is desired (LoD1.2, 1.3 and 2.2)
	if (json.contains(FootrpintBSOName))
	{
		try 
		{ 
			bool footprintBasedBool = getJsonBoolValue(json[FootrpintBSOName]); 
			setFootPrintBased(footprintBasedBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, FootrpintBSOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + FootrpintBSOName);
		}
	}
}

void SettingsCollection::setGeoReference(const nlohmann::json& json)
{
	std::string georeferenceOName = JsonObjectInEnum::getString(JsonObjectInID::JSONGeoreference);
	if (json.contains(georeferenceOName))
	{
		try
		{
			bool georeferenceBool = getJsonBoolValue(json[georeferenceOName]);
			setGeoReference(georeferenceBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, georeferenceOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + georeferenceOName);
		}
	}
	return;
}

void SettingsCollection::setSummaryVoxels(const nlohmann::json& json)
{
	std::string voxelSummarizeOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSummarize);
	if (json.contains(voxelSummarizeOName))
	{
		try
		{
			bool summarizeVoxelBool = getJsonBoolValue(json[voxelSummarizeOName]);
			setSummaryVoxels(summarizeVoxelBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, voxelSummarizeOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelSummarizeOName);
		}
	}
	return;
}

void SettingsCollection::setWriteReport(const nlohmann::json& json)
{
	std::string outputReportOName = JsonObjectInEnum::getString(JsonObjectInID::outputReport);
	if (json.contains(outputReportOName))
	{
		try
		{
			bool reportBool = getJsonBoolValue(json[outputReportOName]);
			setWriteReport(reportBool);
		}
		catch (const ErrorID& exceptionId)
		{
			throw std::string(errorWarningStringEnum::getString(exceptionId) + outputReportOName);
		}
	}
	return;
}

void SettingsCollection::setUseDefaultDiv(const nlohmann::json& json)
{

	std::string defaultDivOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDefaultDiv);
	if (json.contains(defaultDivOName))
	{
		try
		{
			bool defaultDivBool = getJsonBoolValue(json[defaultDivOName]);
			setUseDefaultDiv(defaultDivBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, defaultDivOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + defaultDivOName);
		}
	}
	return;
}

void SettingsCollection::setUseProxy(const nlohmann::json& json)
{
	std::string ignoreProxyOName = JsonObjectInEnum::getString(JsonObjectInID::IFCIgnoreProxy);
	if (json.contains(ignoreProxyOName))
	{
		try
		{
			bool ignoreProxyBool = getJsonBoolValue(json[ignoreProxyOName]);
			setUseProxy(!ignoreProxyBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, ignoreProxyOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + ignoreProxyOName);
		}
	}
	return;
}

void SettingsCollection::setSimpleGeoGrade(const nlohmann::json& json)
{
	std::string simpleGeoOName = JsonObjectInEnum::getString(JsonObjectInID::IFCsimplefyGeo);
	if (json.contains(simpleGeoOName))
	{
		try
		{
			int simpleGeoInt = getJsonInt(json[simpleGeoOName], false, false);

			if (simpleGeoInt < 0 && simpleGeoInt > 3)
			{
				//TODO: add error
				return;
			}

			setSimpleGeoGrade(simpleGeoInt);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, simpleGeoOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + simpleGeoOName);
		}
	}
	return;
}

void SettingsCollection::setCustomDivList(const nlohmann::json& json)
{
	std::string divObjectsOName = JsonObjectInEnum::getString(JsonObjectInID::IFCDivObject);
	if (!json.contains(divObjectsOName))
	{
		return;
	}

	std::unordered_set<std::string> availableIfcTypes = getIfcTypeList();
	std::vector<std::string> stringDivList = json[divObjectsOName];

	for (size_t i = 0; i < stringDivList.size(); i++)
	{
		std::string potentialType = stringDivList[i];
		std::transform(potentialType.begin(), potentialType.end(), potentialType.begin(), ::toupper);

		if (std::find(CustomDivList_.begin(), CustomDivList_.end(), potentialType) != CustomDivList_.end())
		{
			continue;
		}

		if (availableIfcTypes.find(potentialType) == availableIfcTypes.end())
		{
			continue;
		}
		addToCustomDivList(potentialType);
	}

	if (!getCustomDivList().size() && !useDefaultDiv())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorJsonNoDivObjects);
		throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonNoDivObjects));
	}

	return;
}

void SettingsCollection::setmergeSemantics(const nlohmann::json& json)
{
	std::string mergeSemantics = JsonObjectInEnum::getString(JsonObjectInID::JSONMergeSemantics);
	if (json.contains(mergeSemantics))
	{
		try
		{
			bool mergeSemanticBool = getJsonBoolValue(json[mergeSemantics]);
			setmergeSemantics(mergeSemanticBool);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, mergeSemantics);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + mergeSemantics);
		}
	}
	return;
}

void SettingsCollection::setIntersectionLogic(const nlohmann::json& json)
{
	std::string voxelIntersectionOName = JsonObjectInEnum::getString(JsonObjectInID::voxelIntersection);
	if (json.contains(voxelIntersectionOName))
	{
		int voxelLogicInt = 0;
		try
		{
			voxelLogicInt = getJsonInt(json[voxelIntersectionOName], false, false);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, voxelIntersectionOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelIntersectionOName);
		}
		if (voxelLogicInt == 2) { setIntersectionLogic(2); }
		else if (voxelLogicInt == 3) { setIntersectionLogic(3); }
		else
		{
			ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalidLogic);
			throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLogic) + voxelIntersectionOName);
		}
	}
	return;
}

void SettingsCollection::setVoxelSize(const nlohmann::json& json)
{
	std::string voxelSizOName = JsonObjectInEnum::getString(JsonObjectInID::voxelSize);
	if (json.contains(voxelSizOName))
	{
		try
		{
			double voxelSizeDouble = getJsonDouble(json[voxelSizOName]);
			setVoxelSize(voxelSizeDouble);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, voxelSizOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + voxelSizOName);
		}
	}
	return;
}

void SettingsCollection::setRotation(const nlohmann::json& json)
{
	std::string rotationAngleOName = JsonObjectInEnum::getString(JsonObjectInID::IFCRotationAngle);
	if (json.contains(rotationAngleOName))
	{
		try
		{
			double rotationAngleDouble = getJsonDouble(json[rotationAngleOName]);
			setDesiredRotation(rotationAngleDouble);
			setAutoRotateGrid(false);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(ErrorID::errorJsonInvalEntry, rotationAngleOName);
			throw std::string(errorWarningStringEnum::getString(ErrorID::errorJsonInvalEntry) + rotationAngleOName);
		}
	}
	return;
}

void SettingsCollection::setFootprintElevation(const nlohmann::json& json)
{
	const std::string& footprintElevOName = JsonObjectInEnum::getString(JsonObjectInID::JSONFootprintElev);
	if (json.contains(footprintElevOName))
	{
		try
		{
			double footprintElev = getJsonDouble(json[footprintElevOName]);
			setFootprintElevation(footprintElev);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, footprintElevOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + footprintElevOName);
		}
	}
	return;
}

void SettingsCollection::setHorizontalSectionOffset(const nlohmann::json& json)
{
	std::string hSectionOffsetOName = JsonObjectInEnum::getString(JsonObjectInID::JSONSecOffset);
	if (json.contains(hSectionOffsetOName))
	{
		try
		{
			double sectionOffset = getJsonDouble(json[hSectionOffsetOName]);
			setHorizontalSectionOffset(sectionOffset);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, hSectionOffsetOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + hSectionOffsetOName);
		}
	}
	return;
}

void SettingsCollection::setThreadcount(const nlohmann::json& json)
{
	std::string threadMaxOName = JsonObjectInEnum::getString(JsonObjectInID::maxThread);
	if (json.contains(threadMaxOName))
	{
		try
		{
			int desiredThreadCount = getJsonInt(json[threadMaxOName], true, true);
			setThreadcount(desiredThreadCount);
		}
		catch (const ErrorID& exceptionId)
		{
			ErrorCollection::getInstance().addError(exceptionId, threadMaxOName);
			throw std::string(errorWarningStringEnum::getString(exceptionId) + threadMaxOName);
		}
	}
	else // if entry not found set to max availble - 2
	{
		int availableThreads = std::thread::hardware_concurrency();
		if (availableThreads - 2 > 0) { setThreadcount(availableThreads - 2); }
		else { setThreadcount(availableThreads); }
	}
	return;
}

IfcGeom::IteratorSettings SettingsCollection::iteratorSettings(bool simple)
{
	if (simple)
	{
		return simpleIteratorSettings_;
	}
	return iteratorSettings_;
}


std::unordered_set<std::string> getIfcTypeList() { //TODO: make this a resource
	 return {
		"IFC2DCOMPOSITECURVE",
		"IFCACTIONREQUEST",
		"IFCACTOR",
		"IFCACTORROLE",
		"IFCACTUATORTYPE",
		"IFCADDRESS",
		"IFCAIRTERMINALBOXTYPE",
		"IFCAIRTERMINALTYPE",
		"IFCAIRTOAIRHEATRECOVERYTYPE",
		"IFCALARMTYPE",
		"IFCANGULARDIMENSION",
		"IFCANNOTATION",
		"IFCANNOTATIONCURVEOCCURRENCE",
		"IFCANNOTATIONFILLAREA",
		"IFCANNOTATIONFILLAREAOCCURRENCE",
		"IFCANNOTATIONOCCURRENCE",
		"IFCANNOTATIONSURFACE",
		"IFCANNOTATIONSURFACEOCCURRENCE",
		"IFCANNOTATIONSYMBOLOCCURRENCE",
		"IFCANNOTATIONTEXTOCCURRENCE",
		"IFCAPPLICATION",
		"IFCAPPLIEDVALUE",
		"IFCAPPLIEDVALUERELATIONSHIP",
		"IFCAPPROVAL",
		"IFCAPPROVALACTORRELATIONSHIP",
		"IFCAPPROVALPROPERTYRELATIONSHIP",
		"IFCAPPROVALRELATIONSHIP",
		"IFCARBITRARYCLOSEDPROFILEDEF",
		"IFCARBITRARYOPENPROFILEDEF",
		"IFCARBITRARYPROFILEDEFWITHVOIDS",
		"IFCASSET",
		"IFCASYMMETRICISHAPEPROFILEDEF",
		"IFCAXIS1PLACEMENT",
		"IFCAXIS2PLACEMENT2D",
		"IFCAXIS2PLACEMENT3D",
		"IFCBSPLINECURVE",
		"IFCBEAM",
		"IFCBEAMTYPE",
		"IFCBEZIERCURVE",
		"IFCBLOBTEXTURE",
		"IFCBLOCK",
		"IFCBOILERTYPE",
		"IFCBOOLEANCLIPPINGRESULT",
		"IFCBOOLEANRESULT",
		"IFCBOUNDARYCONDITION",
		"IFCBOUNDARYEDGECONDITION",
		"IFCBOUNDARYFACECONDITION",
		"IFCBOUNDARYNODECONDITION",
		"IFCBOUNDARYNODECONDITIONWARPING",
		"IFCBOUNDEDCURVE",
		"IFCBOUNDEDSURFACE",
		"IFCBOUNDINGBOX",
		"IFCBOXEDHALFSPACE",
		"IFCBUILDING",
		"IFCBUILDINGELEMENT",
		"IFCBUILDINGELEMENTCOMPONENT",
		"IFCBUILDINGELEMENTPART",
		"IFCBUILDINGELEMENTPROXY",
		"IFCBUILDINGELEMENTPROXYTYPE",
		"IFCBUILDINGELEMENTTYPE",
		"IFCBUILDINGSTOREY",
		"IFCCSHAPEPROFILEDEF",
		"IFCCABLECARRIERFITTINGTYPE",
		"IFCCABLECARRIERSEGMENTTYPE",
		"IFCCABLESEGMENTTYPE",
		"IFCCALENDARDATE",
		"IFCCARTESIANPOINT",
		"IFCCARTESIANTRANSFORMATIONOPERATOR",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3D",
		"IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM",
		"IFCCENTERLINEPROFILEDEF",
		"IFCCHAMFEREDGEFEATURE",
		"IFCCHILLERTYPE",
		"IFCCIRCLE",
		"IFCCIRCLEHOLLOWPROFILEDEF",
		"IFCCIRCLEPROFILEDEF",
		"IFCCLASSIFICATION",
		"IFCCLASSIFICATIONITEM",
		"IFCCLASSIFICATIONITEMRELATIONSHIP",
		"IFCCLASSIFICATIONNOTATION",
		"IFCCLASSIFICATIONNOTATIONFACET",
		"IFCCLASSIFICATIONREFERENCE",
		"IFCCLOSEDSHELL",
		"IFCCOILTYPE",
		"IFCCOLOURRGB",
		"IFCCOLOURSPECIFICATION",
		"IFCCOLUMN",
		"IFCCOLUMNTYPE",
		"IFCCOMPLEXPROPERTY",
		"IFCCOMPOSITECURVE",
		"IFCCOMPOSITECURVESEGMENT",
		"IFCCOMPOSITEPROFILEDEF",
		"IFCCOMPRESSORTYPE",
		"IFCCONDENSERTYPE",
		"IFCCONDITION",
		"IFCCONDITIONCRITERION",
		"IFCCONIC",
		"IFCCONNECTEDFACESET",
		"IFCCONNECTIONCURVEGEOMETRY",
		"IFCCONNECTIONGEOMETRY",
		"IFCCONNECTIONPOINTECCENTRICITY",
		"IFCCONNECTIONPOINTGEOMETRY",
		"IFCCONNECTIONPORTGEOMETRY",
		"IFCCONNECTIONSURFACEGEOMETRY",
		"IFCCONSTRAINT",
		"IFCCONSTRAINTAGGREGATIONRELATIONSHIP",
		"IFCCONSTRAINTCLASSIFICATIONRELATIONSHIP",
		"IFCCONSTRAINTRELATIONSHIP",
		"IFCCONSTRUCTIONEQUIPMENTRESOURCE",
		"IFCCONSTRUCTIONMATERIALRESOURCE",
		"IFCCONSTRUCTIONPRODUCTRESOURCE",
		"IFCCONSTRUCTIONRESOURCE",
		"IFCCONTEXTDEPENDENTUNIT",
		"IFCCONTROL",
		"IFCCONTROLLERTYPE",
		"IFCCONVERSIONBASEDUNIT",
		"IFCCOOLEDBEAMTYPE",
		"IFCCOOLINGTOWERTYPE",
		"IFCCOORDINATEDUNIVERSALTIMEOFFSET",
		"IFCCOSTITEM",
		"IFCCOSTSCHEDULE",
		"IFCCOSTVALUE",
		"IFCCOVERING",
		"IFCCOVERINGTYPE",
		"IFCCRANERAILASHAPEPROFILEDEF",
		"IFCCRANERAILFSHAPEPROFILEDEF",
		"IFCCREWRESOURCE",
		"IFCCSGPRIMITIVE3D",
		"IFCCSGSOLID",
		"IFCCURRENCYRELATIONSHIP",
		"IFCCURTAINWALL",
		"IFCCURTAINWALLTYPE",
		"IFCCURVE",
		"IFCCURVEBOUNDEDPLANE",
		"IFCCURVESTYLE",
		"IFCCURVESTYLEFONT",
		"IFCCURVESTYLEFONTANDSCALING",
		"IFCCURVESTYLEFONTPATTERN",
		"IFCDAMPERTYPE",
		"IFCDATEANDTIME",
		"IFCDEFINEDSYMBOL",
		"IFCDERIVEDPROFILEDEF",
		"IFCDERIVEDUNIT",
		"IFCDERIVEDUNITELEMENT",
		"IFCDIAMETERDIMENSION",
		"IFCDIMENSIONCALLOUTRELATIONSHIP",
		"IFCDIMENSIONCURVE",
		"IFCDIMENSIONCURVEDIRECTEDCALLOUT",
		"IFCDIMENSIONCURVETERMINATOR",
		"IFCDIMENSIONPAIR",
		"IFCDIMENSIONALEXPONENTS",
		"IFCDIRECTION",
		"IFCDISCRETEACCESSORY",
		"IFCDISCRETEACCESSORYTYPE",
		"IFCDISTRIBUTIONCHAMBERELEMENT",
		"IFCDISTRIBUTIONCHAMBERELEMENTTYPE",
		"IFCDISTRIBUTIONCONTROLELEMENT",
		"IFCDISTRIBUTIONCONTROLELEMENTTYPE",
		"IFCDISTRIBUTIONELEMENT",
		"IFCDISTRIBUTIONELEMENTTYPE",
		"IFCDISTRIBUTIONFLOWELEMENT",
		"IFCDISTRIBUTIONFLOWELEMENTTYPE",
		"IFCDISTRIBUTIONPORT",
		"IFCDOCUMENTELECTRONICFORMAT",
		"IFCDOCUMENTINFORMATION",
		"IFCDOCUMENTINFORMATIONRELATIONSHIP",
		"IFCDOCUMENTREFERENCE",
		"IFCDOOR",
		"IFCDOORLININGPROPERTIES",
		"IFCDOORPANELPROPERTIES",
		"IFCDOORSTYLE",
		"IFCDRAUGHTINGCALLOUT",
		"IFCDRAUGHTINGCALLOUTRELATIONSHIP",
		"IFCDRAUGHTINGPREDEFINEDCOLOUR",
		"IFCDRAUGHTINGPREDEFINEDCURVEFONT",
		"IFCDRAUGHTINGPREDEFINEDTEXTFONT",
		"IFCDUCTFITTINGTYPE",
		"IFCDUCTSEGMENTTYPE",
		"IFCDUCTSILENCERTYPE",
		"IFCEDGE",
		"IFCEDGECURVE",
		"IFCEDGEFEATURE",
		"IFCEDGELOOP",
		"IFCELECTRICAPPLIANCETYPE",
		"IFCELECTRICDISTRIBUTIONPOINT",
		"IFCELECTRICFLOWSTORAGEDEVICETYPE",
		"IFCELECTRICGENERATORTYPE",
		"IFCELECTRICHEATERTYPE",
		"IFCELECTRICMOTORTYPE",
		"IFCELECTRICTIMECONTROLTYPE",
		"IFCELECTRICALBASEPROPERTIES",
		"IFCELECTRICALCIRCUIT",
		"IFCELECTRICALELEMENT",
		"IFCELEMENT",
		"IFCELEMENTASSEMBLY",
		"IFCELEMENTCOMPONENT",
		"IFCELEMENTCOMPONENTTYPE",
		"IFCELEMENTQUANTITY",
		"IFCELEMENTTYPE",
		"IFCELEMENTARYSURFACE",
		"IFCELLIPSE",
		"IFCELLIPSEPROFILEDEF",
		"IFCENERGYCONVERSIONDEVICE",
		"IFCENERGYCONVERSIONDEVICETYPE",
		"IFCENERGYPROPERTIES",
		"IFCENVIRONMENTALIMPACTVALUE",
		"IFCEQUIPMENTELEMENT",
		"IFCEQUIPMENTSTANDARD",
		"IFCEVAPORATIVECOOLERTYPE",
		"IFCEVAPORATORTYPE",
		"IFCEXTENDEDMATERIALPROPERTIES",
		"IFCEXTERNALREFERENCE",
		"IFCEXTERNALLYDEFINEDHATCHSTYLE",
		"IFCEXTERNALLYDEFINEDSURFACESTYLE",
		"IFCEXTERNALLYDEFINEDSYMBOL",
		"IFCEXTERNALLYDEFINEDTEXTFONT",
		"IFCEXTRUDEDAREASOLID",
		"IFCFACE",
		"IFCFACEBASEDSURFACEMODEL",
		"IFCFACEBOUND",
		"IFCFACEOUTERBOUND",
		"IFCFACESURFACE",
		"IFCFACETEDBREP",
		"IFCFACETEDBREPWITHVOIDS",
		"IFCFAILURECONNECTIONCONDITION",
		"IFCFANTYPE",
		"IFCFASTENER",
		"IFCFASTENERTYPE",
		"IFCFEATUREELEMENT",
		"IFCFEATUREELEMENTADDITION",
		"IFCFEATUREELEMENTSUBTRACTION",
		"IFCFILLAREASTYLE",
		"IFCFILLAREASTYLEHATCHING",
		"IFCFILLAREASTYLETILESYMBOLWITHSTYLE",
		"IFCFILLAREASTYLETILES",
		"IFCFILTERTYPE",
		"IFCFIRESUPPRESSIONTERMINALTYPE",
		"IFCFLOWCONTROLLER",
		"IFCFLOWCONTROLLERTYPE",
		"IFCFLOWFITTING",
		"IFCFLOWFITTINGTYPE",
		"IFCFLOWINSTRUMENTTYPE",
		"IFCFLOWMETERTYPE",
		"IFCFLOWMOVINGDEVICE",
		"IFCFLOWMOVINGDEVICETYPE",
		"IFCFLOWSEGMENT",
		"IFCFLOWSEGMENTTYPE",
		"IFCFLOWSTORAGEDEVICE",
		"IFCFLOWSTORAGEDEVICETYPE",
		"IFCFLOWTERMINAL",
		"IFCFLOWTERMINALTYPE",
		"IFCFLOWTREATMENTDEVICE",
		"IFCFLOWTREATMENTDEVICETYPE",
		"IFCFLUIDFLOWPROPERTIES",
		"IFCFOOTING",
		"IFCFUELPROPERTIES",
		"IFCFURNISHINGELEMENT",
		"IFCFURNISHINGELEMENTTYPE",
		"IFCFURNITURESTANDARD",
		"IFCFURNITURETYPE",
		"IFCGASTERMINALTYPE",
		"IFCGENERALMATERIALPROPERTIES",
		"IFCGENERALPROFILEPROPERTIES",
		"IFCGEOMETRICCURVESET",
		"IFCGEOMETRICREPRESENTATIONCONTEXT",
		"IFCGEOMETRICREPRESENTATIONITEM",
		"IFCGEOMETRICREPRESENTATIONSUBCONTEXT",
		"IFCGEOMETRICSET",
		"IFCGRID",
		"IFCGRIDAXIS",
		"IFCGRIDPLACEMENT",
		"IFCGROUP",
		"IFCHALFSPACESOLID",
		"IFCHEATEXCHANGERTYPE",
		"IFCHYGROMATERIALPROPERTIES",
		"IFCSHAPEPROFILEDEF",
		"IFCIMAGE",
		"IFCINVENTORY",
		"IFCIRREGULARTIMESERIES",
		"IFCIRREGULARTIMESERIESVALUE",
		"IFCJUNCTIONBOXTYPE",
		"IFCLSHAPEPROFILEDEF",
		"IFCLABORRESOURCE",
		"IFCLAMPTYPE",
		"IFCLIBRARYINFORMATION",
		"IFCLIBRARYREFERENCE",
		"IFCLIGHTDISTRIBUTIONDATA",
		"IFCLIGHTFIXTURETYPE",
		"IFCLIGHTINTENSITYDISTRIBUTION",
		"IFCLIGHTSOURCE",
		"IFCLIGHTSOURCEAMBIENT",
		"IFCLIGHTSOURCEDIRECTIONAL",
		"IFCLIGHTSOURCEGONIOMETRIC",
		"IFCLIGHTSOURCEPOSITIONAL",
		"IFCLIGHTSOURCESPOT",
		"IFCLINE",
		"IFCLINEARDIMENSION",
		"IFCLOCALPLACEMENT",
		"IFCLOCALTIME",
		"IFCLOOP",
		"IFCMANIFOLDSOLIDBREP",
		"IFCMAPPEDITEM",
		"IFCMATERIAL",
		"IFCMATERIALCLASSIFICATIONRELATIONSHIP",
		"IFCMATERIALDEFINITIONREPRESENTATION",
		"IFCMATERIALLAYER",
		"IFCMATERIALLAYERSET",
		"IFCMATERIALLAYERSETUSAGE",
		"IFCMATERIALLIST",
		"IFCMATERIALPROPERTIES",
		"IFCMEASUREWITHUNIT",
		"IFCMECHANICALCONCRETEMATERIALPROPERTIES",
		"IFCMECHANICALFASTENER",
		"IFCMECHANICALFASTENERTYPE",
		"IFCMECHANICALMATERIALPROPERTIES",
		"IFCMECHANICALSTEELMATERIALPROPERTIES",
		"IFCMEMBER",
		"IFCMEMBERTYPE",
		"IFCMETRIC",
		"IFCMONETARYUNIT",
		"IFCMOTORCONNECTIONTYPE",
		"IFCMOVE",
		"IFCNAMEDUNIT",
		"IFCOBJECT",
		"IFCOBJECTDEFINITION",
		"IFCOBJECTPLACEMENT",
		"IFCOBJECTIVE",
		"IFCOCCUPANT",
		"IFCOFFSETCURVE2D",
		"IFCOFFSETCURVE3D",
		"IFCONEDIRECTIONREPEATFACTOR",
		"IFCOPENSH",
		"IFCOPENINGELEMENT",
		"IFCOPTICALMATERIALPROPERTIES",
		"IFCORDERACTION",
		"IFCORGANIZATION",
		"IFCORGANIZATIONRELATIONSHIP",
		"IFCORIENTEDEDGE",
		"IFCOUTLETTYPE",
		"IFCOWNERHISTORY",
		"IFCPARAMETERIZEDPROFILEDEF",
		"IFCPATH",
		"IFCPERFORMANCEHISTORY",
		"IFCPERMEABLECOVERINGPROPERTIES",
		"IFCPERMIT",
		"IFCPERSON",
		"IFCPERSONANDORGANIZATION",
		"IFCPHYSICALCOMPLEXQUANTITY",
		"IFCPHYSICALQUANTITY",
		"IFCPHYSICALSIMPLEQUANTITY",
		"IFCPILE",
		"IFCPIPEFITTINGTYPE",
		"IFCPIPESEGMENTTYPE",
		"IFCPIXELTEXTURE",
		"IFCPLACEMENT",
		"IFCPLANARBOX",
		"IFCPLANAREXTENT",
		"IFCPLANE",
		"IFCPLATE",
		"IFCPLATETYPE",
		"IFCPOINT",
		"IFCPOINTONCURVE",
		"IFCPOINTONSURFACE",
		"IFCPOLYLOOP",
		"IFCPOLYGONALBOUNDEDHALFSPACE",
		"IFCPOLYLINE",
		"IFCPORT",
		"IFCPOSTALADDRESS",
		"IFCPREDEFINEDCOLOUR",
		"IFCPREDEFINEDCURVEFONT",
		"IFCPREDEFINEDDIMENSIONSYMBOL",
		"IFCPREDEFINEDITEM",
		"IFCPREDEFINEDPOINTMARKERSYMBOL",
		"IFCPREDEFINEDSYMBOL",
		"IFCPREDEFINEDTERMINATORSYMBOL",
		"IFCPREDEFINEDTEXTFONT",
		"IFCPRESENTATIONLAYERASSIGNMENT",
		"IFCPRESENTATIONLAYERWITHSTYLE",
		"IFCPRESENTATIONSTYLE",
		"IFCPRESENTATIONSTYLEASSIGNMENT",
		"IFCPROCEDURE",
		"IFCPROCESS",
		"IFCPRODUCT",
		"IFCPRODUCTDEFINITIONSHAPE",
		"IFCPRODUCTREPRESENTATION",
		"IFCPRODUCTSOFCOMBUSTIONPROPERTIES",
		"IFCPROFILEDEF",
		"IFCPROFILEPROPERTIES",
		"IFCPROJECT",
		"IFCPROJECTORDER",
		"IFCPROJECTORDERRECORD",
		"IFCPROJECTIONCURVE",
		"IFCPROJECTIONELEMENT",
		"IFCPROPERTY",
		"IFCPROPERTYBOUNDEDVALUE",
		"IFCPROPERTYCONSTRAINTRELATIONSHIP",
		"IFCPROPERTYDEFINITION",
		"IFCPROPERTYDEPENDENCYRELATIONSHIP",
		"IFCPROPERTYENUMERATEDVALUE",
		"IFCPROPERTYENUMERATION",
		"IFCPROPERTYLISTVALUE",
		"IFCPROPERTYREFERENCEVALUE",
		"IFCPROPERTYSET",
		"IFCPROPERTYSETDEFINITION",
		"IFCPROPERTYSINGLEVALUE",
		"IFCPROPERTYTABLEVALUE",
		"IFCPROTECTIVEDEVICETYPE",
		"IFCPROXY",
		"IFCPUMPTYPE",
		"IFCQUANTITYAREA",
		"IFCQUANTITYCOUNT",
		"IFCQUANTITYLENGTH",
		"IFCQUANTITYTIME",
		"IFCQUANTITYVOLUME",
		"IFCQUANTITYWEIGHT",
		"IFCROOF",
		"IFCRADIUSDIMENSION",
		"IFCRAILING",
		"IFCRAILINGTYPE",
		"IFCRAMP",
		"IFCRAMPFLIGHT",
		"IFCRAMPFLIGHTTYPE",
		"IFCSLAB",
		"IFCSTRUCTURALACTION",
		"IFCSTRUCTURALACTIVITY",
		"IFCSTRUCTURALANALYSISMODEL",
		"IFCSTRUCTURALCONNECTION",
		"IFCSTRUCTURALCONNECTIONCONDITION",
		"IFCSTRUCTURALCURVECONNECTION",
		"IFCSTRUCTURALCURVEMEMBER",
		"IFCSTRUCTURALCURVEMEMBERVARYING",
		"IFCSTRUCTURALITEM",
		"IFCSTRUCTURALLINEARACTION",
		"IFCSTRUCTURALLINEARACTIONVARYING",
		"IFCSTRUCTURALLOAD",
		"IFCSTRUCTURALLOADGROUP",
		"IFCSTRUCTURALLOADLINEARFORCE",
		"IFCSTRUCTURALLOADPLANARFORCE",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENT",
		"IFCSTRUCTURALLOADSINGLEDISPLACEMENTDISTORTION",
		"IFCSTRUCTURALLOADSINGLEFORCE",
		"IFCSTRUCTURALLOADSINGLEFORCEWARPING",
		"IFCSTRUCTURALLOADTEMPERATURE",
		"IFCSTRUCTURALMEMBER",
		"IFCSTRUCTURALPLANARACTION",
		"IFCSTRUCTURALPOINTACTION",
		"IFCSTRUCTURALPOINTCONNECTION",
		"IFCSTRUCTURALPOINTREACTION",
		"IFCSTRUCTURALREACTION",
		"IFCSTRUCTURALRESULTGROUP",
		"IFCSTRUCTURALSTEELPROFILEPROPERTIES",
		"IFCSTRUCTURALSURFACECONNECTION",
		"IFCSTRUCTURALSURFACEMEMBER",
		"IFCSTRUCTURALSURFACEMEMBERVARYING",
		"IFCSTRUCTUREDDIMENSIONCALLOUT",
		"IFCSUBCONTRACTRESOURCE",
		"IFCSUBEDGE",
		"IFCSURFACE",
		"IFCSURFACECURVESWEPTAREASOLID",
		"IFCSURFACEOFLINEAREXTRUSION",
		"IFCSURFACEOFREVOLUTION",
		"IFCSURFACESTYLE",
		"IFCSURFACESTYLELIGHTING",
		"IFCSURFACESTYLEREFRACTION",
		"IFCSURFACESTYLERENDERING",
		"IFCSURFACESTYLESHADING",
		"IFCSURFACESTYLEWITHTEXTURES",
		"IFCSWEPTAREASOLID",
		"IFCSWITCHINGDEVICETYPE",
		"IFCSYMBOLSTYLE",
		"IFCSYMBOLSTYLESELECT",
		"IFCSYSTEM",
		"IFCSYSTEMFURNITUREELEMENTTYPE",
		"IFCTSHAPEPROFILEDEF",
		"IFCTABLE",
		"IFCTABLEROW",
		"IFCTANKTYPE",
		"IFCTASK",
		"IFCTASKTIME",
		"IFCTAXONOMICCLASSIFICATION",
		"IFCTAXONOMICCLASSIFICATIONRELATIONSHIP",
		"IFCTEXTSTYLE",
		"IFCTHEATRE",
		"IFCTHEATRETYPE",
		"IFCTHERMALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALMATERIALPROPERTIES",
		"IFCTHERMOPHYSICALPROPERTYSET",
		"IFCTHERMOPHYSICALPROPERTYSETUSAGE",
		"IFCTHERMOPHYSICALSIMPLEPROPERTY",
		"IFCTIMEPERIOD",
		"IFCTIMESERIES",
		"IFCTIMESERIESGROUP",
		"IFCTIMESERIESREFERENCE",
		"IFCTIMESERIESSCHEDULE",
		"IFCTIMESERIESVALUE",
		"IFCTOPOLOGICALREPRESENTATIONITEM",
		"IFCTRANSFORMERRESOURCE",
		"IFCTRANSPORTELEMENT",
		"IFCTRANSPORTELEMENTTYPE",
		"IFCTRANSPORTELEMENTSTATICDEFLECTION",
		"IFCTRANSPORTELEMENTSTATICREACTION",
		"IFCTRAPEZIUMPROFILEDEF",
		"IFCTRIMMEDCURVE",
		"IFCTUBEBUNDLETYPE",
		"IFCTWIRLGENERATORTYPE",
		"IFCUNITASSIGNMENT",
		"IFCUNITARYCONTROLELEMENT",
		"IFCUNITARYCONTROLELEMENTTYPE",
		"IFCUNITARYEQUIPMENT",
		"IFCUNITARYEQUIPMENTTYPE",
		"IFCUNITARYPRODUCT",
		"IFCUNITARYPRODUCTTYPE",
		"IFCVALVE",
		"IFCVALVETYPE",
		"IFCVECTOR",
		"IFCVERTEX",
		"IFCVERTEXLOOP",
		"IFCVERTEXPOINT",
		"IFCVIBRATORYPE",
		"IFCVIRTUALELEMENT",
		"IFCVIRTUALELEMENTTYPE",
		"IFCVOIDINGFEATURE",
		"IFCVOIDINGFEATURETYPE",
		"IFCVOLUMEBEAM",
		"IFCVOLUMEBEAMTYPE",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCWALLTYPE",
		"IFCWARPINGCONSTANTMEASURE",
		"IFCWARPINGMOMENTOFINERTIA_MEASURE",
		"IFCWINDOW",
		"IFCWINDOWLININGPROPERTIES",
		"IFCWINDOWPANELPROPERTIES",
		"IFCWINDOWSTYLE",
		"IFCZSHAPEPROFILEDEF"
	};
}