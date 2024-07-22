#include <string>
#include <map>

#include <nlohmann/json.hpp>

#ifndef STRINGMANAGER_H
#define STRINGMANAGER_H

struct ErrorObject {
	std::string errorCode_ = "";
	std::string errorDescript_ = "";
	std::vector<std::string> occuringObjectList_;

	nlohmann::json toJson();

	ErrorObject(
		std::string errorCode,
		std::string errorDescript
		);
};

// collects all the importance elements in communication with the user
enum class CommunicationStringImportanceID
{
	indent,
	info,
	warning,
	error
};

class CommunicationStringImportanceEnum {
public:
	static std::string getString(CommunicationStringImportanceID id);
};

enum class sourceIdentifierID {
	envExtractor,
	voxel,
	ifc
};

class sourceIdentifierEnum {
public:
	static std::string getString(sourceIdentifierID id);
};

// collects all the unit names for user comms, reports, and input
enum class UnitStringID {
	seconds,
	milliseconds,
	meter,
	meterFull,
	millimeter,
	millimeterFull
};

class UnitStringEnum {
public:
	static std::string getString(UnitStringID id);
};

// collects all the direct cout communication with the user
enum class CommunicationStringID {
	infoJsonRequest,
	infoNoFilePath,
	infoNoValFilePath,
	infoParsingFile,
	infoInternalizingGeo,
	infoCreateSpatialIndex,
	infoFoundUnits,
	infoDefaultVolumeUnit,
	indentValidIFCFound,
	indentcompIFCFound,
	indentSuccesFinished,

	errorNoValFilePaths,
	errorUnableToProcessFile,
	errorNoUnits,
	errorMultipleUnits,
	errorNoLengthUnit,
	errorNoAreaUnit,

	errorJSONReportPath,
	errorJSONFilePath,
	errorJSONInputPath,
	errorJSONInvalInputPathFormat,
	errorJSONNoValInputPath,
	errorJSONInvalInputPath,
	errorJSONOutputPath,
	errorJSONInvalOuputPathFormat,
	errorJSONInvalOuputFormat,
	errorJSONInvalOutputFolder,
	errorJSONNoDivObjects,

	warningUnableToParseIFC,
	warningNoValidIFC,
	warningIncompIFC,
	warningNoSlab,
	warningMultipleProjections,
	warningNoVolumeUnit,
	warmingIssueencountered
};

class CommunicationStringEnum {
public:
	static std::string getString(CommunicationStringID id);
};

// collects all the JSON object of the config files
enum class JsonObjectInID {
	outputReport,
	filePaths,
	filePathsInput,
	filePatsOutput,

	voxel,
	voxelSize,
	voxelSummarize,
	voxelIntersection,

	IFC,
	IFCRotation,
	IFCDefaultDiv,
	IFCIgnoreProxy,
	IFCDivObject,

	JSON,
	JSONFootprintElev,
	JSONGenFootPrint,
	JSONGenRoofOutline,
	JSONGenInterior,
	JSONGeoreference,

	generateReport,
	lodOutput
};

class JsonObjectInEnum {
public:
	static std::string getString(JsonObjectInID id);
};

enum class CJObjectID {
	metaDataTitle,
	outerShell,
	innerShell,
	v11,
	ifcDescription,
	ifcObjectType,
	ifcName,
	ifcLongName
};

class CJObjectEnum {
public:
	static std::string getString(CJObjectID id);
};

// collects all the error objects
enum class errorID {
	failedLoD00, // LoD0.0 creation failed
	failedLoD02, // LoD0.2 creation failed
	failedLoD10, // LoD1.0 creation failed
	failedLoD12, // LoD1.2 creation failed
	failedLoD13, // LoD1.3 creation failed
	failedLoD22, // LoD2.2 creation failed
	failedLoD32, // LoD3.2 creation failed
	failedJSONInit, //Config JSON error:
	failedInit, // Basic initialization failed
	failedFootprint, // Footprint creation failed
	failedStorey, // Storey creation failed
	failedConvert, // Failed to convert object
};


class errorMap {
private:
	static const std::map<errorID, ErrorObject> errorCollection;

public:
	static ErrorObject getErrorObject(errorID id);
};

#endif // STRINGMANAGER_H