#include "stringManager.h"

#include <string>
#include <map>

#include <nlohmann/json.hpp>


nlohmann::json ErrorObject::toJson()
{
	nlohmann::json JsonObject;
	JsonObject["ErrorCode"] = errorCode_;
	JsonObject["Error Description"] = errorDescript_;
	JsonObject["Occuring Objects"] = occuringObjectList_;

	return JsonObject;
}

ErrorObject::ErrorObject(std::string errorCode, std::string errorDescript)
{
	errorCode_ = errorCode;
	errorDescript_ = errorDescript;
}

std::string CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID id)
{
	//Include the spaces on the end for spacing
	switch (id) {
	case CommunicationStringImportanceID::indent:
		return "\t";
	case CommunicationStringImportanceID::info:
		return "[INFO] ";
	case CommunicationStringImportanceID::warning: 
		return "[WARNING] ";
	case CommunicationStringImportanceID::error:
		return "[Error] ";
	default:
		return "";
	}
}

std::string UnitStringEnum::getString(UnitStringID id)
{
	//Include the spaces on the end for spacing
	switch (id) {
	case UnitStringID::seconds:
		return "s";
	case UnitStringID::milliseconds:
		return "ms";
	case UnitStringID::meter:
		return "m";
	case UnitStringID::meterFull:
		return "meter";
	case UnitStringID::millimeter:
		return "mm";
	case UnitStringID::millimeterFull:
		return "millimeter";
	default:
		return "";
	}
}

std::string CommunicationStringEnum::getString(CommunicationStringID id)
{
	switch (id) {
	case CommunicationStringID::infoJsonRequest:
		return "Enter filepath of the config JSON";
	case CommunicationStringID::infoNoFilePath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "No filepath has been supplied";
	case CommunicationStringID::infoNoValFilePath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "No valid filepath has been supplied";
	case CommunicationStringID::infoParsingFile:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Parsing file ";
	case CommunicationStringID::infoInternalizingGeo:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Internalizing Geometry of Construction Model";
	case CommunicationStringID::infoCreateSpatialIndex:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Create Spatial Index";
	case CommunicationStringID::infoFoundUnits:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "found units:";
	case CommunicationStringID::infoDefaultVolumeUnit:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "SI unit for volume is set to cubic metre";

	case CommunicationStringID::indentValidIFCFound:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Valid IFC file found";
	case CommunicationStringID::indentcompIFCFound:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Compatible scheme found: ";
	case CommunicationStringID::indentSuccesFinished:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Successfully finished in: ";

	case CommunicationStringID::errorNoValFilePaths:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No valid filepath has been supplied";
	case CommunicationStringID::errorUnableToProcessFile:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "unable to process file(s)";
	case CommunicationStringID::errorNoUnits:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No unit assignment has been found";
	case CommunicationStringID::errorMultipleUnits:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "Multiple unit assignments have been found";
	case CommunicationStringID::errorNoLengthUnit:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "SI unit for length cannot be found";
	case CommunicationStringID::errorNoAreaUnit:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "SI unit for area cannot be found";

	case CommunicationStringID::errorJSONReportPath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain a valid output report entry";
	case CommunicationStringID::errorJSONFilePath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain Filpaths Entry";
	case CommunicationStringID::errorJSONInputPath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain Input Filepath Entry";
	case CommunicationStringID::errorJSONInvalInputPathFormat:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain valid input filepath entry: Input filepath entry should be array";
	case CommunicationStringID::errorJSONNoValInputPath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain valid input filepath entry";
	case CommunicationStringID::errorJSONInvalInputPath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file contains an invalid ifc input path: ";
	case CommunicationStringID::errorJSONOutputPath:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain Output Filepath Entry";
	case CommunicationStringID::errorJSONInvalOuputPathFormat:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain valid output path entry, output filepath entry should be string";
	case CommunicationStringID::errorJSONInvalOuputFormat:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain valid output path, output path should end on .json or .city.json";
	case CommunicationStringID::errorJSONInvalOutputFolder:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "Target filepath folder does not exist";
	case CommunicationStringID::errorJSONNoDivObjects:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No div objects are selected";


	case CommunicationStringID::warningUnableToParseIFC:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Unable to parse .ifc file";
	case CommunicationStringID::warningNoValidIFC:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "No valid ifc scheme found";
	case CommunicationStringID::warningIncompIFC:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Incompatible scheme found: ";
	case CommunicationStringID::warningNoSlab:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "No slab objects were found";
	case CommunicationStringID::warningMultipleProjections:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Multiple map projections detected";
	case CommunicationStringID::warningNoVolumeUnit:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "SI unit for volume cannot be found";
	case CommunicationStringID::warmingIssueencountered:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Encountered an issue";
	default:
		return "Output string not found";
	}
}

std::string sourceIdentifierEnum::getString(sourceIdentifierID id)
{
	switch (id) {
	case sourceIdentifierID::envExtractor:
		return "Env_ex ";
	case sourceIdentifierID::voxel:
		return "V_ex ";
	case sourceIdentifierID::ifc:
		return "IFC ";
	default:
		return "";
	}

	return std::string();
}

std::string JsonObjectInEnum::getString(JsonObjectInID id)
{
	switch (id) {
	case JsonObjectInID::filePaths:
		return "Filepaths";
	case JsonObjectInID::filePathsInput:
		return "Input";
	case JsonObjectInID::filePatsOutput:
		return "Output";

	case JsonObjectInID::outputReport:
		return "Output report";

	case JsonObjectInID::voxel:
		return "Voxel";
	case JsonObjectInID::voxelSize:
		return "Size";
	case JsonObjectInID::voxelSummarize:
		return "Store values";
	case JsonObjectInID::voxelIntersection:
		return "Logic";

	case JsonObjectInID::IFC:
		return "IFC";
	case JsonObjectInID::IFCRotation:
		return "Rotation";
	case JsonObjectInID::IFCDefaultDiv:
		return "Default div";
	case JsonObjectInID::IFCIgnoreProxy:
		return "Ignore proxy";
	case JsonObjectInID::IFCDivObject:
		return "Div objects";

	case JsonObjectInID::JSON:
		return "JSON";
	case JsonObjectInID::JSONGenInterior:
		return "Generate interior";
	case JsonObjectInID::JSONGenFootPrint:
		return "Generate footprint";
	case JsonObjectInID::JSONGenRoofOutline:
		return "Generate roof outline";
	case JsonObjectInID::JSONFootprintElev:
		return "Footprint elevation";
	case JsonObjectInID::JSONGeoreference:
		return "Georeference";
	
	case JsonObjectInID::lodOutput:
		return "LoD output";

	case JsonObjectInID::generateReport:
		return "Generate report";

	default:
		return "";
	}
}

std::string CJObjectEnum::getString(CJObjectID id)
{
	switch (id) {
	case CJObjectID::metaDataTitle:
		return "Auto export from IfcEnvExtractor";
	case CJObjectID::outerShell:
		return "Outer Shell";
	case CJObjectID::innerShell:
		return "Inner Shell";
	case CJObjectID::v11:
		return "1.1";
	case CJObjectID::ifcDescription:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Description";
	case CJObjectID::ifcObjectType:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "ObjectType";
	case CJObjectID::ifcName:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Name";
	case CJObjectID::ifcLongName:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Long Name";
	default:
		return "";
	}
}

const std::map<errorID, ErrorObject> errorMap::errorCollection = {
	{errorID::failedLoD00, ErrorObject("E0001", "LoD0.0 creation failed")},
	{errorID::failedLoD02, ErrorObject("E0002", "LoD0.2 creation failed")},
	{errorID::failedLoD10, ErrorObject("E0010", "LoD1.0 creation failed")},
	{errorID::failedLoD12, ErrorObject("E0012", "LoD1.2 creation failed")},
	{errorID::failedLoD13, ErrorObject("E0013", "LoD1.3 creation failed")},
	{errorID::failedLoD22, ErrorObject("E0022", "LoD2.2 creation failed")},
	{errorID::failedInit, ErrorObject("S0001", "Basic initialization failed")},
	{errorID::failedFootprint, ErrorObject("S0002", "Footprint creation failed")},
	{errorID::failedStorey, ErrorObject("S0003", "Storey creation failed")},
	{errorID::failedConvert, ErrorObject("S0004", "Failed to convert object")}
};

ErrorObject errorMap::getErrorObject(errorID id)
{
	auto it = errorCollection.find(id);
	if (it != errorCollection.end())
	{
		return it->second;
	}
	return ErrorObject("x0000", "Unknown Error");
}
