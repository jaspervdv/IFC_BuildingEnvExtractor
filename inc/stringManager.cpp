#include "stringManager.h"

#include <string>
#include <map>

#include <nlohmann/json.hpp>

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
	case CommunicationStringImportanceID::seperator:
		return "=============================================================";
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
	case UnitStringID::sqrMeter:
		return "m^2";
	case UnitStringID::cubMeter:
		return "m^3";
	case UnitStringID::meterFull:
		return "meter";
	case UnitStringID::millimeter:
		return "mm";
	case UnitStringID::millimeterFull:
		return "millimeter";
	case UnitStringID::footFull:
		return "foot";
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
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Found units:";
	case CommunicationStringID::infoDefaultVolumeUnit:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "SI unit for volume is set to cubic metre";

	case CommunicationStringID::infoPreProcessing:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Pre proccessing";
	case CommunicationStringID::infoCoarseFiltering:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Coarse filtering of roofing structures";
	case CommunicationStringID::infoReduceSurfaces:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Reduce surfaces";
	case CommunicationStringID::infoFineFiltering:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Fine filtering of roofing structures";
	case CommunicationStringID::infoRoofOutlineConstruction:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Construct roof outlines";
	case CommunicationStringID::infoRoofStructureSorting:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Sort roofing structures";
	case CommunicationStringID::infoRoofStructureMerging:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "merge roofing structures";
	case CommunicationStringID::infoCoasreFootFiltering:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Corse filtering footprint at z = ";

	case CommunicationStringID::infoNoVoxelizationReq:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "No voxelization required";
	case CommunicationStringID::infoNocompleteVoxelizationReq:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "No complete voxelization required";
	case CommunicationStringID::infoInterioSpacesGrowing:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Interior spaces growing";
	case CommunicationStringID::infoPairVoxels:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Pair voxels";

	case CommunicationStringID::infoComputingStoreys:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Storey extraction";
	case CommunicationStringID::infoComputingLoD00:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.0 Model";
	case CommunicationStringID::infoComputingLoD02:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.2 Model";
	case CommunicationStringID::infoComputingLoD03:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.3 Model";
	case CommunicationStringID::infoComputingLoD10:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.0 Model";
	case CommunicationStringID::infoComputingLoD12:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.2 Model";
	case CommunicationStringID::infoComputingLoD13:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.3 Model";
	case CommunicationStringID::infoComputingLoD22:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 2.2 Model";
	case CommunicationStringID::infoComputingLoD32:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 3.2 Model";
	case CommunicationStringID::infoComputingLoD50:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 5.0 Model";
	case CommunicationStringID::infoComputingLoD50Rooms:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 5.0 Rooms";
	case CommunicationStringID::infoExtractingSite:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Extracting Site Data";
	case CommunicationStringID::infoPopulateGrid:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Populate Grid";
	case CommunicationStringID::infoExteriorSpaceGrowing:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Exterior space growing";


	case CommunicationStringID::indentValidIFCFound:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Valid IFC file found";
	case CommunicationStringID::indentcompIFCFound:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Compatible scheme found: ";
	case CommunicationStringID::indentSuccesFinished:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Successfully finished in: ";
	case CommunicationStringID::indentUnsuccesful:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Unsuccessful";
	case CommunicationStringID::indentStoreyAtZ:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Floorlevel at z = ";
	case CommunicationStringID::indentExteriorSpaceGrown:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Exterior space succesfully grown";
	case CommunicationStringID::indentNoExteriorSpace:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "No exterior space has been found";
	case CommunicationStringID::indentInteriorSpaceGrown:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Interior space succesfully grown";
	case CommunicationStringID::indentPairedVoxels:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::indent) + "Voxel pairing succesful";

	case CommunicationStringID::errorNoValFilePaths:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No valid filepath has been supplied";
	case CommunicationStringID::errorUnableToProcessFile:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "Unable to process file(s)";
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
	case CommunicationStringID::errorJSONThreadNum:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "JSON file does not contain a valid thread max count entry";
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
	case CommunicationStringID::errorJSONMissingLoD:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No desired LoD output can be found";
	case CommunicationStringID::errorJSONNoDivObjects:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No div objects are selected";
	case CommunicationStringID::errorNoPoints:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "No points could be extracted from the IFC file";
	case CommunicationStringID::errorFootprintFailed:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "Footprint extraction failed";
	case CommunicationStringID::errorStoreyFailed:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "storey extraction failed";
	case CommunicationStringID::errorLoD02StoreyFailed:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + "LoD0.2 Storey shape extraction failed";


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
	case CommunicationStringID::warningNoSolid:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Not all shapes could be converted to solids, output might be incorrect or inaccurate";
	case CommunicationStringID::warningNoSolidLoD50:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "Unable to create solid shape, multisurface stored";	
	case CommunicationStringID::warningDubSites:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "More than one Site Element found, site export terminated";	
	case CommunicationStringID::warningNoSites:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "No Geographic or Site Element was found";	
	case CommunicationStringID::warningSiteReconstructionFailed:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "No site could be reconstructed";
	case CommunicationStringID::warningNoIfcRoomObjects:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + "No room objects present in model, generic semantic data is created";
	default:
		return "Output string not found";
	}
}

std::string sourceIdentifierEnum::getString(sourceIdentifierID id)
{
	switch (id) {
	case sourceIdentifierID::envExtractor:
		return "Env_ex ";
	case sourceIdentifierID::envExtractorVApprox:
		return "Env_vox ";
	case sourceIdentifierID::voxel:
		return "V_ex ";
	case sourceIdentifierID::ifc:
		return "IFC ";
	case sourceIdentifierID::ifcProp:
		return "IFC property";
	default:
		return "";
	}
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
	case JsonObjectInID::JSONFootprintBShape:
		return "Footprint based";
	case JsonObjectInID::JSONSecOffset:
		return "Horizontal section offset";
	case JsonObjectInID::JSONGeoreference:
		return "Georeference";
	
	case JsonObjectInID::lodOutput:
		return "LoD output";

	case JsonObjectInID::generateReport:
		return "Generate report";
	case JsonObjectInID::maxThread:
		return "Threads";
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

	case CJObjectID::CJType:
		return "type";
	case CJObjectID::CJTypeStorey:
		return "BuildingStorey";
	case CJObjectID::CJTypeRoofSurface:
		return "RoofSurface";
	case CJObjectID::CJTypeGroundSurface:
		return "GroundSurface";
	case CJObjectID::CJTypeWallSurface:
		return "WallSurface";	
	case CJObjectID::CJTypeSiteObject:
		return "Site";
	case CJObjectID::CJTypeWindow:
		return "Window";
	case CJObjectID::CJTypeDoor:
		return "Door";
	case CJObjectID::CJAttHasWindow:
		return "+hasWindows";

	case CJObjectID::True:
		return "True";
	case CJObjectID::False:
		return "False";

	case CJObjectID::voxelApproxShellVolume:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "shell volume";
	case CJObjectID::voxelApproxBuildingShellVolume:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "basement shell volume";
	case CJObjectID::voxelApproxBasementShellVolume:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "building shell volume";
	case CJObjectID::voxelApproxRoomVolumeTotal:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "total room volume";
	case CJObjectID::voxelApproxRoomArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "room area";
	case CJObjectID::voxelApproxShellArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "shell area";
	case CJObjectID::voxelApproxBasementShellArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "basement shell area";
	case CJObjectID::voxelApproxBuildingShellArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "building shell area";
	case CJObjectID::voxelApproxFootprintArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "footprint shell area";
	case CJObjectID::voxelApproxFaceadeOpeningArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractorVApprox) + "facade opening area";

	case CJObjectID::EnvVoxelSize:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "voxel size";
	case CJObjectID::EnvVoxelAnchor:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "voxelGrid anchor";
	case CJObjectID::EnvVoxelRotation:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "voxelGrid rotation";

	case CJObjectID::v11:
		return "1.1";
	case CJObjectID::v20:
		return "2.0";

	case CJObjectID::jsonUom:
		return "uom";
	case CJObjectID::jsonValue:
		return "vale";

	case CJObjectID::ifcDescription:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Description";
	case CJObjectID::ifcObjectType:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "ObjectType";
	case CJObjectID::ifcName:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Name";
	case CJObjectID::ifcLongName:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Long Name";
	case CJObjectID::ifcElevation:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Elevation";
	case CJObjectID::ifcGuid:
		return sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + "Guid";
	default:
		return "";
	}
}
