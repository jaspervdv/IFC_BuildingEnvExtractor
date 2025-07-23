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
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Parsing file: ";
	case CommunicationStringID::infoParsingFiles:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Parsing file(s): ";
	case CommunicationStringID::infoInternalizingGeo:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Internalizing Geometry of Construction Model";
	case CommunicationStringID::infoCreateSpatialIndex:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Create Spatial Index";
	case CommunicationStringID::infoApplyVoids:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "apply voids";
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

	case CommunicationStringID::infoComputingStoreys02:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.2 Storeys";
	case CommunicationStringID::infoComputingStoreys03:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.3 Storeys";
	case CommunicationStringID::infoComputingLoD00:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.0 Model";
	case CommunicationStringID::infoComputingLoD02:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.2 Model";
	case CommunicationStringID::infoComputingLoD03:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.3 Model";
	case CommunicationStringID::infoComputingLoD04:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 0.4 Model";
	case CommunicationStringID::infoComputingLoD10:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.0 Model";
	case CommunicationStringID::infoComputingLoD12:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.2 Model";
	case CommunicationStringID::infoComputingLoD13:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 1.3 Model";
	case CommunicationStringID::infoComputingLoD22:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 2.2 Model";
	case CommunicationStringID::infoComputingLoDb0:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD b.0 Model";
	case CommunicationStringID::infoComputingLoDc1:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD c.1 Model";
	case CommunicationStringID::infoComputingLoDc2:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD c.2 Model";
	case CommunicationStringID::infoComputingLoDd1:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD d.1 Model";
	case CommunicationStringID::infoComputingLoDd2:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD d.2 Model";
	case CommunicationStringID::infoComputingLoDe0:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD e.0 Model";
	case CommunicationStringID::infoComputingLoDe1:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD e.1 Model";
	case CommunicationStringID::infoComputingLoD32:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 3.2 Model";
	case CommunicationStringID::infoComputingLoD50:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing LoD 5.0 Model";


	case CommunicationStringID::infoComputingInterior:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing Interior";
	case CommunicationStringID::infoComputingExterior:
		return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::info) + "Computing Exterior";

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
	default:
		return "Output string not found";
	}
}

std::string errorWarningStringEnum::getString(ErrorID id, bool withImportance)
{
	switch (id) {
	case ErrorID::errorNoValFilePaths: {
		const std::string coms = "No valid filepath has been supplied";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorUnableToProcessFile: {
		const std::string coms = "Unable to process file(s)";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorNoUnits: {
		const std::string coms = "No unit assignment has been found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::warningMultipleUnits: {
		const std::string coms = "Multiple unit assignments have been found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::errorNoLengthUnit: {
		const std::string coms = "SI unit for length cannot be found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorNoAreaUnit: {
		const std::string coms = "SI unit for area cannot be found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }

	case ErrorID::errorJsonInvalBool: {
		const std::string coms = "JSON file does not contain a valid bool for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalInt: {
		const std::string coms = "JSON file does not contain a valid int for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalNegInt: {
		const std::string coms = "JSON file contains an invalid negative int for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalZeroInt: {
		const std::string coms = "JSON file contains an invalid zero int for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalNum: {
		const std::string coms = "JSON file does not contain a valid numeric value for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalString: {
		const std::string coms = "JSON file does not contain a valid string for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalPath: {
		const std::string coms = "JSON file contains a path to a file with incorrect type for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonNoRealPath: {
		const std::string coms = "JSON file contains an invalid path for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalArray: {
		const std::string coms = "JSON file does not contain a valid array for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalEntry: {
		const std::string coms = "JSON file does not contain a valid value for entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }

	case ErrorID::errorJsonMissingEntry: {
		const std::string coms = "JSON file does not contain required entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }
	case ErrorID::errorJsonInvalidLogic: {
		const std::string coms = "JSON file does not contain valid logic number (2 or 3) entry";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }

	case ErrorID::errorJsonInvalidLod: {
		const std::string coms = "JSON file contains unsupported required LoD";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms + ": "; }
		return coms; }

	case ErrorID::errorJsonMissingLoD: {
		const std::string coms = "No desired LoD output can be found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorJsonNoDivObjects: {
		const std::string coms = "No div objects are selected";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorNoPoints: {
		const std::string coms = "No points could be extracted from the IFC file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorFootprintFailed: {
		const std::string coms = "Footprint extraction failed";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorStoreyFailed: {
		const std::string coms = "storey extraction failed";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }
	case ErrorID::errorLoD02StoreyFailed: {
		const std::string coms = "LoD0.2 Storey shape extraction failed";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::error) + coms; }
		return coms; }

	case ErrorID::warningIfcUnableToParse: {
		const std::string coms = "Unable to parse .ifc file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms + ": "; }
		return coms; }
	case ErrorID::warningIfcNotValid: {
		const std::string coms = "No valid ifc scheme found in file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms + ": "; }
		return coms; }
	case ErrorID::warningIfcNoSchema: {
		const std::string coms = "No scheme found in file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms + ": "; }
		return coms; }
	case ErrorID::warningIfcIncomp: {
		const std::string coms = "Incompatible scheme found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms + ": "; }
		return coms; }
	case ErrorID::warningIfcNoSlab: {
		const std::string coms = "No slab objects were found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcMultipleProjections: {
		const std::string coms = "Multiple map projections detected";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcNoVolumeUnit: {
		const std::string coms = "SI unit for volume cannot be found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcDubSites: {
		const std::string coms = "More than one Site Element found, site export terminated";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcNoSites: {
		const std::string coms = "No Geographic or Site Element was found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcSiteReconstructionFailed: {
		const std::string coms = "No site could be reconstructed";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcNoRoomObjects: {
		const std::string coms = "No room objects present in model, generic semantic data is created";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcMultipleUniqueObjects: {
		const std::string coms = "Multiple assumed unique objects in file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcNoObjectName: {
		const std::string coms = "Object name could not be found in file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcNoObjectNameLong: {
		const std::string coms = "Long Object name could not be found in file";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }	
	case ErrorID::warningIfcObjectDifferentName: {
		const std::string coms = "Objects have different names in different files, name of first object is taken";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningIfcMissingGeoreference: {
		const std::string coms = "Data required to georeferencing is missing";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }


	case ErrorID::warningIssueencountered: {
		const std::string coms = "Encountered an issue";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningNoSolid: {
		const std::string coms = "Not all shapes could be converted to solids, output might be incorrect or inaccurate";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningUnableToMesh: {
		const std::string coms = "Not all shapes could be completely meshed, some surfaces are ignored";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningUnableToSimplefy: {
		const std::string coms = "Not all shapes could be simplefied, unsimplefied shape is stored";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }	
	case ErrorID::warningUnableToExtrude: {
		const std::string coms = "Not all surfaces can be extruded into a prism, most likely caused by non straight edges in the input model";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }	
	case ErrorID::warningNoRoofOutline: {
		const std::string coms = "No roofoutline surface has been found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }	
	case ErrorID::warningNoFootprint: {
		const std::string coms = "No footprint surface has been found";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningNonLinearEdges: {
		const std::string coms = "Face partially bound by non-linear edge/wire";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningSimplefication: {
		const std::string coms = "Simple geometry is used, this can cause issues with windows and door detection";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningInputIncFootprintElev: {
		const std::string coms = "Footprint elevation falls outside of the bounds of the model, lower bounds z value is used";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	

	case ErrorID::warningFailedObjectSimplefication: {
		const std::string coms = "Simplefication of complex object has failed";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }
	case ErrorID::warningFailedObjectConversion: {
		const std::string coms = "Unable to convert object shape";
		if (withImportance) { return CommunicationStringImportanceEnum::getString(CommunicationStringImportanceID::warning) + coms; }
		return coms; }

	default:
		return "Output string not found";
	}
}


std::string fileExtensionEnum::getString(fileExtensionID id)
{
	switch (id) {
	case fileExtensionID::JSON:
		return ".json";
	case fileExtensionID::OBJ:
		return ".obj";
	case fileExtensionID::STEP:
		return ".step";
	case fileExtensionID::dash:
		return "_";

	case fileExtensionID::interior:
		return "i";
	case fileExtensionID::exterior:
		return "e";

	case fileExtensionID::OBJLoD00:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD00" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD02:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD02" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD03:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD03" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD04:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD04" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD10:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD10" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD12:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD12" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD13:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD13" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD22:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD22" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoDb:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDb" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoDc1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDc1" + getString(fileExtensionID::OBJ);	
	case fileExtensionID::OBJLoDd1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDd1" + getString(fileExtensionID::OBJ);	
	case fileExtensionID::OBJLoDe1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDe1" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD32:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD32" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD50:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD50" + getString(fileExtensionID::OBJ);

	case fileExtensionID::STEPLoD00:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD00" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD02:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD02" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD03:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD03" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD04:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD04" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD10:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD10" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD12:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD12" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD13:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD13" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD22:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD22" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoDb:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDb" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoDc1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDc1" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoDd1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDd1" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoDe1:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoDe1" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD32:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD32" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD50:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::exterior) + "LoD50" + getString(fileExtensionID::STEP);

	case fileExtensionID::OBJLoD02Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD02" + getString(fileExtensionID::OBJ);
	case fileExtensionID::OBJLoD03Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD03" + getString(fileExtensionID::OBJ);

	case fileExtensionID::STEPLoD02Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD02" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD03Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD03" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD12Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD12" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD32Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD32" + getString(fileExtensionID::STEP);
	case fileExtensionID::STEPLoD50Interior:
		return getString(fileExtensionID::dash) + getString(fileExtensionID::interior) + "LoD50" + getString(fileExtensionID::STEP);

	default:
		return "";
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
	case JsonObjectInID::filePathOutput:
		return "Output";
	case JsonObjectInID::filePathReport:
		return "Report";

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
	case JsonObjectInID::IFCRotationAuto:
		return "Rotation auto";
	case JsonObjectInID::IFCRotationAngle:
		return "Rotation angle";
	case JsonObjectInID::IFCDefaultDiv:
		return "Default div";
	case JsonObjectInID::IFCIgnoreProxy:
		return "Ignore proxy";
	case JsonObjectInID::IFCDivObject:
		return "Div objects";
	case JsonObjectInID::IFCsimplefyGeo:
		return "Simplify geometry";
	case JsonObjectInID::IFCapplyVoids:
		return "Apply voids";
	case JsonObjectInID::IFCignoreSimple:
		return "Ignore simplification";

	case JsonObjectInID::JSON:
		return "JSON";
	case JsonObjectInID::JSONGenInterior:
		return "Generate interior";
	case JsonObjectInID::JSONGenExterior:
		return "Generate exterior";	
	case JsonObjectInID::JSONGenSite:
		return "Generate site";
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
	case JsonObjectInID::JSONMergeSemantics:
		return "Merge semantic objects";

	case JsonObjectInID::outputFormat:
		return "Output format";
	case JsonObjectInID::outputFormatJSON:
		return "JSON file";
	case JsonObjectInID::outputFormatOBJ:
		return "OBJ file";
	case JsonObjectInID::outputFormatSTEP:
		return "STEP file";
	
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
	case CJObjectID::CJTypeFloor:
		return "FloorSurface";
	case CJObjectID::CJTypeOuterFloor:
		return "OuterFloorSurface";
	case CJObjectID::CJTypeRoofSurface:
		return "RoofSurface";
	case CJObjectID::CJTypeFloorSurface:
		return "FloorSurface";
	case CJObjectID::CJTypeGroundSurface:
		return "GroundSurface";
	case CJObjectID::CJTypeWallSurface:
		return "WallSurface";	
	case CJObjectID::CJTypeInteriorWallSurface:
		return "InteriorWallSurface";
	case CJObjectID::CJTypeSiteObject:
		return "Site";
	case CJObjectID::CJTypeWindow:
		return "Window";
	case CJObjectID::CJTypeDoor:
		return "Door";
	case CJObjectID::CJTypeNone:
		return "+None";
	case CJObjectID::CJTTypeCeilingSurface:
		return "CeilingSurface";
	case CJObjectID::CJTTypeOuterCeilingSurface:
		return "OuterCeilingSurface";
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

	case CJObjectID::EnvLoDfloorArea:
		return sourceIdentifierEnum::getString(sourceIdentifierID::envExtractor) + "floor area LoD";

	case CJObjectID::v11:
		return "1.1";
	case CJObjectID::v20:
		return "2.0";

	case CJObjectID::jsonUom:
		return "uom";
	case CJObjectID::jsonValue:
		return "value";

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
