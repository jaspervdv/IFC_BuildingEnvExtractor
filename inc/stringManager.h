#include <string>
#include <map>

#include <nlohmann/json.hpp>

#ifndef STRINGMANAGER_H
#define STRINGMANAGER_H

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
	envExtractorVApprox,
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

	infoPreProcessing,
	infoCoarseFiltering,
	infoReduceSurfaces,
	infoFineFiltering,
	infoRoofOutlineConstruction,
	infoRoofStructureSorting,
	infoRoofStructureMerging,
	infoCoasreFootFiltering,

	infoNoVoxelizationReq,
	infoNocompleteVoxelizationReq,
	infoInterioSpacesGrowing,
	infoPairVoxels,

	infoComputingStoreys,
	infoComputingLoD00,
	infoComputingLoD02,
	infoComputingLoD10,
	infoComputingLoD12,
	infoComputingLoD13,
	infoComputingLoD22,
	infoComputingLoD32,
	infoComputingLoD50,
	infoComputingLoD50Rooms,
	infoExtractingSite,
	infoPopulateGrid,
	infoExteriorSpaceGrowing,
	indentNoExteriorSpace,

	indentValidIFCFound,
	indentcompIFCFound,
	indentSuccesFinished,
	indentUnsuccesful,
	indentStoreyAtZ,
	indentExteriorSpaceGrown,
	indentInteriorSpaceGrown,
	indentPairedVoxels,

	errorNoValFilePaths,
	errorUnableToProcessFile,
	errorNoUnits,
	errorMultipleUnits,
	errorNoLengthUnit,
	errorNoAreaUnit,

	errorJSONReportPath,
	errorJSONThreadNum,
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
	errorNoPoints,
	errorFootprintFailed,
	errorStoreyFailed,
	errorLoD02StoreyFailed,

	warningUnableToParseIFC,
	warningNoValidIFC,
	warningIncompIFC,
	warningNoSlab,
	warningMultipleProjections,
	warningNoVolumeUnit,
	warmingIssueencountered,
	warningNoSolid,
	warningNoSolidLoD50,
	warningDubSites,
	warningNoSites,
	warningSiteReconstructionFailed,
	warningNoIfcRoomObjects
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

	lodOutput,

	generateReport,
	maxThread
};

class JsonObjectInEnum {
public:
	static std::string getString(JsonObjectInID id);
};

enum class CJObjectID {
	metaDataTitle,
	outerShell,
	innerShell,
	
	CJType,
	CJTypeStorey,
	CJTypeRoofSurface,
	CJTypeGroundSurface,
	CJTypeWallSurface,
	CJTypeSiteObject,
	CJTypeWindow,
	CJTypeDoor,

	voxelApproxShellVolume,
	voxelApproxBasementShellVolume,
	voxelApproxBuildingShellVolume,
	voxelApproxRoomVolumeTotal,
	voxelApproxRoomArea,
	voxelApproxShellArea,
	voxelApproxBasementShellArea,
	voxelApproxBuildingShellArea,
	voxelApproxFootprintArea,
	voxelApproxFaceadeOpeningArea,

	EnvVoxelSize,
	EnvVoxelAnchor,
	EnvVoxelRotation,

	v11,
	v20,

	ifcDescription,
	ifcObjectType,
	ifcName,
	ifcLongName,
	ifcElevation,
	ifcGuid
};

class CJObjectEnum {
public:
	static std::string getString(CJObjectID id);
};

#endif // STRINGMANAGER_H