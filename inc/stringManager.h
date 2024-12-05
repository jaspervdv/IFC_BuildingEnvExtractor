#include <string>
#include <map>

#include "errorCollection.h"

#include <nlohmann/json.hpp>

#ifndef STRINGMANAGER_H
#define STRINGMANAGER_H

// collects all the importance elements in communication with the user
enum class CommunicationStringImportanceID
{
	indent,
	info,
	warning,
	error,
	seperator
};

class CommunicationStringImportanceEnum {
public:
	static std::string getString(CommunicationStringImportanceID id);
};

enum class sourceIdentifierID {
	envExtractor,
	envExtractorVApprox,
	voxel,
	ifc,
	ifcProp
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
	sqrMeter,
	cubMeter,
	meterFull,
	millimeter,
	millimeterFull,
	footFull
};

class UnitStringEnum {
public:
	static std::string getString(UnitStringID id);
};

// collects all the normal cout communication with the user
enum class CommunicationStringID {
	infoJsonRequest,
	infoNoFilePath,
	infoNoValFilePath,
	infoParsingFile,
	infoParsingFiles,
	infoInternalizingGeo,
	infoCreateSpatialIndex,
	infoApplyVoids,
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
	infoComputingLoD03,
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
};

class CommunicationStringEnum {
public:
	static std::string getString(CommunicationStringID id);
};

// collects all the errors and warnings for both cout and error object description
class errorWarningStringEnum {
public:
	static std::string getString(ErrorID id, bool withImportance = true);
};

// collects all the JSON object of the config files
enum class JsonObjectInID {
	outputReport,
	filePaths,
	filePathsInput,
	filePathOutput,
	filePathReport,

	voxel,
	voxelSize,
	voxelSummarize,
	voxelIntersection,

	IFC,
	IFCRotationAuto,
	IFCRotationAngle,
	IFCDefaultDiv,
	IFCIgnoreProxy,
	IFCDivObject,
	IFCsimplefyGeo,

	JSON,
	JSONFootprintElev,
	JSONFootprintBShape,
	JSONSecOffset,
	JSONGenFootPrint,
	JSONGenRoofOutline,
	JSONGenInterior,
	JSONGeoreference,
	JSONMergeSemantics,

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
	CJTypeFloorSurface,
	CJTypeWallSurface,
	CJTypeInteriorWallSurface,
	CJTypeSiteObject,
	CJTypeWindow,
	CJTypeDoor,
	CJTTypeCeilingSurface,

	CJAttHasWindow,

	True,
	False,

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

	jsonUom,
	jsonValue,

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