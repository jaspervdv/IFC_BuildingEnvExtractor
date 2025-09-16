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
	infoIgnoreVoids,
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

	infoComputingStoreys02,
	infoComputingStoreys03,
	infoComputingLoD00,
	infoComputingLoD02,
	infoComputingLoD03,
	infoComputingLoD04,
	infoComputingLoD10,
	infoComputingLoD12,
	infoComputingLoD13,
	infoComputingLoD22,
	infoComputingLoDb0,
	infoComputingLoDc1,
	infoComputingLoDc2,
	infoComputingLoDd1,
	infoComputingLoDd2,
	infoComputingLoDe0,
	infoComputingLoDe1,
	infoComputingLoD32,
	infoComputingLoD50,

	infoComputingInterior,
	infoComputingExterior,

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
	indentPairedVoxels
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

/// collects all the file extension of the output
enum class fileExtensionID {
	JSON,
	STEP,
	OBJ,
	dash,

	interior,
	exterior,

	OBJLoD00,
	OBJLoD02,
	OBJLoD03,
	OBJLoD04,
	OBJLoD10,
	OBJLoD12,
	OBJLoD13,
	OBJLoD22,
	OBJLoDb,
	OBJLoDc1,
	OBJLoDd1,
	OBJLoDe0,
	OBJLoDe1,
	OBJLoD32,
	OBJLoD50,

	STEPLoD00,
	STEPLoD02,
	STEPLoD03,
	STEPLoD04,
	STEPLoD10,
	STEPLoD12,
	STEPLoD13,
	STEPLoD22,
	STEPLoDb,
	STEPLoDc1,
	STEPLoDd1,
	STEPLoDe0,
	STEPLoDe1,
	STEPLoD32,
	STEPLoD50,

	OBJLoD02Interior,
	OBJLoD03Interior,

	STEPLoD02Interior,
	STEPLoD03Interior,
	STEPLoD12Interior,
	STEPLoD22Interior,
	STEPLoD32Interior,
	STEPLoD50Interior,
};


class fileExtensionEnum {
public:
	static std::string getString(fileExtensionID id);
};

// collects all the JSON object of the config files
enum class JsonObjectInID {
	outputReport,
	filePaths,
	filePathsInput,
	filePathOutput,
	filePathReport,

	tolerances,
	tolerancesSpatial,
	tolerancesAngular,
	tolerancesArea,

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
	IFCignoreVoids,
	IFCignoreSimple,
	IFCCorrentPlacement,

	JSON,
	JSONFootprintElev,
	JSONFootprintBShape,
	JSONSecOffset,
	JSONGenFootPrint,
	JSONGenRoofOutline,
	JSONGenInterior,
	JSONGenExterior,
	JSONGenSite,
	JSONGeoreference,
	JSONMergeSemantics,

	outputFormat,
	outputFormatJSON,
	outputFormatOBJ,
	outputFormatSTEP,

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
	CJTypeFloor,
	CJTypeOuterFloor,
	CJTypeRoofSurface,
	CJTypeGroundSurface,
	CJTypeFloorSurface,
	CJTypeWallSurface,
	CJTypeInteriorWallSurface,
	CJTypeSiteObject,
	CJTypeWindow,
	CJTypeDoor,
	CJTypeNone,
	CJTTypeCeilingSurface, 
	CJTTypeOuterCeilingSurface,

	CJTTypeProjectedRoofOutline,

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
	EnvLoDfloorArea,

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