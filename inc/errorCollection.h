#include <map>
#include <nlohmann/json.hpp>
#include <mutex>

#ifndef ERRORCOLLECTION_ERRORCOLLECTION_H
#define ERRORCOLLECTION_ERRORCOLLECTION_H
enum class ErrorID {
	errorNoValFilePaths,
	errorUnableToProcessFile,
	errorNoUnits,
	errorMultipleUnits,
	errorNoLengthUnit,
	errorNoAreaUnit,

	errorJsonInvalBool,
	errorJsonInvalInt,
	errorJsonInvalNegInt,
	errorJsonInvalZeroInt,
	errorJsonInvalNum,
	errorJsonInvalString,
	errorJsonInvalPath,
	errorJsonNoRealPath,
	errorJsonInvalArray,
	errorJsonInvalEntry,

	errorJsonMissingEntry,
	errorJsonInvalidLogic,

	errorJsonInvalidLod,

	errorJsonMissingLoD,
	errorJsonNoDivObjects,

	errorNoPoints,
	errorFootprintFailed,
	errorStoreyFailed,
	errorLoD02StoreyFailed,
	warningFailedObjectSimplefication,
	warningFailedObjectConversion,
	errorFailedInit,

	warningIfcUnableToParse,
	warningIfcNotValid,
	warningIfcNoSchema,
	warningIfcIncomp,
	warningIfcNoSlab,
	warningIfcMultipleProjections,
	warningIfcNoVolumeUnit,
	warningIfcDubSites,
	warningIfcNoSites,
	warningIfcSiteReconstructionFailed,
	warningIfcNoRoomObjects,
	warningIfcMultipleUniqueObjects,
	warningIfcNoObjectName,
	warningIfcNoObjectNameLong,
	warningIfcObjectDifferentName,
	warningIfcMissingGeoreference,

	warningIssueencountered,
	warningNoSolid,
	warningUnableToMesh,
	warningUnableToSimplefy,
	warningNoRoofOutline,
	warningNoFootprint,
	warningNonLinearEdges,

	warningSimplefication,

	warningInputIncFootprintElev,

	failedLoD00, // LoD0.0 creation failed
	failedLoD02, // LoD0.2 creation failed
	failedLoD03, // LoD0.3 creation failed
	failedLoD04, // LoD0.4 creation failed
	failedLoD10, // LoD1.0 creation failed
	failedLoD12, // LoD1.2 creation failed
	failedLoD13, // LoD1.3 creation failed
	failedLoD22, // LoD2.2 creation failed
	failedLoD30, // LoD3.0 creation failed
	failedLoD32, // LoD3.2 creation failed
	failedLoD50, // voxel creation failed

	propertyNotImplemented

};

struct ErrorObject {
	std::string errorCode_;
	std::string errorDescript_;
	std::vector<std::string> occuringObjectList_;

	ErrorObject() {};

	ErrorObject(
		const std::string& errorCode,
		const std::string& errorDescript
	);

	ErrorObject(
		const std::string& errorCode,
		const std::string& errorDescript,
		const std::string& occuringObb
	);

	ErrorObject(
		const std::string& errorCode,
		const std::string& errorDescript,
		const std::vector<std::string>& occuringObbList
	);

	nlohmann::json toJson();

	void addOccuringObject(const std::string& obb);
};

struct ErrorCollection {
private:
	//Errors and issues present in this process
	std::map<ErrorID, ErrorObject> errorCollection_; 
	// collection of all the possible errors and issues
	std::map<ErrorID, ErrorObject> errorMap_;

	//Prevents datarace when writing to the instance
	std::mutex dataMutex_;
	
	explicit ErrorCollection();

public:
	static ErrorCollection& getInstance() {
		static ErrorCollection instance;
		return instance;
	}

	void addError(ErrorID id, const std::string& objectName = "");
	void addError(ErrorID id, const std::vector<std::string>& objectNameList);
	void removeError(ErrorID id);

	nlohmann::json toJson();
};
#endif // ERRORCOLLECTION_ERRORCOLLECTION_H