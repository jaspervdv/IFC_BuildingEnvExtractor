#include <map>
#include <nlohmann/json.hpp>

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
	nobuildingName, // cant find building name
	nobuildingNameLong, // cant find long building name
	multipleBuildingObjects, // Multiple building objects found
	noProjectName,
	multipleProjectNames,
	missingProptery // Property not implelemted in tool
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
	std::map<errorID, ErrorObject> errorCollection_; 
	// collection of all the possible errors and issues
	std::map<errorID, ErrorObject> errorMap_;
	
	explicit ErrorCollection();

public:
	static ErrorCollection& getInstance() {
		static ErrorCollection instance;
		return instance;
	}

	void addError(errorID id, const std::string& objectName = "");
	void addError(errorID id, const std::vector<std::string>& objectNameList);
	void removeError(errorID id);

	nlohmann::json toJson();
};