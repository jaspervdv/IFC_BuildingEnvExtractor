#include "helper.h"
#include "cjCreator.h"
#include "settingsCollection.h"
#include "stringManager.h"

#include <unordered_set>
#include <string>
#include <vector>

/// <summary>
/// Manages and facilitates the communication between the user and the rest of the application
/// </summary>
class IOManager {
private:
	std::unique_ptr<DataManager> internalDataManager_;

	std::shared_ptr<CJT::CityCollection> cityCollection_;

	// time summary for the output
	long long timeInternalizing_ = 0;
	long long timeVoxel_ = 0;
	long long timeLoD00_ = 0;
	long long timeLoD02_ = 0;
	long long timeLoD03_ = 0;
	long long timeLoD04_ = 0;
	long long timeLoD10_ = 0;
	long long timeLoD12_ = 0;
	long long timeLoD13_ = 0;
	long long timeLoD22_ = 0;
	long long timeLoD32_ = 0;
	long long timeLoDb0_ = 0;
	long long timeLoDc1_ = 0;
	long long timeLoDc2_ = 0;
	long long timeLoDd1_ = 0;
	long long timeLoDd2_ = 0;
	long long timeV_ = 0;

	// 1 is all the geocreation functions were succesfull
	bool succesfullExit_ = 1;

	// checks if the string has the extension that is supplied
	bool hasExtension(const std::string& string, const std::string& ext);
	//  checks if the string supplied is a path that exists on the system
	bool isValidPath(const std::string& path);
	// get target path from user when program is started with no args
	std::string getTargetPath();
	// attempts to get the settings from json file
	bool getJSONValues(const std::string& inputPath);
	
	// console outputs the settings that are utilized
	void printSummary();
	// console output the encountered errors
	void printErrors();
	// outputs yes or no based on a input bool
	std::string boolToString(const bool boolValue);

	// returns a single string representing the enables LoD output
	std::string getLoDEnabled();

	// returns a json object that is populated with the settings in the settingsobject
	nlohmann::json settingsToJSON();
	/// internalize and index geometry
	void internalizeGeo();

	// sets up metadata and transformation object for CJT and adds them to the collection
	void setMetaData(std::shared_ptr<CJT::CityCollection> collection);
	// set the relationships between CJ objects, copy the default attributes from the IFC file and adds them to respective cityobject
	void setDefaultSemantic(CJT::CityObject& cityBuildingObject, CJT::CityObject& cityOuterShellObject, CJT::CityObject& cityInnerShellObject);
	// set, compute additional attributes and adds them to respective cityobject
	void setComputedSemantic(CJGeoCreator* geoCreator, CJT::CityObject& cityBuildingObject, CJT::CityObject& cityOuterShellObject, CJT::CityObject& cityInnerShellObject);
	/// compute the storeys that are equal or above ground level and the storeys that are below ground level
	void ComputeStoreysAboveGround(int* storeysAboveGround, int* storeysBelowGround);

	/// process, collect errors and clock all the geocreating functions
	void processExternalLoD(
		CJGeoCreator* geoCreator, 
		CJT::CityObject& cityOuterShellObject,
		CJT::Kernel* kernel);
	/// process, collect errors and clock the geocreating functions
	void processExternalLoD(
		const std::function<std::vector<CJT::GeoObject>()>& lodCreationFunc,
		CJT::CityObject& cityOuterShellObject,
		ErrorID errorID,
		long long& timeRecord);
	/// process, collect errors and clock the interior geocreating functions
	void processInteriorLod(
		CJGeoCreator* geoCreator, 
		std::shared_ptr<CJT::CityCollection> collection, 
		CJT::CityObject* cityInnerShellObject, 
		CJT::Kernel* kernel);
	/// process, collect errors and clock the site geometry functions
	void processSitelod(CJGeoCreator* geoCreator, std::shared_ptr<CJT::CityCollection> collection, CJT::CityObject* cityBuildingObject, CJT::Kernel* kernel);

public:

	bool init(const std::vector<std::string>& inputPathList);

	bool run();

	bool write(bool reportOnly = false);

	std::unordered_set<std::string> divObjects_ = { // Only used for output purposes
		"IFCSLAB",
		"IFCROOF",
		"IFCWALL",
		"IFCWALLSTANDARDCASE",
		"IFCCOVERING",
		"IFCCOLUMN",
		"IFCBEAM",
		"IFCCURTAINWALL",
		"IFCPLATE",
		"IFCMEMBER",
		"IFCDOOR",
		"IFCWINDOW"
	};
	//TODO: move this data somewhere central
};
