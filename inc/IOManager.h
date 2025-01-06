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
	long long timeLoD10_ = 0;
	long long timeLoD12_ = 0;
	long long timeLoD13_ = 0;
	long long timeLoD22_ = 0;
	long long timeLoD32_ = 0;
	long long timeV_ = 0;

	// simple assist functions
	// outputs yes or no based on a input bool
	std::string boolToString(const bool boolValue);

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

	// returns a single string representing the enables LoD output
	std::string getLoDEnabled();

	// returns a json object that is populated with the settings in the settingsobject
	nlohmann::json settingsToJSON();

	void internalizeGeo();

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
