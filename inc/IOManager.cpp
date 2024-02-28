#include "helper.h"
#include "ioManager.h"
#include "cjCreator.h"

#include <unordered_set>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <direct.h>

#include <boost/filesystem.hpp>

bool IOManager::yesNoQuestion()
{
	std::string cont = "";

	while (true)
	{
		std::getline(std::cin, cont);
		if (cont == "Y" || cont == "y") { return true; }
		if (cont == "N" || cont == "n") { return false; }
	}
}

int IOManager::numQuestion(int n, bool lower)
{
	while (true)
	{
		bool validInput = true;
		std::cout << "Num: ";

		std::string stringNum = "";
		std::getline(std::cin, stringNum);

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (!std::isdigit(stringNum[i]))
			{
				validInput = false;
			}
		}

		if (validInput)
		{
			int intNum = std::stoi(stringNum) - 1;
			if (!lower)
			{
				if (n >= intNum + 1) {
					return intNum;
				}
			}
			else if (lower)
			{
				if (n >= intNum + 1 && intNum >= 0) {
					return intNum;
				}
			}
		}
		std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
	}
}


bool IOManager::getTargetPathList()
{
	std::cout << "Enter filepath of the JSON or IFC input file" << std::endl;
	std::cout << "[INFO] If multifile seperate by enter" << std::endl;
	std::cout << "[INFO] Finish by empty line + enter" << std::endl;

	while (true)
	{
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		if (singlepath.size() == 0 && inputPathList_.size() == 0)
		{
			std::cout << "[INFO] No filepath has been supplied" << std::endl;
			std::cout << "Enter filepath of the JSON or IFC input file (if multiplefile sperate path with enter):" << std::endl;
			continue;
		}
		else if (singlepath.size() == 0)
		{
			break;
		}
		if (!hasExtension(singlepath, "ifc") && !hasExtension(singlepath, "json"))
		{
			std::cout << "[INFO] No valid filepath has been supplied" << std::endl;
			std::cout << "Enter filepath of the JSON or IFC input file (if multiplefile sperate path with enter):" << std::endl;
			continue;
		}

		if (!isValidPath(singlepath))
		{
			std::cout << "[INFO] No valid filepath has been supplied" << std::endl;
			std::cout << "Enter filepath of the JSON or IFC input file (if multiplefile sperate path with enter):" << std::endl;
			continue;
		}

		if (hasExtension(singlepath, "json"))
		{
			isJsonInput_ = true;
			if (inputPathList_.size() > 1)
			{
				return false;
			}
		}
		inputPathList_.emplace_back(singlepath);
	}

	if (inputPathList_.size() > 0) { return true; }
	return false;
}


bool IOManager::getOutputPathList() {

	while (true)
	{
		std::cout << "Enter target filepath of the CityJSON file" << std::endl;
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		if (!hasExtension(singlepath, "json"))
		{
			std::cout << "[WARNING] not a valid CityJSON file path" << std::endl;
			std::cout << "[INFO] CityJSON files have a .json or .city.json extension" << std::endl;
			continue;
		}

		boost::filesystem::path targetFilePath(singlepath);
		boost::filesystem::path targetFolderPath = targetFilePath.parent_path();

		if (!boost::filesystem::exists(targetFolderPath))
		{
			std::cout << "[INFO] Folder path " << targetFolderPath.string() << " does not exist." << std::endl;
			std::cout << "do you want to create a new folder at: " << targetFolderPath.string() << std::endl;
			std::cout <<"(Y/N) " << std::endl;

			if (yesNoQuestion())
			{
				std::cout << "[INFO] Export folder is created at: " << targetFolderPath.string() << std::endl;
				if (boost::filesystem::create_directory(targetFilePath)) { break; }
				std::cout << "[WARNING] Export folder has not been succesfully created" << std::endl;
				return false;
			}
			continue;
		}

		outputPath_ = singlepath;
		return true;
	}

}

bool IOManager::getUseDefaultSettings()
{
	std::cout << "Use default process/export settings? (Y/N):";
	if (!yesNoQuestion()) { return false; }

	boost::filesystem::path targetFilePath(inputPathList_[0]);
	boost::filesystem::path targetFolderPath = targetFilePath.parent_path().string() + "/_exports";

	outputPath_ = targetFolderPath.string() + "/evnBuilding.city.json";
	struct stat info;
	if (!boost::filesystem::exists(targetFolderPath))
	{
		std::cout << "[INFO] Export folder is created at: " << targetFolderPath.string() << std::endl;
		if (!boost::filesystem::create_directory(targetFilePath)) { return false; }
	}
	return true;
}

std::string IOManager::getFileName(const std::string& stringPath)
{
	std::vector<std::string> segments;
	boost::split(segments, stringPath, boost::is_any_of("/, \\"));
	std::string filePath = segments[segments.size() - 1];
	return filePath.substr(0, filePath.size() - 4);
}

bool IOManager::getDesiredLoD()
{
	make00_ = false;
	make02_ = false;
	make10_ = false;
	make12_ = false;
	make13_ = false;
	make22_ = false;
	make32_ = false;

	std::cout << "Please select the desired output LoD" << std::endl;
	std::cout << "Can be a single string (e.g. 123 -> LoD0.1, 0.2, 1.0)" << std::endl;
	std::cout << "1. LoD0.0" << std::endl;
	std::cout << "2. LoD0.2" << std::endl;
	std::cout << "3. LoD1.0" << std::endl;
	std::cout << "4. LoD1.2" << std::endl;
	std::cout << "5. LoD1.3" << std::endl;
	std::cout << "6. LoD2.2" << std::endl;
	std::cout << "7. LoD3.2" << std::endl;
	std::cout << "8. All" << std::endl;

	while (true)
	{
		bool validInput = true;
		std::cout << "Num: ";

		std::string stringNum = "";
		std::getline(std::cin, stringNum);

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (!std::isdigit(stringNum[i]))
			{
				validInput = false;
			}
		}

		if (!validInput)
		{
			std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
			continue;
		}

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (stringNum[i] == '1') { make00_ = true; }
			if (stringNum[i] == '2') { make02_ = true; }
			if (stringNum[i] == '3') { make10_ = true; }
			if (stringNum[i] == '4') { make12_ = true; }
			if (stringNum[i] == '5') { make13_ = true; }
			if (stringNum[i] == '6') { make22_ = true; }
			if (stringNum[i] == '7') { make32_ = true; }
			if (stringNum[i] == '8') {
				make00_ = true;
				make02_ = true;
				make10_ = true;
				make12_ = true;
				make13_ = true;
				make22_ = true;
				make32_ = true;
			}
		}
		return true;
	}
	return false;
}

bool IOManager::getBoudingRules()
{
	std::cout << "Please select a desired rulset for space bounding objects" << std::endl;
	std::cout << "1. Default room bounding objects" << std::endl;
	std::cout << "2. Default room bounding objects + IfcBuildingElementProxy objects" << std::endl;
	std::cout << "3. Default room bounding objects + custom object selection" << std::endl;
	std::cout << "4. custom object selection" << std::endl;

	while (true)
	{
		int ruleNum = numQuestion(3) + 1;

		if (ruleNum == 1)
		{
			return true;
		}

		if (ruleNum == 2)
		{
			useProxy_ = true;
			return true;
		}

		if (ruleNum == 3 || ruleNum == 4)
		{
			if (ruleNum == 3) { useDefaultDiv_ = false; }

			std::cout << std::endl;
			std::cout << "Please enter the desired IfcTypes" << std::endl;
			std::cout << "[INFO] Not case sensitive" << std::endl;
			std::cout << "[INFO] Seperate type by enter" << std::endl;
			std::cout << "[INFO] Finish by empty line + enter" << std::endl;

			while (true)
			{
				std::cout << "IfcType: ";

				std::string singlepath = "";

				std::getline(std::cin, singlepath);

				if (singlepath.size() == 0 && addDivObjects_.size() == 0)
				{
					std::cout << "[INFO] No type has been supplied" << std::endl;
					continue;
				}
				else if (singlepath.size() == 0)
				{
					std::cout << std::endl;
					return true;
				}
				else if (boost::to_upper_copy<std::string>(singlepath.substr(0, 3)) == "IFC") {

					std::string potentialType = boost::to_upper_copy<std::string>(singlepath);

					if (DevObjectsOptions_.find(potentialType) == DevObjectsOptions_.end())
					{
						std::cout << "[INFO] Type is not an IfcType" << std::endl;
						continue;
					}

					if (addDivObjects_.find(potentialType) != addDivObjects_.end())
					{
						std::cout << "[INFO] Type is already used as space bounding object" << std::endl;
						continue;
					}

					addDivObjects_.insert(potentialType);

				}
				else
				{
					std::cout << "[INFO] No valid type has been supplied" << std::endl;
					continue;
				}
			}
		}
		std::cout << "[INFO] No valid option was chosen" << std::endl;
	}
	return false;
}

bool IOManager::getVoxelSize()
{
	// ask user for desired voxel dimensions

	std::string stringXYSize = "";
	std::string stringZSize = "";

	while (true)
	{
		std::cout << "Enter voxel XY dimenion (double):";
		std::getline(std::cin, stringXYSize);

		char* end = nullptr;
		double val = strtod(stringXYSize.c_str(), &end);

		if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
		{
			voxelSize_ = val;
			break;
		}

		std::cout << "[INFO] No valid number has been supplied" << std::endl;
	}
}

bool IOManager::getFootprintElev()
{
	std::string elev = "";
	while (true)
	{
		std::cout << "Please enter the footprint elevation: ";
		std::getline(std::cin, elev);

		double dEleve;
		auto iElev = std::istringstream(elev);
		iElev >> dEleve;

		if (!iElev.fail() && iElev.eof())
		{
			footprintElevation_ = dEleve;
			return true;
		}

		std::cout << "[INFO] No valid elevation has been supplied" << std::endl;
	}
	return false;
}

bool IOManager::getUserValues()
{
	// output targets

	std::cout << std::endl;
	if (getUseDefaultSettings()) {
		printSummary();
		return true;
	}

	std::cout << std::endl;
	if (!getOutputPathList()) { return false; }

	std::cout << std::endl;
	std::cout << "Create report file (Y/N): ";
	writeReport_ = yesNoQuestion();

	std::cout << std::endl;
	if (!getDesiredLoD()) { return false; }

	std::cout << std::endl;
	std::cout << "Customize space bounding objects? (Y/N): ";
	if (yesNoQuestion()) {
		if (!getBoudingRules()) { return false; }
	}

	std::cout << std::endl;
	std::cout << "Desired voxel size? : ";
	if (!getVoxelSize()) { return false; }

	if (make02_)
	{
		std::cout << std::endl;
		getFootprintElev();
	}
	return true;
}

bool IOManager::getJSONValues()
{
	std::ifstream f(inputPathList_[0]);
	nlohmann::json json = nlohmann::json::parse(f);

	inputPathList_.clear();

	if (json.contains("Output report"))
	{
		if (json["Output report"].type() != nlohmann::json::value_t::number_integer && json["Output report"].type() != nlohmann::json::value_t::number_unsigned)
		{
			throw std::string("JSON file does not contain a valid output report entry");
		}

		if (json["Output report"] == 0) { writeReport_ = false; }
	}

	if (json.contains("Voxel summary"))
	{
		if (json["Voxel summary"] == 1) { summaryVoxels_ = true; }
	}

	if (!json.contains("Filepaths"))
	{
		throw std::string("JSON file does not contain Filpaths Entry");
	}

	nlohmann::json filePaths = json["Filepaths"];

	if (!filePaths.contains("Input"))
	{
		throw std::string("JSON file does not contain Input Filepath Entry");
	}
	if (!filePaths.contains("Output"))
	{
		throw std::string("JSON file does not contain Output Filepath Entry");
	}

	nlohmann::json inputPaths = filePaths["Input"];
	if (inputPaths.type() != nlohmann::json::value_t::array )
	{
		throw std::string("JSON file does not contain valid input filepath entry: Input filepath entry should be array");
	}

	for (size_t i = 0; i < inputPaths.size(); i++) {
		if (inputPaths[i].type() != nlohmann::json::value_t::string)
		{
			std::cout << "JSON file does not contain valid input filepath entry" << std::endl;
			return false;
		}

		std::string inputPath = inputPaths[i];

		if (hasExtension(inputPath, "ifc") && isValidPath(inputPath))
		{
			inputPathList_.emplace_back(inputPaths[i]);
		}
		else
		{
			throw std::string("[INFO] JSON file input filepath " + inputPath + " is invalid");
		}	
	}

	if (filePaths["Output"].type() != nlohmann::json::value_t::string)
	{
		throw std::string("JSON file does not contain valid output path entry, output filepath entry should be string");
	}

	outputPath_ = filePaths["Output"];

	if (!hasExtension(outputPath_, "json"))
	{
		throw std::string("JSON file does not contain valid output path, output path should end on .json or .city.json");
	}

	boost::filesystem::path outputFolderPath = boost::filesystem::path(std::string(outputPath_)).parent_path();

	if (!boost::filesystem::exists(outputFolderPath))
	{
		throw std::string("Target filepath folder does not exist");
	}

	if (json.contains("LoD output"))
	{
		nlohmann::json lodList = json["LoD output"];
		make00_ = false;
		make02_ = false;
		make10_ = false;
		make12_ = false;
		make13_ = false;
		make22_ = false;
		make32_ = false;
		makeV_ = false;

		for (size_t i = 0; i < lodList.size(); i++) // check if interior generation is required
		{
			if (LoDWInterior_.find(lodList[i]) == LoDWInterior_.end()) { continue; }

			if (json.contains("Generate interior"))
			{
				makeInterior_ = (int)json["Generate interior"];
			}

			break;
		}

		if (json.contains("Footprint elevation"))
		{
			footprintElevation_ = json["Footprint elevation"];
		}

		for (size_t i = 0; i < lodList.size(); i++)
		{
			if (lodList[i] == 0.0) { make00_ = true; }
			else if (lodList[i] == 0.2) 
			{ 
				make02_ = true; 

				if (json.contains("Generate footprint"))
				{
					makeFootPrint_ = (int)json["Generate footprint"];
				}
				if (json.contains("Generate roof outline"))
				{
					makeRoofPrint_ = (int)json["Generate roof outline"];
					if (makeRoofPrint_) { makeOutlines_ = true; }
				}
			}
			else if (lodList[i] == 1.0) 
			{ 
				make10_ = true; 
			}
			else if (lodList[i] == 1.2) 
			{ 
				make12_ = true; 
				makeOutlines_ = true;
			}
			else if (lodList[i] == 1.3) 
			{ 
				make13_ = true; 
				makeOutlines_ = true;
			}
			else if (lodList[i] == 2.2) 
			{ 
				make22_ = true;
				makeOutlines_ = true;
			}
			else if (lodList[i] == 3.2) 
			{ 
				make32_ = true; 
				makeOutlines_ = true;
			}
			else if (lodList[i] == 5.0) 
			{ 
				makeV_ = true; 
			}
		}
	}

	if (json.contains("voxelSize"))
	{
		nlohmann::json voxelData = json["voxelSize"];

		if (voxelData.contains("xy")) { voxelSize_ = voxelData["xy"]; }
	}

	if (json.contains("Default div"))
	{
		if (json["Default div"] == 0)
		{
			useDefaultDiv_ = false;
		}
	}

	if (json.contains("Ignore proxy"))
	{
		if (json["Ignore proxy"] == 1)
		{
			useProxy_ = true;
		}
	}

	if (json.contains("voxel planes"))
	{
		if (json["voxel planes"] == 1)
		{
			planeIntersection_ = true;
		}
	}

	if (json.contains("Div objects"))
	{
		std::vector<std::string> stringDivList = json["Div objects"];

		for (size_t i = 0; i < stringDivList.size(); i++)
		{
			std::string potentialType = stringDivList[i];
			std::transform(potentialType.begin(), potentialType.end(), potentialType.begin(), ::toupper);

			if (DevObjectsOptions_.find(potentialType) == DevObjectsOptions_.end())
			{
				continue;
			}

			if (addDivObjects_.find(potentialType) != addDivObjects_.end())
			{
				continue;
			}
			addDivObjects_.insert(potentialType);
		}
	}

	return true;
}

bool IOManager::hasExtension(const std::vector<std::string>& stringList, const std::string& ext)
{
	for (size_t i = 0; i < stringList.size(); i++)
	{
		if (!hasExtension(stringList[i], ext)) { return false; }
	}
	return true;
}

bool IOManager::hasExtension(const std::string& string, const std::string& ext)
{
	std::string substring = boost::to_lower_copy<std::string>(string.substr(string.find_last_of(".") + 1));
	if (substring == ext) { return true; }
	return false;
}


bool IOManager::isValidPath(const std::vector<std::string>& path)
{
	for (size_t i = 0; i < path.size(); i++)
	{
		if (!isValidPath(path[i])) { return false; }
	}
	return true;
}


bool IOManager::isValidPath(const std::string& path)
{
	struct stat info;
	if (stat(path.c_str(), &info) != 0) { return false; }
	return true;
}

void addTimeToJSON(nlohmann::json* j, const std::string& valueName, const std::chrono::steady_clock::time_point& startTime, const std::chrono::steady_clock::time_point& endTime)
{

	double duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5) { (*j)[valueName + " (ms)"] = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count(); }
	else { (*j)[valueName + " (s)"] = duration; }
}

void addTimeToJSON(nlohmann::json* j, const std::string& valueName, double duration)
{
	std::string timeUnitString = "Unit";
	std::string timeDurationString = "Duration";
	nlohmann::json timeSet;
	if (duration == 0)  { return; }
	else if (duration < 5000) 
	{
		timeSet[timeDurationString] = duration;
		timeSet[timeUnitString] = "ms";
	}
	else 
	{
		timeSet[timeDurationString] = duration/1000;
		timeSet[timeUnitString] = "s";
	}
	(*j)[valueName] = timeSet;
	return;
}

void IOManager::printSummary()
{
	std::cout << "=============================================================" << std::endl;
	std::cout << "[INFO] Used settings: " << std::endl;

	std::cout << "- Input File(s):" << std::endl;
	for (size_t i = 0; i < inputPathList_.size(); i++) { std::cout << "    " << inputPathList_[i] << std::endl; }
	std::cout << "- Output File:" << std::endl;
	std::cout << "    " << outputPath_ << std::endl;
	std::cout << "- Create Report:" << std::endl;
	if (writeReport_) { std::cout << "    yes" << std::endl; }
	else { std::cout << "    no" << std::endl; }
	std::cout << "- LoD export enabled:" << std::endl;
	std::cout << "    " << getLoDEnabled() << std::endl;
	std::cout << "- Space dividing objects: " << std::endl;
	if (useDefaultDiv_)
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	}
	if (useProxy_)
	{
		std::cout << "    IFCBUILDINGELEMENTPROXY" << std::endl;
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	std::cout << "- Voxel size:" << std::endl;
	std::cout << "    " << voxelSize_ << std::endl;


	if (make02_)
	{
		std::cout << "- Create footprint:" << std::endl;
		if (makeFootPrint_) { std::cout << "    Yes" << std::endl; }
		else { std::cout << "    No" << std::endl; }

		std::cout << "- Store Lod0.2 roof outline:" << std::endl;
		if (makeRoofPrint_) { std::cout << "    Yes" << std::endl; }
		else { std::cout << "    No" << std::endl; }
	}

	std::cout << "- Footprint Elevation:" << std::endl;
	std::cout << "    " << footprintElevation_ << std::endl;

	std::cout << "=============================================================" << std::endl;
}

std::string IOManager::getLoDEnabled()
{
	std::string summaryString = "";

	if (make00_) { summaryString += ", 0.0"; }
	if (make02_) { summaryString += ", 0.2"; }
	if (make12_) { summaryString += ", 1.2"; }
	if (make13_) { summaryString += ", 1.3"; }
	if (make22_) { summaryString += ", 2.2"; }
	if (make32_) { summaryString += ", 3.2"; }
	if (makeV_) { summaryString += ", 5.0 (V)"; }

	summaryString.erase(0, 2);

	return summaryString;
}



nlohmann::json IOManager::settingsToJSON()
{
	nlohmann::json settingsJSON;

	settingsJSON["Input IFC file"] = inputPathList_;
	settingsJSON["Output CityJSON file"] = getOutputPath();
	settingsJSON["Create report"] = "true";
	if (isJsonInput_) { settingsJSON["JSON input"] = "true"; }
	else if (!isJsonInput_) { settingsJSON["JSON input"] = "false"; }

	std::vector<std::string> DivList;

	if (useDefaultDiv_)
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { DivList.emplace_back(boost::to_upper_copy(*it)); }
	}
	if (useProxy_)
	{
		DivList.emplace_back("IFCBUILDINGELEMENTPROXY");
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { DivList.emplace_back(boost::to_upper_copy(*it)); }
	settingsJSON["Space bounding objects"] = DivList;
	settingsJSON["Voxel size"] = voxelSize_;

	if (make02_) 
	{ 
		settingsJSON["Footprint elevation"] = footprintElevation_; 
		settingsJSON["Generate footprint"] = makeFootPrint_;
		settingsJSON["Generate roof outline"] = makeRoofPrint_;
	}
	
	std::vector<std::string> LoDList;
	if (make00_) { LoDList.emplace_back("0.0"); }
	if (make02_) { LoDList.emplace_back("0.2"); }
	if (make10_) { LoDList.emplace_back("1.0"); }
	if (make12_) { LoDList.emplace_back("1.2"); }
	if (make13_) { LoDList.emplace_back("1.3"); }
	if (make22_) { LoDList.emplace_back("2.2"); }
	if (make32_) { LoDList.emplace_back("3.2"); }

	settingsJSON["Desired LoD output"] = LoDList;

	return settingsJSON;
}

bool IOManager::init(const std::vector<std::string>& inputPathList, bool silent)
{
	timeTotal = 0;
	bool isSilent_ = silent;

	// find if input has IFC format
	bool isIFC = true;

	if (!isSilent_)
	{
		std::wcout << "============================================================= \n" << std::endl;
		std::cout << "		IFC_BuildingEnvExtractor " << buildVersion << std::endl;
		std::cout << "    Experimental building shell extractor/approximation\n" << std::endl;
		std::wcout << "=============================================================" << std::endl;
		std::cout << std::endl;
	}

	bool hasDirectInterface = false;
	if (inputPathList.size() == 0)
	{
		getTargetPathList();
		hasDirectInterface = true;
	}
	else if (hasExtension(inputPathList, "ifc") && isValidPath(inputPathList)) // If all files are IFC copy the path list and ask user for info
	{
		inputPathList_ = inputPathList;
		if (!isSilent_)
		{
			std::cout << "[INFO] Input file path(s):" << std::endl;
			for (size_t i = 0; i < inputPathList_.size(); i++) { std::cout << inputPathList_[i] << std::endl; }
			std::cout << std::endl;
		}
		hasDirectInterface = true;
	}
	else if (hasExtension(inputPathList, "json") && isValidPath(inputPathList)) {
		isJsonInput_ = true;
		inputPathList_ = inputPathList;
	}
	if (!isJsonInput_)
	{
		if (!getUserValues()) { return false; } // attempt to ask user for data		 
	}
	else
	{
		if (inputPathList.size() > 1) { return false; }
		isJsonInput_ = true;

		try { getJSONValues(); }
		catch (const std::string& exceptionString)
		{
			throw exceptionString;
		}
	}

	if (!isSilent_)
	{
		std::cout << std::endl;
		printSummary();
	}
	auto internalizingTime = std::chrono::high_resolution_clock::now(); // Time Collection Starts
	
	internalHelper_ = std::make_unique<helper>(inputPathList_);
	helper* internalHelperPtr = internalHelper_.get();
	if (!internalHelperPtr->isPopulated()) { return 0; }
	if (!internalHelperPtr->hasSetUnits()) { return 0; }
	internalHelperPtr->setName(getFileName(inputPathList_[0]));
	internalHelperPtr->setfootprintLvl(footprintElevation_);
	internalHelperPtr->setUseProxy(useProxy_);
	internalHelperPtr->setUseVoxelPlanes(planeIntersection_);

	return true;
}

bool IOManager::run()
{
	// Time Collection Starts
	auto internalizingTime = std::chrono::high_resolution_clock::now();
	
	// internalize the helper data
	internalHelper_->internalizeGeo();
	internalHelper_->indexGeo();

	timeInternalizing_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - internalizingTime).count();
	// create the cjt objects
	std::unique_ptr<CJT::CityCollection> collection = std::make_unique<CJT::CityCollection>();
	CJT::CityCollection* collectionPtr = collection.get();
	gp_Trsf geoRefRotation;
	CJT::ObjectTransformation transformation(0.001);
	CJT::metaDataObject metaData;
	metaData.setTitle(internalHelper_.get()->getFileName() + " Auto export from IfcEnvExtractor");
	internalHelper_.get()->getProjectionData(&transformation, &metaData, &geoRefRotation);
	transformation.setScale(*transformation.getScale()); //TODO: fix cjt to make this not required.

	// Set up objects and their relationships
	CJT::CityObject cityBuildingObject;
	CJT::CityObject cityShellObject;
	CJT::CityObject cityInnerShellObject;

	std::string BuildingName = internalHelper_.get()->getBuildingName();
	if (BuildingName == "") { BuildingName = internalHelper_.get()->getProjectName(); }

	cityBuildingObject.setName(BuildingName);
	cityBuildingObject.setType(CJT::Building_Type::Building);

	cityShellObject.setName("Outer Shell");
	cityShellObject.setType(CJT::Building_Type::BuildingPart);

	cityInnerShellObject.setName("Inner Shell");
	cityInnerShellObject.setType(CJT::Building_Type::BuildingPart);

	cityBuildingObject.addChild(&cityShellObject);
	cityBuildingObject.addChild(&cityInnerShellObject);

	CJT::Kernel kernel = CJT::Kernel(collectionPtr);

	std::map<std::string, std::string> buildingAttributes = internalHelper_.get()->getBuildingInformation();
	for (std::map<std::string, std::string>::iterator iter = buildingAttributes.begin(); iter != buildingAttributes.end(); ++iter) { cityBuildingObject.addAttribute(iter->first, iter->second); }

	// make the geometrycreator and voxelgrid
	auto voxelTime = std::chrono::high_resolution_clock::now();
	CJGeoCreator geoCreator(internalHelper_.get(), voxelSize_);
	timeVoxel_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - voxelTime).count();

	if (makeOutlines_)
	{
		try
		{
			geoCreator.initializeBasic(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorList_.emplace_back("Basic initialization failed");
			return false;
		}
	}

	if (make02_ && makeFootPrint_)
	{
		try
		{
			geoCreator.makeFootprint(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorList_.emplace_back("Footprint creation failed");
		}
	}

	if (makeInterior_)
	{
		try
		{
			geoCreator.makeFloorSectionCollection(internalHelper_.get());
		}
		catch (const std::exception&)
		{
			ErrorList_.emplace_back("storey creation failed");
		}
	}

	if (makeRoofPrint_)
	{
		geoCreator.useroofprint0();
	}

	geoCreator.setRefRotation(geoRefRotation);

	if (makeLoD00())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			CJT::GeoObject* geo00 = geoCreator.makeLoD00(internalHelper_.get(), &kernel, 1);
			cityShellObject.addGeoObject(*geo00);
			timeLoD00_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD0.0 creation failed"); }

	}
	if (makeLoD02())
	{
		auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
		try
		{
			std::vector<CJT::GeoObject*> geo02 = geoCreator.makeLoD02(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo02.size(); i++) { cityShellObject.addGeoObject(*geo02[i]); }
			
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD0.2 creation failed"); }

		if (makeInterior_)
		{
			std::vector<CJT::CityObject> geo02Storeys = geoCreator.makeLoD02Storeys(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo02Storeys.size(); i++) {
				CJT::CityObject currentStoreyObject = geo02Storeys[i];
				currentStoreyObject.addParent(&cityInnerShellObject);
				collectionPtr->addCityObject(currentStoreyObject);
			}
		}
		timeLoD02_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();

	}
	if (makeLoD10())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			CJT::GeoObject* geo10 = geoCreator.makeLoD10(internalHelper_.get(), &kernel, 1);
			cityShellObject.addGeoObject(*geo10);
			timeLoD10_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD1.0 creation failed"); }
	}
	if (makeLoD12())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject*> geo12 = geoCreator.makeLoD12(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo12.size(); i++) { cityShellObject.addGeoObject(*geo12[i]); }
			timeLoD12_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD1.2 creation failed"); }
	}
	if (makeLoD13())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject*> geo13 = geoCreator.makeLoD13(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo13.size(); i++) { cityShellObject.addGeoObject(*geo13[i]); }
			timeLoD13_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD1.3 creation failed"); }
	}
	if (makeLoD22())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject*> geo22 = geoCreator.makeLoD22(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo22.size(); i++) { cityShellObject.addGeoObject(*geo22[i]); }
			timeLoD22_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD2.2 creation failed"); }
	}
	if (makeLoD32())
	{
		try
		{
			auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
			std::vector<CJT::GeoObject*> geo32 = geoCreator.makeLoD32(internalHelper_.get(), &kernel, 1);
			for (size_t i = 0; i < geo32.size(); i++) { cityShellObject.addGeoObject(*geo32[i]); }
			timeLoD32_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
		}
		catch (const std::exception&) { ErrorList_.emplace_back("LoD3.2 creation failed"); }
	}
	if (makeV())
	{
		auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
		std::vector<CJT::GeoObject*> geoV = geoCreator.makeV(internalHelper_.get(), &kernel, 1);
		for (size_t i = 0; i < geoV.size(); i++) { cityShellObject.addGeoObject(*geoV[i]); }
		timeV_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeGeoCreation).count();
	}
	
	collectionPtr->setTransformation(transformation);
	collectionPtr->setVersion("1.1");

	// compute the extends
	gp_Trsf ExtenstTranslation;
	ExtenstTranslation.SetTranslationPart(gp_Vec(transformation.getTranslation()[0], transformation.getTranslation()[1], transformation.getTranslation()[2]));


	gp_Pnt lll = internalHelper_.get()->getLllPoint().Transformed(internalHelper_.get()->getObjectTranslation().Inverted());
	gp_Pnt urr = internalHelper_.get()->getUrrPoint().Transformed(internalHelper_.get()->getObjectTranslation().Inverted());

	gp_Trsf trs;
	trs.SetRotation(geoCreator.getRefRotation().GetRotation());
	trs.SetTranslationPart(
		gp_Vec(transformation.getTranslation()[0], transformation.getTranslation()[1], transformation.getTranslation()[2]
		)
	);

	TopoDS_Shape bboxGeo = helperFunctions::createBBOXOCCT(lll, urr).Moved(trs);
	bg::model::box <BoostPoint3D> extents = helperFunctions::createBBox(bboxGeo);

	extents.min_corner();

	metaData.setExtend(
		CJT::CJTPoint(extents.min_corner().get<0>(), extents.min_corner().get<1>(), extents.min_corner().get<2>()),
		CJT::CJTPoint(extents.max_corner().get<0>(), extents.max_corner().get<1>(), extents.max_corner().get<2>())
	);

	collectionPtr->setMetaData(metaData);

	cityShellObject.addAttribute("Env_ex footprint elevation", footprintElevation_);
	cityShellObject.addAttribute("Env_ex buildingHeight", internalHelper_.get()->getUrrPoint().Z() - footprintElevation_);

	if (summaryVoxel())
	{
		geoCreator.extractVoxelSummary(
			&cityShellObject,
			internalHelper_.get(),
			internalHelper_.get()->getfootprintEvalLvl(),
			geoRefRotation.GetRotation().GetRotationAngle()
		);
	}

	collectionPtr->addCityObject(cityBuildingObject);
	collectionPtr->addCityObject(cityShellObject);
	collectionPtr->addCityObject(cityInnerShellObject);
	collectionPtr->cullDuplicatedVerices();
	cityCollection_ = std::move(collection);

	return true;
}

bool IOManager::write()
{
	cityCollection_.get()->dumpJson(getOutputPath());

	if (!writeReport_) { return true; }
	nlohmann::json report;
	report["Settings"] = settingsToJSON();

	nlohmann::json timeReport;
	addTimeToJSON(&timeReport, "Internalizing", timeInternalizing_);
	addTimeToJSON(&timeReport, "Voxel creation", timeVoxel_);
	addTimeToJSON(&timeReport, "LoD0.0 generation", timeLoD00_);
	addTimeToJSON(&timeReport, "LoD0.2 generation", timeLoD02_);
	addTimeToJSON(&timeReport, "LoD1.0 generation", timeLoD10_);
	addTimeToJSON(&timeReport, "LoD1.2 generation", timeLoD12_);
	addTimeToJSON(&timeReport, "LoD1.3 generation", timeLoD13_);
	addTimeToJSON(&timeReport, "LoD2.2 generation", timeLoD22_);
	addTimeToJSON(&timeReport, "LoD3.2 generation", timeLoD32_);
	addTimeToJSON(&timeReport, "LoD5.0 (V) generation", timeV_);
	addTimeToJSON(&timeReport, "Total Processing",
		timeInternalizing_ +
		timeVoxel_ +
		timeLoD00_ +
		timeLoD02_ +
		timeLoD10_ +
		timeLoD12_ +
		timeLoD13_ +
		timeLoD22_ +
		timeLoD32_ +
		timeV_
	);

	report["Duration"] = timeReport;

	report["Errors"] = ErrorList_;

	//addTimeToJSON(&report, "Total running time", startTime, endTime);
	const std::string extension1 = ".json";

	boost::filesystem::path filePath(getOutputPath());
	filePath.replace_extension("");
	if (hasExtension(filePath.string(), "city")) { filePath.replace_extension(""); }

	boost::filesystem::path filePathWithoutExtension = boost::filesystem::path(getOutputPath()).stem();
	std::ofstream reportFile(filePath.string() + "_report.json");
	reportFile << report;
	reportFile.close();
}