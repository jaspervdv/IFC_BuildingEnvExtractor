#include <vector>

#ifndef SETTINGSCOLLECTION_SETTINGSCOLLECTION_H
#define SETTINGSCOLLECTION_SETTINGSCOLLECTION_H

struct ErrorObject {
	std::string errorCode_ = "";
	std::string errorDescript_ = "";
	std::vector<std::string> occuringObjectList_;

	nlohmann::json toJson();
};

struct SettingsCollection {
	// Input settings

	// if true no comminucation is pushed to console
	bool isSilent_ = false;

	// if programm is instructed by a json file = true
	bool isJsonInput_ = false;

	std::vector<std::string> inputPathList_ = {};
	std::string outputPath_ = "";

	std::unordered_set<double> LoDWInterior_ = { 0.2, 5.0 };

	// sets which LoD envelopes are attampted to be created
	bool make00_ = true;
	bool make02_ = true;
	bool make10_ = true;
	bool make12_ = true;
	bool make13_ = true;
	bool make22_ = true;
	bool make32_ = true;
	bool makeV_ = true;

	bool makeOutlines_ = false;
	bool makeFootPrint_ = false;
	bool makeRoofPrint_ = true;
	bool makeInterior_ = false;

	bool geoReference_ = true;

	bool summaryVoxels_ = false;
	bool writeReport_ = true;

	// variables set the deviding objects
	bool useDefaultDiv_ = true;
	bool useProxy_ = false;

	std::vector<std::string> CustomDivList_;

	// use 3 planes instead of volumetric voxel intersections
	bool planeIntersection_ = false;

	double voxelSize_ = 0.5;

	double footprintElevation_ = 0;

	// how many proxy objects are present in the input
	int proxyCount_ = 0;

	// \/ generated settings \/

	// if LoD0.0 and 1.0 is generated only no voxels are required
	bool requireVoxels_ = true;

	// if Only roof outlines are required no full voxelization is needed.
	bool requireFullVoxels_ = true;

	// if LoD13 output is done and the same as LoD22, copy the data
	bool Lod123IsFlat_ = false;

	double originRot_ = 0;
};
#endif // SETTINGSCOLLECTION_SETTINGSCOLLECTION_H