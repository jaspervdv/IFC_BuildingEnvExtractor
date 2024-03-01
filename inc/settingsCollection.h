#include <vector>

#ifndef SETTINGSCOLLECTION_SETTINGSCOLLECTION_H
#define SETTINGSCOLLECTION_SETTINGSCOLLECTION_H

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

	bool summaryVoxels_ = false;
	bool writeReport_ = true;

	// variables set the deviding objects
	bool useDefaultDiv_ = true;
	bool useProxy_ = false;

	// use 3 planes instead of volumetric voxel intersections
	bool planeIntersection_ = false;

	double voxelSize_ = 0.5;

	double footprintElevation_ = 0;

	// how many proxy objects are present in the input
	int proxyCount_ = 0;

	// generated settings
};
#endif // SETTINGSCOLLECTION_SETTINGSCOLLECTION_H