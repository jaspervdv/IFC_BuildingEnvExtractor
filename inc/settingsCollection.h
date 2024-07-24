#include <vector>
#include <thread>

#ifndef SETTINGSCOLLECTION_SETTINGSCOLLECTION_H
#define SETTINGSCOLLECTION_SETTINGSCOLLECTION_H

struct SettingsCollection {

private:
	// Input settings

	// if true no comminucation is pushed to console
	bool isSilent_ = false;

	std::vector<std::string> inputPathList_ = {};
	std::string outputPath_ = "";

	std::unordered_set<double> LoDWInterior_ = { 0.2, 5.0 }; //TODO: move this

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
	int intersectionLogic_ = 3;

	double voxelSize_ = 0.5;
	double surfaceGridSize_ = 0.3; //TODO: make accessible for user?

	bool autoRotateGrid_ = true;
	double desiredRotation_ = 0;

	double footprintElevation_ = 0;

	double precision_ = 1e-6;
	double precisionCoarse_ = 1e-4;

	int threadcount_ = std::thread::hardware_concurrency();

	// how many proxy objects are present in the input
	int proxyCount_ = 0;

	// \/ generated settings \/
	// if LoD0.0 and 1.0 is generated only no voxels are required
	bool requireVoxels_ = true;

	// if Only roof outlines are required no full voxelization is needed.
	bool requireFullVoxels_ = true;

	// if LoD13 output is done and the same as LoD22, copy the data
	bool Lod123IsFlat_ = false;
	
	double gridRotation_ = 0; // software computes itself or sets it to the desired rotation

	SettingsCollection() = default;

public:
	static SettingsCollection& getInstance() {
		static SettingsCollection instance;
		return instance;
	}

	// disable asignement and copying
	SettingsCollection(const SettingsCollection&) = delete;
	SettingsCollection& operator=(const SettingsCollection&) = delete;

    bool isSilent() const { return isSilent_; }
    void setSilent(bool value) { isSilent_ = value; }

    const std::vector<std::string>& getInputPathList() const { return inputPathList_; }
    void setInputPathList(const std::vector<std::string>& value) { inputPathList_ = value; }
    void addToInputPathList(const std::string& value) { inputPathList_.emplace_back(value); }
    void clearInputPathList() { inputPathList_.clear(); }

    const std::string& getOutputPath() const { return outputPath_; }
    void setOutputPath(const std::string& value) { outputPath_ = value; }

    const std::unordered_set<double>& getLoDWInterior() const { return LoDWInterior_; }
    void setLoDWInterior(const std::unordered_set<double>& value) { LoDWInterior_ = value; }

    bool make00() const { return make00_; }
    void setMake00(bool value) { make00_ = value; }

    bool make02() const { return make02_; }
    void setMake02(bool value) { make02_ = value; }

    bool make10() const { return make10_; }
    void setMake10(bool value) { make10_ = value; }

    bool make12() const { return make12_; }
    void setMake12(bool value) { make12_ = value; }

    bool make13() const { return make13_; }
    void setMake13(bool value) { make13_ = value; }

    bool make22() const { return make22_; }
    void setMake22(bool value) { make22_ = value; }

    bool make32() const { return make32_; }
    void setMake32(bool value) { make32_ = value; }

    bool makeV() const { return makeV_; }
    void setMakeV(bool value) { makeV_ = value; }

    bool makeOutlines() const { return makeOutlines_; }
    void setMakeOutlines(bool value) { makeOutlines_ = value; }

    bool makeFootPrint() const { return makeFootPrint_; }
    void setMakeFootPrint(bool value) { makeFootPrint_ = value; }

    bool makeRoofPrint() const { return makeRoofPrint_; }
    void setMakeRoofPrint(bool value) { makeRoofPrint_ = value; }

    bool makeInterior() const { return makeInterior_; }
    void setMakeInterior(bool value) { makeInterior_ = value; }

    bool geoReference() const { return geoReference_; }
    void setGeoReference(bool value) { geoReference_ = value; }

    bool summaryVoxels() const { return summaryVoxels_; }
    void setSummaryVoxels(bool value) { summaryVoxels_ = value; }

    bool writeReport() const { return writeReport_; }
    void setWriteReport(bool value) { writeReport_ = value; }

    bool useDefaultDiv() const { return useDefaultDiv_; }
    void setUseDefaultDiv(bool value) { useDefaultDiv_ = value; }

    bool useProxy() const { return useProxy_; }
    void setUseProxy(bool value) { useProxy_ = value; }

    const std::vector<std::string>& getCustomDivList() const { return CustomDivList_; }
    void setCustomDivList(const std::vector<std::string>& value) { CustomDivList_ = value; }
    void addToCustomDivList(const std::string& value) { CustomDivList_.emplace_back(value); }

    int intersectionLogic() const { return intersectionLogic_; }
    void setIntersectionLogic(int value) { intersectionLogic_ = value; }

    double voxelSize() const { return voxelSize_; }
    void setVoxelSize(double value) { voxelSize_ = value; }

    double surfaceGridSize() const { return surfaceGridSize_; }
    void setSurfaceGridSize(double value) { surfaceGridSize_ = value; }

    bool autoRotateGrid() const { return autoRotateGrid_; }
    void setAutoRotateGrid(bool value) { autoRotateGrid_ = value; }

    double desiredRotation() const { return desiredRotation_; }
    void setDesiredRotation(double value) { desiredRotation_ = value; }

    double footprintElevation() const { return footprintElevation_; }
    void setFootprintElevation(double value) { footprintElevation_ = value; }

    double precision() const { return precision_; }
    void setPrecision(double value) { precision_ = value; }

    double precisionCoarse() const { return precisionCoarse_; }
    void setPrecisionCoarse(double value) { precisionCoarse_ = value; }

    int threadcount() const { return threadcount_; }
    void setThreadcount(int value) { threadcount_ = value; }

    int proxyCount() const { return proxyCount_; }
    void setProxyCount(int value) { proxyCount_ = value; }

    bool requireVoxels() const { return requireVoxels_; }
    void setRequireVoxels(bool value) { requireVoxels_ = value; }

    bool requireFullVoxels() const { return requireFullVoxels_; }
    void setRequireFullVoxels(bool value) { requireFullVoxels_ = value; }

    bool Lod123IsFlat() const { return Lod123IsFlat_; }
    void setLod123IsFlat(bool value) { Lod123IsFlat_ = value; }

    double gridRotation() const { return gridRotation_; }
    void setGridRotation(double value) { gridRotation_ = value; }
};
#endif // SETTINGSCOLLECTION_SETTINGSCOLLECTION_H