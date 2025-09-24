#include <vector>
#include <thread>
#include <unordered_set>
#include <mutex>

#include <nlohmann/json.hpp>
#include "helper.h"

#ifndef SETTINGSCOLLECTION_SETTINGSCOLLECTION_H
#define SETTINGSCOLLECTION_SETTINGSCOLLECTION_H

// collection of all the settings used by the extractor, both user set and internal code settings
struct SettingsCollection {

private:
	// Input settings

	// if true no comminucation is pushed to console
	bool isSilent_ = false;

    std::string InputJsonPath_;
	std::vector<std::string> inputIFCPathList_ = {};
    std::string outputBasePath_ = "";
	std::string outputReportPath_ = "";

	// sets which LoD envelopes are attampted to be created
	bool make00_ = false;
	bool make02_ = false;
	bool make03_ = false;
	bool make04_ = false;
	bool make10_ = false;
	bool make12_ = false;
	bool make13_ = false;
	bool make22_ = false;
	bool make32_ = false;
	
    // sets which experimental LoD envelopes are attampted to be created
    bool makeV_ = false;
    bool makeb0_ = false;
    bool makec1_ = false;
    bool makec2_ = false;
    bool maked1_ = false;
    bool maked2_ = false;
    bool makee0_ = false;
    bool makee1_ = false;

    bool makeSite_ = false;

	bool makeOutlines_ = false;
	bool makeFootPrint_ = false;
	bool makeRoofPrint_ = true;
	bool makeInterior_ = false;
	bool makeExterior_ = true;

    bool footPrintBased_ = false;
    bool voxelBasedFiltering_ = true;

	bool geoReference_ = true;
    bool correctPlacement_ = true;

	bool summaryVoxels_ = false;
	bool writeReport_ = true;

    bool createJSON_ = true;
    bool createSTEP_ = false;
    bool mergeSTEP_ = false;
    bool createOBJ_ = false;
    bool mereOBJ_ = false;

	// variables set the deviding objects
	bool useDefaultDiv_ = true;
	bool useProxy_ = false;
	std::vector<std::string> CustomDivList_;
    std::vector<std::string> defaultDivList_ = {
        "IfcSlab",
        "IfcRoof",
        "IfcWall",
        "IfcWallstandardcase",
        "IfcCovering",
        "IfcColumn",
        "IfcBeam",
        "IfcCurtainwall",
        "IfcPlate",
        "IfcMember",
        "IfcDoor",
        "IfcWindow"
    };
    int ignoreVoidGrade_ = 0;
    bool simplefyGeo_ = true;
    std::vector<std::string> ignoreSimplificationList_;

    bool mergeSemantics_ = true;

	// use 3 planes instead of volumetric voxel intersections
	int intersectionLogic_ = 3;

	double voxelSize_ = 0.5;
	double surfaceGridSize_ = 0.5; //TODO: make accessible for user?
    int minGridPointCount_ = 5;

	bool autoRotateGrid_ = true;
	double desiredRotation_ = 0;

	double footprintElevation_ = 0;
    double horizontalSectionOffset_ = 0; // offset from the storey and footprint sections
    double horizontalSectionBuffer_ = 0.15; // buffer from which mostly horizontal faces are includes as if part of the section

	int threadcount_ = 0;

    // precision settings
    double spatialTolerance_ = 1e-6;
    double angularTolerance_ = 1e-4;
    double areaTolerance_ = 1e-4;

    // unexposed settings
    bool addCustomWallAttributes_ = false;
    double maxProxyPercentage_ = 0.3;
    double searchBufferLoD32_ = 1.5 * voxelSize_;
    double thinTriangleAngle_ = 0.1745;

	// \/ generated settings \/
	// if LoD0.0 and 1.0 is generated only no voxels are required
	bool requireVoxels_ = true;

	// if Only roof outlines are required no full voxelization is needed.
	bool requireFullVoxels_ = true;

	// if LoD13 output is done and the same as LoD22, copy the data
	bool Lod123IsFlat_ = false;
	
    // software computes itself or sets it to the desired rotation
	double gridRotation_ = 0; 

    // set of the supported versions of the tool (read only!)
    std::unordered_set<std::string> ifcVersionList_ = { "IFC2X3", "IFC4X3", "IFC4" };

    // set of the ifc objects that have voids that could be closed (read only!)
    std::unordered_set<std::string> openingObjects_ = { "IfcWall", "IfcWallStandardCase", "IfcRoof", "IfcSlab" };

    // set of the ifc objects that could fit in voids of other objects (read only!)
    std::unordered_set<std::string> cuttingObjects_ = { "IfcWindow", "IfcDoor", "IfcColumn" };

    // set of the LoD abstractions that could include the iteriors (read only!)
    std::unordered_set<double> LoDWInterior_ = { 0.2, 1.2, 2.2, 3.2, 5.0 };

    // \/ stats \/
    // how many proxy objects are present in the input
    int proxyCount_ = 0;

    // how many objects are present in the input
    int objectCount_ = 0;

	SettingsCollection() = default;

    IfcGeom::IteratorSettings iteratorSettings_;
    IfcGeom::IteratorSettings simpleIteratorSettings_;


    //  \/ global mutex \/
    std::mutex wireOffSetterMutex_; // TODO: make more local

public:
	static SettingsCollection& getInstance() {
		static SettingsCollection instance;
		return instance;
	}

	// disable asignement and copying
	SettingsCollection(const SettingsCollection&) = delete;
	SettingsCollection& operator=(const SettingsCollection&) = delete;

    // read and store the json component related settings
    void setJSONRelatedSettings(const nlohmann::json& json);
    // read and store the voxel component related settings
    void setVoxelRelatedSettings(const nlohmann::json& json);
    // read and store the ifc component related settings
    void setIFCRelatedSettings(const nlohmann::json& json);
    // read and store the output format related settings
    void setFormatRelatedSettings(const nlohmann::json& json);
    // set the tolerances
    void setTolerances(const nlohmann::json& json);
    // set the generative settings related to the user submitted settings
    void generateGeneralSettings();


    bool isSilent() const { return isSilent_; }
    void setSilent(bool value) { isSilent_ = value; }

    // check if path is valid and stores it
    void setInputJSONPath(const std::string& inputString, bool validate);
    std::string getInputJSONPath() { return InputJsonPath_; }

    // populates all the paths except the JSON config path
    void setIOPaths(const nlohmann::json& json);

    const std::vector<std::string>& getIfcPathList() const { return inputIFCPathList_; }
    void setIfcPathList(const std::vector<std::string>& value) { inputIFCPathList_ = value; }
    void addToIfcPathList(const std::string& value) { inputIFCPathList_.emplace_back(value); }
    void clearIfcPathList() { inputIFCPathList_.clear(); }

    const std::string& getOutputBasePath() const { return outputBasePath_; }
    const std::string getOutputJSONPath() { return outputBasePath_ + ".json"; }
    const std::string getOutputOBJPath() { return outputBasePath_ + "_nLoDxx.obj"; }
    const std::string getOutputSTEPPath() { return outputBasePath_ + "_nLoDxx.STEP"; }
    void setOutputJSONPath(const std::string& value);

    const std::string& getOutputReportPath() const { return outputReportPath_; }
    void setOutputReportPath(const std::string& value) { outputReportPath_ = value; }

    const std::unordered_set<double>& getLoDWInterior() const { return LoDWInterior_; }
    void setLoDWInterior(const std::unordered_set<double>& value) { LoDWInterior_ = value; }

    void setLoD(const nlohmann::json& json);

    bool make00() const { return make00_; }
    void setMake00(bool value) { make00_ = value; }

    bool make02() const { return make02_; }
    void setMake02(bool value) { make02_ = value; }

    bool make03() const { return make03_; }
    void setMake03(bool value) { make03_ = value; }

    bool make04() const { return make04_; }
    void setMake04(bool value) { make04_ = value; }

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

    bool makeb0() const { return makeb0_; }
    void setMakeb0(bool value) { makeb0_ = value; }

    bool makec1() const { return makec1_; }
    void setMakec1(bool value) { makec1_ = value; }

    bool makec2() const { return makec2_; }
    void setMakec2(bool value) { makec2_ = value; }   
    
    bool maked1() const { return maked1_; }
    void setMaked1(bool value) { maked1_ = value; }    
    
    bool maked2() const { return maked2_; }
    void setMaked2(bool value) { maked2_ = value; }

    bool makee0() const { return makee0_; }
    void setMakee0(bool value) { makee0_ = value; }

    bool makee1() const { return makee1_; }
    void setMakee1(bool value) { makee1_ = value; }

    bool makeSite() const { return makeSite_; }
    void setMakeSite(bool value) { makeSite_ = value; }
    void setMakeSite(const nlohmann::json& json);

    bool makeOutlines() const { return makeOutlines_; }
    void setMakeOutlines(bool value) { makeOutlines_ = value; }
    void setMakeOutlines(const nlohmann::json& json);

    bool makeFootPrint() const { return makeFootPrint_; }
    void setMakeFootPrint(bool value) { makeFootPrint_ = value; }
    void setMakeFootPrint(const nlohmann::json& json);

    bool makeRoofPrint() const { return makeRoofPrint_; }
    void setMakeRoofPrint(bool value) { makeRoofPrint_ = value; }
    void setMakeRoofPrint(const nlohmann::json& json);

    bool makeInterior() const { return makeInterior_; }
    void setMakeInterior(bool value) { makeInterior_ = value; }
    void setMakeInterior(const nlohmann::json& json);

    bool makeExterior() const { return makeExterior_; }
    void setMakeExterior(bool value) { makeExterior_ = value; }
    void setMakeExterior(const nlohmann::json& json);

    bool footPrintBased() const { return footPrintBased_; }
    void setFootPrintBased(bool value) { footPrintBased_ = value; }
    void setFootPrintBased(const nlohmann::json& json);

    bool voxelBasedFiltering() const { return voxelBasedFiltering_; }
    void setVoxelBasedFiltering(bool value) { voxelBasedFiltering_ = value; }
    void setVoxelBasedFiltering(const nlohmann::json& json);

    bool geoReference() const { return geoReference_; }
    void setGeoReference(bool value) { geoReference_ = value; }
    void setGeoReference(const nlohmann::json& json);

    bool correctPlacement() const { return correctPlacement_; }
    void setCorrectPlacement(bool value) { correctPlacement_ = value; }
    void setCorrectPlacement(const nlohmann::json& json);

    bool summaryVoxels() const { return summaryVoxels_; }
    void setSummaryVoxels(bool value) { summaryVoxels_ = value; }
    void setSummaryVoxels(const nlohmann::json& json);

    bool writeReport() const { return writeReport_; }
    void setWriteReport(bool value) { writeReport_ = value; }
    void setWriteReport(const nlohmann::json& json);

    bool createJSON() const { return createJSON_; }
    void setCreateJSON(bool value) { createJSON_ = value; }
    void setCreateJSON(const nlohmann::json& json);

    bool createSTEP() const { return createSTEP_; }
    void setCreateSTEP(bool value) { createSTEP_ = value; }
    void setCreateSTEP(const nlohmann::json& json);

    bool createOBJ() const { return createOBJ_; }
    void setCreateOBJ(bool value) { createOBJ_ = value; }
    void setCreateOBJ(const nlohmann::json& json);

    bool storeCustomWallAttributes() const { return addCustomWallAttributes_; }
    void setStoreCustomWallAttributes(bool value) { addCustomWallAttributes_ = value; }

    bool useDefaultDiv() const { return useDefaultDiv_; }
    void setUseDefaultDiv(bool value) { useDefaultDiv_ = value; }
    void setUseDefaultDiv(const nlohmann::json& json);

    bool useProxy() const { return useProxy_; }
    void setUseProxy(bool value) { useProxy_ = value; }
    void setUseProxy(const nlohmann::json& json);

    int ignoreVoidGrade() const { return ignoreVoidGrade_; }
    void setIgnoreVoidGrade(int value) { ignoreVoidGrade_ = value; }
    void setIgnoreVoidGrade(const nlohmann::json& json); 
    
    bool simplefyGeo() const { return simplefyGeo_; }
    void setSimplefyGeo(bool value) { simplefyGeo_ = value; }
    void setSimplefyGeo(const nlohmann::json& json);

    const std::vector<std::string>& getIgnoreSimplificationList() const { return ignoreSimplificationList_; }
    void setIgnoreSimplificationList(const std::vector<std::string>& value) { ignoreSimplificationList_ = value; }
    void setIgnoreSimplificationList(const nlohmann::json& json);
    void addToIgnoreSimplificationList(const std::string& value) { ignoreSimplificationList_.emplace_back(value); }

    const std::vector<std::string>& getCustomDivList() const { return CustomDivList_; }
    void setCustomDivList(const std::vector<std::string>& value) { CustomDivList_ = value; }
    void setCustomDivList(const nlohmann::json& json);
    void addToCustomDivList(const std::string& value) { CustomDivList_.emplace_back(value); }

    const std::vector<std::string>& getDefaultDivList() const { return defaultDivList_; }

    bool mergeSemantics() const { return mergeSemantics_; }
    void setmergeSemantics(bool value) { mergeSemantics_ = value; }
    void setmergeSemantics(const nlohmann::json& json);

    int intersectionLogic() const { return intersectionLogic_; }
    void setIntersectionLogic(int value) { intersectionLogic_ = value; }
    void setIntersectionLogic(const nlohmann::json& json);

    double voxelSize() const { return voxelSize_; }
    void setVoxelSize(double value) { voxelSize_ = value; }
    void setVoxelSize(const nlohmann::json& json);

    double surfaceGridSize() const { return surfaceGridSize_; }
    void setSurfaceGridSize(double value) { surfaceGridSize_ = value; }

    int minGridPointCount() const { return minGridPointCount_; }
    void setMinGridPointCount(int value) { minGridPointCount_ = value; }

    void setRotation(const nlohmann::json& json);

    bool autoRotateGrid() const { return autoRotateGrid_; }
    void setAutoRotateGrid(bool value) { autoRotateGrid_ = value; }

    double desiredRotation() const { return desiredRotation_; }
    void setDesiredRotation(double value) { desiredRotation_ = value; }

    double footprintElevation() const { return footprintElevation_; }
    void setFootprintElevation(double value) { footprintElevation_ = value; }
    void setFootprintElevation(const nlohmann::json& json);

    double horizontalSectionOffset() const { return horizontalSectionOffset_; }
    void setHorizontalSectionOffset(double value) { horizontalSectionOffset_ = value; }
    void setHorizontalSectionOffset(const nlohmann::json& json);

    double horizontalSectionBuffer() const { return horizontalSectionBuffer_; }
    void sethorizontalSectionBuffer(double value) { horizontalSectionBuffer_ = value; }

    double spatialTolerance() const { return spatialTolerance_; }
    void setSpatialTolerance(double value) { spatialTolerance_ = value; }
    void setSpatialTolerance(const nlohmann::json& json);

    double angularTolerance() const { return angularTolerance_; }
    void setAngularTolerance(double value) { angularTolerance_ = value; }
    void setAngularTolerance(const nlohmann::json& json);

    double areaTolerance() const { return areaTolerance_; }
    void setAreaTolerance(double value) { areaTolerance_ = value; }
    void setAreaTolerance(const nlohmann::json& json);

    double maxProxyPercentage() const { return maxProxyPercentage_; }
    void setMaxProxyPercentage(double value) { maxProxyPercentage_ = value; }

    double searchBufferLod32() const { return searchBufferLoD32_; }
    void setSearchBufferLoD32(double value) { searchBufferLoD32_ = value; }

    double thinTriangleAngle() const { return thinTriangleAngle_; }
    void setThinTriangleAngle(double value) { thinTriangleAngle_ = value; }

    int threadcount() const { return threadcount_; }
    void setThreadcount(int value) { threadcount_ = value; }
    void setThreadcount(const nlohmann::json& json);

    int proxyCount() const { return proxyCount_; }
    void setProxyCount(int value) { proxyCount_ = value; }

    int objectCount() const { return objectCount_; }
    void setObjectCount(int value) { objectCount_ = value; }

    bool requireVoxels() const { return requireVoxels_; }
    void setRequireVoxels(bool value) { requireVoxels_ = value; }

    bool requireFullVoxels() const { return requireFullVoxels_; }
    void setRequireFullVoxels(bool value) { requireFullVoxels_ = value; }

    bool Lod123IsFlat() const { return Lod123IsFlat_; }
    void setLod123IsFlat(bool value) { Lod123IsFlat_ = value; }

    double gridRotation() const { return gridRotation_; }
    void setGridRotation(double value) { gridRotation_ = value; }

    std::unordered_set<std::string> getSupportedIfcVersionList() { return ifcVersionList_; }
    std::unordered_set<std::string> getOpeningObjectsList() { return openingObjects_; }
    std::unordered_set<std::string> getCuttingObjectsList() { return cuttingObjects_; }

    IfcGeom::IteratorSettings iteratorSettings(bool simple = false);
    void setIterator(const IfcGeom::IteratorSettings& settingsObject) { iteratorSettings_ = settingsObject; }
    void setSimpleIterator(const IfcGeom::IteratorSettings& settingsObject) { simpleIteratorSettings_ = settingsObject; }


    std::mutex* getWireOffsetterMutex() { return &wireOffSetterMutex_; }
};
#endif // SETTINGSCOLLECTION_SETTINGSCOLLECTION_H