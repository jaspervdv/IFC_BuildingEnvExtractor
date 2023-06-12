#include "inc/helper.h"
#include "inc/roomProcessor.h"

// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>

// boost includes
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/algorithm/string.hpp>

// openCASCADE includes
#include <TopoDS.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

// IfcOpenShell includes
#include <ifcparse/IfcFile.h>
#include <ifcparse/IfcHierarchyHelper.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#if USE_VLD
#include <vld.h>
#endif

std::vector<std::string> GetSourcePathArray() {
	std::vector<std::string> sourcePathArray = {
		//IFC 4
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/AC20-FZK-Haus.ifc"
		//"C:/Users/Jasper/DocUments/1_projects/Models_IFC/AC20-Institute-Var-2.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/AC-20-Smiley-West-10-Bldg.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Savigliano.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/FM_ARC_DigitalHub_with_SB.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/RAC_basic_sample_project_ifc4.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/RAC_basic_sample_project-ifc4_edit.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/sweco models/20220221 riederkercke-kievitsweg_gebruiksfunctie 4/20220221 riederkercke-kievitsweg_gebruiksfunctie 4.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Nijmegen/4-Daagse__Ontwerp_Start-finishlocatie_04.ifc"

		//IFC2x3
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/haviklaan-6.ifc" 
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Duplex_A_20110907_optimized.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/INPRO example KP1 demonstration_RevitArch2009.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/RAC_basic_sample_project_ifc2x3.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/CUVO_Ockenburghstraat_KOW.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/RE16_E3D_Building_2x3_Testversion.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Rabarberstraat144.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Myran_modified_Benchmark.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Myran_modified_Benchmark_Edit.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Witte_de_Withstraat_(20150508).ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Schependomlaan.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/sweco models/20415_2021-08-11_14 Woningen Traviataweg Hoogvliet/20415_2021-08-11_14 Woningen Traviataweg Hoogvliet.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/TUD/Gebouw_29/ECHO B-BWK-B-UNS-R19.IFC.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/De Raad.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/De Raad_edited.ifc"
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/15262 - 170406_Bright Rotterdam_Revit Model.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Rotterdam/BWK.ifc"//,
		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Rotterdam/STR.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Kadaster/20230320_high5_bimlegal_hoevesteijn.ifc"
		//"C:/Users/Jasper/Documents/1_projects/IFCTools/IfcSplit/Export/20230320_high5_bimlegal_hoevesteijn_2.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Rotterdam/9252_VRI_Boompjes_constructie.ifc"
	};
	return sourcePathArray;
}

std::vector<std::string> GetFileNames(std::vector<std::string>& sourcePathList) {
	std::vector<std::string> fileNames;
	for (size_t i = 0; i < sourcePathList.size(); i++)
	{
		std::vector<std::string> segments;
		boost::split(segments, sourcePathList[i], boost::is_any_of("/, \\"));
		std::string filePath = segments[segments.size() - 1];
		fileNames.emplace_back(filePath.substr(0, filePath.size() - 4));
	}
	return fileNames;
}

std::vector<std::string> GetSources() {

	// search for test input
	std::vector<std::string> sourcePathArray = GetSourcePathArray();

	// if no override is found use normal interface
	while (true)
	{
		if (sourcePathArray.size() == 0)
		{
			std::cout << "Enter filepath of the IFC file" << std::endl;
			std::cout << "[INFO] If multifile seperate by enter" << std::endl;
			std::cout << "[INFO] Finish by empty line + enter" << std::endl;

			while (true)
			{
				std::cout << "Path: ";

				std::string singlepath = "";
				getline(std::cin, singlepath);

				if (singlepath.size() == 0 && sourcePathArray.size() == 0)
				{
					std::cout << "[INFO] No filepath has been supplied" << std::endl;
					std::cout << "Enter filepath of the IFC file (if multiplefile sperate path with enter):" << std::endl;
					continue;
				}
				else if (singlepath.size() == 0)
				{
					break;
				}

				sourcePathArray.emplace_back(singlepath);
			}
		}

		bool hasError = false;

		for (size_t i = 0; i < sourcePathArray.size(); i++)
		{
			std::string currentPath = sourcePathArray[i];

			if (currentPath.size() <= 4 )
			{
				if (!hasError) { std::cout << "[ERROR] Invalid IFC file found!" << std::endl; }
				std::cout << "[INFO] Invalid file: " + currentPath << std::endl;
				hasError = true;
				break;
			}
			else if (currentPath.substr(sourcePathArray[i].length() - 4) != ".ifc") {
				if (!hasError) { std::cout << "[ERROR] Invalid IFC file found!" << std::endl; }
				std::cout << "[INFO] Invalid file: " + currentPath << std::endl;
				hasError = true;
				break;
			}
			else if (!findSchema(currentPath, true))
			{
				if (!hasError) { std::cout << "[ERROR] Invalid IFC file found!" << std::endl; }
				std::cout << "[INFO] Invalid file: " + currentPath << std::endl;
				hasError = true;
				break;
			}
		}

		std::cout << std::endl;

		if (!hasError) { break; }

		sourcePathArray.clear();
	}

	return sourcePathArray;
}

bool yesNoQuestion() {
	std::string cont = "";

	while (true)
	{
		std::cin >> cont;

		if (cont == "Y" || cont == "y") { return true; }
		if (cont == "N" || cont == "n") { return false; }
	}
}


int numQuestion(int n, bool lower = true) {
	while (true)
	{
		bool validInput = true;
		std::string stringNum = "";

		std::cout << "Num: ";
		std::cin >> stringNum;

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
				if (n >= intNum + 1 &&  intNum >= 0) {
					return intNum;
				}
			}
		}
		std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
	}
}

bool checkproxy(helperCluster* cluster) {

	double proxyCount = 0;
	double totalCount = 0;
	bool hasLot = false;

	for (size_t i = 0; i < cluster->getSize(); i++)
	{
		helper* h = cluster->getHelper(i);

		if (h->getHasProxy()) { proxyCount += h->getProxyNum(); }

		if (h->getHasLotProxy()) { hasLot = true; }

		totalCount += h->getObjectCount();
	}

	if (proxyCount == 0) { return true; }

	if (hasLot)
	{
		std::cout << "[WARNING] A large amount of IfcBuildingElementProxy objects are present in the model!" << std::endl;
	}

	std::cout << "[INFO] " << proxyCount << " of " << totalCount <<  " evaluated objects are IfcBuildingElementProxy objects" << std::endl;
	std::cout << std::endl;
	std::cout << "Continue processing? (Y/N):";
	
	bool answer = yesNoQuestion();
	
	std::cout << std::endl;
	
	return answer;
}

int main(int argc, char** argv) {
	auto startTime = std::chrono::high_resolution_clock::now();

	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	std::vector<std::string> sourcePathArray;

	bool isStandalone = true;
	std::string version = "Standalone";
	bool ignoreProxy = false;
	double voxelSize = -1;
	bool makeReport = false;

	// generate storing data
	std::vector<std::string> fileNames;
	std::string exportRootPath;

	if (argc > 1)
	{
		version = "Internal";
		isStandalone = false;
		sourcePathArray = { argv[1] };
		if (std::string(argv[3]) == "True") { ignoreProxy = true; }
		voxelSize = std::stod(argv[4]);
		fileNames = GetFileNames(sourcePathArray);
		exportRootPath = argv[2];
		if (exportRootPath[-1] != '\\' && exportRootPath[-1] != '/') { exportRootPath += "\\"; }
	}
	else {
		sourcePathArray = GetSources();
		fileNames = GetFileNames(sourcePathArray);
		if (GetSourcePathArray().size() == 0) // if test input is used test output paths can be used
		{
			std::cout << "Enter target folderpath of the CityJSON file" << std::endl;
			std::cout << "Path: ";

			std::string singlepath = "";
			getline(std::cin, singlepath);

			exportRootPath = singlepath;
			if (exportRootPath[-1] != '\\' && exportRootPath[-1] != '/') { exportRootPath += "\\"; }
			std::cout << std::endl;
		}
		else {
			exportRootPath = "C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/";
		}
	}

	// some information on startup
	std::wcout << "============================================================= \n" << std::endl;
	std::cout << "    " << version + " IFC_BuildingEnvExtractor" << std::endl;
	std::cout << "    Experimental building envelope extractor/approximation\n" << std::endl;
	std::wcout << "=============================================================" << std::endl;
	std::cout << std::endl;

	// output targets
	std::cout << "[INFO] Input file paths:" << std::endl;
	for (size_t i = 0; i < sourcePathArray.size(); i++) { std::cout << sourcePathArray[i] << std::endl; }

	std::cout << "[INFO] Output file path: " << std::endl;
	std::cout << exportRootPath + fileNames[0] + ".city.json" << std::endl;
	std::cout << std::endl;

	bool hasAskedBoundingRules = false;
	int constructionIndx = -1;
	int semanticHelper = 0;

	// get construction model num from user
	if (sourcePathArray.size() == 1)
	{
		std::cout << "[INFO] One file found, considered combination file" << std::endl;
	}
	else {

		std::cout << "Please enter number of construction model, if no constuction model enter 0." << std::endl;
		for (size_t i = 0; i < fileNames.size(); i++) { std::cout << i + 1 << ": " << fileNames[i] << std::endl; }
		constructionIndx = numQuestion(fileNames.size(), false);
	}

	std::cout << std::endl;

	// initialize helper
	helperCluster* hCluster = new helperCluster;

	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::cout << "[INFO] Parsing file " << sourcePathArray[i] << std::endl;
		helper* h = new helper(sourcePathArray[i]);
		
		if (sourcePathArray.size() > 1) { 
			h->setDepending(true); 

			// set the construction model
			if (i == constructionIndx) { h->setIsConstruct(true); }
		}
		else { h->setIsConstruct(true); }
		if (!h->hasSetUnits()) { return 0; }
		h->setName(fileNames[i]);

		hCluster->appendHelper(h);
	}

	hCluster->internaliseData();

	std::wcout << "=============================== " << std::endl;
	std::cout << "  Building Envelope Extractor" << std::endl;
	std::wcout << "=============================== \n" << std::endl;

	if (isStandalone)
	{
		if (!checkproxy(hCluster))
		{
			return 0;
		}

		std::cout << "Ignore IfcBuildingElementProxy elements?  (Y/N):";
		hCluster->setUseProxy(!yesNoQuestion());
		std::cout << std::endl;
	}
	else if (ignoreProxy == 0)
	{
		hCluster->setUseProxy(true);
	}
	else if (ignoreProxy == 1)
	{
		hCluster->setUseProxy(false);
	}

	if (isStandalone)
	{
		std::cout << "Make report file? (Y/N):";
		if (yesNoQuestion()) { makeReport = true; }
	}
	else
	{
		makeReport = true;
	}


	// indexation of geometry
	for (int i = 0; i < hCluster->getSize(); i++)
	{
		hCluster->getHelper(i)->indexGeo();
	}

	auto startTimeLod = std::chrono::high_resolution_clock::now();
	CJGeoCreator* geoCreator = new CJGeoCreator(hCluster, voxelSize);

	auto internalizingTime = std::chrono::high_resolution_clock::now();
	CJT::CityCollection* collection = new CJT::CityCollection;
	CJT::ObjectTransformation transformation(0.001);
	CJT::metaDataObject* metaData = new CJT::metaDataObject;
	metaData->setTitle(hCluster->getHelper(semanticHelper)->getName() +  " Auto export from IfcEnvExtractor");
	collection->setTransformation(transformation);
	collection->setMetaData(metaData);
	collection->setVersion("1.1");

	CJT::Kernel* kernel = new CJT::Kernel(collection);

	CJT::CityObject* cityObject = new CJT::CityObject;

	std::string BuildingName = hCluster->getHelper(semanticHelper)->getBuildingName();
	if (BuildingName == "")
	{
		BuildingName = hCluster->getHelper(semanticHelper)->getProjectName();
	}

	cityObject->setName(BuildingName);
	cityObject->setType(CJT::Building_Type::Building);
	
	std::map<std::string, std::string> buildingAttributes = hCluster->getHelper(semanticHelper)->getBuildingInformation();
	for (std::map<std::string, std::string>::iterator iter = buildingAttributes.begin(); iter != buildingAttributes.end(); ++iter) { cityObject->addAttribute(iter->first, iter->second); }

	auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
	CJT::GeoObject* geo00 = geoCreator->makeLoD00(hCluster, collection, kernel, 1);
	cityObject->addGeoObject(geo00);
	auto geo00Time = std::chrono::high_resolution_clock::now();
	std::vector<CJT::GeoObject*> geo02 = geoCreator->makeLoD02(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo02.size(); i++) { cityObject->addGeoObject(geo02[i]); }
	auto geo02Time = std::chrono::high_resolution_clock::now();
	CJT::GeoObject* geo10 = geoCreator->makeLoD10(hCluster, collection, kernel, 1);
	cityObject->addGeoObject(geo10);
	auto geo10Time = std::chrono::high_resolution_clock::now();
	std::vector<CJT::GeoObject*> geo12 = geoCreator->makeLoD12(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo12.size(); i++) { cityObject->addGeoObject(geo12[i]); }
	auto geo12Time = std::chrono::high_resolution_clock::now();
	std::vector<CJT::GeoObject*> geo13 = geoCreator->makeLoD13(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo13.size(); i++) { cityObject->addGeoObject(geo13[i]); }
	auto geo13Time = std::chrono::high_resolution_clock::now();
	std::vector<CJT::GeoObject*> geo22 = geoCreator->makeLoD22(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo22.size(); i++) { cityObject->addGeoObject(geo22[i]); }
	auto geo22Time = std::chrono::high_resolution_clock::now();
	std::vector<CJT::GeoObject*> geo32 = geoCreator->makeLoD32(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo32.size(); i++) { cityObject->addGeoObject(geo32[i]); }
	auto geo32Time = std::chrono::high_resolution_clock::now();
	collection->addCityObject(cityObject);
	collection->CleanVertices();
	collection->dumpJson(exportRootPath + fileNames[0] + ".city.json");
	auto exportTime = std::chrono::high_resolution_clock::now();

	delete kernel;
	delete collection;
	delete metaData;
	delete cityObject;

	std::cout << std::endl;
	std::cout << std::endl;
	auto endTime = std::chrono::high_resolution_clock::now();

	if (makeReport) 
	{
		// TODO: make function
		nlohmann::json report;
		addTimeToJSON(&report, "Time internalizing", startTimeLod, internalizingTime);
		addTimeToJSON(&report, "Time LoD0.0 generation", startTimeGeoCreation, geo00Time);
		addTimeToJSON(&report, "Time LoD0.2 generation", geo00Time, geo02Time);
		addTimeToJSON(&report, "Time LoD1.0 generation", geo02Time, geo10Time);
		addTimeToJSON(&report, "Time LoD1.2 generation", geo10Time, geo12Time);
		addTimeToJSON(&report, "Time LoD1.3 generation", geo12Time, geo13Time);
		addTimeToJSON(&report, "Time LoD2.2 generation", geo13Time, geo22Time);
		addTimeToJSON(&report, "Time LoD3.2 generation", geo22Time, geo32Time);
		addTimeToJSON(&report, "Total Processing time", startTimeLod, endTime);
		addTimeToJSON(&report, "Total running time", startTime, endTime);
		std::ofstream reportFile(exportRootPath + fileNames[0] + "_report.city.json");
		reportFile << report;
		reportFile.close();
		return 0;
	}

	std::cout << "Computing Time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTimeLod).count() << std::endl;
	std::cout << "Total Process Time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << std::endl;
	std::cout << "[INFO] process has been succesfully executed" << std::endl;

	return 0;
}