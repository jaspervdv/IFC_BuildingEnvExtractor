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

std::vector<std::string> GetSources() {

	// easy override 
	std::vector<std::string> sourcePathArray = {
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/AC20-FZK-Haus.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Witte_de_Withstraat_(20150508).ifc"
	"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Savigliano.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Myran_modified_Benchmark.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Ken_models/Rabarberstraat144.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/AC20-Institute-Var-2.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Schependomlaan.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/AC-20-Smiley-West-10-Bldg.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/FM_ARC_DigitalHub_with_SB.ifc"
	//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Revit_Example_Models/RAC_basic_sample_project_ifc4.ifc"
	};

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

		if (!hasError)
		{
			break;
		}

		sourcePathArray.clear();
	}

	return sourcePathArray;
}

void compareElevationsOutput(const std::vector<double> left, const std::vector<double> right) {
	int tab = 35;
	if (left.size() != right.size())
	{
		if (left.size() > right.size())
		{
			int minIndx = (int) right.size();
			for (size_t i = 0; i < left.size(); i++)
			{
				if (i < minIndx) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
				else { std::cout << std::left << std::setw(tab) << left[i] << "-" << std::endl; }
			}
		}
		else {
			int minIndx = (int) left.size();
			for (size_t i = 0; i < right.size(); i++)
			{
				if (i < minIndx) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
				else { std::cout << std::left << std::setw(tab) << "-" << right[i] << std::endl; }
			}
		}
	}
	else
	{
		for (size_t i = 0; i < right.size(); i++) { std::cout << std::left << std::setw(tab) << left[i] << right[i] << std::endl; }
	}
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

double numQuestionD(int n, bool lower = true) {
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
			double intNum = std::stod(stringNum);
			if (!lower)
			{
				if (n >= intNum) {
					return intNum;
				}
			}
			else if (lower)
			{
				if (n >= intNum && intNum >= 0) {
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
		//std::cout << "[INFO] Process is terminated" << std::endl;
		//return false;
	}

	std::cout << "[INFO] " << proxyCount << " of " << totalCount <<  " evaluated objects are IfcBuildingElementProxy objects" << std::endl;
	std::cout << std::endl;
	std::cout << "Continue processing? (Y/N):";
	
	bool answer = yesNoQuestion();
	
	std::cout << std::endl;
	
	return answer;
}


void askBoudingRules(helperCluster* hCluster) {
	std::cout << "Please select a desired rulset for room bounding objects" << std::endl;
	std::cout << "1. Default room bounding objects" << std::endl;
	std::cout << "2. Default room bounding objects + IfcBuildingElementProxy objects" << std::endl;
	std::cout << "3. Default room bounding objects + custom object selection" << std::endl;
	std::cout << "4. custom object selection" << std::endl;

	int ruleNum = numQuestion(4);

	if (ruleNum == 1)
	{
		hCluster->setUseProxy(true);
	}
	if (ruleNum == 2 || ruleNum == 3)
	{
		bool fCustom = false;
		std::vector<std::string> defaultList = {
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

		std::list<std::string> sourceTypeList = hCluster->getObjectList();
		std::list<std::string>* objectList = new std::list<std::string>;

		std::cout << std::endl;
		std::cout << "Please enter the desired IfcTypes" << std::endl;
		std::cout << "[INFO] Not case sensitive" << std::endl;
		std::cout << "[INFO] Seperate type by enter" << std::endl;
		std::cout << "[INFO] Finish by empty line + enter" << std::endl;

		if (ruleNum == 3)
		{
			fCustom = true;
			defaultList = {};
		}

		int i = 0;
		while (true)
		{
			std::cout << "IfcType: ";

			std::string singlepath = "";

			if (i == 0)
			{
				std::cin.ignore();
				i++;
			}

			getline(std::cin, singlepath);

			if (singlepath.size() == 0 && objectList->size() == 0)
			{
				std::cout << "[INFO] No type has been supplied" << std::endl;
				continue;
			}
			else if (singlepath.size() == 0)
			{
				break;
			}
			else if (boost::to_upper_copy<std::string>(singlepath.substr(0, 3)) == "IFC") {

				std::string potentialType = boost::to_upper_copy<std::string>(singlepath);

				bool defaultType = false;
				for (size_t i = 0; i < defaultList.size(); i++)
				{
					if (potentialType == defaultList[i])
					{
						defaultType = true;
						std::cout << "[INFO] Type is present in default set" << std::endl;

						break;
					}

				}

				if (defaultType) { 
					continue;
				}

				bool found = false;

				for (auto it = sourceTypeList.begin(); it != sourceTypeList.end(); ++it)
				{
					if (*it == potentialType)
					{
						objectList->emplace_back(boost::to_upper_copy<std::string>(singlepath));
						found = true;
					}
				}
				if (!found)
				{
					std::cout << "[INFO] Type is not present in file" << std::endl;
					continue;
				}
			}
			else
			{
				std::cout << "[INFO] No valid type has been supplied" << std::endl;
				continue;
			}
		}

		for (size_t i = 0; i < hCluster->getSize(); i++)
		{
			hCluster->getHelper(i)->setRoomBoundingObjects(objectList, true, fCustom);
		}
	}
	std::cout << std::endl;
}

void askApartmentRules(helperCluster* hCluster) {
	std::cout << "Please select a desired rulset" << std::endl;
	std::cout << "1. Default apartment construction rules" << std::endl;
	std::cout << "2. Custom apartment construction rules" << std::endl;

	int ruleNum = numQuestion(2);

	if (ruleNum == 1)
	{
		std::cout << std::endl;
		std::cout << "Minimal apartment room count (int)" << std::endl;
		int roomCount = numQuestion(15) + 1;

		std::cout << std::endl;
		std::cout << "Minimal apartment area size (double)" << std::endl;
		double apArea = numQuestionD(1000);

		std::cout << std::endl;
		std::cout << "Minimal connections needed to be considered splitting point (int)" << std::endl;;
		int conCount = numQuestion(15) + 1;

		hCluster->setApRules(conCount, roomCount, apArea);
	}
	std::cout << std::endl;
}


int main(int argc, char** argv) {

	// some information on startup
	std::wcout << "============================================================= \n" << std::endl;
	std::cout << "    IFC_BuildingEnvExtractor" << std::endl;
	std::cout << "    Experimental building envelope extractor/approximation\n" << std::endl;
	std::wcout << "=============================================================" << std::endl;


	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	std::vector<std::string> sourcePathArray = GetSources();
	bool hasAskedBoundingRules = false;

	// make export path
	std::vector<std::string> exportPathArray;
	std::string graphPath;
	std::vector<std::string> fileNames;
	for (size_t i = 0; i < sourcePathArray.size(); i++)
	{
		std::string exportPath;
		std::vector<std::string> segments;
		boost::split(segments, sourcePathArray[i], boost::is_any_of("/"));

		for (size_t i = 0; i < segments.size()-1; i++) { exportPath += segments[i] + "/"; }

		graphPath = exportPath + "exports/Exported_" + segments[segments.size() - 1];
		graphPath.erase(graphPath.length() - 4);

		fileNames.emplace_back(segments[segments.size() - 1]);
		exportPath += "exports/Exported_" + segments[segments.size() - 1];
		exportPathArray.emplace_back(exportPath);
	}

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

	if (!checkproxy(hCluster))
	{
		return 0;
	}

	std::wcout << "=============================== " << std::endl;
	std::cout << "  Building Envelope Extractor" << std::endl;
	std::wcout << "=============================== \n" << std::endl;

	if (sourcePathArray.size() != 1)
	{
		while (true)
		{
			bool validInput = true;
			std::string stringNum = "";

			std::cout << "Please enter number of the target model where the envelope is to be stored" << std::endl;
			for (size_t i = 0; i < fileNames.size(); i++) { std::cout << i + 1 << ": " << fileNames[i] << std::endl; }
			std::cout << "Num: ";
			std::cin >> stringNum;
			std::cout << std::endl;

			for (size_t i = 0; i < stringNum.size(); i++)
			{
				if (!std::isdigit(stringNum[i]))
				{
					validInput = false;
				}
			}

			if (validInput)
			{
				int roomIndx = std::stoi(stringNum) - 1;

				if (roomIndx >= 0) {
					hCluster->getHelper(roomIndx)->setHasRooms();
					break;
				}
			}
			std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
		}
	}
	else {
		hCluster->getHelper(0)->setHasRooms();
	}

	// indexation of geometry
	for (int i = 0; i < hCluster->getSize(); i++)
	{
		hCluster->getHelper(i)->indexGeo();
	}


	auto startTime = std::chrono::high_resolution_clock::now();
	CJGeoCreator* geoCreator = new CJGeoCreator(hCluster);

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

	CJT::GeoObject* geo00 = geoCreator->makeLoD00(hCluster, collection, kernel, 1);
	cityObject->addGeoObject(geo00);
	std::vector<CJT::GeoObject*> geo02 = geoCreator->makeLoD02(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo02.size(); i++) { cityObject->addGeoObject(geo02[i]); }
	CJT::GeoObject* geo10 = geoCreator->makeLoD10(hCluster, collection, kernel, 1);
	cityObject->addGeoObject(geo10);
	std::vector<CJT::GeoObject*> geo12 = geoCreator->makeLoD12(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo12.size(); i++) { cityObject->addGeoObject(geo12[i]); }
	std::vector<CJT::GeoObject*> geo13 = geoCreator->makeLoD13(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo13.size(); i++) { cityObject->addGeoObject(geo13[i]); }
	std::vector<CJT::GeoObject*> geo22 = geoCreator->makeLoD22(hCluster, collection, kernel, 1);
	for (size_t i = 0; i < geo22.size(); i++) { cityObject->addGeoObject(geo22[i]); }
	//CJT::GeoObject* geo32 = geoCreator->makeLoD32(hCluster, collection, kernel, 1);
	//cityObject->addGeoObject(geo32);

	collection->addCityObject(cityObject);
	collection->dumpJson("C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/" + BuildingName + ".city.json");

	delete kernel;
	delete collection;
	delete metaData;
	delete cityObject;

	std::cout << std::endl;
	std::cout << std::endl;


	auto endTime = std::chrono::high_resolution_clock::now();
	std::cout << "computing time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << std::endl;
	std::cout << "[INFO] process has been succesfully executed" << std::endl;

	return 0;
}