#include "inc/helper.h"
#include "inc/roomProcessor.h"

// basic includes
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>


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
		// "C:/Users/Jasper/Documents/1_projects/Models_IFC/Schependomlaan.ifc"

		//"C:/Users/Jasper/Documents/1_projects/Models_IFC/Siham/Model_IFC.ifc"

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

int main(int argc, char** argv) {
	auto startTime = std::chrono::high_resolution_clock::now();

	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	IOManager manager;
	if (argc > 1) {
		if (!manager.init({ argv[1] }, true))
		{
			std::cout << "encountered an issue" << std::endl;
			return 0;
		}
	}
	else { 
		if (!manager.init(GetSourcePathArray(), false))
		{
			std::cout << "encountered an issue" << std::endl;
			return 0;
		}
	}
	manager.run();

	auto startTimeLod = std::chrono::high_resolution_clock::now();
	CJGeoCreator* geoCreator = new CJGeoCreator(&manager.helpCluster(), manager.voxelSize());

	auto internalizingTime = std::chrono::high_resolution_clock::now();
	CJT::CityCollection* collection = new CJT::CityCollection;
	CJT::ObjectTransformation transformation(0.001);
	CJT::metaDataObject* metaData = new CJT::metaDataObject;
	metaData->setTitle(manager.helpCluster().getHelper(0)->getName() +  " Auto export from IfcEnvExtractor");
	collection->setTransformation(transformation);
	collection->setMetaData(metaData);
	collection->setVersion("1.1");

	CJT::Kernel* kernel = new CJT::Kernel(collection);

	CJT::CityObject* cityObject = new CJT::CityObject;

	std::string BuildingName = manager.helpCluster().getHelper(0)->getBuildingName();
	if (BuildingName == "")
	{
		BuildingName = manager.helpCluster().getHelper(0)->getProjectName();
	}

	cityObject->setName(BuildingName);
	cityObject->setType(CJT::Building_Type::Building);
	
	std::map<std::string, std::string> buildingAttributes = manager.helpCluster().getHelper(0)->getBuildingInformation();
	for (std::map<std::string, std::string>::iterator iter = buildingAttributes.begin(); iter != buildingAttributes.end(); ++iter) { cityObject->addAttribute(iter->first, iter->second); }

	auto startTimeGeoCreation = std::chrono::high_resolution_clock::now();
	if (manager.makeLoD00())
	{
		CJT::GeoObject* geo00 = geoCreator->makeLoD00(&manager.helpCluster(), collection, kernel, 1);
		cityObject->addGeoObject(geo00);
	}
	if (manager.makeLoD02())
	{
		std::vector<CJT::GeoObject*> geo02 = geoCreator->makeLoD02(&manager.helpCluster(), collection, kernel, 1);
		for (size_t i = 0; i < geo02.size(); i++) { cityObject->addGeoObject(geo02[i]); }
	}
	if (manager.makeLoD10())
	{
		CJT::GeoObject* geo10 = geoCreator->makeLoD10(&manager.helpCluster(), collection, kernel, 1);
		cityObject->addGeoObject(geo10);
	}
	if (manager.makeLoD12())
	{
		std::vector<CJT::GeoObject*> geo12 = geoCreator->makeLoD12(&manager.helpCluster(), collection, kernel, 1);
		for (size_t i = 0; i < geo12.size(); i++) { cityObject->addGeoObject(geo12[i]); }
	}
	if (manager.makeLoD13())
	{
		std::vector<CJT::GeoObject*> geo13 = geoCreator->makeLoD13(&manager.helpCluster(), collection, kernel, 1);
		for (size_t i = 0; i < geo13.size(); i++) { cityObject->addGeoObject(geo13[i]); }
	}
	if (manager.makeLoD22())
	{
		std::vector<CJT::GeoObject*> geo22 = geoCreator->makeLoD22(&manager.helpCluster(), collection, kernel, 1);
		for (size_t i = 0; i < geo22.size(); i++) { cityObject->addGeoObject(geo22[i]); }
	}
	if (manager.makeLoD32() && false)
	{
		std::vector<CJT::GeoObject*> geo32 = geoCreator->makeLoD32(&manager.helpCluster(), collection, kernel, 1);
		for (size_t i = 0; i < geo32.size(); i++) { cityObject->addGeoObject(geo32[i]); }
	}

	//auto geo00Time = std::chrono::high_resolution_clock::now();
	//auto geo02Time = std::chrono::high_resolution_clock::now();
	//auto geo10Time = std::chrono::high_resolution_clock::now();
	//auto geo12Time = std::chrono::high_resolution_clock::now();
	//auto geo13Time = std::chrono::high_resolution_clock::now();
	//auto geo22Time = std::chrono::high_resolution_clock::now();
	//auto geo32Time = std::chrono::high_resolution_clock::now();
	collection->addCityObject(cityObject);
	collection->CleanVertices();
	collection->dumpJson(manager.getOutputPath() + "\\" + manager.helpCluster().getHelper(0)->getName() + ".city.json");
	auto exportTime = std::chrono::high_resolution_clock::now();

	delete kernel;
	delete collection;
	delete metaData;
	delete cityObject;

	//std::cout << std::endl;
	//std::cout << std::endl;
	//auto endTime = std::chrono::high_resolution_clock::now();

	//if (manager.makeReport()) 
	//{
	//	// TODO: make function
	//	nlohmann::json report;
	//	addTimeToJSON(&report, "Time internalizing", startTimeLod, internalizingTime);
	//	addTimeToJSON(&report, "Time LoD0.0 generation", startTimeGeoCreation, geo00Time);
	//	addTimeToJSON(&report, "Time LoD0.2 generation", geo00Time, geo02Time);
	//	addTimeToJSON(&report, "Time LoD1.0 generation", geo02Time, geo10Time);
	//	addTimeToJSON(&report, "Time LoD1.2 generation", geo10Time, geo12Time);
	//	addTimeToJSON(&report, "Time LoD1.3 generation", geo12Time, geo13Time);
	//	addTimeToJSON(&report, "Time LoD2.2 generation", geo13Time, geo22Time);
	//	addTimeToJSON(&report, "Time LoD3.2 generation", geo22Time, geo32Time);
	//	addTimeToJSON(&report, "Total Processing time", startTimeLod, endTime);
	//	addTimeToJSON(&report, "Total running time", startTime, endTime);
	//	std::ofstream reportFile(manager.getOutputPath() + manager.helpCluster().getHelper(0)->getName() + "_report.city.json");
	//	reportFile << report;
	//	reportFile.close();
	//	return 0;
	//}

	//std::cout << "Computing Time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTimeLod).count() << std::endl;
	//std::cout << "Total Process Time = " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << std::endl;
	//std::cout << "[INFO] process has been succesfully executed" << std::endl;
	
	return 0;
}