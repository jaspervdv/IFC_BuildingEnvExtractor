#include "inc/helper.h"
#include "inc/IOManager.h"
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
	std::string issueEncounterString = "[INFO] encountered an issue";

	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	IOManager manager;
	if (argc > 1) {
		try
		{
			manager.init({ argv[1] });
		}
		catch (const std::string& exceptionString)
		{
			std::cout << issueEncounterString << std::endl;
			std::cout << exceptionString << std::endl;
			return 0;
		}
	}
	else { 
		try
		{
			manager.init(GetSourcePathArray(), false);
		}
		catch (const std::string& exceptionString)
		{
			std::cout << issueEncounterString << std::endl;
			std::cout << exceptionString << std::endl;
			return 0;
		}
	}
	manager.run();
	manager.write();
	
	return 0;
}