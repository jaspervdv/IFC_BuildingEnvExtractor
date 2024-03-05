#include "inc/helper.h"
#include "inc/IOManager.h"
#include "inc/cjCreator.h"

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



int main(int argc, char** argv) {
	auto startTime = std::chrono::high_resolution_clock::now();
	std::string issueEncounterString = "[INFO] encountered an issue";

	// outputs errors related to the selected objects
	if (false) { Logger::SetOutput(&std::cout, &std::cout); }

	IOManager manager;
	bool success = false;
	if (argc > 1) {
		try
		{
			success = manager.init({ argv[1] });
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
			success = manager.init({}, false);
		}
		catch (const std::string& exceptionString)
		{
			std::cout << issueEncounterString << std::endl;
			std::cout << exceptionString << std::endl;
			return 0;
		}
	}
	if (!success)
	{
		std::cout << "[WARNING] unable to process file(s)" << std::endl;
		return 1;
	}

	if (!manager.run())
	{
		std::cout << "[WARNING] unable to process file(s)" << std::endl;
		manager.write();
		return 1;
	}
	manager.write();
	
	return 0;
}