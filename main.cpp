#include "inc/helper.h"
#include "inc/IOManager.h"
#include "inc/cjCreator.h"
#include "inc/stringManager.h"

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
#include <ifcgeom_schema_agnostic/Kernel.h>

#if USE_VLD
#include <vld.h>
#endif

int main(int argc, char** argv) {
	std::cout << " " << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();
	std::string issueEncounterString = CommunicationStringEnum::getString(CommunicationStringID::warmingIssueencountered);

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
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::errorUnableToProcessFile) << std::endl;
		return 1;
	}

	if (!manager.run())
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::errorUnableToProcessFile) << std::endl;
		manager.write();
		return 1;
	}
	manager.write();	
	return 0;
}