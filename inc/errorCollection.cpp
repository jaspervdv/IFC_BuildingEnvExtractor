#include "errorCollection.h"
#include "stringManager.h"

#include <map>
#include <nlohmann/json.hpp>
#include <iostream>

ErrorObject::ErrorObject(const std::string& errorCode, const std::string& errorDescript)
{
	errorCode_ = errorCode;
	errorDescript_ = errorDescript;
	occuringObjectList_ = {};
}

ErrorObject::ErrorObject(const std::string& errorCode, const std::string& errorDescript, const std::string& occuringObb)
{
	errorCode_ = errorCode;
	errorDescript_ = errorDescript;
	occuringObjectList_ = { occuringObb };
}

ErrorObject::ErrorObject(const std::string& errorCode, const std::string& errorDescript, const std::vector<std::string>& occuringObbList)
{
	errorCode_ = errorCode;
	errorDescript_ = errorDescript;
	occuringObjectList_ = occuringObbList;
}

nlohmann::json ErrorObject::toJson()
{
	nlohmann::json jsonObject;
	jsonObject["ErrorCode"] = errorCode_;
	jsonObject["Error Description"] = errorDescript_;

	if (occuringObjectList_.size()) { jsonObject["Occurring Objects"] = occuringObjectList_; }
	return jsonObject;
}

void ErrorObject::addOccuringObject(const std::string& obb) {
	if (std::find(occuringObjectList_.begin(), occuringObjectList_.end(), obb) != occuringObjectList_.end()) { return;}
	occuringObjectList_.emplace_back(obb);
	return;
}

ErrorCollection::ErrorCollection() {
	errorCollection_ = {};
	errorMap_ = {
		{ErrorID::errorNoValFilePaths, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorNoValFilePaths, false))},

		{ErrorID::errorUnableToProcessFile, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorUnableToProcessFile, false))},
		{ErrorID::errorNoUnits, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorNoUnits, false))},
		{ErrorID::warningMultipleUnits, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::warningMultipleUnits, false))},
		{ErrorID::errorNoLengthUnit, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorNoLengthUnit, false))},
		{ErrorID::errorNoAreaUnit, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorNoAreaUnit, false))},

		{ErrorID::errorJsonInvalBool, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorJsonInvalBool, false))},
		{ErrorID::errorJsonInvalInt, ErrorObject("J0001", errorWarningStringEnum::getString(ErrorID::errorJsonInvalArray, false))},
		{ErrorID::errorJsonInvalNegInt, ErrorObject("J0002", errorWarningStringEnum::getString(ErrorID::errorJsonInvalNegInt, false))},
		{ErrorID::errorJsonInvalZeroInt, ErrorObject("J0002", errorWarningStringEnum::getString(ErrorID::errorJsonInvalZeroInt, false))},
		{ErrorID::errorJsonInvalNum, ErrorObject("J0003", errorWarningStringEnum::getString(ErrorID::errorJsonInvalNum, false))},
		{ErrorID::errorJsonInvalString, ErrorObject("J0004", errorWarningStringEnum::getString(ErrorID::errorJsonInvalString, false))},
		{ErrorID::errorJsonInvalPath, ErrorObject("J0005", errorWarningStringEnum::getString(ErrorID::errorJsonInvalPath, false))},
		{ErrorID::errorJsonNoRealPath, ErrorObject("J0006", errorWarningStringEnum::getString(ErrorID::errorJsonNoRealPath, false))},
		{ErrorID::errorJsonInvalArray, ErrorObject("J0007", errorWarningStringEnum::getString(ErrorID::errorJsonInvalArray, false))},

		{ErrorID::errorJsonInvalEntry, ErrorObject("J0008", errorWarningStringEnum::getString(ErrorID::errorJsonInvalEntry, false))},
		{ErrorID::errorJsonInvalidLogic, ErrorObject("J0010", errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLogic, false))},

		{ErrorID::errorJsonInvalidLod, ErrorObject("J0011", errorWarningStringEnum::getString(ErrorID::errorJsonInvalidLod, false))},
		{ErrorID::errorJsonNoDivObjects, ErrorObject("J0011", errorWarningStringEnum::getString(ErrorID::errorJsonNoDivObjects, false))},

		{ErrorID::errorNoPoints, ErrorObject("P0001", errorWarningStringEnum::getString(ErrorID::errorNoPoints, false))},
		{ErrorID::errorFootprintFailed, ErrorObject("P0002", errorWarningStringEnum::getString(ErrorID::errorFootprintFailed, false))},
		{ErrorID::errorStoreyFailed, ErrorObject("P0003", errorWarningStringEnum::getString(ErrorID::errorStoreyFailed, false))},
		{ErrorID::errorLoD02StoreyFailed, ErrorObject("P0004", errorWarningStringEnum::getString(ErrorID::errorLoD02StoreyFailed, false))},
		{ErrorID::warningFailedObjectSimplefication, ErrorObject("P0005", errorWarningStringEnum::getString(ErrorID::warningFailedObjectSimplefication, false))},
		{ErrorID::warningFailedObjectConversion, ErrorObject("P0006", errorWarningStringEnum::getString(ErrorID::warningFailedObjectConversion, false))},
		{ErrorID::errorFailedInit, ErrorObject("P1000", errorWarningStringEnum::getString(ErrorID::errorFailedInit, false))},

		{ErrorID::warningIfcUnableToParse, ErrorObject("I0001", errorWarningStringEnum::getString(ErrorID::warningIfcUnableToParse, false))},
		{ErrorID::warningIfcNotValid, ErrorObject("I0002", errorWarningStringEnum::getString(ErrorID::warningIfcNotValid, false))},
		{ErrorID::warningIfcNoSchema, ErrorObject("I0003", errorWarningStringEnum::getString(ErrorID::warningIfcNoSchema, false))},
		{ErrorID::warningIfcIncomp, ErrorObject("I0004", errorWarningStringEnum::getString(ErrorID::warningIfcIncomp, false))},
		{ErrorID::warningIfcNoSlab, ErrorObject("I0005", errorWarningStringEnum::getString(ErrorID::warningIfcNoSlab, false))},
		{ErrorID::warningIfcMultipleProjections, ErrorObject("I0006", errorWarningStringEnum::getString(ErrorID::warningIfcMultipleProjections, false))},
		{ErrorID::warningIfcNoVolumeUnit, ErrorObject("I0007", errorWarningStringEnum::getString(ErrorID::warningIfcNoVolumeUnit, false))},
		{ErrorID::warningIfcDubSites, ErrorObject("I0008", errorWarningStringEnum::getString(ErrorID::warningIfcDubSites, false))},
		{ErrorID::warningIfcNoSites, ErrorObject("I0009", errorWarningStringEnum::getString(ErrorID::warningIfcNoSites, false))},
		{ErrorID::warningIfcSiteReconstructionFailed, ErrorObject("I0010", errorWarningStringEnum::getString(ErrorID::warningIfcSiteReconstructionFailed, false))},
		{ErrorID::warningIfcNoRoomObjects, ErrorObject("I0011", errorWarningStringEnum::getString(ErrorID::warningIfcNoRoomObjects, false))},
		{ErrorID::warningIfcMultipleUniqueObjects, ErrorObject("I0012", errorWarningStringEnum::getString(ErrorID::warningIfcMultipleUniqueObjects, false))},
		{ErrorID::warningIfcNoObjectName, ErrorObject("I0013", errorWarningStringEnum::getString(ErrorID::warningIfcNoObjectName, false))},
		{ErrorID::warningIfcNoObjectNameLong, ErrorObject("I0014", errorWarningStringEnum::getString(ErrorID::warningIfcNoObjectNameLong, false))},
		{ErrorID::warningIfcObjectDifferentName, ErrorObject("I0015", errorWarningStringEnum::getString(ErrorID::warningIfcObjectDifferentName, false))},
		{ErrorID::warningIfcMissingGeoreference, ErrorObject("I0016", errorWarningStringEnum::getString(ErrorID::warningIfcMissingGeoreference, false))},

		{ErrorID::warningIssueencountered, ErrorObject("I0017", errorWarningStringEnum::getString(ErrorID::warningIssueencountered, false))},
		{ErrorID::warningNoSolid, ErrorObject("I0018", errorWarningStringEnum::getString(ErrorID::warningNoSolid, false))},
		{ErrorID::warningUnableToMesh, ErrorObject("I0019", errorWarningStringEnum::getString(ErrorID::warningUnableToMesh, false))},
		{ErrorID::warningUnableToSimplefy, ErrorObject("I0020", errorWarningStringEnum::getString(ErrorID::warningUnableToSimplefy, false))},
		{ErrorID::warningUnableToExtrude, ErrorObject("I0021", errorWarningStringEnum::getString(ErrorID::warningUnableToSimplefy, false))},
		{ErrorID::warningNoRoofOutline, ErrorObject("I0022", errorWarningStringEnum::getString(ErrorID::warningNoRoofOutline, false))},
		{ErrorID::warningNoFootprint, ErrorObject("I0023", errorWarningStringEnum::getString(ErrorID::warningNoFootprint, false))},

		{ErrorID::warningUnableToSimplefy, ErrorObject("I0100", errorWarningStringEnum::getString(ErrorID::warningInputIncFootprintElev, false))},

		{ErrorID::warningSimplefication, ErrorObject("N0001", errorWarningStringEnum::getString(ErrorID::warningSimplefication, false))},

		{ErrorID::failedLoD00, ErrorObject("E0001", "LoD0.0 creation failed")},
		{ErrorID::failedLoD02, ErrorObject("E0002", "LoD0.2 creation failed")},
		{ErrorID::failedLoD03, ErrorObject("E0003", "LoD0.3 creation failed")},
		{ErrorID::failedLoD04, ErrorObject("E0004", "LoD0.4 creation failed")},
		{ErrorID::failedLoD10, ErrorObject("E0010", "LoD1.0 creation failed")},
		{ErrorID::failedLoD12, ErrorObject("E0012", "LoD1.2 creation failed")},
		{ErrorID::failedLoD13, ErrorObject("E0013", "LoD1.3 creation failed")},
		{ErrorID::failedLoD22, ErrorObject("E0022", "LoD2.2 creation failed")},
		{ErrorID::failedLoDb0, ErrorObject("E0040", "LoDb.0 creation failed")},
		{ErrorID::failedLoDc1, ErrorObject("E0041", "LoDc.1 creation failed")},
		{ErrorID::failedLoDc2, ErrorObject("E0042", "LoDc.2 creation failed")},
		{ErrorID::failedLoDd1, ErrorObject("E0043", "LoDd.1 creation failed")},
		{ErrorID::failedLoDd2, ErrorObject("E0044", "LoDd.2 creation failed")},
		{ErrorID::failedLoD32, ErrorObject("E0032", "LoD3.2 creation failed")},
		{ErrorID::failedLoD50, ErrorObject("E0042", "Voxel shape creation failed")},

		{ErrorID::propertyNotImplemented, ErrorObject("P0000", "IFC file has attribute property that is not supported")}
	};
}

void ErrorCollection::addError(ErrorID id, const std::string& objectName) 
{
	std::lock_guard<std::mutex> errorLock(dataMutex_);
	//search if error is present ignore or add object
	if (errorCollection_.find(id) != errorCollection_.end())
	{
		if (objectName != "")
		{
			ErrorObject errorObject = errorCollection_[id];
			errorObject.addOccuringObject(objectName);
			errorCollection_[id] = errorObject;
		}
		return;
	}

	// new error and add object
	if (objectName != "")
	{
		ErrorObject errorObject = errorMap_[id];
		errorObject.addOccuringObject(objectName);

		errorCollection_[id] = errorObject;
		return;
	}

	// add normal error object if does not exist yet
	errorCollection_[id] = errorMap_[id];
	return;
}

void ErrorCollection::addError(ErrorID id, const std::vector<std::string>& objectNameList) {
	//search if error is present
	if (errorCollection_.find(id) != errorCollection_.end())
	{
		if (objectNameList.size())
		{
			ErrorObject errorObject = errorCollection_[id];

			for (const std::string& objectName : objectNameList)
			{
				errorObject.addOccuringObject(objectName);
			}
			errorCollection_[id] = errorObject;
		}
		return;
	}

	if (objectNameList.size())
	{
		ErrorObject errorObject = errorMap_[id];
		for (const std::string& objectName : objectNameList)
		{
			errorObject.addOccuringObject(objectName);
		}
		errorCollection_[id] = errorObject;
		return;
	}
	addError(id);
	return;
}


bool ErrorCollection::hasError()
{
	if (errorCollection_.empty()) { return false; }
	return true;
}

nlohmann::json ErrorCollection::toJson() {
	
	nlohmann::json jsonList = nlohmann::json::array();
	for (std::pair<ErrorID, ErrorObject> errorPair : errorCollection_)
	{
		jsonList.emplace_back(errorPair.second.toJson());
	}
	return jsonList;
}
