#include "errorCollection.h"

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

	if (occuringObjectList_.size()) { jsonObject["Occuring Objects"] = occuringObjectList_; }
	return jsonObject;
}

void ErrorObject::addOccuringObject(const std::string& obb) {
	occuringObjectList_.emplace_back(obb);
}

ErrorCollection::ErrorCollection() {
	errorCollection_ = {};
	errorMap_ = {
	{errorID::failedLoD00, ErrorObject("E0001", "LoD0.0 creation failed")},
	{errorID::failedLoD02, ErrorObject("E0002", "LoD0.2 creation failed")},
	{errorID::failedLoD10, ErrorObject("E0010", "LoD1.0 creation failed")},
	{errorID::failedLoD12, ErrorObject("E0012", "LoD1.2 creation failed")},
	{errorID::failedLoD13, ErrorObject("E0013", "LoD1.3 creation failed")},
	{errorID::failedLoD22, ErrorObject("E0022", "LoD2.2 creation failed")},
	{errorID::failedInit, ErrorObject("S0001", "Basic initialization failed")},
	{errorID::failedFootprint, ErrorObject("S0002", "Footprint creation failed")},
	{errorID::failedStorey, ErrorObject("S0003", "Storey creation failed")},
	{errorID::failedConvert, ErrorObject("S0004", "Failed to convert object")},
	{errorID::nobuildingName, ErrorObject("I0001", "Unable to find building name")},
	{errorID::nobuildingNameLong, ErrorObject("I0002", "Unable to find long building name")},
	{errorID::multipleBuildingObjects, ErrorObject("I0003", "Multiple building objects found")},
	{errorID::noProjectName, ErrorObject("I0004", "Unable to find long building name")},
	{errorID::multipleProjectNames, ErrorObject("I0005", "Multiple project objects found")},
	};
}

void ErrorCollection::addError(errorID id, const std::string& objectName) 
{
	//search if error is present
	if (errorCollection_.find(id) != errorCollection_.end())
	{
		if (id == errorID::failedConvert && objectName != "")
		{
			ErrorObject errorObject = errorCollection_[id];
			errorObject.addOccuringObject(objectName);
			errorCollection_[id] = errorObject;
		}
		return;
	}

	if (id == errorID::failedConvert && objectName != "")
	{
		ErrorObject errorObject = errorMap_[id];
		errorObject.addOccuringObject(objectName);
		errorCollection_[id] = errorObject;
		return;
	}

	errorCollection_[id] = errorMap_[id];
	return;
}

void ErrorCollection::addError(errorID id, const std::vector<std::string>& objectNameList) {
	//search if error is present
	if (errorCollection_.find(id) != errorCollection_.end())
	{
		if (id == errorID::failedConvert && objectNameList.size())
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

	if (id == errorID::failedConvert && objectNameList.size())
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


nlohmann::json ErrorCollection::toJson() {
	
	nlohmann::json jsonList = nlohmann::json::array();
	for (std::pair<errorID, ErrorObject> errorPair : errorCollection_)
	{
		jsonList.emplace_back(errorPair.second.toJson());
	}
	return jsonList;
}
