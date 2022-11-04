#include "floorProcessor.h"

double floorProcessor::getMedian(std::vector<double> l)
{
	assert(!l.empty());
	if (l.size() % 2 == 0) {
		const auto median_it1 = l.begin() + l.size() / 2 - 1;
		const auto median_it2 = l.begin() + l.size() / 2;

		std::nth_element(l.begin(), median_it1, l.end());
		const auto e1 = *median_it1;

		std::nth_element(l.begin(), median_it2, l.end());
		const auto e2 = *median_it2;

		return (e1 + e2) / 2;

	}
	else {
		const auto median_it = l.begin() + l.size() / 2;
		std::nth_element(l.begin(), median_it, l.end());
		return *median_it;
	}
}

std::vector<TopoDS_Shape> floorProcessor::getSlabShapes(helper* data)
{
	std::vector<TopoDS_Shape> floorShapes;
	IfcSchema::IfcSlab::list::ptr slabs = data->getSourceFile()->instances_by_type<IfcSchema::IfcSlab>();
	IfcSchema::IfcRoof::list::ptr roofs = data->getSourceFile()->instances_by_type<IfcSchema::IfcRoof>();

	for (IfcSchema::IfcSlab::list::it it = slabs->begin(); it != slabs->end(); ++it) {
		IfcSchema::IfcProduct* slab = *it;
		TopoDS_Shape floorShape = data->getObjectShape(slab);

		floorShapes.emplace_back(floorShape);
	}

	for (IfcSchema::IfcRoof::list::it it = roofs->begin(); it != roofs->end(); ++it) {
		IfcSchema::IfcRoof* roof = *it;
		TopoDS_Shape roofShape = data->getObjectShape(roof);

		floorShapes.emplace_back(roofShape);
	}

	return floorShapes;

}

std::vector<TopoDS_Face> floorProcessor::getSlabFaces(helper* data) {

	std::vector<TopoDS_Face> floorFaces;
	IfcSchema::IfcSlab::list::ptr slabs = data->getSourceFile()->instances_by_type<IfcSchema::IfcSlab>();

	auto kernel = data->getKernel();

	for (IfcSchema::IfcSlab::list::it it = slabs->begin(); it != slabs->end(); ++it) {
		const IfcSchema::IfcSlab* slab = *it;

		// get the IfcShapeRepresentation
		auto slabProduct = slab->Representation()->Representations();

		//get the global coordinate of the local origin
		gp_Trsf trsf;
		kernel->convert_placement(slab->ObjectPlacement(), trsf);

		for (auto et = slabProduct.get()->begin(); et != slabProduct.get()->end(); et++) {
			const IfcSchema::IfcRepresentation* slabRepresentation = *et;

			// select the body of the slabs (ignore the bounding boxes)
			if (slabRepresentation->data().getArgument(1)->toString() == "'Body'")
			{
				// select the geometry format
				auto slabItems = slabRepresentation->Items();

				IfcSchema::IfcRepresentationItem* slabItem = *slabRepresentation->Items().get()->begin();

				auto ob = kernel->convert(slabItem);

				// move to OpenCASCADE
				const TopoDS_Shape rShape = ob[0].Shape();
				const TopoDS_Shape aShape = rShape.Moved(trsf); // location in global space

				// set variables for top face selection
				TopoDS_Face topFace;
				double topHeight = -9999;

				// loop through all faces of slab
				TopExp_Explorer expl;
				for (expl.Init(aShape, TopAbs_FACE); expl.More(); expl.Next())
				{
					TopoDS_Face face = TopoDS::Face(expl.Current());
					BRepAdaptor_Surface brepAdaptorSurface(face, Standard_True);


					// select floor top face
					double faceHeight = face.Location().Transformation().TranslationPart().Z();

					if (faceHeight > topHeight) 
					{ 
						topFace = face; 
						topHeight = faceHeight;
					}
				}
				floorFaces.emplace_back(topFace);
			}
		}
	}
	return floorFaces;
}

std::vector<TopoDS_Face> floorProcessor::getSlabFaces(std::vector<TopoDS_Shape> shapes)
{
	std::vector<TopoDS_Face> floorFaces;
	for (size_t i = 0; i < shapes.size(); i++)
	{
		floorFaces.emplace_back(getTopFace(shapes[i]));
	}

	return floorFaces;
}

TopoDS_Face floorProcessor::getTopFace(TopoDS_Shape shape)
{

	TopExp_Explorer expl;
	std::vector<TopoDS_Face> faceCollection;
	std::vector<double> areaList;
	double commulativeArea = 0;
	int count = 0;


	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face face = TopoDS::Face(expl.Current());
		//get area of topface
		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
		commulativeArea += gprops.Mass();
		areaList.emplace_back(gprops.Mass());
		count++;

	}
	double minArea = commulativeArea / count;
	count = 0;

	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		if (areaList[count] > minArea)
		{
			faceCollection.emplace_back(TopoDS::Face(expl.Current()));
		}
		count++;
	}

	TopoDS_Face topFace;
	double topScore = 0;
	for (size_t j = 0; j < faceCollection.size(); j++)
	{
		double cummulativeH = 0;
		int score = 0;
		for (expl.Init(faceCollection[j], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			cummulativeH += (p.Z() + 9999999);
			score++;
		}

		if (cummulativeH / score > topScore)
		{
			topScore = cummulativeH / score;
			topFace = faceCollection[j];
		}
	}

	if (topScore != 0)
	{
		return topFace;
	}
		return {};
}

std::vector<double> floorProcessor::getFaceAreas(std::vector<TopoDS_Face> faces) {

	std::vector<double> floorArea;

	for (size_t i = 0; i < faces.size(); i++)
	{
		//get area of topface
		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(faces[i], gprops); // Stores results in gprops
		double area = gprops.Mass();

		floorArea.emplace_back(area);
	}

	return floorArea;
}

void floorProcessor::updateFloorGroup(std::vector<FloorGroupStruct>* floorGroups)
{
	std::vector<int> removeIdx;

	int idx = 0;
	for (auto it = floorGroups->begin(); it != floorGroups->end(); it++)
	{
		floorProcessor::FloorGroupStruct currentGroup = *it;

		if (currentGroup.mergeNum_ == 1)
		{
			currentGroup.merger_->mergeGroup(&currentGroup);
			removeIdx.emplace_back(idx);
		}
		idx++;
	}
	std::reverse(removeIdx.begin(), removeIdx.end());

	for (size_t i = 0; i < removeIdx.size(); i++)
	{
		floorGroups->erase(floorGroups->begin() + removeIdx[i]);
	}
}


void floorProcessor::printLevels(std::vector<double> levels) {
	std::sort(levels.begin(), levels.end());
	for (unsigned int i = 0; i < levels.size(); i++)
	{
		std::cout << levels[i] << std::endl;
	}
}


std::vector<double> floorProcessor::getStoreyElevations(helper* data)
{
	IfcSchema::IfcBuildingStorey::list::ptr storeys = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();
	std::vector<double> storeyElevation;

	for (IfcSchema::IfcBuildingStorey::list::it it = storeys->begin(); it != storeys->end(); ++it)
	{
		const IfcSchema::IfcBuildingStorey* storey = *it;
		storeyElevation.emplace_back(storey->Elevation() * data->getLengthMultiplier());
	}

	if (!storeyElevation.size()) { std::cout << "No storeys can be found" << std::endl; }
	std::sort(storeyElevation.begin(), storeyElevation.end());

	return storeyElevation;
}

std::vector<double> floorProcessor::getStoreyElevations(std::vector<helper*> data)
{
	for (size_t i = 0; i < data.size(); i++)
	{
		if (data[i]->getIsConstruct() || !data[i]->getDepending()) { return floorProcessor::getStoreyElevations(data[i]); }
	}
}

std::vector<double> floorProcessor::computeFloorElevations(helper* data)
{
	// get the top faces of the floors
	std::vector< TopoDS_Shape> floorShapes = floorProcessor::getSlabShapes(data);
	std::vector<TopoDS_Face> floorFaces = floorProcessor::getSlabFaces(floorShapes);

	// do not compute elevation if the model is partial and not a construction model 
	if (data->getDepending() && !data->getIsConstruct()) { return {};	}

	if (!data->getDepending())
	{
		// TODO detect floor thickness
		// TODO get orientating bounding box (based on the longest edge)
		// TODO get shortest side of the bounding box
		// TODO make a rule with the width and the orentated bounding box
	}

	bool debug = false;

	// make floor struct
	std::vector<double> faceAreas = floorProcessor::getFaceAreas(floorFaces);

	auto SmallestAllowedArea = 10 ;

	std::vector<FloorStruct> floorList;
	for (size_t i = 0; i < floorFaces.size(); i++)
	{
		// filter out small floor slabs
		if (faceAreas[i] > SmallestAllowedArea)
		{
			floorProcessor::FloorStruct floorobject(floorFaces[i], faceAreas[i]);
			floorList.emplace_back(floorobject);
		}
	}

	std::vector<FloorGroupStruct> floorGroups;

	// pair per "pure" elevation
	for (size_t i = 0; i < floorList.size(); i++) {

		floorProcessor::FloorGroupStruct floorGroup;

		if (!floorList[i].hasGroup)
		{
			floorGroup.addFloor(&floorList[i]);
			floorList[i].hasGroup = true;

			double height = floorList[i].elevation_;

			for (size_t j = 0; j < floorList.size(); j++)
			{
				if (floorList[j].hasGroup) { continue; }
				double otherHeight = floorList[j].elevation_;

				if (otherHeight + 0.000001 > height && otherHeight - 0.000001 < height)
				{
					floorGroup.addFloor(&floorList[j]);
					floorList[j].hasGroup = true;
				}
			}

		}
		if (floorGroup.floors_.size() > 0) { floorGroups.emplace_back(floorGroup); }
	}

	// find neighbours
	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];

		// get a face from a group
		for (size_t j = 0; j < currentGroup->floors_.size(); j++)
		{
			TopoDS_Face face1 = currentGroup->floors_[j]->face_;
			TopExp_Explorer expl;
			std::vector<DistancePair> pairedDistance;
			std::vector<gp_Pnt> pointList;

			// get all vertex from a face
			for (expl.Init(face1, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				pointList.emplace_back(BRep_Tool::Pnt(vertex));
			}

			// pair the vertex to create edges and compute distances
			int maxidx = (int) pointList.size();
			for (size_t j = 0; j < maxidx; j += 2)
			{
				gp_Pnt p1 = pointList[j];
				gp_Pnt p2 = pointList[j + 1];

				floorProcessor::DistancePair pair;
				pair.p1 = p1;
				pair.p2 = p2;
				pair.distance = p1.Distance(p2);;

				pairedDistance.emplace_back(pair);
			}

			for (size_t k = i + 1; k < floorGroups.size(); k++)
			{
				bool found = false;
				auto matchingGroup = &floorGroups[k];

				// get a face from a group to match
				for (size_t l = 0; l < matchingGroup->floors_.size(); l++)
				{

					TopoDS_Face face2 = matchingGroup->floors_[l]->face_;

					for (expl.Init(face2, TopAbs_VERTEX); expl.More(); expl.Next())
					{
						TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
						gp_Pnt p3 = BRep_Tool::Pnt(vertex);

						for (size_t m = 0; m < pairedDistance.size(); m++)
						{
							auto& pair = pairedDistance[m];
							auto referenceDistance = pair.distance;

							double computedDistance = p3.Distance(pair.p1) + p3.Distance(pair.p2);
							if (computedDistance + 0.05 > referenceDistance && computedDistance - 0.05 < referenceDistance) {
								if (matchingGroup->mergeNum_ == -1) {
									matchingGroup->merger_ = currentGroup;
									matchingGroup->mergeNum_ = 1;
								}
								else {
									for (size_t n = 0; n < floorGroups.size(); n++)
									{
										if (floorGroups[n].merger_ == matchingGroup)
										{
											floorGroups[n].merger_ = currentGroup;
										}
										matchingGroup->merger_ = currentGroup;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	// find overlapping buffers
	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];
		for (size_t j = i + 1; j < floorGroups.size(); j++)
		{
			auto matchingGroup = &floorGroups[j];

			if (currentGroup->elevation_ > matchingGroup->elevation_ && currentGroup->topElevation_ < matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->elevation_ && currentGroup->topElevation_ > matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->topElevation_ && currentGroup->topElevation_ > matchingGroup->topElevation_ ||
				currentGroup->elevation_ < matchingGroup->elevation_ && currentGroup->topElevation_ > matchingGroup->elevation_
				)
			{
				if (matchingGroup->mergeNum_ == -1) {
					matchingGroup->merger_ = currentGroup;
					matchingGroup->mergeNum_ = 1;
				}

				else {
					for (size_t k = 0; k < floorGroups.size(); k++)
					{
						if (floorGroups[k].merger_ == matchingGroup)
						{
							floorGroups[k].merger_ = currentGroup;
						}
						matchingGroup->merger_ = currentGroup;
					}
				}

			}

		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	// find small elevation differences
	double maxDistance = 1.f;

	for (size_t i = 0; i < floorGroups.size(); i++)
	{
		auto currentGroup = &floorGroups[i];
		for (size_t j = i + 1; j < floorGroups.size(); j++)
		{
			auto matchingGroup = &floorGroups[j];
			if (std::abs(currentGroup->elevation_ - matchingGroup->elevation_) < maxDistance)
			{
				// always merge to the floor with the smallest elevation
				auto targetGroup = currentGroup;
				auto sourceGroup = matchingGroup;

				if (targetGroup->elevation_ > sourceGroup->elevation_)
				{
					targetGroup = matchingGroup;
					sourceGroup = currentGroup;
				}

				if (sourceGroup->mergeNum_ == -1) {
					sourceGroup->merger_ = targetGroup;
					sourceGroup->mergeNum_ = 1;
				}
				else 
				{
					for (size_t k = 0; k < floorGroups.size(); k++)
					{
						if (floorGroups[k].merger_ == sourceGroup) { floorGroups[k].merger_ = targetGroup; }
						sourceGroup->merger_ = targetGroup;
					}
				}
			}
		}
	}

	// merge the groups
	updateFloorGroup(&floorGroups);

	std::vector<double> computedElev;
	for (size_t i = 0; i < floorGroups.size(); i++) { computedElev.emplace_back(floorGroups[i].elevation_); }


	if (debug)
	{
		for (size_t i = 0; i < computedElev.size(); i++)
		{
			std::cout << "computedElev: " << computedElev[i] << std::endl;
		}
	}

	std::sort(computedElev.begin(), computedElev.end());

	return computedElev;
}

std::vector<double> floorProcessor::computeFloorElevations(std::vector<helper*> data)
{
	for (size_t i = 0; i < data.size(); i++)
	{
		if (data[i]->getIsConstruct() || !data[i]->getDepending())
		{
			std::vector<double> tempFloorElevation = floorProcessor::computeFloorElevations(data[i]);

			if (tempFloorElevation.size() != 0) { return tempFloorElevation; }
			else {
				std::cout << "[WARNING] No storey elevations can be found in the supplied IFC file!" << std::endl;
				return {};
			}
		}
	}
}

bool floorProcessor::compareElevations(std::vector<double> elevations, std::vector<double> floors)
{
	bool sameSize = false;
	if (floors.size() == elevations.size())
	{
		for (size_t i = 0; i < elevations.size(); i++)
		{
			double rightElevation = floors[i];
			double leftElevation = elevations[i];

			if (rightElevation != leftElevation) { return false; }

		}
		return true;
	}
	else
	{
		//std::cout << "- " << floors.size() << " floors detected, " << elevations.size() << " storeys placed." << std::endl;
		return false;
	}
}

void floorProcessor::processStoreys(std::vector<helper*> data, std::vector<double> elevations, bool useOriginal)
{
	if (!useOriginal)
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			// wipe storeys
			floorProcessor::cleanStoreys(data[i]);
			// create new storeys
			floorProcessor::createStoreys(data[i], elevations);

			// match objects to new storeys
			floorProcessor::sortObjects(data[i]);
		}
	}
	else if (useOriginal)
	{
		// get the desirable storeys 
		IfcSchema::IfcBuildingStorey::list::ptr storeys = 0;


		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i]->getIsConstruct())
			{

				// the storey container has to be emptied
				IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = data[i]->getSourceFile()->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();
				std::vector<std::tuple<double, IfcSchema::IfcRelContainedInSpatialStructure*>> pairedContainers;
				for (auto it = containers->begin(); it != containers->end(); ++it)
				{
					IfcSchema::IfcRelContainedInSpatialStructure* structure = *it;
					if (structure->RelatingStructure()->data().type()->name() != "IfcBuildingStorey") { continue; }
					boost::shared_ptr<IfcSchema::IfcProduct::list> list(new IfcSchema::IfcProduct::list);
					structure->setRelatedElements(list);
				}

				floorProcessor::sortObjects(data[i]);
			}
			else
			{
				floorProcessor::cleanStoreys(data[i]);
			}
		}
		for (size_t i = 0; i < data.size(); i++)
		{
			if (!data[i]->getIsConstruct())
			{
				floorProcessor::sortObjects(data[i], storeys);
			}
		}
	}

}

void floorProcessor::cleanStoreys(helper* data)
{
	// remove the storey container
	IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = data->getSourceFile()->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();

	for (IfcSchema::IfcRelContainedInSpatialStructure::list::it it = containers->begin(); it != containers->end(); ++it)
	{
		IfcSchema::IfcRelContainedInSpatialStructure* container = *it;
		if (container->RelatingStructure()->data().type()->name() != "IfcBuildingStorey") { continue; }
		data->getSourceFile()->removeEntity(container);
	}

	// remove the storey object
	IfcSchema::IfcBuildingStorey::list::ptr oldStoreys = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuildingStorey>();

	for (IfcSchema::IfcBuildingStorey::list::it it = oldStoreys->begin(); it != oldStoreys->end(); ++it)
	{
		IfcSchema::IfcBuildingStorey* storey = *it;
		data->getSourceFile()->removeEntity(storey);
	}

}

void floorProcessor::createStoreys(helper* data, std::vector<double> floorStoreys)
{
	IfcSchema::IfcOwnerHistory* ownerHistory = data->getHistory();

	// find original owner history
	IfcSchema::IfcBuilding* building;
	IfcSchema::IfcBuilding::list::ptr buildings = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuilding>();
	
	if (buildings.get()->size() != 1)
	{
		std::cout << "[Error] multiple building objects found!" << std::endl;
		return;
	}

	building = *buildings.get()->begin();
	IfcSchema::IfcObjectPlacement* buildingLoc = building->ObjectPlacement();

	auto targetFile = data->getSourceFile();
	IfcHierarchyHelper<IfcSchema> hierarchyHelper;

	// find the presumed ground floor
	double smallestDistance = 1000;
	int groundfloor = 0;
	for (size_t i = 0; i < floorStoreys.size(); i++)
	{
		double distance = abs(floorStoreys[i]);

		if (distance < smallestDistance)
		{
			smallestDistance = distance;
			groundfloor = i;
		}
	}


	for (int i = 0; i < floorStoreys.size(); i++)
	{
		double lengthMulti = data->getUnits()[0];

		// create storey objects
		auto storey = hierarchyHelper.addBuildingStorey(building, ownerHistory);
		storey->setElevation(floorStoreys[i]/lengthMulti);
		if (i == groundfloor) { storey->setName("Ground Floor"); }
		else { storey->setName("Floor " + std::to_string(i - groundfloor)); }
		storey->setDescription("Automatically generated floor");

		// placement
		storey->setObjectPlacement(hierarchyHelper.addLocalPlacement(buildingLoc, 0, 0, floorStoreys[i] ) );
		IfcSchema::IfcProduct::list::ptr parts(new IfcSchema::IfcProduct::list);

		// make container object
		IfcSchema::IfcRelContainedInSpatialStructure* container = new IfcSchema::IfcRelContainedInSpatialStructure(
			IfcParse::IfcGlobalId(),		// GlobalId
			0,								// OwnerHistory
			std::string(""),				// Name
			boost::none,					// Description
			parts,							// Related Elements
			storey							// Related structure
		);
		hierarchyHelper.addEntity(container);
	}

	// add storey objects to the project
	for (auto it = hierarchyHelper.begin(); it != hierarchyHelper.end(); ++it)
	{
		auto hierarchyElement = *it;
		auto hierarchyDataElement = hierarchyElement.second;
		auto objectName = hierarchyDataElement->declaration().name();

		// remove potential dublications made by the helper
		// TODO remove all dependencies of building and owners as well
		if (objectName == "IfcBuilding" || objectName == "IfcOwnerHistory") { continue; }

		targetFile->addEntity(hierarchyDataElement);
	}
}

void floorProcessor::sortObjects(helper* data, IfcSchema::IfcProduct::list::ptr products)
{
	IfcParse::IfcFile* sourcefile = data->getSourceFile();
	auto kernel = data->getKernel();
	double lengthMulti = data->getLengthMultiplier();

	// make a vector with the height, spatial structure and a temp product  list
	IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containers = sourcefile->instances_by_type<IfcSchema::IfcRelContainedInSpatialStructure>();

	std::vector<std::tuple<double, IfcSchema::IfcRelContainedInSpatialStructure*>> pairedContainers;
	for (auto it = containers->begin(); it != containers->end(); ++it)
	{
		IfcSchema::IfcRelContainedInSpatialStructure* structure = *it;

		if (structure->RelatingStructure()->data().type()->name() != "IfcBuildingStorey") { continue; }
		//if (structure->data().getArgument(5)->toString().size() < 2) { continue; }

		double height = std::stod(structure->RelatingStructure()->data().getArgument(9)->toString()) * lengthMulti;

		pairedContainers.emplace_back(
			std::make_tuple(
				height,			// height
				structure		// spatial structure
			)
		);
	}

	std::sort(pairedContainers.begin(), pairedContainers.end());

	double topBuffer = 0.2;

	for (auto it = products->begin(); it != products->end(); ++it)
	{
		IfcSchema::IfcProduct* product = *it;

		if (product->data().type()->name() == "IfcSite" || 
			product->data().type()->name() == "IfcBuilding" ||
			product->data().type()->name() == "IfcBuildingStorey") {
			continue;
		}

		bool heightFound = false;
		double height = -9999;

		// floors are a special case due to them being placed based on their top face instead of basepoint
		if (product->data().type()->name() == "IfcSlab" || product->data().type()->name() == "IfcRoof")
		{
			TopoDS_Shape productShape =  data->getObjectShape(product, true);
			TopoDS_Face topFace = getTopFace(productShape);
			
			TopExp_Explorer expl;
			double lowHeight = 9999;
			for (expl.Init(topFace, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
				double pHeight = p.Z();
				if (pHeight < lowHeight) { lowHeight = pHeight; }
			}
			height = lowHeight;
		}
		else if (product->data().type()->name() == "IfcWall" || 
				 product->data().type()->name() == "IfcWallStandardCase" ||
				 product->data().type()->name() == "IfcStair" ||
				 product->data().type()->name() == "IfcStairFlight" 

			) 
		{
			TopoDS_Shape productShape = data->getObjectShape(product, true);
			std::vector<gp_Pnt> objectPoints = data->getObjectPoints(productShape, false);

			double lowHeight = 9999;
			double topHeight = -9999;
			for (size_t i = 0; i < objectPoints.size(); i++)
			{
				double pHeight = objectPoints[i].Z();
				if (pHeight < lowHeight) { lowHeight = pHeight; }
				if (pHeight > topHeight) { topHeight = pHeight; }
			}
			height = (topHeight - lowHeight)/3 + lowHeight;
		}
		else 
		{
			TopoDS_Shape productShape = data->getObjectShape(product, true);
			std::vector<gp_Pnt> objectPoints = data->getObjectPoints(productShape, false);

			if (objectPoints.size() == 0)
			{
				continue;
			}

			double lowHeight = 9999;
			double topheight = -9999;
			for (size_t i = 0; i < objectPoints.size(); i++)
			{
				double pHeight = objectPoints[i].Z();
				if (pHeight < lowHeight) { lowHeight = pHeight; }
				if (pHeight > topheight) { topheight = pHeight; }
			}
			
			if ((topheight - lowHeight)/5 > topBuffer)
			{
				height = lowHeight + topBuffer;
			}
			else
			{
				height = lowHeight;
			}


		}

		if (height == -9999 || height == 9999) {
			continue; 
		} // TODO what hits this!

		int maxidx = (int)pairedContainers.size();

		// find smallest distance to floor elevation
		double smallestDistance = 1000;
		int indxSmallestDistance = 0;
		for (int i = 0; i < maxidx; i++) {
			auto currentTuple = pairedContainers[i];
			double distance = height - std::get<0>(currentTuple);

			if (distance < -0.0001 * lengthMulti) { break; }

			if (distance < smallestDistance)
			{
				smallestDistance = distance;
				indxSmallestDistance = i;
			}

			if (distance > -0.0001 * lengthMulti && distance < 0.0001 * lengthMulti) { break; }
		}

		auto d = std::get<1>(pairedContainers[indxSmallestDistance])->RelatedElements();
		d.get()->push(product);
		std::get<1>(pairedContainers[indxSmallestDistance])->setRelatedElements(d);
	}

}

void floorProcessor::sortObjects(helper* data, IfcSchema::IfcBuildingStorey::list::ptr storeys)
{
	auto targetFile = data->getSourceFile();
	IfcSchema::IfcBuilding* building;
	IfcSchema::IfcOwnerHistory* ownerHistory = data->getHistory();
	IfcHierarchyHelper<IfcSchema> hierarchyHelper;
	IfcSchema::IfcBuilding::list::ptr buildings = data->getSourceFile()->instances_by_type<IfcSchema::IfcBuilding>();
	if (buildings.get()->size() != 1)
	{
		std::cout << "[Error] multiple building objects found!" << std::endl;
		return;
	}

	building = *buildings.get()->begin();
	IfcSchema::IfcObjectPlacement* buildingLoc = building->ObjectPlacement();

	for (auto it = storeys->begin(); it != storeys->end(); ++it)
	{
		auto originalStorey = *it;
		auto storey = hierarchyHelper.addBuildingStorey(building, ownerHistory);

		storey->setElevation(originalStorey->Elevation());

		if (originalStorey->hasName()) { storey->setName(originalStorey->Name()); }
		if (originalStorey->hasDescription()) { storey->setDescription(originalStorey->Description() + " Automatically named storey"); }
		if (originalStorey->hasLongName()) { storey->setLongName(originalStorey->LongName()); }
		if (originalStorey->hasObjectPlacement())
		storey->setObjectPlacement(hierarchyHelper.addLocalPlacement(buildingLoc, 0, 0, originalStorey->Elevation()));

		IfcSchema::IfcProduct::list::ptr parts(new IfcSchema::IfcProduct::list);
		IfcSchema::IfcRelContainedInSpatialStructure* container = new IfcSchema::IfcRelContainedInSpatialStructure(
			IfcParse::IfcGlobalId(),		// GlobalId
			0,								// OwnerHistory
			std::string(""),				// Name
			boost::none,					// Description
			parts,							// Related Elements
			storey							// Related structure
		);
		hierarchyHelper.addEntity(container);
	}


	for (auto it = hierarchyHelper.begin(); it != hierarchyHelper.end(); ++it)
	{
		auto hierarchyElement = *it;
		auto hierarchyDataElement = hierarchyElement.second;
		auto objectName = hierarchyDataElement->declaration().name();

		// remove potential dublications made by the helper
		// TODO remove all dependencies of building and owners as well
		if (objectName == "IfcBuilding" || objectName == "IfcOwnerHistory") { continue; }

		targetFile->addEntity(hierarchyDataElement);
	}

	sortObjects(data);

}

void floorProcessor::sortObjects(helper* data)
{
	std::cout << "[INFO] Storey sorting file " << data->getName() << std::endl;

	// get the elevation of all product in the model
	IfcSchema::IfcProduct::list::ptr products = data->getSourceFile()->instances_by_type<IfcSchema::IfcProduct>();

	sortObjects(data, products);
}

floorProcessor::FloorStruct::FloorStruct(TopoDS_Face face, double area)
{
	face_ = face;
	hasFace = true;
		
	isFlat_ = true;
	hasFlatness = true;

	// get the elevation and flatness
	double lowHeight = -9999;
	double topHeight = -9999;
	TopExp_Explorer expl;

	for (expl.Init(face, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt point = BRep_Tool::Pnt(vertex);

		double currentHeight = point.Z();

		if (lowHeight == -9999) { lowHeight = currentHeight; }
		else if (lowHeight != currentHeight)
		{
			isFlat_ = false;
			if (lowHeight > currentHeight) { lowHeight = currentHeight; }
		}

		if (topHeight == -9999) { topHeight = currentHeight; }
		else if (topHeight != currentHeight)
		{
			if (topHeight < currentHeight) { topHeight = currentHeight; }
		}


	}
	elevation_ = lowHeight;
	elevation_ = lowHeight;
	topElevation_ = topHeight;
	hasElevation = true;
}

floorProcessor::FloorGroupStruct::FloorGroupStruct(FloorStruct* floor)
{
	assigned_ = true;
	isFlat_ = floor->isFlat_;
	topElevation_ = floor->elevation_;
	elevation_ = floor->elevation_;

	floors_.emplace_back(floor);
}

void floorProcessor::FloorGroupStruct::addFloor(FloorStruct* floor)
{
	assigned_ = true;
	if (isFlat_) { isFlat_ = floor->isFlat_; }
	if (topElevation_ < floor->topElevation_) { topElevation_ = floor->topElevation_; }
	if (elevation_ > floor->elevation_) { elevation_ = floor->elevation_; }

	floors_.emplace_back(floor);
}

void floorProcessor::FloorGroupStruct::mergeGroup(FloorGroupStruct* group)
{
	for (size_t i = 0; i < group->floors_.size(); i++) { floors_.emplace_back(group->floors_[i]); }
	if (isFlat_) { isFlat_ = group->isFlat_; }
	if (topElevation_ < group->topElevation_) { topElevation_ = group->topElevation_; }
	if (elevation_ > group->elevation_) { elevation_ = group->elevation_; }
}