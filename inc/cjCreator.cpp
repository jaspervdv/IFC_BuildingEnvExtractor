#include "DataManager.h"
#include "cjCreator.h"
#include "helper.h"
#include "voxel.h"
#include "stringManager.h"
#include "DebugUtils.h"

#include <nlohmann/json.hpp>

#include <chrono>

#include <Bnd_Box.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BOPAlgo_Builder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_GTransform.hxx>

#include <TopExp_Explorer.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <Prs3d_ShapeTool.hxx>
#include <Geom_TrimmedCurve.hxx>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopExp.hxx>

#include <CJToKernel.h>

#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>

#include <execution>
#include <algorithm>
#include <thread>  

void CJGeoCreator::garbageCollection()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	//TODO: check if all uses have been exhausted
	if (!settingsCollection.make13() && !LoD03RoofFaces_.empty())
	{
		std::vector< std::vector<TopoDS_Face>>().swap(LoD03RoofFaces_);
	}

	if (!settingsCollection.make22() && 
		!settingsCollection.make30() && 
		!settingsCollection.make31() &&
		!LoD04RoofFaces_.empty())
	{
		std::vector< std::vector<TopoDS_Face>>().swap(LoD04RoofFaces_);
	}

	if (!settingsCollection.make31() && !LoD02Plates_.empty())
	{
		std::map<double, std::vector<TopoDS_Face>>().swap(LoD02Plates_);
	}
	return;
}

std::vector<TopoDS_Face> CJGeoCreator::getFootPrintList()
{
	std::vector<TopoDS_Face> faceList;
	for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
	{
		faceList.emplace_back(buildingSurfaceData.getFootPrint());
	}
	return faceList;
}

std::vector<TopoDS_Face> CJGeoCreator::getFootRoofOutlineList()
{
	std::vector<TopoDS_Face> faceList;
	for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
	{
		faceList.emplace_back(buildingSurfaceData.getRoofOutline());
	}
	return faceList;
}

bool CJGeoCreator::hasFootprints()
{
	for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
	{
		if (buildingSurfaceData.getFootPrint().IsNull())
		{
			return false;
		}
	}
	return true;
}

bool CJGeoCreator::hasRoofOutlines()
{
	for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
	{
		if (buildingSurfaceData.getRoofOutline().IsNull())
		{
			return false;
		}
	}
	return true;
}


std::vector<CJGeoCreator::BuildingSurfaceCollection> CJGeoCreator::sortRoofStructures(const std::vector<TopoDS_Face>& roofOutlines, const std::vector<RCollection>& rCollectionList)
{
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	// if only one footprint is present the surfaces are all considered part of the same building
	std::vector<BuildingSurfaceCollection> buildingSurfaceCollectionList;
	if (roofOutlines.size() == 1)
	{
		BuildingSurfaceCollection buildingCol;
		buildingCol.setRoof(rCollectionList);
		buildingCol.setRoofOutline(roofOutlines[0]);
		return { buildingCol };
	}
	
	// set up the list for if multiple footprints are present
	buildingSurfaceDataList_.reserve(roofOutlines.size());
	for (const TopoDS_Face& currentOutline : roofOutlines)
	{
		BuildingSurfaceCollection buildingCol;
		buildingCol.setRoofOutline(currentOutline);
		buildingSurfaceCollectionList.emplace_back(buildingCol);
	}

	// find the roof surfaces that are related to the roofoutlines
	for (const RCollection& currentSurface : rCollectionList)
	{
		const TopoDS_Face currentFlatFace = currentSurface.getFlatFace();
		std::vector<gp_Pnt> currentFacePoints = helperFunctions::getPointListOnFace(currentFlatFace);

		bool found = false;
		for (const gp_Pnt& currentPoint : currentFacePoints)
		{
			gp_Pnt projectedPoint = gp_Pnt(currentPoint.X(), currentPoint.Y(), 0);
			for (BuildingSurfaceCollection& currentSurfaceCol : buildingSurfaceCollectionList)
			{
				if (helperFunctions::pointOnShape(currentSurfaceCol.getRoofOutline(), projectedPoint, SettingsCollection::getInstance().precisionCoarse()))
				{
					currentSurfaceCol.addRoof(currentSurface);
					found = true;
					break;
				}
			}
			if (found) { break; }
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return buildingSurfaceCollectionList;
}

std::vector<RCollection> CJGeoCreator::mergeRoofSurfaces(std::vector<std::shared_ptr<SurfaceGridPair>>& Collection)
{
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	//index
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;
	std::vector<TopoDS_Face> faceList;
	for (size_t i = 0; i < Collection.size(); i++)
	{
		const TopoDS_Face& currentFace = Collection[i]->getFace();
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace, 0.5);
		spatialIndex.insert(std::make_pair(bbox, i));
		faceList.emplace_back(currentFace);
	}

	//// group surfaces
	std::vector<RCollection> mergedRSurfaces;
	std::vector<int>evalList(faceList.size());
	for (size_t i = 0; i < faceList.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		const TopoDS_Face& currentFace = faceList[i];

		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);

		std::vector<TopoDS_Face> toBeGroupdSurfaces = {};
		std::vector<TopoDS_Face> outerSurfaceRingList = { currentFace };

		while (true)
		{
			std::vector<TopoDS_Face> bufferList = {};
			for (size_t j = 0; j < outerSurfaceRingList.size(); j++)
			{
				const TopoDS_Face& evalFace = outerSurfaceRingList[j];
				toBeGroupdSurfaces.emplace_back(evalFace);

				std::vector<Value> qResult;
				qResult.clear();
				spatialIndex.query(bgi::intersects(helperFunctions::createBBox(evalFace, 0.1)), std::back_inserter(qResult));

				if (qResult.size() == 1) { break; }
				for (const Value& qValue : qResult)
				{
					int potentialNeigbhbourIdx = qValue.second;
					if (evalList[potentialNeigbhbourIdx] == 1) { continue; }

					const TopoDS_Face& potentialNeighbourFace = faceList[potentialNeigbhbourIdx];

					gp_Vec otherNormal = helperFunctions::computeFaceNormal(potentialNeighbourFace);

					if (helperFunctions::shareEdge(evalFace, potentialNeighbourFace))
					{
						bufferList.emplace_back(potentialNeighbourFace);
						evalList[potentialNeigbhbourIdx] = 1;
					}
				}
			}
			if (bufferList.size() == 0) { break; }
			outerSurfaceRingList = bufferList;
			bufferList.clear();
		}
		if (!toBeGroupdSurfaces.size()) { continue; }

		std::vector<TopoDS_Face> mergedSurfaces = helperFunctions::mergeFaces(toBeGroupdSurfaces);
		mergedRSurfaces.emplace_back(RCollection(mergedSurfaces));
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return mergedRSurfaces;
}


void CJGeoCreator::initializeBasic(DataManager* cluster) 
{
	if (!SettingsCollection::getInstance().makeOutlines()) { return; }

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoPreProcessing) << std::endl;
	// generate data required for most exports
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCoarseFiltering) << std::endl;
	std::vector<TopoDS_Shape> filteredObjects = getTopObjects(cluster);

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoReduceSurfaces) << std::endl;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> shapeIdx;
	std::vector<std::shared_ptr<SurfaceGridPair>> shapeList;
	reduceSurfaces(filteredObjects, &shapeIdx, &shapeList);

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoFineFiltering) << std::endl;
	std::vector<std::shared_ptr<SurfaceGridPair>> fineFilteredShapeList = FinefilterSurfaces(shapeList); //TODO: apply the shape idx
	shapeIdx.clear();
	shapeList.clear();
	shapeList.shrink_to_fit();

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofStructureMerging) << std::endl;
	std::vector<RCollection> mergedSurfaceRList = mergeRoofSurfaces(fineFilteredShapeList);
	fineFilteredShapeList.clear();
	fineFilteredShapeList.shrink_to_fit();

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofOutlineConstruction) << std::endl;
	std::vector<TopoDS_Face> roofOutlines = createRoofOutline(mergedSurfaceRList);

	// sort surface groups based on the roof/footprints
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofStructureSorting) << std::endl;
	buildingSurfaceDataList_ = sortRoofStructures(roofOutlines, mergedSurfaceRList);

	hasGeoBase_ = true;
	return;
}


std::vector<TopoDS_Face> CJGeoCreator::section2Faces(const std::vector<Value>& productLookupValues, DataManager* h, double cutlvl)
{
	double precision = SettingsCollection::getInstance().precision();

	// make a cutting plane 
	gp_Pnt p0(-1000, -1000, cutlvl);
	gp_Pnt p2(1000, 1000, cutlvl);
	TopoDS_Face cuttingFace = helperFunctions::createHorizontalFace(p0, p2, 0, cutlvl);

	std::vector<TopoDS_Shape> shapeList;
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(productLookupValues[i].second);
		TopoDS_Shape currentShape;
		if (lookup->hasSimpleShape()) { currentShape = lookup->getSimpleShape(); }
		else { currentShape = lookup->getProductShape(); }
		shapeList.emplace_back(currentShape);
	}
	return section2Faces(shapeList, cutlvl);
}

template <typename T>
std::vector<TopoDS_Face> CJGeoCreator::section2Faces(const std::vector<T>& shapes, double cutlvl)
{
	double buffer = 0.15;

	std::vector<TopoDS_Face> spltFaceCollection;
	double precision = SettingsCollection::getInstance().precision();

	gp_Pnt lll;
	gp_Pnt urr;
	helperFunctions::bBoxDiagonal(shapes, &lll, &urr);

	gp_Pnt p0 = gp_Pnt(lll.X() - 10, lll.Y() - 10, 0);
	gp_Pnt p2 = gp_Pnt(urr.X() + 10, urr.Y() + 10, 0);
	TopoDS_Face cuttingFace = helperFunctions::createHorizontalFace(p0, p2, 0, cutlvl);

	for (const TopoDS_Shape& currentShape : shapes)
	{
		std::vector<TopoDS_Edge> edgeList;
		for (TopExp_Explorer expl(currentShape, TopAbs_FACE); expl.More(); expl.Next())
		{
			TopoDS_Face face = TopoDS::Face(expl.Current());

			// ignore extremely small surfaces
			if (helperFunctions::computeArea(face) < 0.005) { continue; }

			// check if the face is flush to the cuttting plane if flat 
			gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);
			
			if (std::abs(faceNormal.X()) < 0.05 && std::abs(faceNormal.Y()) < 0.05)
			{
				// if flush store as is
				std::vector<gp_Pnt> facePoints =  helperFunctions::getUniquePoints(face);

				for (const gp_Pnt& currentFacePoint : facePoints)
				{
					if (abs(currentFacePoint.Z() - cutlvl) > buffer) { continue; }
					spltFaceCollection.emplace_back(helperFunctions::projectFaceFlat(face, cutlvl));

					break;
				}
				// else ignore the face
				continue;
			}

			// check if the surface bbox falls on the cut level
			bg::model::box <BoostPoint3D> faceBox = helperFunctions::createBBox(face);
			if (faceBox.min_corner().get<2>() > cutlvl || faceBox.max_corner().get<2>() < cutlvl) { continue; }

			// get the cut edges on the plane
			BRepAlgoAPI_Cut cutter(face, cuttingFace);
			if (!cutter.IsDone()) { continue; }

			TopTools_ListOfShape cutterResults = cutter.SectionEdges();
			for (auto it = cutterResults.begin(); it != cutterResults.end(); ++it)
			{
				for (TopExp_Explorer expl2(*it, TopAbs_EDGE); expl2.More(); expl2.Next()) {
					TopoDS_Edge currentEdge = TopoDS::Edge(expl2.Current());
					edgeList.emplace_back(currentEdge);
				}
			}
		}

		if (!edgeList.size()) { continue; }
		std::vector<TopoDS_Wire> splitWireList = helperFunctions::growWires(edgeList);

		if (!splitWireList.size()) { continue; }
		for (const TopoDS_Wire& splitWire : splitWireList)
		{
			if (!splitWire.Closed()) { continue; }
			spltFaceCollection.emplace_back(BRepBuilderAPI_MakeFace(splitWire));
		}
	}
	return spltFaceCollection;
}

void CJGeoCreator::SplitInAndOuterHFaces(const std::vector<TopoDS_Face>& inputFaces, std::vector<TopoDS_Face>& innerFaces, std::vector<TopoDS_Face>& outerFaces)
{
	for (const TopoDS_Face currentFace : inputFaces)
	{
		// get point on face
		std::vector<gp_Pnt> facePointList = helperFunctions::getPointListOnFace(currentFace);
		std::vector<gp_Pnt> edgePointList = helperFunctions::getUniquePoints(currentFace);
		facePointList.insert(facePointList.end(), edgePointList.begin(), edgePointList.end());
		if (!facePointList.size()) { continue; }
		bool isFound = false;
		for (gp_Pnt facePoint : facePointList)
		{
			// get closest upper voxel
			facePoint.SetZ(facePoint.Z() + SettingsCollection::getInstance().voxelSize() / 0.66);
			int voxelIndx = voxelGrid_->getCloseByVoxel(facePoint);
			voxel boxel = voxelGrid_->getVoxel(voxelIndx);
			if (!boxel.getIsIntersecting() && !boxel.getIsInside())
			{
				outerFaces.emplace_back(currentFace);
				isFound = true;
				break;
			}
		}
		if (!isFound)
		{
			innerFaces.emplace_back(currentFace);
		}
	}
	return;
}

void CJGeoCreator::SplitInAndOuterHFaces(const TopoDS_Shape& inputFaces, std::vector<TopoDS_Face>& innerFaces, std::vector<TopoDS_Face>& outerFaces)
{
	std::vector<TopoDS_Face> splitFaceList;
	for (TopExp_Explorer faceExpl(inputFaces, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		splitFaceList.emplace_back(TopoDS::Face(faceExpl.Current()));
	}
	SplitInAndOuterHFaces(splitFaceList, innerFaces, outerFaces);
	return;
}

void CJGeoCreator::planarFaces2OutlineComplex(std::vector<TopoDS_Face>& intFacesOut, std::vector<TopoDS_Face>& extFacesOut, const std::vector<TopoDS_Face>& planarFaces, const TopoDS_Face& boundingFace, bool filterExternal)
{
	std::vector<TopoDS_Shape> faceCluster = helperFunctions::planarFaces2Cluster(planarFaces);

	intFacesOut = {};
	extFacesOut = {};

	for (const TopoDS_Shape& faceComplex : faceCluster)
	{
		// split section face with the merged splitting faces
		BRepAlgoAPI_Splitter innerSplitter;
		BRepAlgoAPI_Splitter outerSplitter;
		innerSplitter.SetFuzzyValue(1e-4);
		outerSplitter.SetFuzzyValue(1e-4);
		TopTools_ListOfShape innerToolList;
		TopTools_ListOfShape outerToolList;
		TopTools_ListOfShape argumentList;

		argumentList.Append(boundingFace);
		innerSplitter.SetArguments(argumentList);
		outerSplitter.SetArguments(argumentList);
		if (filterExternal)
		{

			std::vector<TopoDS_Face> innerFaces;
			std::vector<TopoDS_Face> outerFaces;
			SplitInAndOuterHFaces(faceComplex, innerFaces, outerFaces);
			for (const TopoDS_Face& filteredFace : innerFaces)
			{
				innerToolList.Append(filteredFace);
			}
			for (const TopoDS_Face& filteredFace : outerFaces)
			{
				outerToolList.Append(filteredFace);
			}
		}

		if (innerToolList.Size())
		{
			innerSplitter.SetTools(innerToolList);
			innerSplitter.Build();

			std::vector<TopoDS_Face> invertedFaces = helperFunctions::invertFace(helperFunctions::getOuterFace(innerSplitter.Shape(), boundingFace));
			for (const TopoDS_Face& invertedFace : invertedFaces)
			{
				intFacesOut.emplace_back(invertedFace);
			}
		}

		if (outerToolList.Size())
		{
			outerSplitter.SetTools(outerToolList);
			outerSplitter.Build();

			std::vector<TopoDS_Face> invertedFaces = helperFunctions::invertFace(helperFunctions::getOuterFace(outerSplitter.Shape(), boundingFace));
			for (const TopoDS_Face& invertedFace : invertedFaces)
			{
				extFacesOut.emplace_back(invertedFace);
			}
		}
	}
	return;
}

TopoDS_Solid CJGeoCreator::extrudeFace(const TopoDS_Face& evalFace, bool downwards, double splittingFaceHeight)
{
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidShape;
	brepBuilder.MakeSolid(solidShape);

	TopoDS_Face projectedFace = helperFunctions::projectFaceFlat(evalFace, splittingFaceHeight);
	brepSewer.Add(evalFace);
	brepSewer.Add(projectedFace.Reversed());
	int edgeCount = 0;
	for (TopExp_Explorer edgeExplorer(evalFace, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
		const TopoDS_Edge& edge = TopoDS::Edge(edgeExplorer.Current());
		gp_Pnt p0 = helperFunctions::getFirstPointShape(edge);
		gp_Pnt p1 = helperFunctions::getLastPointShape(edge);

		if (downwards)
		{
			if (p0.Z() <= splittingFaceHeight || p1.Z() <= splittingFaceHeight) { return TopoDS_Solid(); }
		}
		else
		{
			if (p0.Z() >= splittingFaceHeight || p1.Z() >= splittingFaceHeight) { return TopoDS_Solid(); }
		}
		

		TopoDS_Face sideFace = helperFunctions::createPlanarFace(p0, p1, gp_Pnt(p1.X(), p1.Y(), splittingFaceHeight), gp_Pnt(p0.X(), p0.Y(), splittingFaceHeight));
		brepSewer.Add(sideFace);
		edgeCount++;
	}

	if (edgeCount <= 2)
	{
		return TopoDS_Solid();
	}

	brepSewer.Perform();
	TopoDS_Shape sewedShape = brepSewer.SewedShape();

	if (sewedShape.Closed())
	{
		brepBuilder.Add(solidShape, sewedShape);
	}
	else {
		return TopoDS_Solid(); //TODO: resolve this issue
		brepBuilder.Add(solidShape, sewedShape); 
	}
	return solidShape;
}


void CJGeoCreator::makeFootprint(DataManager* h)
{
	auto startTime = std::chrono::steady_clock::now();

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// set up floorlvl
	double floorlvl = settingsCollection.footprintElevation();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCoasreFootFiltering) << floorlvl << std::endl;
	// check if storey elev falls within bbox
	if (h->getLllPoint().Z() > floorlvl || h->getUrrPoint().Z() < floorlvl)
	{
		floorlvl = h->getLllPoint().Z();
		settingsCollection.setFootprintElevation(floorlvl);
		ErrorCollection::getInstance().addError(ErrorID::warningInputIncFootprintElev);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningInputIncFootprintElev) << std::endl;
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCoasreFootFiltering) << floorlvl << std::endl;
	}

	// local user choses z offset
	double storeyBuffer = settingsCollection.horizontalSectionOffset();
	gp_Trsf translation;
	translation.SetTranslation(gp_Vec(0, 0, -storeyBuffer));

	try
	{
		std::vector<TopoDS_Face> footprintList;
		makeFloorSection(footprintList, h, floorlvl + storeyBuffer);
		for (TopoDS_Face& footprintItem : footprintList) { footprintItem.Move(translation); }

		for (BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			for (const TopoDS_Face& currentFootprint :  footprintList)
			{
				BRepExtrema_DistShapeShape distanceFootRoof(buildingSurfaceData.getRoofOutline(), currentFootprint);
				double verticalDistatance = abs(distanceFootRoof.PointOnShape1(1).Z() - distanceFootRoof.PointOnShape2(1).Z());
				if (abs(verticalDistatance - distanceFootRoof.Value()) > settingsCollection.precision() )
				{
					continue;
				}
				buildingSurfaceData.setFootPrint(currentFootprint);
				break;
			}			
		}
	}
	catch (const std::exception&)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		throw std::invalid_argument("");
		return;
	}

	hasFootprints_ = true;
	printTime(startTime, std::chrono::steady_clock::now());
	std::cout << std::endl;
	return;
}


void CJGeoCreator::makeFloorSection(std::vector<TopoDS_Face>& facesOut, DataManager* h, double sectionHeight)
{
	// create plane on which the projection has to be made
	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();

	gp_Pnt p0 = gp_Pnt(lll.X() - 10, lll.Y() - 10, 0);
	gp_Pnt p1 = gp_Pnt(urr.X() + 10, urr.Y() + 10, 0);
	TopoDS_Face cuttingPlane = helperFunctions::createHorizontalFace(p0, p1, 0, sectionHeight);

	// get all edges that meet the cutting plane

	std::vector<Value> productLookupValues;
	bg::model::box <BoostPoint3D> searchBox = helperFunctions::createBBox(cuttingPlane, 0.15);
	h->getIndexPointer()->query(bgi::intersects(searchBox), std::back_inserter(productLookupValues));
	
	std::vector<TopoDS_Face> splitFaceList = section2Faces(productLookupValues, h, sectionHeight);

	if (!splitFaceList.size())
	{
		//TODO: add error
		return;
	}
	std::vector<TopoDS_Face> cleanedFaceList = helperFunctions::removeDubFaces(splitFaceList, true);
	if (!cleanedFaceList.size())
	{
		//TODO: add error
		return;
	}

	facesOut = helperFunctions::planarFaces2Outline(cleanedFaceList, cuttingPlane);
	return;
}

void CJGeoCreator::makeFloorSectionComplex(
	std::vector<TopoDS_Face>& intFacesOut, 
	std::vector<TopoDS_Face>& extFacesOut,
	DataManager* h, 
	double sectionHeight, 
	const std::vector<IfcSchema::IfcBuildingStorey*>& buildingStoreyObjectList
)
{
	// create plane on which the projection has to be made
	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();

	gp_Pnt p0 = gp_Pnt(lll.X() - 10, lll.Y() - 10, 0);
	gp_Pnt p1 = gp_Pnt(urr.X() + 10, urr.Y() + 10, 0);
	TopoDS_Face cuttingPlane = helperFunctions::createHorizontalFace(p0, p1, 0, sectionHeight);

	// get object shapes that are related to the input storeys 
	std::vector<TopoDS_Shape> storeyRelatedShapeList;
	for (const IfcSchema::IfcBuildingStorey* IfcBuildingStorey : buildingStoreyObjectList)
	{
		IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containedStructure = IfcBuildingStorey->ContainsElements()->as<IfcSchema::IfcRelContainedInSpatialStructure>();
		for (auto csit = containedStructure->begin(); csit != containedStructure->end(); ++csit)
		{
			IfcSchema::IfcProduct::list::ptr storeyRelatedProducts = (*csit)->RelatedElements();
			for (auto srit = storeyRelatedProducts->begin(); srit != storeyRelatedProducts->end(); ++srit)
			{
				IfcSchema::IfcProduct* currentProduct = *srit;
				TopoDS_Shape currentShape = h->getObjectShapeFromMem(currentProduct, true);

				if (currentShape.IsNull()) { continue; }
				storeyRelatedShapeList.emplace_back(currentShape);
			}
		}
	}

	// generate shapes
	std::vector<TopoDS_Face> splitFaceList = section2Faces(storeyRelatedShapeList, sectionHeight);
	std::vector<TopoDS_Face> cleanedFaceList = helperFunctions::removeDubFaces(splitFaceList);
	if (!cleanedFaceList.size())
	{
		//TODO: add error
		return;
	}
	planarFaces2OutlineComplex(intFacesOut, extFacesOut, cleanedFaceList, cuttingPlane, true);
	return;
}

std::vector<TopoDS_Face> CJGeoCreator::getSplitFaces(
	const std::vector<TopoDS_Face>& inputFaceList,
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx
)
{
	double precision = SettingsCollection::getInstance().precisionCoarse();

	// split the topfaces with the cutting faces
	std::vector<TopoDS_Face> splitFaceList;
	for (const TopoDS_Face& currentRoofSurface : inputFaceList)
	{
		std::vector<std::pair<BoostBox3D, TopoDS_Face>> qResult;
		bg::model::box <BoostPoint3D> searchBox = helperFunctions::createBBox(currentRoofSurface);
		cuttingFaceIdx.query(bgi::intersects(searchBox), std::back_inserter(qResult));

		if (qResult.size() <= 1)
		{
			splitFaceList.emplace_back(currentRoofSurface);
			continue;
		}

		BOPAlgo_Splitter divider;
		divider.SetFuzzyValue(precision);
		divider.SetRunParallel(Standard_False);
		divider.AddArgument(currentRoofSurface);

		for (const auto& [cuttingBox, cuttingFace] : qResult)
		{
			TopoDS_Face currentSplitter = cuttingFace;
			if (currentSplitter.IsEqual(currentRoofSurface))
			{
				continue;
			}

			divider.AddTool(currentSplitter);
		}
		divider.Perform();

		for (TopExp_Explorer expl(divider.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face subFace = TopoDS::Face(expl.Current());
			splitFaceList.emplace_back(subFace);
		}
	}
	return splitFaceList;
}

std::vector<TopoDS_Face> CJGeoCreator::getVisTopSurfaces(const std::vector<TopoDS_Face>& faceList, double lowestZ, const TopoDS_Face& bufferSurface)
{
	// index surfaces
	bgi::rtree<BoxFacePair, bgi::rstar<25>> faceIdx;
	for (const TopoDS_Face& currentFace : faceList)
	{
		std::optional<gp_Pnt> optionalBasePoint = helperFunctions::getPointOnFace(currentFace);
		if (optionalBasePoint == std::nullopt) { continue; } //filters out non-man
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		faceIdx.insert(std::make_pair(bbox, currentFace));
	}

	// extrude the trimmed surfaces and join
	std::vector<TopoDS_Face> outputFaceList;
	for (const TopoDS_Face& currentFace : faceList)
	{
		if (helperFunctions::getHighestZ(currentFace) <= lowestZ) { continue; }

		std::optional<gp_Pnt> optionalBasePoint = helperFunctions::getPointOnFace(currentFace);
		if (optionalBasePoint == std::nullopt) { continue; }

		// test if falls within buffersurface
		gp_Pnt basePoint = *optionalBasePoint;
		gp_Pnt topPoint = gp_Pnt(basePoint.X(), basePoint.Y(), basePoint.Z() + 100000);
		gp_Pnt bottomPoint = gp_Pnt(basePoint.X(), basePoint.Y(), basePoint.Z() - 100000);
		
		if (!bufferSurface.IsNull())
		{
			TopoDS_Edge lowerEvalLine = BRepBuilderAPI_MakeEdge(basePoint, bottomPoint);
			BRepExtrema_DistShapeShape distanceWireCalc(lowerEvalLine, bufferSurface);
			if (distanceWireCalc.Value() > 1e-6) { continue; }
		}
		TopoDS_Edge upperEvalLine = BRepBuilderAPI_MakeEdge(basePoint, topPoint);
		
		std::vector<BoxFacePair> qResult;
		qResult.clear();
		faceIdx.query(bgi::intersects(
			helperFunctions::createBBox(basePoint, topPoint, 0.2)), std::back_inserter(qResult));

		// test if is hidden
		bool isHidden = false;
		for (const BoxFacePair& otherSurfaceValue : qResult)
		{
			TopoDS_Face otherFace = otherSurfaceValue.second;
			if (currentFace.IsEqual(otherFace)) { continue; }

			BRepExtrema_DistShapeShape distanceWireCalc(upperEvalLine, otherFace);
			if (distanceWireCalc.Value() < 1e-6)
			{
				isHidden = true;
				break;
			}
		}
		if (!isHidden)
		{
			outputFaceList.emplace_back(currentFace);
		}
	}
	return outputFaceList;
}

std::vector<TopoDS_Shape> CJGeoCreator::computePrisms(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ, bool preFilter, const TopoDS_Face& bufferSurface)
{
	double precision = SettingsCollection::getInstance().precisionCoarse();

	std::vector<TopoDS_Face> splitTopSurfaceList;
	if (!preFilter) { splitTopSurfaceList = inputFaceList; }
	else { splitTopSurfaceList = getSplitTopFaces(inputFaceList, lowestZ, bufferSurface); }

	if (splitTopSurfaceList.size() == 1)
	{
		return { extrudeFace(splitTopSurfaceList[0], true, lowestZ) };
	}

	// extrude the trimmed top surfaces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> toBeSplitfaceIdx; // pair bbox | extruded shape faces
	std::vector<TopoDS_Face> toBesSplitFaceList;
	for (const TopoDS_Face& currentFace : splitTopSurfaceList)
	{
		TopoDS_Solid extrudedShape = extrudeFace(currentFace, true, lowestZ);
		for (TopExp_Explorer expl(extrudedShape, TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face extrusionFace = TopoDS::Face(expl.Current());

			// ignore if not vertical face
			gp_Vec currentNormal = helperFunctions::computeFaceNormal(extrusionFace);
			if (abs(currentNormal.Z()) > 1e-4) { continue; };

			// find if already found in model 
			BoostBox3D faceBox = helperFunctions::createBBox(extrusionFace);
			toBeSplitfaceIdx.insert(std::make_pair(faceBox, extrusionFace));
			toBesSplitFaceList.emplace_back(extrusionFace);
		}
		BoostBox3D topFaceBox = helperFunctions::createBBox(currentFace);
		toBeSplitfaceIdx.insert(std::make_pair(topFaceBox, currentFace));
		toBesSplitFaceList.emplace_back(currentFace);
	}

	// remove dub faces and split them
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx = indexUniqueFaces(toBeSplitfaceIdx);
	std::vector<TopoDS_Face> splitFaceList = getSplitFaces(toBesSplitFaceList, cuttingFaceIdx);
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> SplitfaceIdx;
	for (const TopoDS_Face& currentFace : splitFaceList)
	{
		gp_Vec currentVec = helperFunctions::computeFaceNormal(currentFace);
		if (currentVec.Magnitude() < 1e-6) { continue; }
		BoostBox3D faceBox = helperFunctions::createBBox(currentFace);
		SplitfaceIdx.insert(std::make_pair(faceBox, currentFace));
	}

	BRepBuilderAPI_Sewing brepSewer(precision);
	for (const auto& [currentBox, currentFace] : SplitfaceIdx)
	{
		std::vector<BoxFacePair> qResult;
		qResult.clear();
		SplitfaceIdx.query(bgi::intersects(currentBox), std::back_inserter(qResult));

		bool isDub = false;
		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);
		for (const auto& [otherBox, otherFace] : qResult)
		{
			if (currentFace.IsEqual(otherFace)) { continue; }
			if (!currentNormal.IsParallel(helperFunctions::computeFaceNormal(otherFace), precision)) { continue; }
			if (!helperFunctions::isSame(currentBox, otherBox)) { continue; }

			isDub = true;
		}
		if (!isDub)
		{
			brepSewer.Add(currentFace);

			if (abs(currentNormal.Z()) < 1e-4) { continue; }
			TopoDS_Face flattenedFace = helperFunctions::projectFaceFlat(currentFace, lowestZ);

			if (helperFunctions::computeFaceNormal(flattenedFace).Magnitude() < 1e-4) { continue; }
			brepSewer.Add(helperFunctions::projectFaceFlat(currentFace, lowestZ));
		}
	}

	brepSewer.Perform();
	TopoDS_Shape sewedShape = brepSewer.SewedShape();

	if (sewedShape.IsNull())
	{
		return {};
	}
	if (sewedShape.Closed() != 1)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningNoSolid, "prism computation");
		std::cout << errorWarningStringEnum::getString(ErrorID::warningNoSolid) << std::endl;
	}

	std::vector<TopoDS_Shape> prismList;
	TopoDS_Shape simplefiedShape = simplefySolid(sewedShape);
	if (simplefiedShape.IsNull())
	{
		prismList.emplace_back(sewedShape);
		ErrorCollection::getInstance().addError(ErrorID::warningUnableToSimplefy, "prism computation");
		std::cout << errorWarningStringEnum::getString(ErrorID::warningUnableToSimplefy) << std::endl;
	}
	else
	{
		prismList.emplace_back(simplefiedShape);
	}
	return prismList;
}

std::vector<TopoDS_Face> CJGeoCreator::getSplitTopFaces(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ, const TopoDS_Face& bufferSurface)
{
	double precision = SettingsCollection::getInstance().precision();

	// extrude the surfaces and collect their faces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> faceIdx; // pair bbox | extruded shape faces
	for (const TopoDS_Face& currentTopFace : inputFaceList)
	{
		TopoDS_Solid extrudedShape = extrudeFace(currentTopFace, true, lowestZ);
		for (TopExp_Explorer expl(extrudedShape, TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face extrusionFace = TopoDS::Face(expl.Current());

			// ignore if not vertical face
			gp_Vec currentNormal = helperFunctions::computeFaceNormal(extrusionFace);
			if (abs(currentNormal.Z()) > 1e-4) { continue; };

			// find if already found in model 
			BoostBox3D faceBox = helperFunctions::createBBox(extrusionFace);
			faceIdx.insert(std::make_pair(faceBox, extrusionFace));
		}
		BoostBox3D topFaceBox = helperFunctions::createBBox(currentTopFace);
		faceIdx.insert(std::make_pair(topFaceBox, currentTopFace));
	}

	if (!bufferSurface.IsNull())
	{
		TopoDS_Solid extrudedShape = extrudeFace(bufferSurface, false, 1000000);
		for (TopExp_Explorer expl(extrudedShape, TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face extrusionFace = TopoDS::Face(expl.Current());

			// ignore if not vertical face
			gp_Vec currentNormal = helperFunctions::computeFaceNormal(extrusionFace);
			if (abs(currentNormal.Z()) > 1e-4) { continue; };

			// find if already found in model 
			BoostBox3D faceBox = helperFunctions::createBBox(extrusionFace);
			faceIdx.insert(std::make_pair(faceBox, extrusionFace));
		}
	}
	// remove the faces that will presumably not split a single face
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx = indexUniqueFaces(faceIdx);
	std::vector<TopoDS_Face> splitFaceList = getSplitFaces(inputFaceList, cuttingFaceIdx);
	std::vector<TopoDS_Face> cleanedsplitFaceList = helperFunctions::cleanFaces(splitFaceList);
	std::vector<TopoDS_Face> visibleFaceList = getVisTopSurfaces(cleanedsplitFaceList, lowestZ, bufferSurface);
	//clean the surfaces
	return  visibleFaceList;
}


TopoDS_Shape CJGeoCreator::simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap)
{
	std::vector<TopoDS_Face> facelist;
	std::vector<gp_Dir> normalList;

	for (TopExp_Explorer expl(solidShape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(face);

		if (faceNomal.Magnitude() < 1e-6)
		{
			continue;
		}

		facelist.emplace_back(face);
		normalList.emplace_back(faceNomal);
	}

	if (facelist.size() != normalList.size()) { return solidShape; }
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid simpleBuilding;
	brepBuilder.MakeSolid(simpleBuilding);

	std::vector<TopoDS_Face> mergedFaceList = simplefyFacePool(facelist, normalList, evalOverlap);

	if (mergedFaceList.size() == facelist.size())
	{
		return solidShape;
	}

	for (size_t i = 0; i < mergedFaceList.size(); i++)
	{
		brepSewer.Add(mergedFaceList[i]);
	}

	brepSewer.Perform();
	TopoDS_Shape sewedShape = brepSewer.SewedShape();
	if (sewedShape.IsNull()) { return{}; }
	if (sewedShape.ShapeType() == TopAbs_COMPOUND) { return sewedShape; }
	brepBuilder.Add(simpleBuilding, sewedShape);
	return simpleBuilding;
}


std::vector<TopoDS_Face> CJGeoCreator::simplefyFacePool(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap) {
	std::vector<gp_Dir> normalList;
	for (size_t i = 0; i < surfaceList.size(); i++)
	{
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(surfaceList[i]);

		if (faceNomal.Magnitude() < 1e-6)
		{
			continue;
		}
		normalList.emplace_back(faceNomal);
	}

	if (surfaceList.size() != normalList.size()) { return surfaceList; }
	return simplefyFacePool(surfaceList, normalList, evalOverlap);
}

template <typename T>
std::vector<T> CJGeoCreator::simplefyFacePool(const std::vector<T>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap) {

	if (!surfaceList.size()) { return surfaceList; }

	constexpr bool usePair = !std::is_same_v<T, TopoDS_Face>;

	auto getFace = [](const T& item) -> const TopoDS_Face& {
		if constexpr (std::is_same_v<T, TopoDS_Face>) {
			return item;
		}
		else {
			return item.first;
		}
	};

	auto getType = [](const T& item) -> const std::string& {
		if constexpr (std::is_same_v<T, TopoDS_Face>) {
			return "";
		}
		else {
			return item.second;
		}
	};

	if (surfaceList.size() != normalList.size()) { return surfaceList; }

	// make spatial index
	bgi::rtree<Value, bgi::rstar<25>> shapeIdx;
	for (int i = 0; i < surfaceList.size(); i++)
	{
		TopoDS_Face currentFace = getFace(surfaceList[i]);
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		shapeIdx.insert(std::make_pair(bbox, i));
	}

	std::vector<T> cleanedFaceList;
	std::vector<int> mergedSurfaceIdxList = {0};
	std::vector<int> evalList(surfaceList.size());

	double precision = SettingsCollection::getInstance().precision();
	while (true)
	{
		size_t currentSurfaceIdxSize = mergedSurfaceIdxList.size();
		for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
		{
			int currentIdx = mergedSurfaceIdxList[i];
			if (evalList[currentIdx] == 1) { continue; }
			evalList[currentIdx] = 1;

			std::string currentType = getType(surfaceList[currentIdx]);
			if (currentType == "IfcWindow" || currentType == "IfcDoor") { break; }

			TopoDS_Face currentFace = getFace(surfaceList[currentIdx]);
			gp_Dir currentdir = normalList[currentIdx];

			bg::model::box < BoostPoint3D > cummulativeBox = helperFunctions::createBBox(currentFace);
			
			std::vector<Value> qResult;
			qResult.clear();
			shapeIdx.query(bgi::intersects(
				cummulativeBox), std::back_inserter(qResult));

			for (size_t j = 0; j < qResult.size(); j++)
			{
				int otherFaceIdx = qResult[j].second;
				TopoDS_Face otherFace = getFace(surfaceList[otherFaceIdx]); 
				gp_Dir otherdir = normalList[otherFaceIdx];

				std::string otherTpe = getType(surfaceList[otherFaceIdx]);
				if (currentType == "IfcWindow" || currentType == "IfcDoor") { continue; }

				if (currentIdx == otherFaceIdx) { continue; }
				if (evalList[otherFaceIdx] == 1) { continue; }
				if (!currentdir.IsParallel(otherdir, precision)) { continue; }

				if (!helperFunctions::shareEdge(currentFace, otherFace)) { continue; }

				if (std::find(mergedSurfaceIdxList.begin(), mergedSurfaceIdxList.end(), otherFaceIdx) == mergedSurfaceIdxList.end())
				{
					mergedSurfaceIdxList.emplace_back(otherFaceIdx);
					continue;
				}
			}
		}

		if (mergedSurfaceIdxList.size() == currentSurfaceIdxSize)
		{
			if (mergedSurfaceIdxList.size() == 1)
			{
				if constexpr (usePair)
				{
					cleanedFaceList.emplace_back(surfaceList[mergedSurfaceIdxList[0]]);
				}
				else
				{
					cleanedFaceList.emplace_back(getFace(surfaceList[mergedSurfaceIdxList[0]]));
				}		
			}
			else
			{
				std::vector<TopoDS_Face> tempFaceList;
				std::map<std::string, double> functionAreaMap;
				for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
				{
					TopoDS_Face currentFace = getFace(surfaceList[mergedSurfaceIdxList[i]]);
					std::string currentType = getType(surfaceList[mergedSurfaceIdxList[i]]);
					double currentArea = helperFunctions::computeArea(currentFace);
					tempFaceList.emplace_back(currentFace);

					if (functionAreaMap.find(currentType) == functionAreaMap.end())
					{
						functionAreaMap.emplace(currentType, 0);
					}
					functionAreaMap[currentType] += currentArea;
				}
				TopoDS_Face mergedFace = mergeFaces(tempFaceList);
				if (!mergedFace.IsNull())
				{ 
					
					// get the surface type of the type that has the largest area
					double maxArea = 0;
					std::string surfaceTypeName = "";
					for (const auto& [typeName, areaValue] : functionAreaMap)
					{
						if (areaValue > maxArea)
						{
							maxArea = areaValue;
							surfaceTypeName = typeName;
						}
					}

					if constexpr (usePair)
					{
						cleanedFaceList.emplace_back(std::make_pair(mergedFace, surfaceTypeName));
					}
					else
					{
						cleanedFaceList.emplace_back(mergedFace);
					}
				}
				else
				{
					if constexpr (usePair)
					{
						for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
						{
							cleanedFaceList.emplace_back(surfaceList[mergedSurfaceIdxList[i]]);
						}
					}
					else
					{
						cleanedFaceList.insert(cleanedFaceList.end(), tempFaceList.begin(), tempFaceList.end());
					}
					
				}
			}

			bool newSetFound = false;
			for (size_t i = 0; i < surfaceList.size(); i++)
			{
				if (evalList[i] == 0)
				{
					mergedSurfaceIdxList = { (int) i };
					newSetFound = true;
					break;
				}
			}
			if (newSetFound) { continue; }
			break;
		}
	}
	return cleanedFaceList;
}

bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> CJGeoCreator::indexUniqueFaces(const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>>& faceIndx)
{
	double precision = SettingsCollection::getInstance().precision();

	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx; // pair bbox | extruded shape faces
	for (const auto& [currentBox, currentFace] : faceIndx)
	{
		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);
		std::vector<std::pair<BoostBox3D, TopoDS_Face>> qResult;
		qResult.clear();
		faceIndx.query(bgi::intersects(currentBox), std::back_inserter(qResult));

		bool isDub = false;
		for (const auto& [otherBox, otherFace] : qResult)
		{
			if (currentFace.IsEqual(otherFace)) { continue; }

			if (!currentNormal.IsParallel(helperFunctions::computeFaceNormal(otherFace), precision)) { continue; }
			if (!helperFunctions::isSame(currentBox, otherBox)) { continue; }
			//TODO: add actual comparison

			isDub = true;
			break;
		}
		if (isDub)
		{
			continue;
		}
		cuttingFaceIdx.insert(std::make_pair(currentBox, currentFace));
	}
	return cuttingFaceIdx;
}


TopoDS_Face CJGeoCreator::mergeFaces(const std::vector<TopoDS_Face>& mergeFaces) {
	if (mergeFaces.size() == 1) { return mergeFaces[0]; }

	gp_Vec clusterNormal = helperFunctions::computeFaceNormal(mergeFaces[0]);
	gp_Vec horizontalNormal = gp_Vec(0, 0, 1);

	gp_Trsf transform;
	std::vector<TopoDS_Face> mergingFaces;
	if (clusterNormal.Magnitude() < 1e-6)
	{
		return TopoDS_Face();
	}

	if (!clusterNormal.IsParallel(horizontalNormal, 1e-6))
	{
		std::optional<gp_Pnt> optionalbasePoint = helperFunctions::getPointOnFace(mergeFaces[0]);

		if (optionalbasePoint == std::nullopt) { return TopoDS_Face(); }

		gp_Vec normalCrossProduct = clusterNormal ^ horizontalNormal;
		gp_Ax1 rotationAxis(*optionalbasePoint, normalCrossProduct);
		Standard_Real rotationAngle = clusterNormal.AngleWithRef(horizontalNormal, rotationAxis.Direction());

		transform.SetRotation(rotationAxis, rotationAngle);

		for (const TopoDS_Face& face : mergeFaces) {
			BRepBuilderAPI_Transform transformer(face, transform);
			if (transformer.IsDone()) {
				TopoDS_Face dubface = TopoDS::Face(transformer.Shape());
				mergingFaces.emplace_back(dubface);
			}
		}
	}
	else
	{
		mergingFaces = mergeFaces;
	}

	std::vector<TopoDS_Face> cleanedMergingFaces = helperFunctions::removeDubFaces(mergingFaces);
	std::vector<TopoDS_Face> mergedFaces = helperFunctions::planarFaces2Outline(cleanedMergingFaces);
	if (!mergedFaces.size())
	{
		//TODO: add error
		return TopoDS_Face();
	}

	std::vector<TopoDS_Wire> wireList;
	for (TopExp_Explorer expl(mergedFaces[0], TopAbs_WIRE); expl.More(); expl.Next())
	{
		wireList.emplace_back(TopoDS::Wire(expl.Current()));
	}
	std::vector<TopoDS_Wire> cleanWireList = helperFunctions::cleanWires(wireList);
	if (cleanWireList.size() == 0) { return TopoDS_Face(); }
	TopoDS_Face cleanedFace = helperFunctions::wireCluster2Faces(cleanWireList);
	if (cleanedFace.IsNull()) { return TopoDS_Face(); }
	transform.Invert();
	BRepBuilderAPI_Transform transformer(cleanedFace, transform);
	return TopoDS::Face(transformer.Shape());
}


void CJGeoCreator::printTime(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime) {
	long long duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentSuccesFinished) << 
			std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << UnitStringEnum::getString(UnitStringID::milliseconds) << std::endl;
	}
	else {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentSuccesFinished) << 
			std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << UnitStringEnum::getString(UnitStringID::seconds) << std::endl;
	}
}


bool CJGeoCreator::surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape) //TODO: might be dup
{
	double precision = SettingsCollection::getInstance().precision();
	for (TopExp_Explorer explorer(encapsulatedShape, TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& outerSurface = TopoDS::Face(explorer.Current());
		bool encapsulated = true;

		for (TopExp_Explorer explorer2(innerSurface, TopAbs_VERTEX); explorer2.More(); explorer2.Next())
		{
			const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer2.Current());

			if (!helperFunctions::pointOnShape(outerSurface, BRep_Tool::Pnt(vertex)))
			{
				encapsulated = false;
				break;
			}
		}
		if (encapsulated)
		{
			return true;
		}
	}
	return false;
}


TopoDS_Face makeFace(const std::vector<gp_Pnt>& voxelPointList, const std::vector<int>& pointFaceIndx) {
	gp_Pnt p0(voxelPointList[pointFaceIndx[0]]);
	gp_Pnt p1(voxelPointList[pointFaceIndx[1]]);
	gp_Pnt p2(voxelPointList[pointFaceIndx[2]]);
	gp_Pnt p3(voxelPointList[pointFaceIndx[3]]);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3);
	return BRepBuilderAPI_MakeFace(wire);
}

bool CJGeoCreator::useFace(const TopoDS_Face& face, gp_Pnt* centerPoint)
{
	// ignore face when the area is very small
	GProp_GProps gprops;
	BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
	if (gprops.Mass() < 0.001) { return false; }
	// ignore if the z component of normal is 0 
	gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);
	if (std::abs(faceNormal.Z()) < 0.001) { return false; }
	*centerPoint = gprops.CentreOfMass();
	return true;
}

std::vector<FaceComplex> CJGeoCreator::groupFaces(const std::vector<TopoDS_Face>& inputFaceList)
{
	std::vector<FaceComplex> complexList;

	bgi::rtree<Value, bgi::rstar<25>> shapeIdx;
	std::vector<int>evalList(inputFaceList.size());
	for (int i = 0; i < inputFaceList.size(); i++)
	{
		TopoDS_Face currentFace = inputFaceList[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		shapeIdx.insert(std::make_pair(bbox, i));
	}
	for (int i = 0; i < inputFaceList.size(); i++)
	{
		// group surfaces
		if (evalList[i]) { continue; }
		evalList[i] = 1;
		std::vector<TopoDS_Face> evalFaceList = { inputFaceList[i] };
		std::vector<TopoDS_Face> totalFaceCluster = { inputFaceList[i] };

		while (evalFaceList.size())
		{
			std::vector<TopoDS_Face> bufferFaceList;
			for (const TopoDS_Face& currentFace : evalFaceList)
			{
				std::vector<Value> qResult;
				qResult.clear();
				shapeIdx.query(bgi::intersects(
					helperFunctions::createBBox(currentFace)), std::back_inserter(qResult));

				for (const auto& [bbox, otherIndx] : qResult)
				{
					if (evalList[otherIndx]) { continue; }

					TopoDS_Face otherFace = inputFaceList[otherIndx];
					BRepExtrema_DistShapeShape distanceFaceCalc(currentFace, otherFace);
					distanceFaceCalc.Perform();

					if (distanceFaceCalc.Value() > 0.001)
					{
						continue;
					}
					evalList[otherIndx] = 1;
					bufferFaceList.emplace_back(otherFace);
					totalFaceCluster.emplace_back(otherFace);
				}
			}
			evalFaceList = bufferFaceList;
		}
		FaceComplex faceComplex;
		faceComplex.faceList_ = totalFaceCluster;
		complexList.emplace_back(faceComplex);
	}
	return complexList;
}

std::vector<TopoDS_Shape> CJGeoCreator::beamProjection(DataManager* h)
{
	auto startTime = std::chrono::steady_clock::now();
	std::vector<int> boxelIdx = voxelGrid_->getTopBoxelIndx();
	std::unordered_set<int> topValueIdxset;
	std::vector<TopoDS_Shape> topObjectList;

	for (int currentVoxelIdx : boxelIdx)
	{
		while (true)
		{
			const voxel& currentVoxel = voxelGrid_->getVoxel(currentVoxelIdx);
			const std::vector<Value>& internalValueList = currentVoxel.getInternalProductList();

			if (!internalValueList.empty())
			{
				for (const Value& currentValue : internalValueList)
				{
					topValueIdxset.insert(currentValue.second);
				}
				break;
			}
			currentVoxelIdx = voxelGrid_->getLowerNeighbour(currentVoxelIdx);
			if (currentVoxelIdx == -1) { break; }
		}
	}

	for (int currentValue : topValueIdxset)
	{
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(currentValue);
		TopoDS_Shape currentShape;
		if (lookup->hasSimpleShape()) { currentShape = lookup->getSimpleShape(); }
		else { currentShape = lookup->getProductShape(); }
		topObjectList.emplace_back(currentShape);
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return topObjectList;
}

std::vector<TopoDS_Shape> CJGeoCreator::getUniqueShapedObjects(const std::vector<TopoDS_Shape>& topObjectList)
{
	std::vector<TopoDS_Shape> uniqueTopObjects;
	std::vector<gp_Pnt> uniqueCenterPoint;
	std::vector<double> uniqueTopMass;
	std::vector<double> uniqueTopArea;

	for (const TopoDS_Shape& currentShape : topObjectList) //TODO: this could be using indexing
	{
		bool isDub = false;

		GProp_GProps volGprops;
		BRepGProp::VolumeProperties(currentShape, volGprops);

		GProp_GProps surfGprops;
		BRepGProp::SurfaceProperties(currentShape, surfGprops);

		gp_Pnt currentCenterPoint = volGprops.CentreOfMass();
		double currentMass = volGprops.Mass();
		double currentArea = surfGprops.Mass();

		for (size_t j = 0; j < uniqueTopObjects.size(); j++)
		{
			gp_Pnt otherCenterPoint = uniqueCenterPoint[j];
			if (!currentCenterPoint.IsEqual(otherCenterPoint, 1e-6)) { continue; }
			
			double otherMass = uniqueTopMass[j];
			if (abs(currentMass - otherMass) > 1e-6) { continue; }
			
			double otherArea = uniqueTopArea[j];
			if (abs(currentArea - otherArea) > 1e-6) { continue; }

			isDub = true;
			break;
		}

		if (isDub) { continue; }

		uniqueTopObjects.emplace_back(currentShape);
		uniqueCenterPoint.emplace_back(currentCenterPoint);
		uniqueTopMass.emplace_back(currentMass);
		uniqueTopArea.emplace_back(currentArea);
	}
	return uniqueTopObjects;
}

std::vector<TopoDS_Shape> CJGeoCreator::getTopObjects(DataManager* h)
{
	std::vector<TopoDS_Shape> topObjects = beamProjection(h);
	std::vector<TopoDS_Shape> topCleanObjects = getUniqueShapedObjects(topObjects);
	return topCleanObjects;
}

std::vector<TopoDS_Face> CJGeoCreator::createRoofOutline(const std::vector<RCollection>& rCollectionList)
{
	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	std::vector<TopoDS_Face> projectedFaceList;
	for (const RCollection& currentGroup : rCollectionList)
	{
		projectedFaceList.emplace_back(currentGroup.getProjectedFace());
	}

	// create plane on which the projection has to be made
	gp_Pnt lll;
	gp_Pnt urr;
	helperFunctions::bBoxDiagonal(projectedFaceList, &lll, &urr);

	gp_Pnt p0 = gp_Pnt(lll.X() - 10, lll.Y() - 10, 0);
	gp_Pnt p1 = gp_Pnt(urr.X() + 10, urr.Y() + 10, 0);

	TopoDS_Face cuttingPlane = helperFunctions::createHorizontalFace(p0, p1, 0, 0);
	std::vector<TopoDS_Face> mergedSurfaces = helperFunctions::planarFaces2Outline(projectedFaceList, cuttingPlane);
	printTime(startTime, std::chrono::steady_clock::now());
	return mergedSurfaces;
}




void CJGeoCreator::reduceSurfaces(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList)
{
	auto startTime = std::chrono::steady_clock::now();
	// split the range over cores
	int coreUse = SettingsCollection::getInstance().threadcount();
	if (coreUse > inputShapes.size())
	{
		while (coreUse > inputShapes.size()) { coreUse /= 2; }
	}
	int splitListSize = static_cast<int>(floor(inputShapes.size() / coreUse));

	std::vector<std::thread> threadList;
	std::mutex processMutex;

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = inputShapes.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? inputShapes.end() : startIdx + splitListSize;

		std::vector<TopoDS_Shape> sublist(startIdx, endIdx);

		threadList.emplace_back([this, sublist, &processMutex, &shapeIdx, &shapeList]() {reduceSurface(sublist, processMutex, shapeIdx, shapeList); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
}


void CJGeoCreator::reduceSurface(const std::vector<TopoDS_Shape>& inputShapes, std::mutex& processMutex, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<std::shared_ptr<SurfaceGridPair>>* shapeList)
{
	for (size_t i = 0; i < inputShapes.size(); i++)
	{
		std::vector<std::shared_ptr<SurfaceGridPair>> coarseFilteredTopSurfacePairList = getObjectTopSurfaces(inputShapes[i]);
		for (const auto& coarseFilteredTopSurfacePair : coarseFilteredTopSurfacePairList)
		{
			std::unique_lock<std::mutex> rtreeLock(processMutex);
			auto rtreePair = std::make_pair(helperFunctions::createBBox(coarseFilteredTopSurfacePair->getFace()), static_cast<int>(shapeList->size()));
			shapeIdx->insert(rtreePair);
			shapeList->emplace_back(coarseFilteredTopSurfacePair);
			rtreeLock.unlock();
		}
		hasTopFaces_ = true;
	}
}

std::vector<std::shared_ptr<SurfaceGridPair>> CJGeoCreator::FinefilterSurfaces(const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList)
{
	auto startTime = std::chrono::steady_clock::now();
	// split the range over cores
	int coreUse = SettingsCollection::getInstance().threadcount();
	if (coreUse > shapeList.size())
	{
		while (coreUse > shapeList.size()) { coreUse /=2; }
	}
	int splitListSize = static_cast<int>(floor(shapeList.size() / coreUse));

	std::vector<std::thread> threadList;
	std::mutex processMutex;

	std::vector<std::shared_ptr<SurfaceGridPair>> fineFilteredShapeList;
	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = shapeList.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? shapeList.end() : startIdx + splitListSize;

		std::vector<std::shared_ptr<SurfaceGridPair>> sublist(startIdx, endIdx);
		threadList.emplace_back([this, sublist, &shapeList, &processMutex, &fineFilteredShapeList]() {
			FinefilterSurface(sublist, shapeList, processMutex, &fineFilteredShapeList); 
			});
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return fineFilteredShapeList;
}

void CJGeoCreator::FinefilterSurface(
	const std::vector<std::shared_ptr<SurfaceGridPair>>& shapeList,
	const std::vector<std::shared_ptr<SurfaceGridPair>>& otherShapeList,
	std::mutex& processMutex,
	std::vector<std::shared_ptr<SurfaceGridPair>>* fineFilteredShapeList
)
{
	for (const std::shared_ptr<SurfaceGridPair>& currentSurfacePair : shapeList)
	{
		if (!currentSurfacePair->testIsVisable(otherShapeList)) { continue; }
		std::lock_guard<std::mutex> faceLock(processMutex);
		fineFilteredShapeList->emplace_back(currentSurfacePair);
	}
	return;
}

std::vector<std::shared_ptr<SurfaceGridPair>> CJGeoCreator::getObjectTopSurfaces(const TopoDS_Shape& shape)
{
	// coarse pre processing of the surfaces
	std::vector<std::shared_ptr<SurfaceGridPair>> gridPairList;
	std::vector<gp_Pnt> centerpointHList;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;

	// index the valid surfaces
	for (TopExp_Explorer expl(shape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		gp_Pnt centerPoint;
		if (!useFace(face, &centerPoint)) { continue; }

		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(face);
		spatialIndex.insert(std::make_pair(bbox, gridPairList.size()));
		gridPairList.emplace_back(std::make_shared<SurfaceGridPair>(face));
		centerpointHList.emplace_back(gp_Pnt(centerPoint.X(), centerPoint.Y(), 0));
	}

	std::vector<std::shared_ptr<SurfaceGridPair>> visibleSurfaces;
	for (int i = 0; i < gridPairList.size(); i++)
	{
		std::shared_ptr<SurfaceGridPair> currentGroup = gridPairList[i];
		if (!currentGroup->isVisible()) { continue; }

		TopoDS_Face currentFace = currentGroup->getFace();
		gp_Pnt currentCenter = centerpointHList[i];

		// ignore lowest if identical projected points
		double height = currentGroup->getAvHeight();
		int vertCount = currentGroup->getVertCount();

		// querry
		gp_Pnt upperPoint = currentGroup->getURRPoint();
		upperPoint.Translate(gp_Vec(0, 0, 1000));
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentGroup->getLLLPoint(), upperPoint);
		std::vector<Value> qResult;
		qResult.clear();
		spatialIndex.query(bgi::intersects(
			bbox), std::back_inserter(qResult));

		// cull faces completely overlapped by one other face
		for (size_t j = 0; j < qResult.size(); j++)
		{
			int otherIdx = qResult[j].second;
			if (i == otherIdx) { continue; }

			std::shared_ptr<SurfaceGridPair> otherGroup = gridPairList[otherIdx];
			double otherHeight = otherGroup->getAvHeight();
			if (height > otherHeight) { continue; }
			if (!otherGroup->isVisible()) { continue; }
			if (!currentCenter.IsEqual(centerpointHList[otherIdx], 1e-6)) { continue; }

			TopoDS_Face otherFace = otherGroup->getFace();
			if (helperFunctions::faceFaceOverlapping(otherFace, currentFace)) {
				currentGroup->setIsHidden();
				break;
			}
		}
		if (!currentGroup->isVisible()) { continue; }
		// cull faces completely overlapped by other faces
		std::vector<std::shared_ptr<SurfaceGridPair>> rayReceivingPairList;
		for (size_t j = 0; j < qResult.size(); j++)
		{
			int otherIdx = qResult[j].second;
			if (i == otherIdx) { continue; }
			std::shared_ptr<SurfaceGridPair> otherGroup = gridPairList[otherIdx];
			if (!otherGroup->isVisible()) { continue; }
			rayReceivingPairList.emplace_back(otherGroup);
		}
		if (!currentGroup->isVisible()) { continue; }
		if (currentGroup->testIsVisable(rayReceivingPairList, false))
		{
			visibleSurfaces.emplace_back(currentGroup);
		}
	}
	return visibleSurfaces;
}


std::vector<std::shared_ptr<CJT::CityObject>> CJGeoCreator::makeStoreyObjects(DataManager* h)
{
	std::vector<std::shared_ptr<CJT::CityObject>> cityStoreyObjects;
	std::vector<double> elevList;
	for (int i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcBuildingStorey::list::ptr storeyList = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcBuildingStorey>();
		for (auto it = storeyList->begin(); it != storeyList->end(); ++it)
		{
			IfcSchema::IfcBuildingStorey* storeyObject = *it;
			double storeyElevation = storeyObject->Elevation().get() * h->getScaler(0);

			// check if the storey object is already made
			bool isDub = false;
			for (size_t i = 0; i < elevList.size(); i++)
			{
				double otherStoreyElevation = elevList[i];
				if (abs(storeyElevation - otherStoreyElevation) > 1e-6) { continue; }

				// if already made merge the data
				std::vector<std::string> guidList = cityStoreyObjects[i]->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)].get<std::vector<std::string>>(); ;
				guidList.emplace_back(storeyObject->GlobalId());
				cityStoreyObjects[i]->removeAttribute(CJObjectEnum::getString(CJObjectID::ifcGuid));
				cityStoreyObjects[i]->addAttribute<std::vector<std::string>>(CJObjectEnum::getString(CJObjectID::ifcGuid), { guidList });
				isDub = true;
				break;
			}
			if (isDub) { continue; }
			// if new add a new object

			CJT::CityObject cityStoreyObject;
			cityStoreyObject.setType(CJT::Building_Type::BuildingStorey);

			if (storeyObject->Name().has_value())
			{
				cityStoreyObject.setName(storeyObject->Name().get());
				cityStoreyObject.addAttribute(CJObjectEnum::getString(CJObjectID::ifcName), storeyObject->Name().get());
			}
			if (storeyObject->LongName().has_value())
			{
				cityStoreyObject.addAttribute(CJObjectEnum::getString(CJObjectID::ifcLongName), storeyObject->LongName().get());
			}
			cityStoreyObject.addAttribute<std::vector<std::string>>(CJObjectEnum::getString(CJObjectID::ifcGuid), { storeyObject->GlobalId() });
			cityStoreyObject.addAttribute(CJObjectEnum::getString(CJObjectID::ifcElevation), storeyObject->Elevation().get());
			cityStoreyObjects.emplace_back(std::make_shared< CJT::CityObject>(cityStoreyObject));
			elevList.emplace_back(storeyElevation);
		}
	}
	return cityStoreyObjects;
}

std::vector<std::shared_ptr<CJT::CityObject>> CJGeoCreator::makeRoomObjects(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& cityStoreyObjects)
{
	std::vector<std::shared_ptr<CJT::CityObject>> cityRoomObjects;

	IfcSchema::IfcSpace::list::ptr spaceList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcSpace>();

	for (auto spaceIt = spaceList->begin(); spaceIt != spaceList->end(); ++spaceIt)
	{
		IfcSchema::IfcSpace* spaceObject = *spaceIt;

		// check if proper kind of room object
		if (spaceObject->CompositionType() == IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT)
		{
			std::shared_ptr<CJT::CityObject> cjRoomObject = std::make_unique<CJT::CityObject>();
			 
			// store generic data
			if (spaceObject->Name().has_value())
			{
				cjRoomObject->setName(*spaceObject->Name());
				cjRoomObject->addAttribute(CJObjectEnum::getString(CJObjectID::ifcName), spaceObject->Name().get());
			}
			if (spaceObject->LongName().has_value())
			{
				cjRoomObject->addAttribute(CJObjectEnum::getString(CJObjectID::ifcLongName), spaceObject->LongName().get());
			}
			cjRoomObject->addAttribute(CJObjectEnum::getString(CJObjectID::ifcGuid), spaceObject->GlobalId());
			cjRoomObject->setType(CJT::Building_Type::BuildingRoom);

			//store added data
			std::vector<nlohmann::json> attributeList = helperFunctions::collectPropertyValues(spaceObject->GlobalId(), h->getSourceFile(0));	
			for (nlohmann::json attributeObject : attributeList)
			{
				for (auto jsonObIt = attributeObject.begin(); jsonObIt != attributeObject.end(); ++jsonObIt) {
					cjRoomObject->addAttribute(jsonObIt.key(), jsonObIt.value());
				}
			}

			// get rooms storey
			bool storeyFound = false;

#if defined(USE_IFC4) || defined(USE_IFC4x3)
			IfcSchema::IfcRelAggregates::list::ptr relAggregateList = spaceObject->Decomposes();
#else
			IfcSchema::IfcRelDecomposes::list::ptr relAggregateList = spaceObject->Decomposes();
#endif
			for (auto aggregateIt = relAggregateList->begin(); aggregateIt != relAggregateList->end(); ++aggregateIt)
			{
#if defined(USE_IFC4) || defined(USE_IFC4x3)

				IfcSchema::IfcRelAggregates* ifcRelAggregate = *aggregateIt;
#else
				IfcSchema::IfcRelDecomposes* ifcRelAggregate = *aggregateIt;
#endif
				IfcSchema::IfcObjectDefinition* potentialStorey = ifcRelAggregate->RelatingObject();

				if (potentialStorey->data().type()->name() != "IfcBuildingStorey")
				{
					continue;
				}
				std::string targetStoreyGuid = potentialStorey->GlobalId();

				for (std::shared_ptr<CJT::CityObject> cjtStorey : cityStoreyObjects)
				{
					std::vector<std::string> test = cjtStorey->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)];
					for (const std::string& storeyGUI : test)
					{
						if (storeyGUI == targetStoreyGuid)
						{
							cjtStorey->addChild(cjRoomObject);
							storeyFound = true;
							break;
						}
					}

				}
				if (storeyFound)
				{
					cityRoomObjects.emplace_back(cjRoomObject);
					break;
				}
			}
		}
	}

	//TODO: clean the spatial index?

	return cityRoomObjects;
}

CJT::GeoObject CJGeoCreator::makeLoD00(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD00) << std::endl;

	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = settingsCollection.gridRotation();
	TopoDS_Shape floorProjection = helperFunctions::createHorizontalFace(lll, urr, -rotationAngle, settingsCollection.footprintElevation());

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(floorProjection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD00));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(floorProjection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD00));
	}

	CJT::GeoObject geoObject = kernel->convertToJSON(floorProjection, "0.0");
	std::map<std::string, std::string> semanticData;
	semanticData.emplace(CJObjectEnum::getString(CJObjectID::CJType) , CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	geoObject.appendSurfaceData(semanticData);
	geoObject.appendSurfaceTypeValue(0);
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD02(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD02) << std::endl;

	SettingsCollection& settingCollection = SettingsCollection::getInstance();
	if (!hasTopFaces_ && settingCollection.footPrintBased()) { return std::vector< CJT::GeoObject>(); }

	std::vector< CJT::GeoObject> geoObjectCollection;

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	std::map<std::string, std::string> semanticFootData;
	semanticFootData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));

	gp_Pnt urr = h->getUrrPoint();

	std::vector<TopoDS_Shape> faceCopyCollection;
	if (settingCollection.makeRoofPrint())
	{	
		// make the roof
		for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			TopoDS_Shape roofOutline = buildingSurfaceData.getRoofOutline();

			gp_Trsf trsf;
			trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)),  -settingCollection.gridRotation());
			if (hasFootprints_) { trsf.SetTranslationPart(gp_Vec(0, 0, urr.Z())); }
			roofOutline.Move(trsf);
			faceCopyCollection.emplace_back(roofOutline);

			CJT::GeoObject geoObject = kernel->convertToJSON(roofOutline, "0.2");
			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (hasFootprints_) 
	{ 
		// make the footprint
		for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			TopoDS_Shape footprint = buildingSurfaceData.getFootPrint();
			if (footprint.IsNull()) { continue; }

			gp_Trsf trsf;
			trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingCollection.gridRotation());
			footprint.Move(trsf);
			faceCopyCollection.emplace_back(footprint);

			CJT::GeoObject geoObject = kernel->convertToJSON(footprint, "0.2");
			geoObject.appendSurfaceData(semanticFootData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (settingCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(faceCopyCollection, settingCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD02));
	}

	if (settingCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(faceCopyCollection, settingCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD02));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectCollection;
}

void CJGeoCreator::monitorStoreys(std::mutex& storeyMutex, std::map<std::string, int>& progressMap, int totalProcesses)
{
	bool running = true;
	while (running)
	{
		std::unique_lock<std::mutex> faceLock(storeyMutex);
		running = false;
		int ongoingProcessNum = 0;
		int finishedProcessesNum = 0;
		int failedProcessesNum = 0;

		for (const auto& [stringKey, status] : progressMap)
		{
			if (!status) { ongoingProcessNum++; running = true; }
			else if (status == 1) { finishedProcessesNum++; }
			else if (status == 2) { failedProcessesNum++; }
		}

		std::cout 
			<< "\tTotal Storeys: " << totalProcesses 
			<< "; Active Proceses: " << ongoingProcessNum 
			<< "; Finished Processes: " << finishedProcessesNum + failedProcessesNum << "      \r";

		faceLock.unlock();

		if (running)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	std::cout << "\n";
	return;
}


void CJGeoCreator::make2DStoreys(
	DataManager* h, 
	CJT::Kernel* kernel, 
	std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, 
	int unitScale,
	bool is03
) 
{
	auto startTime = std::chrono::steady_clock::now();

	if (is03)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingStoreys03) << std::endl;
	}
	else
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingStoreys02) << std::endl;
	}

	std::vector<std::thread> threadList;
	std::mutex storeyMutex;
	std::map<std::string, int> storyProgressList;

	std::vector<TopoDS_Shape> copyGeoList;
	for (const std::shared_ptr<CJT::CityObject>& storeyCityObject : storeyCityObjects)
	{
		//make2DStorey(storeyMutex, h, kernel, storeyCityObject, copyGeoList, storyProgressList, unitScale, is03);
		threadList.emplace_back([&]() {make2DStorey(storeyMutex ,h, kernel, storeyCityObject, copyGeoList, storyProgressList, unitScale, is03); });
	}

	threadList.emplace_back([&] {monitorStoreys(storeyMutex, storyProgressList, storeyCityObjects.size()); });

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}

	if (SettingsCollection::getInstance().createSTEP())
	{
		if (is03)
		{
			helperFunctions::writeToSTEP(copyGeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD03Interior));
		}
		else
		{
			helperFunctions::writeToSTEP(copyGeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD02Interior));
		}
	}

	if (SettingsCollection::getInstance().createOBJ())
	{
		if (is03)
		{
			helperFunctions::writeToOBJ(copyGeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD03Interior));
		}
		else
		{
			helperFunctions::writeToOBJ(copyGeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD02Interior));
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return;
}

void CJGeoCreator::make2DStorey(
	std::mutex& storeyMutex,
	DataManager* h,
	CJT::Kernel* kernel,
	const std::shared_ptr<CJT::CityObject>& storeyCityObject,
	std::vector<TopoDS_Shape>& copyGeoList,
	std::map<std::string, int>& progressMap,
	int unitScale,
	bool is03
)
{
	std::string LoDString = "0.2";
	if (is03) { LoDString = "0.3"; }
	double storeyUserBuffer = SettingsCollection::getInstance().horizontalSectionOffset();

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	std::vector<std::string> storeyGuidList = storeyCityObject->getAttributes()["IFC Guid"];
	std::vector< IfcSchema::IfcBuildingStorey*> ifcStoreyList = fetchStoreyObjects(h, storeyGuidList);

	IfcSchema::IfcBuildingStorey* ifcStorey = ifcStoreyList[0];
	IfcSchema::IfcObjectPlacement* storeyObjectPlacement = ifcStorey->ObjectPlacement();
	double storeyElevation = helperFunctions::getObjectZOffset(storeyObjectPlacement, false) * h->getScaler(0);
	double userStoreyElevation = ifcStorey->Elevation().get() * h->getScaler(0);

	std::unique_lock<std::mutex> faceLock(storeyMutex);
	std::string storeyKey = std::to_string(userStoreyElevation) + " (" + std::to_string(storeyElevation) + ")";
	progressMap.emplace(storeyKey, 0);
	faceLock.unlock();

	std::vector<TopoDS_Face> storeySurfaceList;
	std::vector<TopoDS_Face> storeyExternalSurfaceList;
	if (is03)
	{
		makeFloorSectionComplex(storeySurfaceList, storeyExternalSurfaceList, h, storeyElevation + storeyUserBuffer, ifcStoreyList);
	}
	else
	{
		makeFloorSection(storeySurfaceList, h, storeyElevation + storeyUserBuffer);
		std::unique_lock<std::mutex> faceLock(storeyMutex);
		LoD02Plates_.emplace(storeyElevation, storeySurfaceList);
		faceLock.unlock();
	}
	if (!storeySurfaceList.size())
	{
		ErrorCollection::getInstance().addError(ErrorID::errorStoreyFailed, storeyGuidList[0]);
		progressMap[storeyKey] = 2;
		return;
	}

	std::map<std::string, std::string> semanticExternalStoreyData;
	std::map<std::string, std::string> semanticStoreyData;
	semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeFloor));
	semanticExternalStoreyData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeOuterFloor));

	double totalArea = 0;
	trsf.SetTranslationPart(gp_Vec(0, 0, -storeyUserBuffer));

	for (const TopoDS_Face& currentStoreyFace : storeySurfaceList)
	{
		std::lock_guard<std::mutex> faceLock(storeyMutex);
		TopoDS_Shape movedStoreyFace = currentStoreyFace.Moved(trsf);

		if (SettingsCollection::getInstance().createSTEP() || SettingsCollection::getInstance().createOBJ())
		{
			copyGeoList.emplace_back(movedStoreyFace);
		}

		CJT::GeoObject geoObject = kernel->convertToJSON(movedStoreyFace, LoDString);
		geoObject.appendSurfaceData(semanticStoreyData);
		geoObject.appendSurfaceTypeValue(0);
		storeyCityObject->addGeoObject(geoObject);

		totalArea += helperFunctions::computeArea(currentStoreyFace);
	}

	for (const TopoDS_Face& currentStoreyFace : storeyExternalSurfaceList)
	{
		std::lock_guard<std::mutex> faceLock(storeyMutex);
		TopoDS_Shape movedStoreyFace = currentStoreyFace.Moved(trsf);

		if (SettingsCollection::getInstance().createSTEP() || SettingsCollection::getInstance().createOBJ())
		{
			copyGeoList.emplace_back(movedStoreyFace);
		}

		CJT::GeoObject geoObject = kernel->convertToJSON(movedStoreyFace, LoDString);
		geoObject.appendSurfaceData(semanticExternalStoreyData);
		geoObject.appendSurfaceTypeValue(0);
		storeyCityObject->addGeoObject(geoObject);
	}
	storeyCityObject->addAttribute(CJObjectEnum::getString(CJObjectID::EnvLoDfloorArea) + LoDString, totalArea);
	progressMap[storeyKey] = 1;
	return;
}

void CJGeoCreator::makeSimpleLodRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale) {
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	
	IfcSchema::IfcSpace::list::ptr spaceList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcSpace>();

	std::vector<TopoDS_Shape> copyLoD02GeoList;
	std::vector<TopoDS_Shape> copyLoD12GeoList;
	std::vector<TopoDS_Shape> copyLoD22GeoList;

	for (auto spaceIt = spaceList->begin(); spaceIt != spaceList->end(); ++spaceIt)
	{
		//TODO: make function?
		// find the matching cityspace object
		IfcSchema::IfcSpace* spaceIfcObject = *spaceIt;
		std::string spaceGuid = spaceIfcObject->GlobalId();

		bool spaceFound = false;
		std::shared_ptr<CJT::CityObject> matchingCityRoomObject;
		for (std::shared_ptr<CJT::CityObject> roomCityObject : roomCityObjects)
		{
			if (spaceGuid == roomCityObject->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)])
			{
				spaceFound = true;
				matchingCityRoomObject = roomCityObject;
				break;
			}
		}
		if (!spaceFound) { continue; }

		// get height values
		TopoDS_Shape spaceShape = h->getObjectShape(spaceIfcObject, false);
		double lowestZ = helperFunctions::getLowestZ(spaceShape);
		double highestZ = helperFunctions::getHighestZ(spaceShape);

		// get the top faces
		std::vector<TopoDS_Face> flatFaceList;
		std::vector<TopoDS_Face> topFaceList;
		for (TopExp_Explorer faceExp(spaceShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
			TopoDS_Face currentFace = TopoDS::Face(faceExp.Current());

			if (abs(helperFunctions::computeFaceNormal(currentFace).Z()) < SettingsCollection::getInstance().precisionCoarse()) { continue; }

			std::vector<gp_Pnt> facePointList = helperFunctions::getPointListOnFace(currentFace);
			for (const gp_Pnt& facePoint : facePointList)
			{
				bool clearLine = true;
				gp_Pnt topPoint = gp_Pnt(facePoint.X(), facePoint.Y(), facePoint.Z() + 10000);

				for (TopExp_Explorer otherFaceExp(spaceShape, TopAbs_FACE); otherFaceExp.More(); otherFaceExp.Next()) {
					TopoDS_Face otherFace = TopoDS::Face(otherFaceExp.Current());

					if (abs(helperFunctions::computeFaceNormal(otherFace).Z()) < SettingsCollection::getInstance().precisionCoarse()) { continue; }
					if (currentFace.IsSame(otherFace)) { continue; }


					// check if face is the same face but translated upwards
					if (helperFunctions::faceFaceOverlapping(otherFace, currentFace))
					{
						clearLine = false;
						break;
					}

					// check if covered via raycasting
					TopLoc_Location loc;
					auto mesh = BRep_Tool::Triangulation(otherFace, loc);

					for (int j = 1; j <= mesh.get()->NbTriangles(); j++) 
					{
						const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

						std::vector<gp_Pnt> trianglePoints{
							mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
							mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
							mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
						};

						if (helperFunctions::triangleIntersecting({ facePoint, topPoint }, trianglePoints))
						{
							clearLine = false;
							break;
						}
					}
					if (!clearLine){ break; }
				}
				if (!clearLine){ continue; }

				if (settingsCollection.make02() || settingsCollection.make12())
				{
					flatFaceList.emplace_back(helperFunctions::projectFaceFlat(currentFace, lowestZ));
				}
				if (settingsCollection.make22())
				{
					topFaceList.emplace_back(currentFace);
				}
				break;
			}
		}
		// simplefy and store the LoD0.2 faces
		if (settingsCollection.make02() || settingsCollection.make12())
		{
			for (TopoDS_Face& face : helperFunctions::planarFaces2Outline(flatFaceList))
			{
				face.Move(trsf);
				if (settingsCollection.make02())
				{
					CJT::GeoObject roomGeoObject02 = kernel->convertToJSON(face, "0.2");;

					//if (settingsCollection.createSTEP() || settingsCollection.createOBJ()) { copyLoD02GeoList.emplace_back(face); }

					std::map<std::string, std::string> rMap;
					rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTTypeCeilingSurface));
					roomGeoObject02.appendSurfaceData(rMap);
					roomGeoObject02.appendSurfaceTypeValue(0);

					matchingCityRoomObject->addGeoObject(roomGeoObject02);
				}
				if (settingsCollection.make12())
				{
					TopoDS_Solid solidShape12 = extrudeFace(face, false, highestZ);
					if (solidShape12.IsNull()) { continue; }

					//if (settingsCollection.createSTEP() || settingsCollection.createOBJ()) { copyLoD12GeoList.emplace_back(solidShape12); }

					CJT::GeoObject roomGeoObject12 = kernel->convertToJSON(solidShape12, "1.2");
					createSemanticData(&roomGeoObject12, solidShape12, false);
					matchingCityRoomObject->addGeoObject(roomGeoObject12);
				}
			}
		}
		if (settingsCollection.make22()) {
			if (!topFaceList.size()) { continue; }
			std::vector<TopoDS_Shape> roomPrismList = computePrisms(topFaceList, lowestZ);

			if (roomPrismList.size() == 1)
			{
				if (roomPrismList[0].IsNull()) { return; } //TODO: check why this is needed for the gaia model (also at LoD12 creation)
				roomPrismList[0].Move(trsf);

				//if (settingsCollection.createSTEP() || settingsCollection.createOBJ()) { copyLoD12GeoList.emplace_back(roomPrismList[0]); }

				CJT::GeoObject roomGeoObject22 = kernel->convertToJSON(roomPrismList[0], "2.2");;
				createSemanticData(&roomGeoObject22, roomPrismList[0], false);
				matchingCityRoomObject->addGeoObject(roomGeoObject22);
			}
		}
	}

	if (settingsCollection.createSTEP())
	{
		//helperFunctions::writeToSTEP(copyLoD02GeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD02Interior));
		//helperFunctions::writeToSTEP(copyLoD12GeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD12Interior));
		//helperFunctions::writeToSTEP(copyLoD22GeoList, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD22Interior));
	}

	return;
}

CJT::GeoObject CJGeoCreator::makeLoD10(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD10) << std::endl;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = settingsCollection.gridRotation();
	TopoDS_Shape bbox = helperFunctions::createBBOXOCCT(lll, urr, 0, rotationAngle);

	CJT::GeoObject geoObject = kernel->convertToJSON(bbox, "1.0");
	createSemanticData(&geoObject, bbox);

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObject;
}

std::vector<std::vector<TopoDS_Face>> CJGeoCreator::makeRoofFaces(DataManager* h, CJT::Kernel* kernel, int unitScale, bool useFlatFaces, bool footprintBased)
{
	if (!hasTopFaces_)
	{
		initializeBasic(h);
	}
	if (buildingSurfaceDataList_.size() == 0)
	{
		return {};
	}

	std::vector<std::vector<TopoDS_Face>> nestedFaceList;
	for (const BuildingSurfaceCollection& buildingSurfaceData: buildingSurfaceDataList_)
	{
		std::vector<RCollection> faceCluster = buildingSurfaceData.getRoof();
		TopoDS_Face footprintFace = buildingSurfaceData.getFootPrint();

		std::vector<TopoDS_Face> faceCollection;
		for (RCollection surfaceGroup : faceCluster)
		{
			if (useFlatFaces)
			{
				faceCollection.emplace_back(surfaceGroup.getFlatFace());
			}
			else
			{
				for (const TopoDS_Face currentFace : surfaceGroup.getFaces())
				{
					faceCollection.emplace_back(currentFace);
				}
			}
		}

		if (footprintBased && !footprintFace.IsNull())
		{
			std::vector<TopoDS_Face> faceList = getSplitTopFaces(faceCollection, h->getLllPoint().Z(), footprintFace);
			nestedFaceList.emplace_back(faceList);
		}
		else
		{
			std::vector<TopoDS_Face> faceList = getSplitTopFaces(faceCollection, h->getLllPoint().Z());
			nestedFaceList.emplace_back(faceList);
		}
	}
	return nestedFaceList;
}

std::vector< CJT::GeoObject> CJGeoCreator::makeLoD03(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD03) << std::endl;

	if (LoD03RoofFaces_.size() == 0) {  LoD03RoofFaces_ = makeRoofFaces(h, kernel, 1, true); }

	std::vector< CJT::GeoObject> geoObjectCollection;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	for (const std::vector<TopoDS_Face>& faceCluster : LoD03RoofFaces_)
	{
		for (TopoDS_Face currentShape : faceCluster)
		{
			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			currentShape.Move(localRotationTrsf);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "0.3");
			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (settingsCollection.createOBJ())
	{
		std::vector<TopoDS_Face> flattenedRoofFace;
		for (const auto& roofList : LoD03RoofFaces_)
		{
			for (const auto& roofSurf : roofList)
			{
				flattenedRoofFace.emplace_back(roofSurf);
			}
		}

		helperFunctions::writeToOBJ(flattenedRoofFace, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD03));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(LoD03RoofFaces_, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD03));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectCollection;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoD04(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD04) << std::endl;

	if (LoD04RoofFaces_.size() == 0) { LoD04RoofFaces_ = makeRoofFaces(h, kernel, 1, false); }

	std::vector< CJT::GeoObject> geoObjectCollection;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	for (std::vector<TopoDS_Face> faceCluster : LoD04RoofFaces_)
	{
		for (TopoDS_Face currentShape : faceCluster)
		{
			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			currentShape.Move(localRotationTrsf);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "0.4");
			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (settingsCollection.createOBJ())
	{
		std::vector<TopoDS_Face> flattenedRoofFace;
		for (const auto& roofList : LoD04RoofFaces_)
		{
			for (const auto& roofSurf : roofList)
			{
				flattenedRoofFace.emplace_back(roofSurf);
			}
		}

		helperFunctions::writeToOBJ(flattenedRoofFace, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD04));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(LoD04RoofFaces_, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD04));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectCollection;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD12(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD12) << std::endl;
	if (!hasTopFaces_ || !hasGeoBase_) { { return std::vector< CJT::GeoObject>(); } }
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	std::vector<TopoDS_Face> geometryBase;
	if (!settingsCollection.footPrintBased())
	{
		geometryBase = getFootRoofOutlineList();
		if (geometryBase.size() == 0) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoD1.2");
			return {};
		}
	}
	else
	{
		geometryBase = getFootPrintList();
		if (geometryBase.size() == 0) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoFootprint, "LoD1.2");
			return {}; 
		}
	}
	
	double height = h->getUrrPoint().Z() - h->getLllPoint().Z();
	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	trsf.SetTranslationPart(gp_Vec(0, 0, h->getLllPoint().Z()));

	std::vector< CJT::GeoObject> geoObjectList;
	std::vector<TopoDS_Shape> shapeCopyCollection;
	for (size_t i = 0; i < geometryBase.size(); i++)
	{
		TopoDS_Face currentFootprint = geometryBase[i];
		currentFootprint.Move(trsf);

		BRepPrimAPI_MakePrism sweeper(currentFootprint, gp_Vec(0, 0, height), Standard_True);
		sweeper.Build();
		TopoDS_Shape extrudedShape = sweeper.Shape();
		shapeCopyCollection.emplace_back(extrudedShape);

		CJT::GeoObject geoObject = kernel->convertToJSON(extrudedShape, "1.2");		
		createSemanticData(&geoObject, extrudedShape);
		geoObjectList.emplace_back(geoObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD12));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD12));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD13(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD13) << std::endl;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	if (!settingsCollection.footPrintBased())
	{
		if (!hasRoofOutlines()) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoD1.3");
			return {};
		}
	}
	else
	{
		if (!hasFootprints()) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoFootprint, "LoD1.3");
			return {};
		}
	}

	std::vector< CJT::GeoObject> geoObjectList;
	// get the correct top surface
	std::vector<std::vector<TopoDS_Face>> roofList;
	if (LoD03RoofFaces_.size() == 0) {
		roofList = makeRoofFaces(h, kernel, 1, true, settingsCollection.footPrintBased());
	}
	else if (!settingsCollection.footPrintBased())
	{
		roofList = LoD03RoofFaces_;
	}
	else {
		for (size_t i = 0; i < LoD03RoofFaces_.size(); i++)
		{
			std::vector<TopoDS_Face> splittedFaceList = trimFacesToFootprint(LoD03RoofFaces_[i], buildingSurfaceDataList_[i].getFootPrint());
			roofList.emplace_back(splittedFaceList);
		}
	}

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	std::vector<TopoDS_Shape> prismList;
	for (std::vector<TopoDS_Face> faceCluster : roofList)
	{
		for (TopoDS_Shape prism : computePrisms(faceCluster, h->getLllPoint().Z(), false))
		{
			prism.Move(localRotationTrsf);
			prismList.emplace_back(prism);
		}
	}

	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "1.3");
		createSemanticData(&geoObject, currentShape);
		geoObjectList.emplace_back(geoObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(prismList, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD13));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(prismList, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD13));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD22(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD22) << std::endl;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::vector< CJT::GeoObject> geoObjectList;

	if (!SettingsCollection::getInstance().footPrintBased())
	{
		if (!hasRoofOutlines()) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoD2.2");
			return {};
		}
	}
	else
	{
		if (!hasFootprints()) {
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			ErrorCollection::getInstance().addError(ErrorID::warningNoFootprint, "LoD2.2");
			return {};
		}
	}

	std::vector<std::vector<TopoDS_Face>> roofList;
	if (LoD04RoofFaces_.size() == 0) {
		roofList = makeRoofFaces(h, kernel, 1, false, settingsCollection.footPrintBased());
	}
	else if (!settingsCollection.footPrintBased())
	{
		roofList = LoD04RoofFaces_;
	}
	else {
		for (size_t i = 0; i < LoD04RoofFaces_.size(); i++)
		{
			std::vector<TopoDS_Face> splittedFaceList = trimFacesToFootprint(LoD04RoofFaces_[i], buildingSurfaceDataList_[i].getFootPrint());
			roofList.emplace_back(splittedFaceList);
		}
	}

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	std::vector<TopoDS_Shape> prismList;
	for (std::vector<TopoDS_Face> faceCluster : roofList)
	{
		for (TopoDS_Shape prism : computePrisms(faceCluster, h->getLllPoint().Z(), false))
		{
			prism.Move(trsf);
			prismList.emplace_back(prism);
		}
	}


	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "2.2");
		
		createSemanticData(&geoObject, currentShape);
		geoObjectList.emplace_back(geoObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(prismList, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD22));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(prismList, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD22));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoD30(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD30) << std::endl;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::vector< CJT::GeoObject> geoObjectList;

	if (!hasRoofOutlines()) {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoD3.0");
		return {};
	}
	if (!hasFootprints()) {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		ErrorCollection::getInstance().addError(ErrorID::warningNoFootprint, "LoD3.0");
		return {};
	}

	std::vector<std::vector<TopoDS_Face>> roofList;
	std::vector<std::vector<TopoDS_Face>> innerRoofList;
	std::vector<std::vector<TopoDS_Face>> overhangList;
	if (LoD04RoofFaces_.size() == 0) {
		roofList = makeRoofFaces(h, kernel, 1, false);
	}
	else
	{
		roofList = LoD04RoofFaces_;
	}

	for (size_t i = 0; i < roofList.size(); i++)
	{
		std::vector<TopoDS_Face> tempInnerRoofList;
		std::vector<TopoDS_Face> tempOverhangList;
		splitFacesToFootprint(tempInnerRoofList, tempOverhangList, roofList[i], buildingSurfaceDataList_[i].getFootPrint());
		innerRoofList.emplace_back(tempInnerRoofList);
		overhangList.emplace_back(tempOverhangList);
	}

	std::vector<TopoDS_Shape> prismList;
	for (std::vector<TopoDS_Face> faceCluster : innerRoofList)
	{
		for (TopoDS_Shape prism : computePrisms(faceCluster, h->getLllPoint().Z(), false))
		{
			prismList.emplace_back(prism);
			break; //TODO: check this
		}
	}

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	std::vector<TopoDS_Shape> shapeCopyCollection;
	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		currentShape.Move(trsf);
		shapeCopyCollection.emplace_back(currentShape);
		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "3.0");

		createSemanticData(&geoObject, currentShape);
		geoObjectList.emplace_back(geoObject);

		// create the roof overhang
		std::vector<TopoDS_Face> overhangRoof = overhangList[i];

		BRep_Builder builder;
		TopoDS_Compound compound;
		builder.MakeCompound(compound);
		for (auto face : overhangRoof)
		{
			face.Move(trsf);
			builder.Add(compound, face);
		}
		shapeCopyCollection.emplace_back(compound);

		CJT::GeoObject geoOverhangObject = kernel->convertToJSON(compound, "3.0");
		geoOverhangObject.appendSurfaceData(semanticRoofData);
		for (size_t i = 0; i < overhangRoof.size(); i++)
		{
			geoOverhangObject.appendSurfaceTypeValue(0);
		}
		geoObjectList.emplace_back(geoOverhangObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD30));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD30));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoD31(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD31) << std::endl;

	std::vector< CJT::GeoObject> geoObjectList;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precision = settingsCollection.precision();

	if (LoD02Plates_.empty())
	{
		//TODO: fetch this data from somewhere
		return{};
	}

	// get all the height data
	std::vector<double> heightList;
	heightList.reserve(LoD02Plates_.size());
	for (const auto& [height, storeys] : LoD02Plates_)
	{
		heightList.emplace_back(height);
	}

	std::vector<TopoDS_Face> outerShapeFaces;
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>> horizontalFaceIndex;
	double topHeight = h->getUrrPoint().Z();
	for (size_t i = 0; i < heightList.size(); i++)
	{
		double nextHeight = topHeight;
		double currentHeight = heightList[i];
		if (i + 1 != heightList.size())
		{
			nextHeight = heightList[i + 1];
		}
		if (nextHeight < currentHeight)
		{
			continue;
		}

		std::vector<TopoDS_Face> currentStoreyFaceList = LoD02Plates_[currentHeight];
		std::vector<TopoDS_Face> nextStoreyFaceList = LoD02Plates_[nextHeight];

		for (const TopoDS_Face& currentStoryFace : currentStoreyFaceList)
		{
			// get the surface that is compliant with the current storey face and the face of the storey above it
			if (helperFunctions::computeArea(currentStoryFace) < 1e-4) { continue; }

			TopTools_ListOfShape toolList;
			for (const TopoDS_Face& otherStoryFace : nextStoreyFaceList)
			{
				toolList.Append(helperFunctions::projectFaceFlat(otherStoryFace, currentHeight));
			}

			TopTools_ListOfShape argumentList;
			argumentList.Append(currentStoryFace);

			BRepAlgoAPI_Splitter splitter;
			splitter.SetFuzzyValue(1e-4);
			splitter.SetArguments(argumentList);
			splitter.SetTools(toolList);
			splitter.Build();

			std::vector<TopoDS_Face> toBeExtrudedFaces;
			for (TopExp_Explorer explorer(splitter.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
			{
				const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());

				std::optional<gp_Pnt> optionalPoint = helperFunctions::getPointOnFace(currentFace);
				if (optionalPoint == std::nullopt) { continue; }

				for (const TopoDS_Shape& otherStoryFace : toolList)
				{
					if (!helperFunctions::pointOnShape(otherStoryFace, *optionalPoint))
					{
						continue;
					}
					toBeExtrudedFaces.emplace_back(currentFace);
					break;
				}

			}

			if (toBeExtrudedFaces.empty())
			{
				toBeExtrudedFaces = { currentStoryFace };
			}

			// extrude the surface and split to filter further
			for (const TopoDS_Face currentFace : toBeExtrudedFaces)
			{
				TopoDS_Solid currentSolid = extrudeFace(currentFace, false, nextHeight);
				if (currentSolid.IsNull())
				{
					//TODO: add error
					continue;
				}

				for (TopExp_Explorer explorer(currentSolid, TopAbs_FACE); explorer.More(); explorer.Next())
				{
					const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
					if (abs(helperFunctions::computeFaceNormal(currentFace).Z()) < settingsCollection.precisionCoarse())
					{
						outerShapeFaces.emplace_back(currentFace);
						continue;
					}
					horizontalFaceIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), currentFace));
				}
			}
		}
	}

	for (const auto& [boundingBox, horizontalFace] : horizontalFaceIndex)
	{
		TopTools_ListOfShape fuseFaces;
		fuseFaces.Append(horizontalFace);

		std::vector<std::pair<BoostBox3D, TopoDS_Face>> qResult;
		qResult.clear();
		horizontalFaceIndex.query(bgi::intersects(
			boundingBox), std::back_inserter(qResult));

		for (const auto& [otherBoundingBox, otherHorizontalFace] : qResult)
		{
			if (horizontalFace.IsEqual(otherHorizontalFace)) { continue; }
			fuseFaces.Append(otherHorizontalFace);
		}

		BRepAlgoAPI_Fuse fuser;
		fuser.SetArguments(fuseFaces);
		fuser.SetTools(fuseFaces);
		fuser.SetFuzzyValue(1e-4);
		fuser.Build();

		for (TopExp_Explorer explorer(fuser.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
		{
			const TopoDS_Face& currentSplitFace = TopoDS::Face(explorer.Current());
			std::optional<gp_Pnt> currentCenterPoint = helperFunctions::getPointOnFace(currentSplitFace);

			if (currentCenterPoint == std::nullopt) { continue; }

			bool found = false;
			for (const auto& [otherBoundingBox, otherHorizontalFace] : qResult)
			{
				if (horizontalFace.IsEqual(otherHorizontalFace)) { continue; }

				if (helperFunctions::pointOnShape(otherHorizontalFace, *currentCenterPoint))
				{
					found = true;
					break;
				}
			}

			if (!found)
			{
				outerShapeFaces.emplace_back(currentSplitFace);
			}
		}
	}

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	BRepBuilderAPI_Sewing brepSewer;
	for (const TopoDS_Face face : outerShapeFaces) {
		brepSewer.Add(face.Moved(trsf));
	}
	brepSewer.Perform();
	TopoDS_Shape simplefiedShape = simplefySolid(brepSewer.SewedShape());


	CJT::GeoObject geoObject = kernel->convertToJSON(simplefiedShape, "3.1");
	createSemanticData(&geoObject, simplefiedShape);
	geoObjectList.emplace_back(geoObject);

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(simplefiedShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD31));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(simplefiedShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD31));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject>CJGeoCreator::makeLoD32(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD32) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	int maxCastAttempts = 100; // set the maximal amout of cast attempts before the surface is considered interior
	bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>> voxelIndex;

	// collect and index the voxels to which rays are cast
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = voxelGrid_->getIntersectingVoxels();
	std::vector<std::shared_ptr<voxel>> externalVoxel = voxelGrid_->getExternalVoxels();
	intersectingVoxels.insert(intersectingVoxels.end(), externalVoxel.begin(), externalVoxel.end());
	populateVoxelIndex(&voxelIndex, intersectingVoxels);

	// collect and index the products which are presumed to be part of the exterior
	std::vector<Value> productLookupValues = getUniqueProductValues(intersectingVoxels);
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> faceIndx;
	if (productLookupValues.size() <= 0)
	{
		throw ErrorID::failedLoD32;
		return{};
	}

	double searchBuffer = settingsCollection.searchBufferLod32();

	std::vector<int> scoreList;
	std::vector<Value> cleanedProductLookupValues;
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(productLookupValues[i].second);
		std::string lookupType = lookup->getProductPtr()->data().type()->name();
		TopoDS_Shape currentShape;
		if (lookupType == "IfcDoor" || lookupType == "IfcWindow")
		{
			if (lookup->hasSimpleShape()) { currentShape = lookup->getSimpleShape(); }
			else { continue; }
		}
		else { currentShape = lookup->getProductShape(); }

		BoostBox3D totalBox = helperFunctions::createBBox(currentShape, searchBuffer);
		int score = static_cast<int>(std::distance(voxelIndex.qbegin(bgi::intersects(totalBox)), voxelIndex.qend()));
		if (score == 0) { continue; }

		scoreList.emplace_back(score);
		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
			BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
			faceIndx.insert(std::make_pair(currentBox, currentFace));
		}
		cleanedProductLookupValues.emplace_back(productLookupValues[i]);
	}
	std::vector<Value>().swap(productLookupValues);
	// evaluate which surfaces are visible from the exterior
	std::vector<std::pair<TopoDS_Face, std::string>> outerSurfacePairList;
	getOuterRaySurfaces(outerSurfacePairList, cleanedProductLookupValues, scoreList, h, faceIndx, voxelIndex);
	std::vector<int>().swap(scoreList);
	// clip surfaces that are in contact with eachother
	std::vector<std::pair<TopoDS_Face, std::string>> splitOuterSurfacePairList;
	std::vector<std::pair<TopoDS_Face, std::string>> unSplitOuterSurfacePairList;
	splitOuterSurfaces(splitOuterSurfacePairList, unSplitOuterSurfacePairList, outerSurfacePairList);
	std::vector<std::pair<TopoDS_Face, std::string>>().swap(outerSurfacePairList);
	// remove internal faces
	bgi::rtree<std::pair<BoostBox3D, int>, bgi::rstar<25>> splitFaceIndx;
	std::vector<std::pair<TopoDS_Face, std::string>> totalSplitOuterSurfacePairList;
	for (const auto& [currentFace, currentType] : splitOuterSurfacePairList)
	{
		totalSplitOuterSurfacePairList.emplace_back(std::pair(currentFace, currentType));
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		splitFaceIndx.insert(std::make_pair(currentBox, splitFaceIndx.size()));
	}
	std::vector<std::pair<TopoDS_Face, std::string>> finalOuterSurfacePairList;
	for (const auto& [currentFace, currentType] : unSplitOuterSurfacePairList)
	{
		totalSplitOuterSurfacePairList.emplace_back(std::pair(currentFace, currentType));
		finalOuterSurfacePairList.emplace_back(std::pair(currentFace, currentType));
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		splitFaceIndx.insert(std::make_pair(currentBox, splitFaceIndx.size()));
	}

	for (const auto& [currentFace, currentType] : splitOuterSurfacePairList)
	{
		std::optional<gp_Pnt> optionalCurrentPoint = helperFunctions::getPointOnFace(currentFace);
		if (optionalCurrentPoint == std::nullopt) { continue; }
		gp_Pnt currentPoint = *optionalCurrentPoint;
		bg::model::box<BoostPoint3D> pointQuerybox(
			{ currentPoint.X() - searchBuffer, currentPoint.Y() - searchBuffer, currentPoint.Z() - searchBuffer },
			{ currentPoint.X() + searchBuffer, currentPoint.Y() + searchBuffer, currentPoint.Z() + searchBuffer }
		);

		std::vector<std::pair<BoostBox3D, std::shared_ptr<voxel>>> pointQResult;
		voxelIndex.query(bgi::intersects(pointQuerybox), std::back_inserter(pointQResult));

		if (pointQResult.empty()) { continue; }

		bool isExterior = true;
		for (const auto& [voxelBbox, voxel] : pointQResult)
		{
			bool clearLine = true;
			gp_Pnt voxelCore = voxel->getOCCTCenterPoint();

			bg::model::box<BoostPoint3D> productQuerybox(helperFunctions::createBBox(currentPoint, voxelCore, settingsCollection.precision()));
			std::vector<std::pair<BoostBox3D, int>>faceQResult;
			splitFaceIndx.query(bgi::intersects(productQuerybox), std::back_inserter(faceQResult));

			for (const std::pair<BoostBox3D, int>& facePair : faceQResult)
			{
				// get the potential faces
				const TopoDS_Face& otherFace = totalSplitOuterSurfacePairList[facePair.second].first;
				if (currentFace.IsEqual(otherFace)) { continue; }

				//test for linear intersections
				TopLoc_Location loc;
				auto mesh = BRep_Tool::Triangulation(otherFace, loc);
				if (mesh.IsNull()) {
					clearLine = false;
					continue;
				}

				for (int j = 1; j <= mesh.get()->NbTriangles(); j++)
				{
					const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

					std::vector<gp_Pnt> trianglePoints{
						mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
						mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
						mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
					};

					if (helperFunctions::triangleIntersecting({ currentPoint, voxelCore }, trianglePoints))
					{
						clearLine = false;
						break;
					}
				}
				if (!clearLine) { break; }
			}
			if (clearLine)
			{
				finalOuterSurfacePairList.emplace_back(std::pair(currentFace, currentType));
				break;
			}
		}
	}

	// remove dub and incapsulated surfaces by merging them
	std::vector<gp_Dir> normalList;
	normalList.reserve(finalOuterSurfacePairList.size());
	for (const auto& [currentFace, currentType] : finalOuterSurfacePairList)
	{
		normalList.emplace_back(helperFunctions::computeFaceNormal(currentFace));
	}
	std::vector<std::pair<TopoDS_Face, std::string>> cleanedOuterSurfacePairList = simplefyFacePool(finalOuterSurfacePairList, normalList);
	std::vector<std::pair<TopoDS_Face, std::string>>().swap(finalOuterSurfacePairList);

	// make the collection compund shape
	BRep_Builder builder;
	TopoDS_Compound collectionShape;
	builder.MakeCompound(collectionShape);
	std::vector<int> typeValueList;
	for (const std::pair<TopoDS_Face, std::string>& currentFacePair : cleanedOuterSurfacePairList)
	{
		const std::string& lookupType = currentFacePair.second;
		const TopoDS_Face& currentFace = currentFacePair.first;
		if (lookupType == "IfcRoof")
		{
			typeValueList.emplace_back(2);
		}
		else if (lookupType == "IfcWindow")
		{
			typeValueList.emplace_back(3);
		}
		else if (lookupType == "IfcDoor")
		{
			typeValueList.emplace_back(4);
		}
		else if (lookupType == "IfcSlab")
		{
			std::optional<gp_Pnt> pointOnface = helperFunctions::getPointOnFace(currentFacePair.first);
			gp_Vec vecOfFace = helperFunctions::computeFaceNormal(currentFace);

			if (pointOnface == std::nullopt)
			{
				typeValueList.emplace_back(1);
				continue;
			}

			if (pointOnface->Z() < settingsCollection.footprintElevation() && abs(vecOfFace.Z()) > 0.1)
			{
				//TODO: do a raycast straight downwards

				typeValueList.emplace_back(0);
			}
			else if (pointOnface->Z() < settingsCollection.footprintElevation())
			{
				typeValueList.emplace_back(1);
			}
			else if (abs(helperFunctions::computeFaceNormal(currentFace).Z()) > 0.1)
			{
				//TODO: do a raycast straight upwards

				typeValueList.emplace_back(2);
			}
			else
			{
				typeValueList.emplace_back(1);
			}
		}
		else
		{
			typeValueList.emplace_back(1);
		}

		builder.Add(collectionShape, currentFace);
	}

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	collectionShape.Move(localRotationTrsf);

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	CJT::GeoObject geoObject = kernel->convertToJSON(collectionShape, "3.2");

	// create semantic data map
	std::map<std::string, std::string> grMap;
	grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
	std::map<std::string, std::string> wMap;
	wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
	std::map<std::string, std::string> rMap;
	rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	std::map<std::string, std::string> windowMap;
	windowMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWindow));
	std::map<std::string, std::string> dMap;
	dMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeDoor));
	geoObject.appendSurfaceData(grMap);
	geoObject.appendSurfaceData(wMap);
	geoObject.appendSurfaceData(rMap);
	geoObject.appendSurfaceData(windowMap);
	geoObject.appendSurfaceData(dMap);
	geoObject.setSurfaceTypeValues(typeValueList);

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(collectionShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD32));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(collectionShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD32));
	}

	geoObjectList.emplace_back(geoObject);
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


void CJGeoCreator::makeComplexLoDRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale) {
	IfcSchema::IfcSpace::list::ptr spaceList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcSpace>();
	
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	for (auto spaceIt = spaceList->begin(); spaceIt != spaceList->end(); ++spaceIt)
	{
		//TODO: make function?
		// find the matching cityspace object
		IfcSchema::IfcSpace* spaceIfcObject = *spaceIt;
		std::string spaceGuid = spaceIfcObject->GlobalId();

		bool spaceFound = false;
		std::shared_ptr<CJT::CityObject> matchingCityRoomObject;
		for (std::shared_ptr<CJT::CityObject> roomCityObject : roomCityObjects)
		{
			if (spaceGuid == roomCityObject->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)])
			{
				spaceFound = true;
				matchingCityRoomObject = roomCityObject;
				break;
			}
		}
		if (!spaceFound) { continue; }

		// get height values
		TopoDS_Shape spaceShape = h->getObjectShape(spaceIfcObject, false);
		spaceShape.Move(localRotationTrsf);
		CJT::GeoObject roomGeoObject = kernel->convertToJSON(spaceShape, "3.2");;
		matchingCityRoomObject->addGeoObject(roomGeoObject);
	}
	return;
}

std::vector< CJT::GeoObject>CJGeoCreator::makeV(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD50) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	voxelGrid_->computeSurfaceSemantics(h);
	std::vector<int> typeValueList;
	TopoDS_Shape sewedShape = voxels2Shape(0, &typeValueList); //TODO: make work with multiple buildings in a single model

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	sewedShape.Move(localRotationTrsf);

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	CJT::GeoObject geoObject;
	if (sewedShape.ShapeType() == TopAbs_COMPOUND)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningNoSolid, "LoD5.0");
		std::cout << errorWarningStringEnum::getString(ErrorID::warningNoSolid) << std::endl;
		geoObject = kernel->convertToJSON(sewedShape, "5.0");
		
		if (SettingsCollection::getInstance().createSTEP())
		{
			helperFunctions::writeToOBJ(sewedShape, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD50));
		}

		if (SettingsCollection::getInstance().createSTEP())
		{
			helperFunctions::writeToSTEP(sewedShape, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD50));
		}

	}
	else
	{
		BRep_Builder brepBuilder;
		TopoDS_Shell shell;
		brepBuilder.MakeShell(shell);
		TopoDS_Solid voxelSolid;
		brepBuilder.MakeSolid(voxelSolid);
		brepBuilder.Add(voxelSolid, sewedShape);

		if (SettingsCollection::getInstance().createSTEP())
		{
			helperFunctions::writeToOBJ(voxelSolid, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD50));
		}

		if (SettingsCollection::getInstance().createSTEP())
		{
			helperFunctions::writeToSTEP(voxelSolid, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD50));
		}

		geoObject = kernel->convertToJSON(voxelSolid, "5.0", true);
	}

	std::map<std::string, std::string> nMap;
	nMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeNone));
	std::map<std::string, std::string> wMap;
	wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWindow));
	std::map<std::string, std::string> dMap;
	dMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeDoor));
	std::map<std::string, std::string> rMap;
	rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	std::map<std::string, std::string> wallMap;
	wallMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
	std::map<std::string, std::string> ceilMap;
	ceilMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTTypeOuterCeilingSurface));
	std::map<std::string, std::string> groundMap;
	groundMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
	geoObject.appendSurfaceData(nMap);
	geoObject.appendSurfaceData(wMap);
	geoObject.appendSurfaceData(dMap);
	geoObject.appendSurfaceData(rMap);
	geoObject.appendSurfaceData(wallMap);
	geoObject.appendSurfaceData(ceilMap);
	geoObject.appendSurfaceData(groundMap);
	geoObject.setSurfaceTypeValues(typeValueList);

	geoObjectList.emplace_back(geoObject);

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}

void CJGeoCreator::makeVRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD50Rooms) << std::endl;

	auto startTime = std::chrono::steady_clock::now();
	// bool indicating if the voxelroom data has to be created 
	bool genData = false;
	if (!h->getSpaceIndexPointer()->size())
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcNoRoomObjects);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoRoomObjects) << std::endl;
		genData = true;
	}

	for (int i = 1; i < voxelGrid_->getRoomSize(); i++) //TODO: multithread (-6 for the surface creation)
	{
		TopoDS_Shape sewedShape = voxels2Shape(i); //TODO: remove windows from this

		gp_Trsf localRotationTrsf;
		localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
		sewedShape.Move(localRotationTrsf);

		std::vector< std::shared_ptr<CJT::CityObject>> potentialRoomCityObjectList;
		if (genData) { potentialRoomCityObjectList = fetchRoomObject(h, roomCityObjects, i); }

		if (sewedShape.ShapeType() == TopAbs_COMPOUND)
		{
			std::cout << errorWarningStringEnum::getString(ErrorID::warningNoSolid) << std::endl;
			CJT::GeoObject geoObject = kernel->convertToJSON(sewedShape, "5.0");

			if (!potentialRoomCityObjectList.size() || genData) 
			{ 
				double lowestZ = helperFunctions::getLowestZ(sewedShape);
				std::shared_ptr<CJT::CityObject> cjRoomObject = createDefaultRoomObject(storeyCityObjects, i, lowestZ);
				if (!cjRoomObject->getParents().size()) { continue; }
				cjRoomObject->addGeoObject(geoObject);
				roomCityObjects.emplace_back(cjRoomObject);
				continue; 
			}
			for (size_t j = 0; j < potentialRoomCityObjectList.size(); j++)
			{
				potentialRoomCityObjectList[j]->addGeoObject(geoObject);
			}	
			continue;
		}

		BRep_Builder brepBuilder;
		TopoDS_Shell shell;
		brepBuilder.MakeShell(shell);
		TopoDS_Solid voxelSolid;
		brepBuilder.MakeSolid(voxelSolid);
		brepBuilder.Add(voxelSolid, sewedShape);

		CJT::GeoObject geoObject = kernel->convertToJSON(voxelSolid, "5.0");

		if (!potentialRoomCityObjectList.size() || genData)
		{
			double lowestZ = helperFunctions::getLowestZ(sewedShape);
			std::shared_ptr<CJT::CityObject> cjRoomObject = createDefaultRoomObject(storeyCityObjects, i, lowestZ);

			if (!cjRoomObject->getParents().size()) { continue; }
			cjRoomObject->addGeoObject(geoObject);
			roomCityObjects.emplace_back(cjRoomObject);
			continue;
		}
		for (size_t j = 0; j < potentialRoomCityObjectList.size(); j++)
		{
			potentialRoomCityObjectList[j]->addGeoObject(geoObject);
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
}

std::vector<CJT::CityObject> CJGeoCreator::makeSite(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoExtractingSite) << std::endl;
	std::vector<CJT::CityObject> siteObjectList;
	double buffer = 0.001;
	double lowPrecision = SettingsCollection::getInstance().precisionCoarse();
	int geoCount = 0;

	TopTools_ListOfShape completeFuseToolList;

	std::vector<TopoDS_Face> verticalFaces;
	std::vector<TopoDS_Face> groundPlaneFaces;

	// get the surfaces from the geo or site objects
	for (int i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcSite::list::ptr siteElements = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcSite>();

		if (!siteElements->size()) { continue; }
		geoCount += siteElements->size();

		if (geoCount > 1)
		{
			ErrorCollection::getInstance().addError(ErrorID::warningIfcDubSites);
			std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcDubSites) << std::endl;
			return std::vector<CJT::CityObject>();
		}

		IfcSchema::IfcSite* siteElement = *siteElements->begin();
		
		if (!siteElement->Representation()) { continue; }
		TopoDS_Shape siteShape = h->getObjectShape(siteElement);
		for (TopExp_Explorer explorer(siteShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			const TopoDS_Face& siteFace = TopoDS::Face(explorer.Current());
			completeFuseToolList.Append(siteFace);
		}
	}
	if (!groundPlaneFaces.size())
	{
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		for (int i = 0; i < h->getSourceFileCount(); i++)
		{
			IfcSchema::IfcGeographicElement::list::ptr geographicElements = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcGeographicElement>();

			if (!geographicElements->size()) { continue; }
			geoCount += geographicElements->size();
			for (auto it = geographicElements->begin(); it != geographicElements->end(); ++it)
			{
				IfcSchema::IfcGeographicElement* geographicElement = *it;
				if (geographicElement->PredefinedType() != IfcSchema::IfcGeographicElementTypeEnum::Value::IfcGeographicElementType_TERRAIN) { continue; }

				TopoDS_Shape geographicShape = h->getObjectShape(geographicElement);
				for (TopExp_Explorer explorer(geographicShape, TopAbs_FACE); explorer.More(); explorer.Next())
				{
					const TopoDS_Face& geoFace = TopoDS::Face(explorer.Current());
					completeFuseToolList.Append(geoFace);
				}
			}
		}
#endif // USE_IFC4
	}
	
	if (completeFuseToolList.Size() == 0)
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSites);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoSites) << std::endl;
		return std::vector<CJT::CityObject>();
	}

	// fuse all surfaces so they are all properly split
	BRepAlgoAPI_Fuse fuser;
	fuser.SetArguments(completeFuseToolList);
	fuser.SetTools(completeFuseToolList);
	fuser.SetFuzzyValue(lowPrecision);
	fuser.Build();

	// split flat faces from vertical faces
	for (TopExp_Explorer explorer(fuser.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& geoFace = TopoDS::Face(explorer.Current());
		if (abs(helperFunctions::computeFaceNormal(geoFace).Z()) < 0.001)
		{
			verticalFaces.emplace_back(geoFace);
			continue;
		}
		groundPlaneFaces.emplace_back(geoFace);
	}

	// make index 
	bgi::rtree<Value, bgi::rstar<treeDepth_>> siteFacesSpatialIndex;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> siteSelectionFacesSpatialIndex;
	std::vector<TopoDS_Face> siteSelectionFaceList;
	for (size_t i = 0; i < groundPlaneFaces.size(); i++)
	{
		TopoDS_Face currentFace = groundPlaneFaces[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		siteFacesSpatialIndex.insert(std::make_pair(bbox, (int)i));
	}

	// find if flat surface is covered
	TopTools_ListOfShape toolList;
	for (size_t i = 0; i < groundPlaneFaces.size(); i++)
	{
		TopoDS_Face currentFace = groundPlaneFaces[i];
		std::optional<gp_Pnt> optionalBasePoint = helperFunctions::getPointOnFace(currentFace);

		if (optionalBasePoint == std::nullopt) { continue; }
		gp_Pnt basePoint = *optionalBasePoint;
		gp_Pnt endPoint = gp_Pnt(
			basePoint.X(),
			basePoint.Y(),
			basePoint.Z() + 10000
		);

		TopoDS_Edge ray = BRepBuilderAPI_MakeEdge(basePoint, endPoint);

		std::vector<Value> qResult;
		siteFacesSpatialIndex.query(
			bgi::intersects(
				bg::model::box <BoostPoint3D>(
					BoostPoint3D(basePoint.X() - buffer, basePoint.Y() - buffer, basePoint.Z() - buffer),
					BoostPoint3D(endPoint.X() + buffer, endPoint.Y() + buffer, endPoint.Z() + buffer)
					)
			),
			std::back_inserter(qResult)
		);

		bool isCovered = false;
		for (size_t j = 0; j < qResult.size(); j++)
		{
			int currentIndx = qResult[j].second;
			if (currentIndx == i) { continue; }

			BRepExtrema_DistShapeShape distanceWireCalc(groundPlaneFaces[currentIndx], ray);
			distanceWireCalc.Perform();

			if (distanceWireCalc.Value() < 0.00001)
			{
				isCovered = true;
				break;
			}
		}

		if (!isCovered)
		{
			toolList.Append(currentFace);
			siteSelectionFaceList.emplace_back(currentFace);
			bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
			siteSelectionFacesSpatialIndex.insert(std::make_pair(bbox, static_cast<int>(siteSelectionFacesSpatialIndex.size())));
		}

	}

	if (!toolList.Size())
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcSiteReconstructionFailed);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcSiteReconstructionFailed) << std::endl;
		return std::vector<CJT::CityObject>();
	}

	// get the outer edges from the flat surfaces
	fuser.SetArguments(toolList);
	fuser.SetTools(toolList);
	fuser.SetFuzzyValue(lowPrecision);
	fuser.Build();

	std::vector<TopoDS_Edge> outerEdgesList;
	Prs3d_ShapeTool Tool(fuser.Shape());
	for (Tool.InitCurve(); Tool.MoreCurve(); Tool.NextCurve())
	{
		const TopoDS_Edge& E = Tool.GetCurve();
		if (Tool.FacesOfEdge().get()->Size() == 1) {
			outerEdgesList.emplace_back(E);
		}
	}

	if (!verticalFaces.size())
	{
		return std::vector<CJT::CityObject>();
	}

	// find the bounding vertical faces
	for (size_t i = 0; i < verticalFaces.size(); i++)
	{
		TopoDS_Face currentFace = verticalFaces[i];
		std::optional<gp_Pnt> optionalFacePoint = helperFunctions::getPointOnFace(currentFace);

		if (optionalFacePoint == std::nullopt) { continue; }

		gp_Pnt facePoint = *optionalFacePoint;
		gp_Vec faceNormal = helperFunctions::computeFaceNormal(currentFace) / 100;

		gp_Pnt p1 = facePoint.Translated(faceNormal);
		gp_Pnt p2 = facePoint.Translated(faceNormal.Reversed());

		TopoDS_Edge ray1 = BRepBuilderAPI_MakeEdge(p1, gp_Pnt(p1.X(), p1.Y(), p1.Z() + 1000));
		TopoDS_Edge ray2 = BRepBuilderAPI_MakeEdge(p2, gp_Pnt(p2.X(), p2.Y(), p2.Z() + 1000));

		std::vector<Value> qResult;
		siteSelectionFacesSpatialIndex.query(
			bgi::intersects(
				helperFunctions::createBBox({ ray1, ray2 })
			),
			std::back_inserter(qResult)
		);

		int intersectionCount = 0;
		for (size_t j = 0; j < qResult.size(); j++)
		{
			TopoDS_Face horizontalFace = siteSelectionFaceList[qResult[j].second];

			BRepExtrema_DistShapeShape distanceWireCalc1(horizontalFace, ray1);
			BRepExtrema_DistShapeShape distanceWireCalc2(horizontalFace, ray2);
			distanceWireCalc1.Perform();
			distanceWireCalc2.Perform();

			if (distanceWireCalc1.Value() < 0.00001) { intersectionCount++; }
			if (distanceWireCalc2.Value() < 0.00001) { intersectionCount++; }

			if (intersectionCount >=2)
			{
				break;
			}
		}

		if (intersectionCount < 2)
		{
			toolList.Append(currentFace);
		}
	}

	// merge the filtered vertical and horizontal site faces
	fuser.SetArguments(toolList);
	fuser.SetTools(toolList);
	fuser.SetFuzzyValue(lowPrecision);
	fuser.Build();

	if (!fuser.IsDone())
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcSiteReconstructionFailed);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcSiteReconstructionFailed) << std::endl;
		return std::vector<CJT::CityObject>();
	}

	CJT::CityObject siteObject;
	CJT::GeoObject geoObject = kernel->convertToJSON(fuser.Shape(), "1");
	siteObject.addGeoObject(geoObject);
	siteObject.setType(CJT::Building_Type::TINRelief);
	siteObject.setName(CJObjectEnum::getString(CJObjectID::CJTypeSiteObject));

	siteObjectList.emplace_back(siteObject);
	return siteObjectList;
}

TopoDS_Shape CJGeoCreator::voxels2Shape(int roomNum, std::vector<int>* typeList)
{
	std::vector<std::thread> threads;
	std::vector<std::pair<TopoDS_Face, CJObjectID>> threadFaceLists;
	std::mutex faceListMutex;

	for (int i = 0; i < 6; i++) {
		threads.emplace_back([this, &threadFaceLists, &faceListMutex, i, roomNum]() {processDirectionalFaces(i, roomNum, faceListMutex, std::ref(threadFaceLists)); });
	}
	for (auto& thread : threads) { thread.join(); }

	BRepBuilderAPI_Sewing brepSewer;
	for (const auto& [face, surfaceType] : threadFaceLists) {
		brepSewer.Add(face);
	}
	brepSewer.Perform();
	TopoDS_Shape sewedShape = brepSewer.SewedShape();
	if (typeList == nullptr) { return sewedShape; }

	//TODO: make this a function 
	bgi::rtree<Value, bgi::rstar<treeDepth_>> typedSurfaceIndx;
	for (const auto& [face, surfaceType] : threadFaceLists) {
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(face);
		typedSurfaceIndx.insert(std::make_pair(bbox, typedSurfaceIndx.size()));
	}

	for (TopExp_Explorer explorer(sewedShape, TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
		GProp_GProps currentGprops;
		BRepGProp::SurfaceProperties(currentFace, currentGprops);

		bg::model::box <BoostPoint3D> qbox = helperFunctions::createBBox(currentFace);

		std::vector<Value> qResult;
		typedSurfaceIndx.query(
			bgi::intersects(qbox),
			std::back_inserter(qResult)
		);

		bool found = false;
		for (const auto& value : qResult)
		{
			const auto& typeFacePair = threadFaceLists[value.second];
			const TopoDS_Face& typedFace = typeFacePair.first;
			const CJObjectID& surfaceType = typeFacePair.second;

			GProp_GProps typedGprops;
			BRepGProp::SurfaceProperties(typedFace, typedGprops);

			if (!currentGprops.CentreOfMass().IsEqual(typedGprops.CentreOfMass(), 1e-6)) { continue; }
			if (abs(currentGprops.Mass() - typedGprops.Mass()) > 1e-4) { continue; }
			if (surfaceType == CJObjectID::CJTypeWindow)
			{
				typeList->emplace_back(1);
			}
			else if (surfaceType == CJObjectID::CJTypeDoor)
			{
				typeList->emplace_back(2);
			}
			else if (surfaceType == CJObjectID::CJTypeRoofSurface)
			{
				typeList->emplace_back(3);
			}
			else if (surfaceType == CJObjectID::CJTypeWallSurface)
			{
				typeList->emplace_back(4);
			}
			else if (surfaceType == CJObjectID::CJTTypeOuterCeilingSurface)
			{
				typeList->emplace_back(5);
			}
			else if (surfaceType == CJObjectID::CJTypeGroundSurface)
			{
				typeList->emplace_back(6);
			}
			else
			{
				typeList->emplace_back(0);
			}
			found = true;
			break;
		}		
		if (!found)
		{
			typeList->emplace_back(0);
		}
	}	
	//TODO: until here 
	return brepSewer.SewedShape();
}


void CJGeoCreator::processDirectionalFaces(int direction, int roomNum, std::mutex& faceListMutex, std::vector<std::pair<TopoDS_Face, CJObjectID>>& collectionList)
{ 
	std::vector<std::pair<std::vector<TopoDS_Edge>, CJObjectID>> edgeTypeList = voxelGrid_->getDirectionalFaces(direction, -SettingsCollection::getInstance().gridRotation(), roomNum);
	for (const auto& [currentedgeCollection, surfaceType] : edgeTypeList)
	{
		std::vector<TopoDS_Wire> wireList = helperFunctions::growWires(currentedgeCollection);
		std::vector<TopoDS_Wire> cleanWireList = helperFunctions::cleanWires(wireList);
		TopoDS_Face cleanFace = helperFunctions::wireCluster2Faces(cleanWireList);
		std::unique_lock<std::mutex> listLock(faceListMutex);
		collectionList.emplace_back(std::make_pair(cleanFace, surfaceType));
		listLock.unlock();
	}
	return;
}

void CJGeoCreator::getOuterRaySurfaces(std::vector<std::pair<TopoDS_Face, std::string>>& outerSurfacePairList, const std::vector<Value>& totalValueObjectList, const std::vector<int>& scoreList, DataManager* h, const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIdx, const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>& voxelIndex)
{
	// split the range over cores
	int coreUse = SettingsCollection::getInstance().threadcount();
	if (coreUse > totalValueObjectList.size())
	{
		while (coreUse > totalValueObjectList.size()) { coreUse /= 2; }
	}
	coreUse -= 1;
	double targetScore = std::accumulate(scoreList.begin(), scoreList.end(), 0) / coreUse;

	std::vector<std::thread> threadList;
	std::mutex listMutex;

	int beginIndx = 0;
	int processedObjects = 0;
	for (int i = 0; i < coreUse; i++)
	{
		if (beginIndx >= scoreList.size())
		{
			break;
		}

		size_t endList = scoreList.size() - 1;
		double currentScore = 0;
		if (i != coreUse - 1)
		{
			for (size_t j = beginIndx; j < scoreList.size(); j++)
			{
				currentScore += scoreList[j];
				if (targetScore <= currentScore)
				{
					endList = j;
					break;
				}
			}
		}

		auto startIdx = totalValueObjectList.begin() + beginIndx;
		auto endIdx = totalValueObjectList.begin() + endList + 1;
		beginIndx = endList + 1;

		std::vector<Value> sublist(startIdx, endIdx);
		threadList.emplace_back([this, &outerSurfacePairList, sublist = std::move(sublist), &processedObjects, &listMutex, &h, &faceIdx, &voxelIndex]() {
			getOuterRaySurfaces(outerSurfacePairList, sublist, processedObjects, listMutex, h, faceIdx, voxelIndex);
		});
	}
	threadList.emplace_back([&] {monitorRayCasting(totalValueObjectList.size(), processedObjects, listMutex);  });

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	return;
}

void CJGeoCreator::getOuterRaySurfaces(
	std::vector<std::pair<TopoDS_Face, std::string>>& outerSurfacePairList,
	const std::vector<Value>& valueObjectList, 
	int& processedObject,
	std::mutex& listmutex,
	DataManager* h,
	const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIdx,
	const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>&voxelIndex
)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double searchBuffer = settingsCollection.searchBufferLod32();
	for (const Value& currentValue : valueObjectList)
	{
		processedObject++;
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(currentValue.second);
		std::string lookupType = lookup->getProductPtr()->data().type()->name();
		TopoDS_Shape currentShape;
		if (lookupType == "IfcDoor" || lookupType == "IfcWindow")
		{
			if (lookup->hasSimpleShape()) { currentShape = lookup->getSimpleShape(); }
			else { continue; }
		}
		else { currentShape = lookup->getProductShape(); }
		
		if (lookupType == "IfcPlate") 
		{
			IfcSchema::IfcProduct* plateProduct = lookup->getProductPtr();
			if (helperFunctions::hasGlassMaterial(plateProduct))
			{
				lookupType = "IfcWindow";
			}
		}

		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			bool faceIsExterior = false;
			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
			if (helperFunctions::getPointCount(currentFace) < 3) { continue; }

			//Create a grid over the surface and the offsetted wire
			std::vector<gp_Pnt> surfaceGridList = helperFunctions::getPointGridOnSurface(currentFace);
			std::vector<gp_Pnt> wireGridList;// = helperFunctions::getPointGridOnWire(currentFace);
			surfaceGridList.insert(surfaceGridList.end(), wireGridList.begin(), wireGridList.end());

			// cast a line from the grid to surrounding voxels
			for (const gp_Pnt& gridPoint : surfaceGridList)
			{
				
				bg::model::box<BoostPoint3D> pointQuerybox(
					{ gridPoint.X() - searchBuffer, gridPoint.Y() - searchBuffer, gridPoint.Z() - searchBuffer },
					{ gridPoint.X() + searchBuffer, gridPoint.Y() + searchBuffer, gridPoint.Z() + searchBuffer }
				);

				std::vector<std::pair<BoostBox3D, std::shared_ptr<voxel>>> pointQResult;
				voxelIndex.query(bgi::intersects(pointQuerybox), std::back_inserter(pointQResult));
				//check if ray castline cleared
				for (const auto& [voxelBBox, targetVoxel] : pointQResult)
				{
					bool clearLine = true;
					bg::model::box<BoostPoint3D> productQuerybox(helperFunctions::createBBox(gridPoint, targetVoxel->getOCCTCenterPoint(), settingsCollection.precision()));
					std::vector<std::pair<BoostBox3D, TopoDS_Face>>faceQResult;
					faceIdx.query(bgi::intersects(productQuerybox), std::back_inserter(faceQResult));

					for (const std::pair<BoostBox3D, TopoDS_Face>& facePair : faceQResult)
					{
						// get the potential faces
						const TopoDS_Face& otherFace = facePair.second;
						if (currentFace.IsEqual(otherFace)) { continue; }

						//test for linear intersections
						TopLoc_Location loc;
						auto mesh = BRep_Tool::Triangulation(otherFace, loc);

						if (mesh.IsNull()) {
							//TODO: add error
							continue;
						}

						for (int j = 1; j <= mesh.get()->NbTriangles(); j++)
						{
							const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

							std::vector<gp_Pnt> trianglePoints{
								mesh->Nodes().Value(theTriangle(1)).Transformed(loc),
								mesh->Nodes().Value(theTriangle(2)).Transformed(loc),
								mesh->Nodes().Value(theTriangle(3)).Transformed(loc)
							};

							if (helperFunctions::triangleIntersecting({ gridPoint, targetVoxel->getOCCTCenterPoint() }, trianglePoints))
							{
								clearLine = false;
								break;
							}
						}
						if (!clearLine) { break; }
					}
					if (clearLine)
					{
						faceIsExterior = true;
						break;
					}
				}
				if (faceIsExterior)
				{
					break;
				}
			}
			if (!faceIsExterior) { continue; }

			//store the plane representing the surface
			std::unique_lock<std::mutex> listLock(listmutex);
			outerSurfacePairList.emplace_back(std::make_pair(currentFace, lookupType));
			listLock.unlock();
		}
	}
	return;
}


void CJGeoCreator::monitorRayCasting(
	int totalObjects,
	int& processedObject,
	std::mutex& listmutex
)
{
	bool running = true;
	while (running)
	{
		std::unique_lock<std::mutex> listlock(listmutex);
		int currentObjectCount = processedObject;
		listlock.unlock();

		std::cout
			<< "\tTotal objects: " << totalObjects
			<< "; Processed objects: " << currentObjectCount << "      \r";

		if (currentObjectCount == totalObjects)
		{
			break;
		}

		if (running)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	std::cout << "\n";
	return;
}

void CJGeoCreator::extractOuterVoxelSummary(CJT::CityObject* shellObject, DataManager* h, double footprintHeight, double geoRot)
{
	voxelGrid_->computeSurfaceSemantics(h);

	std::map<std::string, double> summaryMap;

	std::vector<std::shared_ptr<voxel>> internalVoxels = voxelGrid_->getInternalVoxels();

	double voxelSize = SettingsCollection::getInstance().voxelSize();
	double voxelVolume = voxelSize * voxelSize * voxelSize;
	double shellVolume = internalVoxels.size()* voxelVolume;

	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxShellVolume), shellVolume);

	double lowerEvalHeight = footprintHeight - (0.5 * voxelSize);
	double higherEvalHeight = footprintHeight + (0.5 * voxelSize);
	double basementVolume = 0;

	double shellArea = 0;
	double basementArea = 0;
	double footprintArea = 0;
	double voxelArea = voxelSize * voxelSize;

	double windowArea = 0;

	for (size_t i = 0; i < internalVoxels.size(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = internalVoxels[i];
		bool isOuterShell = currentVoxel->getIsShell();
		double zHeight = currentVoxel->getCenterPoint().get<2>();
		
		if (isOuterShell) { shellArea += currentVoxel->numberOfFaces() * voxelArea; }

		if (lowerEvalHeight >= zHeight)
		{
			// for sure building basement
			basementVolume += voxelVolume;

			if (!isOuterShell) { continue; }
			for (int j = 0; j < 6; j++)
			{
				if (currentVoxel->hasFace(j)) { basementArea += voxelArea; }
			}
			continue;
		}

		if (lowerEvalHeight < zHeight && zHeight < higherEvalHeight)
		{
			// partial building basement
			basementVolume += voxelSize * voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize);

			footprintArea += voxelArea;

			if (!isOuterShell) { continue; }
			if (currentVoxel->hasFace(0)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(1)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(2)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(3)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(5)) { 
				basementArea += voxelArea;
				
			}
		}

		if (!isOuterShell) { continue; }

		for (int i = 0; i < 6; i++)
		{
			if (currentVoxel->faceType(i) != CJObjectID::CJTypeWindow) { continue; }
			windowArea += voxelArea;
		}

		
		windowArea += voxelArea;
	}

	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxBasementShellVolume), basementVolume);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxBuildingShellVolume), shellVolume - basementVolume);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxShellArea), shellArea);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxBasementShellArea), basementArea + footprintArea);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxBuildingShellArea), shellArea - basementArea + footprintArea);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxFootprintArea), footprintArea);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxFaceadeOpeningArea), windowArea);
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::EnvVoxelSize), voxelSize);

	gp_Pnt anchor = voxelGrid_->getAnchor();
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::EnvVoxelAnchor), (anchor.X(), anchor.Y(), anchor.Z() ) );
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::EnvVoxelRotation), voxelGrid_->getRotation() + geoRot);
}

void CJGeoCreator::extractInnerVoxelSummary(CJT::CityObject* shellObject, DataManager* h)
{
	double totalRoomVolume = 0;

	double voxelSize =  SettingsCollection::getInstance().voxelSize();
	double voxelVolume = voxelSize * voxelSize * voxelSize;

	std::vector<std::shared_ptr<voxel>> voxelList = voxelGrid_->getVoxels();

	for (auto i = voxelList.begin(); i != voxelList.end(); i++)
	{
		std::shared_ptr<voxel> currentVoxel = *i;
		if (currentVoxel->getRoomNum() > 0)
		{
			totalRoomVolume += voxelVolume;
		}
	}

	//TODO: add room area?
	shellObject->addAttribute(CJObjectEnum::getString(CJObjectID::voxelApproxRoomVolumeTotal), totalRoomVolume);

}

std::vector < std::shared_ptr<CJT::CityObject >> CJGeoCreator::fetchRoomObject(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int roomNum)
{
	gp_Pnt roomPoint = helperFunctions::rotatePointWorld(voxelGrid_->getPointInRoom(roomNum), 0); //TODO: update this to find all the room objects?
	std::vector<Value> qResult;
	qResult.clear();
	h->getSpaceIndexPointer()->query(
		bgi::intersects(
			bg::model::box < BoostPoint3D >(
				BoostPoint3D(roomPoint.X() - 0.01, roomPoint.Y() - 0.01, roomPoint.Z() - 0.01),
				BoostPoint3D(roomPoint.X() + 0.01, roomPoint.Y() + 0.01, roomPoint.Z() + 0.01)
				)
		),
		std::back_inserter(qResult)
	);

	std::vector<std::shared_ptr<CJT::CityObject>> roomObjects;
	for (size_t k = 0; k < qResult.size(); k++)
	{
		// find the room that point is located in

		bool encapsulating = true;
		std::shared_ptr<IfcProductSpatialData> lookup = h->getSpaceLookup(qResult[k].second);
		IfcSchema::IfcProduct* product = lookup->getProductPtr();

		TopoDS_Shape productShape = h->getObjectShape(product, true);
		BRepClass3d_SolidClassifier solidClassifier;
		solidClassifier.Load(productShape);
		solidClassifier.Perform(roomPoint, 0.1);

		if (!solidClassifier.State() == TopAbs_State::TopAbs_OUT) { continue; }

		std::string guid = product->GlobalId();

		for (std::shared_ptr<CJT::CityObject > roomCityObject : roomCityObjects)
		{
			if (guid == roomCityObject->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)])
			{
				roomObjects.emplace_back(roomCityObject);
				break;
			}
		}
	}
	return roomObjects;
}


void CJGeoCreator::populateVoxelIndex(
	bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>* voxelIndex,
	const std::vector<std::shared_ptr<voxel>> exteriorVoxels
)
{
	for (auto voxelIt = exteriorVoxels.begin(); voxelIt != exteriorVoxels.end(); ++ voxelIt)
	{
		std::shared_ptr<voxel> currentBoxel = *voxelIt;
		std::vector<Value> internalProducts = currentBoxel->getInternalProductList();

		// voxels that have no internal products do not have an intersection and are stored as completely external voxels
		if (internalProducts.size() == 0 && currentBoxel->getRoomNum() == 0)
		{
			auto cornerPoints = currentBoxel->getCornerPoints();
			gp_Pnt lllPoint = cornerPoints[0];
			gp_Pnt urrPoint = cornerPoints[4];

			BoostPoint3D boostlllPoint = BoostPoint3D(lllPoint.X(), lllPoint.Y(), lllPoint.Z());
			BoostPoint3D boosturrPoint = BoostPoint3D(urrPoint.X(), urrPoint.Y(), urrPoint.Z());

			bg::model::box <BoostPoint3D> box = bg::model::box < BoostPoint3D >(boostlllPoint, boosturrPoint);

			voxelIndex->insert(std::make_pair(box, currentBoxel));
		}
	}
	return;
}





std::vector<Value> CJGeoCreator::makeUniqueValueList(const std::vector<Value>& valueList)
{
	// make unique productLookupValues
	std::vector<Value> valueSet;

	for (unsigned i = 0; i < valueList.size(); ++i) {
		bool dub = false;
		for (size_t j = 0; j < valueSet.size(); j++)
		{
			if (valueList[i].second == valueSet[j].second)
			{
				dub = true;
				break;
			}
		}
		if (dub) { continue; }
		valueSet.emplace_back(valueList[i]);
	}	
	return valueSet;
}

std::vector<Value> CJGeoCreator::getUniqueProductValues(std::vector<std::shared_ptr<voxel>> voxelList)
{
	std::vector<Value> productLookupValues;
	for (std::shared_ptr<voxel> voxel : voxelList)
	{
		for (const Value& productValue : voxel->getInternalProductList())
		{
			productLookupValues.emplace_back(productValue);
		}
	}
	return makeUniqueValueList(productLookupValues);
}


std::shared_ptr<CJT::CityObject> CJGeoCreator::createDefaultRoomObject(std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int roomNum, double lowestZ)
{
	std::shared_ptr<CJT::CityObject> cjRoomObject = std::make_unique<CJT::CityObject>();
	cjRoomObject->setName("generic " + std::to_string(roomNum));
	cjRoomObject->setType(CJT::Building_Type::BuildingRoom);

	double smallestZDistance = 999999;
	int storeyIndx = -1;
	for (int j = 0; j < storeyCityObjects.size(); j++)
	{
		nlohmann::json jsonElev = storeyCityObjects[j]->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcElevation)];
		double zDistance = abs(static_cast<double>(jsonElev) - lowestZ);

		if (smallestZDistance > zDistance)
		{
			storeyIndx = j;
			smallestZDistance = zDistance;
		}
	}

	if (storeyIndx == -1)
	{
		return cjRoomObject;
	}

	cjRoomObject->addParent(storeyCityObjects[storeyIndx]);

	return cjRoomObject;
}

void CJGeoCreator::createSemanticData(CJT::GeoObject* geoObject, const TopoDS_Shape& geometryShape, bool isExterior)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	double lowestShapeZ = helperFunctions::getLowestZ(geometryShape);

	std::vector<int> functionList; // detect what surface is what based on the normals and face height
	for (TopExp_Explorer faceExp(geometryShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		TopoDS_Face face = TopoDS::Face(faceExp.Current());

		gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);
		if (abs(faceNormal.Z()) > settingsCollection.precision())
		{
			if (helperFunctions::getHighestZ(face) - lowestShapeZ < settingsCollection.precision()) // is floor
			{
				functionList.emplace_back(0);
			}
			else // is roof 
			{
				functionList.emplace_back(2);
			}
		}
		else // is wall
		{
			functionList.emplace_back(1);
		}
	}

	populateSurfaceData(geoObject, functionList, isExterior);
	return;
}

void CJGeoCreator::splitOuterSurfaces(
	std::vector<std::pair<TopoDS_Face, std::string>>& splittedFacesOut, 
	std::vector<std::pair<TopoDS_Face, std::string>>& untouchedFacesOut, 
	const std::vector<std::pair<TopoDS_Face, std::string>>& outerSurfacePairList
)
{

	bgi::rtree<std::pair<BoostBox3D, int>, bgi::rstar<25>> faceIndx;
	for (const auto& [currentFace, currentType] : outerSurfacePairList)
	{
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		faceIndx.insert(std::pair(currentBox, faceIndx.size()));
	}

	for (const auto& [currentFace, currentType] : outerSurfacePairList)
	{
		BOPAlgo_Splitter divider;
		divider.SetFuzzyValue(SettingsCollection::getInstance().precisionCoarse());
		divider.SetRunParallel(Standard_False);
		divider.AddArgument(currentFace);

		std::vector<std::pair<BoostBox3D, int>> qResult;
		qResult.clear();
		faceIndx.query(bgi::intersects(helperFunctions::createBBox(currentFace)), std::back_inserter(qResult));

		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);
		if (currentNormal.Magnitude() < 1e-6) { continue; }

		int toolCount = 0;
		for (const auto& [otherBox, otherIndx] : qResult)
		{
			const TopoDS_Face& otherFace = outerSurfacePairList[otherIndx].first;
			if (currentFace.IsEqual(otherFace)) { continue; }

			gp_Vec otherNormal = helperFunctions::computeFaceNormal(otherFace);
			if (otherNormal.Magnitude() < 1e-6) { continue; }

			if (currentNormal.IsParallel(otherNormal, 1e-4)) {
				std::optional<gp_Pnt> otherPointOpt = helperFunctions::getPointOnFace(otherFace);
				if (otherPointOpt == std::nullopt) { continue; }

				BRepExtrema_DistShapeShape distanceCalc(currentFace, BRepBuilderAPI_MakeVertex(*otherPointOpt));
				if (distanceCalc.Value() > 1e-4) { continue; }
			}
			else
			{
				BRepExtrema_DistShapeShape distanceCalc(currentFace, otherFace);
				distanceCalc.Perform();
				if (distanceCalc.Value() > 1e-4) { continue; }
			}			
			divider.AddTool(otherFace);
			toolCount++;
		}

		if (toolCount == 0)
		{
			helperFunctions::triangulateShape(currentFace);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentType));
			continue;
		}

		divider.Perform();
		TopoDS_Shape splitFaceList = divider.Shape();
		if (splitFaceList.IsNull())
		{
			helperFunctions::triangulateShape(currentFace);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentType));
			continue;
		}

		std::vector<TopoDS_Face> faceList;
		for (TopExp_Explorer faceExp(splitFaceList, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
			TopoDS_Face face = TopoDS::Face(faceExp.Current());
			faceList.emplace_back(face); 
		}

		if (faceList.size() <= 1)
		{
			helperFunctions::triangulateShape(currentFace);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentType));
		}
		else
		{
			for (const TopoDS_Face face : faceList)
			{
				helperFunctions::triangulateShape(face);
				splittedFacesOut.emplace_back(std::pair(face, currentType));
			}
		}

	}
	return;
}

void CJGeoCreator::populateSurfaceData(CJT::GeoObject* geoObject, const std::vector<int>& SurfaceIndxDataList, bool isExterior)
{
	std::map<std::string, std::string> grMap;
	std::map<std::string, std::string> wMap;
	std::map<std::string, std::string> rMap;
	if (isExterior)
	{
		grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
		wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
		rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	}
	else
	{
		grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeFloorSurface));
		wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeInteriorWallSurface));
		rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTTypeCeilingSurface));
	}

	if (SettingsCollection::getInstance().mergeSemantics())
	{
		geoObject->appendSurfaceData(grMap);
		geoObject->appendSurfaceData(wMap);
		geoObject->appendSurfaceData(rMap);

		for (int functionIdx : SurfaceIndxDataList)
		{
			geoObject->appendSurfaceTypeValue(functionIdx);
		}
	}
	else
	{
		for (int j = 0; j < SurfaceIndxDataList.size(); j++)
		{
			int functionIdx = SurfaceIndxDataList[j];

			if (functionIdx == 0)
			{
				geoObject->appendSurfaceData(grMap);
				geoObject->appendSurfaceTypeValue(j);
			}
			else if (functionIdx == 1)
			{
				geoObject->appendSurfaceData(wMap);
				geoObject->appendSurfaceTypeValue(j);
			}
			else
			{
				geoObject->appendSurfaceData(rMap);
				geoObject->appendSurfaceTypeValue(j);
			}
		}
	}
	return;
}

std::vector<TopoDS_Face> CJGeoCreator::trimFacesToFootprint(const std::vector<TopoDS_Face>& roofFaces, const TopoDS_Face& footprintFace)
{
	std::vector<TopoDS_Face> outRoofFaces;
	std::vector<TopoDS_Face> outOverhangFaces;
	splitFacesToFootprint(outRoofFaces, outOverhangFaces, roofFaces, footprintFace);
	return outRoofFaces;
}

void CJGeoCreator::splitFacesToFootprint(std::vector<TopoDS_Face>& outRoofFaces, std::vector<TopoDS_Face>& outOverhangFaces, const std::vector<TopoDS_Face>& roofFaces, const TopoDS_Face& footprintFace)
{

	TopoDS_Solid extrudedFootprint = extrudeFace(footprintFace, false, 10000);

	BOPAlgo_Splitter divider;
	divider.SetFuzzyValue(SettingsCollection::getInstance().precision());
	divider.SetRunParallel(Standard_False);
	divider.AddTool(extrudedFootprint);

	for (const TopoDS_Face& untrimmedRoofFace : roofFaces)
	{
		divider.AddArgument(untrimmedRoofFace);
	}
	divider.Perform();


	for (TopExp_Explorer expl(divider.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face subFace = TopoDS::Face(expl.Current());
		std::optional<gp_Pnt> optionalBasePoint = helperFunctions::getPointOnFace(subFace);
		if (optionalBasePoint == std::nullopt) { continue; }

		gp_Pnt basePoint = *optionalBasePoint;
		gp_Pnt bottomPoint = gp_Pnt(basePoint.X(), basePoint.Y(), basePoint.Z() - 100000);

		// test if falls within buffersurface
		TopoDS_Edge lowerEvalLine = BRepBuilderAPI_MakeEdge(basePoint, bottomPoint);

		BRepExtrema_DistShapeShape distanceWireCalc(lowerEvalLine, footprintFace);
		if (distanceWireCalc.Value() > 1e-6) 
		{
			outOverhangFaces.emplace_back(subFace);
			continue; 
		}
		outRoofFaces.emplace_back(subFace);
	}
	return;
}

std::vector<IfcSchema::IfcBuildingStorey*> CJGeoCreator::fetchStoreyObjects(DataManager* h, const std::vector<std::string>& storeyGuidList)
{
	std::vector< IfcSchema::IfcBuildingStorey*> ifcStoreyList;
	for (const std::string& storeyGuid : storeyGuidList)
	{
		for (int i = 0; i < h->getSourceFileCount(); i++)
		{
			IfcUtil::IfcBaseClass* ifcBaseStorey = nullptr;
			try { ifcBaseStorey = h->getSourceFile(i)->instance_by_guid(storeyGuid); }
			catch (const std::exception&) { continue; }
			if (ifcBaseStorey == nullptr) { continue; }
			IfcSchema::IfcBuildingStorey* ifcSubStorey = ifcBaseStorey->as<IfcSchema::IfcBuildingStorey>();
			ifcStoreyList.emplace_back(ifcSubStorey);
		}
	}
	return ifcStoreyList;
}

CJGeoCreator::CJGeoCreator(DataManager* h, double vSize)
{
	// compute generic voxelfield data
	voxelGrid_ = std::make_shared<VoxelGrid>(h);
}
