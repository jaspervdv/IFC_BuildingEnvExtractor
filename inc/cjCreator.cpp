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
#include <ShapeFix_Shell.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeAnalysis_Shell.hxx>

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

double roundDoubleToPrecision(double value, double precision) {
	return std::round(value / precision) * precision;
}

void CJGeoCreator::garbageCollection()
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// check if LoD03 roofs can be released
	bool LoD03RoofsRequired = false;
	if (!finishedLoD13_ && settingsCollection.make13())
	{
		LoD03RoofsRequired = true;
	}

	if (!LoD03RoofsRequired && !LoD03RoofFaces_.empty())
	{
		std::vector< std::vector<TopoDS_Face>>().swap(LoD03RoofFaces_);
	}

	// check if LoD04 roofs can be released
	bool LoD04RoofsRequired = false;
	if (!finishedLoD22_ && settingsCollection.make22() ||
		!finishedLoDb0_ && settingsCollection.makeb0() ||
		!finishedLoDc2_ && settingsCollection.makec2() ||
		!finishedLoDd2_ && settingsCollection.maked2() )
	{
		LoD04RoofsRequired = true;
	}

	if (!LoD04RoofsRequired && !LoD04RoofFaces_.empty())
	{
		std::vector< std::vector<TopoDS_Face>>().swap(LoD04RoofFaces_);
	}

	// check if LoD02 plates can be released
	bool LoD02PLatesRequired = false;
	if (!finishedLoDc1_ && settingsCollection.makec1() ||
		!finishedLoDc2_ && settingsCollection.makec2())
	{
		LoD02PLatesRequired = true;
	}

	if (!LoD02PLatesRequired && !LoD02Plates_.empty())
	{
		std::map<double, std::vector<TopoDS_Face>>().swap(LoD02Plates_);
	}

	// check if LoD03 plates and exterior faces can be released
	bool LoD03PlatesRequired = false;
	if (!finishedLoDd1_ && settingsCollection.maked1() ||
		!finishedLoDd2_ && settingsCollection.maked2())
	{
		LoD03PlatesRequired = true;
	}

	if (!LoD02PLatesRequired && !LoD03Plates_.empty())
	{
		std::map<double, std::vector<TopoDS_Face>>().swap(LoD03Plates_);
		std::map<double, std::vector<TopoDS_Face>>().swap(LoD03ExtriorHFaces_);
	}

	// check if e.1 surfaces can be released
	bool LoDe1FacesRequired = false;
	if (!finishedLoD32_ && settingsCollection.make32())
	{
		LoDe1FacesRequired = true;
	}

	if (!LoDe1FacesRequired && !LoDE1Faces_.empty()) //TODO: release
	{
		std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>().swap(LoDE1Faces_);
	}

	// check if storey objects can be released
	bool requireStoreyObjects = false;
	if (!finishedLoDc1_ && settingsCollection.makec1() ||
		!finishedLoDc2_ && settingsCollection.makec2() ||
		!finishedLoDd1_ && settingsCollection.maked1() ||
		!finishedLoDd2_ && settingsCollection.maked2() ||
		!finishedLoDe0_ && settingsCollection.makee0())
	{
		requireStoreyObjects = true;
	}

	if (!requireStoreyObjects && !storeyObjects_.empty())
	{
		std::vector<std::shared_ptr<CJT::CityObject>>().swap(storeyObjects_);
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
	double precision = SettingsCollection::getInstance().precision();
	//index
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;
	std::vector<TopoDS_Face> faceList;

	for (size_t i = 0; i < Collection.size(); i++)
	{
		const TopoDS_Face& currentFace = Collection[i]->getFace();

		if (helperFunctions::computeArea(currentFace) <= precision)
		{
			continue;
		}
		TopoDS_Face currentCleanFace = helperFunctions::TessellateFace(currentFace); //tODO: should be more central
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentCleanFace, 0.5);
		spatialIndex.insert(std::make_pair(bbox, i));
		faceList.emplace_back(currentCleanFace);
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

				gp_Vec currentNormal = helperFunctions::computeFaceNormal(evalFace);

				for (const Value& qValue : qResult)
				{
					int potentialNeigbhbourIdx = qValue.second;
					if (evalList[potentialNeigbhbourIdx] == 1) { continue; }

					const TopoDS_Face& potentialNeighbourFace = faceList[potentialNeigbhbourIdx];

					gp_Vec otherNormal = helperFunctions::computeFaceNormal(potentialNeighbourFace);

					// check if shared edge
					if (helperFunctions::shareEdge(evalFace, potentialNeighbourFace))
					{
						bufferList.emplace_back(potentialNeighbourFace);
						evalList[potentialNeigbhbourIdx] = 1;
						continue;
					}

					// check if overlapping
					if (!currentNormal.IsParallel(otherNormal, 1e-4))
					{
						continue;
					}

					if (!helperFunctions::coplanarOverlapping(evalFace, potentialNeighbourFace))
					{
						continue;
					}

					bufferList.emplace_back(potentialNeighbourFace);
					evalList[potentialNeigbhbourIdx] = 1;
				}
			}
			if (bufferList.size() == 0) { break; }
			outerSurfaceRingList = bufferList;
			bufferList.clear();
		}

		if (!toBeGroupdSurfaces.size()) { continue; }

		std::vector<TopoDS_Face> mergedSurfaces = helperFunctions::mergeFaces(toBeGroupdSurfaces);
		//DebugUtils::printFaces(mergedSurfaces);
		mergedRSurfaces.emplace_back(RCollection(mergedSurfaces));
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return mergedRSurfaces;
}


void CJGeoCreator::simpleRaySurfaceCast(
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& outList,
	const std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& surfaceList,
	const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>& voxelIndex,
	const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& surfaceIndx
)
{
	double searchBuffer = SettingsCollection::getInstance().searchBufferLod32();
	double precision = SettingsCollection::getInstance().precision();

	int c = 0;
	for (const auto& [currentFace, currentProduct] : surfaceList)
	{
		c++;
		std::cout << "\tIsolating outer surfaces - " << c << " of " << surfaceList.size() << "\r";

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

		int score = 0; // the number of clear lines cast from the surface
		for (const auto& [voxelBbox, voxel] : pointQResult)
		{
			bool clearLine = true;
			gp_Pnt voxelCore = voxel->getOCCTCenterPoint();

			bg::model::box<BoostPoint3D> productQuerybox(helperFunctions::createBBox(currentPoint, voxelCore, precision));
			std::vector<std::pair<BoostBox3D, TopoDS_Face>>faceQResult;
			surfaceIndx.query(bgi::intersects(productQuerybox), std::back_inserter(faceQResult));

			for (const std::pair<BoostBox3D, TopoDS_Face>& facePair : faceQResult)
			{
				// get the potential faces
				const TopoDS_Face& otherFace = facePair.second;
				if (currentFace.IsEqual(otherFace)) { continue; }

				if (helperFunctions::LineShapeIntersection(otherFace, currentPoint, voxelCore))
				{
					clearLine = false;
					break;
				}

				Handle(Geom_Surface) surface = BRep_Tool::Surface(otherFace);
				Handle(Geom_Plane) geomPlane = Handle(Geom_Plane)::DownCast(surface);
				if (geomPlane.IsNull()) { continue; }

				gp_Pln plane = geomPlane->Pln();
				Standard_Real dist1 = plane.Distance(voxelCore);

				if (dist1 <= 1e-4)
				{
					clearLine = false;
					break;
				}
			}
			if (clearLine) {
				score++; 
			}
			if (score > 1) 
			{ 
				outList.emplace_back(std::pair(currentFace, currentProduct));
				break;
			}
		}
	}

	std::cout << "\n";
	return;
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
	std::vector<std::shared_ptr<SurfaceGridPair>> fineFilteredShapeList = FinefilterSurfaces(shapeList);
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
		TopoDS_Shape currentShape = lookup->getProductShape();
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

TopoDS_Solid CJGeoCreator::extrudeFace(const TopoDS_Face& evalFace, bool downwards, double splittingFaceHeight)
{
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer(SettingsCollection::getInstance().precision());
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidShape;
	brepBuilder.MakeSolid(solidShape);
	TopoDS_Face projectedFace = helperFunctions::projectFaceFlat(evalFace, splittingFaceHeight);

	std::vector<TopoDS_Wire> wireList;
	TopoDS_Wire outerWire = BRepTools::OuterWire(evalFace);
	wireList.emplace_back(outerWire);
	for (TopExp_Explorer wireExplorer(evalFace, TopAbs_WIRE); wireExplorer.More(); wireExplorer.Next())
	{
		TopoDS_Wire currentWire = TopoDS::Wire(wireExplorer.Current());
		if (outerWire.IsEqual(currentWire)) { continue; }
		wireList.emplace_back(currentWire);
	}

	std::vector<TopoDS_Wire> wireCopyTop;
	std::vector<TopoDS_Wire> wireCopyBottom;
	wireCopyTop.reserve(wireList.size());
	wireCopyBottom.reserve(wireList.size());

	for (const TopoDS_Wire& currentWire : wireList)
	{
		int edgeCount = 0;
		BRepBuilderAPI_MakeWire wireMakerTop;
		BRepBuilderAPI_MakeWire wireMakerBottom;
		std::vector<TopoDS_Face> TempFaceList;
		for (BRepTools_WireExplorer expl(currentWire); expl.More(); expl.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(expl.Current());
			gp_Pnt p0 = helperFunctions::getFirstPointShape(edge);
			gp_Pnt p1 = helperFunctions::getLastPointShape(edge);
			edgeCount++;

			if (downwards)
			{
				if (p0.Z() - splittingFaceHeight < 1e-6 || p1.Z() - splittingFaceHeight < 1e-6) { return TopoDS_Solid(); }
			}
			else
			{
				if (p0.Z() - splittingFaceHeight > 1e-6 || p1.Z() - splittingFaceHeight > 1e-6) { return TopoDS_Solid(); }
			}

			gp_Pnt p2 = gp_Pnt(p1.X(), p1.Y(), splittingFaceHeight);
			gp_Pnt p3 = gp_Pnt(p0.X(), p0.Y(), splittingFaceHeight);

			TopoDS_Edge topEdge = BRepBuilderAPI_MakeEdge(p0, p1);
			TopoDS_Edge buttomEdge = BRepBuilderAPI_MakeEdge(p2, p3);
			wireMakerTop.Add(topEdge);
			wireMakerBottom.Add(buttomEdge);

			bool p0Flat = false;
			bool p1Flat = false;
			if (abs(p0.Z() - splittingFaceHeight) <= 1e-6)
			{
				p0Flat = true;
			}
			if (abs(p1.Z() - splittingFaceHeight) <= 1e-6)
			{
				p1Flat = true;
			}

			if (p0Flat && p1Flat)
			{
				continue;
			}

			if (p0Flat)
			{
				TopoDS_Face sideFace = helperFunctions::createPlanarFace(p2, p1, p0);
				TempFaceList.emplace_back(sideFace);
			}
			else if (p1Flat)
			{
				TopoDS_Face sideFace = helperFunctions::createPlanarFace(p3, p1, p0);
				TempFaceList.emplace_back(sideFace);
			}
			else
			{
				TopoDS_Face sideFace = helperFunctions::createPlanarFace(p3, p2, p1, p0);
				TempFaceList.emplace_back(sideFace);
			}
		}

		if (edgeCount <= 2)
		{
			continue;
		}

		if (!wireMakerBottom.IsDone() || !wireMakerTop.IsDone())
		{
			return TopoDS_Solid();
		}
		wireCopyTop.emplace_back(wireMakerTop.Wire());
		wireCopyBottom.emplace_back(wireMakerBottom.Wire());

		for (const TopoDS_Face& currentSideFace : TempFaceList)
		{
			brepSewer.Add(currentSideFace);
		}
	}

	gp_Pnt p0 = helperFunctions::getFirstPointShape(evalFace);
	gp_Vec normal = helperFunctions::computeFaceNormal(evalFace);
	Handle(Geom_Plane) plane = new Geom_Plane(p0, normal);

	Handle(Geom_Plane) planeFlat = new Geom_Plane(gp_Pnt(0, 0, splittingFaceHeight), gp_Vec(0, 0, -1));

	BRepBuilderAPI_MakeFace faceMakerTop(plane, wireCopyTop[0], 1e-6);
	BRepBuilderAPI_MakeFace faceMakerBottom(planeFlat, wireCopyBottom[0], 1e-6);

	for (size_t i = 1; i < wireCopyTop.size(); i++)
	{
		faceMakerTop.Add(wireCopyTop[i]);
		faceMakerBottom.Add(wireCopyBottom[i]);
	}

	TopoDS_Face TopFace = faceMakerTop.Face();
	TopoDS_Face bottomFace = faceMakerBottom.Face();

	helperFunctions::fixFace(&TopFace);
	helperFunctions::fixFace(&bottomFace);

	brepSewer.Add(TopFace);
	brepSewer.Add(bottomFace);
	brepSewer.Perform();

	TopoDS_Shape sewedShape = brepSewer.SewedShape();
	if (sewedShape.ShapeType() != TopAbs_SHELL)
	{
		return solidShape;
	}

	ShapeAnalysis_Shell shellChecker;
	shellChecker.LoadShells(sewedShape);
	brepBuilder.Add(solidShape, sewedShape);
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
		makeFloorSection(footprintList, h, floorlvl + storeyBuffer + h->getObjectTranslation().TranslationPart().Z());
		for (TopoDS_Face& footprintItem : footprintList) { footprintItem.Move(translation); }

		for (BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			for (const TopoDS_Face& currentFootprint :  footprintList)
			{
				TopoDS_Face currentCleanFootprint = eleminateInnerVoids(currentFootprint);

				BRepExtrema_DistShapeShape distanceFootRoof(buildingSurfaceData.getRoofOutline(), currentCleanFootprint);
				double verticalDistatance = abs(distanceFootRoof.PointOnShape1(1).Z() - distanceFootRoof.PointOnShape2(1).Z());
				if (abs(verticalDistatance - distanceFootRoof.Value()) > settingsCollection.precision() )
				{
					continue;
				}
				buildingSurfaceData.setFootPrint(currentCleanFootprint);
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
	std::vector<TopoDS_Face> floorSectionList = helperFunctions::planarFaces2Outline(cleanedFaceList, cuttingPlane);
	
	facesOut.reserve(floorSectionList.size());
	for (const TopoDS_Face& currentOutFace : floorSectionList)
	{
		TopoDS_Face cleanedOutFace = helperFunctions::TessellateFace(currentOutFace);
		facesOut.emplace_back(cleanedOutFace);
	}

	return;
}

void CJGeoCreator::makeFloorSectionComplex(
	bool horizontalSection,
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

	std::vector<TopoDS_Face> splitFaceList;
	if (!horizontalSection)
	{
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
		splitFaceList = section2Faces(storeyRelatedShapeList, sectionHeight);
	}
	// get objects that fall within the sotrey height
	else if (horizontalSection)
	{
		std::vector<Value> productLookupValues;
		bg::model::box <BoostPoint3D> searchBox = helperFunctions::createBBox(cuttingPlane, 0.15);
		h->getIndexPointer()->query(bgi::intersects(searchBox), std::back_inserter(productLookupValues));
		splitFaceList = section2Faces(productLookupValues, h, sectionHeight);
	}

	// generate shapes
	std::vector<TopoDS_Face> cleanedFaceList = helperFunctions::removeDubFaces(splitFaceList);
	if (!cleanedFaceList.size())
	{
		//TODO: add error
		return;
	}
	std::vector<TopoDS_Shape> faceCluster = helperFunctions::planarFaces2Cluster(cleanedFaceList); //TODO: list fix?


	std::vector<TopoDS_Face> innerFaces;
	std::vector<TopoDS_Face> outerFaces;
	SplitInAndOuterHFaces(faceCluster[0], innerFaces, outerFaces);

	intFacesOut = helperFunctions::planarFaces2Outline(innerFaces);
	extFacesOut = helperFunctions::planarFaces2Outline(outerFaces);
	return;
}

TopoDS_Face CJGeoCreator::eleminateInnerVoids(const TopoDS_Face& theFace)
{
	const TopoDS_Wire& outerWire = BRepTools::OuterWire(theFace);
	BRepBuilderAPI_MakeFace faceMaker(outerWire);

	for (TopExp_Explorer expl(theFace, TopAbs_WIRE); expl.More(); expl.Next())
	{
		const TopoDS_Wire& innerWire = TopoDS::Wire(expl.Current());

		if (outerWire.IsSame(innerWire)) { continue; }

		BRepBuilderAPI_MakeFace innerFaceMaker(innerWire);

		if (!innerFaceMaker.IsDone()) { continue; }
		TopoDS_Face innerFace = innerFaceMaker.Face();

		std::optional<gp_Pnt> optionalPoint = helperFunctions::getPointOnFace(innerFace);
		if (optionalPoint == std::nullopt) { continue; }
		gp_Pnt facePoint = *optionalPoint;


		facePoint.SetZ(facePoint.Z() + SettingsCollection::getInstance().voxelSize() / 0.66);
		int voxelIndx = voxelGrid_->getCloseByVoxel(facePoint);
		voxel boxel = voxelGrid_->getVoxel(voxelIndx);
		if (!boxel.getIsIntersecting() && !boxel.getIsInside())
		{
			faceMaker.Add(innerWire);
		}
	}
	TopoDS_Face currentCleanFace = faceMaker.Face();

	return currentCleanFace;

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
			TopTools_IndexedMapOfShape aMap;
			TopExp::MapShapes(subFace, TopAbs_EDGE, aMap);

			if (aMap.Size() <= 2) { continue; }
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
			if (!helperFunctions::LineShapeIntersection(bufferSurface, basePoint, bottomPoint))
			{
				continue;
			}
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

			if (helperFunctions::LineShapeIntersection(otherFace, basePoint, topPoint))
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
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	std::vector<TopoDS_Face> splitTopSurfaceList; // horizontal faces that are to be output
	if (!preFilter) { splitTopSurfaceList = inputFaceList; }
	else { splitTopSurfaceList = getSplitTopFaces(inputFaceList, lowestZ, bufferSurface); }

	if (splitTopSurfaceList.size() == 1)
	{
		return { extrudeFace(splitTopSurfaceList[0], true, lowestZ) };
	}

	// extrude the trimmed top surfaces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> splittingfaceIdx; // horizontal and veritcal faces that can be used to split the to be split faces
	std::vector<TopoDS_Face> toBesSplitFaceList; // vertical faces that are to be split

	for (const TopoDS_Face& currentFace : splitTopSurfaceList)
	{
		TopoDS_Solid extrudedShape = extrudeFace(currentFace, true, lowestZ);
		if (extrudedShape.IsNull())
		{
			ErrorCollection::getInstance().addError(ErrorID::warningUnableToExtrude);
			continue;
		}

		for (TopExp_Explorer expl(extrudedShape, TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face extrusionFace = TopoDS::Face(expl.Current());

			// ignore if not vertical face
			gp_Vec currentNormal = helperFunctions::computeFaceNormal(extrusionFace);
			if (abs(currentNormal.Z()) > precisionCoarse) { continue; };

			// find if already found in model 
			BoostBox3D faceBox = helperFunctions::createBBox(extrusionFace);
			splittingfaceIdx.insert(std::make_pair(faceBox, extrusionFace));
			toBesSplitFaceList.emplace_back(extrusionFace);
		}

		BoostBox3D flatFaceBox = helperFunctions::createBBox(currentFace);
		splittingfaceIdx.insert(std::make_pair(flatFaceBox, currentFace));
	}

	// remove dub faces and split them
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx = indexUniqueFaces(splittingfaceIdx);
	std::vector<TopoDS_Face> splitFaceList = getSplitFaces(toBesSplitFaceList, cuttingFaceIdx);

	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> SplitfaceIdx;
	for (const TopoDS_Face& currentFace : splitFaceList)
	{

		gp_Vec currentVec = helperFunctions::computeFaceNormal(currentFace);
		if (currentVec.Magnitude() < precision) { continue; }
		BoostBox3D faceBox = helperFunctions::createBBox(currentFace);
		SplitfaceIdx.insert(std::make_pair(faceBox, currentFace));
	}

	// add the non - dub vertical faces
	BRepBuilderAPI_Sewing brepSewer(precision);
	for (const auto& [currentBox, currentFace] : SplitfaceIdx)
	{
		std::vector<BoxFacePair> qResult;
		qResult.clear();
		SplitfaceIdx.query(bgi::intersects(currentBox), std::back_inserter(qResult));

		bool isDub = false;
		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);
		std::optional<gp_Pnt> optionalCurrentPoint = helperFunctions::getPointOnFace(currentFace);
		gp_Pnt currentPoint = *optionalCurrentPoint;

		for (const auto& [otherBox, otherFace] : qResult)
		{
			if (currentFace.IsEqual(otherFace)) { continue; }
			if (!currentNormal.IsParallel(helperFunctions::computeFaceNormal(otherFace), precision)) { continue; }

			if (!helperFunctions::pointOnShape(otherFace, currentPoint)) { continue; }

			isDub = true;
		}
		if (!isDub)
		{
			brepSewer.Add(currentFace);
		}
	}

	// add the horizontal faces (both roof and projects)
	for (const TopoDS_Face& currentFace : splitTopSurfaceList)
	{
		brepSewer.Add(currentFace);
		brepSewer.Add(helperFunctions::projectFaceFlat(currentFace, lowestZ));
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
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();
	// extrude the surfaces and collect their faces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> faceIdx; // pair bbox | extruded shape faces
	for (const TopoDS_Face& currentTopFace : inputFaceList)
	{
		TopoDS_Solid extrudedShape = extrudeFace(currentTopFace, true, lowestZ);
		for (TopExp_Explorer expl(extrudedShape, TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face extrusionFace = TopoDS::Face(expl.Current());
			// ignore if not vertical face
			gp_Vec currentNormal = helperFunctions::computeFaceNormal(extrusionFace);
			if (abs(currentNormal.Z()) > precisionCoarse) { continue; };

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
			if (abs(currentNormal.Z()) > precisionCoarse) { continue; };

			// find if already found in model 
			BoostBox3D faceBox = helperFunctions::createBBox(extrusionFace);
			faceIdx.insert(std::make_pair(faceBox, extrusionFace));
		}
	}

	// remove the faces that will presumably not split a single face
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> cuttingFaceIdx = indexUniqueFaces(faceIdx);
	std::vector<TopoDS_Face> splitFaceList = getSplitFaces(inputFaceList, cuttingFaceIdx);
	std::vector<TopoDS_Face> visibleFaceList = getVisTopSurfaces(splitFaceList, lowestZ, bufferSurface);
	std::vector<TopoDS_Face> visibleCleanFaceList = helperFunctions::TessellateFace(visibleFaceList);

	//clean the surfaces
	return  visibleCleanFaceList;
}


TopoDS_Shape CJGeoCreator::simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap)
{
	double precision = SettingsCollection::getInstance().precision();

	std::vector<TopoDS_Face> facelist;
	std::vector<gp_Dir> normalList;

	for (TopExp_Explorer expl(solidShape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(face);

		if (faceNomal.Magnitude() < precision)
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


std::vector<TopoDS_Face> CJGeoCreator::simplefyFacePool(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap) 
{
	double precision = SettingsCollection::getInstance().precision();

	std::vector<gp_Dir> normalList;
	for (size_t i = 0; i < surfaceList.size(); i++)
	{
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(surfaceList[i]);

		if (faceNomal.Magnitude() < precision)
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

	auto getProduct = [](const T& item) -> IfcSchema::IfcProduct* {
		if constexpr (std::is_same_v<T, TopoDS_Face>) {
			return nullptr;
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

			IfcSchema::IfcProduct* currentProduct = getProduct(surfaceList[currentIdx]);
			std::string currentType = "";
			if (currentProduct != nullptr)
			{
				currentType = currentProduct->data().type()->name();
			}
			if (currentType == "IfcWindow" || currentType == "IfcDoor") { break; } // windows and doors can not be merged

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
				std::map<IfcSchema::IfcProduct*, double> productAreaMap;
				for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
				{
					TopoDS_Face currentFace = getFace(surfaceList[mergedSurfaceIdxList[i]]);
					IfcSchema::IfcProduct* currentProduct = getProduct(surfaceList[mergedSurfaceIdxList[i]]);

					double currentArea = helperFunctions::computeArea(currentFace);
					tempFaceList.emplace_back(currentFace);

					if (productAreaMap.find(currentProduct) == productAreaMap.end())
					{
						productAreaMap.emplace(currentProduct, 0);
					}
					productAreaMap[currentProduct] += currentArea;
				}
				TopoDS_Face mergedFace = mergeFaces(tempFaceList);
				if (!mergedFace.IsNull())
				{ 
					
					// get the surface type of the type that has the largest area
					double maxArea = 0;
					IfcSchema::IfcProduct* surfaceProduct = nullptr;
					for (const auto& [typeName, areaValue] : productAreaMap)
					{
						if (areaValue > maxArea)
						{
							maxArea = areaValue;
							surfaceProduct = typeName;
						}
					}

					if constexpr (usePair)
					{
						cleanedFaceList.emplace_back(std::make_pair(mergedFace, surfaceProduct));
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

	double precision = SettingsCollection::getInstance().precision();
	gp_Vec clusterNormal = helperFunctions::computeFaceNormal(mergeFaces[0]);
	gp_Vec horizontalNormal = gp_Vec(0, 0, 1);

	gp_Trsf transform;
	std::vector<TopoDS_Face> mergingFaces;
	if (clusterNormal.Magnitude() < precision)
	{
		return TopoDS_Face();
	}

	if (!clusterNormal.IsParallel(horizontalNormal, precision))
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
		topObjectList.emplace_back(lookup->getProductShape());
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

	double precision = SettingsCollection::getInstance().precision();

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
			if (!currentCenterPoint.IsEqual(otherCenterPoint, precision)) { continue; }
			
			double otherMass = uniqueTopMass[j];
			if (abs(currentMass - otherMass) > precision) { continue; }
			
			double otherArea = uniqueTopArea[j];
			if (abs(currentArea - otherArea) > precision) { continue; }

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

	std::vector<TopoDS_Face> t;
	for (const RCollection& currentGroup : rCollectionList)
	{
		TopoDS_Face currentFace = currentGroup.getProjectedFace();
		if (currentFace.IsNull()) { continue; }
		projectedFaceList.emplace_back(currentFace);
	}

	if (projectedFaceList.empty())
	{
		return {};
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
			auto rtreePair = std::make_pair(helperFunctions::createBBox(coarseFilteredTopSurfacePair->getFace()), static_cast<int>(shapeList->size()));
			std::unique_lock<std::mutex> rtreeLock(processMutex);
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

	// make spatial index of the shapes
	bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<SurfaceGridPair>>, bgi::rstar<25>> shapeIdx;
	for (const std::shared_ptr<SurfaceGridPair>& surfGridPair : shapeList)
	{
		bg::model::box <BoostPoint3D> bbox = bg::model::box < BoostPoint3D >(
			BoostPoint3D(helperFunctions::Point3DOTB(surfGridPair->getLLLPoint())),
			BoostPoint3D(helperFunctions::Point3DOTB(surfGridPair->getURRPoint()))
			);
		shapeIdx.insert(std::make_pair(bbox, surfGridPair));
	}

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
		threadList.emplace_back([this, sublist, &shapeIdx, &processMutex, &fineFilteredShapeList]() {
			FinefilterSurface(sublist, shapeIdx, processMutex, &fineFilteredShapeList);
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
	const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<SurfaceGridPair>>, bgi::rstar<25>>& shapeIdx,
	std::mutex& processMutex,
	std::vector<std::shared_ptr<SurfaceGridPair>>* fineFilteredShapeList
)
{
	for (const std::shared_ptr<SurfaceGridPair>& currentSurfacePair : shapeList)
	{
		//	querry potential overlapping surfacesI 
		gp_Pnt lll = currentSurfacePair->getLLLPoint();
		gp_Pnt urr = currentSurfacePair->getURRPoint();
		urr.Translate(gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 1000));

		BoostBox3D bbox = BoostBox3D(helperFunctions::Point3DOTB(lll), helperFunctions::Point3DOTB(urr));

		std::vector<std::pair<BoostBox3D, std::shared_ptr<SurfaceGridPair>>> qResult;
		qResult.clear();
		shapeIdx.query(bgi::intersects(
			bbox), std::back_inserter(qResult));

		const TopoDS_Face currentFace = currentSurfacePair->getFace();
		std::vector<std::shared_ptr<SurfaceGridPair>> potentialBlockingFaces;
		for (const auto& [otherbbox, otherSurfacePair] : qResult)
		{
			if (currentFace.IsEqual(otherSurfacePair->getFace()))
			{
				continue;
			}
			potentialBlockingFaces.emplace_back(otherSurfacePair);
		}

		if (!currentSurfacePair->testIsVisable(potentialBlockingFaces)) { continue; }

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

	double precision = SettingsCollection::getInstance().precision();
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
			if (!currentCenter.IsEqual(centerpointHList[otherIdx], precision)) { continue; }

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

	double precision = SettingsCollection::getInstance().precision();
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
				if (abs(storeyElevation - otherStoreyElevation) > precision) { continue; }

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
	storeyObjects_ = cityStoreyObjects;
	garbageCollection();
	return cityStoreyObjects;
}

std::vector<std::shared_ptr<CJT::CityObject>> CJGeoCreator::makeRoomObjects(DataManager* h, const std::vector<std::shared_ptr<CJT::CityObject>>& cityStoreyObjects)
{
	std::vector<std::shared_ptr<CJT::CityObject>> cityRoomObjects;

	std::vector<std::pair<int, IfcSchema::IfcSpace*>> spacePairList;
	for (size_t i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcSpace::list::ptr sourceSpaceList = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcSpace>();

		for (auto spaceIt = sourceSpaceList->begin(); spaceIt != sourceSpaceList->end(); ++spaceIt)
		{
			spacePairList.emplace_back(std::make_pair(i, *spaceIt));
		}
	}


	for (const std::pair<int, IfcSchema::IfcSpace*>& currentSpacePair : spacePairList)
	{
		IfcSchema::IfcSpace* spaceObject = currentSpacePair.second;

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
			nlohmann::json attributeList = h->collectPropertyValues(spaceObject->GlobalId(), currentSpacePair.first);
			for (auto jsonObIt = attributeList.begin(); jsonObIt != attributeList.end(); ++jsonObIt) {
				cjRoomObject->addAttribute(jsonObIt.key(), jsonObIt.value());
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

void CJGeoCreator::setLoD32SurfaceAttributes(
	std::vector<nlohmann::json>& outSurfaceTypeCollection, 
	std::vector<int>& outTypeValueList, 
	const std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& surfacePairList,
	DataManager* h
)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	nlohmann::json grMap;
	grMap[CJObjectEnum::getString(CJObjectID::CJType)] = CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface);
	nlohmann::json wMap;
	wMap[CJObjectEnum::getString(CJObjectID::CJType)] =  CJObjectEnum::getString(CJObjectID::CJTypeWallSurface);
	nlohmann::json rMap;
	rMap[CJObjectEnum::getString(CJObjectID::CJType)] = CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface);

	outSurfaceTypeCollection.emplace_back(grMap);
	outSurfaceTypeCollection.emplace_back(wMap);
	outSurfaceTypeCollection.emplace_back(rMap);

	std::map<std::string, int> attributeLookup; //gui to lookup indx;
	int c = 0;
	for (const std::pair<TopoDS_Face, IfcSchema::IfcProduct*>& currentFacePair : surfacePairList)
	{
		c++;
		std::cout << "\tCopying Attribute data - " << c << " of " << surfacePairList.size() << "\r";
		const  IfcSchema::IfcProduct* product = currentFacePair.second;
		std::string productType = product->data().type()->name();
		const TopoDS_Face& currentFace = currentFacePair.first;

		if (productType == "IfcPlate") //TODO: make this smarter
		{
			std::map<std::string, std::string> windowMap;
			if (!helperFunctions::hasGlassMaterial(product))
			{
				outTypeValueList.emplace_back(1);
				continue;
			}
			productType = "IfcWindow";
		}
		if (productType == "IfcRoof")
		{
			outTypeValueList.emplace_back(2);
			continue;
		}
		if (productType == "IfcWindow" || productType == "IfcDoor")
		{
			if (attributeLookup.find(product->GlobalId()) != attributeLookup.end())
			{
				outTypeValueList.emplace_back(attributeLookup[product->GlobalId()]);
				continue;
			}
			nlohmann::json objectMap;
			if (product->Name().has_value())
			{
				objectMap[CJObjectEnum::getString(CJObjectID::ifcName)] = product->Name().get();
			}
			objectMap[CJObjectEnum::getString(CJObjectID::ifcGuid)] = product->GlobalId();

			if (productType == "IfcWindow")
			{
				objectMap[CJObjectEnum::getString(CJObjectID::CJType)] = CJObjectEnum::getString(CJObjectID::CJTypeWindow);
			}
			else
			{
				objectMap[CJObjectEnum::getString(CJObjectID::CJType)] = CJObjectEnum::getString(CJObjectID::CJTypeDoor);
			}

			nlohmann::json attributeList = h->collectPropertyValues(product->GlobalId());
			for (auto jsonObIt = attributeList.begin(); jsonObIt != attributeList.end(); ++jsonObIt) {
				objectMap[sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + jsonObIt.key()] = jsonObIt.value();
			}

			attributeLookup[product->GlobalId()] = outSurfaceTypeCollection.size();
			outTypeValueList.emplace_back(outSurfaceTypeCollection.size());
			outSurfaceTypeCollection.emplace_back(objectMap);
			continue;
		}
		if (productType == "IfcSlab")
		{
			std::optional<gp_Pnt> pointOnface = helperFunctions::getPointOnFace(currentFacePair.first);
			gp_Vec vecOfFace = helperFunctions::computeFaceNormal(currentFace);

			if (pointOnface == std::nullopt)
			{
				outTypeValueList.emplace_back(1);
				continue;
			}

			if (pointOnface->Z() < settingsCollection.footprintElevation() && abs(vecOfFace.Z()) > 0.1)
			{
				//TODO: do a raycast straight downwards

				outTypeValueList.emplace_back(0);
			}
			else if (pointOnface->Z() < settingsCollection.footprintElevation())
			{
				outTypeValueList.emplace_back(1);
			}
			else if (abs(helperFunctions::computeFaceNormal(currentFace).Z()) > 0.1)
			{
				//TODO: do a raycast straight upwards

				outTypeValueList.emplace_back(2);
			}
			else
			{
				outTypeValueList.emplace_back(1);
			}
			continue;
		}
		outTypeValueList.emplace_back(1);
	}
	std::cout << "\n";
	return;
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
	garbageCollection();
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
	garbageCollection();
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
	bool is03, 
	bool output
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

	if (!output) { return; }

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

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	double storeyUserBuffer = settingsCollection.horizontalSectionOffset();

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());

	std::vector<std::string> storeyGuidList = storeyCityObject->getAttributes()["IFC Guid"];
	std::vector< IfcSchema::IfcBuildingStorey*> ifcStoreyList = fetchStoreyObjects(h, storeyGuidList);

	IfcSchema::IfcBuildingStorey* ifcStorey = ifcStoreyList[0];
	IfcSchema::IfcObjectPlacement* storeyObjectPlacement = ifcStorey->ObjectPlacement();
	double storeyElevation = roundDoubleToPrecision((helperFunctions::getObjectZOffset(storeyObjectPlacement, false) * h->getScaler(0) + h->getObjectTranslation().TranslationPart().Z()), 1e-6);
	double userStoreyElevation = ifcStorey->Elevation().get() * h->getScaler(0);

	std::unique_lock<std::mutex> faceLock(storeyMutex);
	std::string storeyKey = std::to_string(userStoreyElevation) + " (" + std::to_string(storeyElevation) + ")";
	progressMap.emplace(storeyKey, 0);
	faceLock.unlock();

	std::vector<TopoDS_Face> storeySurfaceList;
	std::vector<TopoDS_Face> storeyExternalSurfaceList;
	if (is03)
	{
		makeFloorSectionComplex(false, storeySurfaceList, storeyExternalSurfaceList, h, storeyElevation + storeyUserBuffer, ifcStoreyList);
		if (settingsCollection.maked1() || settingsCollection.maked2())
		{
			std::unique_lock<std::mutex> faceLock(storeyMutex);
			LoD03ExtriorHFaces_.emplace(storeyElevation, storeyExternalSurfaceList);
			faceLock.unlock();
		}

		if (settingsCollection.maked1() || settingsCollection.maked2())
		{
			std::vector<TopoDS_Face> storeySurfaceListFlat;
			std::vector<TopoDS_Face> storeyExternalSurfaceListFlat;
			makeFloorSectionComplex(true, storeySurfaceListFlat, storeyExternalSurfaceListFlat, h, storeyElevation + storeyUserBuffer, ifcStoreyList);

			std::vector<TopoDS_Face> cleanStoreySurfaceList;
			for (const TopoDS_Face& currentStoreySurface : storeySurfaceListFlat)
			{
				TopoDS_Face cleanStoreySurface = eleminateInnerVoids(currentStoreySurface);
				cleanStoreySurfaceList.emplace_back(cleanStoreySurface);
			}
			std::unique_lock<std::mutex> faceLock(storeyMutex);
			LoD03Plates_.emplace(storeyElevation, cleanStoreySurfaceList);
			faceLock.unlock();
		}

		// lod 03 plates are not stored because all object horizontal surfaces are used for lodd.1 and d.2
	}
	else
	{
		makeFloorSection(storeySurfaceList, h, storeyElevation + storeyUserBuffer);

		if (settingsCollection.makec1() || settingsCollection.makec2())
		{
			std::vector<TopoDS_Face> cleanStoreySurfaceList;
			for (const TopoDS_Face& currentStoreySurface : storeySurfaceList)
			{
				TopoDS_Face cleanStoreySurface = eleminateInnerVoids(currentStoreySurface);
				cleanStoreySurfaceList.emplace_back(cleanStoreySurface);
			}
			std::unique_lock<std::mutex> faceLock(storeyMutex);
			LoD02Plates_.emplace(storeyElevation, cleanStoreySurfaceList); 
			faceLock.unlock();
		}
	}

	if (!settingsCollection.makeInterior() || 
		!settingsCollection.make02() && !is03 ||
		!settingsCollection.make03() && is03)
	{
		progressMap[storeyKey] = 1;
		return;
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

	IfcSchema::IfcSpace::list spaceList;
	for (size_t i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcSpace::list::ptr sourceSpaceList = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcSpace>();

		for (auto spaceIt = sourceSpaceList->begin(); spaceIt != sourceSpaceList->end(); ++spaceIt)
		{
			spaceList.push(*spaceIt);
		}
	}

	std::vector<TopoDS_Shape> copyLoD02GeoList;
	std::vector<TopoDS_Shape> copyLoD12GeoList;
	std::vector<TopoDS_Shape> copyLoD22GeoList;

	for (auto spaceIt = spaceList.begin(); spaceIt != spaceList.end(); ++spaceIt)
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

				TopoDS_Face cleanedCurrentFace = helperFunctions::TessellateFace(currentFace);
				if (settingsCollection.make02() || settingsCollection.make12())
				{
					flatFaceList.emplace_back(helperFunctions::projectFaceFlat(cleanedCurrentFace, lowestZ));
				}
				if (settingsCollection.make22())
				{
					topFaceList.emplace_back(cleanedCurrentFace);
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

				CJT::GeoObject roomGeoObject22 = kernel->convertToJSON(roomPrismList[0], "2.2");

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

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(bbox, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD00));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(bbox, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD00));
	}
	 
	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
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

	std::vector<TopoDS_Shape> faceCopyCollection;
	for (const std::vector<TopoDS_Face>& faceCluster : LoD03RoofFaces_)
	{
		for (TopoDS_Face currentShape : faceCluster)
		{
			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			currentShape.Move(localRotationTrsf);
			faceCopyCollection.emplace_back(currentShape);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "0.3");

			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (hasFootprints_)
	{
		std::map<std::string, std::string> semanticFootData;
		semanticFootData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));

		// make the footprint
		for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			TopoDS_Shape footprint = buildingSurfaceData.getFootPrint();
			if (footprint.IsNull()) { continue; }

			gp_Trsf trsf;
			trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			footprint.Move(trsf);
			faceCopyCollection.emplace_back(footprint);

			CJT::GeoObject geoObject = kernel->convertToJSON(footprint, "0.3");
			geoObject.appendSurfaceData(semanticFootData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(faceCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD03));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(faceCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD03));
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

	std::vector<TopoDS_Shape> faceCopyCollection;
	for (std::vector<TopoDS_Face> faceCluster : LoD04RoofFaces_)
	{
		for (TopoDS_Face currentShape : faceCluster)
		{
			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			currentShape.Move(localRotationTrsf);
			faceCopyCollection.emplace_back(currentShape);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "0.4");
			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (hasFootprints_)
	{
		std::map<std::string, std::string> semanticFootData;
		semanticFootData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));

		// make the footprint
		for (const BuildingSurfaceCollection& buildingSurfaceData : buildingSurfaceDataList_)
		{
			TopoDS_Shape footprint = buildingSurfaceData.getFootPrint();
			if (footprint.IsNull()) { continue; }

			gp_Trsf trsf;
			trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
			footprint.Move(trsf);
			faceCopyCollection.emplace_back(footprint);

			CJT::GeoObject geoObject = kernel->convertToJSON(footprint, "0.4");
			geoObject.appendSurfaceData(semanticFootData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(faceCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD04));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(faceCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD04));
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
	garbageCollection();
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD13(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD13) << std::endl;
	finishedLoD13_ = true;

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
	garbageCollection();
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD22(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD22) << std::endl;
	finishedLoD22_ = true;

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
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDb0(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDb0) << std::endl;
	finishedLoDb0_ = true;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::vector< CJT::GeoObject> geoObjectList;

	if (!hasRoofOutlines()) {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoDb.0");
		return {};
	}
	if (!hasFootprints()) {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		ErrorCollection::getInstance().addError(ErrorID::warningNoFootprint, "LoDb.0");
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
		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "b.0");

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

		CJT::GeoObject geoOverhangObject = kernel->convertToJSON(compound, "b.0");
		geoOverhangObject.appendSurfaceData(semanticRoofData);
		for (size_t i = 0; i < overhangRoof.size(); i++)
		{
			geoOverhangObject.appendSurfaceTypeValue(0);
		}
		geoObjectList.emplace_back(geoOverhangObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoDb));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoDb));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDc1(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDc1) << std::endl;


	std::vector< CJT::GeoObject> geoObjectList;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	std::vector<TopoDS_Face> outerShapeFaces;
	std::map<double, std::vector<TopoDS_Face>> horizontalStoreyFaces;

	if (LoD02Plates_.empty())
	{
		//TODO: add line stating that plates are required
		if (storeyObjects_.empty()) {makeStoreyObjects(h); }
		make2DStoreys(h, kernel, storeyObjects_, 1, false, false);
	}	
	finishedLoDc1_ = true;

	extrudeStoreyGeometry(true, true, h, LoD02Plates_, outerShapeFaces, horizontalStoreyFaces);
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>> horizontalFaceIndex;
	for (const std::pair<double, std::vector<TopoDS_Face>>& currentFacePair : horizontalStoreyFaces)
	{
		for (const TopoDS_Face& currentFace : currentFacePair.second)
		{
			horizontalFaceIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), currentFace));
		}
	}

	for (const TopoDS_Face currentFace: TrimHStoreyFaces(horizontalFaceIndex))
	{
		outerShapeFaces.emplace_back(currentFace);
	}

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	BRepBuilderAPI_Sewing brepSewer;
	for (const TopoDS_Face face : outerShapeFaces) {
		brepSewer.Add(face.Moved(trsf));
	}
	brepSewer.Perform();
	TopoDS_Shape simplefiedShape = simplefySolid(brepSewer.SewedShape());

	CJT::GeoObject geoObject = kernel->convertToJSON(simplefiedShape, "c.1");
	createSemanticData(&geoObject, simplefiedShape);
	geoObjectList.emplace_back(geoObject);

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(simplefiedShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoDc1));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(simplefiedShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoDc1));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDc2(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDc2) << std::endl;

	std::vector< CJT::GeoObject> geoObjectList;
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	BRepBuilderAPI_Sewing brepSewer;

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	std::vector<TopoDS_Shape> shapeCopyCollection;
	// split roof surfaces over the storey elevations
	if (!hasRoofOutlines()) {
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		ErrorCollection::getInstance().addError(ErrorID::warningNoRoofOutline, "LoDc.2");
		return {};
	}

	if (LoD02Plates_.empty())
	{
		if (storeyObjects_.empty()) { makeStoreyObjects(h); }
		make2DStoreys(h, kernel, storeyObjects_, 1, false, false);
	}

	std::vector<std::vector<TopoDS_Face>> roofList;
	if (LoD04RoofFaces_.size() == 0) {
		roofList = makeRoofFaces(h, kernel, 1, false, false);
	}
	else
	{
		roofList = LoD04RoofFaces_;
	}
	finishedLoDc2_ = true;

	std::vector<TopoDS_Face> roofOverhangSurfaceList;
	for (const std::vector<TopoDS_Face>& currentRoofFaceList : roofList)
	{
		std::map<double, std::vector<TopoDS_Face>> RoofFaces;
		for (const TopoDS_Face& currentRoofFace : currentRoofFaceList)
		{
			// filter out the flat surfaces that rest on a roofing section
			gp_Vec currentFaceNormal = helperFunctions::computeFaceNormal(currentRoofFace);
			if (abs(currentFaceNormal.X()) < 1e-4 && abs(currentFaceNormal.Y()) < 1e-4)
			{
				if (LoD02Plates_.find(roundDoubleToPrecision(helperFunctions::getHighestZ(currentRoofFace), 1e-6)) != LoD02Plates_.end())
				{
					continue;
				}
			}

			// check if faces need to be splitted because they span multiple storey elevations
			gp_Pnt lllPoint;
			gp_Pnt urrPoint;
			helperFunctions::bBoxDiagonal(currentRoofFace, &lllPoint, &urrPoint);

			std::vector<double> splitHeight;
			for (const auto& storeyFacePair : LoD02Plates_)
			{
				double height = storeyFacePair.first;
				if (height - lllPoint.Z() < 1e-6 || height - urrPoint.Z() > 1e-6)
				{
					continue;
				}
				splitHeight.emplace_back(height);
			}

			std::vector<TopoDS_Face> currentFaceList;
			if (!splitHeight.empty())
			{
				//TODO: split the faces
			}
			else
			{
				currentFaceList = { currentRoofFace };
			}

			// extrude the roof surfaces to the storey elevation they fall directly above
			double baseHeight = 0;
			for (const TopoDS_Face& currentSplitFace : currentFaceList )
			{
				gp_Pnt spltlllPoint;
				gp_Pnt splturrPoint;

				helperFunctions::bBoxDiagonal(currentRoofFace, &spltlllPoint, &splturrPoint);

				for (const auto& storeyFacePair : LoD02Plates_)
				{
					double height = storeyFacePair.first;

					if (height <= spltlllPoint.Z())
					{
						baseHeight = roundDoubleToPrecision(height, 1e-6);
						continue;
					}
					break;
				}

				TopTools_ListOfShape toolList;
				TopTools_ListOfShape argumentList;
				argumentList.Append(currentSplitFace);

				std::vector<TopoDS_Face> splitterFaceList = LoD02Plates_[baseHeight];

				for (const TopoDS_Face& splitterFace : splitterFaceList)
				{
					TopoDS_Shape extrudedShape = extrudeFace(splitterFace, false, h->getUrrPoint().Z() + 5);
					toolList.Append(extrudedShape);
				}

				BRepAlgoAPI_Splitter splitter;
				splitter.SetFuzzyValue(precisionCoarse);
				splitter.SetArguments(argumentList);
				splitter.SetTools(toolList);
				splitter.Build();

				for (TopExp_Explorer explorer(splitter.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
				{
					const TopoDS_Face& currentSplitFace = TopoDS::Face(explorer.Current());

					std::optional<gp_Pnt> optionalPoint =  helperFunctions::getPointOnFace(currentSplitFace);
					if (optionalPoint == std::nullopt) { continue; }
					if (!helperFunctions::pointOnFace(splitterFaceList, gp_Pnt(optionalPoint->X(), optionalPoint->Y(), baseHeight)))
					{
						roofOverhangSurfaceList.emplace_back(currentSplitFace);
						continue;
					}

					if (RoofFaces.find(baseHeight) == RoofFaces.end())
					{
						RoofFaces[baseHeight] = {};
					}
					RoofFaces[baseHeight].emplace_back(currentSplitFace);
				}
			}
		}		

		bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>> horizontalFaceIndex;
		std::vector<TopoDS_Face> outerShapeFaces;
		for (const auto& [baseHeight, roofList] : RoofFaces)
		{
			std::vector<TopoDS_Shape> extrudedRoofs = computePrisms(roofList, baseHeight, false);

			for (const TopoDS_Shape currentShape : extrudedRoofs)
			{
				for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
				{
					const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
					gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);

					if (abs(currentNormal.X()) > 1e-6 || abs(currentNormal.Y()) > 1e-6)
					{
						outerShapeFaces.emplace_back(currentFace);
						
					}
					else
					{
						horizontalFaceIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), currentFace));
					}
				}
			}
		}

		std::map<double, std::vector<TopoDS_Face>> horizontalStoreyFaces;
		extrudeStoreyGeometry(true, false, h, LoD02Plates_, outerShapeFaces, horizontalStoreyFaces);


		for (const std::pair<double, std::vector<TopoDS_Face>>& currentFacePair : horizontalStoreyFaces)
		{
			for (const TopoDS_Face& currentFace : currentFacePair.second)
			{
				horizontalFaceIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), currentFace));
			}
		}

		for (const TopoDS_Face currentFace : TrimHStoreyFaces(horizontalFaceIndex))
		{
			outerShapeFaces.emplace_back(currentFace);
		}

		BRepBuilderAPI_Sewing brepSewer;
		for (const TopoDS_Face face : outerShapeFaces) {
			brepSewer.Add(face);
		}
		brepSewer.Perform();

		TopoDS_Shape simplefiedShape = simplefySolid(brepSewer.SewedShape());
		gp_Trsf trsf;
		trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

		CJT::GeoObject geoObject = kernel->convertToJSON(simplefiedShape, "c.2");
		createSemanticData(&geoObject, simplefiedShape);
		geoObjectList.emplace_back(geoObject);

		if (!roofOverhangSurfaceList.empty())
		{
			BRep_Builder builder;
			TopoDS_Compound compound;
			builder.MakeCompound(compound);
			for (auto face : roofOverhangSurfaceList)
			{
				face.Move(trsf);
				builder.Add(compound, face);
			}
			shapeCopyCollection.emplace_back(compound);

			CJT::GeoObject geoOverhangObject = kernel->convertToJSON(compound, "c.2");
			geoOverhangObject.appendSurfaceData(semanticRoofData);
			for (size_t i = 0; i < roofOverhangSurfaceList.size(); i++)
			{
				geoOverhangObject.appendSurfaceTypeValue(0);
			}
			geoObjectList.emplace_back(geoOverhangObject);
		}	
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoDb));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoDb));
	}
	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDd1(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDd1) << std::endl;


	std::vector< CJT::GeoObject> geoObjectList;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	if (LoD03Plates_.empty())
	{
		if (storeyObjects_.empty()) { makeStoreyObjects(h); }
		make2DStoreys(h, kernel, storeyObjects_, 1, true, false);
	}
	finishedLoDd1_ = true;

	std::vector<TopoDS_Face> outerShapeFaces;
	std::map<double, std::vector<TopoDS_Face>> horizontalStoreyFaces;
	extrudeStoreyGeometry(false, true, h, LoD03Plates_, outerShapeFaces, horizontalStoreyFaces);

	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>> horizontalFaceIndex;
	for (const std::pair<double, std::vector<TopoDS_Face>>& currentFacePair : horizontalStoreyFaces)
	{
		for (const TopoDS_Face& currentFace : currentFacePair.second)
		{
			horizontalFaceIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), currentFace));
		}
	}

	for (const TopoDS_Face currentFace : TrimHStoreyFaces(horizontalFaceIndex))
	{
		outerShapeFaces.emplace_back(currentFace);
	}

	std::vector<TopoDS_Shape> shapeCopyCollection;
	gp_Trsf trsf;
	trsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	BRepBuilderAPI_Sewing brepSewer;
	for (const TopoDS_Face face : outerShapeFaces) {
		brepSewer.Add(face.Moved(trsf));
	}
	brepSewer.Perform();
	TopoDS_Shape simplefiedShape = simplefySolid(brepSewer.SewedShape());

	CJT::GeoObject geoObject = kernel->convertToJSON(simplefiedShape, "d.1");
	createSemanticData(&geoObject, simplefiedShape);
	geoObjectList.emplace_back(geoObject);

	shapeCopyCollection.emplace_back(simplefiedShape);

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	for (const auto& [elevation, surfaceList] : LoD03ExtriorHFaces_)
	{
		BRep_Builder builder;
		TopoDS_Compound compound;
		builder.MakeCompound(compound);

		bool hasFaces = false;
		for (auto face : surfaceList)
		{
			face.Move(trsf);
			builder.Add(compound, face);
			hasFaces = true;
		}
		shapeCopyCollection.emplace_back(compound);

		if (!hasFaces)
		{
			continue;
		}

		CJT::GeoObject geoOverhangObject = kernel->convertToJSON(compound, "d.1");
		geoOverhangObject.appendSurfaceData(semanticRoofData);
		for (size_t i = 0; i < surfaceList.size(); i++)
		{
			geoOverhangObject.appendSurfaceTypeValue(0);
		}
		geoObjectList.emplace_back(geoOverhangObject);
	}

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoDd1));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(shapeCopyCollection, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoDd1));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDd2(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDd2) << std::endl;
	finishedLoDd2_ = true;

	std::cout << "[WARNING] LoDd.2 processes are not yet implemented\n";
	return std::vector<CJT::GeoObject>();
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDe0(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDe0) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	if (storeyObjects_.empty()){makeStoreyObjects(h);}
	finishedLoDe0_ = true;

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	for (const std::shared_ptr<CJT::CityObject>& storeyObject: storeyObjects_)
	{
		std::vector<std::string> storeyGuidList = storeyObject->getAttributes()["IFC Guid"];
		std::vector< IfcSchema::IfcBuildingStorey*> ifcStoreyList = fetchStoreyObjects(h, storeyGuidList);

		for (IfcSchema::IfcBuildingStorey* currentStorey : ifcStoreyList)
		{
			IfcSchema::IfcRelContainedInSpatialStructure::list::ptr containedStructure = currentStorey->ContainsElements()->as<IfcSchema::IfcRelContainedInSpatialStructure>();
			for (auto csit = containedStructure->begin(); csit != containedStructure->end(); ++csit)
			{
				IfcSchema::IfcProduct::list::ptr storeyRelatedProducts = (*csit)->RelatedElements();
				for (auto srit = storeyRelatedProducts->begin(); srit != storeyRelatedProducts->end(); ++srit)
				{
					IfcSchema::IfcProduct* currentProduct = *srit;
					TopoDS_Shape currentShape = h->getObjectShapeFromMem(currentProduct, true);

					if (currentShape.IsNull()) { continue; }
					CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "4.0");

					nlohmann::json attributeMap;
					attributeMap[CJObjectEnum::getString(CJObjectID::CJType)] = "+" + currentProduct->data().type()->name();
					nlohmann::json attributeList = h->collectPropertyValues(currentProduct->GlobalId());
					for (auto jsonObIt = attributeList.begin(); jsonObIt != attributeList.end(); ++jsonObIt) {
						attributeMap[sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + jsonObIt.key()] = jsonObIt.value();
					}

					int faceCount = 0;
					for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next()) { faceCount++; }
					std::vector<int>TypeValueList(faceCount, 0);

					geoObject.setSurfaceTypeValues(TypeValueList);
					geoObject.appendSurfaceData(attributeMap);
					geoObjectList.emplace_back(geoObject);
				}
			}
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}

std::vector<CJT::GeoObject> CJGeoCreator::makeLoDe1(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoDe1) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>> voxelIndex;
	// collect and index the voxels to which rays are cast
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = voxelGrid_->getIntersectingVoxels();
	std::vector<std::shared_ptr<voxel>> externalVoxel = voxelGrid_->getExternalVoxels();
	intersectingVoxels.insert(intersectingVoxels.end(), externalVoxel.begin(), externalVoxel.end());
	populateVoxelIndex(&voxelIndex, intersectingVoxels);
	LoDE1Faces_ = getE1Faces(h, kernel, unitScale, intersectingVoxels, voxelIndex);

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());

	BRep_Builder builder;
	TopoDS_Compound collectionShape;
	builder.MakeCompound(collectionShape);

	std::vector<nlohmann::json> SurfaceTypeCollection;
	std::vector<int> typeValueList;
	setLoD32SurfaceAttributes(SurfaceTypeCollection, typeValueList, LoDE1Faces_, h);
	for (const std::pair<TopoDS_Face, IfcSchema::IfcProduct*>& currentFacePair : LoDE1Faces_)
	{
		const TopoDS_Face& currentFace = currentFacePair.first;
		builder.Add(collectionShape, currentFace);
	}
	collectionShape.Move(localRotationTrsf);

	CJT::GeoObject geoObject = kernel->convertToJSON(collectionShape, "e.1");
	geoObject.setSurfaceTypeValues(typeValueList);

	geoObject.setSurfaceData(SurfaceTypeCollection);
	geoObjectList.emplace_back(geoObject);

	if (settingsCollection.createOBJ())
	{
		helperFunctions::writeToOBJ(collectionShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoDe1));
	}

	if (settingsCollection.createSTEP())
	{
		helperFunctions::writeToSTEP(collectionShape, settingsCollection.getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoDe1));
	}

	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
	return geoObjectList;
}


std::vector< CJT::GeoObject>CJGeoCreator::makeLoD32(DataManager* h, CJT::Kernel* kernel, int unitScale)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD32) << std::endl;

	finishedLoD32_ = true;

	auto startTime = std::chrono::steady_clock::now();
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>> voxelIndex;
	// collect and index the voxels to which rays are cast
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = voxelGrid_->getIntersectingVoxels();
	std::vector<std::shared_ptr<voxel>> externalVoxel = voxelGrid_->getExternalVoxels();
	intersectingVoxels.insert(intersectingVoxels.end(), externalVoxel.begin(), externalVoxel.end());
	populateVoxelIndex(&voxelIndex, intersectingVoxels);

	// evaluate which surfaces are visible from the exterior
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> outerSurfacePairList;
	if (!LoDE1Faces_.empty())
	{
		outerSurfacePairList = LoDE1Faces_;
	}
	else
	{
		outerSurfacePairList = getE1Faces(h, kernel, unitScale, intersectingVoxels, voxelIndex);
	}

	// clip surfaces that are in contact with eachother
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> splitOuterSurfacePairList;
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> unSplitOuterSurfacePairList;
	splitOuterSurfaces(splitOuterSurfacePairList, unSplitOuterSurfacePairList, outerSurfacePairList);
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>().swap(outerSurfacePairList);

	// remove internal faces
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> splitFaceIndx;
	for (const auto& [currentFace, currentProduct] : splitOuterSurfacePairList)
	{
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		splitFaceIndx.insert(std::make_pair(currentBox, currentFace));
	}
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> finalOuterSurfacePairList;
	for (const auto& [currentFace, currentProduct] : unSplitOuterSurfacePairList)
	{
		finalOuterSurfacePairList.emplace_back(std::pair(currentFace, currentProduct));
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		splitFaceIndx.insert(std::make_pair(currentBox, currentFace));
	}
	simpleRaySurfaceCast(finalOuterSurfacePairList, splitOuterSurfacePairList, voxelIndex, splitFaceIndx);

	// remove dub and incapsulated surfaces by merging them
	std::vector<gp_Dir> normalList;
	normalList.reserve(finalOuterSurfacePairList.size());
	for (const auto& [currentFace, currentProduct] : finalOuterSurfacePairList)
	{
		normalList.emplace_back(helperFunctions::computeFaceNormal(currentFace));
	}
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> cleanedOuterSurfacePairList = simplefyFacePool(finalOuterSurfacePairList, normalList); //TODO: multithread
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>().swap(finalOuterSurfacePairList);

	// make the collection compund shape
	BRep_Builder builder;
	TopoDS_Compound collectionShape;
	builder.MakeCompound(collectionShape);

	std::vector<nlohmann::json> SurfaceTypeCollection;
	std::vector<int> typeValueList;
	setLoD32SurfaceAttributes(SurfaceTypeCollection, typeValueList, cleanedOuterSurfacePairList, h);
	for (const std::pair<TopoDS_Face, IfcSchema::IfcProduct*>& currentFacePair : cleanedOuterSurfacePairList)
	{
		const TopoDS_Face& currentFace = currentFacePair.first;
		builder.Add(collectionShape, currentFace);
	}
	collectionShape.Move(localRotationTrsf);

	CJT::GeoObject geoObject = kernel->convertToJSON(collectionShape, "3.2");
	geoObject.setSurfaceData(SurfaceTypeCollection);
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
	garbageCollection();
	return geoObjectList;
}


void CJGeoCreator::makeComplexLoDRooms(DataManager* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale) {	
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	IfcSchema::IfcSpace::list spaceList;
	for (size_t i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcSpace::list::ptr sourceSpaceList = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcSpace>();

		for (auto spaceIt = sourceSpaceList->begin(); spaceIt != sourceSpaceList->end(); ++spaceIt)
		{
			spaceList.push(*spaceIt);
		}
	}

	for (auto spaceIt = spaceList.begin(); spaceIt != spaceList.end(); ++spaceIt)
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

	std::vector< CJT::GeoObject> geoObjectList; // final output collection

	voxelGrid_->computeSurfaceSemantics(h);
	std::vector<int> globalTypeValueList;
	TopoDS_Shape sewedShape = voxels2Shape(0, &globalTypeValueList);

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	sewedShape.Move(localRotationTrsf);

	int typevalCounter = 0;
	for (TopExp_Explorer expl(sewedShape, TopAbs_SHELL); expl.More(); expl.Next())
	{
		CJT::GeoObject geoObject;
		TopoDS_Shell currentShell = TopoDS::Shell(expl.Current());

		if (currentShell.Closed())
		{
			BRep_Builder brepBuilder;
			TopoDS_Shell shell;
			brepBuilder.MakeShell(shell);
			TopoDS_Solid voxelSolid;
			brepBuilder.MakeSolid(voxelSolid);
			brepBuilder.Add(voxelSolid, currentShell);
			geoObject = kernel->convertToJSON(voxelSolid, "5.0", true);

			if (SettingsCollection::getInstance().createOBJ())
			{
				helperFunctions::writeToOBJ(voxelSolid, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD50));
			}

			if (SettingsCollection::getInstance().createSTEP())
			{
				helperFunctions::writeToSTEP(voxelSolid, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD50));
			}
		}
		else
		{
			ErrorCollection::getInstance().addError(ErrorID::warningNoSolid, "LoD5.0");
			std::cout << errorWarningStringEnum::getString(ErrorID::warningNoSolid) << std::endl;
			geoObject = kernel->convertToJSON(currentShell, "5.0");

			if (SettingsCollection::getInstance().createSTEP())
			{
				helperFunctions::writeToOBJ(currentShell, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::OBJLoD50));
			}

			if (SettingsCollection::getInstance().createSTEP())
			{
				helperFunctions::writeToSTEP(currentShell, SettingsCollection::getInstance().getOutputBasePath() + fileExtensionEnum::getString(fileExtensionID::STEPLoD50));
			}
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


		// split the typevalue list over the seperate building objects
		int localTypeStartCounter = typevalCounter;
		for (TopExp_Explorer faceExpl(currentShell, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
		{
			typevalCounter++;
		}

		std::vector<int>::const_iterator localTypeListStart = globalTypeValueList.begin() + localTypeStartCounter;
		std::vector<int>::const_iterator localTypeListEnd = globalTypeValueList.begin() + typevalCounter;
		std::vector<int> localTypeValueList(localTypeListStart, localTypeListEnd);

		geoObject.setSurfaceTypeValues(localTypeValueList);

		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::steady_clock::now());
	garbageCollection();
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
	auto startTime = std::chrono::steady_clock::now();

	std::vector<CJT::CityObject> siteObjectList;
	double buffer = 0.001;
	double lowPrecision = SettingsCollection::getInstance().precisionCoarse();
	int geoCount = 0;

	std::vector<TopoDS_Shape> siteShapeList;

	// get the surfaces from the geo or site objects
	for (int i = 0; i < h->getSourceFileCount(); i++)
	{
		IfcSchema::IfcSite::list::ptr siteElements = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcSite>();

		if (!siteElements->size()) { continue; }

		IfcSchema::IfcSite* siteElement = *siteElements->begin();
		
		if (!siteElement->Representation()) { continue; }
		TopoDS_Shape siteShape = h->getObjectShape(siteElement);

		if (siteShape.IsNull()) { continue; }

		siteShapeList.emplace_back(siteShape);

	}
	if (siteShapeList.empty())
	{
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		for (int i = 0; i < h->getSourceFileCount(); i++)
		{
			IfcSchema::IfcGeographicElement::list::ptr geographicElements = h->getSourceFile(i)->instances_by_type<IfcSchema::IfcGeographicElement>();

			if (!geographicElements->size()) { continue; }
			for (auto it = geographicElements->begin(); it != geographicElements->end(); ++it)
			{
				IfcSchema::IfcGeographicElement* geographicElement = *it;
				if (geographicElement->PredefinedType() != IfcSchema::IfcGeographicElementTypeEnum::Value::IfcGeographicElementType_TERRAIN) { continue; }

				TopoDS_Shape geographicShape = h->getObjectShape(geographicElement);

				if (geographicShape.IsNull()) { continue; }

				siteShapeList.emplace_back(geographicShape);
			}
		}
#endif // USE_IFC4
	}
	
	if (siteShapeList.empty())
	{
		ErrorCollection::getInstance().addError(ErrorID::warningIfcNoSites);
		std::cout << errorWarningStringEnum::getString(ErrorID::warningIfcNoSites) << std::endl;
		return std::vector<CJT::CityObject>();
	}

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	for (TopoDS_Shape currentSiteShape : siteShapeList)
	{
		CJT::CityObject siteObject;
		currentSiteShape.Move(localRotationTrsf);
		CJT::GeoObject geoObject = kernel->convertToJSON(currentSiteShape, "3");
		siteObject.addGeoObject(geoObject);
		siteObject.setType(CJT::Building_Type::TINRelief);
		siteObject.setName(CJObjectEnum::getString(CJObjectID::CJTypeSiteObject));

		siteObjectList.emplace_back(siteObject);
	}

	printTime(startTime, std::chrono::steady_clock::now());
	return siteObjectList;
}

TopoDS_Shape CJGeoCreator::voxels2Shape(int roomNum, std::vector<int>* typeList)
{
	std::vector<std::thread> threads;
	std::vector<std::pair<TopoDS_Face, CJObjectID>> threadFaceLists;
	std::mutex faceListMutex;

	double precision = SettingsCollection::getInstance().precision();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	for (int i = 0; i < 6; i++) {
		//processDirectionalFaces(i, roomNum, faceListMutex, std::ref(threadFaceLists));
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

			if (!currentGprops.CentreOfMass().IsEqual(typedGprops.CentreOfMass(), precision)) { continue; }
			if (abs(currentGprops.Mass() - typedGprops.Mass()) > precisionCoarse) { continue; }
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

std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> CJGeoCreator::getE1Faces(
	DataManager* h,
	CJT::Kernel* kernel, 
	int unitScale, 
	const std::vector < std::shared_ptr<voxel>>& intersectingVoxels,
	const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>& voxelIdx)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();


	// collect and index the products which are presumed to be part of the exterior
	std::vector<Value> productLookupValues = getUniqueProductValues(intersectingVoxels);
	if (productLookupValues.size() <= 0)
	{
		throw ErrorID::failedLoD32;
		return{};
	}

	std::vector<int> scoreList;
	std::vector<Value> cleanedProductLookupValues;
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> faceIndx;
	double searchBuffer = settingsCollection.searchBufferLod32();
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(productLookupValues[i].second);
		std::string lookupType = lookup->getProductPtr()->data().type()->name();
		TopoDS_Shape currentShape = lookup->getProductShape();

		BoostBox3D totalBox = helperFunctions::createBBox(currentShape, searchBuffer);
		int score = static_cast<int>(std::distance(voxelIdx.qbegin(bgi::intersects(totalBox)), voxelIdx.qend()));
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
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> outerSurfacePairList;
	getOuterRaySurfaces(outerSurfacePairList, cleanedProductLookupValues, scoreList, h, faceIndx, voxelIdx);
	std::vector<int>().swap(scoreList);

	return outerSurfacePairList;
	
}

void CJGeoCreator::getOuterRaySurfaces(std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& outerSurfacePairList, const std::vector<Value>& totalValueObjectList, const std::vector<int>& scoreList, DataManager* h, const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIdx, const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>& voxelIndex)
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
	std::mutex processedCountMutex;

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
		threadList.emplace_back([this, &outerSurfacePairList, sublist = std::move(sublist), &processedObjects, &processedCountMutex, &listMutex, &h, &faceIdx, &voxelIndex]() {
			getOuterRaySurfaces(outerSurfacePairList, sublist, processedObjects, processedCountMutex, listMutex, h, faceIdx, voxelIndex);
		});
	}

	threadList.emplace_back([&] {updateCounter("Evaluating outer objects", totalValueObjectList.size(), processedObjects, listMutex);  });

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	return;
}

void CJGeoCreator::getOuterRaySurfaces(
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& outerSurfacePairList,
	const std::vector<Value>& valueObjectList, 
	int& processedObject,
	std::mutex& processedObjectmutex,
	std::mutex& listmutex,
	DataManager* h,
	const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIdx,
	const bgi::rtree<std::pair<BoostBox3D, std::shared_ptr<voxel>>, bgi::rstar<25>>&voxelIndex
)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precision = settingsCollection.precision();
	double searchBuffer = settingsCollection.searchBufferLod32();
	for (const Value& currentValue : valueObjectList)
	{
		std::unique_lock<std::mutex> processCountLock(processedObjectmutex);
		processedObject++;
		processCountLock.unlock();

		std::shared_ptr<IfcProductSpatialData> lookup = h->getLookup(currentValue.second);
		const std::string& lookupType = lookup->getProductPtr()->data().type()->name();
		const TopoDS_Shape& currentShape = lookup->getProductShape(); 

		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			bool faceIsExterior = false;
			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
			if (helperFunctions::getPointCount(currentFace) < 3) { continue; }

			//Create a grid over the surface and the offsetted wire
			std::vector<gp_Pnt> surfaceGridList = helperFunctions::getPointGridOnSurface(currentFace);
			std::unique_lock<std::mutex> lock(listmutex); //TODO: better mutex
			std::vector<gp_Pnt> wireGridList = helperFunctions::getPointGridOnWire(currentFace);
			lock.unlock();
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
					const gp_Pnt& targetPoint = targetVoxel->getOCCTCenterPoint();
					bg::model::box<BoostPoint3D> productQuerybox(helperFunctions::createBBox(gridPoint, targetPoint, precision));
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

							if (helperFunctions::triangleIntersecting({ gridPoint, targetPoint }, trianglePoints))
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

			std::unique_lock<std::mutex> listLock(listmutex);
			outerSurfacePairList.emplace_back(std::make_pair(currentFace, lookup->getProductPtr()));
			listLock.unlock();
		}
	}
	return;
}


void CJGeoCreator::updateCounter(
	const std::string& prefixText,
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
			<< "\t" << prefixText << " - " << currentObjectCount << " of " << totalObjects << "      \r";		

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

void CJGeoCreator::extrudeStoreyGeometry(
	bool refine, 
	bool useTopHeight,
	DataManager* h,
	const std::map<double, std::vector<TopoDS_Face>>& inHorizontalStoreyPlates,
	std::vector<TopoDS_Face>& outVerticalextFaces,
	std::map<double, std::vector<TopoDS_Face>>& outHorizontalStoreyFaces)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precision = settingsCollection.precision();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	std::vector<double> heightList;
	heightList.reserve(inHorizontalStoreyPlates.size());
	for (const auto& [height, storeys] : inHorizontalStoreyPlates)
	{
		heightList.emplace_back(height);
	}
	std::sort(heightList.begin(), heightList.end());

	double topHeight = h->getUrrPoint().Z();
	if (!useTopHeight)
	{
		topHeight = heightList.back();
	}

	for (size_t i = 0; i < heightList.size(); i++)
	{
		double nextHeight = topHeight;
		double currentHeight = heightList[i];

		std::vector<TopoDS_Face> currentStoreyFaceList = inHorizontalStoreyPlates.at(currentHeight);
		std::vector<TopoDS_Face> nextStoreyFaceList;

		if (i + 1 != heightList.size())
		{
			nextHeight = heightList[i + 1];
			nextStoreyFaceList = inHorizontalStoreyPlates.at(nextHeight);
		}
		if (nextHeight - currentHeight < 1e-6) { continue; }

		std::vector<TopoDS_Face> horizontalFaces;

		for (const TopoDS_Face& currentStoryFace : currentStoreyFaceList)
		{	
			if (currentStoryFace.IsNull()) { continue; }
			if (helperFunctions::computeArea(currentStoryFace) < precisionCoarse) { continue; }
			std::vector<TopoDS_Face> toBeExtrudedFaces;

			if (refine)// get the surface that is compliant with the current storey face and the face of the storey above it
			{

				TopTools_ListOfShape toolList;
				for (const TopoDS_Face& otherStoryFace : nextStoreyFaceList)
				{
					TopoDS_Face flattenedFace = helperFunctions::projectFaceFlat(otherStoryFace, currentHeight);
					toolList.Append(flattenedFace);
				}

				TopTools_ListOfShape argumentList;
				argumentList.Append(currentStoryFace);

				BRepAlgoAPI_Splitter splitter;
				splitter.SetFuzzyValue(precisionCoarse);
				splitter.SetArguments(argumentList);
				splitter.SetTools(toolList);
				splitter.Build();

				for (TopExp_Explorer explorer(splitter.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
				{
					const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());

					std::optional<gp_Pnt> optionalPoint = helperFunctions::getPointOnFace(currentFace);
					if (optionalPoint == std::nullopt) { continue; }

					for (const TopoDS_Shape& otherStoryFace : toolList)
					{
						if (!helperFunctions::pointOnShape(otherStoryFace, *optionalPoint)) { continue; }
						toBeExtrudedFaces.emplace_back(currentFace);
						break;
					}
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
					TopoDS_Face tesselatedFace = helperFunctions::TessellateFace(currentFace);
					if (abs(helperFunctions::computeFaceNormal(tesselatedFace).Z()) < settingsCollection.precisionCoarse())
					{
						outVerticalextFaces.emplace_back(tesselatedFace);
						continue;
					}

					double currentFaceZ = roundDoubleToPrecision(helperFunctions::getLowestZ(tesselatedFace), 1e-6);
					if (outHorizontalStoreyFaces.count(currentFaceZ) != 0)
					{
						outHorizontalStoreyFaces[currentFaceZ].emplace_back(tesselatedFace);
					}
					else
					{
						outHorizontalStoreyFaces[currentFaceZ] = { tesselatedFace };
					}
				}
			}
		}
	}
	return;
}

std::vector<TopoDS_Face> CJGeoCreator::TrimHStoreyFaces(const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<treeDepth_>>& horizontalFaceIndex)
{
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	std::vector<TopoDS_Face> outerShapeFaces;
	// split and filter out the repeating horizontal faces
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
		fuser.SetFuzzyValue(precisionCoarse);
		fuser.Build();

		for (TopExp_Explorer explorer(fuser.Shape(), TopAbs_FACE); explorer.More(); explorer.Next())
		{
			const TopoDS_Face& currentSplitFace = TopoDS::Face(explorer.Current());
			TopoDS_Face tesselatedFace = helperFunctions::TessellateFace(currentSplitFace);
			std::optional<gp_Pnt> currentCenterPoint = helperFunctions::getPointOnFace(tesselatedFace);

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
				outerShapeFaces.emplace_back(tesselatedFace);
			}
		}
	}
	return outerShapeFaces;
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
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& splittedFacesOut, 
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& untouchedFacesOut,
	const std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& outerSurfacePairList
)
{
	bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>> faceIndx;
	for (const auto& [currentFace, currentType] : outerSurfacePairList)
	{
		BoostBox3D currentBox = helperFunctions::createBBox(currentFace);
		faceIndx.insert(std::pair(currentBox, currentFace));
	}

	// split the range over cores
	int coreUse = SettingsCollection::getInstance().threadcount();
	if (coreUse > outerSurfacePairList.size())
	{
		while (coreUse > outerSurfacePairList.size()) { coreUse /= 2; }
	}
	int splitListSize = static_cast<int>(floor(outerSurfacePairList.size() / coreUse));

	std::mutex untouchedListMutex;
	std::mutex splittedListMutex;
	std::mutex processedCountMutex;
	std::vector<std::thread> threadList;

	int processedCount = 0;
	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = outerSurfacePairList.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? outerSurfacePairList.end() : startIdx + splitListSize;

		std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>> sublist(startIdx, endIdx);

		threadList.emplace_back([this, &splittedFacesOut, &splittedListMutex, &untouchedFacesOut, &untouchedListMutex, &processedCount, &processedCountMutex, &faceIndx, sublist]() {
			splitOuterSurfaces(splittedFacesOut, splittedListMutex, untouchedFacesOut, untouchedListMutex, processedCount, processedCountMutex, faceIndx, sublist);
		});
	}


	threadList.emplace_back([&] {updateCounter("Splitting outer surfaces", outerSurfacePairList.size(), processedCount, processedCountMutex);  });


	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}
	//TODO: process indicator
	return;
}

void CJGeoCreator::splitOuterSurfaces(
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& splittedFacesOut,
	std::mutex& splittedListMutex,
	std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& untouchedFacesOut,
	std::mutex& untouchedListMutex,
	int& totalObjectsProcessed,
	std::mutex& totalObjectsProcessedMutex,
	const bgi::rtree<std::pair<BoostBox3D, TopoDS_Face>, bgi::rstar<25>>& faceIndx,
	const std::vector<std::pair<TopoDS_Face, IfcSchema::IfcProduct*>>& outerSurfacePairList
)
{
	double precision = SettingsCollection::getInstance().precision();
	double precisionCoarse = SettingsCollection::getInstance().precisionCoarse();

	for (const auto& [currentFace, currentProduct] : outerSurfacePairList)
	{
		std::unique_lock<std::mutex> processCountLock(totalObjectsProcessedMutex);
		totalObjectsProcessed++;
		processCountLock.unlock();

		BOPAlgo_Splitter divider;
		divider.SetFuzzyValue(SettingsCollection::getInstance().precisionCoarse());
		divider.SetRunParallel(Standard_False);
		divider.AddArgument(currentFace);

		std::vector<std::pair<BoostBox3D, TopoDS_Face>> qResult;
		qResult.clear();
		faceIndx.query(bgi::intersects(helperFunctions::createBBox(currentFace)), std::back_inserter(qResult));

		gp_Vec currentNormal = helperFunctions::computeFaceNormal(currentFace);
		if (currentNormal.Magnitude() < precision) { continue; }

		int toolCount = 0;
		for (const auto& [otherBox, otherFace] : qResult)
		{
			if (currentFace.IsEqual(otherFace)) { continue; }

			gp_Vec otherNormal = helperFunctions::computeFaceNormal(otherFace);
			if (otherNormal.Magnitude() < precision) { continue; }

			if (currentNormal.IsParallel(otherNormal, precisionCoarse)) {
				std::optional<gp_Pnt> otherPointOpt = helperFunctions::getPointOnFace(otherFace);
				if (otherPointOpt == std::nullopt) { continue; }

				if (!helperFunctions::pointOnShape(currentFace, *otherPointOpt, precisionCoarse)) { continue; }
			}
			else
			{
				BRepExtrema_DistShapeShape distanceCalc(currentFace, otherFace);
				distanceCalc.Perform();
				if (distanceCalc.Value() > precisionCoarse) { continue; }
			}
			divider.AddTool(otherFace);
			toolCount++;
		}

		if (toolCount == 0)
		{
			helperFunctions::triangulateShape(currentFace);

			const std::lock_guard<std::mutex> lock(untouchedListMutex);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentProduct));
			continue;
		}

		divider.Perform();
		TopoDS_Shape splitFaceList = divider.Shape();
		if (splitFaceList.IsNull())
		{
			helperFunctions::triangulateShape(currentFace);

			const std::lock_guard<std::mutex> lock(untouchedListMutex);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentProduct));
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
			const std::lock_guard<std::mutex> lock(untouchedListMutex);
			untouchedFacesOut.emplace_back(std::pair(currentFace, currentProduct));
		}
		else
		{
			const std::lock_guard<std::mutex> lock(splittedListMutex);
			for (const TopoDS_Face face : faceList)
			{
				helperFunctions::triangulateShape(face);
				splittedFacesOut.emplace_back(std::pair(face, currentProduct));
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

		if (!helperFunctions::LineShapeIntersection(footprintFace, basePoint, bottomPoint))
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
