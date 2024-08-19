#include "DataManager.h"
#include "cjCreator.h"
#include "helper.h"
#include "voxel.h"
#include "stringManager.h"

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

void flipPoints(gp_Pnt* p1, gp_Pnt* p2) {
	gp_Pnt tempPoint = *p1;
	*p1 = *p2;
	*p2 = tempPoint;
	return;
}


std::vector<Edge> CJGeoCreator::getUniqueEdges(const TopoDS_Shape& flattenedEdges)
{
	std::vector<Edge> UniqueEdgeList;
	for (TopExp_Explorer expl(flattenedEdges, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		if (!isInList(currentEdge, UniqueEdgeList))
		{
			UniqueEdgeList.emplace_back(Edge(currentEdge));
		}
	}
	return UniqueEdgeList;
}


std::vector<Edge> CJGeoCreator::getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges)
{
	std::vector<Edge> UniqueEdgeList;
	for (size_t i = 0; i < flattenedEdges.size(); i++)
	{
		TopoDS_Edge currentEdge = flattenedEdges[i];
		if (!isInList(currentEdge, UniqueEdgeList))
		{
			UniqueEdgeList.emplace_back(Edge(currentEdge));
		}
	}
	return UniqueEdgeList;
}


std::vector<TopoDS_Face> CJGeoCreator::getUniqueFaces(const std::vector<TopoDS_Face>& faceList) {
	std::vector<TopoDS_Face> uniqueFaces;
	
	// get unique faces
	for (size_t i = 0; i < faceList.size(); i++)
	{
		bool dub = false;
		for (size_t j = 0; j < uniqueFaces.size(); j++)
		{
			if (j == i)
			{
				continue;
			}
			if (faceList[i].IsEqual(faceList[j]) ||
				faceList[i].IsEqual(faceList[j].Reversed()))
			{
				dub = true;
				break;
			}
		}

		if (!dub)
		{
			uniqueFaces.emplace_back(faceList[i]);
		}
	}
	return uniqueFaces;
}

std::vector<TopoDS_Face> CJGeoCreator::getEncompassedFaces(const std::vector<TopoDS_Face>& faceList)
{
	std::vector<TopoDS_Face> filteredFaces;
	double precision = SettingsCollection::getInstance().precision();

	for (size_t j = 0; j < faceList.size(); j++)
	{
		int vertCount = 0;
		int overlapCount = 0;

		for (TopExp_Explorer vertexExplorer(faceList[j], TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next()) {
			TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);

			bool hasOverlap = false;
			vertCount++;

			for (size_t k = 0; k < faceList.size(); k++)
			{
				if (j == k) { continue; }
				for (TopExp_Explorer otherVertexExplorer(faceList[k], TopAbs_VERTEX); otherVertexExplorer.More(); otherVertexExplorer.Next()) {
					TopoDS_Vertex otherVertex = TopoDS::Vertex(otherVertexExplorer.Current());
					gp_Pnt otherPoint = BRep_Tool::Pnt(otherVertex);

					if (point.IsEqual(otherPoint, precision))
					{
						overlapCount++;
						hasOverlap = true;
						break;
					}
				}
				if (hasOverlap)
				{
					break;
				}
			}
		}
		if (overlapCount == vertCount)
		{
			filteredFaces.emplace_back(faceList[j]);
		}
	}
	return filteredFaces;
}


bool CJGeoCreator::isInList(const TopoDS_Edge& currentEdge, const std::vector<Edge>& edgeList)
{
	double precision = SettingsCollection::getInstance().precision();

	gp_Pnt startPoint = helperFunctions::getFirstPointShape(currentEdge);
	gp_Pnt endPoint = helperFunctions::getLastPointShape(currentEdge);

	bool dub = false;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		Edge currentEdge = edgeList[i];
		gp_Pnt otherStartPoint = currentEdge.getStart(false);
		gp_Pnt otherEndPoint = currentEdge.getEnd(false);

		if (startPoint.IsEqual(otherStartPoint, precision) && endPoint.IsEqual(otherEndPoint, precision) ||
			endPoint.IsEqual(otherStartPoint, precision) && startPoint.IsEqual(otherEndPoint, precision))
		{
			return true;
		}
	}
	return false;
}


std::vector<Edge> CJGeoCreator::mergeOverlappingEdges(const std::vector<Edge>& uniqueEdges, bool project)
{
	double buffer = 0.001;

	// merge lines that are on the same plane
	std::vector<Edge> cleanedEdgeList;
	std::vector<int> evalList(uniqueEdges.size());

	std::vector<int> discardIndx;
	for (int i = 0; i < uniqueEdges.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		Edge currentEdge = uniqueEdges[i];
		gp_Pnt startPoint = currentEdge.getStart(project);
		gp_Pnt2d startPoint2d = gp_Pnt2d(startPoint.X(), startPoint.Y());
		gp_Pnt endPoint = currentEdge.getEnd(project);
		gp_Pnt2d endPoint2d = gp_Pnt2d(endPoint.X(), endPoint.Y());

		gp_Vec2d currentDir(startPoint2d, endPoint2d);
		currentDir.Normalize();

		// find edges that are located on the same line as the eval line
		std::vector<gp_Pnt> mergeList;
		std::vector<int> evalIndxList;

		for (int j = 0; j < uniqueEdges.size(); j++)
		{
			if (i == j) { continue; }
			if (evalList[j] == 1) { continue; }

			Edge otherEdge = uniqueEdges[j];
			gp_Pnt otherStartPoint = otherEdge.getStart(project);
			gp_Pnt2d otherStartPoint2d = gp_Pnt2d(otherStartPoint.X(), otherStartPoint.Y());
			gp_Pnt otherEndPoint = otherEdge.getEnd(project);
			gp_Pnt2d otherEndPoint2d = gp_Pnt2d(otherEndPoint.X(), otherEndPoint.Y());

			gp_Vec2d otherDir(otherStartPoint2d, otherEndPoint2d);
			otherDir.Normalize();

			if (!currentDir.IsParallel(otherDir, 0.0001)) { continue; }

			if (!startPoint.IsEqual(otherStartPoint, 0.0001))
			{
				if (!currentDir.IsParallel(
					gp_Vec2d( startPoint2d, otherStartPoint2d ),
					0.0001
				))
				{
					continue;
				}
			}
			evalIndxList.emplace_back(j);
		}

		// if no edges on the same line were found
		if (evalIndxList.size() == 0)
		{
			evalList[i] = 1;
			if (startPoint.Distance(endPoint) > 0.001) //TODO: make smarter
			{
				cleanedEdgeList.emplace_back(uniqueEdges[i]);
			}
			continue;
		}

		discardIndx.emplace_back(i);

		// flip main merging edge
		if (abs(currentDir.X()) >= abs(currentDir.Y()) &&
			startPoint.X() > endPoint.X())
		{
			flipPoints(&startPoint, &endPoint);
		}
		if (abs(currentDir.X()) < abs(currentDir.Y()) &&
			startPoint.Y() > endPoint.Y())
		{
			flipPoints(&startPoint, &endPoint);
		}
		while (true)
		{
			int grow = 0;
			for (size_t j = 0; j < evalIndxList.size(); j++)
			{
				int edgeIdx = evalIndxList[j];

				if (evalList[edgeIdx] == 1) { continue; }

				Edge otherEdge = uniqueEdges[edgeIdx];
				gp_Pnt otherStartPoint = otherEdge.getStart(project);
				gp_Pnt otherEndPoint = otherEdge.getEnd(project);

				if (abs(currentDir.X()) >= abs(currentDir.Y()) &&
					otherStartPoint.X() > otherEndPoint.X())
				{
					flipPoints(&otherStartPoint, &otherEndPoint);
				}
				if (abs(currentDir.X()) < abs(currentDir.Y()) &&
					otherStartPoint.Y() > otherEndPoint.Y())
				{
					flipPoints(&otherStartPoint, &otherEndPoint);
				}

				double x1 = startPoint.X();
				double x2 = endPoint.X();
				double x3 = otherStartPoint.X();
				double x4 = otherEndPoint.X();

				if (abs(currentDir.X()) < abs(currentDir.Y()))
				{
					x1 = startPoint.Y();
					x2 = endPoint.Y();
					x3 = otherStartPoint.Y();
					x4 = otherEndPoint.Y();
				}

				if (x3 - buffer <= x1 && x2 <= x4 + buffer)
				{
					evalList[edgeIdx] = 1;
					startPoint = otherStartPoint;
					endPoint = otherEndPoint;
					grow++;
					continue;
				}

				if (!( //No overlap
					x1 - buffer <= x3 && x3 <= x2 + buffer ||
					x1 - buffer <= x4 && x4 <= x2 + buffer)
					)
				{
					continue;
				}

				if (x1 - buffer <= x3 && x4 <= x2 + buffer)
				{
					evalList[edgeIdx] = 1;
					continue;
				}

				if (x2 - buffer < x3 && x2 + buffer > x3)
				{
					evalList[edgeIdx] = 1;
					endPoint = otherEndPoint;
					grow++;
					continue;
				}

				if (x4 - buffer < x1 && x4 + buffer > x1)
				{
					evalList[edgeIdx] = 1;
					startPoint = otherStartPoint;
					grow++;
					continue;
				}

				if (x1 - buffer <= x3 && x3 <= x2 + buffer &&
					x1 - buffer <= x4 && x2 <= x4 + buffer)
				{
					evalList[edgeIdx] = 1;
					endPoint = otherEndPoint;
					grow++;
					continue;
				}

				if (x1 - buffer <= x4 && x4 <= x2 + buffer &&
					x3 - buffer < x1 && x3 < x2 + buffer)
				{
					evalList[edgeIdx] = 1;
					startPoint = otherStartPoint;
					grow++;
					continue;
				}
			}
			if (grow == 0)
			{
				break;
			}
			grow = 0;
		}
		evalIndxList.clear();

		TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(startPoint, endPoint);
		cleanedEdgeList.emplace_back(Edge(edge));
	}
	return cleanedEdgeList;
}


std::vector<Edge> CJGeoCreator::splitIntersectingEdges(const std::vector<Edge>& edges, bool project) {
	
	std::vector<Edge> splitEdgeList;
	std::vector<int> discardIndx;
	
	for (int i = 0; i < edges.size(); i++)
	{
		std::vector<gp_Pnt> intPoints;

		Edge currentEdge = edges[i];
		gp_Pnt startPoint = currentEdge.getStart(project);
		gp_Pnt endPoint = currentEdge.getEnd(project);

		for (size_t j = 0; j < edges.size(); j++)
		{
			if (i == j) { continue; }
			std::optional<gp_Pnt> intersection = helperFunctions::linearLineIntersection(currentEdge, edges[j], project);
			if (intersection == std::nullopt) { continue; }
			intPoints.emplace_back(*intersection);;
		}
		if (intPoints.size() == 0) 
		{ 
			splitEdgeList.emplace_back(currentEdge);
			continue;
		}
		discardIndx.emplace_back(i);

		std::vector<gp_Pnt> cleanedPoints = helperFunctions::getUniquePoints(intPoints);

		std::vector<int> evaluated(cleanedPoints.size());
		gp_Pnt currentPoint = startPoint;
		while (true)
		{
			double smallestDistance = 99999999;
			int nextPoint;
			bool found = false;
			for (int j = 0; j < cleanedPoints.size(); j++)
			{
				if (evaluated[j] == 1) { continue; }

				double distance = currentPoint.Distance(cleanedPoints[j]);
				if (distance < smallestDistance)
				{
					smallestDistance = distance;
					nextPoint = j;
					found = true;
				}
			}

			if (found)
			{
				evaluated[nextPoint] = 1;

				if (currentPoint.IsEqual(cleanedPoints[nextPoint], 0.0001))
				{
					currentPoint = cleanedPoints[nextPoint];
					continue;
				}

				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentPoint, cleanedPoints[nextPoint]);
				splitEdgeList.emplace_back(Edge(edge));
				currentPoint = cleanedPoints[nextPoint];
			}
			else {
				if (currentPoint.IsEqual(endPoint, 0.0001)) { break; }
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentPoint, endPoint);
				splitEdgeList.emplace_back(Edge(edge));
				break;
			}
		}
	}
	return splitEdgeList;
}


std::vector<TopoDS_Face> CJGeoCreator::simplefyProjection(const std::vector<TopoDS_Face> inputFaceList) {

	if (inputFaceList.size() == 1) { return inputFaceList; }

	std::vector<TopoDS_Face> outputFaceList;

	double precision = SettingsCollection::getInstance().precision();

	// split all the faces
	BOPAlgo_Builder aBuilder;
	aBuilder.SetFuzzyValue(precision);
	aBuilder.SetRunParallel(Standard_True);

	for (TopoDS_Face roofFace : inputFaceList)
	{
		aBuilder.AddArgument(roofFace);
	}
	aBuilder.Perform();
	if (aBuilder.HasErrors()) { return {}; }

	// index faceList
	bgi::rtree<Value, bgi::rstar<25>> faceIndex;
	std::vector<TopoDS_Face> faceList;
	std::vector<bg::model::box <BoostPoint3D>> boxList;
	for (TopExp_Explorer expl(aBuilder.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(face);
		faceIndex.insert(std::make_pair(bbox, (int)faceList.size()));
		boxList.emplace_back(bbox);
		faceList.emplace_back(face);
	}

	//find unique edges
	std::vector<TopoDS_Edge> edgeList;
	for (size_t i = 0; i < faceList.size(); i++)
	{
		TopoDS_Face currentFace = faceList[i];
		bg::model::box <BoostPoint3D> currentBox = boxList[i];

		std::vector<Value> qResult;
		qResult.clear();
		faceIndex.query(bgi::intersects(
			currentBox), std::back_inserter(qResult));

		for (TopExp_Explorer currentEdgeExp(currentFace, TopAbs_EDGE); currentEdgeExp.More(); currentEdgeExp.Next()) {
			TopoDS_Edge currentEdge = TopoDS::Edge(currentEdgeExp.Current());
			gp_Pnt currentP1 = helperFunctions::getFirstPointShape(currentEdge);
			gp_Pnt currentP2 = helperFunctions::getLastPointShape(currentEdge);

			bool isFound = false;

			for (size_t j = 0; j < qResult.size(); j++)
			{
				int otherInt = qResult[j].second;

				if (otherInt == i) { continue; }

				for (TopExp_Explorer otherEdgeExp(faceList[otherInt], TopAbs_EDGE); otherEdgeExp.More(); otherEdgeExp.Next()) {
					TopoDS_Edge otherEdge = TopoDS::Edge(otherEdgeExp.Current());
					gp_Pnt otherP1 = helperFunctions::getFirstPointShape(otherEdge);
					gp_Pnt otherP2 = helperFunctions::getLastPointShape(otherEdge);

					if (currentP1.IsEqual(otherP1, precision) && currentP2.IsEqual(otherP2, precision) ||
						currentP1.IsEqual(otherP2, precision) && currentP2.IsEqual(otherP1, precision))
					{
						isFound = true;
						break;
					}
				}
				if (isFound) { break; }
			}
			if (isFound) { continue; }
			edgeList.emplace_back(currentEdge);
		}
	}
	std::vector<TopoDS_Wire> wireList = growWires(edgeList);
	std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
	std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);
	for (TopoDS_Face currentFace : cleanedFaceList) { outputFaceList.emplace_back(currentFace); }
	return outputFaceList;
}


bool CJGeoCreator::isOuterEdge(Edge currentEdge, const std::vector<TopoDS_Face>& flatFaceList, const bgi::rtree<Value, bgi::rstar<treeDepth_>>& spatialIndex)
{
	gp_Pnt startPoint = currentEdge.getStart(true);
	gp_Pnt endPoint = currentEdge.getEnd(true);

	if (startPoint.IsEqual(endPoint, 0.0001))
	{
		return false;
	}

	gp_Pnt2d middlePoint = gp_Pnt2d(
		(endPoint.X() + startPoint.X()) / 2,
		(endPoint.Y() + startPoint.Y()) / 2
	);

	gp_Vec2d lineVec = gp_Vec2d(
		gp_Pnt2d(startPoint.X(), startPoint.Y()),
		gp_Pnt2d(endPoint.X(), endPoint.Y())
	);
	lineVec.Normalize();

	double div = 1000; // TODO: get set distance working
	gp_Pnt2d evalPoint1 = middlePoint.Translated(gp_Vec2d(lineVec.Y() * -1 / div, lineVec.X() / div));
	gp_Pnt2d evalPoint2 = middlePoint.Translated(gp_Vec2d(lineVec.Y() / div, lineVec.X() * -1 / div));

	TopoDS_Edge evalEdge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(evalPoint1.X(), evalPoint1.Y(), 0), gp_Pnt(evalPoint1.X(), evalPoint1.Y() + 10000, 0));
	TopoDS_Edge evalEdge2 = BRepBuilderAPI_MakeEdge(gp_Pnt(evalPoint2.X(), evalPoint2.Y(), 0), gp_Pnt(evalPoint2.X(), evalPoint2.Y() + 10000, 0));

	bool hasEval1 = false;
	bool hasEval2 = false;

	// querry spatialIndex
	gp_Pnt lll(
		std::min(startPoint.X(), endPoint.X()),
		std::min(startPoint.Y(), endPoint.Y()),
		std::min(startPoint.Z(), endPoint.Z())
	);

	gp_Pnt urr(
		std::max(startPoint.X(), endPoint.X()),
		std::max(startPoint.Y(), endPoint.Y()),
		std::max(startPoint.Z(), endPoint.Z())
	);

	std::vector<Value> qResult;
	bg::model::box<BoostPoint3D> boostbbox({ lll.X() - 0.5, lll.Y() -0.5, -1 }, { urr.X() + 0.5, urr.Y() + 0.5, 1 });
	spatialIndex.query(bgi::intersects(boostbbox), std::back_inserter(qResult));

	double precision = SettingsCollection::getInstance().precision();

	for (size_t i = 0; i < qResult.size(); i++)
	{
		TopoDS_Face evalFace = flatFaceList[qResult[i].second];
		int intersectionCount1 = 0;
		int intersectionCount2 = 0;

		for (TopExp_Explorer explorer(evalFace, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			
			if (helperFunctions::linearLineIntersection(evalEdge1, edge, false, precision) == std::nullopt) { intersectionCount1++; }
			if (helperFunctions::linearLineIntersection(evalEdge2, edge, false, precision) == std::nullopt) { intersectionCount2++; }
		}

		if (intersectionCount1 % 2 == 1) { hasEval1 = true; }
		if (intersectionCount2 % 2 == 1) { hasEval2 = true; }

		if (hasEval1 && hasEval2)
		{
			break;
		}
	}

	if (hasEval1 && !hasEval2 || !hasEval1 && hasEval2)
	{
		return true;
	}
	return false;
}


std::vector<TopoDS_Edge> CJGeoCreator::getOuterEdges(const std::vector<Edge>& edgeList, const std::vector<SurfaceGroup>& faceList) {
	// project and make indexing
	std::vector<TopoDS_Face> evalSurfList;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		TopoDS_Face currentFace = faceList[i].getProjectedFace();
		spatialIndex.insert(std::make_pair(helperFunctions::createBBox(currentFace), (int) i));
		evalSurfList.emplace_back(currentFace);
	}

	std::vector<TopoDS_Edge> outerEdgeList;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		Edge edgeCol = edgeList[i];
		TopoDS_Edge currentEdge = edgeCol.getEdge();
		if (isOuterEdge(edgeList[i], evalSurfList, spatialIndex)) { outerEdgeList.emplace_back(currentEdge);
		}
	}
	return outerEdgeList;
}

std::vector<TopoDS_Edge> CJGeoCreator::getOuterEdges(
	const std::vector<Edge>& edgeList, 
	const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex,
	const std::vector<std::shared_ptr<voxel>>& originVoxels,
	double floorlvl
) {

	//create index of the splitEdges
	bgi::rtree<Value, bgi::rstar<25>> edgeIndex;
	for (int i = 0; i < edgeList.size(); i++)
	{
		Edge currentEdge = edgeList[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentEdge.getStart(false), currentEdge.getEnd(false));
		edgeIndex.insert(std::make_pair(bbox, i));
	}

	// raycast
	double distance = 2 * SettingsCollection::getInstance().voxelSize();
	double precision = SettingsCollection::getInstance().precision();

	std::vector<TopoDS_Edge> outerFootPrintList;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		bool isExterior = false;

		Edge currentEdge = edgeList[i];
		gp_Pnt startPoint = currentEdge.getStart(false);
		gp_Pnt endPoint = currentEdge.getEnd(false);

		gp_Pnt centerPoint = gp_Pnt(
			(startPoint.X() + endPoint.X()) / 2,
			(startPoint.Y() + endPoint.Y()) / 2,
			(startPoint.Z() + endPoint.Z()) / 2
		);
		// querry voxels

		std::vector<Value> qResult;
		qResult.clear();
		voxelIndex.query(bgi::intersects(
			bg::model::box <BoostPoint3D>(
				BoostPoint3D(centerPoint.X() - distance, centerPoint.Y() - distance, centerPoint.Z() - distance),
				BoostPoint3D(centerPoint.X() + distance, centerPoint.Y() + distance, centerPoint.Z() + distance)
				)), std::back_inserter(qResult));

		for (size_t j = 0; j < qResult.size(); j++)
		{
			bool rayInersects = false;
			// construct ray between voxels and centerpoint of line
			voxel currentBoxel = *originVoxels[qResult[j].second];

			if (currentBoxel.getIsInside()) { continue; }
			BoostPoint3D boostCastPoint = currentBoxel.getCenterPoint();
			gp_Pnt castPoint(boostCastPoint.get<0>(), boostCastPoint.get<1>(), floorlvl);
			TopoDS_Edge castRay = BRepBuilderAPI_MakeEdge(centerPoint, castPoint);

			std::vector<Value> qResult2;
			qResult2.clear();
			edgeIndex.query(bgi::intersects(
				bg::model::box <BoostPoint3D>(
					helperFunctions::createBBox(centerPoint, castPoint)
					)), std::back_inserter(qResult2));

			for (size_t k = 0; k < qResult2.size(); k++)
			{

				if (qResult2[k].second == i)
				{
					continue;
				}
				Edge evalEdge = edgeList[qResult2[k].second];
				BRepExtrema_DistShapeShape distanceCalc(evalEdge.getEdge(), castRay);
				distanceCalc.Perform();

				if (!distanceCalc.IsDone())
				{
					continue;
				}

				if (distanceCalc.Value() < precision)
				{
					rayInersects = true;
					break;
				}
			}

			if (!rayInersects)
			{
				isExterior = true;
				break;
			}
		}

		if (isExterior)
		{
			outerFootPrintList.emplace_back(currentEdge.getEdge());
		}
	}	
	return outerFootPrintList;
}


std::vector<TopoDS_Face> CJGeoCreator::outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList)
{
	std::vector<TopoDS_Wire> wireList = growWires(edgeList);
	std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);	
	std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);
	return cleanedFaceList;
}


std::vector<TopoDS_Wire> CJGeoCreator::growWires(const std::vector<TopoDS_Edge>& edgeList){
	std::vector<TopoDS_Wire> wireCollection;
	std::vector<TopoDS_Wire> wireCollectionClosed;
	std::vector<TopoDS_Edge> tempEdgeList;

	//BRepBuilderAPI_MakeWire wireMaker;
	bool loopFound = false;

	TopoDS_Edge currentEdge = edgeList[0];
	std::vector<int> evaluated(edgeList.size());
	evaluated[0] = 1;
	
	gp_Pnt originPoint = helperFunctions::getFirstPointShape(edgeList[0]); // the original point of the original edge
	gp_Pnt extendingPoint = helperFunctions::getLastPointShape(edgeList[0]); // the point from which will be extended

	tempEdgeList.emplace_back(currentEdge);

	double precision = SettingsCollection::getInstance().precision();
	bool isReversed = false;
	while (true)
	{
		bool hasStapped = false; // true if a stap is found in the while iteration
		bool closed = false; // true if the extensionpoint meets the originpoint

		for (size_t i = 0; i < edgeList.size(); i++)
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges
			TopoDS_Edge otherEdge = edgeList[i];

			gp_Pnt p1 = helperFunctions::getFirstPointShape(otherEdge);
			gp_Pnt p2 = helperFunctions::getLastPointShape(otherEdge);

			if (p1.IsEqual(extendingPoint, precision)) // check if edge is neighbour
			{
				extendingPoint = p2;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				else { tempEdgeList.emplace_back(otherEdge); }
				break;
			}
			else if (p2.IsEqual(extendingPoint, precision)) // check if reversed edge is neighbour 
			{
				extendingPoint = p1;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), otherEdge); }
				else { tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				break;
			}
			else if (extendingPoint.IsEqual(originPoint, precision)) // check if a closed loop is found if no new neighbour is there
			{
				closed = true;
				break;
			}
		}

		if (hasStapped) { continue; } // if step is taken, try to make a next step

		if (!closed && !isReversed) // reverse the search and contine in the loop
		{
			gp_Pnt tempPoint = extendingPoint;
			extendingPoint = originPoint;
			originPoint = tempPoint;
			isReversed = true;
			continue;
		}

		BRepBuilderAPI_MakeWire wireMaker;
		for (size_t i = 0; i < tempEdgeList.size(); i++) { wireMaker.Add(tempEdgeList[i]); }
		tempEdgeList.clear();
		bool newRingStarted = false;
		wireMaker.Build();

		if (wireMaker.IsDone())
		{
			TopoDS_Wire wire = wireMaker.Wire();

			if (wire.Closed()) { wireCollectionClosed.emplace_back(wire); }
			else { wireCollection.emplace_back(wireMaker.Wire()); }
			wireMaker = BRepBuilderAPI_MakeWire();
		}


		for (size_t i = 0; i < edgeList.size(); i++) // search next unused edge to create new wire
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges

			originPoint = helperFunctions::getFirstPointShape(edgeList[i]); // the original point of the original edge
			extendingPoint = helperFunctions::getLastPointShape(edgeList[i]); // the point from which will be extended
			tempEdgeList.emplace_back(edgeList[i]);

			evaluated[i] = 1;
			isReversed = false;
			newRingStarted = true;
			break;
		}

		if (!newRingStarted)
		{
			break;
		}
	}

	if (wireCollection.size() == 0) { return wireCollectionClosed; }

	BRepBuilderAPI_MakeWire wireMaker = BRepBuilderAPI_MakeWire();
	TopoDS_Wire currentWire = wireCollection[0];
	wireCollection.erase(wireCollection.begin());

	double maxWireDistance = 1.5;

	int currentWireIdx = 0;
	while (true) // merge the openWires
	{
		bool stepped = false;

		double distance = 99999999999;
		int idxMatch = -1;
		TopoDS_Edge connectionEdge;

		gp_Pnt startpoint = helperFunctions::getFirstPointShape(currentWire);
		gp_Pnt endpoint = helperFunctions::getLastPointShape(currentWire);

		for (int i = 0; i < wireCollection.size(); i++) 
		{
			TopoDS_Wire otherwire = wireCollection[i];
			gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(otherwire);
			gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(otherwire);

			double d1 = startpoint.Distance(otherStartpoint);
			double d2 = startpoint.Distance(otherEndpoint);
			double d3 = endpoint.Distance(otherStartpoint);
			double d4 = endpoint.Distance(otherEndpoint);

			if (d1 < maxWireDistance && d1 < distance)
			{
				idxMatch = i;
				distance = d1;
				connectionEdge = BRepBuilderAPI_MakeEdge(startpoint, otherStartpoint);
			}
			if (d2 < maxWireDistance && d2 < distance)
			{
				idxMatch = i;
				distance = d2;
				connectionEdge = BRepBuilderAPI_MakeEdge(startpoint, otherEndpoint);
			}
			if (d3 < maxWireDistance && d3 < distance)
			{
				idxMatch = i;
				distance = d3;
				connectionEdge = BRepBuilderAPI_MakeEdge(endpoint, otherStartpoint);
			}
			if (d4 < maxWireDistance && d4 < distance)
			{
				idxMatch = i;
				distance = d4;
				connectionEdge = BRepBuilderAPI_MakeEdge(endpoint, otherEndpoint);
			}
		}

		if (idxMatch!= -1)
		{
			currentWire = helperFunctions::mergeWireOrientated(currentWire, BRepBuilderAPI_MakeWire(connectionEdge));
			currentWire = helperFunctions::mergeWireOrientated(currentWire, wireCollection[idxMatch]);
			wireCollection.erase(wireCollection.begin() + idxMatch);
			stepped = true;
		}

		if (!stepped)
		{
			wireCollectionClosed.emplace_back(helperFunctions::closeWireOrientated(currentWire));
			if (wireCollection.size() == 0) { break; }

			currentWireIdx++;
			currentWire = wireCollection[0];
			wireCollection.erase(wireCollection.begin());
		}
	}
	return wireCollectionClosed;
}


std::vector<TopoDS_Wire> CJGeoCreator::cleanWires(const std::vector<TopoDS_Wire>& wireList) {

	std::vector<TopoDS_Wire> cleanedWires;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		cleanedWires.emplace_back(cleanWire(wireList[i]));
	}
	return cleanedWires;
}


TopoDS_Wire CJGeoCreator::cleanWire(const TopoDS_Wire& wire) {
	
	BRepBuilderAPI_MakeWire wireMaker;
	std::vector<TopoDS_Edge> orderedEdgeList;
	for (TopExp_Explorer edgeExp(wire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
		orderedEdgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
	}

	std::vector<int> merged(orderedEdgeList.size());

	double precision = SettingsCollection::getInstance().precision();

	bool ttt = false;
	gp_Pnt connection = helperFunctions::getFirstPointShape(orderedEdgeList[0]);
	gp_Vec startingVec = helperFunctions::getDirEdge(orderedEdgeList[0]);
	if (startingVec.IsParallel(helperFunctions::getDirEdge(orderedEdgeList.back()), precision)) //get another startpoint
	{
		ttt = true;
		for (size_t i = 1; i < orderedEdgeList.size(); i++)
		{
			if (!startingVec.IsParallel(helperFunctions::getDirEdge(orderedEdgeList[i]), precision))
			{
				std::rotate(orderedEdgeList.begin(), orderedEdgeList.begin() + i, orderedEdgeList.end());
				break;
			}
		}
	}
	connection = helperFunctions::getFirstPointShape(orderedEdgeList[0]);

	for (size_t i = 0; i < orderedEdgeList.size(); i++) // merge parralel 
	{
		if (merged[i] == 1) { continue; }
		merged[i] = 1;

		TopoDS_Edge currentEdge = orderedEdgeList[i];
		gp_Vec currentVec = helperFunctions::getDirEdge(currentEdge);

		if (currentVec.Magnitude() == 0)
		{
			merged[i] = 1;
			continue;
		}

		gp_Pnt endPoint = helperFunctions::getLastPointShape(currentEdge);

		for (size_t j = i + 1; j < orderedEdgeList.size(); j++)
		{
			if (merged[j] == 1) { continue; }

			TopoDS_Edge otherEdge = orderedEdgeList[j];
			gp_Vec otherVec = helperFunctions::getDirEdge(otherEdge);

			if (otherVec.Magnitude() == 0)
			{
				merged[j] = 1;
				continue; 
			}

			if (currentVec.IsParallel(otherVec, 0.01)) { 
				merged[j] = 1;
				continue; 
			}

			if (j + 1 == orderedEdgeList.size())
			{
				endPoint = helperFunctions::getFirstPointShape(otherEdge);
				break;
			}

			endPoint = helperFunctions::getFirstPointShape(otherEdge);

			gp_Vec endPointVec = gp_Vec(connection, endPoint);

			if (endPointVec.Magnitude() == 0) { break; }
			else if (!currentVec.IsParallel(endPointVec, 0.01)){endPoint = helperFunctions::getLastPointShape(otherEdge);}

			break;
		}

		if (connection.IsEqual(endPoint, precision)) { continue; }

		wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));
		connection = endPoint;
	}

	if (connection.IsEqual(helperFunctions::getFirstPointShape(orderedEdgeList[0]), precision))
	{
		return wireMaker.Wire();
	}

	gp_Pnt finalPoint = helperFunctions::getFirstPointShape(orderedEdgeList[0]);


	wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, finalPoint));
	return wireMaker.Wire();
}


std::vector<TopoDS_Face> CJGeoCreator::wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList) {
	BRepBuilderAPI_MakeFace faceBuilder;
	std::vector<TopoDS_Face> faceList;
	gp_Vec normal;
	gp_Pnt originPoint;
	gp_Vec castingVector;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		if (i == 0) // use the first face to compute normal dir 
		{
			gp_Vec vec1;
			gp_Vec vec2;
			for (TopExp_Explorer vertexExp(wireList[i], TopAbs_EDGE); vertexExp.More(); vertexExp.Next()) {
				TopoDS_Edge edge = TopoDS::Edge(vertexExp.Current());

				originPoint = helperFunctions::getFirstPointShape(edge);
				gp_Pnt endpoint = helperFunctions::getLastPointShape(edge);

				vec1 = gp_Vec(originPoint, endpoint);
				vec2 = gp_Vec(originPoint, endpoint);
				while (vec1.IsParallel(vec2, 0.0001))
				{
					vertexExp.Next();
					edge = TopoDS::Edge(vertexExp.Current());

					gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(edge);
					gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(edge);
					vec2 = gp_Vec(otherStartpoint, otherEndpoint);
				}
				break;
			}
			normal = vec1.Crossed(vec2);
			normal.Normalize();
			castingVector = vec1;
		}
		faceBuilder = BRepBuilderAPI_MakeFace(
			gp_Pln(originPoint, normal),
			wireList[i]
		);

		if (faceBuilder.Error() == BRepBuilderAPI_FaceDone) { faceList.emplace_back(faceBuilder.Face()); }
	}
	if (faceList.size() == 1) { return faceList; }

	// test which surfaces are inner loops
	std::vector<double> areaList;
	std::vector<TopoDS_Face> correctedFaceList;
	for (size_t i = 0; i < faceList.size(); i++)
	{
		TopoDS_Face currentFootprint = faceList[i];

		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(currentFootprint, gprops); // Stores results in gprops
		double area = gprops.Mass(); 

		if (area < 0.001) { continue; }

		areaList.emplace_back(area);
		correctedFaceList.emplace_back(faceList[i]);
	}

	std::vector<TopoDS_Face> orderedFootprintList;
	std::vector<double> orderedAreaList;
	std::vector<int> ordered(areaList.size());
	for (size_t i = 0; i < areaList.size(); i++)
	{
		double evalArea = 0;
		int evalIdx = -1;
		for (int j = 0; j < areaList.size(); j++)
		{
			if (ordered[j] == 1) { continue; }

			if (evalArea < areaList[j])
			{
				evalArea = areaList[j];
				evalIdx = j;
			}
		}

		orderedFootprintList.emplace_back(correctedFaceList[evalIdx]);
		orderedAreaList.emplace_back(areaList[evalIdx]);
		ordered[evalIdx] = 1;
	}

	std::vector<int> clipped(areaList.size());
	std::vector<TopoDS_Face> cleanedFaceList;
	double precision = SettingsCollection::getInstance().precision();

	for (size_t i = 0; i < orderedFootprintList.size(); i++)
	{
		if (clipped[i] == 1) { continue; }
		clipped[i] = 1;

		TopoDS_Face clippedFace = orderedFootprintList[i];

		for (size_t j = i + 1; j < orderedFootprintList.size(); j++)
		{
			if (clipped[j] == 1) { continue; }

			TopExp_Explorer expl;
			for (expl.Init(orderedFootprintList[j], TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);

				TopoDS_Edge evalEdge = BRepBuilderAPI_MakeEdge(
					p, gp_Pnt(
						p.X() + abs(castingVector.X()) * 10000,
						p.Y() + abs(castingVector.Y()) * 10000,
						p.Z() + abs(castingVector.Z()) * 10000
					));

				int intersectionCount = 0;
				TopExp_Explorer edgeExplorer(orderedFootprintList[i], TopAbs_EDGE);
				for (; edgeExplorer.More(); edgeExplorer.Next()) {
					const TopoDS_Edge& currentEdge = TopoDS::Edge(edgeExplorer.Current());

					BRepExtrema_DistShapeShape distanceCalc1(evalEdge, currentEdge); //TODO: speed up with linear intersection function?
					distanceCalc1.Perform();

					if (distanceCalc1.Value() < precision) { intersectionCount++; }
				}
				if (intersectionCount % 2 == 1)
				{
					for (expl.Init(orderedFootprintList[j], TopAbs_WIRE); expl.More(); expl.Next())
					{
						TopoDS_Wire voidWire = TopoDS::Wire(expl.Current());
						if (orderedAreaList[j] > 0) { voidWire = TopoDS::Wire(expl.Current().Reversed()); }
						BRepBuilderAPI_MakeFace merger = BRepBuilderAPI_MakeFace(clippedFace, voidWire);
						clippedFace = merger.Face();
						break;
					}
					clipped[j] = 1;
				}
				break;
			}
		}
		cleanedFaceList.emplace_back(clippedFace);
	}
	return cleanedFaceList;
}


void CJGeoCreator::sortRoofStructures() {
	std::vector<SurfaceGroup> tempSurfaceGroup = faceList_[0];
	faceList_.clear();

	for (size_t i = 0; i < roofOutlineList_.size(); i++) {
		faceList_.emplace_back(std::vector<SurfaceGroup>());
	}

	TopExp_Explorer expl;
	for (size_t i = 0; i < tempSurfaceGroup.size(); i++)
	{
		bool found = false;
		for (expl.Init(tempSurfaceGroup[i].getProjectedFace(), TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			for (size_t j = 0; j < roofOutlineList_.size(); j++)
			{
				BRepExtrema_DistShapeShape distanceCalc(roofOutlineList_[j], vertex);

				distanceCalc.Perform();
				double distance = distanceCalc.Value();

				if (distance < 0.001)
				{
					faceList_[j].emplace_back(tempSurfaceGroup[i]);
					found = true;
					break;
				}

			}
			if (found)
			{
				break;
			}
		}
	}
}

void CJGeoCreator::mergeRoofSurfaces()
{
	double lowPrecision = SettingsCollection::getInstance().precisionCoarse();

	std::vector <std::vector<SurfaceGroup>> mergedFaceListBuilding;

	for (auto buildingFaceIt = faceList_.begin(); buildingFaceIt != faceList_.end(); ++buildingFaceIt)
	{
		// loop per sub building
		std::vector<SurfaceGroup > mergedFaceList;
		std::vector<SurfaceGroup> faceList = *buildingFaceIt;

		// compute the face normal
		std::vector<gp_Vec> normalList = {};
		for (auto faceIt = faceList.begin(); faceIt != faceList.end(); ++faceIt)
		{
			SurfaceGroup currentFace = *faceIt;
			normalList.emplace_back(helperFunctions::computeFaceNormal(currentFace.getFaces()[0]));
		}

		bool isDub = false;
		// check if any face normal is equal to another
		for (size_t i = 0; i < normalList.size(); ++i) {
			for (size_t j = i + 1; j < normalList.size(); ++j) {
				if (normalList[i].IsParallel(normalList[j], 1.0e-4)) {
					isDub = true;
					break;
				}
			}
			if (isDub)
			{
				break;
			}
		}
		if (!isDub) 
		{
			mergedFaceListBuilding.emplace_back(faceList);
			continue; 
		}

		// make index
		bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;
		std::vector<bg::model::box <BoostPoint3D>> bboxList;
		for (size_t i = 0; i < faceList.size(); i++)
		{
			TopoDS_Face currentFace = faceList[i].getProjectedFace();
			bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
			bboxList.emplace_back(bbox);
			spatialIndex.insert(std::make_pair(bbox, (int)i));
		}

		// find mergable faces
		std::vector<int> faceGroupList(faceList.size(), -1);
		std::vector<int> bufferList = {0};
		faceGroupList[0] = 1;
		gp_Vec currentNormal = normalList[0];

		std::vector<TopoDS_Face> neighbourFaceList;

		BOPAlgo_Builder aBuilder;
		while (true)
		{
			std::vector<int> tempBuffer = {};
			for (auto it = bufferList.begin(); it != bufferList.end(); ++it)
			{
				int faceIdx = *it;
				neighbourFaceList.emplace_back(faceList[faceIdx].getFaces()[0]);

				std::vector<Value> qResult;
				spatialIndex.query(bgi::intersects(bboxList[faceIdx]), std::back_inserter(qResult));

				for (size_t j = 0; j < qResult.size(); j++)
				{
					int otherFaceIndx = qResult[j].second;

					if (faceGroupList[otherFaceIndx] != -1) { continue; }

					if (!currentNormal.IsParallel(normalList[otherFaceIndx], lowPrecision)) { continue; }

					BRepExtrema_DistShapeShape distanceCalc(faceList[faceIdx].getFaces()[0], faceList[otherFaceIndx].getFaces()[0]);
					distanceCalc.Perform();

					if (!distanceCalc.IsDone()) { continue;  }
					if (distanceCalc.Value() > lowPrecision) { continue; }

					tempBuffer.emplace_back(otherFaceIndx);
					faceGroupList[otherFaceIndx] = 1;

				}
			}
			if (tempBuffer.size() == 0)
			{
				// merge faces
				if (neighbourFaceList.size() > 1)
				{
					TopTools_ListOfShape toolList;
					for (size_t i = 0; i < neighbourFaceList.size(); i++)
					{
						toolList.Append(neighbourFaceList[i]);
					}
					BRepAlgoAPI_Fuse fuser;
					fuser.SetArguments(toolList);
					fuser.SetTools(toolList);
					fuser.SetFuzzyValue(lowPrecision);
					fuser.Build();

					std::vector<TopoDS_Edge> edgeList;

					Prs3d_ShapeTool Tool(fuser.Shape()); //TODO: check this
					for (Tool.InitCurve(); Tool.MoreCurve(); Tool.NextCurve())
					{
						const TopoDS_Edge& E = Tool.GetCurve();
						if (Tool.FacesOfEdge().get()->Size() == 1) {
							edgeList.emplace_back(E);
						}
					}

					std::vector<TopoDS_Wire> wireList = growWires(edgeList);
					std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
					std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);

					for (size_t it = 0; it < cleanWireList.size(); it++)
					{
						for (TopExp_Explorer vertexExplorer(cleanWireList[it], TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next()) {

							TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
							gp_Pnt point = BRep_Tool::Pnt(vertex);
						}
					}


					if (cleanedFaceList.size())
					{
						SurfaceGroup mergedFaceGroup(cleanedFaceList[0]);
						mergedFaceGroup.projectFace();
						mergedFaceList.emplace_back(mergedFaceGroup);
					}
				}
				else {
					SurfaceGroup mergedFaceGroup(neighbourFaceList[0]);
					mergedFaceGroup.projectFace();
					mergedFaceList.emplace_back(mergedFaceGroup);
				}

				neighbourFaceList.clear();
				bufferList.clear();
				aBuilder.Clear();

				for (int i = 0; i < faceGroupList.size(); i++)
				{
					if (faceGroupList[i] == -1)
					{
						bufferList = { i };
						faceGroupList[i] = 1;
						currentNormal = normalList[i];
						break;
					}
				}
				if (bufferList.size() == 0) { 
					break; 
				}
				continue;
			}
			bufferList = tempBuffer;
			tempBuffer.clear();
		}
		mergedFaceListBuilding.emplace_back(mergedFaceList);
	}

	faceList_ = mergedFaceListBuilding;
}


void CJGeoCreator::initializeBasic(helper* cluster) {
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoPreProcessing) << std::endl;
	// generate data required for most exports
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCoarseFiltering) << std::endl;
	std::vector<TopoDS_Shape> filteredObjects = getTopObjects(cluster);

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoReduceSurfaces) << std::endl;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> shapeIdx;
	std::vector<SurfaceGroup> shapeList;
	reduceSurfaces(filteredObjects, &shapeIdx, &shapeList);

	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoFineFiltering) << std::endl;
	FinefilterSurfaces(shapeList);

	std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofOutlineConstruction) << std::endl;

	std::vector<TopoDS_Face> projectedFaceList;
	for (size_t i = 0; i < faceList_.size(); i++)
	{
		for (SurfaceGroup currentGroup : faceList_[i])
		{
			projectedFaceList.emplace_back(currentGroup.getProjectedFace());
		}
	}
	roofOutlineList_ = simplefyProjection(projectedFaceList);

	printTime(startTime, std::chrono::steady_clock::now());

	hasGeoBase_ = true;

	// sort surface groups based on the footprints
	startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofStructureSorting) << std::endl;
	if (roofOutlineList_.size() != 1) { sortRoofStructures(); }
	printTime(startTime, std::chrono::steady_clock::now());

	startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoRoofStructureMerging) << std::endl;
	mergeRoofSurfaces();
	printTime(startTime, std::chrono::steady_clock::now());

	return;
}


std::vector<TopoDS_Edge> CJGeoCreator::section2edges(const std::vector<Value>& productLookupValues, helper* h, double cutlvl)
{
	std::vector<TopoDS_Edge> rawEdgeList;

	// make a cutting plane 
	gp_Pnt p0(-1000, -1000, cutlvl);
	gp_Pnt p1(-1000, 1000, cutlvl);
	gp_Pnt p2(1000, 1000, cutlvl);
	gp_Pnt p3(1000, -1000, cutlvl);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Face cuttingFace = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));

	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		std::shared_ptr<lookupValue> lookup = h->getLookup(productLookupValues[i].second);
		TopoDS_Shape currentShape;

		if (lookup->hasCBox()) { currentShape = lookup->getCBox(); }
		else { currentShape = h->getObjectShape(lookup->getProductPtr(), true); }

		for (TopExp_Explorer expl(currentShape, TopAbs_FACE); expl.More(); expl.Next()) {

			// ignore if the z component of normal is 0 
			TopoDS_Face face = TopoDS::Face(expl.Current());

			GProp_GProps gprops;
			BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
			double area = gprops.Mass();

			if (area < 0.001) { continue; }

			gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);
			if (std::abs(faceNormal.X() - 0) < 0.001 && std::abs(faceNormal.Y() - 0) < 0.001) { continue; }

			BRepAlgoAPI_Cut cutter(face, cuttingFace);
			if (!cutter.IsDone()) { continue; }

			TopTools_ListOfShape cutterResults = cutter.SectionEdges();
			for (auto it = cutterResults.begin(); it != cutterResults.end(); ++it)
			{
				TopExp_Explorer expl2;
				for (expl2.Init(*it, TopAbs_EDGE); expl2.More(); expl2.Next()) {
					rawEdgeList.emplace_back(TopoDS::Edge(expl2.Current()));
				}
			}
		}
	}
	return rawEdgeList;
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


void CJGeoCreator::makeFootprint(helper* h)
{
	// get footprint
	double floorlvl = SettingsCollection::getInstance().footprintElevation();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoCoasreFootFiltering) << floorlvl << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	double storeyBuffer = 0.15;

	gp_Trsf translation;
	translation.SetTranslation(gp_Vec(0, 0, -storeyBuffer));

	try
	{
		std::vector<TopoDS_Face> footprintList = makeFloorSection(h, floorlvl + storeyBuffer);
		for (TopoDS_Face& footprintItem : footprintList) { footprintItem.Move(translation); }
		footprintList_ = footprintList;
	}
	catch (const std::exception&)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
		throw std::invalid_argument(CommunicationStringEnum::getString(CommunicationStringID::errorFootprintFailed));
		return;
	}

	hasFootprints_ = true;
	printTime(startTime, std::chrono::steady_clock::now());
	std::cout << std::endl;
	return;
}


void CJGeoCreator::makeFloorSectionCollection(helper* h)
{
	//TODO: find out where to take the storeys from
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingStoreys) << std::endl;
	auto startTime = std::chrono::steady_clock::now();
	double storeyBuffer = 0.15;

	IfcSchema::IfcBuildingStorey::list::ptr storeyList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcBuildingStorey>();

	std::vector<std::map<std::string, std::string>> storeyAttributeList;

	gp_Trsf translation;
	translation.SetTranslation(gp_Vec(0, 0, -storeyBuffer));

	for (auto it = storeyList->begin(); it != storeyList->end(); ++it) 
	{
		IfcSchema::IfcBuildingStorey* storeyObject = *it;
		double storeyElevation = storeyObject->Elevation().get() * h->getScaler(0);

		std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentStoreyAtZ) << storeyElevation << std::endl;

		try
		{
			std::vector<TopoDS_Face> storeySurface = makeFloorSection(h, storeyElevation + storeyBuffer);
			for (TopoDS_Face& storeySurfaceItem : storeySurface) { storeySurfaceItem.Move(translation); }

			std::map<std::string, std::string> semanticStoreyData;

			semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::CJType) , CJObjectEnum::getString(CJObjectID::CJTypeStorey));
			if (storeyObject->Name().get() != "") 
			{ 
				semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::ifcName), storeyObject->Name().get()); 
			}
			if (storeyObject->LongName().get() != "") 
			{ 
				semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::ifcLongName), storeyObject->LongName().get()); 
			}
			semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::ifcElevation), std::to_string(storeyObject->Elevation().get() * h->getScaler(0)));
			semanticStoreyData.emplace(CJObjectEnum::getString(CJObjectID::ifcGuid), storeyObject->GlobalId());

			std::map<std::string, std::string> storeyAttributeCollection = h->getProductPropertySet(storeyObject->GlobalId(), 0);

			for (const auto& pair : storeyAttributeCollection) {
				semanticStoreyData.emplace(sourceIdentifierEnum::getString(sourceIdentifierID::ifc) + pair.first, pair.second);
			}

			storeyPrintList_.emplace_back(FloorOutlineObject(storeySurface, semanticStoreyData, storeyObject->GlobalId()));
			hasStoreyPrints_ = true;
		}
		catch (const std::exception&)
		{
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::indentUnsuccesful) << std::endl;
			throw std::invalid_argument(CommunicationStringEnum::getString(CommunicationStringID::errorStoreyFailed));
			continue;
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	std::cout << std::endl;
}


std::vector<TopoDS_Face> CJGeoCreator::makeFloorSection(helper* h, double sectionHeight)
{
	// get plate of voxels at groundplane height
	std::vector<std::shared_ptr<voxel>> exteriorLvlVoxels = voxelGrid_->getVoxelPlate(sectionHeight);

	std::vector<Value> productLookupValues;
	std::vector<std::shared_ptr<voxel>> originVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;
	populateVoxelIndex(&voxelIndex, &originVoxels, exteriorLvlVoxels);

	for (std::shared_ptr<voxel> voxel : exteriorLvlVoxels)
	{
		for (const Value& productValue : voxel->getInternalProductList())
		{
			productLookupValues.emplace_back(productValue);
		} 
	}
	productLookupValues = makeUniqueValueList(productLookupValues);

	if (productLookupValues.size() <= 0)
	{
		throw std::invalid_argument(CommunicationStringEnum::getString(CommunicationStringID::errorLoD02StoreyFailed));
		return{};
	}

	// create unique index
	bgi::rtree<Value, bgi::rstar<25>> exteriorProductIndex;
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		exteriorProductIndex.insert(productLookupValues[i]);
	}

	// evaluate which shapes are completely encapsulated by another shape
	filterEncapsulatedObjects(
		&productLookupValues,
		&exteriorProductIndex,
		h
	);

	// get all edges that meet the cutting plane
	std::vector<TopoDS_Edge> rawEdgeList = section2edges(productLookupValues, h, sectionHeight);
	if (rawEdgeList.size() <= 0)
	{
		throw std::invalid_argument(CommunicationStringEnum::getString(CommunicationStringID::errorLoD02StoreyFailed));
		return{};
	}

	// prepare and clean the edges for ray cast
	std::vector<Edge> uniqueEdges = getUniqueEdges(rawEdgeList);
	std::vector<Edge> cleanedEdges = mergeOverlappingEdges(uniqueEdges, false);
	std::vector<Edge> splitEdges = splitIntersectingEdges(cleanedEdges, false);
	// raycast
	std::vector<TopoDS_Edge> outerFootPrintList = getOuterEdges(splitEdges, voxelIndex, originVoxels, sectionHeight);
	return outerEdges2Shapes(outerFootPrintList);
}


std::vector<TopoDS_Shape> CJGeoCreator::computePrisms(const std::vector<TopoDS_Face>& inputFaceList, double lowestZ)
{
	std::vector<TopoDS_Shape> prismList;
	bool allSolids = true;

	double precision = SettingsCollection::getInstance().precision();

	// make extrusions of untrimmed top surfaces
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;
	std::vector<TopoDS_Solid> ExtrudedShapes;

	for (const TopoDS_Face currentFace : inputFaceList)
	{
		TopoDS_Solid extrudedShape = extrudeFace(currentFace, true, lowestZ);
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(extrudedShape);

		spatialIndex.insert(std::make_pair(bbox, (int)ExtrudedShapes.size()));
		ExtrudedShapes.emplace_back(extrudedShape);
	}

	if (ExtrudedShapes.size() == 1) { return { ExtrudedShapes [0]}; }

	// split top surfaces with the extrustions
	std::mutex faceListMutex;
	std::vector<TopoDS_Face> faceList;
	bgi::rtree<Value, bgi::rstar<25>> faceIdx;

	for (size_t i = 0; i < inputFaceList.size(); i++)
	{
		TopoDS_Face currentFace = inputFaceList[i];
		bg::model::box <BoostPoint3D> searchBox = helperFunctions::createBBox(currentFace);

		std::vector<Value> qResult;
		spatialIndex.query(bgi::intersects(searchBox), std::back_inserter(qResult));

		if (qResult.size() <= 1)
		{
			std::lock_guard<std::mutex> faceLock(faceListMutex);
			faceIdx.insert(std::make_pair(searchBox, static_cast<int>(faceList.size())));
			faceList.emplace_back(currentFace);
		}
		else {
			BOPAlgo_Splitter divider;
			divider.SetFuzzyValue(precision);
			divider.SetRunParallel(Standard_False);
			divider.AddArgument(currentFace);

			for (size_t j = 0; j < qResult.size(); j++)
			{
				int extruIndx = qResult[j].second;
				if (i == extruIndx) { continue; }

				TopoDS_Solid currentSplitter = ExtrudedShapes[extruIndx];
				divider.AddTool(currentSplitter);
			}
			divider.Perform();

			for (TopExp_Explorer expl(divider.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
				TopoDS_Face subFace = TopoDS::Face(expl.Current());
				std::lock_guard<std::mutex> faceLock(faceListMutex);
				faceIdx.insert(std::make_pair(searchBox, static_cast<int>(faceList.size())));
				faceList.emplace_back(subFace);
			}
		}
	}

	// extrude the trimmed surfaces and join
	BOPAlgo_Builder aBuilder;
	aBuilder.SetFuzzyValue(precision);
	aBuilder.SetRunParallel(Standard_True);
	for (size_t i = 0; i < faceList.size(); i++)
	{
		TopoDS_Face currentFace = faceList[i];
		bool isHidden = false;

		std::optional<gp_Pnt> optionalBasePoint = helperFunctions::getPointOnFace(currentFace);
		if (optionalBasePoint == std::nullopt) { continue; }

		gp_Pnt basePoint = *optionalBasePoint;
		gp_Pnt topPoint = gp_Pnt(basePoint.X(), basePoint.Z(), basePoint.Z() + 100000);

		TopoDS_Edge evalLine = BRepBuilderAPI_MakeEdge(basePoint, topPoint);

		std::vector<Value> qResult;
		qResult.clear();
		faceIdx.query(bgi::intersects(
			helperFunctions::createBBox(basePoint, topPoint, 0.2)), std::back_inserter(qResult));

		for (size_t j = 0; j < qResult.size(); j++)
		{
			int otherFaceIdx = qResult[j].second;
			if (i == otherFaceIdx)
			{
				continue;
			}

			BRepExtrema_DistShapeShape distanceWireCalc(evalLine, faceList[otherFaceIdx]);

			if (distanceWireCalc.Value() < 1e-6)
			{
				isHidden = true;
				break;
			}
		}

		if (!isHidden)
		{
			TopoDS_Solid extrudedShape = extrudeFace(currentFace, true, lowestZ);
			if (!extrudedShape.IsNull())
			{
				aBuilder.AddArgument(extrudedShape);
			}
		}
	}
	aBuilder.Perform();

	// clean the overlapping faces
	TopTools_DataMapOfShapeShape ttt = aBuilder.ShapesSD();
	BRepBuilderAPI_Sewing brepSewer;
	for (TopExp_Explorer expl(aBuilder.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face currentFace = TopoDS::Face(expl.Current());
		bool isDub = false;

		for (auto itt = ttt.begin(); itt != ttt.end(); ++itt)
		{
			if (currentFace.IsSame(*itt))
			{
				isDub = true;
				break;
			}
		}

		if (!isDub)
		{
			brepSewer.Add(currentFace);
		}
	}
	brepSewer.Perform();

	try
	{
		TopoDS_Shape simplefiedShape = simplefySolid(brepSewer.SewedShape());
		if (simplefiedShape.IsNull()) { return prismList; }
		prismList.emplace_back(simplefiedShape);
	}
	catch (const std::exception&)
	{
		prismList.emplace_back(brepSewer.SewedShape());
		//TODO: error Output
	}

	if (!allSolids)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningNoSolid) << std::endl;
	}
	return prismList;
}


TopoDS_Shape CJGeoCreator::simplefySolid(const TopoDS_Shape& solidShape, bool evalOverlap)
{
	std::vector<TopoDS_Face> facelist;
	std::vector<gp_Dir> normalList;

	for (TopExp_Explorer expl(solidShape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(face);

		if (faceNomal.Magnitude() == 0)
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

	std::vector<TopoDS_Face> mergedFaceList = simplefySolid(facelist, normalList, evalOverlap);

	if (mergedFaceList.size() == facelist.size())
	{
		return solidShape;
	}

	for (size_t i = 0; i < mergedFaceList.size(); i++)
	{
		brepSewer.Add(mergedFaceList[i]);
	}

	brepSewer.Perform();

	if (brepSewer.SewedShape().ShapeType() == TopAbs_COMPOUND)
	{
		return brepSewer.SewedShape();
	}

	brepBuilder.Add(simpleBuilding, brepSewer.SewedShape());
	return simpleBuilding;
}


std::vector<TopoDS_Face> CJGeoCreator::simplefySolid(const std::vector<TopoDS_Face>& surfaceList, bool evalOverlap) {
	std::vector<gp_Dir> normalList;
	for (size_t i = 0; i < surfaceList.size(); i++)
	{
		gp_Vec faceNomal = helperFunctions::computeFaceNormal(surfaceList[i]);

		if (faceNomal.Magnitude() == 0)
		{
			continue;
		}
		normalList.emplace_back(faceNomal);
	}

	if (surfaceList.size() != normalList.size()) { return surfaceList; }
	return simplefySolid(surfaceList, normalList, evalOverlap);
}


std::vector<TopoDS_Face> CJGeoCreator::simplefySolid(const std::vector<TopoDS_Face>& surfaceList, const std::vector<gp_Dir>& normalList, bool evalOverlap) {

	if (!surfaceList.size()) { return surfaceList; }
	if (surfaceList.size() != normalList.size()) { return surfaceList; }

	// make spatial index
	bgi::rtree<Value, bgi::rstar<25>> shapeIdx;
	for (int i = 0; i < surfaceList.size(); i++)
	{
		TopoDS_Face currentFace = surfaceList[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		shapeIdx.insert(std::make_pair(bbox, i));
	}

	std::vector<TopoDS_Face> cleanedFaceList;

	std::vector<int> mergedSurfaceIdxList = {0};
	std::vector<int> tempMergeSurfaceIdxList;
	std::vector<int> evalList(surfaceList.size());

	double precision = SettingsCollection::getInstance().precision();

	while (true)
	{
		for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
		{
			int currentIdx = mergedSurfaceIdxList[i];
			if (evalList[currentIdx] == 1) { continue; }
			evalList[currentIdx] = 1;

			TopoDS_Face currentFace = surfaceList[currentIdx];
			gp_Dir currentdir = normalList[currentIdx];

			bg::model::box < BoostPoint3D > cummulativeBox = helperFunctions::createBBox(currentFace);
			
			std::vector<Value> qResult;
			qResult.clear();
			shapeIdx.query(bgi::intersects(
				cummulativeBox), std::back_inserter(qResult));

			for (size_t j = 0; j < qResult.size(); j++)
			{
				int otherFaceIdx = qResult[j].second;
				TopoDS_Face otherFace = surfaceList[otherFaceIdx];
				gp_Dir otherdir = normalList[otherFaceIdx];

				if (currentIdx == otherFaceIdx) { continue; }
				if (evalList[otherFaceIdx] == 1) { continue; }
				if (!currentdir.IsParallel(otherdir, precision)) { continue; }
				bool touching = false;

				for (TopExp_Explorer edgeExp(currentFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
					TopoDS_Edge edge1 = TopoDS::Edge(edgeExp.Current());
					gp_Pnt currentStartpoint = helperFunctions::getFirstPointShape(edge1);
					gp_Pnt currentEndpoint = helperFunctions::getLastPointShape(edge1);

					// loop through the edges of face2
					for (TopExp_Explorer edgeExp2(otherFace, TopAbs_EDGE); edgeExp2.More(); edgeExp2.Next()) {
						TopoDS_Edge edge2 = TopoDS::Edge(edgeExp2.Current());
						gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(edge2);
						gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(edge2);

						// check if the edges have the same vertices
						if (currentStartpoint.IsEqual(otherEndpoint, precision) && currentEndpoint.IsEqual(otherStartpoint, precision) ||
							currentEndpoint.IsEqual(otherEndpoint, precision) && currentStartpoint.IsEqual(otherStartpoint, precision)) {
							touching = true;
							break;
						}

						if (!evalOverlap) { continue; }

						BRepExtrema_DistShapeShape distanceWireCalc(edge1, edge2);

						if (distanceWireCalc.Value() < 0.001)
						{
							touching = true;
							break;
						}
					}
					if (touching) { break; }
				}
				if (touching)
				{
					bool isDub = false;
					for (size_t k = 0; k < mergedSurfaceIdxList.size(); k++)
					{
						if (mergedSurfaceIdxList[k] == otherFaceIdx)
						{
							isDub = true;
							break;
						}
					}

					if (!isDub)
					{
						for (size_t k = 0; k < tempMergeSurfaceIdxList.size(); k++)
						{
							if (tempMergeSurfaceIdxList[k] == otherFaceIdx)
							{
								isDub = true;
								break;
							}
						}
					}


					if (!isDub)
					{
						tempMergeSurfaceIdxList.emplace_back(otherFaceIdx);
						continue;
					}
				}
			}
		}

		if (tempMergeSurfaceIdxList.size() == 0)
		{
			if (mergedSurfaceIdxList.size() == 1)
			{
				cleanedFaceList.emplace_back(surfaceList[mergedSurfaceIdxList[0]]);
			}
			else
			{
				std::vector<TopoDS_Face> tempFaceList;
				for (size_t i = 0; i < mergedSurfaceIdxList.size(); i++)
				{
					tempFaceList.emplace_back(surfaceList[mergedSurfaceIdxList[i]]);
				}
				TopoDS_Face mergedFace = mergeFaces(tempFaceList);
				cleanedFaceList.emplace_back(mergedFace);
			}

			bool newSetFound = false;
			for (size_t i = 0; i < surfaceList.size(); i++)
			{
				if (evalList[i] == 0)
				{
					mergedSurfaceIdxList = { (int) i };
					tempMergeSurfaceIdxList.clear();
					newSetFound = true;
					break;
				}
			}
			if (newSetFound)
			{
				continue;
			}
			break;
		}

		for (size_t i = 0; i < tempMergeSurfaceIdxList.size(); i++)
		{
			mergedSurfaceIdxList.emplace_back(tempMergeSurfaceIdxList[i]);
		}
		tempMergeSurfaceIdxList.clear();
	}
	return cleanedFaceList;
}


TopoDS_Face CJGeoCreator::mergeFaces(const std::vector<TopoDS_Face>& mergeFaces) {
	double lowPrecision = SettingsCollection::getInstance().precisionCoarse();
	
	std::vector<TopoDS_Edge> edgeList;
	for (size_t i = 0; i < mergeFaces.size(); i++)
	{
		for (TopExp_Explorer edgeExp(mergeFaces[i], TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
			edgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
		}
	}

	std::vector<TopoDS_Edge> cleanList;
	std::vector<int> evalList(edgeList.size());
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		bool dub = false;
		gp_Pnt currentStartpoint = helperFunctions::getFirstPointShape(edgeList[i]);
		gp_Pnt currentEndpoint = helperFunctions::getLastPointShape(edgeList[i]);

		if (currentStartpoint.IsEqual(currentEndpoint, lowPrecision)) { continue; }

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (j == i) { continue; }
			if (evalList[j] == 1) { continue; }
			gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(edgeList[j]);
			gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(edgeList[j]);

			if (currentStartpoint.IsEqual(otherEndpoint, lowPrecision) && currentEndpoint.IsEqual(otherStartpoint, lowPrecision) ||
				currentEndpoint.IsEqual(otherEndpoint, lowPrecision) && currentStartpoint.IsEqual(otherStartpoint, lowPrecision)) {
				evalList[j] = 1;
				dub = true;
				break;
			}
		}
		if (!dub)
		{
			cleanList.emplace_back(edgeList[i]);
		}
	}
	if (cleanList.size() == 0) { return TopoDS_Face(); }
	std::vector<TopoDS_Wire> wireList = growWires(cleanList);
	if (wireList.size() == 0) { return TopoDS_Face(); }
	std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
	if (cleanWireList.size() == 0) { return TopoDS_Face(); }
	std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);
	if (cleanedFaceList.size() == 0) { return TopoDS_Face(); }

	return cleanedFaceList[0];
}


std::vector<int> CJGeoCreator::getTypeValuesBySample(const TopoDS_Shape& prism, bool flat) {
	std::vector<int> valueList;

	double lowestZ = std::numeric_limits<double>::max();
	int lowestFaceIdx = 0;
	int faceIdx = 0;

	for (TopExp_Explorer faceExp(prism, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		TopoDS_Face currentface = TopoDS::Face(faceExp.Current());
		gp_Vec faceNormal = helperFunctions::computeFaceNormal(currentface).Normalized();

		if (abs(faceNormal.Z() ) < 0.0001)
		{
			valueList.emplace_back(1);
			faceIdx++;
			continue;
		}
		
		valueList.emplace_back(2);

		Bnd_Box bbox;
		BRepBndLib::Add(currentface, bbox);
		Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
		bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		if (zmin < lowestZ) {
			lowestZ = zmin;
			lowestFaceIdx = faceIdx;
		}
		faceIdx++;
	}

	valueList[lowestFaceIdx] = 0;
	return valueList;
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


bool CJGeoCreator::surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape)
{
	double precision = SettingsCollection::getInstance().precision();
	for (TopExp_Explorer explorer(encapsulatedShape, TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& outerSurface = TopoDS::Face(explorer.Current());
		bool encapsulated = true;

		for (TopExp_Explorer explorer2(innerSurface, TopAbs_VERTEX); explorer2.More(); explorer2.Next())
		{
			const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer2.Current());
			BRepExtrema_DistShapeShape distCalculator(outerSurface, vertex);
			distCalculator.Perform();

			if (distCalculator.Value() >= precision)
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


bool CJGeoCreator::checkShapeIntersection(const TopoDS_Edge& ray, const TopoDS_Shape& shape)
{
	for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());

		if (checksurfaceIntersection(ray, currentFace))
		{
			return true;
		}
	}
	return false;
}


bool CJGeoCreator::checksurfaceIntersection(const TopoDS_Edge& ray, const TopoDS_Face& face)
{
	double precision = SettingsCollection::getInstance().precision();
	std::cout << "in" << std::endl;

	BRepExtrema_DistShapeShape intersection(ray, face, precision);

	intersection.Perform();
	std::cout << "out" << std::endl;
	// Check if there is an intersection
	if (!intersection.IsDone()) { return false; }
	if (intersection.NbSolution() < 1) { return false; }
	if (intersection.PointOnShape1(1).Distance(intersection.PointOnShape2(1)) <= precision)
	{
		return true;
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


std::vector<TopoDS_Shape> CJGeoCreator::getTopObjects(helper* h)
{
	auto startTime = std::chrono::steady_clock::now();

	std::vector<int> boxelIdx = voxelGrid_->getTopBoxelIndx(); //TODO: this can be written more pretty 
	std::vector<Value> topValues;
	std::vector<TopoDS_Shape> topObjects;

	for (size_t i = 0; i < boxelIdx.size(); i++)
	{
		int currentVoxelIdx = boxelIdx[i];
		while (true)
		{
			voxel currentVoxel = voxelGrid_->getVoxel(currentVoxelIdx);
			std::vector<Value> internalValueList= currentVoxel.getInternalProductList();

			if (internalValueList.size())
			{
				for (size_t j = 0; j < internalValueList.size(); j++)
				{
					double dub = false;
					Value internalValue = internalValueList[j];

					for (size_t k = 0; k < topValues.size(); k++)
					{
						if (topValues[k].second == internalValue.second)
						{
							dub = true;
							break;
						}
					}
					if (dub) { continue; }

					std::shared_ptr<lookupValue> lookup = h->getLookup(internalValue.second);
					TopoDS_Shape currentShape;

					if (lookup->hasCBox()) { currentShape = lookup->getCBox(); }
					else { currentShape = h->getObjectShape(lookup->getProductPtr(), true); }

					topValues.emplace_back(internalValue);
					topObjects.emplace_back(currentShape);
				}
				break;
			}
			currentVoxelIdx = voxelGrid_->getLowerNeighbour(currentVoxelIdx);
			if (currentVoxelIdx == -1) { break; }
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return topObjects;
}


void CJGeoCreator::reduceSurfaces(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<SurfaceGroup>* shapeList)
{
	auto startTime = std::chrono::steady_clock::now();

	// split the range over cores
	int coreCount = SettingsCollection::getInstance().threadcount();
	int coreUse = coreCount;
	int splitListSize = static_cast<int>(floor(inputShapes.size() / coreUse));
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = inputShapes.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? inputShapes.end() : startIdx + splitListSize;

		std::vector<TopoDS_Shape> sublist(startIdx, endIdx);

		threadList.emplace_back([=, &voxelsGrown]() {reduceSurface(sublist, shapeIdx, shapeList); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
}


void CJGeoCreator::reduceSurface(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<SurfaceGroup>* shapeList)
{
	for (size_t i = 0; i < inputShapes.size(); i++)
	{
		std::vector<SurfaceGroup>  objectFaces = getXYFaces(inputShapes[i]);
		for (size_t j = 0; j < objectFaces.size(); j++)
		{
			if (!helperFunctions::isOverlappingCompletely(objectFaces[j], *shapeList, *shapeIdx))
			{
				auto rtreePair = std::make_pair(helperFunctions::createBBox(objectFaces[j].getFaces()[0]), static_cast<int>(shapeList->size()));
				std::unique_lock<std::mutex> rtreeLock(indexInjectMutex_);
				shapeIdx->insert(rtreePair);
				shapeList->emplace_back(objectFaces[j]);
				rtreeLock.unlock();

				hasTopFaces_ = true;
			}
		}
	}
}

void CJGeoCreator::FinefilterSurfaces(const std::vector<SurfaceGroup>& shapeList)
{
	auto startTime = std::chrono::steady_clock::now();
	// split the range over cores
	int coreCount = SettingsCollection::getInstance().threadcount();
	int coreUse = coreCount;
	int splitListSize = static_cast<int>(floor(shapeList.size() / coreUse));
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;
	faceList_.emplace_back(std::vector<SurfaceGroup>());

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = shapeList.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? shapeList.end() : startIdx + splitListSize;

		std::vector<SurfaceGroup> sublist(startIdx, endIdx);
		threadList.emplace_back([=]() {FinefilterSurface(sublist, shapeList); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
}

void CJGeoCreator::FinefilterSurface(const std::vector<SurfaceGroup>& shapeList, const std::vector<SurfaceGroup>& otherShapeList)
{
	// get the faces visible from the top 
	std::vector<SurfaceGroup> cleanedShapeList;
	for (size_t i = 0; i < shapeList.size(); i++)
	{
		SurfaceGroup currentSurfaceGroup = shapeList[i];
		if (currentSurfaceGroup.testIsVisable(otherShapeList, true))
		{
			std::lock_guard<std::mutex> faceLock(faceListMutex_);
			faceList_[0].emplace_back(currentSurfaceGroup);
		}
	}
}


std::vector<SurfaceGroup> CJGeoCreator::getXYFaces(const TopoDS_Shape& shape) {
	std::vector<SurfaceGroup> surfaceGroupList;
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
		// ignore if the z component of normal is 0 
		TopoDS_Face face = TopoDS::Face(expl.Current());

		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
		double area = gprops.Mass();

		if (area < 0.01) { continue; }

		gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);

		if (std::abs(faceNormal.Z()) < 0.001) { continue;}
		surfaceGroupList.emplace_back(SurfaceGroup(face));
	}

	std::vector<SurfaceGroup> cleanedSurfaceGroupList;
	for (size_t i = 0; i < surfaceGroupList.size(); i++)
	{
		SurfaceGroup currentGroup = surfaceGroupList[i];
		TopoDS_Face currentFace = currentGroup.getFaces()[0];

		if (!currentGroup.isVisible()) { continue; }

		// ignore lowest if identical projected points
		double height = currentGroup.getAvHeight();
		int vertCount = currentGroup.getVertCount();

		for (size_t j = 0; j < surfaceGroupList.size(); j++)
		{
			if (i == j) { continue; }

			SurfaceGroup otherGroup = surfaceGroupList[j];
			TopoDS_Face otherFace = otherGroup.getFaces()[0];

			if (!otherGroup.isVisible()) { continue; }

			double otherHeight = otherGroup.getAvHeight();
			if (height > otherHeight) { continue; }

			int overlapCount = 0;

			TopExp_Explorer expl;
			TopExp_Explorer expl2;
			for (expl.Init(currentFace, TopAbs_VERTEX); expl.More(); expl.Next()) {
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt point = BRep_Tool::Pnt(vertex);

				for (expl2.Init(otherFace, TopAbs_VERTEX); expl2.More(); expl2.Next()) {

					TopoDS_Vertex otherVertex = TopoDS::Vertex(expl2.Current());
					gp_Pnt otherPoint = BRep_Tool::Pnt(otherVertex);

					if (otherPoint.IsEqual(point, 0.0001))
					{
						overlapCount++;
						break;
					}
				}
			}
			if (vertCount == overlapCount) {
				currentGroup.setIsHidden();
				break;
			}
		}

		if (!currentGroup.isVisible()) { continue; }
		
		if (currentGroup.isVisible())
		{ 
			currentGroup.projectFace();
			currentGroup.populateGrid(SettingsCollection::getInstance().surfaceGridSize());
			cleanedSurfaceGroupList.emplace_back(currentGroup);
			continue;
		}
	}

	// do raycasting on itself
	std::vector<SurfaceGroup> filteredSurfaceGroupList;
	for (size_t i = 0; i < cleanedSurfaceGroupList.size(); i++)
	{
		SurfaceGroup currentGroup = cleanedSurfaceGroupList[i];

		if (currentGroup.testIsVisable(cleanedSurfaceGroupList))
		{
			filteredSurfaceGroupList.emplace_back(currentGroup);
		}
	}
	return filteredSurfaceGroupList;
}


std::vector<std::shared_ptr<CJT::CityObject>> CJGeoCreator::makeStoreyObjects(helper* h)
{
	std::vector<std::shared_ptr<CJT::CityObject>> cityStoreyObjects;
	IfcSchema::IfcBuildingStorey::list::ptr storeyList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcBuildingStorey>();


	for (auto it = storeyList->begin(); it != storeyList->end(); ++it)
	{
		IfcSchema::IfcBuildingStorey* storeyObject = *it;
		double storeyElevation = storeyObject->Elevation().get() * h->getScaler(0);

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
		cityStoreyObject.addAttribute(CJObjectEnum::getString(CJObjectID::ifcGuid), storeyObject->GlobalId());
		cityStoreyObject.addAttribute(CJObjectEnum::getString(CJObjectID::ifcElevation), storeyObject->Elevation().get());
		cityStoreyObjects.emplace_back(std::make_shared< CJT::CityObject>(cityStoreyObject));
	}
	return cityStoreyObjects;
}

std::vector<std::shared_ptr<CJT::CityObject>> CJGeoCreator::makeRoomObjects(helper* h, const std::vector<std::shared_ptr<CJT::CityObject>>& cityStoreyObjects)
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
					if (cjtStorey->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)] == targetStoreyGuid)
					{
						cjtStorey->addChild(cjRoomObject);
						storeyFound = true;
						break;
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

CJT::GeoObject CJGeoCreator::makeLoD00(helper* h, CJT::Kernel* kernel, int unitScale)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD00) << std::endl;

	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = settingsCollection.gridRotation();
	TopoDS_Shape floorProjection = helperFunctions::createHorizontalFace(lll, urr, rotationAngle);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	trs.SetTranslationPart(gp_Vec(0, 0, settingsCollection.footprintElevation()));

	helperFunctions::geoTransform(&floorProjection, h->getObjectTranslation(), trs);
	CJT::GeoObject geoObject = kernel->convertToJSON(floorProjection, "0.0");

	std::map<std::string, std::string> semanticData;
	semanticData.emplace(CJObjectEnum::getString(CJObjectID::CJType) , CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
	geoObject.appendSurfaceData(semanticData);
	geoObject.appendSurfaceTypeValue(0);
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD02(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD02) << std::endl;

	SettingsCollection& settingCollection = SettingsCollection::getInstance();

	if (!hasTopFaces_ && useRoofprints_) { return std::vector< CJT::GeoObject>(); }

	std::vector< CJT::GeoObject> geoObjectCollection;

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	std::map<std::string, std::string> semanticFootData;
	semanticFootData.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));

	gp_Pnt urr = h->getUrrPoint();

	if (useRoofprints_)
	{	 // make the roof
		for (size_t i = 0; i < roofOutlineList_.size(); i++)
		{
			TopoDS_Shape currentShape = roofOutlineList_[i];

			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)),  -settingCollection.gridRotation());
			currentShape.Move(localRotationTrsf);

			gp_Trsf trs;
			trs.SetRotation(geoRefRotation_.GetRotation());
			if (hasFootprints_) { trs.SetTranslationPart(gp_Vec(0, 0, urr.Z())); }
			else { trs.SetTranslationPart(gp_Vec(0, 0, settingCollection.footprintElevation())); }
			helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "0.2");
			geoObject.appendSurfaceData(semanticRoofData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (hasFootprints_) 
	{ 
		// make the footprint
		gp_Trsf trs;
		trs.SetRotation(geoRefRotation_.GetRotation());
		trs.SetTranslationPart(gp_Vec(0, 0, settingCollection.footprintElevation()));

		for (size_t i = 0; i < footprintList_.size(); i++)
		{
			TopoDS_Shape currentFace = footprintList_[i];

			gp_Trsf localRotationTrsf;
			localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingCollection.gridRotation());
			currentFace.Move(localRotationTrsf);

			helperFunctions::geoTransform(&currentFace, h->getObjectTranslation(), trs);

			CJT::GeoObject geoObject = kernel->convertToJSON(currentFace, "0.2");
			geoObject.appendSurfaceData(semanticFootData);
			geoObject.appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectCollection;
}


void CJGeoCreator::makeLoD02Storeys(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale) {
	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());

	if (hasStoreyPrints_)
	{
		for (size_t i = 0; i < storeyPrintList_.size(); i++)
		{
			std::vector<TopoDS_Face> currentStoreyGeo = storeyPrintList_[i].getOutlines();
			std::map<std::string, std::string> currentStoreySemantic = storeyPrintList_[i].getSemanticInfo();

			for (size_t j = 0; j < storeyCityObjects.size(); j++)
			{
				if (currentStoreySemantic[CJObjectEnum::getString(CJObjectID::ifcGuid)] != storeyCityObjects[j]->getAttributes()[CJObjectEnum::getString(CJObjectID::ifcGuid)])
				{
					continue;
				}

				for (size_t k = 0; k < currentStoreyGeo.size(); k++)
				{
					TopoDS_Face currentFace = currentStoreyGeo[k];

					currentFace.Move(localRotationTrsf);

					helperFunctions::geoTransform(&currentFace, h->getObjectTranslation(), trs);
					CJT::GeoObject geoObject = kernel->convertToJSON(currentFace, "0.2");
					geoObject.appendSurfaceTypeValue(0);
					storeyCityObjects[j]->addGeoObject(geoObject);
					for (std::pair<std::string, std::string> semanticPair : currentStoreySemantic)
					{
						storeyCityObjects[j]->addAttribute(semanticPair.first, semanticPair.second, false);
					}
				}
			}
		}
	}
}

void CJGeoCreator::makeSimpleLodRooms(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale) {
	
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	
	IfcSchema::IfcSpace::list::ptr spaceList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcSpace>();

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
		double lowestZ = helperFunctions::getLowestPoint(spaceShape, false).Z();
		double highestZ = helperFunctions::getHighestPoint(spaceShape).Z();

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
		for (TopoDS_Face& face : simplefyProjection(flatFaceList))
		{
			face.Move(localRotationTrsf);
			helperFunctions::geoTransform(&face, h->getObjectTranslation(), trs);
			if (settingsCollection.make02())
			{
				CJT::GeoObject roomGeoObject02 = kernel->convertToJSON(face, "0.2");;
				matchingCityRoomObject->addGeoObject(roomGeoObject02);

			}
			if (settingsCollection.make12())
			{
				TopoDS_Solid solidShape12 = extrudeFace(face, false, highestZ);
				CJT::GeoObject roomGeoObject12 = kernel->convertToJSON(solidShape12, "1.2");;
				matchingCityRoomObject->addGeoObject(roomGeoObject12);
			}
		}
		if (!topFaceList.size()) { continue; }
		

		std::vector<TopoDS_Shape> roomPrismList = computePrisms(topFaceList, lowestZ);
		if (roomPrismList.size() == 1)
		{
			roomPrismList[0].Move(localRotationTrsf);
			helperFunctions::geoTransform(&roomPrismList[0], h->getObjectTranslation(), trs);
			CJT::GeoObject roomGeoObject22 = kernel->convertToJSON(roomPrismList[0], "2.2");;
			matchingCityRoomObject->addGeoObject(roomGeoObject22);
		}
	}
	return;
}


CJT::GeoObject CJGeoCreator::makeLoD10(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD10) << std::endl;

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = settingsCollection.gridRotation();

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid bbox;
	brepBuilder.MakeSolid(bbox);

	double footprintHeight = settingsCollection.footprintElevation();

	gp_Pnt p0 = helperFunctions::rotatePointWorld(lll, -rotationAngle);
	gp_Pnt p1 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p2 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p3 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -rotationAngle);

	gp_Pnt p4(helperFunctions::rotatePointWorld(urr, -rotationAngle));
	gp_Pnt p5 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -rotationAngle);
	gp_Pnt p6 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -rotationAngle);
	gp_Pnt p7 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -rotationAngle);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Edge edge4 = BRepBuilderAPI_MakeEdge(p4, p5);
	TopoDS_Edge edge5 = BRepBuilderAPI_MakeEdge(p5, p6);
	TopoDS_Edge edge6 = BRepBuilderAPI_MakeEdge(p6, p7);
	TopoDS_Edge edge7 = BRepBuilderAPI_MakeEdge(p7, p4);

	TopoDS_Edge edge8 = BRepBuilderAPI_MakeEdge(p0, p6);
	TopoDS_Edge edge9 = BRepBuilderAPI_MakeEdge(p3, p7);
	TopoDS_Edge edge10 = BRepBuilderAPI_MakeEdge(p2, p4);
	TopoDS_Edge edge11 = BRepBuilderAPI_MakeEdge(p1, p5);

	std::vector<TopoDS_Face> faceList;
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }

	brepSewer.Perform();
	brepBuilder.Add(bbox, brepSewer.SewedShape());

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	trs.SetTranslationPart(gp_Vec(0, 0, settingsCollection.footprintElevation()));
	helperFunctions::geoTransform(&bbox, h->getObjectTranslation(), trs);

	CJT::GeoObject geoObject = kernel->convertToJSON(bbox, "1.0");
	std::map<std::string, std::string> grMap;
	grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
	std::map<std::string, std::string> wMap;
	wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
	std::map<std::string, std::string> rMap;
	rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));

	geoObject.appendSurfaceData(grMap);
	geoObject.appendSurfaceData(wMap);
	geoObject.appendSurfaceData(rMap);
	geoObject.appendSurfaceTypeValue(0);
	geoObject.appendSurfaceTypeValue(1);
	geoObject.appendSurfaceTypeValue(1);
	geoObject.appendSurfaceTypeValue(1);
	geoObject.appendSurfaceTypeValue(1);
	geoObject.appendSurfaceTypeValue(2);
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD12(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD12) << std::endl;
	if (!hasTopFaces_ || !hasGeoBase_) { { return std::vector< CJT::GeoObject>(); } }

	std::vector< CJT::GeoObject> geoObjectList;
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}

	double height = h->getUrrPoint().Z() - h->getLllPoint().Z();
	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	trs.SetTranslationPart(gp_Vec(0, 0, h->getLllPoint().Z()));
	
	for (size_t i = 0; i < roofOutlineList_.size(); i++)
	{
		TopoDS_Face currentFootprint = roofOutlineList_[i];

		gp_Trsf localRotationTrsf;
		localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
		currentFootprint.Move(localRotationTrsf);

		helperFunctions::geoTransform(&currentFootprint, h->getObjectTranslation(), trs);

		BRepPrimAPI_MakePrism sweeper(currentFootprint, gp_Vec(0, 0, height), Standard_True);
		sweeper.Build();
		TopoDS_Shape extrudedShape = sweeper.Shape();

		CJT::GeoObject geoObject = kernel->convertToJSON(extrudedShape, "1.2");
		std::map<std::string, std::string> grMap;
		grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
		std::map<std::string, std::string> wMap;
		wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
		std::map<std::string, std::string> rMap;
		rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
		geoObject.appendSurfaceData(grMap);
		geoObject.appendSurfaceData(wMap);
		geoObject.appendSurfaceData(rMap);

		int counter = 0;
		for (TopExp_Explorer faceExp(extrudedShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
			TopoDS_Face face = TopoDS::Face(faceExp.Current());
			geoObject.appendSurfaceTypeValue(1);
			counter++;
		}
		geoObject.setSurfaceTypeValue(counter - 2, 0);
		geoObject.setSurfaceTypeValue(counter - 1, 2);
		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD13(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD13) << std::endl;
	std::vector< CJT::GeoObject> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(h);
	}
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}

	std::vector<TopoDS_Shape> prismList;
	for (std::vector<SurfaceGroup> faceCluster : faceList_)
	{
		std::vector<TopoDS_Face> faceCollection;
		for (SurfaceGroup surfaceGroup : faceCluster)
		{
			faceCollection.emplace_back(surfaceGroup.getFlatFace());
		}

		for (TopoDS_Shape prism : computePrisms(faceCollection, h->getLllPoint().Z()))
		{
			prismList.emplace_back(prism);
		}
	}

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		currentShape.Move(localRotationTrsf);
		helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "1.3");
		std::map<std::string, std::string> grMap;
		grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
		std::map<std::string, std::string> wMap;
		wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
		std::map<std::string, std::string> rMap;
		rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
		geoObject.appendSurfaceData(grMap);
		geoObject.appendSurfaceData(wMap);
		geoObject.appendSurfaceData(rMap);

		std::vector<int> typeValueList = getTypeValuesBySample(prismList[i], true);
		geoObject.setSurfaceTypeValues(typeValueList);

		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject> CJGeoCreator::makeLoD22(helper* h, CJT::Kernel* kernel, int unitScale) 
{
	auto startTime = std::chrono::steady_clock::now();
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD22) << std::endl;
	std::vector< CJT::GeoObject> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(h);
	}
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}

	std::vector<TopoDS_Shape> prismList;
	for (std::vector<SurfaceGroup> faceCluster : faceList_)
	{
		std::vector<TopoDS_Face> faceCollection;
		for (SurfaceGroup surfaceGroup : faceCluster)
		{
			for (TopoDS_Face surfaceFace : surfaceGroup.getFaces())
			{
				faceCollection.emplace_back(surfaceFace);
			}
		}

		for (TopoDS_Shape prism : computePrisms(faceCollection, h->getLllPoint().Z()))
		{
			prismList.emplace_back(prism);
		}
	}

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];

		gp_Trsf localRotationTrsf;
		localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
		currentShape.Move(localRotationTrsf);

		helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

		CJT::GeoObject geoObject = kernel->convertToJSON(currentShape, "2.2");
		std::map<std::string, std::string> grMap;
		grMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeGroundSurface));
		std::map<std::string, std::string> wMap;
		wMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeWallSurface));
		std::map<std::string, std::string> rMap;
		rMap.emplace(CJObjectEnum::getString(CJObjectID::CJType), CJObjectEnum::getString(CJObjectID::CJTypeRoofSurface));
		geoObject.appendSurfaceData(grMap);
		geoObject.appendSurfaceData(wMap);
		geoObject.appendSurfaceData(rMap);

		std::vector<int> typeValueList = getTypeValuesBySample(prismList[i], false);
		geoObject.setSurfaceTypeValues(typeValueList);

		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject>CJGeoCreator::makeLoD32(helper* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD32) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double buffer = 1 * settingsCollection.voxelSize(); // set the distance from the bb of the evaluated object
	int maxCastAttempts = 100; // set the maximal amout of cast attempts before the surface is considered interior
	std::vector<std::shared_ptr<voxel>> targetVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;

	// collect and index the voxels to which rays are cast
	std::vector<std::shared_ptr<voxel>> intersectingVoxels = voxelGrid_->getIntersectingVoxels();
	std::vector<std::shared_ptr<voxel>> externalVoxel = voxelGrid_->getExternalVoxels();
	intersectingVoxels.insert(intersectingVoxels.end(), externalVoxel.begin(), externalVoxel.end());
	populateVoxelIndex(&voxelIndex, &targetVoxels, intersectingVoxels);

	// collect and index the products which are presumed to be part of the exterior
	std::vector<Value> productLookupValues = getUniqueProductValues(intersectingVoxels);
	bgi::rtree<Value, bgi::rstar<25>> exteriorProductIndex;
	if (productLookupValues.size() <= 0)
	{
		throw std::invalid_argument(CommunicationStringEnum::getString(CommunicationStringID::errorLoD02StoreyFailed));
		return{};
	}

	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		exteriorProductIndex.insert(productLookupValues[i]);
	}

	// evaluate which shapes are completely encapsulated by another shape
	//filterEncapsulatedObjects(
	//	&productLookupValues,
	//	&exteriorProductIndex,
	//	h
	//);

	// make the collection compund shape
	BRep_Builder builder;
	TopoDS_Compound collectionShape;
	builder.MakeCompound(collectionShape);

	// set up data for the conversion to json
	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	std::vector<int> typeValueList;

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -settingsCollection.gridRotation());
	// evaluate which surfaces are visible
	for (size_t i = 0; i < productLookupValues.size(); i++) //TODO: multithread?
	{
		std::shared_ptr<lookupValue> lookup = h->getLookup(productLookupValues[i].second);
		std::string lookupType = lookup->getProductPtr()->data().type()->name();

		TopoDS_Shape currentShape;
		if (lookupType == "IfcDoor" || lookupType == "IfcWindow")
		{
			if (lookup->hasCBox()) { currentShape = lookup->getCBox(); }
			else { continue; }
		}
		else { currentShape = h->getObjectShape(lookup->getProductPtr(), true); }
		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			bool faceIsExterior = false;
			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
			//Create a grid over the surface and the offsetted wire
			std::vector<gp_Pnt> surfaceGridList = helperFunctions::getPointGridOnSurface(currentFace);
			std::vector<gp_Pnt> wireGridList = helperFunctions::getPointGridOnWire(currentFace);
			surfaceGridList.insert(surfaceGridList.end(), wireGridList.begin(), wireGridList.end());
			// cast a line from the grid to surrounding voxels
			for (const gp_Pnt& gridPoint : surfaceGridList)
			{
				bg::model::box<BoostPoint3D> pointQuerybox(
					{ gridPoint.X() - buffer, gridPoint.Y() - buffer, gridPoint.Z() - buffer }, 
					{ gridPoint.X() + buffer, gridPoint.Y() + buffer, gridPoint.Z() + buffer }
				);
				std::vector<Value> pointQResult;
				voxelIndex.query(bgi::intersects(pointQuerybox) , std::back_inserter(pointQResult));

				//check if ray castline cleared
				for (const Value& voxelValue : pointQResult)
				{
					bool clearLine = true;

					std::shared_ptr<voxel> targetVoxel = targetVoxels[voxelValue.second];

					bg::model::box<BoostPoint3D> productQuerybox( helperFunctions::createBBox(gridPoint, targetVoxel->getOCCTCenterPoint(), settingsCollection.precision()) );
					std::vector<Value> productQResult;
					exteriorProductIndex.query(bgi::intersects(productQuerybox), std::back_inserter(productQResult));

					for (const Value& productValue : productQResult)
					{
						// get the potential faces
						TopoDS_Shape otherShape;

						std::shared_ptr<lookupValue> otherLookup = h->getLookup(productValue.second);
						std::string otherLookupType = otherLookup->getProductPtr()->data().type()->name();

						if (otherLookupType == "IfcDoor" || otherLookupType == "IfcWindow")
						{
							if (otherLookup->hasCBox()) { otherShape = otherLookup->getCBox(); }
							else { continue; }
						}
						else { otherShape = h->getObjectShape(otherLookup->getProductPtr(), true); }

						for (TopExp_Explorer otherExplorer(otherShape, TopAbs_FACE); otherExplorer.More(); otherExplorer.Next())
						{
							//test for linear intersections (get function from helper class)
							const TopoDS_Face& otherFace = TopoDS::Face(otherExplorer.Current());
							TopLoc_Location loc;
							auto mesh = BRep_Tool::Triangulation(otherFace, loc);

							if (currentFace.IsEqual(otherFace)) { continue; }

							for (int j = 1; j <= mesh.get()->NbTriangles(); j++) //TODO: find out if there is use to keep the opencascade structure
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
			// add the face to the compound
			TopoDS_Face currentFaceCopy = currentFace;
			currentFaceCopy.Move(localRotationTrsf);

			helperFunctions::geoTransform(&currentFaceCopy, h->getObjectTranslation(), trs);
			builder.Add(collectionShape, currentFaceCopy);

			// add the semantic data to the map
			if (lookupType == "IfcRoof")
			{
				typeValueList.emplace_back(2);
			}
			if (lookupType == "IfcSlab")
			{
				std::optional<gp_Pnt> pointOnface = helperFunctions::getPointOnFace(currentFaceCopy);
				gp_Vec vecOfFace = helperFunctions::computeFaceNormal(currentFaceCopy);

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
				else if (abs(helperFunctions::computeFaceNormal(currentFaceCopy).Z()) > 0.1)
				{
					//TODO: do a raycast straight upwards

					typeValueList.emplace_back(2);
				}
				else
				{
					typeValueList.emplace_back(1);
				}
			}
			else if (lookupType == "IfcWindow")
			{
				typeValueList.emplace_back(3);
			}
			else if (lookupType == "IfcDoor")
			{
				typeValueList.emplace_back(4);
			}
			else
			{
				typeValueList.emplace_back(1);
			}
		}
	}
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

	geoObjectList.emplace_back(geoObject);
	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}


void CJGeoCreator::makeComplexLoDRooms(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale) {
	IfcSchema::IfcSpace::list::ptr spaceList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcSpace>();
	
	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
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
		helperFunctions::geoTransform(&spaceShape, h->getObjectTranslation(), trs);
		CJT::GeoObject roomGeoObject = kernel->convertToJSON(spaceShape, "3.2");;
		matchingCityRoomObject->addGeoObject(roomGeoObject);
	}
	return;
}

std::vector< CJT::GeoObject>CJGeoCreator::makeV(helper* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD50) << std::endl;
	auto startTime = std::chrono::steady_clock::now();

	voxelGrid_->computeSurfaceSemantics(h);
	TopoDS_Shape sewedShape = voxels2Shape(0); //TODO: make work with multiple buildings in a single model

	gp_Trsf localRotationTrsf;
	localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
	sewedShape.Move(localRotationTrsf);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	helperFunctions::geoTransform(&sewedShape, h->getObjectTranslation(), trs);

	std::vector< CJT::GeoObject> geoObjectList; // final output collection
	if (sewedShape.ShapeType() == TopAbs_COMPOUND)
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningNoSolidLoD50) << std::endl;
		CJT::GeoObject geoObject = kernel->convertToJSON(sewedShape, "5.0");
		geoObjectList.emplace_back(geoObject);

		return geoObjectList;
	}

	BRep_Builder brepBuilder;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid voxelSolid;
	brepBuilder.MakeSolid(voxelSolid);
	brepBuilder.Add(voxelSolid, sewedShape);

	CJT::GeoObject geoObject = kernel->convertToJSON(voxelSolid, "5.0", true);
	geoObjectList.emplace_back(geoObject);

	printTime(startTime, std::chrono::steady_clock::now());
	return geoObjectList;
}

void  CJGeoCreator::makeVRooms(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, const std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int unitScale)
{
	std::cout << CommunicationStringEnum::getString(CommunicationStringID::infoComputingLoD50Rooms) << std::endl;

	auto startTime = std::chrono::steady_clock::now();
	// bool indicating if the voxelroom data has to be created 
	bool genData = false;
	if (!h->getSpaceIndexPointer()->size())
	{
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningNoIfcRoomObjects) << std::endl;
		genData = true;
	}

	for (int i = 1; i < voxelGrid_->getRoomSize(); i++) //TODO: multithread (-6 for the surface creation)
	{
		std::vector< std::shared_ptr<CJT::CityObject>> potentialRoomCityObjectList = fetchRoomObject(h, roomCityObjects, i);

		if (!potentialRoomCityObjectList.size()) { continue; }

		TopoDS_Shape sewedShape = voxels2Shape(i);

		gp_Trsf trs;
		trs.SetRotation(geoRefRotation_.GetRotation());
		helperFunctions::geoTransform(&sewedShape, h->getObjectTranslation(), trs);

		gp_Trsf localRotationTrsf;
		localRotationTrsf.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Vec(0, 0, 1)), -SettingsCollection::getInstance().gridRotation());
		sewedShape.Move(localRotationTrsf);

		if (sewedShape.ShapeType() == TopAbs_COMPOUND)
		{
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningNoSolidLoD50) << std::endl;
			CJT::GeoObject geoObject = kernel->convertToJSON(sewedShape, "5.0");
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
		for (size_t j = 0; j < potentialRoomCityObjectList.size(); j++)
		{
			potentialRoomCityObjectList[j]->addGeoObject(geoObject);
		}
	}

	printTime(startTime, std::chrono::steady_clock::now());
}

std::vector<CJT::CityObject> CJGeoCreator::makeSite(helper* h, CJT::Kernel* kernel, int unitScale)
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
			std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningDubSites) << std::endl;
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
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningNoSites) << std::endl;
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
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningSiteReconstructionFailed) << std::endl;
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
		std::cout << CommunicationStringEnum::getString(CommunicationStringID::warningSiteReconstructionFailed) << std::endl;
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

TopoDS_Shape CJGeoCreator::voxels2Shape(int roomNum)
{
	std::vector<std::thread> threads;
	std::vector<std::vector<TopoDS_Face>> threadFaceLists(6);

	for (int i = 0; i < 6; i++) {
		threads.emplace_back([this, &threadFaceLists, i, roomNum]() {processDirectionalFaces(i, roomNum, std::ref(threadFaceLists[i])); });
	}
	for (auto& t : threads) { t.join(); }

	BRepBuilderAPI_Sewing brepSewer;
	for (const std::vector<TopoDS_Face>& facesList : threadFaceLists) {
		for (const TopoDS_Face face : facesList) {
			brepSewer.Add(face);
		}
	}
	brepSewer.Perform();
	
	return brepSewer.SewedShape();
}


void CJGeoCreator::processDirectionalFaces(int direction, int roomNum, std::vector<TopoDS_Face>& collectionList) { 
	std::vector<std::vector<TopoDS_Edge>> faceList = voxelGrid_->getDirectionalFaces(direction, -SettingsCollection::getInstance().gridRotation(), roomNum);
	

	for (size_t i = 0; i < faceList.size(); i++) {
		std::vector<TopoDS_Wire> wireList = growWires(faceList[i]);
		std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
		std::vector<TopoDS_Face> cleanFaceList = wireCluster2Faces(cleanWireList);

		for (size_t j = 0; j < cleanFaceList.size(); j++) {
			collectionList.emplace_back(cleanFaceList[j]);
		}
	}
	return;
}

void CJGeoCreator::extractOuterVoxelSummary(CJT::CityObject* shellObject, helper* h, double footprintHeight, double geoRot)
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
		if (!currentVoxel->hasWindow()) { continue; }
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

void CJGeoCreator::extractInnerVoxelSummary(CJT::CityObject* shellObject, helper* h)
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

std::vector < std::shared_ptr<CJT::CityObject >> CJGeoCreator::fetchRoomObject(helper* h, const std::vector<std::shared_ptr<CJT::CityObject>>& roomCityObjects, int roomNum)
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
		std::shared_ptr<lookupValue> lookup = h->getSpaceLookup(qResult[k].second);
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
	bgi::rtree<Value, bgi::rstar<25>>* voxelIndex, 
	std::vector<std::shared_ptr<voxel>>* originVoxels,
	const std::vector<std::shared_ptr<voxel>> exteriorVoxels
)
{
	for (auto voxelIt = exteriorVoxels.begin(); voxelIt != exteriorVoxels.end(); ++ voxelIt)
	{
		std::shared_ptr<voxel> currentBoxel = *voxelIt;
		std::vector<Value> internalProducts = currentBoxel->getInternalProductList();

		// voxels that have no internal products do not have an intersection and are stored as completely external voxels
		if (internalProducts.size() == 0)
		{
			auto cornerPoints = currentBoxel->getCornerPoints();
			gp_Pnt lllPoint = cornerPoints[0];
			gp_Pnt urrPoint = cornerPoints[4];

			BoostPoint3D boostlllPoint = BoostPoint3D(lllPoint.X(), lllPoint.Y(), lllPoint.Z());
			BoostPoint3D boosturrPoint = BoostPoint3D(urrPoint.X(), urrPoint.Y(), urrPoint.Z());

			bg::model::box <BoostPoint3D> box = bg::model::box < BoostPoint3D >(boostlllPoint, boosturrPoint);

			voxelIndex->insert(std::make_pair(box, (int)originVoxels->size()));
			originVoxels->emplace_back(currentBoxel);
		}
	}
}


void CJGeoCreator::filterEncapsulatedObjects(std::vector<Value>* productLookupValues, bgi::rtree<Value, bgi::rstar<25>>* exteriorProductIndex, helper* h)
{
	bgi::rtree<Value, bgi::rstar<25>> cleandExteriorProductIndex;
	std::vector<Value> cleanedProductLookupValues;
	double precision = SettingsCollection::getInstance().precision();

	for (size_t i = 0; i < productLookupValues->size(); i++)
	{
		bool isExposed = true;
		std::shared_ptr<lookupValue> lookup = h->getLookup(productLookupValues->at(i).second);
		TopoDS_Shape currentShape;

		if (lookup->hasCBox()) { currentShape = lookup->getCBox(); }
		else { currentShape = h->getObjectShape(lookup->getProductPtr(), true); }

		bg::model::box <BoostPoint3D> box = productLookupValues->at(i).first;

		std::vector<Value> qResult;
		qResult.clear();
		exteriorProductIndex->query(bgi::intersects(box), std::back_inserter(qResult));

		for (size_t k = 0; k < qResult.size(); k++)
		{
			bool encapsulating = true;

			std::shared_ptr<lookupValue> otherLookup = h->getLookup(qResult[k].second);

			TopoDS_Shape otherShape;
			if (otherLookup->hasCBox()) { otherShape = otherLookup->getCBox(); }
			else { otherShape = h->getObjectShape(otherLookup->getProductPtr(), true); }

			if (currentShape.IsEqual(otherShape)) { continue; }

			BRepClass3d_SolidClassifier solidClassifier;
			solidClassifier.Load(otherShape);

			for (TopExp_Explorer vertexExplorer(currentShape, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next())
			{
				const TopoDS_Vertex& vertex = TopoDS::Vertex(vertexExplorer.Current());
				gp_Pnt vertexPoint = BRep_Tool::Pnt(vertex);

				solidClassifier.Perform(vertexPoint, precision);

				TopAbs_State classifierState = solidClassifier.State();

				if (classifierState == TopAbs_IN || classifierState == TopAbs_ON)
				{
					continue;
				}

				encapsulating = false;
				break;
			}
			if (!encapsulating) { continue; }

			isExposed = false;
			break;
		}

		if (isExposed)
		{
			cleandExteriorProductIndex.insert(productLookupValues->at(i));
			cleanedProductLookupValues.emplace_back(productLookupValues->at(i));
		}
	}

	*exteriorProductIndex = cleandExteriorProductIndex;
	*productLookupValues = cleanedProductLookupValues;
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


CJGeoCreator::CJGeoCreator(helper* h, double vSize)
{
	// compute generic voxelfield data
	voxelGrid_ = std::make_shared<VoxelGrid>(h);
}

FloorOutlineObject::FloorOutlineObject(const std::vector<TopoDS_Face>& outlineList, const std::map<std::string, std::string>& semanticInformation, const std::string& guid)
{
	outlineList_ = outlineList;
	semanticInformation_ = semanticInformation;
}

FloorOutlineObject::FloorOutlineObject(const TopoDS_Face& outlineList, const std::map<std::string, std::string>& semanticInformation, const std::string& guid)
{
	outlineList_ = std::vector<TopoDS_Face>{ outlineList };
	semanticInformation_ = semanticInformation;
}
