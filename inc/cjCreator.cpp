#include "DataManager.h"
#include "cjCreator.h"
#include "helper.h"
#include "voxel.h"

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

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopExp.hxx>

#include <CJToKernel.h>

#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>


void flipPoints(gp_Pnt* p1, gp_Pnt* p2) {
	gp_Pnt tempPoint = *p1;
	*p1 = *p2;
	*p2 = tempPoint;
	return;
}


std::vector<Edge> CJGeoCreator::getUniqueEdges(const TopoDS_Shape& flattenedEdges) //TODO: check if pointer is needed
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


std::vector<Edge> CJGeoCreator::getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges) //TODO: check if pointer is needed
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
	for (size_t j = 0; j < faceList.size(); j++)
	{
		int vertCount = 0;
		int overlapCount = 0;

		for (TopExp_Explorer vertexExplorer(faceList[j], TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next()) {

			bool hasOverlap = false;
			vertCount++;

			TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);

			for (size_t k = 0; k < faceList.size(); k++)
			{
				if (j == k)
				{
					continue;
				}
				for (TopExp_Explorer otherVertexExplorer(faceList[k], TopAbs_VERTEX); otherVertexExplorer.More(); otherVertexExplorer.Next()) {
					TopoDS_Vertex otherVertex = TopoDS::Vertex(otherVertexExplorer.Current());
					gp_Pnt otherPoint = BRep_Tool::Pnt(otherVertex);

					if (point.IsEqual(otherPoint, 1e-6))
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
	gp_Pnt startPoint = helperFunctions::getFirstPointShape(currentEdge);
	gp_Pnt endPoint = helperFunctions::getLastPointShape(currentEdge);

	bool dub = false;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		Edge currentEdge = edgeList[i];
		gp_Pnt otherStartPoint = currentEdge.getStart(true);
		gp_Pnt otherEndPoint = currentEdge.getEnd(true);

		if (startPoint.IsEqual(otherStartPoint, 0.0001) && endPoint.IsEqual(otherEndPoint, 0.0001) ||
			endPoint.IsEqual(otherStartPoint, 0.0001) && startPoint.IsEqual(otherEndPoint, 0.0001))
		{
			return true;
		}
	}
	return false;
}


std::vector<Edge> CJGeoCreator::mergeOverlappingEdges(const std::vector<Edge>& uniqueEdges, bool project)
{
	double buffer = 0.01; //TODO: have it scale with units

	// merge lines that are on the same plane
	std::vector<Edge> cleanedEdgeList;
	std::vector<int> evalList(uniqueEdges.size());

	std::vector<int> discardIndx;
	for (size_t i = 0; i < uniqueEdges.size(); i++)
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

		for (size_t j = 0; j < uniqueEdges.size(); j++)
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
			if (startPoint.Distance(endPoint) > 0.01) //TODO: make smarter
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
	
	for (size_t i = 0; i < edges.size(); i++)
	{
		std::vector<gp_Pnt> intPoints;

		Edge currentEdge = edges[i];
		gp_Pnt startPoint = currentEdge.getStart(project);
		gp_Pnt endPoint = currentEdge.getEnd(project);

		for (size_t j = 0; j < edges.size(); j++)
		{
			if (i == j) { continue; }
			gp_Pnt* intersection = helperFunctions::linearLineIntersection(currentEdge, edges[j], project);

			if (intersection == nullptr) { continue; }
			intPoints.emplace_back(*intersection);
			delete intersection;
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
			for (size_t j = 0; j < cleanedPoints.size(); j++)
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


std::vector<Edge> CJGeoCreator::makeJumbledGround() {
	std::vector<TopoDS_Edge> projectedEdges;
	for (size_t i = 0; i < faceList_.size(); i++) 
	{
		for (size_t j = 0; j < faceList_[i].size(); j++)
		{
			TopoDS_Face porjectedFace = faceList_[i][j].getProjectedFace();
			for (TopExp_Explorer edgeExplorer(porjectedFace, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
				projectedEdges.emplace_back(TopoDS::Edge(edgeExplorer.Current()));
			}
		}
	}
	std::vector<Edge> uniqueEdges = getUniqueEdges(projectedEdges);
	std::vector<Edge> cleanedEdges = mergeOverlappingEdges(uniqueEdges);
	return splitIntersectingEdges(cleanedEdges);
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

	for (size_t i = 0; i < qResult.size(); i++)
	{
		TopoDS_Face evalFace = flatFaceList[qResult[i].second];
		int intersectionCount1 = 0;
		int intersectionCount2 = 0;

		for (TopExp_Explorer explorer(evalFace, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			
			if (helperFunctions::linearLineIntersection(evalEdge1, edge, false, 1e-6) != nullptr) { intersectionCount1++; }
			if (helperFunctions::linearLineIntersection(evalEdge2, edge, false, 1e-6) != nullptr) { intersectionCount2++; }
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
	const std::vector<voxel*>& originVoxels,
	double floorlvl
) {

	//create index of the splitEdges
	bgi::rtree<Value, bgi::rstar<25>> edgeIndex;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		Edge currentEdge = edgeList[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentEdge.getStart(false), currentEdge.getEnd(false));
		edgeIndex.insert(std::make_pair(bbox, i));
	}

	// raycast
	double distance = 2 * sudoSettings_->voxelSize_;

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

			auto boostCastPoint = currentBoxel.getCenterPoint(planeRotation_);
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

				if (distanceCalc.Value() < 1e-6)
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

			if (p1.IsEqual(extendingPoint, 1e-6)) // check if edge is neighbour
			{
				extendingPoint = p2;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				else { tempEdgeList.emplace_back(otherEdge); }
				break;
			}
			else if (p2.IsEqual(extendingPoint, 1e-6)) // check if reversed edge is neighbour 
			{
				extendingPoint = p1;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), otherEdge); }
				else { tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				break;
			}
			else if (extendingPoint.IsEqual(originPoint, 1e-6)) // check if a closed loop is found if no new neighbour is there
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

		for (size_t i = 0; i < wireCollection.size(); i++) 
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

	gp_Pnt connection = helperFunctions::getFirstPointShape(orderedEdgeList[0]);
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

			endPoint = helperFunctions::getFirstPointShape(otherEdge);
			gp_Vec endPointVec = gp_Vec(connection, endPoint);

			if (endPointVec.Magnitude() == 0) { break; }
			else if (!currentVec.IsParallel(endPointVec, 0.01)){endPoint = helperFunctions::getLastPointShape(otherEdge);}

			break;
		}

		if (connection.IsEqual(endPoint, 1e-6)) { continue; }

		wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));
		connection = endPoint;
	}

	if (connection.IsEqual(helperFunctions::getFirstPointShape(orderedEdgeList[0]), 1e-6))
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
		for (size_t j = 0; j < areaList.size(); j++)
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

					if (distanceCalc1.Value() < 1e-6) { intersectionCount++; }
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
	std::vector <std::vector<SurfaceGroup>> mergedFaceListBuilding;
	std::vector<TopoDS_Face> mergeFaceList;

	for (auto buildingFaceIt = faceList_.begin(); buildingFaceIt != faceList_.end(); ++buildingFaceIt)
	{
		//std::cout << "in" << std::endl;
		// loop per sub building

		std::vector<SurfaceGroup > mergedFaceList;

		std::vector<SurfaceGroup> faceList = *buildingFaceIt;

		// compute the face normal
		std::vector<gp_Vec> normalList = {};
		for (auto faceIt = faceList.begin(); faceIt != faceList.end(); ++faceIt)
		{
			SurfaceGroup currentFace = *faceIt;
			normalList.emplace_back(helperFunctions::computeFaceNormal(currentFace.getFace()));
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
		if (!isDub) { return; }

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

		std::vector<TopoDS_Face> mergeFaceList;

		BOPAlgo_Builder aBuilder;
		while (true)
		{
			std::vector<int> tempBuffer = {};
			for (auto it = bufferList.begin(); it != bufferList.end(); ++it)
			{
				int faceIdx = *it;
				mergeFaceList.emplace_back(faceList[faceIdx].getFace());

				std::vector<Value> qResult;
				spatialIndex.query(bgi::intersects(bboxList[faceIdx]), std::back_inserter(qResult));

				for (size_t j = 0; j < qResult.size(); j++)
				{
					int otherFaceIndx = qResult[j].second;

					if (faceGroupList[otherFaceIndx] != -1) { continue; }

					if (!currentNormal.IsParallel(normalList[otherFaceIndx], 1e-4)) { continue; }

					BRepExtrema_DistShapeShape distanceCalc(faceList[faceIdx].getFace(), faceList[otherFaceIndx].getFace());
					distanceCalc.Perform();

					if (!distanceCalc.IsDone()) { continue;  }
					if (distanceCalc.Value() > 1e-4) { continue; }

					tempBuffer.emplace_back(otherFaceIndx);
					faceGroupList[otherFaceIndx] = 1;

				}
			}
			if (tempBuffer.size() == 0)
			{
				// merge faces
				if (mergeFaceList.size() > 1)
				{
					TopTools_ListOfShape toolList;
					for (size_t i = 0; i < mergeFaceList.size(); i++)
					{
						toolList.Append(mergeFaceList[i]);
					}
					BRepAlgoAPI_Fuse fuser;
					fuser.SetArguments(toolList);
					fuser.SetTools(toolList);
					fuser.SetFuzzyValue(1e-4);
					fuser.Build();

					std::vector<TopoDS_Edge> edgeList;

					Prs3d_ShapeTool Tool(fuser.Shape());
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

					if (cleanedFaceList.size())
					{
						SurfaceGroup mergedFaceGroup(cleanedFaceList[0]);
						mergedFaceGroup.projectFace();
						mergedFaceList.emplace_back(mergedFaceGroup);
					}
				}
				else {
					SurfaceGroup mergedFaceGroup(mergeFaceList[0]);
					mergedFaceGroup.projectFace();
					mergedFaceList.emplace_back(mergedFaceGroup);
				}
				mergeFaceList.clear();
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
	std::cout << "- Pre proccessing" << std::endl;
	// generate data required for most exports
	std::vector<TopoDS_Shape> filteredObjects = getTopObjects(cluster);

	std::cout << "- Reduce surfaces" << std::endl;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> shapeIdx;
	std::vector<SurfaceGroup> shapeList;
	reduceSurfaces(filteredObjects, &shapeIdx, &shapeList);

	std::cout << "- Fine filtering of roofing structures" << std::endl;
	FinefilterSurfaces(shapeList);

	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Construct roof outlines" << std::endl;
	std::vector<Edge> edgeList = makeJumbledGround();

	// find outer edge
	std::vector<TopoDS_Edge> outerList;
	for (size_t i = 0; i < faceList_.size(); i++)
	{
		std::vector<TopoDS_Edge> subOuterList = getOuterEdges(edgeList, faceList_[0]);
		for (size_t j = 0; j < subOuterList.size(); j++)
		{
			outerList.emplace_back(subOuterList[j]);
		}
	}

	roofOutlineList_ = outerEdges2Shapes(outerList);
	printTime(startTime, std::chrono::high_resolution_clock::now());

	hasGeoBase_ = true;

	// sort surface groups based on the footprints
	startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Sort roofing structures" << std::endl;
	if (roofOutlineList_.size() != 1) { sortRoofStructures(); }
	printTime(startTime, std::chrono::high_resolution_clock::now());

	startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- merge roofing structures" << std::endl;
	mergeRoofSurfaces();
	printTime(startTime, std::chrono::high_resolution_clock::now());

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
		lookupValue* lookup = h->getLookup(productLookupValues[i].second);
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


TopoDS_Solid CJGeoCreator::extrudeFaceDW(const TopoDS_Face& evalFace, double splittingFaceHeight)
{
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidShape;
	brepBuilder.MakeSolid(solidShape);

	TopoDS_Face projectedFace = helperFunctions::projectFaceFlat(evalFace, splittingFaceHeight);
	brepSewer.Add(evalFace);
	brepSewer.Add(projectedFace);

	for (TopExp_Explorer edgeExplorer(evalFace, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
		const TopoDS_Edge& edge = TopoDS::Edge(edgeExplorer.Current());
		gp_Pnt p0 = helperFunctions::getFirstPointShape(edge);
		gp_Pnt p1 = helperFunctions::getLastPointShape(edge);

		if (p0.Z() <= splittingFaceHeight || p1.Z() <= splittingFaceHeight)
		{
			return TopoDS_Solid();
		}

		TopoDS_Face sideFace = helperFunctions::createPlanarFace(p0, p1, gp_Pnt(p1.X(), p1.Y(), splittingFaceHeight), gp_Pnt(p0.X(), p0.Y(), splittingFaceHeight));
		brepSewer.Add(sideFace);
	}

	brepSewer.Perform();
	if (brepSewer.SewedShape().Closed())
	{
		brepBuilder.Add(solidShape, brepSewer.SewedShape());
	}
	return solidShape;
}


void CJGeoCreator::makeFootprint(helper* h)
{
	// get footprint
	double floorlvl = sudoSettings_->footprintElevation_;
	std::cout << "- Corse filtering footprint at z = " << floorlvl << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	try
	{
		footprintList_ = makeFloorSection(h, floorlvl);

	}
	catch (const std::exception&)
	{
		std::cout << "\tUnsuccessful" << std::endl;
		throw std::invalid_argument("Footprint extraction failed");
		return;
	}

	hasFootprints_ = true;
	printTime(startTime, std::chrono::high_resolution_clock::now());
	std::cout << std::endl;
	return;
}


void CJGeoCreator::makeFloorSectionCollection(helper* h)
{
	//TODO: find out where to take the storeys from
	std::cout << "- Storey extraction" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();
	double storeyBuffer = 0.15;

	IfcSchema::IfcBuildingStorey::list::ptr storeyList = h->getSourceFile(0)->instances_by_type<IfcSchema::IfcBuildingStorey>();

	std::vector<std::map<std::string, std::string>> storeyAttributeList;
	for (auto it = storeyList->begin(); it != storeyList->end(); ++it) 
	{
		IfcSchema::IfcBuildingStorey* storeyObject = *it;
		double storeyElevation = storeyObject->Elevation() * h->getScaler(0);

		std::cout << "\t Floorlevel at z = " << storeyElevation << std::endl;

		try
		{
			std::vector<TopoDS_Face> storeySurface = makeFloorSection(h, storeyElevation - storeyBuffer);

			std::map<std::string, std::string> semanticStoreyData;
			semanticStoreyData.emplace("type", "BuildingStorey");
			if (storeyObject->hasName()) { semanticStoreyData.emplace("IFC_Name", storeyObject->Name()); }
			if (storeyObject->hasLongName()) { semanticStoreyData.emplace("IFC_LongName", storeyObject->LongName()); }
			semanticStoreyData.emplace("IFC_Elevation", std::to_string(storeyObject->Elevation() * h->getScaler(0)));
			semanticStoreyData.emplace("IFC_Guid", storeyObject->GlobalId());

			std::map<std::string, std::string> storeyAttributeCollection = h->getProductPropertySet(storeyObject->GlobalId(), 0);

			for (const auto& pair : storeyAttributeCollection) {
				semanticStoreyData.emplace("IFC_" + pair.first, pair.second);
			}

			storeyPrintList_.emplace_back(FloorOutlineObject(storeySurface, semanticStoreyData, storeyObject->GlobalId()));
			hasStoreyPrints_ = true;
		}
		catch (const std::exception&)
		{
			std::cout << "\tUnsuccessful" << std::endl;
			throw std::invalid_argument("storey extraction failed");
			continue;
		}
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	std::cout << std::endl;
}


std::vector<TopoDS_Face> CJGeoCreator::makeFloorSection(helper* h, double sectionHeight)
{
	// get plate of voxels at groundplane height
	std::vector<voxel*> exteriorLvlVoxels = voxelGrid_->getVoxelPlate(sectionHeight);

	std::vector<Value> productLookupValues;
	std::vector<voxel*> originVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;
	populateVoxelIndex(&voxelIndex, &originVoxels, &productLookupValues, exteriorLvlVoxels);

	productLookupValues = makeUniqueValueList(productLookupValues);

	if (productLookupValues.size() <= 0)
	{
		throw std::invalid_argument("floorprint extraction failed");
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
		throw std::invalid_argument("floorprint extraction failed");
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



std::vector<TopoDS_Shape> CJGeoCreator::computePrisms(bool isFlat, helper* h)
{
	std::vector<TopoDS_Shape> prismList;

	bool allSolids = true;
	for (size_t i = 0; i < faceList_.size(); i++)
	{
		if (faceList_[i].size() == 0) { continue; }

		BOPAlgo_Builder aBuilder;
		for (size_t j = 0; j < faceList_[i].size(); j++)
		{
			SurfaceGroup currentRoof = faceList_[i][j];
			if (currentRoof.getURRPoint().Z() == 0) { continue; }

			TopoDS_Face currentFace;
			if (!isFlat) { currentFace = currentRoof.getFace(); }
			else { currentFace = currentRoof.getFlatFace(); }
			TopoDS_Solid extrudedShape = extrudeFaceDW(currentFace, h->getLllPoint().Z());
			aBuilder.AddArgument(extrudedShape);
		}

		aBuilder.SetFuzzyValue(1e-7);
		aBuilder.SetRunParallel(Standard_True);
		aBuilder.SetUseOBB(Standard_True);
		aBuilder.Perform();

		Sleep(0.1); //TODO: find out why the pause is required

		std::vector<TopoDS_Face> faceList;
		bgi::rtree<Value, bgi::rstar<25>> faceIdx;

		for (TopExp_Explorer expl(aBuilder.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
			TopoDS_Face currentFace = TopoDS::Face(expl.Current());
			gp_Vec faceNormal = helperFunctions::computeFaceNormal(currentFace);

			if (abs(faceNormal.Z()) > 1e-4)
			{
				bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);

				bool isDub = false;
				for (size_t j = 0; j < faceList.size(); j++) //TODO: use index
				{
					if (currentFace.IsPartner(faceList[j]))
					{
						isDub = true;
					}
				}

				if (!isDub)
				{
					faceIdx.insert(std::make_pair(bbox, faceList.size()));
					faceList.emplace_back(currentFace);
				}

			}
		}

		aBuilder.Clear();
		std::vector<TopoDS_Face> topTrimFaceList;
		for (size_t j = 0; j < faceList.size(); j++)
		{
			TopoDS_Face currentFace = faceList[j];
			bool isHidden = false;

			gp_Pnt basePoint = helperFunctions::getPointOnFace(currentFace);
			gp_Pnt topPoint = gp_Pnt(basePoint.X(), basePoint.Z(), basePoint.Z() + 100000);

			TopoDS_Edge evalLine = BRepBuilderAPI_MakeEdge(basePoint, topPoint);

			std::vector<Value> qResult;
			qResult.clear();
			faceIdx.query(bgi::intersects(
				helperFunctions::createBBox(basePoint, topPoint, 0.2)), std::back_inserter(qResult));

			for (size_t k = 0; k < qResult.size(); k++)
			{
				int otherFaceIdx = qResult[k].second;
				if (j == otherFaceIdx)
				{
					continue;
				}

				BRepExtrema_DistShapeShape distanceWireCalc(evalLine, faceList[otherFaceIdx]);

				if (distanceWireCalc.Value() < 0.00001)
				{
					isHidden = true;
					break;
				}
			}

			if (!isHidden)
			{	
				TopoDS_Solid extrudedShape = extrudeFaceDW(currentFace, h->getLllPoint().Z());
				if (!extrudedShape.IsNull())
				{
					aBuilder.AddArgument(extrudedShape);
					topTrimFaceList.emplace_back(currentFace);
				}

			}
		}

		aBuilder.Perform();
		BRepAlgoAPI_Fuse fuser;
		fuser.SetFuzzyValue(1e-6);
		fuser.SetRunParallel(Standard_True);
		fuser.SetUseOBB(Standard_True);
		TopTools_ListOfShape toolList;

		for (TopExp_Explorer expl(aBuilder.Shape(), TopAbs_SOLID); expl.More(); expl.Next()) {
			TopoDS_Solid currentSolid = TopoDS::Solid(expl.Current());

			int overlapCount = 0;

			for (size_t i = 0; i < topTrimFaceList.size(); i++)
			{
				bool allVerts = true;
				for (TopExp_Explorer FaceVertExpl(topTrimFaceList[i], TopAbs_VERTEX); FaceVertExpl.More(); FaceVertExpl.Next())
				{
					bool vertFound = false;
					TopoDS_Vertex vertex = TopoDS::Vertex(FaceVertExpl.Current());
					gp_Pnt p = BRep_Tool::Pnt(vertex);

					for (TopExp_Explorer SolidVertExpl(currentSolid, TopAbs_VERTEX); SolidVertExpl.More(); SolidVertExpl.Next())
					{
						TopoDS_Vertex solidDertex = TopoDS::Vertex(SolidVertExpl.Current());
						gp_Pnt solidP = BRep_Tool::Pnt(solidDertex);

						if (solidP.IsEqual(p, 1e-6))
						{
							vertFound = true;
							break;
						}
					}

					if (vertFound) { continue; }
					allVerts = false;
					break;
				}

				if (allVerts) { overlapCount++; }
				if (overlapCount > 1) { break; }
			}

			if (overlapCount == 1)
			{
				toolList.Append(currentSolid);
			}
		}

		fuser.SetArguments(toolList);
		fuser.SetTools(toolList);
		fuser.Build();

		prismList.emplace_back(simplefySolid(fuser.Shape()));
	}

	if (!allSolids)
	{
		std::cout << "	Not all shapes could be converted to solids, output might be incorrect or inaccurate!" << std::endl;
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

	if (surfaceList.size() != normalList.size()) { return surfaceList; }

	// make spatial index
	bgi::rtree<Value, bgi::rstar<25>> shapeIdx;
	for (size_t i = 0; i < surfaceList.size(); i++)
	{
		TopoDS_Face currentFace = surfaceList[i];
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		shapeIdx.insert(std::make_pair(bbox, i));
	}

	std::vector<TopoDS_Face> cleanedFaceList;

	std::vector<int> mergedSurfaceIdxList = {0};
	std::vector<int> tempMergeSurfaceIdxList;
	std::vector<int> evalList(surfaceList.size());

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
				if (!currentdir.IsParallel(otherdir, 1e-6)) { continue; }
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
						if (currentStartpoint.IsEqual(otherEndpoint, 1e-6) && currentEndpoint.IsEqual(otherStartpoint, 1e-6) ||
							currentEndpoint.IsEqual(otherEndpoint, 1e-6) && currentStartpoint.IsEqual(otherStartpoint, 1e-6)) {
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

		if (currentStartpoint.IsEqual(currentEndpoint, 1e-4)) { continue; }

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (j == i) { continue; }
			if (evalList[j] == 1) { continue; }
			gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(edgeList[j]);
			gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(edgeList[j]);

			if (currentStartpoint.IsEqual(otherEndpoint, 1e-4) && currentEndpoint.IsEqual(otherStartpoint, 1e-4) ||
				currentEndpoint.IsEqual(otherEndpoint, 1e-4) && currentStartpoint.IsEqual(otherStartpoint, 1e-4)) {
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


std::vector<int> CJGeoCreator::getTypeValuesBySample(const TopoDS_Shape& prism, int prismNum, bool flat) {
	std::vector<int> valueList;

	for (TopExp_Explorer faceExp(prism, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		valueList.emplace_back(1);
	}

	std::vector<TopoDS_Face> faceList;
	if (flat)
	{
		for (size_t i = 0; i < faceList_[prismNum].size(); i++)
		{
			faceList.emplace_back(faceList_[prismNum][i].getFlatFace());
		}
	}
	else {
		for (size_t i = 0; i < faceList_[prismNum].size(); i++)
		{
			faceList.emplace_back(faceList_[prismNum][i].getFace());
		}
	}

	double lowestZ = std::numeric_limits<double>::max();
	int lowestFaceIdx = 0;
	int faceIdx = 0;

	for (TopExp_Explorer faceExp(prism, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		TopoDS_Face currentface = TopoDS::Face(faceExp.Current());
		gp_Pnt evalpoint = helperFunctions::getPointOnFace(currentface);
		for (size_t i = 0; i < faceList.size(); i++)
		{
			TopoDS_Face evalFace = faceList[i];

			BRepExtrema_DistShapeShape distanceWireCalc(evalFace, BRepBuilderAPI_MakeVertex(evalpoint));
			distanceWireCalc.Perform();

			if (distanceWireCalc.Value() < 0.00001)
			{
				valueList[faceIdx] = 2;
				break;
			}
		}
		Bnd_Box bbox;
		BRepBndLib::Add(currentface, bbox);
		Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
		bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		if (zmin < lowestZ && zmin - 0.001 < zmax && zmin + 0.001 > zmax) {
			lowestZ = zmin;
			lowestFaceIdx = faceIdx;
		}
		faceIdx++;
	}

	valueList[lowestFaceIdx] = 0;
	return valueList;
}


void CJGeoCreator::printTime(const std::chrono::steady_clock::time_point& startTime, const std::chrono::steady_clock::time_point& endTime) {
	LONGLONG duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5)
	{
		std::cout << "	Successfully finished in: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << "ms" << std::endl;
	}
	else {
		std::cout << "	Successfully finished in: " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << "s" << std::endl;
	}
}


bool CJGeoCreator::surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Shape& encapsulatedShape)
{
	for (TopExp_Explorer explorer(encapsulatedShape, TopAbs_FACE); explorer.More(); explorer.Next())
	{
		const TopoDS_Face& outerSurface = TopoDS::Face(explorer.Current());
		bool encapsulated = true;

		for (TopExp_Explorer explorer2(innerSurface, TopAbs_VERTEX); explorer2.More(); explorer2.Next())
		{
			const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer2.Current());
			BRepExtrema_DistShapeShape distCalculator(outerSurface, vertex);
			distCalculator.Perform();

			if (distCalculator.Value() >= 1e-6) 
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
	BRepExtrema_DistShapeShape intersection(ray, face, 1e-6);

	intersection.Perform();

	// Check if there is an intersection
	if (!intersection.IsDone()) { return false; }
	if (intersection.NbSolution() < 1) { return false; }
	if (intersection.PointOnShape1(1).Distance(intersection.PointOnShape2(1)) <= 1e-6)
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
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Coarse filtering of roofing structures" << std::endl;

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

					lookupValue* lookup = h->getLookup(internalValue.second);
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
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return topObjects;
}


void CJGeoCreator::reduceSurfaces(const std::vector<TopoDS_Shape>& inputShapes, bgi::rtree<Value, bgi::rstar<treeDepth_>>* shapeIdx, std::vector<SurfaceGroup>* shapeList)
{
	auto startTime = std::chrono::high_resolution_clock::now();

	// split the range over cores
	int coreCount = std::thread::hardware_concurrency();
	int coreUse = coreCount - 1;
	int splitListSize = floor(inputShapes.size() / coreUse);
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

	printTime(startTime, std::chrono::high_resolution_clock::now());
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
				auto rtreePair = std::make_pair(helperFunctions::createBBox(objectFaces[j].getFace()), shapeList->size());

				std::unique_lock<std::mutex> rtreeLock(indexInjectMutex_);
				shapeIdx->insert(rtreePair);
				shapeList->emplace_back(objectFaces[j]);
				rtreeLock.unlock();

				hasTopFaces_ = true;
			}
		}
	}
}

void CJGeoCreator::FinefilterSurfaces(const std::vector<SurfaceGroup>& shapeList) //TODO: monitor this
{
	auto startTime = std::chrono::high_resolution_clock::now();
	// split the range over cores
	int coreCount = std::thread::hardware_concurrency();
	int coreUse = 1; //coreCount - 1;
	int splitListSize = floor(shapeList.size() / coreUse);
	int voxelsGrown = 0;

	std::vector<std::thread> threadList;
	faceList_.emplace_back(std::vector<SurfaceGroup>());

	for (size_t i = 0; i < coreUse; i++)
	{
		auto startIdx = shapeList.begin() + i * splitListSize;
		auto endIdx = (i == coreUse - 1) ? shapeList.end() : startIdx + splitListSize;

		std::vector<SurfaceGroup> sublist(startIdx, endIdx);
		threadList.emplace_back([=]() {FinefilterSurface(sublist); });
	}

	for (auto& thread : threadList) {
		if (thread.joinable()) {
			thread.join();
		}
	}

	printTime(startTime, std::chrono::high_resolution_clock::now());
}

void CJGeoCreator::FinefilterSurface(const std::vector<SurfaceGroup>& shapeList)
{
	// get the faces visible from the top 
	std::vector<SurfaceGroup> cleanedShapeList;
	for (size_t i = 0; i < shapeList.size(); i++)
	{
		SurfaceGroup currentSurfaceGroup = shapeList[i];
		if (currentSurfaceGroup.testIsVisable(shapeList, true))
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
		TopoDS_Face currentFace = currentGroup.getFace();

		if (!currentGroup.isVisible()) { continue; }

		// ignore lowest if identical projected points
		double height = currentGroup.getAvHeight();
		int vertCount = currentGroup.getVertCount();

		for (size_t j = 0; j < surfaceGroupList.size(); j++)
		{
			if (i == j) { continue; }

			SurfaceGroup otherGroup = surfaceGroupList[j];
			TopoDS_Face otherFace = otherGroup.getFace();

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
			currentGroup.populateGrid(0.3); //TODO: use voxel size?
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
		double storeyElevation = storeyObject->Elevation() * h->getScaler(0);

		CJT::CityObject cityStoreyObject;
		cityStoreyObject.setType(CJT::Building_Type::BuildingStorey);

		if (storeyObject->hasName()) 
		{ 
			cityStoreyObject.setName(storeyObject->Name());
			cityStoreyObject.addAttribute("Name", storeyObject->Name());
		}
		if (storeyObject->hasLongName()) { cityStoreyObject.addAttribute("Name Long", storeyObject->LongName()); }
		cityStoreyObject.addAttribute("IFC Guid", storeyObject->GlobalId());
		cityStoreyObject.addAttribute("IFC Elevation", storeyObject->Elevation());
		cityStoreyObjects.emplace_back(std::make_shared< CJT::CityObject>(cityStoreyObject));
	}
	return cityStoreyObjects;
}

CJT::GeoObject* CJGeoCreator::makeLoD00(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 0.0 Model" << std::endl;

	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = h->getRotation();
	TopoDS_Shape floorProjection = helperFunctions::createHorizontalFace(lll, urr, rotationAngle);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	trs.SetTranslationPart(gp_Vec(0, 0, sudoSettings_->footprintElevation_));

	helperFunctions::geoTransform(&floorProjection, h->getObjectTranslation(), trs);
	CJT::GeoObject* geoObject = kernel->convertToJSON(floorProjection, "0.0");

	std::map<std::string, std::string> semanticData;
	semanticData.emplace("type", "RoofSurface");
	geoObject->appendSurfaceData(semanticData);
	geoObject->appendSurfaceTypeValue(0);
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD02(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 0.2 Model" << std::endl;
	if (!hasTopFaces_ && useRoofprints_) { return std::vector< CJT::GeoObject*>(); }

	std::vector< CJT::GeoObject*> geoObjectCollection;

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace("type", "RoofSurface");

	std::map<std::string, std::string> semanticFootData;
	semanticFootData.emplace("type", "GroundSurface");

	gp_Pnt urr = h->getUrrPoint();

	if (useRoofprints_)
	{	 // make the roof
		for (size_t i = 0; i < roofOutlineList_.size(); i++)
		{
			TopoDS_Shape currentShape = roofOutlineList_[i];

			gp_Trsf trs;
			trs.SetRotation(geoRefRotation_.GetRotation());
			if (hasFootprints_) { trs.SetTranslationPart(gp_Vec(0, 0, urr.Z())); }
			else { trs.SetTranslationPart(gp_Vec(0, 0, sudoSettings_->footprintElevation_)); }
			helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

			CJT::GeoObject* geoObject = kernel->convertToJSON(currentShape, "0.2");
			geoObject->appendSurfaceData(semanticRoofData);
			geoObject->appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}

	if (hasFootprints_) 
	{ 
		// make the footprint
		gp_Trsf trs;
		trs.SetRotation(geoRefRotation_.GetRotation());
		trs.SetTranslationPart(gp_Vec(0, 0, sudoSettings_->footprintElevation_));

		for (size_t i = 0; i < footprintList_.size(); i++)
		{
			TopoDS_Shape currentFace = footprintList_[i];
			helperFunctions::geoTransform(&currentFace, h->getObjectTranslation(), trs);

			CJT::GeoObject* geoObject = kernel->convertToJSON(currentFace, "0.2");
			geoObject->appendSurfaceData(semanticFootData);
			geoObject->appendSurfaceTypeValue(0);
			geoObjectCollection.emplace_back(geoObject);
		}
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectCollection;
}


void CJGeoCreator::makeLoD02Storeys(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale) {
	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	if (hasStoreyPrints_)
	{
		for (size_t i = 0; i < storeyPrintList_.size(); i++)
		{
			std::vector<TopoDS_Face> currentStoreyGeo = storeyPrintList_[i].getOutlines();
			std::map<std::string, std::string> currentStoreySemantic = storeyPrintList_[i].getSemanticInfo();

			for (size_t j = 0; j < storeyCityObjects.size(); j++)
			{
				if (currentStoreySemantic["IFC_Guid"] != storeyCityObjects[j]->getAttributes()["IFC Guid"])
				{
					continue;
				}

				for (size_t k = 0; k < currentStoreyGeo.size(); k++)
				{
					TopoDS_Face currentFace = currentStoreyGeo[k];
					helperFunctions::geoTransform(&currentFace, h->getObjectTranslation(), trs);
					CJT::GeoObject* geoObject = kernel->convertToJSON(currentFace, "0.2");
					geoObject->appendSurfaceTypeValue(0);
					storeyCityObjects[j]->addGeoObject(*geoObject);
				}
			}
		}
	}
}


CJT::GeoObject* CJGeoCreator::makeLoD10(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.0 Model" << std::endl;
	gp_Pnt lll = h->getLllPoint();
	gp_Pnt urr = h->getUrrPoint();
	double rotationAngle = h->getRotation();

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid bbox;
	brepBuilder.MakeSolid(bbox);

	double footprintHeight = sudoSettings_->footprintElevation_;

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
	trs.SetTranslationPart(gp_Vec(0, 0, sudoSettings_->footprintElevation_));
	helperFunctions::geoTransform(&bbox, h->getObjectTranslation(), trs);

	CJT::GeoObject* geoObject = kernel->convertToJSON(bbox, "1.0");
	std::map<std::string, std::string> grMap;
	grMap.emplace("type", "GroundSurface");
	std::map<std::string, std::string> wMap;
	wMap.emplace("type", "WallSurface");
	std::map<std::string, std::string> rMap;
	rMap.emplace("type", "RoofSurface");

	geoObject->appendSurfaceData(grMap);
	geoObject->appendSurfaceData(wMap);
	geoObject->appendSurfaceData(rMap);
	geoObject->appendSurfaceTypeValue(0);
	geoObject->appendSurfaceTypeValue(1);
	geoObject->appendSurfaceTypeValue(1);
	geoObject->appendSurfaceTypeValue(1);
	geoObject->appendSurfaceTypeValue(1);
	geoObject->appendSurfaceTypeValue(2);
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD12(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.2 Model" << std::endl;
	if (!hasTopFaces_ || !hasGeoBase_) { { return std::vector< CJT::GeoObject*>(); } }

	std::vector< CJT::GeoObject*> geoObjectList;
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
		helperFunctions::geoTransform(&currentFootprint, h->getObjectTranslation(), trs);

		BRepPrimAPI_MakePrism sweeper(currentFootprint, gp_Vec(0, 0, height), Standard_True);
		sweeper.Build();
		TopoDS_Shape extrudedShape = sweeper.Shape();

		CJT::GeoObject* geoObject = kernel->convertToJSON(extrudedShape, "1.2");
		std::map<std::string, std::string> grMap;
		grMap.emplace("type", "GroundSurface");
		std::map<std::string, std::string> wMap;
		wMap.emplace("type", "WallSurface");
		std::map<std::string, std::string> rMap;
		rMap.emplace("type", "RoofSurface");
		geoObject->appendSurfaceData(grMap);
		geoObject->appendSurfaceData(wMap);
		geoObject->appendSurfaceData(rMap);

		int counter = 0;
		for (TopExp_Explorer faceExp(extrudedShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
			TopoDS_Face face = TopoDS::Face(faceExp.Current());
			geoObject->appendSurfaceTypeValue(1);
			counter++;
		}
		geoObject->setSurfaceTypeValue(counter - 2, 0);
		geoObject->setSurfaceTypeValue(counter - 1, 2);
		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD13(helper* h, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.3 Model" << std::endl;
	std::vector< CJT::GeoObject*> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(h);
	}
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}

	std::vector<TopoDS_Shape> prismList = computePrisms(true, h);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

		CJT::GeoObject* geoObject = kernel->convertToJSON(currentShape, "1.3");
		std::map<std::string, std::string> grMap;
		grMap.emplace("type", "GroundSurface");
		std::map<std::string, std::string> wMap;
		wMap.emplace("type", "WallSurface");
		std::map<std::string, std::string> rMap;
		rMap.emplace("type", "RoofSurface");
		geoObject->appendSurfaceData(grMap);
		geoObject->appendSurfaceData(wMap);
		geoObject->appendSurfaceData(rMap);

		std::vector<int> typeValueList = getTypeValuesBySample(prismList[i], i, true);
		geoObject->setSurfaceTypeValues(typeValueList);

		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD22(helper* h, CJT::Kernel* kernel, int unitScale) 
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 2.2 Model" << std::endl;
	std::vector< CJT::GeoObject*> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(h);
	}
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}
	
	std::vector<TopoDS_Shape> prismList = computePrisms(false, h);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	for (size_t i = 0; i < prismList.size(); i++)
	{
		TopoDS_Shape currentShape = prismList[i];
		helperFunctions::geoTransform(&currentShape, h->getObjectTranslation(), trs);

		CJT::GeoObject* geoObject = kernel->convertToJSON(currentShape, "2.2");
		std::map<std::string, std::string> grMap;
		grMap.emplace("type", "GroundSurface");
		std::map<std::string, std::string> wMap;
		wMap.emplace("type", "WallSurface");
		std::map<std::string, std::string> rMap;
		rMap.emplace("type", "RoofSurface");
		geoObject->appendSurfaceData(grMap);
		geoObject->appendSurfaceData(wMap);
		geoObject->appendSurfaceData(rMap);

		std::vector<int> typeValueList = getTypeValuesBySample(prismList[i], i, false);
		geoObject->setSurfaceTypeValues(typeValueList);

		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject*>CJGeoCreator::makeLoD32(helper* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << "- Computing LoD 3.2 Model" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	std::vector< CJT::GeoObject*> geoObjectList; // final output collection

	double buffer = 1 * sudoSettings_->voxelSize_; // set the distance from the bb of the evaluated object
	int maxCastAttempts = 100; // set the maximal amout of cast attempts before the surface is considered interior

	std::vector<Value> productLookupValues;
	std::vector<voxel*> originVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;

	std::vector<voxel*> intersectingVoxels = voxelGrid_->getIntersectingVoxels();
	std::vector<voxel*> externalVoxel = voxelGrid_->getExternalVoxels();
	intersectingVoxels.insert(intersectingVoxels.end(), externalVoxel.begin(), externalVoxel.end());

	populateVoxelIndex(&voxelIndex, &originVoxels, &productLookupValues, intersectingVoxels);
	productLookupValues = makeUniqueValueList(productLookupValues);

	double gridDistance = sudoSettings_->voxelSize_;

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

	std::vector<TopoDS_Face> rawFaces;

	// evaluate which surfaces are visible
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		lookupValue* lookup = h->getLookup(productLookupValues[i].second);

		TopoDS_Shape currentShape;

		std::string lookupType = lookup->getProductPtr()->data().type()->name();

		if (lookupType == "IfcDoor" || lookupType == "IfcWindow")
		{
			if (lookup->hasCBox()) { currentShape = lookup->getCBox(); }
			else { continue; }
		}
		else { currentShape = h->getObjectShape(lookup->getProductPtr(), true); }

		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{
			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());

			if (isWireVisible(
				h,
				currentShape,
				currentFace,
				voxelIndex,
				originVoxels,
				exteriorProductIndex,
				gridDistance,
				buffer)
				)
			{
				rawFaces.emplace_back(currentFace);
				continue;
			}

			if (isSurfaceVisible(
				h, 
				currentShape, 
				currentFace, 
				voxelIndex, 
				originVoxels,
				exteriorProductIndex, 
				gridDistance, 
				buffer)
				)
			{
				rawFaces.emplace_back(currentFace);
			}
		}
	}

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());

	for (size_t i = 0; i < rawFaces.size(); i++)
	{
		TopoDS_Shape currentFace = rawFaces[i];
		helperFunctions::geoTransform(&currentFace, h->getObjectTranslation(), trs);

		CJT::GeoObject* geoObject = kernel->convertToJSON(currentFace, "3.2");
		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}


std::vector< CJT::GeoObject*>CJGeoCreator::makeV(helper* h, CJT::Kernel* kernel, int unitScale)
{
	std::cout << "- Computing LoD 5.0 Model" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	voxelGrid_->computeSurfaceSemantics(h);
	TopoDS_Shape sewedShape = voxels2Shape(0);

	std::vector< CJT::GeoObject*> geoObjectList; // final output collection
	if (sewedShape.ShapeType() == TopAbs_COMPOUND)
	{
		std::cout << "	Unable to create solid shape, multisurface stored" << std::endl;
		CJT::GeoObject* geoObject = kernel->convertToJSON(sewedShape, "5.0");
		geoObjectList.emplace_back(geoObject);

		return geoObjectList;
	}

	BRep_Builder brepBuilder;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid voxelSolid;
	brepBuilder.MakeSolid(voxelSolid);
	brepBuilder.Add(voxelSolid, sewedShape);

	gp_Trsf trs;
	trs.SetRotation(geoRefRotation_.GetRotation());
	helperFunctions::geoTransform(&voxelSolid, h->getObjectTranslation(), trs);

	CJT::GeoObject* geoObject = kernel->convertToJSON(voxelSolid, "5.0");
	geoObjectList.emplace_back(geoObject);

	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}

std::vector<CJT::CityObject> CJGeoCreator::makeVRooms(helper* h, CJT::Kernel* kernel, std::vector<std::shared_ptr<CJT::CityObject>>& storeyCityObjects, int unitScale)
{
	std::cout << "- Computing LoD 5.0 Rooms" << std::endl;

	auto startTime = std::chrono::high_resolution_clock::now();
	std::vector<CJT::CityObject> roomObjectList; // final output collection
	for (size_t i = 1; i < voxelGrid_->getRoomSize(); i++)
	{
		CJT::CityObject roomObject;
		gp_Pnt roomPoint = helperFunctions::rotatePointWorld(voxelGrid_->getPointInRoom(i), 0);
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

		for (size_t k = 0; k < qResult.size(); k++)
		{
			// find the room that point is located in

			bool encapsulating = true;
			lookupValue* lookup = h->getSpaceLookup(qResult[k].second);
			IfcSchema::IfcProduct* product = lookup->getProductPtr();

			TopoDS_Shape productShape = h->getObjectShape(product, true);
			BRepClass3d_SolidClassifier solidClassifier;
			solidClassifier.Load(productShape);
			solidClassifier.Perform(roomPoint, 0.1);

			if (!solidClassifier.State() == TopAbs_State::TopAbs_OUT) { continue; }

			std::string longName = product->data().getArgument(7)->toString();
			longName = longName.substr(1, longName.size() - 2);

			if (product->hasName()) {

				roomObject.setName(product->Name());
				roomObject.addAttribute("Name", product->Name());
				roomObject.addAttribute("IFC Guid", product->GlobalId());
			}

			roomObject.addAttribute("NameLong", longName);
			// find the storey that the room is located at

#ifdef USE_IFC4
			IfcSchema::IfcRelAggregates::list::ptr relDefByPropList = product->Decomposes();
#else
			IfcSchema::IfcRelDecomposes::list::ptr relDefByPropList = product->Decomposes();
#endif // USE_IFC4

			for (auto relPropIt = relDefByPropList->begin(); relPropIt != relDefByPropList->end(); ++relPropIt)
			{

#ifdef USE_IFC4
				IfcSchema::IfcRelAggregates* relDefByProp = *relPropIt;
#else
				IfcSchema::IfcRelDecomposes* relDefByProp = *relPropIt;
#endif // USE_IFC4

				std::string targetStoreyGuid = relDefByProp->RelatingObject()->GlobalId();
				for (size_t j = 0; j < storeyCityObjects.size(); j++)
				{
					std::shared_ptr < CJT::CityObject> storeyCityObject = storeyCityObjects[j];

					if (targetStoreyGuid != storeyCityObject->getAttributes()["IFC Guid"])
					{
						continue;
					}
					roomObject.addParent(storeyCityObject.get());
				}
			}
			break;
		}

		TopoDS_Shape sewedShape = voxels2Shape(i);

		gp_Trsf trs;
		trs.SetRotation(geoRefRotation_.GetRotation());
		helperFunctions::geoTransform(&sewedShape, h->getObjectTranslation(), trs);


		if (sewedShape.ShapeType() == TopAbs_COMPOUND)
		{
			std::cout << "	Unable to create solid shape, multisurface stored" << std::endl;
			CJT::GeoObject* geoObject = kernel->convertToJSON(sewedShape, "5.0");
			roomObject.addGeoObject(*geoObject);
			continue;
		}

		BRep_Builder brepBuilder;
		TopoDS_Shell shell;
		brepBuilder.MakeShell(shell);
		TopoDS_Solid voxelSolid;
		brepBuilder.MakeSolid(voxelSolid);
		brepBuilder.Add(voxelSolid, sewedShape);

		CJT::GeoObject* geoObject = kernel->convertToJSON(voxelSolid, "5.0");


		roomObject.setType(CJT::Building_Type::BuildingRoom);
		roomObject.addGeoObject(*geoObject);

		roomObjectList.emplace_back(roomObject);
	}

	printTime(startTime, std::chrono::high_resolution_clock::now());
	return roomObjectList;
}

TopoDS_Shape CJGeoCreator::voxels2Shape(int roomNum)
{
	BRepBuilderAPI_Sewing brepSewer;
	for (size_t i = 0; i < 6; i++)
	{
		std::vector<std::vector<TopoDS_Edge>> faceList = voxelGrid_->getDirectionalFaces(i, planeRotation_, roomNum);

		for (size_t j = 0; j < faceList.size(); j++)
		{
			std::vector<TopoDS_Wire> wireList = growWires(faceList[j]);
			std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
			std::vector<TopoDS_Face> cleanFaceList = wireCluster2Faces(cleanWireList);

			for (size_t j = 0; j < cleanFaceList.size(); j++)
			{
				brepSewer.Add(cleanFaceList[j]);
			}
		}
	}
	brepSewer.Perform();
	
	return brepSewer.SewedShape();
}

void CJGeoCreator::extractOuterVoxelSummary(CJT::CityObject* shellObject, helper* h, double footprintHeight, double geoRot)
{
	voxelGrid_->computeSurfaceSemantics(h);

	std::map<std::string, double> summaryMap;

    std::vector<voxel*> internalVoxels = voxelGrid_->getInternalVoxels();

	double voxelSize = sudoSettings_->voxelSize_;
	double voxelVolume = voxelSize * voxelSize * voxelSize;
	double shellVolume = internalVoxels.size()* voxelVolume;

	shellObject->addAttribute("Env_ex V shell volume", shellVolume);

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
		voxel* currentVoxel = internalVoxels[i];
		bool isOuterShell = currentVoxel->getIsShell();
		double zHeight = currentVoxel->getCenterPoint().get<2>();

		if (isOuterShell) { shellArea += currentVoxel->numberOfFaces() * voxelArea; }

		if (lowerEvalHeight >= zHeight)
		{
			// for sure building basement
			basementVolume += voxelVolume;

			if (!isOuterShell) { continue; }
			for (size_t j = 0; j < 6; j++)
			{
				if (currentVoxel->hasFace(j)) { basementArea += voxelArea; }
			}
			continue;
		}

		if (lowerEvalHeight < zHeight && zHeight < higherEvalHeight)
		{
			// partial building basement
			basementVolume += voxelSize * voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize);

			if (!isOuterShell) { continue; }

			footprintArea += voxelArea;

			if (currentVoxel->hasFace(0)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(1)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(2)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(3)) { basementArea += voxelSize * abs(footprintHeight - zHeight + 0.5 * voxelSize); }
			if (currentVoxel->hasFace(5)) { basementArea += voxelArea; }
		}

		if (!isOuterShell) { continue; }
		if (!currentVoxel->hasWindow()) { continue; }
		windowArea += voxelArea;
	}

	shellObject->addAttribute("Env_ex V basement shell volume", basementVolume);
	shellObject->addAttribute("Env_ex V building shell volume", shellVolume - basementVolume);
	shellObject->addAttribute("Env_ex V shell area", shellArea);
	shellObject->addAttribute("Env_ex V basement shell area", basementArea + footprintArea);
	shellObject->addAttribute("Env_ex V building shell area", shellArea - basementArea + footprintArea);
	shellObject->addAttribute("Env_ex V footprint area", footprintArea);
	shellObject->addAttribute("Env_ex V facade opening area", windowArea);
	shellObject->addAttribute("Env_ex voxelSize", voxelSize);

	gp_Pnt anchor = voxelGrid_->getAnchor();
	shellObject->addAttribute("Env_ex voxelGrid Anchor", (anchor.X(), anchor.Y(), anchor.Z() ) );
	shellObject->addAttribute("Env_ex voxelGrid rotation", voxelGrid_->getRotation() + geoRot);
}

void CJGeoCreator::extractInnerVoxelSummary(CJT::CityObject* shellObject, helper* h)
{
	double totalRoomVolume = 0;

	double voxelSize =  sudoSettings_->voxelSize_;
	double voxelVolume = voxelSize * voxelSize * voxelSize;

	std::vector<voxel*> voxelList = voxelGrid_->getVoxels();

	for (auto i = voxelList.begin(); i != voxelList.end(); i++)
	{
		voxel* currentVoxel = *i;
		if (currentVoxel->getRoomNum() > 0)
		{
			totalRoomVolume += voxelVolume;
		}
	}


	shellObject->addAttribute("Env_ex Vvolume rooms", totalRoomVolume);

}


void CJGeoCreator::populateVoxelIndex(
	bgi::rtree<Value, bgi::rstar<25>>* voxelIndex, 
	std::vector<voxel*>* originVoxels, 
	std::vector<Value>* productLookupValues,
	const std::vector<voxel*> exteriorVoxels
)
{
	for (auto voxelIt = exteriorVoxels.begin(); voxelIt != exteriorVoxels.end(); ++ voxelIt)
	{
		voxel* currentBoxel = *voxelIt;
		std::vector<Value> internalProducts = currentBoxel->getInternalProductList();
		for (size_t k = 0; k < internalProducts.size(); k++) { productLookupValues->emplace_back(internalProducts[k]); }

		// voxels that have no internal products do not have an intersection and are stored as completely external voxels
		if (internalProducts.size() == 0)
		{
			auto cornerPoints = currentBoxel->getCornerPoints(planeRotation_);
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
	for (size_t i = 0; i < productLookupValues->size(); i++)
	{
		bool isExposed = true;
		lookupValue* lookup = h->getLookup(productLookupValues->at(i).second);
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

			lookupValue* otherLookup = h->getLookup(qResult[k].second);

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

				solidClassifier.Perform(vertexPoint, 1e-6);

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


bool CJGeoCreator::isSurfaceVisible(
	helper* h,
	const TopoDS_Shape& currentShape, 
	const TopoDS_Face& currentFace, 
	const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex, 
	const std::vector<voxel*>& originVoxels,
	const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
	double gridDistance, 
	double buffer)
{
	Handle(Geom_Surface) surface = BRep_Tool::Surface(currentFace);
	
	// greate points on grid over surface
	// get the uv bounds to create a point grid on the surface
	Standard_Real uMin, uMax, vMin, vMax, wMin, wMax;
	BRepTools::UVBounds(currentFace, BRepTools::OuterWire(currentFace), uMin, uMax, vMin, vMax);

	uMin = uMin + 0.05;
	uMax = uMax - 0.05;
	vMin = vMin + 0.05;
	vMax = vMax - 0.05;

	int numUPoints = ceil(abs(uMax - uMin) / gridDistance);
	int numVPoints = ceil(abs(vMax - vMin) / gridDistance);

	if (numUPoints < 2) { numUPoints = 2; }
	else if (numUPoints > 10) { numUPoints = 10; }
	if (numVPoints < 2) { numVPoints = 2; }
	else if (numVPoints > 10) { numVPoints = 10; }

	double uStep = (uMax - uMin) / (numUPoints - 1);
	double vStep = (vMax - vMin) / (numVPoints - 1);

	// create grid
	for (int i = 0; i < numUPoints; ++i)
	{
		double u = uMin + i * uStep;

		for (int j = 0; j < numVPoints; ++j)
		{
			double v = vMin + j * vStep;
			gp_Pnt point;
			surface->D0(u, v, point);
			BRepClass_FaceClassifier faceClassifier(currentFace, point, 1e-6);
			if (faceClassifier.State() != TopAbs_ON && faceClassifier.State() != TopAbs_IN) { continue; }
			//printPoint(point);
			if (pointIsVisible(h, currentShape, currentFace, voxelIndex, originVoxels, exteriorProductIndex, point, buffer))
			{
				return true;
			}
		}
	}
	return false;
}

bool CJGeoCreator::isWireVisible(
	helper* h, 
	const TopoDS_Shape& currentShape, 
	const TopoDS_Face& currentFace, 
	const bgi::rtree<Value, 
	bgi::rstar<25>>& voxelIndex, 
	const std::vector<voxel*>& originVoxels, 
	const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
	double gridDistance, 
	double buffer)
{
	//std::cout << "x" << std::endl;
	//std::cout << BRepTools::OuterWire(currentFace).IsNull() << std::endl;
	//std::cout << "l1" << std::endl;
	// create points on offset outer wire
	BRepOffsetAPI_MakeOffset offsetter(BRepTools::OuterWire(currentFace), GeomAbs_Arc);
	//std::cout << "l2" << std::endl;
	offsetter.Perform(-0.02);

	//std::cout << offsetter.IsDone() << std::endl;

	TopExp_Explorer expl;
	TopoDS_Wire correctedOuterWire;

	//std::cout << "a" << std::endl;
	for (expl.Init(offsetter.Shape(), TopAbs_WIRE); expl.More(); expl.Next())
	{
		correctedOuterWire = TopoDS::Wire(expl.Current());
		break;
	}

	if (correctedOuterWire.IsNull()) { return false; }

	for (TopExp_Explorer edgeExp(correctedOuterWire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
		TopoDS_Edge currentEdge = TopoDS::Edge(edgeExp.Current());

		BRepAdaptor_Curve curveAdaptor(currentEdge);
		double uStart = curveAdaptor.Curve().FirstParameter();
		double uEnd = curveAdaptor.Curve().LastParameter();

		int numUPoints = ceil(abs(uStart - uEnd)) / gridDistance;

		if (numUPoints < 2) { numUPoints = 2; }
		else if (numUPoints > 10) { numUPoints = 10; }

		double uStep = abs(uStart - uEnd) / (numUPoints - 1);
		for (double u = uStart; u < uEnd; u += uStep){
			gp_Pnt point;
			curveAdaptor.D0(u, point);
			if (pointIsVisible(h, currentShape, currentFace, voxelIndex, originVoxels, exteriorProductIndex, point, buffer))
			{
				return true;
			}
		}
	}
	return false;
}


bool CJGeoCreator::pointIsVisible(helper* h,
	const TopoDS_Shape& currentShape,
	const TopoDS_Face& currentFace,
	const bgi::rtree<Value,bgi::rstar<25>>& voxelIndex,
	const std::vector<voxel*>& originVoxels,
	const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
	const gp_Pnt& point,
	const double& buffer)
{

	std::vector<Value> qResult;
	voxelIndex.query(
		bgi::intersects(
			bg::model::box <BoostPoint3D>(
				BoostPoint3D(point.X() - buffer, point.Y() - buffer, point.Z() - buffer),
				BoostPoint3D(point.X() + buffer, point.Y() + buffer, point.Z() + buffer)
				)
		),
		std::back_inserter(qResult)
	);

	for (size_t j = 0; j < qResult.size(); j++)
	{
		bool intersecting = false;

		voxel* currentBoxel = originVoxels[qResult[j].second];
		gp_Pnt voxelCore = currentBoxel->getOCCTCenterPoint();

		TopoDS_Edge ray = BRepBuilderAPI_MakeEdge(point, voxelCore);

		// check for self intersection
		for (TopExp_Explorer explorer2(currentShape, TopAbs_FACE); explorer2.More(); explorer2.Next())
		{
			const TopoDS_Face& otherFace = TopoDS::Face(explorer2.Current());
			if (otherFace.IsSame(currentFace)) { continue; }

			if (checksurfaceIntersection(ray, otherFace))
			{
				intersecting = true;
				break;
			}
		}
		if (intersecting) { continue; }

		double xMin = std::min(point.X(), voxelCore.X());
		double yMin = std::min(point.Y(), voxelCore.Y());
		double zMin = std::min(point.Z(), voxelCore.Z());

		double xMax = std::max(point.X(), voxelCore.X());
		double yMax = std::max(point.Y(), voxelCore.Y());
		double zMax = std::max(point.Z(), voxelCore.Z());

		std::vector<Value> qResult2;
		qResult2.clear();
		exteriorProductIndex.query(bgi::intersects(
			bg::model::box <BoostPoint3D>(
				BoostPoint3D(xMin, yMin, zMin),
				BoostPoint3D(xMax, yMax, zMax)
				)), std::back_inserter(qResult2));


		for (size_t k = 0; k < qResult2.size(); k++)
		{
			lookupValue* otherLookup = h->getLookup(qResult2[k].second);

			TopoDS_Shape otherShape;
			if (otherLookup->hasCBox()) { otherShape = otherLookup->getCBox(); }
			else { otherShape = h->getObjectShape(otherLookup->getProductPtr(), true); }

			if (otherShape.IsEqual(currentShape)) { continue; }

			if (surfaceIsIncapsulated(currentFace, otherShape))
			{
				return false;
			};
			if (checkShapeIntersection(ray, otherShape))
			{
				intersecting = true;
				break;
			}
		}
		if (intersecting) { continue; }
		return true;
	}
	return false;
}


CJGeoCreator::CJGeoCreator(helper* h, std::shared_ptr<SettingsCollection> settings, double vSize)
{
	// ask user for desired voxel dimensions
	if (vSize == -1)
	{
		std::string stringXYSize = "";
		std::string stringZSize = "";

		while (true)
		{
			std::cout << "Enter voxel XY dimenion (double):";
			std::cin >> stringXYSize;

			char* end = nullptr;
			double val = strtod(stringXYSize.c_str(), &end);


			if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
			{
				break;
			}
		}

		while (true)
		{
			std::cout << "Enter voxel Z dimension (double):";
			std::cin >> stringZSize;

			char* end = nullptr;
			double val = strtod(stringXYSize.c_str(), &end);


			if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
			{
				break;
			}
		}

		std::cout << std::endl;
		sudoSettings_->voxelSize_= std::stod(stringXYSize);
	}

	// compute generic voxelfield data
	sudoSettings_ = settings;
	voxelGrid_ = new VoxelGrid(h, sudoSettings_);
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
