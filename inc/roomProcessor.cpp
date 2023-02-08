#include "roomProcessor.h"

double getAvFaceHeight(TopoDS_Face face) {
	double totalZ = 0;
	int pCount = 0;
	TopExp_Explorer expl;
	for (expl.Init(face, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		totalZ += p.Z();
		pCount++;
	}
	return totalZ/pCount;
}


std::tuple<gp_Pnt, gp_Pnt> getPointsEdge(TopoDS_Edge edge) {
	TopExp_Explorer expl;

	int counter = 0;
	gp_Pnt startPoint(0, 0, 0);
	gp_Pnt endPoint(0, 0, 0);

	for (expl.Init(edge, TopAbs_VERTEX); expl.More(); expl.Next()) {
		TopoDS_Vertex currentVertex = TopoDS::Vertex(expl.Current());
		gp_Pnt currentPoint = BRep_Tool::Pnt(currentVertex);
		if (counter == 0)
		{
			startPoint = gp_Pnt(currentPoint.X(), currentPoint.Y(), 0);
			counter++;
			continue;
		}
		if (counter == 1)
		{
			endPoint = gp_Pnt(currentPoint.X(), currentPoint.Y(), 0);
			break;
		}
	}
	
	return std::make_tuple(startPoint, endPoint);
}


gp_Vec getDirEdge(TopoDS_Edge edge) {
	std::tuple<gp_Pnt, gp_Pnt> pp = getPointsEdge(edge);
	gp_Vec vec = gp_Vec(std::get<0>(pp), std::get<1>(pp));
	return vec.Normalized();
}


gp_Pnt* linearLineIntersection(Edge* edge1, Edge* edge2) {

	gp_Pnt sP1 = edge1->getProjectedStart();
	gp_Pnt eP1 = edge1->getProjectedEnd();
	gp_Pnt sP2 = edge2->getProjectedStart();
	gp_Pnt eP2 = edge2->getProjectedEnd();

	gp_Vec v1(sP1, eP1);
	v1.Normalize();
	gp_Vec v2(sP2, eP2);
	v2.Normalize();

	if (v1.IsEqual(v2, 0.001, 0.001))
	{
		return nullptr;
	}

	double x1 = sP1.X();
	double x2 = eP1.X();;
	double x3 = sP2.X();;
	double x4 = eP2.X();;

	double y1 = sP1.Y();
	double y2 = eP1.Y();
	double y3 = sP2.Y();
	double y4 = eP2.Y();

	double dom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	if (abs(dom) == 0)
	{
		return nullptr;
	}

	double xI = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / (dom);
	double yI = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / (dom);
	double buffer = 0.01;
	
	if (x1 - 0.001 < xI && x1 + 0.001 > xI && y1 - 0.001 < yI && y1 + 0.001 > yI ||
		x2 - 0.001 < xI && x2 + 0.001 > xI && y2 - 0.001 < yI && y2 + 0.001 > yI
		)
	{
		return nullptr;
	}

	if (x1 - buffer <= xI && xI <= x2 + buffer ||
		x1 + buffer >= xI && xI >= x2 - buffer)
	{
		if (y1 - buffer <= yI && yI <= y2 + buffer ||
			y1 + buffer >= yI && yI >= y2 - buffer)
		{
			if (x3 - buffer <= xI && xI <= x4 + buffer ||
				x3 + buffer >= xI && xI >= x4 - buffer)
			{
				if (y3 - buffer <= yI && yI <= y4 + buffer ||
					y3 + buffer >= yI && yI >= y4 - buffer)
				{
					return new gp_Pnt(xI, yI, 0);
				}
			}
		}
	}
	return nullptr;
}

EvaluationPoint::EvaluationPoint(gp_Pnt p)
{
	thePoint_ = p;
	evalEdge_ = BRepBuilderAPI_MakeEdge(
		p,
		gp_Pnt(p.X(), p.Y(), p.Z() + 1000)
	);
	evalLin_ = gp_Lin(p, gp_Dir(0, 0, 1000));

}

SurfaceGroup::SurfaceGroup(TopoDS_Face aFace)
{
	theFace_ = aFace;
	lllPoint_ = gp_Pnt(999999999,999999999,99999999);
	urrPoint_ = gp_Pnt(-999999999, -999999999, -99999999);

	int vertCount = 0;
	TopExp_Explorer expl;
	for (expl.Init(aFace, TopAbs_VERTEX); expl.More(); expl.Next()) 
	{ 
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt point = BRep_Tool::Pnt(vertex);

		if (point.X() > urrPoint_.X()) { urrPoint_.SetX(point.X()); }
		if (point.Y() > urrPoint_.Y()) { urrPoint_.SetY(point.Y()); }
		if (point.Z() > urrPoint_.Z()) { urrPoint_.SetZ(point.Z()); }
		if (point.X() < lllPoint_.X()) { lllPoint_.SetX(point.X()); }
		if (point.Y() < lllPoint_.Y()) { lllPoint_.SetY(point.Y()); }
		if (point.Z() < lllPoint_.Z()) { lllPoint_.SetZ(point.Z()); }

		vertCount++; 
	}
	vertCount_ = vertCount;

	llPoint_ = gp_Pnt2d(lllPoint_.X(), lllPoint_.Y());
	urPoint_ = gp_Pnt2d(urrPoint_.X(), urrPoint_.Y());

	avHeight_ = getAvFaceHeight(aFace);


}

bool SurfaceGroup::overlap(SurfaceGroup* other) {

	bool inXD = false;
	bool inYD = false;

	if (llPoint_.X() <= other->llPoint_.X() && urPoint_.X() >= other->lllPoint_.X()) { inXD = true; }
	if (other->llPoint_.X() <= llPoint_.X() && other->urrPoint_.X() > llPoint_.X()) { inXD = true; }
	if (llPoint_.Y() <= other->llPoint_.Y() && urPoint_.Y() >= other->lllPoint_.Y()) { inYD = true; }
	if (other->llPoint_.Y() <= llPoint_.Y() && other->urrPoint_.Y() >= llPoint_.Y()) { inYD = true; }

	if (inXD && inYD)
	{
		return true;
	}

	return false;
}

void SurfaceGroup::projectFace() {
	BRep_Builder brepBuilder;
	BRepBuilderAPI_MakeFace faceBuilder;
	TopExp_Explorer expl;
	TopExp_Explorer expl2;

	bool isInner = false;
	bool invalidFace = false;
	gp_Pnt lastPoint;

	for (expl.Init(theFace_, TopAbs_WIRE); expl.More(); expl.Next()) {
		TopTools_ListOfShape edgeList;
		TopoDS_Wire faceWire;
		TopoDS_Wire wire = TopoDS::Wire(expl.Current());

		bool invalidEdge = false;
		int counter = 0;
		int surfSize = 0;

		for (expl2.Init(wire, TopAbs_VERTEX); expl2.More(); expl2.Next()) {
			counter++;
			TopoDS_Vertex vertex = TopoDS::Vertex(expl2.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);
			point = gp_Pnt(point.X(), point.Y(), 0);
			if (counter % 2 == 0)
			{
				if (point.IsEqual(lastPoint, 0.0001))
				{
					invalidEdge = true;
					continue;
				}
				surfSize++;
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(lastPoint, point);
				edgeList.Append(edge);
			}

			if (invalidEdge)
			{
				invalidEdge = false;
			}
			else {
				lastPoint = point;
			}
		}

		if (surfSize < 3 && !isInner)
		{
			invalidFace = true;
			break;
		}

		if (surfSize < 3)
		{
			continue;
		}

		BRepBuilderAPI_MakeWire wireBuilder;
		wireBuilder.Add(edgeList);
		wireBuilder.Build();

		if (wireBuilder.Error() != BRepBuilderAPI_WireDone && !isInner)
		{
			invalidFace = true;
			break;
		}

		if (wireBuilder.Error() != BRepBuilderAPI_WireDone)
		{
			continue;
		}

		faceWire = wireBuilder.Wire();

		if (!isInner)
		{
			faceBuilder = BRepBuilderAPI_MakeFace(faceWire);
		}
		else
		{
			faceBuilder.Add(faceWire);
		}

		isInner = true;
	}

	TopoDS_Face tempFace;

	if (invalidFace)
	{
		theProjectedFace_ = tempFace;
	}

	theProjectedFace_ = faceBuilder.Face();
}

void SurfaceGroup::populateGrid(double distance) {
	double xRange = urrPoint_.X() - lllPoint_.X();
	double yRange = urrPoint_.Y() - lllPoint_.Y();

	double xDistance = distance;
	double yDistance = distance;

	if (xRange < distance)
	{
		xDistance = xRange / 3;
	}
	if (yRange < distance)
	{
		yDistance = yRange / 3;
	}

	int xSteps = ceil(xRange / xDistance);
	int ySteps = ceil(yRange / yDistance);

	xDistance = xRange / xSteps;
	yDistance = yRange / ySteps;

	IntCurvesFace_Intersector intersector(theFace_, 0.0001);


	for (size_t i = 0; i <= xSteps; i++)
	{
		for (size_t j = 0; j <= ySteps; j++)
		{
			gp_Pnt tempPoint = gp_Pnt(lllPoint_.X() + xDistance * i, lllPoint_.Y() + yDistance * j, 0);

			intersector.Perform(
				gp_Lin(
					gp_Pnt(lllPoint_.X() + xDistance * i, lllPoint_.Y() + yDistance * j, -1000),
					gp_Dir(0, 0, 1000)),
				-INFINITE,
				+INFINITE);

			if (intersector.NbPnt() == 1) { 
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				EvaluationPoint* evalPoint = new EvaluationPoint(intersector.Pnt(1));
				pointGrid_.emplace_back(evalPoint);
			}
		}
	}
}

bool SurfaceGroup::testIsVisable(std::vector<SurfaceGroup*> otherSurfaces, bool preFilter)
{
	TopoDS_Face currentFace = getFace();
	std::vector<EvaluationPoint*> currentGrid = getPointGrid();

	for (size_t i = 0; i < otherSurfaces.size(); i++)
	{
		SurfaceGroup* otherGroup = otherSurfaces[i];

		if (preFilter)
		{
			if (!overlap(otherGroup)) { continue; }
		}


		TopoDS_Face otherFace = otherGroup->getFace();

		if (currentFace.IsEqual(otherFace)) { continue; }

		IntCurvesFace_Intersector intersector(otherFace, 0.0001);

		for (size_t k = 0; k < currentGrid.size(); k++)
		{
			EvaluationPoint* currentEvalPoint = currentGrid[k];

			if (!currentEvalPoint->isVisible()) { continue; }

			intersector.Perform(
				currentEvalPoint->getEvalLine(),
				-0,
				+INFINITE);

			if (intersector.NbPnt() > 0)
			{
				currentEvalPoint->setInvisible();
				continue;
			}

			TopExp_Explorer expl;
			for (expl.Init(otherFace, TopAbs_EDGE); expl.More(); expl.Next())
			{
				TopoDS_Edge otherEdge = TopoDS::Edge(expl.Current());
				BRepExtrema_DistShapeShape distanceCalc(otherEdge, currentEvalPoint->getEvalEdge());

				if (distanceCalc.Value() < 0.001)
				{
					currentEvalPoint->setInvisible();
					break;
				}
			}
		}
	}

	for (size_t i = 0; i < currentGrid.size(); i++)
	{
		if (currentGrid[i]->isVisible())
		{
			return true;
		}
	}

	visibility_ = false;
	return false;
}

std::vector<Edge*> CJGeoCreator::getUniqueEdges(TopoDS_Shape& flattenedEdges)
{
	TopExp_Explorer expl;
	std::vector<Edge*> UniqueEdgeList;

	for (expl.Init(flattenedEdges, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		std::tuple<gp_Pnt, gp_Pnt> points = getPointsEdge(currentEdge);
		gp_Pnt startPoint = std::get<0>(points);
		gp_Pnt endPoint = std::get<1>(points);

		bool dub = false;
		for (size_t i = 0; i < UniqueEdgeList.size(); i++)
		{
			gp_Pnt otherStartPoint = UniqueEdgeList[i]->getProjectedStart();
			gp_Pnt otherEndPoint = UniqueEdgeList[i]->getProjectedEnd();

			if (startPoint.IsEqual(otherStartPoint, 0.0001) && endPoint.IsEqual(otherEndPoint, 0.0001) ||
				endPoint.IsEqual(otherStartPoint, 0.0001) && startPoint.IsEqual(otherEndPoint, 0.0001))
			{
				dub = true;
			}
		}
		if (!dub)
		{
			UniqueEdgeList.emplace_back(new Edge(currentEdge));
		}
	}
	return UniqueEdgeList;
}


std::vector<Edge*> CJGeoCreator::mergeOverlappingEdges(std::vector<Edge*>& uniqueEdges)//, std::vector<TopoDS_Face> flatFaceList) 
{
	
	double buffer = 0.01; //TODO: have it scale with units
	std::vector<int> discardIndx;

	TopExp_Explorer expl;
	// merge lines that are on the same plane
	std::vector<Edge*> cleanedEdgeList;
	std::vector<int> evalList(uniqueEdges.size());
	for (size_t i = 0; i < uniqueEdges.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		gp_Pnt startPoint = uniqueEdges[i]->getProjectedStart();
		gp_Pnt endPoint = uniqueEdges[i]->getProjectedEnd();
		gp_Lin2d line = gce_MakeLin2d(
			gp_Pnt2d(startPoint.X(), startPoint.Y()),
			gp_Pnt2d(endPoint.X(), endPoint.Y())
		).Value();

		gp_Vec2d dir(gp_Pnt2d(startPoint.X(), startPoint.Y()), gp_Pnt2d(endPoint.X(), endPoint.Y()));
		dir.Normalize();

		std::vector<gp_Pnt> mergeList;
		std::vector<int> evalIndxList;

		for (size_t j = 0; j < uniqueEdges.size(); j++)
		{
			if (i == j) { continue; }
			if (evalList[j] == 1) { continue; }

			gp_Pnt otherStartPoint = uniqueEdges[j]->getProjectedStart();
			gp_Pnt otherEndPoint = uniqueEdges[j]->getProjectedEnd();

			gp_Vec2d otherDir(gp_Pnt2d(otherStartPoint.X(), otherStartPoint.Y()), gp_Pnt2d(otherEndPoint.X(), otherEndPoint.Y()));
			otherDir.Normalize();

			if (!dir.IsParallel(otherDir, 0.0001) && !dir.IsParallel(otherDir.Reversed(), 0.0001))
			{
				continue;
			}

			if (!startPoint.IsEqual(otherStartPoint, 0.0001))
			{
				if (!dir.IsParallel(
					gp_Vec2d(
						gp_Pnt2d(
							startPoint.X(), startPoint.Y()
						),
						gp_Pnt2d(
							otherStartPoint.X(), otherStartPoint.Y()
						)
					),
					0.0001
				))
				{
					continue;
				}
			}
			evalIndxList.emplace_back(j);
		}

		if (evalIndxList.size() == 0)
		{
			evalList[i] = 1;
			if (startPoint.Distance(endPoint) > 0.1) //TODO: make smarter
			{
				//if (isOuterEdge(edgeList[i], flatFaceList))
				//{
					cleanedEdgeList.emplace_back(uniqueEdges[i]);
				//}
			}
			continue;
		}

		discardIndx.emplace_back(i);

		// flip main merging ege
		if (abs(dir.X()) >= abs(dir.Y()))
		{
			//std::cout << "X based" << std::endl;
			if (startPoint.X() > endPoint.X())
			{
				gp_Pnt tempPoint = startPoint;
				startPoint = endPoint;
				endPoint = tempPoint;
			}
		}
		if (abs(dir.X()) < abs(dir.Y()))
		{
			if (startPoint.Y() > endPoint.Y())
			{
				gp_Pnt tempPoint = startPoint;
				startPoint = endPoint;
				endPoint = tempPoint;
			}
		}

		while (true)
		{
			int grow = 0;
			for (size_t j = 0; j < evalIndxList.size(); j++)
			{
				int edgeIdx = evalIndxList[j];

				if (evalList[edgeIdx] == 1) { continue; }

				gp_Pnt otherStartPoint = uniqueEdges[edgeIdx]->getProjectedStart();
				gp_Pnt otherEndPoint = uniqueEdges[edgeIdx]->getProjectedEnd();

				if (abs(dir.X()) >= abs(dir.Y()))
				{
					if (otherStartPoint.X() > otherEndPoint.X())
					{
						gp_Pnt tempPoint = otherStartPoint;
						otherStartPoint = otherEndPoint;
						otherEndPoint = tempPoint;
					}
				}
				if (abs(dir.X()) < abs(dir.Y()))
				{
					if (otherStartPoint.Y() > otherEndPoint.Y())
					{
						gp_Pnt tempPoint = otherStartPoint;
						otherStartPoint = otherEndPoint;
						otherEndPoint = tempPoint;
					}
				}

				double x1 = startPoint.X();
				double x2 = endPoint.X();
				double x3 = otherStartPoint.X();
				double x4 = otherEndPoint.X();

				if (abs(dir.X()) < abs(dir.Y()))
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
		cleanedEdgeList.emplace_back(new Edge(edge));
	}

	for (size_t i = 0; i < discardIndx.size(); i++) { delete uniqueEdges[discardIndx[i]]; }
	return cleanedEdgeList;
}


std::vector<Edge*> CJGeoCreator::splitIntersectingEdges(std::vector<Edge*>& edges) {
	
	std::vector<Edge*> splitEdgeList;
	std::vector<int> discardIndx;
	
	for (size_t i = 0; i < edges.size(); i++)
	{
		std::vector<gp_Pnt*> intPoints;
		gp_Pnt startPoint = edges[i]->getProjectedStart();
		gp_Pnt endPoint = edges[i]->getProjectedEnd();

		for (size_t j = 0; j < edges.size(); j++)
		{
			if (i == j) { continue; }
			gp_Pnt* intersection = linearLineIntersection(edges[i], edges[j]);

			if (intersection == nullptr) { continue; }


			intPoints.emplace_back(intersection);
			//printPoint(*intersection);
		}

		if (intPoints.size() == 0) 
		{ 
			splitEdgeList.emplace_back(edges[i]);
			continue;
		}
		discardIndx.emplace_back(i);

		std::vector<gp_Pnt*> cleanedPoints;
		for (size_t j = 0; j < intPoints.size(); j++)
		{
			bool dub = false;
			for (size_t k = 0; k < cleanedPoints.size(); k++)
			{
				if (intPoints[j]->IsEqual(*cleanedPoints[k], 0.001))
				{
					dub = true;
					break;
				}
			}
			if (!dub)
			{
				cleanedPoints.emplace_back(intPoints[j]);
			}
		}

		std::vector<int> evaluated(cleanedPoints.size());
		gp_Pnt currentPoint = startPoint;
		while (true)
		{
			double smallestDistance = 99999999;
			int nextPoint;
			bool found = false;
			for (size_t i = 0; i < cleanedPoints.size(); i++)
			{
				if (evaluated[i] == 1)
				{
					continue;
				}

				double distance = currentPoint.Distance(*cleanedPoints[i]);
				if (distance < smallestDistance)
				{
					smallestDistance = distance;
					nextPoint = i;
					found = true;
				}
			}

			if (found)
			{
				evaluated[nextPoint] = 1;

				if (currentPoint.IsEqual(*cleanedPoints[nextPoint], 0.0001))
				{
					currentPoint = *cleanedPoints[nextPoint];
					continue;
				}

				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentPoint, *cleanedPoints[nextPoint]);
				splitEdgeList.emplace_back(new Edge(edge));
				currentPoint = *cleanedPoints[nextPoint];

			}
			else {
				if (currentPoint.IsEqual(endPoint, 0.0001))
				{
					break;
				}
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentPoint, endPoint);
				splitEdgeList.emplace_back(new Edge(edge));
				break;
			}
		}
	}
	
	for (size_t i = 0; i < discardIndx.size(); i++) { delete edges[discardIndx[i]]; }
	return splitEdgeList;
}


TopoDS_Face CJGeoCreator::getFlatFace(TopoDS_Face face) {
	BRep_Builder brepBuilder;
	BRepBuilderAPI_MakeFace faceBuilder;
	TopExp_Explorer expl;
	TopExp_Explorer expl2;

	bool isInner = false;
	bool invalidFace = false;
	gp_Pnt lastPoint;

	for (expl.Init(face, TopAbs_WIRE); expl.More(); expl.Next()) {
		TopTools_ListOfShape edgeList;
		TopoDS_Wire faceWire;
		TopoDS_Wire wire = TopoDS::Wire(expl.Current());

		bool invalidEdge = false;
		int counter = 0;
		int surfSize = 0;

		for (expl2.Init(wire, TopAbs_VERTEX); expl2.More(); expl2.Next()) {
			counter++;
			TopoDS_Vertex vertex = TopoDS::Vertex(expl2.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);
			point = gp_Pnt(point.X(), point.Y(), 0);
			if (counter % 2 == 0)
			{
				if (point.IsEqual(lastPoint, 0.0001))
				{
					invalidEdge = true;
					continue;
				}
				surfSize++;
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(lastPoint, point);
				edgeList.Append(edge);
			}

			if (invalidEdge)
			{
				invalidEdge = false;
			}
			else {
				lastPoint = point;
			}
		}

		if (surfSize < 3 && !isInner)
		{
			invalidFace = true;
			break;
		}

		if (surfSize < 3)
		{
			continue;
		}

		BRepBuilderAPI_MakeWire wireBuilder;
		wireBuilder.Add(edgeList);
		wireBuilder.Build();

		if (wireBuilder.Error() != BRepBuilderAPI_WireDone && !isInner)
		{
			invalidFace = true;
			break;
		}

		if (wireBuilder.Error() != BRepBuilderAPI_WireDone)
		{
			continue;
		}

		faceWire = wireBuilder.Wire();

		if (!isInner)
		{
			faceBuilder = BRepBuilderAPI_MakeFace(faceWire);
		}
		else
		{
			faceBuilder.Add(faceWire);
		}

		isInner = true;
	}

	TopoDS_Face tempFace;

	if (invalidFace)
	{
		return tempFace;
	}

	return faceBuilder.Face();
}


std::vector<Edge*> CJGeoCreator::makeJumbledGround() {
	
	gp_Ax2 axis(
		gp_Pnt(0, 0, 0), 
		gp_Dir(0, 0, 100)
	);
	HLRBRep_Algo* Edgeprojector = new HLRBRep_Algo();
	Edgeprojector->Projector(HLRAlgo_Projector(axis));
	
	BOPAlgo_Builder aBuilder;
	TopTools_ListOfShape aLSObjects;

	for (size_t i = 0; i < projectedFaceList_.size(); i++) { Edgeprojector->Add(*projectedFaceList_[i]); }
	Edgeprojector->Update();
	HLRBRep_HLRToShape projectToShape(Edgeprojector);

	TopoDS_Shape flattenedEdges = projectToShape.CompoundOfEdges(HLRBRep_Undefined, true, false);

	std::vector<Edge*> uniqueEdges = getUniqueEdges(flattenedEdges);
	std::vector<Edge*> cleanedEdges = mergeOverlappingEdges(uniqueEdges);
	return splitIntersectingEdges(cleanedEdges);
}

bool CJGeoCreator::isOuterEdge(Edge* currentEdge, std::vector<TopoDS_Face*> flatFaceList)
{
	gp_Pnt startPoint = currentEdge->getProjectedStart();
	gp_Pnt endPoint = currentEdge->getProjectedEnd();

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

	bool hasEval1 = false;
	bool hasEval2 = false;
	for (size_t i = 0; i < flatFaceList.size(); i++)
	{
		TopoDS_Face* evalFace = flatFaceList[i];
		if (!hasEval1)
		{
			BRepExtrema_DistShapeShape distanceCalc(
				*evalFace,
				BRepBuilderAPI_MakeVertex(
					gp_Pnt(
						evalPoint1.X(),
						evalPoint1.Y(),
						0
					)
				).Vertex());

			distanceCalc.Perform();
			double distance = distanceCalc.Value();

			if (distance < 0.0001)
			{
				hasEval1 = true;
			}
		}

		if (!hasEval2)
		{
			BRepExtrema_DistShapeShape distanceCalc(
				*evalFace,
				BRepBuilderAPI_MakeVertex(
					gp_Pnt(
						evalPoint2.X(),
						evalPoint2.Y(),
						0
					)
				).Vertex());

			distanceCalc.Perform();
			double distance = distanceCalc.Value();

			if (distance < 0.0001)
			{
				hasEval2 = true;
			}
		}

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

std::vector<TopoDS_Edge> CJGeoCreator::getOuterEdges(std::vector<Edge*> edgeList, std::vector<TopoDS_Face*> faceList) {
	std::vector<TopoDS_Edge> outerEdgeList;
	TopExp_Explorer expl;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		TopoDS_Edge* currentEdge = edgeList[i]->getEdge();
		if (isOuterEdge(edgeList[i], faceList)) { outerEdgeList.emplace_back(*currentEdge); }
	}

	return outerEdgeList;
}


std::vector<TopoDS_Face> CJGeoCreator::outerEdges2Shapes(std::vector<TopoDS_Edge> edgeList, CJT::Kernel* kernel)
{
	BRepBuilderAPI_MakeFace faceBuilder;

	TopoDS_Edge currentEdge = edgeList[0];
	std::vector<int> evaluated(edgeList.size());
	evaluated[0] = 1;
	gp_Pnt connectionPoint = std::get<1>(getPointsEdge(edgeList[0]));
	gp_Pnt originPoint = std::get<0>(getPointsEdge(edgeList[0]));

	std::vector<TopoDS_Edge> orderedEdgeList;
	orderedEdgeList.emplace_back(currentEdge);

	std::vector<TopoDS_Face> geoObjectList;

	bool con = true;
	while (con)
	{
		TopoDS_Wire faceWire;
		BRepBuilderAPI_MakeWire wireMaker;

		bool Finalized = true;
		bool loopFound = false;
		con = false;

		while (true)
		{
			int listLength = orderedEdgeList.size();

			double distance = 999999999;
			gp_Pnt backupPoint;

			for (size_t i = 0; i < edgeList.size(); i++) // select and order loop's edges
			{
				if (evaluated[i] == 1)
				{
					continue;
				}

				std::tuple<gp_Pnt, gp_Pnt> pp = getPointsEdge(edgeList[i]);
				gp_Pnt otherStart = std::get<0>(pp);
				gp_Pnt otherEnd = std::get<1>(pp);

				double distance2Start = connectionPoint.Distance(otherStart);
				double distance2End = connectionPoint.Distance(otherEnd);

				if (distance2Start < distance)
				{
					distance = distance2Start;
					backupPoint = otherStart;
				}

				if (distance2End < distance)
				{
					distance = distance2End;
					backupPoint = otherEnd;
				}

				if (connectionPoint.IsEqual(otherStart, 0.0001))
				{
					orderedEdgeList.emplace_back(edgeList[i]);
					connectionPoint = otherEnd;
					evaluated[i] = 1;
					break;
				}
				else if (connectionPoint.IsEqual(otherEnd, 0.0001))
				{
					orderedEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(otherEnd, otherStart));
					connectionPoint = otherStart;
					evaluated[i] = 1;
					break;
				}
			}

			if (originPoint.IsEqual(connectionPoint, 0.0001))
			{
				loopFound = true;
				break;
			}

			if (listLength == orderedEdgeList.size())
			{
				if (distance < 0.1)
				{
					orderedEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(connectionPoint, backupPoint));
					connectionPoint = backupPoint;
				}
				else
				{
					break;
				}
			}
		}

		if (!loopFound)
		{
			for (size_t j = 0; j < edgeList.size(); j++)
			{
				if (evaluated[j] == 0)
				{
					connectionPoint = std::get<1>(getPointsEdge(edgeList[j]));
					originPoint = std::get<0>(getPointsEdge(edgeList[j]));

					orderedEdgeList.clear();
					orderedEdgeList.emplace_back(edgeList[j]);

					evaluated[j] = 1;
					con = true;
					loopFound = false;
					break;
				}
			}
			if (!con)
			{
				break;
			}
			continue;
		}

		int count = 0;
		std::vector<int> merged(orderedEdgeList.size());
		gp_Vec evalVec = getDirEdge(orderedEdgeList[0]);
		for (size_t i = 0; i < orderedEdgeList.size(); i++) // pick corner point as start point
		{
			TopoDS_Edge currentEdge = orderedEdgeList[i];
			gp_Vec currentVec = getDirEdge(currentEdge);

			gp_Pnt startPoint = std::get<0>(getPointsEdge(currentEdge));
			gp_Pnt endPoint = std::get<1>(getPointsEdge(currentEdge));



			/*if (!currentVec.IsParallel(evalVec, 0.01)) {
				std::vector<TopoDS_Edge> splitToFront(orderedEdgeList.begin() + i + 1, orderedEdgeList.end());
				std::vector<TopoDS_Edge> splitToBack(orderedEdgeList.begin(), orderedEdgeList.begin() + i);

				splitToFront.insert(splitToFront.end(), splitToBack.begin(), splitToBack.end());
				orderedEdgeList = splitToFront;
				break;
			}*/
		}
		gp_Pnt connection = std::get<0>(getPointsEdge(orderedEdgeList[0]));

		for (size_t i = 0; i < orderedEdgeList.size(); i++) // merge parralel 
		{
			if (merged[i] == 1) { continue; }
			merged[i] = 1;

			TopoDS_Edge currentEdge = orderedEdgeList[i];
			gp_Vec currentVec = getDirEdge(currentEdge);
			gp_Pnt endPoint = std::get<1>(getPointsEdge(currentEdge));

			for (size_t j = i + 1; j < orderedEdgeList.size(); j++)
			{
				if (merged[j] == 1) { continue; }

				TopoDS_Edge otherEdge = orderedEdgeList[j];
				gp_Vec otherVec = getDirEdge(otherEdge);

				if (!currentVec.IsParallel(otherVec, 0.01)) { break; }

				merged[j] = 1;
				endPoint = std::get<1>(getPointsEdge(otherEdge));
			}

			wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));


			if (wireMaker.Error() != BRepBuilderAPI_WireDone)
			{
				for (size_t j = 0; j < edgeList.size(); j++)
				{
					if (evaluated[j] == 0)
					{
						connectionPoint = std::get<1>(getPointsEdge(edgeList[j]));
						originPoint = std::get<0>(getPointsEdge(edgeList[j]));

						orderedEdgeList.clear();
						orderedEdgeList.emplace_back(edgeList[j]);
						con = true;
						evaluated[j] = 1;
						Finalized = false;

						break;
					}
				}
				break;
			}

			faceWire = wireMaker.Wire();
			connection = endPoint;

			count++;
		}

		if (!Finalized)
		{
			continue;
		}

		orderedEdgeList.clear();

		if (!faceWire.IsNull() && count >= 3)
		{
			faceBuilder = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), faceWire);

			if (faceBuilder.Error() == BRepBuilderAPI_FaceDone)
			{
				footPrintList_.emplace_back(faceBuilder.Face());
				//geoObjectList.emplace_back(faceBuilder.Face());
			}
		}

		for (size_t i = 0; i < edgeList.size(); i++)
		{
			if (evaluated[i] == 0)
			{
				connectionPoint = std::get<1>(getPointsEdge(edgeList[i]));
				originPoint = std::get<0>(getPointsEdge(edgeList[i]));

				orderedEdgeList.emplace_back(edgeList[i]);

				con = true;
				evaluated[i] = 1;
				break;
			}
		}
	}

	// test which surfaces are inner loops
	//TODO: make function
	std::vector<double> areaList;
	for (size_t i = 0; i < footPrintList_.size(); i++)
	{
		TopoDS_Face currentFootprint = footPrintList_[i];

		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(currentFootprint, gprops); // Stores results in gprops
		double area = gprops.Mass();

		areaList.emplace_back(area);
	}


	std::vector<TopoDS_Face> orderedFootprintList;
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

		orderedFootprintList.emplace_back(footPrintList_[evalIdx]);
		ordered[evalIdx] = 1;
	}

	footPrintList_.clear();

	std::vector<int> clipped(areaList.size());
	for (size_t i = 0; i < orderedFootprintList.size(); i++)
	{
		if (clipped[i] == 1) { continue; }
		clipped[i] = 1;

		TopoDS_Face clippedFace = orderedFootprintList[i];

		for (size_t j = i + 1;  j < orderedFootprintList.size();  j++)
		{
			if (clipped[j] == 1) { continue; }

			bool overlap = false;

			TopExp_Explorer expl;
			for (expl.Init(orderedFootprintList[j], TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());

				BRepExtrema_DistShapeShape distanceCalc(clippedFace, vertex);
				distanceCalc.Perform();
				double distance = distanceCalc.Value();

				if (distance < 0.001)
				{
					overlap = true;
					clipped[j] = 1;
				}
				break;
			}

			if (!overlap) { continue; }

			GProp_GProps gprop;
			BRepGProp::VolumeProperties(orderedFootprintList[j], gprop);
			double mass = abs(gprop.Mass());

			if (mass < 5) { continue; }

			for (expl.Init(orderedFootprintList[j], TopAbs_WIRE); expl.More(); expl.Next())
			{
				BRepBuilderAPI_MakeFace merger = BRepBuilderAPI_MakeFace(clippedFace, TopoDS::Wire(expl.Current()));
				clippedFace = merger.Face();
				break;
			}
		}

		geoObjectList.emplace_back(clippedFace);
	}
	return geoObjectList;
}


void CJGeoCreator::initializeBasic(helperCluster* cluster) {
	// generate data required for most exports
	std::vector<SurfaceGroup*> shapeList;
	std::vector<TopoDS_Shape> filteredFaces = getTopObjects(cluster);

	for (size_t i = 0; i < filteredFaces.size(); i++)
	{
		std::vector<SurfaceGroup*>  objectFaces = getXYFaces(filteredFaces[i]);
		for (size_t j = 0; j < objectFaces.size(); j++)
		{
			shapeList.emplace_back(objectFaces[j]);
			hasTopFaces_ = true;
		}
	}

	for (size_t i = 0; i < shapeList.size(); i++)
	{
		SurfaceGroup* currentSurfaceGroup = shapeList[i];
		if (currentSurfaceGroup->testIsVisable(shapeList, true))
		{
			topFaceList_.emplace_back(shapeList[i]);
		}
	}

	for (size_t i = 0; i < topFaceList_.size(); i++)
	{
		SurfaceGroup* currentSurfaceGroup = topFaceList_[i];
		projectedFaceList_.emplace_back(new TopoDS_Face(topFaceList_[i]->getProjectedFace()));
		hasProjectedFaces_ = true;
	}
}


TopoDS_Face makeFace(std::vector<gp_Pnt> voxelPointList, std::vector<int> pointFaceIndx) {
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

template<typename T>
T CJGeoCreator::linearToRelative(int i) {
	double x = i % xRelRange_;
	double z = round(i / (xRelRange_ * yRelRange_)) - round(i / (xRelRange_ * yRelRange_) % 1);
	double y = (i - x) / xRelRange_ - z * yRelRange_;

	return T(x, y, z);
}
std::vector<int> CJGeoCreator::getNeighbours(int voxelIndx, bool connect6)
{
	std::vector<int> neightbours;
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool xSmall = loc3D.X() - 1 >= 0;
	bool xBig = loc3D.X() + 1 < xRelRange_;

	bool ySmall = loc3D.Y() - 1 >= 0;
	bool yBig = loc3D.Y() + 1 < yRelRange_;

	bool zSmall = loc3D.Z() - 1 >= 0;
	bool zBig = loc3D.Z() + 1 < zRelRange_;

	// connectivity
	if (xSmall) 
	{ 
		neightbours.emplace_back(voxelIndx - 1); 

		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - 1); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - 1); }
		}

	}
	if (xBig) 
	{ 
		neightbours.emplace_back(voxelIndx + 1); 

		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + 1); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + 1); }
		}
	}
	if (ySmall)
	{ 
		neightbours.emplace_back(voxelIndx - xRelRange_); 
		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - xRelRange_); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - xRelRange_); }
		}
	}
	if (yBig) 
	{ 
		neightbours.emplace_back(voxelIndx + xRelRange_); 
		if (!connect6)
		{
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ + xRelRange_); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ + xRelRange_); }
		}
	}

	if (zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)); }
	if (zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_ )); }

	if (!connect6)
	{
		return neightbours;
	}

	if (xSmall && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
	if (xBig && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ + 1); }

	if (xBig && ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ + 1); }
	if (xSmall && yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ - 1); }

	if (xSmall && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_) - xRelRange_ - 1 ); }
	if (xBig && ySmall && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)- xRelRange_ + 1); }

	if (xSmall && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_) + xRelRange_ - 1); }
	if (xBig && yBig && zSmall) { neightbours.emplace_back(voxelIndx - (xRelRange_) * (yRelRange_)+ xRelRange_ + 1); }

	if (xSmall && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) + xRelRange_ - 1); }
	if (xBig && yBig && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) + xRelRange_ + 1); }

	if (xSmall && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) - xRelRange_ - 1); }
	if (xBig && ySmall && zBig) { neightbours.emplace_back(voxelIndx + (xRelRange_) * (yRelRange_) - xRelRange_ + 1); }

	return neightbours;
}


BoostPoint3D CJGeoCreator::relPointToWorld(BoostPoint3D p)
{
	double xCoord = anchor_.X() + (bg::get<0>(p) * voxelSize_);
	double yCoord = anchor_.Y() + (bg::get<1>(p) * voxelSize_);
	double zCoord = anchor_.Z() + (bg::get<2>(p) * voxelSizeZ_);

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

BoostPoint3D CJGeoCreator::relPointToWorld(int px, int py, int pz)
{
	double xCoord = px * voxelSize_ + voxelSize_ / 2;
	double yCoord = py * voxelSize_ + voxelSize_ / 2;
	double zCoord = pz * voxelSizeZ_ + voxelSizeZ_ / 2;

	return BoostPoint3D(xCoord, yCoord, zCoord);
}

std::vector<TopoDS_Face> CJGeoCreator::getPartialFaces(std::vector<int> roomIndx, int voxelIndx)
{
	gp_Pnt loc3D = linearToRelative<gp_Pnt>(voxelIndx);

	bool faceLeft = true;
	bool faceRight = true;
	bool faceFront = true;
	bool faceBack = true;
	bool faceUp = true;
	bool faceDown = true;

	// leftFace
	for (size_t i = 0; i < roomIndx.size(); i++)
	{
		if (roomIndx[i] == voxelIndx - 1) { faceLeft = false; }
		if (roomIndx[i] == voxelIndx + 1) { faceRight = false; }
		if (roomIndx[i] == voxelIndx - xRelRange_) { faceFront = false; }
		if (roomIndx[i] == voxelIndx + xRelRange_) { faceBack = false; }
		if (roomIndx[i] == voxelIndx - (xRelRange_) * (yRelRange_)) { faceDown = false; }
		if (roomIndx[i] == voxelIndx + (xRelRange_) * (yRelRange_)) { faceUp = false; }
	}

	if (!faceLeft && !faceRight && !faceFront && !faceBack && !faceUp && !faceDown){ return {};}
	voxel* currentBoxel = VoxelLookup_[voxelIndx];

	std::vector<TopoDS_Face> faceList;
	std::vector<gp_Pnt> voxelPointList = currentBoxel->getCornerPoints(planeRotation_);
	std::vector<std::vector<int>> voxelFaceList = currentBoxel->getVoxelFaces();

	if (faceLeft) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[3])); }
	if (faceRight) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[1])); }
	if (faceFront) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[0])); }
	if (faceBack) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[2])); }
	if (faceUp) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[4])); }
	if (faceDown) { faceList.emplace_back(makeFace(voxelPointList, voxelFaceList[5])); }

	return faceList;
}

TopoDS_Face CJGeoCreator::getLowestFace(TopoDS_Shape shape)
{
	TopExp_Explorer expl;
	std::vector<TopoDS_Face> faceList;
	double lowestZ = 9999;
	int lowestFaceIndx = -1;

	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) { faceList.emplace_back(TopoDS::Face(expl.Current())); }

	for (int j = 0; j < faceList.size(); j++)
	{
		int aEdges = 0;
		double totalZ = 0;

		for (expl.Init(faceList[j], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			aEdges++;
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			totalZ += p.Z();
		}

		if (lowestZ > (totalZ / aEdges))
		{
			lowestZ = totalZ / aEdges;
			lowestFaceIndx = j;
		}
	}
	return faceList[lowestFaceIndx];
}


void CJGeoCreator::addVoxel(int indx, helperCluster* cluster)
{
	auto midPoint = relPointToWorld(linearToRelative<BoostPoint3D>(indx));
	voxel* boxel = new voxel(midPoint, voxelSize_, voxelSizeZ_);
	VoxelLookup_.emplace(indx, boxel);
}

void CJGeoCreator::outputFieldToFile()
{
	std::ofstream storageFile;
	storageFile.open("D:/Documents/Uni/Thesis/sources/Models/exports/voxels.txt");
	for (auto it = VoxelLookup_.begin(); it != VoxelLookup_.end(); ++ it )
	{
		std::vector<gp_Pnt> pointList = it->second->getCornerPoints(planeRotation_);

		//if (it->second->getRoomNumbers().size() == 0) { continue; }
		if (!it->second->getIsInside()) { continue; }
		if (it->second->getIsIntersecting()) { continue; }

		for (size_t k = 0; k < pointList.size(); k++)
		{
			storageFile << pointList[k].X() << ", " << pointList[k].Y() << ", " << pointList[k].Z() << std::endl;
		}

		//storageFile << it->second->getRoomNumbers().back() << std::endl;
		//storageFile << "1" << std::endl;

		storageFile << "\n";
	}

	storageFile << -planeRotation_;
	storageFile.close();
}

std::vector<TopoDS_Shape> CJGeoCreator::getTopObjects(helperCluster* cluster)
{
	std::vector<int> boxelIdx = getTopBoxelIndx();
	std::vector<IfcSchema::IfcProduct*> topProducts;
	std::vector<TopoDS_Shape> topObjects;

	for (size_t i = 0; i < boxelIdx.size(); i++)
	{
		double step = 2;
		double downstep = 2;
		bool found = false;

		int idx = boxelIdx[i];
		voxel* boxel = VoxelLookup_[idx];
		std::vector<gp_Pnt> pointList = boxel->getCornerPoints(planeRotation_);
		while (true)
		{
			BoostPoint3D lll(pointList[0].X(), pointList[0].Y(), pointList[0].Z() - downstep);
			BoostPoint3D urr(pointList[4].X(), pointList[4].Y(), pointList[4].Z());
			bg::model::box<BoostPoint3D> boostBoxel = bg::model::box<BoostPoint3D>(lll, urr);
			std::vector<Value> qResult;

			for (int j = 0; j < cluster->getSize(); j++)
			{
				qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
				cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(boostBoxel), std::back_inserter(qResult));

				if (qResult.size() == 0) { continue; }
				found = true;

				for (size_t k = 0; k < qResult.size(); k++)
				{
					bool dub = false;

					LookupValue lookup = cluster->getHelper(j)->getLookup(qResult[k].second);
					IfcSchema::IfcProduct* product = std::get<0>(lookup);

					if (product->data().type()->name() != "IfcSlab" &&
						product->data().type()->name() != "IfcWallStandardCase" &&
						product->data().type()->name() != "IfcWall" &&
						product->data().type()->name() != "IfcRoof")
					{
						continue;
					}

					if (!product->hasRepresentation()) { continue; }

					for (size_t l = 0; l < topProducts.size(); l++)
					{
						IfcSchema::IfcProduct* otherproduct = topProducts[l];

						if (otherproduct == product)
						{
							dub = true;
							break;
						}
					}

					if (!dub)
					{
						topProducts.emplace_back(product);
						topObjects.emplace_back(cluster->getHelper(j)->getObjectShape(product, false));
					}
				}
			}

			if (found) { break; }

			downstep = downstep + step;

			if (pointList[0].Z() - (downstep + step) < cluster->getLllPoint().Z()) { break; }
		}
	}

	return topObjects;
}

std::vector<int> CJGeoCreator::getTopBoxelIndx() {

	std::vector<int> voxelIndx;

	for (size_t i = xRelRange_ * yRelRange_ * zRelRange_ - xRelRange_ * yRelRange_; i < xRelRange_ * yRelRange_ * zRelRange_ - 1; i++)
	{
		voxelIndx.emplace_back(i);
	}
	return voxelIndx;

}

std::vector<SurfaceGroup*> CJGeoCreator::getXYFaces(TopoDS_Shape shape) {
	std::vector<SurfaceGroup*> surfaceGroupList;
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
		
		// ignore if the z component of normal is 0 
		TopoDS_Face face = TopoDS::Face(expl.Current());

		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
		double area = gprops.Mass();

		if (area < 0.01) { continue; }

		Handle(Geom_Surface) Surface = BRep_Tool::Surface(face);

		GeomAdaptor_Surface theGASurface(Surface);
		if (theGASurface.GetType() != GeomAbs_Plane) {
			continue;
		}
		GeomLProp_SLProps SLProps(Surface, 1, 0.1);
		gp_Pnt2d P2d = gp_Pnt2d(0.5,0.5);
		SLProps.SetParameters(P2d.X(), P2d.Y());
		gp_Dir direc = SLProps.Normal();
		double magnitude = direc.X() + direc.Y() + direc.Z();

		if (direc.Z() == 0 ) { continue; }
		if (direc.Z()/magnitude < 0.001 && direc.Z()/magnitude > - 0.001) { continue; }
		surfaceGroupList.emplace_back(new SurfaceGroup(face));
	}

	std::vector<SurfaceGroup*> cleanedSurfaceGroupList;
	for (size_t i = 0; i < surfaceGroupList.size(); i++)
	{
		SurfaceGroup* currentGroup = surfaceGroupList[i];
		TopoDS_Face currentFace = currentGroup->getFace();

		if (!currentGroup->isVisible()) { continue; }

		// ignore lowest if identical projected points
		double height = currentGroup->getAvHeight();
		int vertCount = currentGroup->getVertCount();

		for (size_t j = 0; j < surfaceGroupList.size(); j++)
		{
			if (i == j) { continue; }

			SurfaceGroup* otherGroup = surfaceGroupList[j];
			TopoDS_Face otherFace = otherGroup->getFace();

			if (!otherGroup->isVisible()) { continue; }

			double otherHeight = otherGroup->getAvHeight();
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
				currentGroup->setIsHidden();
				break;
			}
		}

		if (!currentGroup->isVisible()) { continue; }
		
		if (currentGroup->isVisible())
		{ 
			cleanedSurfaceGroupList.emplace_back(currentGroup);
			currentGroup->projectFace();
			currentGroup->populateGrid(0.3); //TODO: use voxel size?
			continue;
		}
	}

	// do raycasting on itself
	std::vector<SurfaceGroup*> filteredSurfaceGroupList;
	for (size_t i = 0; i < cleanedSurfaceGroupList.size(); i++)
	{
		SurfaceGroup* currentGroup = cleanedSurfaceGroupList[i];

		if (currentGroup->testIsVisable(cleanedSurfaceGroupList))
		{
			filteredSurfaceGroupList.emplace_back(currentGroup);
		}
	}
	return filteredSurfaceGroupList;
}

CJT::GeoObject* CJGeoCreator::makeLoD00(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	gp_Pnt lll = cluster->getLllPoint();
	gp_Pnt urr = cluster->getUrrPoint();
	double rotationAngle = cluster->getDirection();

	// LoD 0.0
	gp_Pnt p0 = rotatePointWorld(lll, -rotationAngle);
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -rotationAngle);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Shape floorProjection = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));

	CJT::GeoObject* geoObject = kernel->convertToJSON(floorProjection, "0.0");

	return geoObject;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD02(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	if (!hasTopFaces_ || !hasProjectedFaces_) { initializeBasic(cluster); }

	std::vector<Edge*> edgeList = makeJumbledGround();
	// find outer edge
	std::vector<TopoDS_Edge> outerList = getOuterEdges(edgeList, projectedFaceList_);
	footPrintList_ = outerEdges2Shapes(outerList, kernel);
	hasFootPrint_ = true;

	std::vector< CJT::GeoObject*> geoObjectList;
	for (size_t i = 0; i < footPrintList_.size(); i++)
	{
		geoObjectList.emplace_back(kernel->convertToJSON(footPrintList_[i], "0.2"));
	}
	return geoObjectList;
}

CJT::GeoObject* CJGeoCreator::makeLoD10(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	gp_Pnt lll = cluster->getLllPoint();
	gp_Pnt urr = cluster->getUrrPoint();
	double rotationAngle = cluster->getDirection();

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid bbox;
	brepBuilder.MakeSolid(bbox);

	gp_Pnt p0 = rotatePointWorld(lll, -rotationAngle);
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -rotationAngle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -rotationAngle);

	gp_Pnt p4(rotatePointWorld(urr, -rotationAngle));
	gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -rotationAngle);
	gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -rotationAngle);
	gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -rotationAngle);

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
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));
	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }

	brepSewer.Perform();
	brepBuilder.Add(bbox, brepSewer.SewedShape());

	CJT::GeoObject* geoObject = kernel->convertToJSON(bbox, "1.0");

	return geoObject;
}

std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD12(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	if (!hasTopFaces_ || !hasProjectedFaces_) { initializeBasic(cluster); }

	std::vector< CJT::GeoObject*> geoObjectList;
	if (!hasFootPrint_)
	{
		makeLoD02(cluster, cjCollection, kernel, unitScale);
	}
	if (footPrintList_.size() == 0)
	{
		return geoObjectList;
	}

	double height = cluster->getUrrPoint().Z();
	
	for (size_t i = 0; i < footPrintList_.size(); i++)
	{
		TopoDS_Face currentFootprint = footPrintList_[i];

		BRepPrimAPI_MakePrism sweeper(currentFootprint, gp_Vec(0, 0, height), Standard_True);
		sweeper.Build();
		TopoDS_Shape extrudedShape = sweeper.Shape();

		CJT::GeoObject* geoObject = kernel->convertToJSON(extrudedShape, "1.2");
		geoObjectList.emplace_back(geoObject);
	}
	return geoObjectList;
}

std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD22(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale) 
{
	std::vector< CJT::GeoObject*> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(cluster);
	}
	if (footPrintList_.size() == 0)
	{
		return geoObjectList;
	}
	
	// make plane to cut of lower parts
	BOPAlgo_Splitter aSplitter;
	TopTools_ListOfShape aLSObjects;
	TopTools_ListOfShape aLSTools;

	gp_Pnt p0(-1000, -1000, 0);
	gp_Pnt p1(-1000, 1000, 0);
	gp_Pnt p2(1000, 1000, 0);
	gp_Pnt p3(1000, -1000, 0);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	aLSTools.Append(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3)));
	aSplitter.SetTools(aLSTools);
	aSplitter.SetRunParallel(Standard_True);
	aSplitter.SetNonDestructive(Standard_True);

	TopTools_ListOfShape aLSFuseObjects;
	TopExp_Explorer expl;
	for (size_t i = 0; i < topFaceList_.size(); i++)
	{
		SurfaceGroup* currentRoof = topFaceList_[i];

		if (currentRoof->getURRPoint().Z() == 0) //TODO: make smarter
		{
			continue;
		}

		BRepPrimAPI_MakePrism sweeper(currentRoof->getFace(), gp_Vec(0, 0, - currentRoof->getURRPoint().Z()), Standard_True);
		sweeper.Build();
		TopoDS_Shape extrudedShape = sweeper.Shape();
		aLSObjects.Clear();
		aLSObjects.Append(extrudedShape);
		aSplitter.SetArguments(aLSObjects);
		aSplitter.Perform();

		const TopoDS_Shape& aResult = aSplitter.Shape();

		// find top solid after the split
		TopExp_Explorer expl2;
		for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) {
			bool above = false;
			for (expl2.Init(TopoDS::Solid(expl.Current()), TopAbs_VERTEX); expl2.More(); expl2.Next()) {
				TopoDS_Vertex vertex = TopoDS::Vertex(expl2.Current());
				if (BRep_Tool::Pnt(vertex).Z() > 0)
				{
					above = true;
					break;
				}
				if (BRep_Tool::Pnt(vertex).Z() < 0)
				{
					break;
				}
			}

			if (above)
			{
				aLSFuseObjects.Append(TopoDS::Solid(expl.Current()));
			}

		}
	}

	BOPAlgo_Builder aBuilder;
	aBuilder.SetArguments(aLSFuseObjects);
	aBuilder.SetRunParallel(Standard_True);
	aBuilder.Perform();

	std::vector<TopoDS_Face> ObjectFaceList;
	for (expl.Init(aBuilder.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
		ObjectFaceList.emplace_back(TopoDS::Face(expl.Current()));
	}


	std::vector<TopoDS_Face> cleanedFaceList;
	// check if surfaces overlap
	for (size_t i = 0; i < ObjectFaceList.size(); i++)
	{
		bool dub = false;
		for (size_t j = 0; j < ObjectFaceList.size(); j++)
		{

			if (j == i)
			{
				continue;
			}
			if (ObjectFaceList[i].IsEqual(ObjectFaceList[j]) ||
				ObjectFaceList[i].IsEqual(ObjectFaceList[j].Reversed()))
			{
				dub = true;
				break;
			}
		}

		if (!dub)
		{
			cleanedFaceList.emplace_back(ObjectFaceList[i]);
		}
	}

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidShape;
	brepBuilder.MakeSolid(solidShape);

	// check if edge has overlap
	for (size_t i = 0; i < cleanedFaceList.size(); i++)
	{
		bool valid = false;
		for (size_t j = 0; j < cleanedFaceList.size(); j++)
		{
			if (i == j)
			{
				continue;
			}
			double distance = BRepExtrema_DistShapeShape(cleanedFaceList[i], cleanedFaceList[j]).Value();
			if (distance < 0.001)
			{
				valid = true;
			}
		}
		if (valid)
		{
			brepSewer.Add(cleanedFaceList[i]);
		}
	}

	brepSewer.Perform();
	brepBuilder.Add(solidShape, brepSewer.SewedShape());

	CJT::GeoObject* geoObject = kernel->convertToJSON(solidShape, "2.2");
	geoObjectList.emplace_back(geoObject);

	return geoObjectList;

}

CJT::GeoObject* CJGeoCreator::makeLoD32(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	// asign rooms
	int roomnum = 0;
	int temps = 0;

	TopoDS_Shape outSideShape;

	std::vector<int> totalRoom;
	std::cout << "[INFO] Exterior space growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting())
		{
			totalRoom = growExterior(i, roomnum, cluster);
			break;
		}
	}
	std::cout << std::endl;
	std::cout << std::endl;

	if (totalRoom.size() == 0)
	{
		std::cout << "Unable to find exterior space" << std::endl;
		return nullptr;
	}

	std::cout << "[INFO] Exterior space succesfully grown" << std::endl << std::endl;

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid roughRoomShape;
	brepBuilder.MakeSolid(roughRoomShape);

	gp_Pnt inRoomPoint(99999, 99999, 99999);
	bool hasInsidePoint = false;

	std::vector<int> productLookupValues;

	// create bbox around rough room shape
	gp_Pnt lll(9999, 9999, 9999);
	gp_Pnt urr(-9999, -9999, -9999);

	gp_Pnt qlll(9999, 9999, 9999);
	gp_Pnt qurr(-9999, -9999, -9999);

	for (size_t j = 1; j < totalRoom.size(); j++)
	{
		voxel* currentBoxel = VoxelLookup_[totalRoom[j]];
		// get unique product lookup values
		std::vector<int> internalProducts = currentBoxel->getInternalProductList();
		for (size_t k = 0; k < internalProducts.size(); k++) { productLookupValues.emplace_back(internalProducts[k]); }

		// create bbox
		std::vector<gp_Pnt> cornerPoints = currentBoxel->getCornerPoints(planeRotation_);
		std::vector<gp_Pnt> cornerPointsRel = currentBoxel->getCornerPoints(0);
		for (size_t k = 0; k < cornerPoints.size(); k++)
		{
			auto currentCorner = rotatePointWorld(cornerPoints[k], planeRotation_);
			if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
			if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
			if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
			if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
			if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
			if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }

			auto currentCornerRel = cornerPointsRel[k];
			if (qurr.X() < currentCornerRel.X()) { qurr.SetX(currentCornerRel.X()); }
			if (qurr.Y() < currentCornerRel.Y()) { qurr.SetY(currentCornerRel.Y()); }
			if (qurr.Z() < currentCornerRel.Z()) { qurr.SetZ(currentCornerRel.Z()); }
			if (qlll.X() > currentCornerRel.X()) { qlll.SetX(currentCornerRel.X()); }
			if (qlll.Y() > currentCornerRel.Y()) { qlll.SetY(currentCornerRel.Y()); }
			if (qlll.Z() > currentCornerRel.Z()) { qlll.SetZ(currentCornerRel.Z()); }
		}

		// search for inside point
		if (!currentBoxel->getIsIntersecting() && !hasInsidePoint)
		{
			inRoomPoint = rotatePointWorld(cornerPoints[5], planeRotation_);
			inRoomPoint.SetZ(inRoomPoint.Z() - currentBoxel->getZ() / 2);
			hasInsidePoint = true;
		}
	}

	gp_Pnt p0(rotatePointWorld(lll, -planeRotation_));
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -planeRotation_);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -planeRotation_);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -planeRotation_);

	gp_Pnt p4(rotatePointWorld(urr, -planeRotation_));
	gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -planeRotation_);
	gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -planeRotation_);
	gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -planeRotation_);

	gp_Pnt pC = rotatePointWorld(gp_Pnt(lll.X() + (urr.X() - lll.X()) / 2, lll.Y() + (urr.Y() - lll.Y()) / 2, lll.Z() + (urr.Z() - lll.Z()) / 2), -planeRotation_);
	gp_Pnt pCR = gp_Pnt(qlll.X() + (qurr.X() - qlll.X()) / 2, qlll.Y() + (qurr.Y() - qlll.Y()) / 2, qlll.Z() + (qurr.Z() - qlll.Z()) / 2);

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
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));

	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }

	brepSewer.Perform();
	brepBuilder.Add(roughRoomShape, brepSewer.SewedShape());

	gp_Trsf scaler;
	scaler.SetScale(pC, 1.3);

	// finalize rough room shape
	TopoDS_Shape sizedRoomShape = BRepBuilderAPI_Transform(roughRoomShape, scaler).ModifiedShape(roughRoomShape);
	p0.Transform(scaler);
	p4.Transform(scaler);

	// finalize qbox shape
	scaler.SetScale(pCR, 1.1);
	qlll.Transform(scaler);
	qurr.Transform(scaler);
	boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(qlll), Point3DOTB(qurr));

	// intersect roomshape with objects 
	BOPAlgo_Splitter aSplitter;
	TopTools_ListOfShape aLSObjects;
	aLSObjects.Append(sizedRoomShape);
	TopTools_ListOfShape aLSTools;

	// make unique productLookupValues
	std::set<int> productLookupValuesSet;
	for (unsigned i = 0; i < productLookupValues.size(); ++i) productLookupValuesSet.insert(productLookupValues[i]);
	productLookupValues.assign(productLookupValuesSet.begin(), productLookupValuesSet.end());

	TopExp_Explorer expl;
	std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList;

	for (int j = 0; j < cluster->getSize(); j++)
	{
		for (size_t k = 0; k < productLookupValues.size(); k++)
		{
			LookupValue lookup = cluster->getHelper(j)->getLookup(productLookupValues[k]);
			IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);
			TopoDS_Shape shape;

			if (std::get<3>(lookup))
			{
				shape = std::get<4>(lookup);
			}
			else {
				shape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup), true);
			}

			int sCount = 0;

			for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
				aLSTools.Append(TopoDS::Solid(expl.Current()));
				sCount++;
			}

			if (sCount == 0)
			{
				if (qProduct->data().type()->name() == "IfcSlab") // TODO: replace this statement
				{
					for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
						aLSTools.Append(TopoDS::Face(expl.Current()));
					}
				}
			}
		}
	}

	aLSTools.Reverse();
	aSplitter.SetArguments(aLSObjects);
	aSplitter.SetTools(aLSTools);
	aSplitter.SetRunParallel(Standard_True);
	//aSplitter.SetFuzzyValue(0.0001);
	aSplitter.SetNonDestructive(Standard_True);

	aSplitter.Perform();

	const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

	STEPControl_Writer writer;
	writer.Transfer(aResult, STEPControl_AsIs);
	//writer.Write("C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/test.STEP");

	// get outside shape
	std::vector<TopoDS_Solid> solids;
	for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) {
		solids.emplace_back(TopoDS::Solid(expl.Current()));
	}

	std::vector<gp_Pnt> bboxPoints;
	for (expl.Init(sizedRoomShape, TopAbs_VERTEX); expl.More(); expl.Next()) {
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		bboxPoints.emplace_back(BRep_Tool::Pnt(vertex));
	}

	bool found = false;
	for (size_t j = 0; j < solids.size(); j++) // TODO: make function
	{
		for (expl.Init(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			for (size_t k = 0; k < bboxPoints.size(); k++)
			{
				if (p.IsEqual(bboxPoints[k], 0.01))
				{
					outSideShape = solids[j];
					found = true;
					break;
				}
			}
			if (found)
			{
				break;
			}
		}
		if (found)
		{
			break;
		}
	}

	std::vector<TopoDS_Shell> shellList;
	for (expl.Init(outSideShape, TopAbs_SHELL); expl.More(); expl.Next()) {
		shellList.emplace_back(TopoDS::Shell(expl.Current()));
	}

	// extract the inner shell of the shape
	double score = 0;
	for (size_t j = 0; j < shellList.size(); j++) // TODO: make function
	{
		found = false;
		for (expl.Init(shellList[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			for (size_t k = 0; k < bboxPoints.size(); k++)
			{
				if (p.IsEqual(bboxPoints[k], 0.01)) {
					found = true;
					break;
				}
			}
			if (found)
			{
				break;
			}
		}
		if (!found)
		{
			GProp_GProps gprop;
			BRepGProp::VolumeProperties(shellList[j], gprop);
			double mass = abs(gprop.Mass());
			if (score < mass)
			{
				outSideShape = shellList[j];
				score = mass;
			}
		}
	}

	CJT::GeoObject* geoObject = kernel->convertToJSON(outSideShape, "3.0");
	return geoObject;
}

CJGeoCreator::CJGeoCreator(helperCluster* cluster, bool isFlat)
{
	// ask user for desired voxel dimensions

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
			voxelSize_ = val;
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
			voxelSizeZ_ = val;
			break;
		}
	}

	std::cout << std::endl;

	double xySize = std::stod(stringXYSize);
	double zSize = std::stod(stringZSize);

	// compute generic voxelfield data
	anchor_ = cluster->getLllPoint();
	gp_Pnt urrPoints = cluster->getUrrPoint();

	// resize to allow full voxel encapsulation
	anchor_.SetX(anchor_.X() - (xySize * 2));
	anchor_.SetY(anchor_.Y() - (xySize * 2));
	anchor_.SetZ(anchor_.Z() - (zSize * 2));

	urrPoints.SetX(urrPoints.X() + (xySize * 2));
	urrPoints.SetY(urrPoints.Y() + (xySize * 2));
	urrPoints.SetZ(urrPoints.Z() + (zSize * 2));

	// set range
	double xRange = urrPoints.X() - anchor_.X();
	double yRange = urrPoints.Y() - anchor_.Y();
	double zRange = urrPoints.Z() - anchor_.Z();

	xRelRange_ = (int) ceil(xRange / voxelSize_) + 1;
	yRelRange_ = (int) ceil(yRange / voxelSize_) + 1;
	zRelRange_ = (int) ceil(zRange / voxelSizeZ_) + 1;

	totalVoxels_ = xRelRange_ * yRelRange_ * zRelRange_;
	Assignment_ = std::vector<int>(totalVoxels_, 0);

	planeRotation_ = cluster->getDirection();

	if (false)
	{
		std::cout << "cluster debug:" << std::endl;

		std::cout << anchor_.X() << std::endl;
		std::cout << anchor_.Y() << std::endl;
		std::cout << anchor_.Z() << std::endl;


		std::cout << xRange << std::endl;
		std::cout << yRange << std::endl;
		std::cout << zRange << std::endl;

		std::cout << xRelRange_ << std::endl;
		std::cout << yRelRange_ << std::endl;
		std::cout << zRelRange_ << std::endl;

		std::cout << totalVoxels_ << std::endl;
	}

	hallwayNum_ = cluster->getHallwayNum();
	minRoom_ = cluster->getMinRoomNum();
	minArea_ = cluster->getMinArea();

	std::cout << "[INFO] Populate Grid" << std::endl;
	for (int i = 0; i < totalVoxels_; i++) {

		if (i % 1000 == 0)
		{
			std::cout.flush();
			std::cout << i << " of " << totalVoxels_ << "\r";
		}

		addVoxel(i, cluster);
	}
	std::cout << totalVoxels_ << " of " << totalVoxels_ << std::endl;
	std::cout << std::endl;

}
	
std::vector<int> CJGeoCreator::growExterior(int startIndx, int roomnum, helperCluster* cluster)
{
	std::vector<int> buffer = { startIndx };
	std::vector<int> totalRoom = { startIndx };
	Assignment_[startIndx] = 1;

	bool isOutSide = false;
	int cSize = cluster->getSize();

	while (buffer.size() > 0)
	{
		std::vector<int> tempBuffer;
		for (size_t j = 0; j < buffer.size(); j++)
		{
			if (j % 1000 == 0)
			{
				std::cout.flush();
				std::cout << "Size: " << totalRoom.size() << "\r";
			}

			int currentIdx = buffer[j];
			voxel* currentBoxel = VoxelLookup_[currentIdx];

			// find potential intersecting objects


			if (currentBoxel->getIsIntersecting())
			{
				continue;
			}

			if (!currentBoxel->getHasEvalIntt())
			{
				currentBoxel->getCenterPoint();
				// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
				auto boxelGeo = currentBoxel->getVoxelGeo();
				std::vector<gp_Pnt> pointList = currentBoxel->getCornerPoints(planeRotation_);

				bool intt = false;
				std::vector<Value> qResult;
				for (int j = 0; j < cSize; j++) // TODO: make this a function
				{
					qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
					cluster->getHelper(j)->getIndexPointer()->query(bgi::intersects(boxelGeo), std::back_inserter(qResult));

					if (qResult.size() == 0) { continue; }

					for (size_t k = 0; k < qResult.size(); k++)
					{
						LookupValue lookup = cluster->getHelper(j)->getLookup(qResult[k].second);
						IfcSchema::IfcProduct* product = std::get<0>(lookup);

						if (!product->hasRepresentation()) { continue; }

						if (currentBoxel->checkIntersecting(lookup, pointList, cluster->getHelper(j)))
						{
							currentBoxel->addInternalProduct(qResult[k].second);
							intt = true;
						}
					}
				}

				if (intt)
				{
					continue;
				}
			}

			// find neighbours
			std::vector<int> neighbourIndx = getNeighbours(currentIdx);

			if (neighbourIndx.size() < 26) { isOutSide = true; }

			for (size_t k = 0; k < neighbourIndx.size(); k++)
			{
				// exlude if already assigned
				if (Assignment_[neighbourIndx[k]] == 0) {
					bool dupli = false;
					for (size_t l = 0; l < tempBuffer.size(); l++)
					{
						// exlude if already in buffer
						if (neighbourIndx[k] == tempBuffer[l])
						{
							dupli = true;
							break;
						}
					}
					if (!dupli)
					{
						tempBuffer.emplace_back(neighbourIndx[k]);
						totalRoom.emplace_back(neighbourIndx[k]);
						Assignment_[neighbourIndx[k]] = 1;
					}
				}
				else if (Assignment_[neighbourIndx[k]] == -1) {
					bool dupli = false;

					for (size_t l = 0; l < totalRoom.size(); l++)
					{
						if (neighbourIndx[k] == totalRoom[l]) {
							dupli = true;
						}
					}
					if (!dupli)
					{
						totalRoom.emplace_back(neighbourIndx[k]);
						tempBuffer.emplace_back(neighbourIndx[k]);
					}
				}
			}
		}
		buffer.clear();
		buffer = tempBuffer;
	}
	if (!isOutSide)
	{
		for (size_t k = 0; k < totalRoom.size(); k++)
		{
			int currentIdx = totalRoom[k];
			voxel* currentBoxel = VoxelLookup_[currentIdx];
			if (!currentBoxel->getIsIntersecting())
			{
				currentBoxel->setOutside();
			}
		}
		return{};
	}
	return totalRoom;
}

voxel::voxel(BoostPoint3D center, double sizeXY, double sizeZ)
{
	sizeXY_ = sizeXY;
	sizeZ_ = sizeZ;
	center_ = center;

	gp_Pnt minPoint(bg::get<0>(center) - 1 / 2 * sizeXY, bg::get<1>(center) - 1 / 2 * sizeXY, bg::get<2>(center) - 1 / 2 * sizeZ);
	gp_Pnt maxPoint(bg::get<0>(center) + 1 / 2 * sizeXY, bg::get<1>(center) + 1 / 2 * sizeXY, bg::get<2>(center) + 1 / 2 * sizeZ);
}

bg::model::box<BoostPoint3D> voxel::getVoxelGeo()
{
	double offsetXY = sizeXY_ / 2;
	double offsetZ = sizeZ_ / 2;

	BoostPoint3D lll (bg::get<0>(center_) - offsetXY, bg::get<1>(center_) - offsetXY, bg::get<2>(center_) - offsetZ);
	BoostPoint3D urr(bg::get<0>(center_) + offsetXY, bg::get<1>(center_) + offsetXY, bg::get<2>(center_) + offsetZ);
	
	return bg::model::box<BoostPoint3D>(lll, urr);
}

std::vector<gp_Pnt> voxel::getCornerPoints(double angle)
{
	auto boxelGeo = getVoxelGeo();

	auto minPoint = Point3DBTO(boxelGeo.min_corner());
	auto maxPoint = Point3DBTO(boxelGeo.max_corner());

	// make a pointlist 0 - 3 lower ring, 4 - 7 upper ring
	std::vector<gp_Pnt> pointList;
	pointList.emplace_back(minPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), minPoint.Z());
	pointList.emplace_back(maxPoint);
	pointList.emplace_back(maxPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), minPoint.Y(), maxPoint.Z());
	pointList.emplace_back(minPoint.X(), maxPoint.Y(), maxPoint.Z());

	for (size_t i = 0; i < pointList.size(); i++)
	{
		pointList[i] = rotatePointWorld(pointList[i], -angle);
	}


	return pointList;
}

std::vector<std::vector<int>> voxel::getVoxelTriangles()
{
	return {
	{ 0, 1, 5 }, // side	
	{ 0, 5, 6 },
	{ 1, 2, 4 },
	{ 1, 4, 5 },
	{ 2, 3, 7 },
	{ 2, 7, 4 },
	{ 3, 0, 6 },
	{ 3, 6, 7 },
	{ 6, 5, 4 }, // top
	{ 6, 4, 7 },
	{ 0, 3, 2 }, // buttom
	{ 0, 2, 1 }
	};
}

std::vector<std::vector<int>> voxel::getVoxelFaces()
{
	return {
	{ 0, 1, 5, 6 }, // side	
	{ 1, 2, 4, 5 },
	{ 2, 3, 7, 4 },
	{ 3, 0, 6, 7 },
	{ 6, 5, 4, 7 }, // top
	{ 0, 3, 2, 1 }, // buttom
	};
}

std::vector<std::vector<int>> voxel::getVoxelEdges()
{
	return {
		{ 0, 1},
		{ 1, 2},
		{ 2, 3},
		{ 3, 0},
		{ 4, 5},
		{ 5, 6},
		{ 6, 7},
		{ 7, 4},
		{ 1, 5},
		{ 2, 4},
		{ 3, 7},
		{ 0, 6}
	};
}

bool voxel::checkIntersecting(LookupValue lookup, std::vector<gp_Pnt> voxelPoints, helper* h)
{
	hasEvalIntt = true;
	std::vector<std::vector<int>> vets = getVoxelEdges();

	IfcSchema::IfcProduct* product = std::get<0>(lookup);
	std::vector<gp_Pnt> productPoints = h->getObjectPoints(product, false, true);
	
	// check if any cornerpoints fall inside voxel
	if (linearEqIntersection(productPoints, voxelPoints))
	{
		isIntersecting_ = true;
		return true;
	}

	std::vector<std::vector<int>> triangleVoxels = getVoxelTriangles();
	
	for (size_t i = 0; i < triangleVoxels.size(); i++)
	{
		std::vector<gp_Pnt> voxelTriangle = { voxelPoints[triangleVoxels[i][0]], voxelPoints[triangleVoxels[i][1]], voxelPoints[triangleVoxels[i][2]] };

		for (size_t k = 0; k < productPoints.size(); k+=2)
		{
			if (triangleIntersecting({ productPoints[k], productPoints[k + 1] }, voxelTriangle))
			{
				isIntersecting_ = true;
				return true;
			}
		}
	}

	// check with triangulated object
	std::vector<std::vector<gp_Pnt>> triangleMesh = std::get<1>(lookup);

	for (size_t i = 0; i < triangleMesh.size(); i++)
	{
		std::vector<gp_Pnt> triangle = triangleMesh[i];

		for (size_t k = 0; k < vets.size(); k++)
		{
			if (triangleIntersecting({ voxelPoints[vets[k][0]], voxelPoints[vets[k][1]] }, triangle))
			{
				isIntersecting_ = true;
				return true; 
			}
		}
	}
	return false;
}

bool voxel::linearEqIntersection(std::vector<gp_Pnt> productPoints, std::vector<gp_Pnt> voxelPoints)
{
	// check if any cornerpoints fall inside voxel
	gp_Pnt p1 = voxelPoints[0];
	gp_Pnt p2 = voxelPoints[1];
	gp_Pnt p3 = voxelPoints[3];

	if (p2.Y() - p1.Y() == 0)
	{
		for (size_t i = 0; i < productPoints.size(); i++)
		{
			gp_Pnt currentPP = productPoints[i];

			if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) { continue; }
			if (currentPP.X() < p1.X() && currentPP.X() < voxelPoints[4].X() ||
				currentPP.X() > p1.X() && currentPP.X() > voxelPoints[4].X()) {
				continue;
			}
			if (currentPP.Y() < p1.Y() && currentPP.Y() < voxelPoints[4].Y() ||
				currentPP.Y() > p1.Y() && currentPP.Y() > voxelPoints[4].Y()) {
				continue;
			}
			return true;
		}
		return false;
	}

	double a1 = (p2.Y() - p1.Y()) / (p2.X() - p1.X());
	double b11 = p2.Y() - a1 * p2.X();
	double b12 = p3.Y() - a1 * p3.X();

	double a2 = -1 / a1;
	double b21 = p3.Y() - a2 * p3.X();
	double b22 = p2.Y() - a2 * p2.X();

	for (size_t i = 0; i < productPoints.size(); i++)
	{
		gp_Pnt currentPP = productPoints[i];

		// check if point is in z range
		if (currentPP.Z() < p1.Z() || currentPP.Z() > voxelPoints[4].Z()) { continue; }

		// check if point is in voxel
		double x = currentPP.X();

		double y11 = a1 * x + b11;
		double y12 = a1 * x + b12;

		if (currentPP.Y() < y11 && currentPP.Y() < y12 ||
			currentPP.Y() > y11 && currentPP.Y() > y12) {
			continue;
		}

		double y21 = a2 * x + b21;
		double y22 = a2 * x + b22;

		if (currentPP.Y() < y21 && currentPP.Y() > y22 ||
			currentPP.Y() > y21 && currentPP.Y() < y22)
		{
			return true;
		}
	}
	return false;
}


