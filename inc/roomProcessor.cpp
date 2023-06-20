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

double getTopFaceHeight(TopoDS_Face face) {
	double maxHeight = -999999;
	TopExp_Explorer expl;
	for (expl.Init(face, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (maxHeight < p.Z()) { maxHeight = p.Z(); }
	}
	return maxHeight;
}


std::tuple<gp_Pnt, gp_Pnt> getProPointsEdge(TopoDS_Edge edge) {
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

	if (std::get<0>(pp).Distance(std::get<1>(pp)) == 0)  { return gp_Vec(0, 0, 0); }

	gp_Vec vec = gp_Vec(std::get<0>(pp), std::get<1>(pp));
	return vec.Normalized();
}

gp_Pnt* linearLineIntersection(gp_Pnt sP1, gp_Pnt eP1, gp_Pnt sP2, gp_Pnt eP2, double buffer = 0.01) {
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

	if (x1 - buffer < xI && x1 + buffer > xI && y1 - buffer < yI && y1 + buffer > yI ||
		x2 - buffer < xI && x2 + buffer > xI && y2 - buffer < yI && y2 + buffer > yI
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

gp_Pnt* linearLineIntersection(Edge* edge1, Edge* edge2, double buffer = 0.01) {

	gp_Pnt sP1 = edge1->getProjectedStart();
	gp_Pnt eP1 = edge1->getProjectedEnd();
	gp_Pnt sP2 = edge2->getProjectedStart();
	gp_Pnt eP2 = edge2->getProjectedEnd();

	return linearLineIntersection(sP1, eP1, sP2, eP2, buffer);
}


gp_Pnt* linearLineIntersection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, double buffer = 0.01) {

	std::tuple<gp_Pnt, gp_Pnt> points1 = getProPointsEdge(edge1);
	std::tuple<gp_Pnt, gp_Pnt> points2 = getProPointsEdge(edge2);

	return linearLineIntersection(std::get<0>(points1), std::get<1>(points1), std::get<0>(points2), std::get<1>(points2), buffer);
}


/// using the grids to see if surfaces are identical
bool isOverlappingCompletely(SurfaceGroup* evalFace, SurfaceGroup* otherFace) {
	std::vector<EvaluationPoint*> evalGrid =  evalFace->getPointGrid();
	std::vector<EvaluationPoint*> otherGrid = otherFace->getPointGrid();

	if (evalGrid.size() != otherGrid.size()) { return false; }

	for (size_t i = 0; i < evalGrid.size(); i++)
	{
		if (!evalGrid[i]->getPoint().IsEqual(otherGrid[i]->getPoint(), 1e-6)) { return false; }
	}
	return true;
}


bool isOverlappingCompletely(SurfaceGroup* evalFace, std::vector<SurfaceGroup*>  facePool) {
	for (size_t i = 0; i < facePool.size(); i++)
	{
		if (isOverlappingCompletely(evalFace, facePool[i])) { return true; }
	}
	return false;
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
	topHeight_ = getTopFaceHeight(aFace);
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

	bool isFlat = true;
	double z = 0;
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

			if (surfSize == 0)
			{
				z = point.Z();
			}
			else if (z != point.Z())
			{
				isFlat = false;
			}

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
		theFlatFace_ = tempFace;
		return;
	}

	TopoDS_Face projectedFace = faceBuilder.Face();
	theProjectedFace_ = projectedFace;

	if (isFlat)
	{
		theFlatFace_ = theFace_;
		return;
	}

	theFlatFace_ = projectedFace;

	gp_Vec v(0, 0, getTopHeight());
	gp_Trsf t;
	t.SetTranslation(v);
	theFlatFace_.Move(t);
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

	
	//std::cout << "face" << std::endl;
	TopExp_Explorer expl1;
	for (expl1.Init(currentFace, TopAbs_VERTEX); expl1.More(); expl1.Next()) {
		TopoDS_Vertex vertex = TopoDS::Vertex(expl1.Current());
		gp_Pnt point = BRep_Tool::Pnt(vertex);
		//printPoint(point);
	}
	//std::cout << "click" << std::endl;

	for (size_t i = 0; i < otherSurfaces.size(); i++)
	{

		SurfaceGroup* otherGroup = otherSurfaces[i];

		if (preFilter)
		{
			if (!overlap(otherGroup)) { continue; }
		}

		TopoDS_Face otherFace = otherGroup->getFace();
		gp_Pnt otherLLLPoint = otherGroup->getLLLPoint();
		gp_Pnt otherURRPoint = otherGroup->getURRPoint();

		if (currentFace.IsEqual(otherFace)) { continue; }

		IntCurvesFace_Intersector intersector(otherFace, 0.0001);

		for (size_t k = 0; k < currentGrid.size(); k++)
		{
			EvaluationPoint* currentEvalPoint = currentGrid[k];

			if (!currentEvalPoint->isVisible()) { continue; }

			// check if the projection line falls withing the bbox of the surface
			gp_Pnt evalPoint = currentEvalPoint->getPoint();
			if (evalPoint.X() - otherLLLPoint.X() < -0.1 || evalPoint.X() - otherURRPoint.X() > 0.1) { continue; }
			if (evalPoint.Y() - otherLLLPoint.Y() < -0.1 || evalPoint.Y() - otherURRPoint.Y() > 0.1) { continue; }
			if (evalPoint.Z() - urrPoint_.Z()  > 0.1)  { continue; }

			intersector.Perform( //TODO: find a smarter way to proccess
				currentEvalPoint->getEvalLine(),
				-0,
				+INFINITE);

			if (intersector.NbPnt() > 0)
			{

				//printPoint(currentEvalPoint->getPoint());
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
		std::tuple<gp_Pnt, gp_Pnt> points = getProPointsEdge(currentEdge);
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


std::vector<Edge*> CJGeoCreator::makeJumbledGround() { //TODO: make smarter
	
	gp_Ax2 axis(
		gp_Pnt(0, 0, 0), 
		gp_Dir(0, 0, 100)
	);
	HLRBRep_Algo* Edgeprojector = new HLRBRep_Algo();
	Edgeprojector->Projector(HLRAlgo_Projector(axis));
	
	BOPAlgo_Builder aBuilder;
	TopTools_ListOfShape aLSObjects;

	for (size_t i = 0; i < faceList_.size(); i++) 
	{
		for (size_t j = 0; j < faceList_[i].size(); j++)
		{
			Edgeprojector->Add(faceList_[i][j]->getProjectedFace());
		}
	}

	Edgeprojector->Update();
	HLRBRep_HLRToShape projectToShape(Edgeprojector);

	TopoDS_Shape flattenedEdges = projectToShape.CompoundOfEdges(HLRBRep_Undefined, true, false);

	std::vector<Edge*> uniqueEdges = getUniqueEdges(flattenedEdges);
	std::vector<Edge*> cleanedEdges = mergeOverlappingEdges(uniqueEdges);
	return splitIntersectingEdges(cleanedEdges);
}

bool CJGeoCreator::isOuterEdge(Edge* currentEdge, std::vector<TopoDS_Face*> flatFaceList, bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex)
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

	TopoDS_Edge evalEdge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(evalPoint1.X(), evalPoint1.Y(), 0), gp_Pnt(evalPoint1.X(), evalPoint1.Y() + 10000, 0));
	TopoDS_Edge evalEdge2 = BRepBuilderAPI_MakeEdge(gp_Pnt(evalPoint2.X(), evalPoint2.Y(), 0), gp_Pnt(evalPoint2.X(), evalPoint2.Y() + 10000, 0));

	bool hasEval1 = false;
	bool hasEval2 = false;

	// querry spatialIndex
	gp_Pnt lll = startPoint;
	gp_Pnt urr = endPoint;

	 if (startPoint.X() < endPoint.X())
	{
		lll.SetX(startPoint.X());
		urr.SetX(endPoint.X());
	}
	else if (startPoint.X() > endPoint.X())
	{
		lll.SetX(endPoint.X());
		urr.SetX(startPoint.X());
	}

	if (startPoint.Y() < endPoint.Y())
	{
		lll.SetY(startPoint.Y());
		urr.SetY(endPoint.Y());
	}
	else if (startPoint.Y() > endPoint.Y())
	{
		lll.SetY(endPoint.Y());
		urr.SetY(startPoint.Y());
	}
	else
	{
		lll.SetY(startPoint.Y() - 0.5);
		urr.SetY(startPoint.Y() + 0.5);
	}

	std::vector<Value> qResult;
	bg::model::box<BoostPoint3D> boostbbox({ lll.X() - 0.5, lll.Y() -0.5, -1 }, { urr.X() + 0.5, urr.Y() + 0.5, 1 });
	spatialIndex.query(bgi::intersects(boostbbox), std::back_inserter(qResult));

	for (size_t i = 0; i < qResult.size(); i++)
	{
		TopoDS_Face* evalFace = flatFaceList[qResult[i].second];
		int intersectionCount1 = 0;
		int intersectionCount2 = 0;

		for (TopExp_Explorer explorer(*evalFace, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			
			if (linearLineIntersection(evalEdge1, edge, 1e-6) != nullptr) { intersectionCount1++; }
			if (linearLineIntersection(evalEdge2, edge, 1e-6) != nullptr) { intersectionCount2++; }
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

std::vector<TopoDS_Edge> CJGeoCreator::getOuterEdges(std::vector<Edge*> edgeList, std::vector<SurfaceGroup*> faceList) {
	// project and make indexing
	std::vector<TopoDS_Face*> evalSurfList;
	bgi::rtree<Value, bgi::rstar<treeDepth_>> spatialIndex;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		TopoDS_Face* currentFace = faceList[i]->getProjectedFacePtr();

		gp_Pnt lll(999999, 999999, 999999);
		gp_Pnt urr(-999999, -999999, -999999);
		for (TopExp_Explorer explorer(*currentFace, TopAbs_VERTEX); explorer.More(); explorer.Next())
		{
			const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			if (p.X() < lll.X()) { lll.SetX(p.X()); }
			if (p.Y() < lll.Y()) { lll.SetY(p.Y()); }
			if (p.Z() < lll.Z()) { lll.SetZ(p.Z()); }
			if (p.X() > urr.X()) { urr.SetX(p.X()); }
			if (p.Y() > urr.Y()) { urr.SetY(p.Y()); }
			if (p.Z() > urr.Z()) { urr.SetZ(p.Z()); }
		}
		bg::model::box <BoostPoint3D> boostBBox(Point3DOTB(lll), Point3DOTB(urr));

		spatialIndex.insert(std::make_pair(boostBBox, (int) i));
		evalSurfList.emplace_back(currentFace);
	}

	std::vector<TopoDS_Edge> outerEdgeList;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		TopoDS_Edge* currentEdge = edgeList[i]->getEdge();
		if (isOuterEdge(edgeList[i], evalSurfList, spatialIndex)) { outerEdgeList.emplace_back(*currentEdge); }
	}

	return outerEdgeList;
}


std::vector<TopoDS_Face> CJGeoCreator::outerEdges2Shapes(std::vector<TopoDS_Edge> edgeList)
{
	std::vector<TopoDS_Wire> wireList = growWires(edgeList);
	std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);	
	std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);
	return cleanedFaceList;
}

std::vector<TopoDS_Wire> CJGeoCreator::growWires(std::vector<TopoDS_Edge> edgeList){
	std::vector<TopoDS_Wire> wireCollection;

	BRepBuilderAPI_MakeWire wireMaker;
	bool loopFound = false;

	TopoDS_Edge currentEdge = edgeList[0];
	std::vector<int> evaluated(edgeList.size());
	evaluated[0] = 1;
	gp_Pnt connectionPoint = std::get<1>(getPointsEdge(edgeList[0]));
	gp_Pnt originPoint = std::get<0>(getPointsEdge(edgeList[0]));
	wireMaker.Add(BRepBuilderAPI_MakeEdge(originPoint, connectionPoint));
	while (true)
	{
		bool stepped = false;

		double distance = 999999999;
		gp_Pnt backupPoint;

		for (size_t i = 0; i < edgeList.size(); i++) // select and order loop's edges
		{
			if (evaluated[i] == 1) { continue; }

			std::tuple<gp_Pnt, gp_Pnt> pp = getPointsEdge(edgeList[i]);
			gp_Pnt otherStart = std::get<0>(pp);
			gp_Pnt otherEnd = std::get<1>(pp);

			// aquire data for a backup point
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

			if (connectionPoint.IsEqual(otherStart, 1e-6))
			{
				wireMaker.Add(edgeList[i]);
				connectionPoint = otherEnd;
				evaluated[i] = 1;
				stepped = true;
				break;
			}
			else if (connectionPoint.IsEqual(otherEnd, 1e-6))
			{
				wireMaker.Add(BRepBuilderAPI_MakeEdge(otherEnd, otherStart));
				connectionPoint = otherStart;
				evaluated[i] = 1;
				stepped = true;
				break;
			}
		}

		if (stepped) // check if step is taken
		{
			continue;
		}

		if (originPoint.IsEqual(connectionPoint, 1e-6)) // if no step is taken, check if looped
		{
			loopFound = true;
		}
		else if (distance < 0.1) // if no loop check for a backup point
		{
			wireMaker.Add(BRepBuilderAPI_MakeEdge(connectionPoint, backupPoint));
			connectionPoint = backupPoint;
			continue;
		}

		if (loopFound)
		{
			wireMaker.Build();
			wireCollection.emplace_back(wireMaker.Wire());
			wireMaker = BRepBuilderAPI_MakeWire();
			loopFound = false;
		}
		if (!loopFound) // Find new startpoint if no loop is found
		{
			bool con = false;
			for (size_t j = 0; j < edgeList.size(); j++)
			{
				if (evaluated[j] == 0)
				{
					wireMaker = BRepBuilderAPI_MakeWire();
					connectionPoint = std::get<1>(getPointsEdge(edgeList[j]));
					originPoint = std::get<0>(getPointsEdge(edgeList[j]));
					wireMaker.Add(BRepBuilderAPI_MakeEdge(originPoint, connectionPoint));
					evaluated[j] = 1;
					con = true;
					break;
				}
			}
			if (!con)
			{
				break;
			}
		}
	}
	return wireCollection;
}

std::vector<TopoDS_Wire> CJGeoCreator::cleanWires(std::vector<TopoDS_Wire> wireList) {

	std::vector<TopoDS_Wire> cleanedWires;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		cleanedWires.emplace_back(cleanWire(wireList[i]));
	}

	return cleanedWires;
}


TopoDS_Wire CJGeoCreator::cleanWire(TopoDS_Wire wire) {
	
	BRepBuilderAPI_MakeWire wireMaker;
	std::vector<TopoDS_Edge> orderedEdgeList;
	for (TopExp_Explorer edgeExp(wire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
		orderedEdgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
	}

	for (size_t i = 0; i < orderedEdgeList.size(); i++)
	{
		gp_Pnt startPoint = std::get<0>(getPointsEdge(orderedEdgeList[i]));
		gp_Pnt endPoint = std::get<1>(getPointsEdge(orderedEdgeList[i]));

		//printPoint(startPoint);
		//printPoint(endPoint);
	}

	std::vector<int> merged(orderedEdgeList.size());
	//gp_Vec evalVec = getDirEdge(orderedEdgeList[0]);
	//for (size_t i = 1; i < orderedEdgeList.size(); i++) // pick corner point as start point
	//{
	//	TopoDS_Edge currentEdge = orderedEdgeList[i];
	//	gp_Vec currentVec = getDirEdge(currentEdge);

	//	gp_Pnt startPoint = std::get<0>(getPointsEdge(currentEdge));
	//	gp_Pnt endPoint = std::get<1>(getPointsEdge(currentEdge));

	//	if (!currentVec.IsParallel(evalVec, 0.01)) {
	//		std::rotate(orderedEdgeList.begin(), orderedEdgeList.begin() + i, orderedEdgeList.end());
	//		break;
	//	}
	//}
	gp_Pnt connection = std::get<0>(getPointsEdge(orderedEdgeList[0]));
	for (size_t i = 0; i < orderedEdgeList.size(); i++) // merge parralel 
	{
		if (merged[i] == 1) { continue; }
		merged[i] = 1;

		TopoDS_Edge currentEdge = orderedEdgeList[i];
		gp_Vec currentVec = getDirEdge(currentEdge);

		if (currentVec.Magnitude() == 0)
		{
			merged[i] = 1;
			continue;
		}


		gp_Pnt endPoint = std::get<1>(getPointsEdge(currentEdge));

		for (size_t j = i + 1; j < orderedEdgeList.size(); j++)
		{
			if (merged[j] == 1) { continue; }

			TopoDS_Edge otherEdge = orderedEdgeList[j];
			gp_Vec otherVec = getDirEdge(otherEdge);

			if (otherVec.Magnitude() == 0)
			{
				merged[j] = 1;
				continue; 
			}

			if (currentVec.IsParallel(otherVec, 0.01)) { 
				merged[j] = 1;
				continue; 
			}

			endPoint = std::get<0>(getPointsEdge(otherEdge));
			break;
		}

		if (connection.IsEqual(endPoint, 1e-6)) { continue; }

		//printPoint(connection);
		//printPoint(endPoint);
		wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));
		connection = endPoint;
	}

	if (connection.IsEqual(std::get<0>(getPointsEdge(orderedEdgeList[0])), 1e-6))
	{
		return wireMaker.Wire();
	}

	//printPoint(connection);
	//printPoint(std::get<0>(getPointsEdge(orderedEdgeList[0])));
	wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, std::get<0>(getPointsEdge(orderedEdgeList[0]))));
	return wireMaker.Wire();
}

std::vector<TopoDS_Face> CJGeoCreator::wireCluster2Faces(std::vector<TopoDS_Wire> wireList) {
	BRepBuilderAPI_MakeFace faceBuilder;
	std::vector<TopoDS_Face> faceList;
	gp_Vec normal;
	gp_Pnt originPoint;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		if (i == 0)
		{
			gp_Vec vec1;
			gp_Vec vec2;
			for (TopExp_Explorer vertexExp(wireList[i], TopAbs_EDGE); vertexExp.More(); vertexExp.Next()) {
				TopoDS_Edge edge = TopoDS::Edge(vertexExp.Current());

				std::tuple<gp_Pnt, gp_Pnt> sE = getPointsEdge(edge);
				originPoint = std::get<0>(sE);
				vec1 = gp_Vec(std::get<0>(sE), std::get<1>(sE));
				vec2 = gp_Vec(std::get<0>(sE), std::get<1>(sE));
				while (vec1.IsParallel(vec2, 0.0001))
				{
					vertexExp.Next();
					edge = TopoDS::Edge(vertexExp.Current());
					std::tuple<gp_Pnt, gp_Pnt> sE = getPointsEdge(edge);
					vec2 = gp_Vec(std::get<0>(sE), std::get<1>(sE));
				}
				break;
			}
			normal = vec1.Crossed(vec2);
			normal.Normalize();
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

				TopoDS_Edge evalEdge = BRepBuilderAPI_MakeEdge(p, gp_Pnt(p.X() + 1000, p.Y(), p.Z()));

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

void CJGeoCreator::initializeBasic(helperCluster* cluster) {
	std::cout << "- Pre proccessing" << std::endl;
	// generate data required for most exports
	std::vector<SurfaceGroup*> shapeList;
	std::vector<TopoDS_Shape> filteredFaces = getTopObjects(cluster);

	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Reduce surfaces" << std::endl;

	for (size_t i = 0; i < filteredFaces.size(); i++)
	{
		std::vector<SurfaceGroup*>  objectFaces = getXYFaces(filteredFaces[i]);
		for (size_t j = 0; j < objectFaces.size(); j++)
		{
			bool isDub = false;


			if (shapeList.size() <= 1)
			{
				shapeList.emplace_back(objectFaces[j]);
				hasTopFaces_ = true;
				continue;
			}

			if (!isOverlappingCompletely(objectFaces[j], shapeList))
			{
				shapeList.emplace_back(objectFaces[j]);
			}
		}
	}

	printTime(startTime, std::chrono::high_resolution_clock::now());

	startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Fine filtering of roofing structures" << std::endl;
	// get the faces visible from the top 
	faceList_.emplace_back(std::vector<SurfaceGroup*>());
	std::vector<SurfaceGroup*> cleanedShapeList;
	for (size_t i = 0; i < shapeList.size(); i++)
	{ 
		SurfaceGroup* currentSurfaceGroup = shapeList[i];
		if (currentSurfaceGroup->testIsVisable(shapeList, true))
		{
			faceList_[0].emplace_back(currentSurfaceGroup);
		}
	}

	printTime(startTime, std::chrono::high_resolution_clock::now());

	startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Construct footprints" << std::endl;

	// create building footprints 
	std::vector<Edge*> edgeList = makeJumbledGround();

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

	footPrintList_ = outerEdges2Shapes(outerList);
	printTime(startTime, std::chrono::high_resolution_clock::now());

	hasFootPrint_ = true;

	// sort surface groups based on the footprints
	if (footPrintList_.size() == 1)
	{
		hasSortedFaces_ = true;
		return;
	}

	startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Sort roofing structures" << std::endl;
	std::vector<SurfaceGroup*> tempSurfaceGroup = faceList_[0];
	faceList_.clear();

	for (size_t i = 0; i < footPrintList_.size(); i++) { 
		faceList_.emplace_back(std::vector<SurfaceGroup*>()); 	
	}

	TopExp_Explorer expl;
	for (size_t i = 0; i < tempSurfaceGroup.size(); i++)
	{
		bool found = false;
		for (expl.Init(tempSurfaceGroup[i]->getProjectedFace(), TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			for (size_t j = 0; j < footPrintList_.size(); j++)
			{
				BRepExtrema_DistShapeShape distanceCalc(footPrintList_[j], vertex);

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

	hasSortedFaces_ = true;
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return;
}


std::vector<TopoDS_Shape> CJGeoCreator::computePrisms(bool isFlat)
{
	std::vector<TopoDS_Shape> prismList;

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

	bool allSolids = true;

	for (size_t i = 0; i < faceList_.size(); i++)
	{
		if (faceList_[i].size() == 0) { continue; }
		TopTools_ListOfShape aLSFuseObjects;
		TopExp_Explorer expl;
		for (size_t j = 0; j < faceList_[i].size(); j++)
		{
			SurfaceGroup* currentRoof = faceList_[i][j]; //TODO: make work for multiple

			if (currentRoof->getURRPoint().Z() == 0) //TODO: make smarter
			{
				continue;
			}

			TopoDS_Face currentFace;

			if (!isFlat) { currentFace = currentRoof->getFace(); }
			else { currentFace = currentRoof->getFlatFace(); }

			BRepPrimAPI_MakePrism sweeper(currentFace, gp_Vec(0, 0, -currentRoof->getURRPoint().Z() - 5), Standard_True);
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
		aBuilder.SetFuzzyValue(1e-7);
		aBuilder.SetRunParallel(Standard_True);
		aBuilder.Perform();

		std::vector<TopoDS_Face> ObjectFaceList;
		for (expl.Init(aBuilder.Shape(), TopAbs_FACE); expl.More(); expl.Next()) {
			ObjectFaceList.emplace_back(TopoDS::Face(expl.Current()));
		}

		if (ObjectFaceList.size() == 0) { continue; }

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
		// check if all the vertices have overlap
		bool success = false;
		for (size_t j = 0; j < cleanedFaceList.size(); j++)
		{
			int vertCount = 0;
			int overlapCount = 0;

			for (TopExp_Explorer vertexExplorer(cleanedFaceList[j], TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next()) {

				bool hasOverlap = false;
				vertCount++;

				TopoDS_Vertex vertex = TopoDS::Vertex(vertexExplorer.Current());
				gp_Pnt point = BRep_Tool::Pnt(vertex);

				for (size_t k = 0; k < cleanedFaceList.size(); k++)
				{
					if (j == k)
					{
						continue;
					}
					for (TopExp_Explorer otherVertexExplorer(cleanedFaceList[k], TopAbs_VERTEX); otherVertexExplorer.More(); otherVertexExplorer.Next()) {
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
				brepSewer.Add(cleanedFaceList[j]);
				success = true;
			}
		}
		if (success)
		{
			brepSewer.Perform();
			if (brepSewer.SewedShape().ShapeType() == TopAbs_COMPOUND)
			{
				prismList.emplace_back(brepSewer.SewedShape());
				allSolids = false;
				continue;
			}

			brepBuilder.Add(solidShape, brepSewer.SewedShape());
			TopoDS_Shape cleanedSolid = simplefySolid(solidShape);
			prismList.emplace_back(cleanedSolid);
		}
	}
	if (!allSolids)
	{
		std::cout << "	Not all shapes could be converted to solids, output might be incorrect or inaccurate!" << std::endl;
	}
	return prismList;
}


TopoDS_Shape CJGeoCreator::simplefySolid(TopoDS_Shape solidShape)
{
	std::vector<TopoDS_Face> facelist;
	std::vector<gp_Dir> normalList;
	for (TopExp_Explorer expl(solidShape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		gp_Vec faceNomal = computeFaceNormal(face);

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

	std::vector<int> evalList(facelist.size());
	for (size_t i = 0; i < facelist.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		std::vector<TopoDS_Face> mergeList;
		mergeList.emplace_back(facelist[i]);

		TopoDS_Face currentFace = facelist[i];
		gp_Dir currentdir = normalList[i];
		int count = 0;

		while (true)
		{
			for (size_t j = 0; j < facelist.size(); j++) //TODO: step
			{
				if (i == j) { continue; }
				if (evalList[j] == 1) { continue; }

				TopoDS_Face otherFace = facelist[j];
				gp_Dir otherdir = normalList[j];

				if (!currentdir.IsParallel(otherdir, 1e-6)) { continue; }

				bool touching = false;
				for (size_t k = 0; k < mergeList.size(); k++)
				{
					for (TopExp_Explorer edgeExp(mergeList[k], TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
						TopoDS_Edge edge1 = TopoDS::Edge(edgeExp.Current());
						std::tuple<gp_Pnt, gp_Pnt> currentSE = getPointsEdge(edge1);

						// loop through the edges of face2
						for (TopExp_Explorer edgeExp2(otherFace, TopAbs_EDGE); edgeExp2.More(); edgeExp2.Next()) {
							TopoDS_Edge edge2 = TopoDS::Edge(edgeExp2.Current());
							std::tuple<gp_Pnt, gp_Pnt> otherSE = getPointsEdge(edge2);

							// check if the edges have the same vertices
							if (std::get<0>(currentSE).IsEqual(std::get<1>(otherSE), 1e-6) && std::get<1>(currentSE).IsEqual(std::get<0>(otherSE), 1e-6) ||
								std::get<1>(currentSE).IsEqual(std::get<1>(otherSE), 1e-6) && std::get<0>(currentSE).IsEqual(std::get<0>(otherSE), 1e-6)) {
								touching = true;
								break;
							}
						}

						if (touching) { break; }
					}
					if (touching) { break; }
				}

				if (touching) {
					evalList[j] = 1;
					mergeList.emplace_back(facelist[j]);
				}
			}

			if (mergeList.size() == 1)
			{
				brepSewer.Add(mergeList[0]);
				break;
			}
			else
			{
				if (count != mergeList.size())
				{
					count = mergeList.size();
				}
				else {
					TopoDS_Face mergedFace = mergeFaces(mergeList);
					if (mergedFace.IsNull())
					{
						for (size_t i = 0; i < mergeList.size(); i++) { brepSewer.Add(mergeList[i]); }
					}
					brepSewer.Add(mergedFace);
					break;
				}
			}
		}
	}
	brepSewer.Perform();

	if (brepSewer.SewedShape().ShapeType() == TopAbs_COMPOUND)
	{
		return brepSewer.SewedShape();
	}

	brepBuilder.Add(simpleBuilding, brepSewer.SewedShape());
	return simpleBuilding;
}


TopoDS_Face CJGeoCreator::mergeFaces(std::vector<TopoDS_Face> mergeFaces) {
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
		std::tuple<gp_Pnt, gp_Pnt> currentSE = getPointsEdge(edgeList[i]);

		if (std::get<0>(currentSE).IsEqual(std::get<1>(currentSE), 1e-6)) { continue; }

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (j == i) { continue; }
			if (evalList[j] == 1) { continue; }
			std::tuple<gp_Pnt, gp_Pnt> otherSE = getPointsEdge(edgeList[j]);

			if (std::get<0>(currentSE).IsEqual(std::get<1>(otherSE), 1e-6) && std::get<1>(currentSE).IsEqual(std::get<0>(otherSE), 1e-6) ||
				std::get<1>(currentSE).IsEqual(std::get<1>(otherSE), 1e-6) && std::get<0>(currentSE).IsEqual(std::get<0>(otherSE), 1e-6)) {
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

std::vector<int> CJGeoCreator::getTypeValuesBySample(TopoDS_Shape prism, int prismNum, bool flat) {
	std::vector<int> valueList;

	for (TopExp_Explorer faceExp(prism, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		valueList.emplace_back(1);
	}

	std::vector<TopoDS_Face> faceList;
	if (flat)
	{
		for (size_t i = 0; i < faceList_[prismNum].size(); i++)
		{
			faceList.emplace_back(faceList_[prismNum][i]->getFlatFace());
		}
	}
	else {
		for (size_t i = 0; i < faceList_[prismNum].size(); i++)
		{
			faceList.emplace_back(faceList_[prismNum][i]->getFace());
		}
	}

	double lowestZ = std::numeric_limits<double>::max();
	int lowestFaceIdx = 0;
	int faceIdx = 0;

	for (TopExp_Explorer faceExp(prism, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
		TopoDS_Face currentface = TopoDS::Face(faceExp.Current());
		gp_Pnt evalpoint = getPointOnFace(currentface);
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


void CJGeoCreator::printTime(std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime) {
	LONGLONG duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5)
	{
		std::cout << "	Successfully finished in: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() << "ms" << std::endl;
	}
	else {
		std::cout << "	Successfully finished in: " << std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count() << "s" << std::endl;
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
			if (ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
			if (yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ - 1); }
			if (zSmall) { neightbours.emplace_back(voxelIndx - xRelRange_ * yRelRange_ - 1); }
			if (zBig) { neightbours.emplace_back(voxelIndx + xRelRange_ * yRelRange_ - 1); }
		}

	}
	if (xBig) 
	{ 
		neightbours.emplace_back(voxelIndx + 1); 

		if (!connect6)
		{
			if (ySmall) { neightbours.emplace_back(voxelIndx - xRelRange_ - 1); }
			if (yBig) { neightbours.emplace_back(voxelIndx + xRelRange_ + 1); }
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
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Coarse filtering of roofing structures" << std::endl;

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
			//printPoint(lll);
			//printPoint(urr);
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
	printTime(startTime, std::chrono::high_resolution_clock::now());
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

		gp_Vec faceNormal = computeFaceNormal(face);

		if (std::abs(faceNormal.Z() - 0) < 0.001) { continue;}
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
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 0.0 Model" << std::endl;


	gp_Pnt lll = cluster->getLllPoint();
	gp_Pnt urr = cluster->getUrrPoint();
	double rotationAngle = cluster->getDirection();

	// LoD 0.0
	gp_Pnt p0 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), 0), -rotationAngle);
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), 0), -rotationAngle);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Shape floorProjection = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));

	CJT::GeoObject* geoObject = kernel->convertToJSON(floorProjection.Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "0.0");

	std::map<std::string, std::string> semanticData;
	semanticData.emplace("type", "RoofSurface");
	geoObject->appendSurfaceData(semanticData);
	geoObject->appendSurfaceTypeValue(0);
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObject;
}


std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD02(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 0.2 Model" << std::endl;
	if (!hasTopFaces_ || !hasFootPrint_) { initializeBasic(cluster); }

	std::vector< CJT::GeoObject*> geoObjectList;

	std::map<std::string, std::string> semanticData;
	semanticData.emplace("type", "RoofSurface");

	for (size_t i = 0; i < footPrintList_.size(); i++)
	{
		CJT::GeoObject* geoObject = kernel->convertToJSON(footPrintList_[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "0.2");
		geoObject->appendSurfaceData(semanticData);
		geoObject->appendSurfaceTypeValue(0);
		geoObjectList.emplace_back(geoObject);
	}
	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}

CJT::GeoObject* CJGeoCreator::makeLoD10(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.0 Model" << std::endl;
	gp_Pnt lll = cluster->getLllPoint();
	gp_Pnt urr = cluster->getUrrPoint();
	double rotationAngle = cluster->getDirection();

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid bbox;
	brepBuilder.MakeSolid(bbox);

	gp_Pnt p0 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), 0), -rotationAngle);
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), 0), -rotationAngle);

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
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge8, edge5, edge11)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge3, edge9, edge6, edge8)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge2, edge10, edge7, edge9)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge1, edge11, edge4, edge10)));
	faceList.emplace_back(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge4, edge5, edge6, edge7)));
	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }

	brepSewer.Perform();
	brepBuilder.Add(bbox, brepSewer.SewedShape());

	CJT::GeoObject* geoObject = kernel->convertToJSON(bbox.Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "1.0");
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

std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD12(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.2 Model" << std::endl;
	if (!hasTopFaces_ || !hasFootPrint_) { initializeBasic(cluster); }

	std::vector< CJT::GeoObject*> geoObjectList;
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

		CJT::GeoObject* geoObject = kernel->convertToJSON(extrudedShape.Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "1.2");
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

std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD13(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 1.3 Model" << std::endl;
	std::vector< CJT::GeoObject*> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(cluster);
	}
	if (footPrintList_.size() == 0)
	{
		return geoObjectList;
	}

	std::vector<TopoDS_Shape> prismList = computePrisms(true);

	for (size_t i = 0; i < prismList.size(); i++)
	{
		CJT::GeoObject* geoObject = kernel->convertToJSON(prismList[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "1.3");
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

std::vector< CJT::GeoObject*> CJGeoCreator::makeLoD22(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale) 
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 2.2 Model" << std::endl;
	std::vector< CJT::GeoObject*> geoObjectList;

	if (!hasTopFaces_)
	{
		initializeBasic(cluster);
	}
	if (footPrintList_.size() == 0)
	{
		return geoObjectList;
	}
	
	std::vector<TopoDS_Shape> prismList = computePrisms(false);

	for (size_t i = 0; i < prismList.size(); i++)
	{
		CJT::GeoObject* geoObject = kernel->convertToJSON(prismList[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "2.2");
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

std::vector< CJT::GeoObject*>CJGeoCreator::makeLoD32(helperCluster* cluster, CJT::CityCollection* cjCollection, CJT::Kernel* kernel, int unitScale)
{
	auto startTime = std::chrono::high_resolution_clock::now();
	std::cout << "- Computing LoD 3.2 Model" << std::endl;
	// asign rooms
	int roomnum = 0;
	int temps = 0;

	std::vector< CJT::GeoObject*> geoObjectList;

	TopoDS_Shape outSideShape;

	std::vector<int> totalRoom;
	std::cout << "\tExterior space growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting())
		{
			totalRoom = growExterior(i, roomnum, cluster);
			break;
		}
	}
	std::cout << std::endl;
	if (totalRoom.size() == 0)
	{
		std::cout << "Unable to find exterior space" << std::endl;
		return geoObjectList;
	}

	std::cout << "\tExterior space succesfully grown" << std::endl << std::endl;


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
			if (std::get<3>(lookup)) { shape = std::get<4>(lookup); }
			else { shape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup), true); }

			bool hasSolid = false;

			for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
				aLSTools.Append(TopoDS::Solid(expl.Current()));
				hasSolid = true;	
			}

			if (!hasSolid)
			{
				for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
					aLSTools.Append(TopoDS::Face(expl.Current()));
				}
			}
		}
	}
	std::cout << "\tProcessing Surfaces (this process might take a while)" << std::endl;
	aLSTools.Reverse();
	aSplitter.SetArguments(aLSObjects);
	aSplitter.SetTools(aLSTools);
	aSplitter.SetRunParallel(Standard_True);
	aSplitter.SetNonDestructive(Standard_True);

	aSplitter.Perform();

	const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

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

	std::vector<TopoDS_Shape> OuterShapeList;
	std::vector<double> massList;
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
		if (found)
		{
			continue;
		}

		GProp_GProps gprop;
		BRepGProp::VolumeProperties(shellList[j], gprop);
		massList.emplace_back(abs(gprop.Mass()));

		OuterShapeList.emplace_back(shellList[j]);
	}

	double massThreshold = 27;
	for (size_t i = 0; i < OuterShapeList.size(); i++)
	{
		if (massList[i] < massThreshold) { continue; }
		TopoDS_Shape cleanedSolid = simplefySolid(OuterShapeList[i]);
		CJT::GeoObject* geoObject = kernel->convertToJSON(cleanedSolid.Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "3.0");
		geoObjectList.emplace_back(geoObject);
	}

	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}

CJGeoCreator::CJGeoCreator(helperCluster* cluster, double vSize, bool isFlat)
{
	double xySize;
	double zSize;

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

		xySize = std::stod(stringXYSize);
		zSize = std::stod(stringZSize);
	}
	else
	{
		voxelSize_ = vSize;
		voxelSizeZ_ = vSize;

		xySize = vSize;
		zSize = vSize;
	}

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
	initializeBasic(cluster);
	std::cout << std::endl;
	std::cout << "- Processing" << std::endl;
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
				std::cout << "\tSize: " << totalRoom.size() << "\r";
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

						//IfcSchema::IfcProduct* product = std::get<0>(lookup);	
						//if (!product->hasRepresentation()) { continue; }

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


