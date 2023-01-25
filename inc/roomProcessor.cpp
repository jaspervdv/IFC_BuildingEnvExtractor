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


gp_Pnt* linearLineIntersection(TopoDS_Edge edge1, TopoDS_Edge edge2) {
	std::tuple<gp_Pnt, gp_Pnt> points1 = getPointsEdge(edge1);
	std::tuple<gp_Pnt, gp_Pnt> points2 = getPointsEdge(edge2);

	gp_Pnt sP1 = std::get<0>(points1);
	gp_Pnt eP1 = std::get<1>(points1);
	gp_Pnt sP2 = std::get<0>(points2);
	gp_Pnt eP2 = std::get<1>(points2);

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
	
	if (x1 == xI && y1 == yI ||
		x2 == xI && y2 == yI 
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


TopoDS_Face getFlatFace(TopoDS_Face face) {
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
				//faceWire = BRepBuilderAPI_MakeWire(faceWire, edge);
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


std::vector<TopoDS_Edge> makeJumbledGround(std::vector<TopoDS_Face> faceList) {

	double buffer = 0.01; //TODO: have it scale with units

	gp_Ax2 axis(
		gp_Pnt(0, 0, 0), 
		gp_Dir(0, 0, 100)
	);
	HLRBRep_Algo* Edgeprojector = new HLRBRep_Algo();
	Edgeprojector->Projector(HLRAlgo_Projector(axis));
	
	for (size_t i = 0; i < faceList.size(); i++)
	{
		Edgeprojector->Add(faceList[i]);
	}

	Edgeprojector->Update();
	HLRBRep_HLRToShape projectToShape(Edgeprojector);
	TopoDS_Shape flattenedEdges = projectToShape.CompoundOfEdges(HLRBRep_Undefined, true, false);

	std::vector<TopoDS_Edge> edgeList;
	std::vector<TopoDS_Edge> splitEdgeList;

	// remove duplicate edges
	TopExp_Explorer expl;
	for (expl.Init(flattenedEdges, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		std::tuple<gp_Pnt, gp_Pnt> points = getPointsEdge(currentEdge);
		gp_Pnt startPoint = std::get<0>(points);
		gp_Pnt endPoint = std::get<1>(points);

		bool dub = false;
		for (size_t i = 0; i < edgeList.size(); i++)
		{
			std::tuple<gp_Pnt, gp_Pnt> otherPoints = getPointsEdge(edgeList[i]);
			gp_Pnt otherStartPoint = std::get<0>(otherPoints);
			gp_Pnt otherEndPoint = std::get<1>(otherPoints);

			if (startPoint.IsEqual(otherStartPoint, 0.0001) && endPoint.IsEqual(otherEndPoint, 0.0001) ||
				endPoint.IsEqual(otherStartPoint, 0.0001) && startPoint.IsEqual(otherEndPoint, 0.0001))
			{
				dub = true;
			}
		}
		if (!dub)
		{
			edgeList.emplace_back(currentEdge);
		}
	}
	// merge lines that are on the same plane
	std::vector<TopoDS_Edge> cleanedEdgeList;
	std::vector<int> evalList(edgeList.size());
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		std::tuple<gp_Pnt, gp_Pnt> points = getPointsEdge(edgeList[i]);
		gp_Pnt startPoint = std::get<0>(points);
		gp_Pnt endPoint = std::get<1>(points);
		gp_Lin2d line = gce_MakeLin2d(
			gp_Pnt2d(startPoint.X(), startPoint.Y()),
			gp_Pnt2d(endPoint.X(), endPoint.Y())
		).Value();

		gp_Vec2d dir(gp_Pnt2d(startPoint.X(), startPoint.Y()), gp_Pnt2d(endPoint.X(), endPoint.Y()));
		dir.Normalize();

		std::vector<gp_Pnt> mergeList;
		std::vector<int> evalIndxList;


		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (i == j) { continue; }
			if (evalList[j] == 1) { continue; }

			std::tuple<gp_Pnt, gp_Pnt> otherPoints = getPointsEdge(edgeList[j]);
			gp_Pnt otherStartPoint = std::get<0>(otherPoints);
			gp_Pnt otherEndPoint = std::get<1>(otherPoints);

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
				cleanedEdgeList.emplace_back(edgeList[i]);
			}
			continue;
		}

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

				std::tuple<gp_Pnt, gp_Pnt> otherPoints = getPointsEdge(edgeList[edgeIdx]);
				gp_Pnt otherStartPoint = std::get<0>(otherPoints);
				gp_Pnt otherEndPoint = std::get<1>(otherPoints);

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

				if (x3 - buffer <= x1 && x2 <= x4 + buffer)
				{
					evalList[edgeIdx] = 1;
					startPoint = otherStartPoint;
					endPoint = otherEndPoint;
					grow++;
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
		cleanedEdgeList.emplace_back(edge);
	}

	edgeList = cleanedEdgeList;

	for (size_t i = 0; i < edgeList.size(); i++)
	{
		std::vector<gp_Pnt*> intPoints;
		std::tuple<gp_Pnt, gp_Pnt> points = getPointsEdge(edgeList[i]);
		gp_Pnt startPoint = std::get<0>(points);
		gp_Pnt endPoint = std::get<1>(points);
		//printPoint(startPoint);
		//printPoint(endPoint);

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (i == j){ continue; }
			gp_Pnt* intersection = linearLineIntersection(edgeList[i], edgeList[j]);

			if (intersection == nullptr) { continue; }

			intPoints.emplace_back(intersection);
			//printPoint(*intersection);
		}
		
		std::vector<gp_Pnt*> cleanedPoints;
		for (size_t j = 0; j < intPoints.size(); j++)
		{
			bool dub = false;
			for (size_t k = 0; k < cleanedPoints.size(); k++)
			{
				if (intPoints[j]->IsEqual(*cleanedPoints[k], 0.0001))
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
				splitEdgeList.emplace_back(edge);
				currentPoint = *cleanedPoints[nextPoint];

			}
			else {
				if (currentPoint.IsEqual(endPoint, 0.0001))
				{
					break;
				}
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentPoint, endPoint);
				splitEdgeList.emplace_back(edge);
				break;
			}
		}
	}
	return splitEdgeList;
}

std::vector<TopoDS_Edge> getOuterEdges(std::vector<TopoDS_Edge> jumbledSurface, std::vector<TopoDS_Face> flatFaceList) {
	std::vector<TopoDS_Edge> outerEdgeList;

	TopExp_Explorer expl;
	for (size_t ii = 0; ii < jumbledSurface.size(); ii++)
	{
		TopoDS_Edge currentEdge = jumbledSurface[ii];
		int counter = 0;

		std::tuple<gp_Pnt, gp_Pnt> edgePoints = getPointsEdge(currentEdge);

		gp_Pnt startPoint = std::get<0>(edgePoints);
		gp_Pnt endPoint = std::get<1>(edgePoints);

		if (startPoint.IsEqual(endPoint, 0.0001))
		{
			continue;
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
		double div = 100;
		gp_Pnt2d evalPoint1 = middlePoint.Translated(gp_Vec2d(lineVec.Y() * -1 / div, lineVec.X() / div));
		gp_Pnt2d evalPoint2 = middlePoint.Translated(gp_Vec2d(lineVec.Y() / div, lineVec.X() * -1 / div));

		bool hasEval1 = false;
		bool hasEval2 = false;
		for (size_t i = 0; i < flatFaceList.size(); i++)
		{
			TopoDS_Face evalFace = flatFaceList[i];
			if (!hasEval1)
			{
				BRepExtrema_DistShapeShape distanceCalc(
					evalFace,
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
					evalFace,
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
			outerEdgeList.emplace_back(currentEdge);
		}
	}

	return outerEdgeList;
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

		if (it->second->getRoomNumbers().size() == 0) { continue; }
		if (!it->second->getIsInside()) { continue; }
		if (it->second->getIsIntersecting()) { continue; }

		for (size_t k = 0; k < pointList.size(); k++)
		{
			storageFile << pointList[k].X() << ", " << pointList[k].Y() << ", " << pointList[k].Z() << std::endl;
		}

		storageFile << it->second->getRoomNumbers().back() << std::endl;
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

std::vector<TopoDS_Face> CJGeoCreator::getXYFaces(TopoDS_Shape shape) {
	
	std::vector<TopoDS_Face> faceList;
	
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) {
		TopoDS_Face face = TopoDS::Face(expl.Current());
		Handle(Geom_Surface) Surface = BRep_Tool::Surface(face);

		// ignore if the z component of normal is 0 
		GeomAdaptor_Surface theGASurface(Surface);
		if (theGASurface.GetType() != GeomAbs_Plane) {
			continue;
		}
		GeomLProp_SLProps SLProps(Surface, 1, 0.1);
		gp_Pnt2d P2d;
		SLProps.SetParameters(P2d.X(), P2d.Y());
		gp_Dir direc = SLProps.Normal();
		double magnitude = direc.X() + direc.Y() + direc.Z();

		if (direc.Z()/magnitude < 0.0001 && direc.Z()/magnitude > - 0.0001) { continue; }
		faceList.emplace_back(face);
	}

	// ignore lowest if identical projected points
	std::vector<TopoDS_Face> cleanedFaceList;
	for (size_t i = 0; i < faceList.size(); i++)
	{
		bool isHidded = false;
		double height = getAvFaceHeight(faceList[i]);
		for (size_t j = 0; j < faceList.size(); j++)
		{
			if (i == j) { continue; }

			double otherHeight = getAvFaceHeight(faceList[j]);

			if (otherHeight >  height) { continue; }

			int vertCount = 0;
			int overlapCount = 0;

			TopExp_Explorer expl;
			TopExp_Explorer expl1;
			for (expl.Init(faceList[i], TopAbs_VERTEX); expl.More(); expl.Next()) {
				vertCount++;
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt point = BRep_Tool::Pnt(vertex);
				point = gp_Pnt(point.X(), point.Y(), 0);

				for (expl1.Init(faceList[j], TopAbs_VERTEX); expl1.More(); expl1.Next()) {
					TopoDS_Vertex otherVertex = TopoDS::Vertex(expl1.Current());
					gp_Pnt otherPoint = BRep_Tool::Pnt(otherVertex);
					otherPoint = gp_Pnt(otherPoint.X(), otherPoint.Y(), 0);

					if (otherPoint.IsEqual(point, 0.0001))
					{
						overlapCount++;
						break;
					}
				}
			}
			if (vertCount == overlapCount)
			{
				isHidded = true;
				break;
			}
		}
		if (!isHidded)
		{
			cleanedFaceList.emplace_back(faceList[i]);
		}
	}

	return cleanedFaceList;
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
	// get slabs and walls
	std::vector<TopoDS_Face> shapeList;

	std::vector<TopoDS_Shape> test = getTopObjects(cluster);

	for (size_t i = 0; i < test.size(); i++)
	{
		std::vector<TopoDS_Face> objectFaces = getXYFaces(test[i]);
		for (size_t j = 0; j < objectFaces.size(); j++)
		{
			shapeList.emplace_back(objectFaces[j]);
		}
	}

	// project to 2D
	std::vector<TopoDS_Face> flatFaceList;
	TopExp_Explorer expl;
	for (size_t i = 0; i < shapeList.size(); i++)
	{
		TopoDS_Face face = getFlatFace(shapeList[i]);

		if (!face.IsNull())
		{
			flatFaceList.emplace_back(face);
		}		
	}
	std::vector<TopoDS_Edge> jumbledSurface = makeJumbledGround(flatFaceList);
	// find outer edge
	std::vector<TopoDS_Edge> edgeList = getOuterEdges(jumbledSurface, flatFaceList);

	for (size_t i = 0; i < edgeList.size(); i++)
	{
		std::tuple<gp_Pnt, gp_Pnt> pp = getPointsEdge(edgeList[i]);
		gp_Pnt startPoint = std::get<0>(pp);
		gp_Pnt endPoint = std::get<1>(pp);
		//printPoint(startPoint);
		//printPoint(endPoint);
	}

	// TODO: find if interior ring
	BRepBuilderAPI_MakeFace faceBuilder;

	TopoDS_Edge currentEdge = edgeList[0];
	std::vector<int> evaluated(edgeList.size());
	evaluated[0] = 1;
	TopoDS_Wire faceWire;

	gp_Pnt connectionPoint = std::get<1>(getPointsEdge(edgeList[0]));

	std::vector<TopoDS_Edge> orderedEdgeList;
	orderedEdgeList.emplace_back(currentEdge);

	std::vector< CJT::GeoObject*> geoObjectList;

	bool con = true;
	while (con)
	{
		con = false;
		for (size_t i = 0; i < edgeList.size(); i++)
		{
			if (evaluated[i] == 1)
			{
				continue;
			}

			std::tuple<gp_Pnt, gp_Pnt> pp = getPointsEdge(edgeList[i]);
			if (connectionPoint.IsEqual(std::get<0>(pp), 0.0001))
			{
				orderedEdgeList.emplace_back(edgeList[i]);
				connectionPoint = std::get<1>(pp);
				evaluated[i] = 1;
				i = 0;
			}
			else if (connectionPoint.IsEqual(std::get<1>(pp), 0.0001))
			{
				orderedEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(std::get<1>(pp), std::get<0>(pp)));
				connectionPoint = std::get<0>(pp);
				evaluated[i] = 1;
				i = 0;
			}
		}

		int count = 0;
		std::vector<int> merged(orderedEdgeList.size());
		gp_Vec evalVec = getDirEdge(orderedEdgeList[0]);
		for (size_t i = 0; i < orderedEdgeList.size(); i++)
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
		for (size_t i = 0; i < orderedEdgeList.size(); i++)
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
			faceWire = BRepBuilderAPI_MakeWire(faceWire, BRepBuilderAPI_MakeEdge(connection, endPoint));
			connection = endPoint;

			count++;
		}

		orderedEdgeList.clear();

		if (!faceWire.IsNull() && count >= 3)
		{
			faceBuilder = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), faceWire);
			if (faceBuilder.Error() == BRepBuilderAPI_FaceDone)
			{
				footPrintList_.emplace_back(faceBuilder.Face());
				hasFootPrint_ = true;
				geoObjectList.emplace_back(kernel->convertToJSON(faceBuilder.Shape(), "0.2"));
			}
		}

		for (size_t i = 0; i < edgeList.size(); i++) 
		{
			if (evaluated[i] == 0)
			{
				faceWire.Nullify();
				faceWire = TopoDS_Wire();
				connectionPoint = std::get<1>(getPointsEdge(edgeList[i]));
				con = true;
				evaluated[i] = 1;
				break;
			}
		}
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
	if (!hasFootPrint_)
	{
		makeLoD02(cluster, cjCollection, kernel, unitScale);
	}

	std::vector< CJT::GeoObject*> geoObjectList;
	double height = cluster->getUrrPoint().Z();
	
	for (size_t i = 0; i < footPrintList_.size(); i++)
	{
		TopoDS_Face currentFootprint = footPrintList_[i];
		int counter = 0;
		gp_Pnt lastPoint;

		BRep_Builder brepBuilder;
		BRepBuilderAPI_Sewing brepSewer;
		TopoDS_Shell shell;
		brepBuilder.MakeShell(shell);
		TopoDS_Solid solidShape;
		brepBuilder.MakeSolid(solidShape);

		BRepBuilderAPI_MakeWire topWireMaker;

		TopExp_Explorer expl;
		for (expl.Init(currentFootprint, TopAbs_VERTEX); expl.More(); expl.Next()) {
			counter++;
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);

			if (counter % 2 != 0)
			{
				lastPoint = point;
				continue;
			}

			gp_Pnt hPoint = gp_Pnt(point.X(), point.Y(), height);
			gp_Pnt hLastPoint = gp_Pnt(lastPoint.X(), lastPoint.Y(), height);

			// create side face
			TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(point, hPoint);
			TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(hPoint, hLastPoint);
			TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(hLastPoint, lastPoint);
			TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(lastPoint, point);
			brepSewer.Add(BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3)));

			topWireMaker.Add(BRepBuilderAPI_MakeEdge(hLastPoint, hPoint));
		}

		topWireMaker.Build();
		brepSewer.Add(BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), topWireMaker.Wire()));
		brepSewer.Add(currentFootprint);

		brepSewer.Perform();
		brepBuilder.Add(solidShape, brepSewer.SewedShape());

		CJT::GeoObject* geoObject = kernel->convertToJSON(solidShape, "1.2");
		geoObjectList.emplace_back(geoObject);
	}
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

		if (i % 250 == 0)
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
			if (j % 250 == 0)
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
				currentBoxel->addRoomNumber(roomnum);
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