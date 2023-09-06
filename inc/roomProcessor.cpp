#include "roomProcessor.h"

#include <chrono>

#include <Bnd_Box.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
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
#include <gce_MakeLin2d.hxx>
#include <Geom_Surface.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <TopoDS.hxx>

#include <CJToKernel.h>

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


gp_Pnt& getFirstPointShape(TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex startVertex;

	if (vertexExplorer.More()) {
		startVertex = TopoDS::Vertex(vertexExplorer.Current());
	}

	gp_Pnt startPoint = BRep_Tool::Pnt(startVertex);
	return startPoint;
}


gp_Pnt& getLastPointShape(TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex endVertex;

	while (vertexExplorer.More()) {
		endVertex = TopoDS::Vertex(vertexExplorer.Current());
		vertexExplorer.Next();
	}
	return BRep_Tool::Pnt(endVertex);;
}

gp_Vec getDirEdge(TopoDS_Edge edge) {
	gp_Pnt startpoint = getFirstPointShape(edge);
	gp_Pnt endpoint = getLastPointShape(edge);

	if (startpoint.Distance(endpoint) == 0)  { return gp_Vec(0, 0, 0); }

	gp_Vec vec = gp_Vec(startpoint, endpoint);
	return vec.Normalized();
}

TopoDS_Wire mergeWireOrientated(const TopoDS_Wire& baseWire, const TopoDS_Wire& mergingWire) { // merges both the wires in the correct order
	gp_Pnt connectionPoint1 = getFirstPointShape(baseWire);
	gp_Pnt connectionPoint2 = getLastPointShape(baseWire);

	gp_Pnt p1 = getFirstPointShape(mergingWire);
	gp_Pnt p2 = getLastPointShape(mergingWire);

	BRepBuilderAPI_MakeWire wiremaker;

	if (connectionPoint1.Distance(p2) < 1e-6) // correct orentation placed in front
	{

		for (TopExp_Explorer explorer(mergingWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}

		for (TopExp_Explorer explorer(baseWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}
		
		if (wiremaker.IsDone()) { return wiremaker.Wire(); }
	}
	if (connectionPoint1.Distance(p1) < 1e-6) // wrong orentation placed in front
	{
		std::vector<TopoDS_Edge> tempEdgeList;
		for (TopExp_Explorer explorer(mergingWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			gp_Pnt beginPoint = getFirstPointShape(edge);
			gp_Pnt endPoint = getLastPointShape(edge);
			tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(endPoint, beginPoint));
		}

		for (std::vector<TopoDS_Edge>::reverse_iterator i = tempEdgeList.rbegin();
			i != tempEdgeList.rend(); ++i) {
			
			wiremaker.Add(*i);
		}

		for (TopExp_Explorer explorer(baseWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}

		if (wiremaker.IsDone()) { return wiremaker.Wire(); }
	}
	if (connectionPoint2.Distance(p1) < 1e-6) // correct orentation placed after
	{
		for (TopExp_Explorer explorer(baseWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}

		for (TopExp_Explorer explorer(mergingWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}

		if (wiremaker.IsDone()) { return wiremaker.Wire(); }
	}
	if (connectionPoint2.Distance(p2) < 1e-6) // wrong orentation placed after
	{

		for (TopExp_Explorer explorer(baseWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			wiremaker.Add(edge);
		}

		std::vector<TopoDS_Edge> tempEdgeList;
		for (TopExp_Explorer explorer(mergingWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			gp_Pnt beginPoint = getFirstPointShape(edge);
			gp_Pnt endPoint = getLastPointShape(edge);
			tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(endPoint, beginPoint));
		}

		for (std::vector<TopoDS_Edge>::reverse_iterator i = tempEdgeList.rbegin();
			i != tempEdgeList.rend(); ++i) {

			wiremaker.Add(*i);
		}

		if (wiremaker.IsDone()) { return wiremaker.Wire(); }
	}

}

TopoDS_Wire closeWireOrientated(const TopoDS_Wire& baseWire) {
	gp_Pnt p1 = getFirstPointShape(baseWire);
	gp_Pnt p2 = getLastPointShape(baseWire);

	if (p1.Distance(p2) < 1e-6) { return baseWire; }

	TopoDS_Wire closingWire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(p2, p1));

	return mergeWireOrientated(baseWire, closingWire);
}

bg::model::box <BoostPoint3D> createBBox(TopoDS_Shape shape) {

	double buffer = 0.05;
	Bnd_Box boundingBox;
	BRepBndLib::Add(shape, boundingBox);

	// Step 3: Get the bounds of the bounding box.
	Standard_Real minX, minY, minZ, maxX, maxY, maxZ;
	boundingBox.Get(minX, minY, minZ, maxX, maxY, maxZ);

	return  bg::model::box < BoostPoint3D >(
		BoostPoint3D(minX - buffer, minY - buffer, minZ - buffer),
		BoostPoint3D(maxX + buffer, maxY + buffer, maxZ + buffer)
		);
}




bg::model::box <BoostPoint3D> createBBox(gp_Pnt p1, gp_Pnt p2) {

	gp_Pnt lll;
	gp_Pnt urr;

	// get proper order for the bbox
	if (p1.X() > p2.X())
	{
		lll.SetX(p2.X());
		urr.SetX(p1.X());
	}
	else
	{
		lll.SetX(p1.X());
		urr.SetX(p2.X());
	}

	if (p1.Y() > p2.Y())
	{
		lll.SetY(p2.Y());
		urr.SetY(p1.Y());
	}
	else
	{
		lll.SetY(p1.Y());
		urr.SetY(p2.Y());
	}

	if (p1.Z() > p2.Z())
	{
		lll.SetZ(p2.Z());
		urr.SetZ(p1.Z());
	}
	else
	{
		lll.SetZ(p1.Z());
		urr.SetZ(p2.Z());
	}

	if ((lll.X() - urr.X()) < 0.05)
	{
		lll.SetX(lll.X() - 0.05);
		urr.SetX(urr.X() + 0.05);
	}

	if ((lll.Y() - urr.Y()) < 0.05)
	{
		lll.SetY(lll.Y() - 0.05);
		urr.SetY(urr.Y() + 0.05);
	}

	if ((lll.Z() - urr.Z()) < 0.05)
	{
		lll.SetZ(lll.Z() - 0.05);
		urr.SetZ(urr.Z() + 0.05);
	}

	BoostPoint3D boostlllPoint = BoostPoint3D(lll.X(), lll.Y(), lll.Z());
	BoostPoint3D boosturrPoint = BoostPoint3D(urr.X(), urr.Y(), urr.Z());

	bg::model::box <BoostPoint3D> box = bg::model::box < BoostPoint3D >(boostlllPoint, boosturrPoint);

	return box;
}

gp_Pnt* linearLineIntersection(gp_Pnt sP1, gp_Pnt eP1, gp_Pnt sP2, gp_Pnt eP2, bool projected, double buffer = 0.01) {
	
	if (projected)
	{
		sP1.SetZ(0);
		eP1.SetZ(0);
		sP2.SetZ(0);
		eP2.SetZ(0);
	}

	if (sP1.IsEqual(eP1, 1e-6)) { return nullptr; }
	if (sP2.IsEqual(eP2, 1e-6)) { return nullptr; }

	double z = 0; //Todo: make work in 3d
	if (!projected) { z = eP1.Z(); }

	gp_Vec v1(sP1, eP1);
	v1.Normalize();
	gp_Vec v2(sP2, eP2);
	v2.Normalize();

	if (v1.IsEqual(v2, 1e-6, 1e-6)) { return nullptr; }

	double x1 = sP1.X();
	double x2 = eP1.X();;
	double x3 = sP2.X();;
	double x4 = eP2.X();;

	double y1 = sP1.Y();
	double y2 = eP1.Y();
	double y3 = sP2.Y();
	double y4 = eP2.Y();

	double dom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	if (abs(dom) == 0) { return nullptr; }

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
					return new gp_Pnt(xI, yI, z);
				}
			}
		}
	}
	return nullptr;
}

gp_Pnt* linearLineIntersection(Edge* edge1, Edge* edge2, bool projected, double buffer = 0.01) {

	gp_Pnt sP1 = edge1->getStart();
	gp_Pnt eP1 = edge1->getEnd();
	gp_Pnt sP2 = edge2->getStart();
	gp_Pnt eP2 = edge2->getEnd();

	return linearLineIntersection(sP1, eP1, sP2, eP2, projected, buffer);
}


gp_Pnt* linearLineIntersection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, bool projected, double buffer = 0.01) {
	return linearLineIntersection(
		getFirstPointShape(edge1), 
		getLastPointShape(edge1), 
		getFirstPointShape(edge2), 
		getLastPointShape(edge2), 
		projected,
		buffer
	);
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

template<typename T>
bool isOverlappingCompletely(SurfaceGroup* evalFace, std::vector<SurfaceGroup*> facePool, T shapeIdx) {
	std::vector<Value> qResult;
	shapeIdx.query(bgi::intersects(
		bg::model::box <BoostPoint3D>(
			createBBox(evalFace->getFace())
			)), std::back_inserter(qResult));
	for (size_t i = 0; i < qResult.size(); i++)
	{
		if (isOverlappingCompletely(evalFace, facePool[qResult[i].second])) { return true; }
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

std::vector<Edge*> CJGeoCreator::getUniqueEdges(const TopoDS_Shape& flattenedEdges)
{
	TopExp_Explorer expl;
	std::vector<Edge*> UniqueEdgeList;

	for (expl.Init(flattenedEdges, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		if (!isInList(currentEdge, UniqueEdgeList))
		{
			UniqueEdgeList.emplace_back(new Edge(currentEdge));
		}
	}
	return UniqueEdgeList;
}

std::vector<Edge*> CJGeoCreator::getUniqueEdges(const std::vector<TopoDS_Edge>& flattenedEdges)
{
	std::vector<Edge*> UniqueEdgeList;

	for (size_t i = 0; i < flattenedEdges.size(); i++)
	{
		TopoDS_Edge currentEdge = flattenedEdges[i];
		if (!isInList(currentEdge, UniqueEdgeList))
		{
			UniqueEdgeList.emplace_back(new Edge(currentEdge));
		}
	}
	return UniqueEdgeList;
}

bool CJGeoCreator::isInList(TopoDS_Edge currentEdge, std::vector<Edge*> edgeList)
{
	gp_Pnt startPoint = getFirstPointShape(currentEdge);
	gp_Pnt endPoint = getLastPointShape(currentEdge);

	bool dub = false;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		gp_Pnt otherStartPoint = edgeList[i]->getProjectedStart();
		gp_Pnt otherEndPoint = edgeList[i]->getProjectedEnd();

		if (startPoint.IsEqual(otherStartPoint, 0.0001) && endPoint.IsEqual(otherEndPoint, 0.0001) ||
			endPoint.IsEqual(otherStartPoint, 0.0001) && startPoint.IsEqual(otherEndPoint, 0.0001))
		{
			return true;
		}
	}
	return false;
}


std::vector<Edge*> CJGeoCreator::mergeOverlappingEdges(std::vector<Edge*>& uniqueEdges, bool project)//, std::vector<TopoDS_Face> flatFaceList) 
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

		gp_Pnt startPoint;
		gp_Pnt endPoint;

		if (project)
		{
			startPoint = uniqueEdges[i]->getProjectedStart();
			endPoint = uniqueEdges[i]->getProjectedEnd();
		}
		else {
			startPoint = uniqueEdges[i]->getStart();
			endPoint = uniqueEdges[i]->getEnd();
		}

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

			gp_Pnt otherStartPoint;
			gp_Pnt otherEndPoint;

			if (project)
			{
				otherStartPoint = uniqueEdges[j]->getProjectedStart();
				otherEndPoint = uniqueEdges[j]->getProjectedEnd();
			}
			else {
				otherStartPoint = uniqueEdges[j]->getStart();
				otherEndPoint = uniqueEdges[j]->getEnd();
			}

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
			if (startPoint.Distance(endPoint) > 0.01) //TODO: make smarter
			{
				//if (isOuterEdge(edgeList[i], flatFaceList))
				//{
					cleanedEdgeList.emplace_back(uniqueEdges[i]);
				//}
			}
			continue;
		}

		discardIndx.emplace_back(i);

		// flip main merging edge
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

				gp_Pnt otherStartPoint;
				gp_Pnt otherEndPoint;

				if (project)
				{
					otherStartPoint = uniqueEdges[edgeIdx]->getProjectedStart();
					otherEndPoint = uniqueEdges[edgeIdx]->getProjectedEnd();
				}
				else
				{
					otherStartPoint = uniqueEdges[edgeIdx]->getStart();
					otherEndPoint = uniqueEdges[edgeIdx]->getEnd();
				}

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

	//TODO: change pointer to reference
	//for (size_t i = 0; i < discardIndx.size(); i++) { delete uniqueEdges[discardIndx[i]]; } 
	return cleanedEdgeList;
}


std::vector<Edge*> CJGeoCreator::splitIntersectingEdges(std::vector<Edge*>& edges, bool project) {
	
	std::vector<Edge*> splitEdgeList;
	std::vector<int> discardIndx;
	
	for (size_t i = 0; i < edges.size(); i++)
	{
		std::vector<gp_Pnt*> intPoints;

		gp_Pnt startPoint;
		gp_Pnt endPoint;

		if (project)
		{
			startPoint = edges[i]->getProjectedStart();
			endPoint = edges[i]->getProjectedEnd();
		}
		else {
			startPoint = edges[i]->getStart();
			endPoint = edges[i]->getEnd();
		}

		for (size_t j = 0; j < edges.size(); j++)
		{
			if (i == j) { continue; }
			gp_Pnt* intersection = linearLineIntersection(edges[i], edges[j], project);

			if (intersection == nullptr) { continue; }


			intPoints.emplace_back(intersection);
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
			
			if (linearLineIntersection(evalEdge1, edge, false, 1e-6) != nullptr) { intersectionCount1++; }
			if (linearLineIntersection(evalEdge2, edge, false, 1e-6) != nullptr) { intersectionCount2++; }
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
		if (isOuterEdge(edgeList[i], evalSurfList, spatialIndex)) { outerEdgeList.emplace_back(*currentEdge);
		}
	}

	return outerEdgeList;
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
	
	gp_Pnt originPoint = getFirstPointShape(edgeList[0]); // the original point of the original edge
	gp_Pnt extendingPoint = getLastPointShape(edgeList[0]); // the point from which will be extended

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

			gp_Pnt p1 = getFirstPointShape(otherEdge);
			gp_Pnt p2 = getLastPointShape(otherEdge);

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

			originPoint = getFirstPointShape(edgeList[i]); // the original point of the original edge
			extendingPoint = getLastPointShape(edgeList[i]); // the point from which will be extended
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

		gp_Pnt startpoint = getFirstPointShape(currentWire);
		gp_Pnt endpoint = getLastPointShape(currentWire);

		for (size_t i = 0; i < wireCollection.size(); i++) 
		{
			TopoDS_Wire otherwire = wireCollection[i];
			gp_Pnt otherStartpoint = getFirstPointShape(otherwire);
			gp_Pnt otherEndpoint = getLastPointShape(otherwire);

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
			currentWire = mergeWireOrientated(currentWire, BRepBuilderAPI_MakeWire(connectionEdge));
			currentWire = mergeWireOrientated(currentWire, wireCollection[idxMatch]);
			wireCollection.erase(wireCollection.begin() + idxMatch);
			stepped = true;
		}

		if (!stepped)
		{
			wireCollectionClosed.emplace_back(closeWireOrientated(currentWire));

			//for (TopExp_Explorer explorer(currentWire, TopAbs_VERTEX); explorer.More(); explorer.Next())
			//{
			//	const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer.Current());
			//	gp_Pnt p = BRep_Tool::Pnt(vertex);

			//	printPoint(p);
			//}

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

	gp_Pnt connection = getFirstPointShape(orderedEdgeList[0]);
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

		gp_Pnt endPoint = getLastPointShape(currentEdge);

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

			endPoint = getFirstPointShape(otherEdge);
			gp_Vec endPointVec = gp_Vec(connection, endPoint);

			if (endPointVec.Magnitude() == 0) { break; }
			else if (!currentVec.IsParallel(endPointVec, 0.01)){endPoint = getLastPointShape(otherEdge);}

			break;
		}

		if (connection.IsEqual(endPoint, 1e-6)) { continue; }

		wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));
		connection = endPoint;
	}

	if (connection.IsEqual(getFirstPointShape(orderedEdgeList[0]), 1e-6))
	{
		return wireMaker.Wire();
	}

	gp_Pnt finalPoint = getFirstPointShape(orderedEdgeList[0]);

	wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, finalPoint));
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

				originPoint = getFirstPointShape(edge);
				gp_Pnt endpoint = getLastPointShape(edge);

				vec1 = gp_Vec(originPoint, endpoint);
				vec2 = gp_Vec(originPoint, endpoint);
				while (vec1.IsParallel(vec2, 0.0001))
				{
					vertexExp.Next();
					edge = TopoDS::Edge(vertexExp.Current());

					gp_Pnt otherStartpoint = getFirstPointShape(edge);
					gp_Pnt otherEndpoint = getLastPointShape(edge);
					vec2 = gp_Vec(otherStartpoint, otherEndpoint);
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

	bgi::rtree<Value, bgi::rstar<treeDepth_>> shapeIdx;

	for (size_t i = 0; i < filteredFaces.size(); i++)
	{
		std::vector<SurfaceGroup*>  objectFaces = getXYFaces(filteredFaces[i]);
		for (size_t j = 0; j < objectFaces.size(); j++)
		{
			bool isDub = false;

			if (shapeList.size() <= 1)
			{
				shapeIdx.insert(std::make_pair(createBBox(objectFaces[j]->getFace()), shapeList.size()));
				shapeList.emplace_back(objectFaces[j]);
				hasTopFaces_ = true;
				continue;
			}
			if (!isOverlappingCompletely(objectFaces[j], shapeList, shapeIdx))
			{		
				shapeIdx.insert(std::make_pair(createBBox(objectFaces[j]->getFace()), shapeList.size()));
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
	std::cout << "- Construct roof outlines" << std::endl;

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

	roofOutlineList_ = outerEdges2Shapes(outerList);
	printTime(startTime, std::chrono::high_resolution_clock::now());

	hasGeoBase_ = true;

	// sort surface groups based on the footprints
	if (roofOutlineList_.size() == 1)
	{
		hasSortedFaces_ = true;
	}
	else {

		startTime = std::chrono::high_resolution_clock::now();
		std::cout << "- Sort roofing structures" << std::endl;
		std::vector<SurfaceGroup*> tempSurfaceGroup = faceList_[0];
		faceList_.clear();

		for (size_t i = 0; i < roofOutlineList_.size(); i++) {
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

		hasSortedFaces_ = true;
		printTime(startTime, std::chrono::high_resolution_clock::now());
	}
	return;
}

void CJGeoCreator::makeFootprint(helperCluster* cluster)
{
	// get footprint

	double floorlvl = cluster->getHelper(0)->getfootprintEvalLvl();
	std::cout << "- Corse filering footprint at z = " << floorlvl << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	double voxelCount = VoxelLookup_.size();
	double zlvls = voxelCount / (xRelRange_ * yRelRange_);
	double smallestDistanceToLvl = 999999;

	int groundfloorVoxelLvl;

	for (size_t i = 0; i < zlvls; i++)
	{
		voxel v = *VoxelLookup_[i * xRelRange_ * yRelRange_];

		double coreHeight = v.getCenterPoint().get<2>();
		double distanceToLvl = abs(floorlvl - coreHeight);

		if (distanceToLvl < smallestDistanceToLvl)
		{
			smallestDistanceToLvl = distanceToLvl;
			groundfloorVoxelLvl = i;
			continue;
		}
		break;
	}

	int lvl = groundfloorVoxelLvl * xRelRange_ * yRelRange_;
	int topLvL = (groundfloorVoxelLvl + 1) * xRelRange_ * yRelRange_ - 1;

	std::vector<int> exteriorLvlVoxelsIdx;

	for (size_t i = 0; i < exteriorVoxelsIdx_.size(); i++)
	{
		int currentVoxelIdx = exteriorVoxelsIdx_[i];

		if (currentVoxelIdx < lvl || currentVoxelIdx > topLvL) { continue; }

		exteriorLvlVoxelsIdx.emplace_back(currentVoxelIdx);
	}

	std::vector<Value> productLookupValues;
	std::vector<int> originVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;
	populateVoxelIndex(&voxelIndex, &originVoxels, &productLookupValues, exteriorLvlVoxelsIdx);
	productLookupValues = makeUniqueValueList(productLookupValues);

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
		cluster
	);

	// make a cutting plane 

	gp_Pnt p0(-1000, -1000, floorlvl);
	gp_Pnt p1(-1000, 1000, floorlvl);
	gp_Pnt p2(1000, 1000, floorlvl);
	gp_Pnt p3(1000, -1000, floorlvl);

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	TopoDS_Face cuttingFace = BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));

	// get all edges that meet the cutting plane
	std::vector<TopoDS_Edge> rawEdgeList;
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		LookupValue lookup = cluster->getHelper(0)->getLookup(productLookupValues[i].second);
		TopoDS_Shape currentShape;

		if (std::get<3>(lookup)) { currentShape = std::get<4>(lookup); }
		else { currentShape = cluster->getHelper(0)->getObjectShape(std::get<0>(lookup), true); }

		std::vector<SurfaceGroup*> surfaceGroupList;
		TopExp_Explorer expl;
		std::vector<TopoDS_Edge> untrimmedEdgeList;
		for (expl.Init(currentShape, TopAbs_FACE); expl.More(); expl.Next()) {

			// ignore if the z component of normal is 0 
			TopoDS_Face face = TopoDS::Face(expl.Current());

			GProp_GProps gprops;
			BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
			double area = gprops.Mass();

			if (area < 0.001) { continue; }

			gp_Vec faceNormal = computeFaceNormal(face);
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

	// prepare and clean the edges for ray cast
	std::vector<Edge*> uniqueEdges = getUniqueEdges(rawEdgeList);
	std::vector<Edge*> cleanedEdges = mergeOverlappingEdges(uniqueEdges, false);
	std::vector<Edge*> splitEdges = splitIntersectingEdges(cleanedEdges, false);

	//create index of the splitEdges
	bgi::rtree<Value, bgi::rstar<25>> edgeIndex;
	for (size_t i = 0; i < splitEdges.size(); i++)
	{
		Edge* currentEdge = splitEdges[i];
		bg::model::box <BoostPoint3D> bbox = createBBox(currentEdge->getStart(), currentEdge->getEnd());
		edgeIndex.insert(std::make_pair(bbox, i));
	}

	double distance = 2 * voxelSize_;

	// raycast
	std::vector<TopoDS_Edge> outerFootPrintList;
	for (size_t i = 0; i < splitEdges.size(); i++)
	{
		bool isExterior = false;

		Edge* currentEdge = splitEdges[i];

		gp_Pnt startPoint = currentEdge->getStart();
		gp_Pnt endPoint = currentEdge->getEnd();

		//printPoint(startPoint);
		//printPoint(endPoint);

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
			int voxelInt = originVoxels[qResult[j].second];
			voxel* currentBoxel = VoxelLookup_[voxelInt];

			auto boostCastPoint = currentBoxel->getCenterPoint(planeRotation_);
			gp_Pnt castPoint(boostCastPoint.get<0>(), boostCastPoint.get<1>(), floorlvl);
			TopoDS_Edge castRay = BRepBuilderAPI_MakeEdge(centerPoint, castPoint);

			//printPoint(centerPoint);
			//printPoint(castPoint);

			std::vector<Value> qResult2;
			qResult2.clear();
			edgeIndex.query(bgi::intersects(
				bg::model::box <BoostPoint3D>(
					createBBox(centerPoint, castPoint)
					)), std::back_inserter(qResult2));

			for (size_t k = 0; k < qResult2.size(); k++)
			{

				if (qResult2[k].second == i)
				{
					continue;
				}

				BRepExtrema_DistShapeShape distanceCalc(*splitEdges[qResult2[k].second]->getEdge(), castRay);
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
			outerFootPrintList.emplace_back(*currentEdge->getEdge());
		}
	}

	footprintList_ = outerEdges2Shapes(outerFootPrintList);
	hasFootprints_ = true;
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

			for (TopExp_Explorer explorer(currentRoof->getFace(), TopAbs_VERTEX); explorer.More(); explorer.Next())
			{
				const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer.Current());
				const gp_Pnt& point = BRep_Tool::Pnt(vertex);
				//printPoint(point);
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


TopoDS_Shape CJGeoCreator::simplefySolid(TopoDS_Shape solidShape, bool evalOverlap)
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
		gp_Vec faceNomal = computeFaceNormal(surfaceList[i]);

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

	std::vector<TopoDS_Face> cleanedFaceList;

	std::vector<int> evalList(surfaceList.size());
	for (size_t i = 0; i < surfaceList.size(); i++)
	{
		if (evalList[i] == 1) { continue; }
		evalList[i] = 1;

		std::vector<TopoDS_Face> mergeList;
		mergeList.emplace_back(surfaceList[i]);

		TopoDS_Face currentFace = surfaceList[i];
		gp_Dir currentdir = normalList[i];
		int count = 0;

		while (true)
		{
			for (size_t j = 0; j < surfaceList.size(); j++) //TODO: step
			{
				if (i == j) { continue; }
				if (evalList[j] == 1) { continue; }

				TopoDS_Face otherFace = surfaceList[j];
				gp_Dir otherdir = normalList[j];

				if (!currentdir.IsParallel(otherdir, 1e-6)) { continue; }

				bool touching = false;
				for (size_t k = 0; k < mergeList.size(); k++)
				{
					for (TopExp_Explorer edgeExp(mergeList[k], TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
						TopoDS_Edge edge1 = TopoDS::Edge(edgeExp.Current());
						gp_Pnt currentStartpoint = getFirstPointShape(edge1);
						gp_Pnt currentEndpoint = getLastPointShape(edge1);

						// loop through the edges of face2
						for (TopExp_Explorer edgeExp2(otherFace, TopAbs_EDGE); edgeExp2.More(); edgeExp2.Next()) {
							TopoDS_Edge edge2 = TopoDS::Edge(edgeExp2.Current());
							gp_Pnt otherStartpoint = getFirstPointShape(edge2);
							gp_Pnt otherEndpoint = getLastPointShape(edge2);

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
					if (touching) { break; }
				}

				if (touching) {
					evalList[j] = 1;
					mergeList.emplace_back(surfaceList[j]);
				}
			}

			if (mergeList.size() == 1)
			{
				cleanedFaceList.emplace_back(mergeList[0]);
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
						for (size_t i = 0; i < mergeList.size(); i++) { cleanedFaceList.emplace_back(mergeList[i]); }
					}
					cleanedFaceList.emplace_back(mergedFace);
					break;
				}
			}
		}
	}

	return cleanedFaceList;

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
		gp_Pnt currentStartpoint = getFirstPointShape(edgeList[i]);
		gp_Pnt currentEndpoint = getLastPointShape(edgeList[i]);

		if (currentStartpoint.IsEqual(currentEndpoint, 1e-6)) { continue; }

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (j == i) { continue; }
			if (evalList[j] == 1) { continue; }
			gp_Pnt otherStartpoint = getFirstPointShape(edgeList[j]);
			gp_Pnt otherEndpoint = getLastPointShape(edgeList[j]);

			if (currentStartpoint.IsEqual(otherEndpoint, 1e-6) && currentEndpoint.IsEqual(otherStartpoint, 1e-6) ||
				currentEndpoint.IsEqual(otherEndpoint, 1e-6) && currentStartpoint.IsEqual(otherStartpoint, 1e-6)) {
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
	//IntCurvesFace_Intersector intersection(face, 1e-6);
	//intersection.Perform(ray, RealLast, RealLast);

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

std::vector<TopoDS_Face>& CJGeoCreator::getGeoBase_()
{
	if (useRoofOutline_) { return roofOutlineList_; }

	return footprintList_;
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
						TopoDS_Shape shape;
						if (std::get<3>(lookup)) { shape = std::get<4>(lookup); }
						else { shape = cluster->getHelper(j)->getObjectShape(std::get<0>(lookup), true); }


						topProducts.emplace_back(product);
						topObjects.emplace_back(shape);
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
	if (!hasTopFaces_ || !hasGeoBase_) { initializeBasic(cluster); }
	if (!hasFootprints_) { makeFootprint(cluster); }

	std::vector< CJT::GeoObject*> geoObjectList;

	std::map<std::string, std::string> semanticRoofData;
	semanticRoofData.emplace("type", "RoofSurface");

	std::map<std::string, std::string> semanticFootData;
	semanticFootData.emplace("type", "GroundSurface");

	gp_Pnt urr = cluster->getUrrPoint();

	for (size_t i = 0; i < roofOutlineList_.size(); i++)
	{
		TopoDS_Shape movedShape = roofOutlineList_[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted());
		gp_Trsf trs;
		trs.SetTranslation(gp_Vec(0, 0, urr.Z()));

		CJT::GeoObject* geoObject = kernel->convertToJSON(movedShape.Moved(trs), "0.2");
		geoObject->appendSurfaceData(semanticRoofData);
		geoObject->appendSurfaceTypeValue(0);
		geoObjectList.emplace_back(geoObject);
	}

	for (size_t i = 0; i < footprintList_.size(); i++)
	{
		CJT::GeoObject* geoObject = kernel->convertToJSON(footprintList_[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "0.2");
		geoObject->appendSurfaceData(semanticFootData);
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
	if (!hasTopFaces_ || !hasGeoBase_) { initializeBasic(cluster); }

	std::vector< CJT::GeoObject*> geoObjectList;
	if (roofOutlineList_.size() == 0)
	{
		return geoObjectList;
	}

	double height = cluster->getUrrPoint().Z();
	
	for (size_t i = 0; i < roofOutlineList_.size(); i++)
	{
		TopoDS_Face currentFootprint = roofOutlineList_[i];

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
	if (roofOutlineList_.size() == 0)
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
	if (roofOutlineList_.size() == 0)
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
	std::cout << "- Computing LoD 3.2 Model" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();

	std::vector< CJT::GeoObject*> geoObjectList; // final output collection

	double buffer = 1 * voxelSize_; // set the distance from the bb of the evaluated object
	int maxCastAttempts = 100; // set the maximal amout of cast attempts before the surface is considered interior

	std::cout << std::endl;
	if (exteriorVoxelsIdx_.size() == 0)
	{
		std::cout << "No exterior space has been found" << std::endl;
		return geoObjectList;
	}

	std::cout << "\tExterior space succesfully grown" << std::endl << std::endl;

	std::vector<Value> productLookupValues;
	std::vector<int> originVoxels; // voxels from which ray cast processing can be executed, 100% sure exterior voxels
	bgi::rtree<Value, bgi::rstar<25>> voxelIndex;

	populateVoxelIndex(&voxelIndex, &originVoxels, &productLookupValues, exteriorVoxelsIdx_);
	productLookupValues = makeUniqueValueList(productLookupValues);

	double gridDistance = 1.5;

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
		cluster
	);

	std::vector<TopoDS_Face> rawFaces;

	//ofstream myfile;
	//myfile.open("C:/Users/Jasper/Desktop/test.txt");

	// evaluate which surfaces are visible
	for (size_t i = 0; i < productLookupValues.size(); i++)
	{
		LookupValue lookup = cluster->getHelper(0)->getLookup(productLookupValues[i].second);
		std::cout << std::get<0>(lookup)->data().toString() << std::endl;

		TopoDS_Shape currentShape;

		if (std::get<3>(lookup)) { currentShape = std::get<4>(lookup); }
		else { currentShape = cluster->getHelper(0)->getObjectShape(std::get<0>(lookup), true); }

		for (TopExp_Explorer explorer(currentShape, TopAbs_FACE); explorer.More(); explorer.Next())
		{

			const TopoDS_Face& currentFace = TopoDS::Face(explorer.Current());
			std::cout << currentFace.IsNull() << std::endl;
			std::cout << "in" << std::endl;
			if (isWireVisible(
				cluster,
				currentShape,
				currentFace,
				voxelIndex,
				originVoxels,
				exteriorProductIndex,
				gridDistance,
				buffer)
				)
			{
				std::cout << "out" << std::endl;
				rawFaces.emplace_back(currentFace);
				continue;
			}

			if (isSurfaceVisible(
				cluster, 
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

	//myfile.close();

	for (size_t i = 0; i < rawFaces.size(); i++)
	{
		CJT::GeoObject* geoObject = kernel->convertToJSON(rawFaces[i].Moved(cluster->getHelper(0)->getObjectTranslation().Inverted()), "3.2");
		geoObjectList.emplace_back(geoObject);
	}



	printTime(startTime, std::chrono::high_resolution_clock::now());
	return geoObjectList;
}

void CJGeoCreator::populateVoxelIndex(
	bgi::rtree<Value, bgi::rstar<25>>* voxelIndex, 
	std::vector<int>* originVoxels, 
	std::vector<Value>* productLookupValues, 
	const std::vector<int>& exteriorVoxels
)
{
	for (size_t j = 0; j < exteriorVoxels.size(); j++)
	{
		voxel* currentBoxel = VoxelLookup_[exteriorVoxels[j]];
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
			originVoxels->emplace_back(exteriorVoxels[j]);
		}
	}
}

void CJGeoCreator::filterEncapsulatedObjects(std::vector<Value>* productLookupValues, bgi::rtree<Value, bgi::rstar<25>>* exteriorProductIndex, helperCluster* cluster)
{
	bgi::rtree<Value, bgi::rstar<25>> cleandExteriorProductIndex;
	std::vector<Value> cleanedProductLookupValues;
	for (size_t i = 0; i < productLookupValues->size(); i++)
	{
		bool isExposed = true;
		LookupValue lookup = cluster->getHelper(0)->getLookup(productLookupValues->at(i).second);
		TopoDS_Shape currentShape;

		if (std::get<3>(lookup)) { currentShape = std::get<4>(lookup); }
		else { currentShape = cluster->getHelper(0)->getObjectShape(std::get<0>(lookup), true); }

		bg::model::box <BoostPoint3D> box = productLookupValues->at(i).first;

		std::vector<Value> qResult;
		qResult.clear();
		exteriorProductIndex->query(bgi::intersects(box), std::back_inserter(qResult));

		for (size_t k = 0; k < qResult.size(); k++)
		{
			bool encapsulating = true;

			LookupValue otherLookup = cluster->getHelper(0)->getLookup(qResult[k].second);

			TopoDS_Shape otherShape;
			if (std::get<3>(otherLookup)) { otherShape = std::get<4>(otherLookup); }
			else { otherShape = cluster->getHelper(0)->getObjectShape(std::get<0>(otherLookup), true); }

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
	helperCluster* cluster,
	const TopoDS_Shape& currentShape, 
	const TopoDS_Face& currentFace, 
	const bgi::rtree<Value, bgi::rstar<25>>& voxelIndex, 
	const std::vector<int>& originVoxels,
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
			printPoint(point);
			if (pointIsVisible(cluster, currentShape, currentFace, voxelIndex, originVoxels, exteriorProductIndex, point, buffer))
			{
				return true;
			}
		}
	}
	return false;
}

bool CJGeoCreator::isWireVisible(
	helperCluster* cluster, 
	const TopoDS_Shape& currentShape, 
	const TopoDS_Face& currentFace, 
	const bgi::rtree<Value, 
	bgi::rstar<25>>& voxelIndex, 
	const std::vector<int>& originVoxels, 
	const bgi::rtree<Value, bgi::rstar<25>>& exteriorProductIndex,
	double gridDistance, 
	double buffer)
{
	std::cout << "x" << std::endl;
	std::cout << BRepTools::OuterWire(currentFace).IsNull() << std::endl;
	std::cout << "l1" << std::endl;
	// create points on offset outer wire
	BRepOffsetAPI_MakeOffset offsetter(BRepTools::OuterWire(currentFace), GeomAbs_Arc);
	std::cout << "l2" << std::endl;
	offsetter.Perform(-0.02);

	std::cout << offsetter.IsDone() << std::endl;

	TopExp_Explorer expl;
	TopoDS_Wire correctedOuterWire;

	std::cout << "a" << std::endl;
	for (expl.Init(offsetter.Shape(), TopAbs_WIRE); expl.More(); expl.Next())
	{
		correctedOuterWire = TopoDS::Wire(expl.Current());
		break;
	}
	std::cout << "b" << std::endl;

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
			printPoint(point);
			if (pointIsVisible(cluster, currentShape, currentFace, voxelIndex, originVoxels, exteriorProductIndex, point, buffer))
			{
				return true;
			}
		}
	}
	return false;
}

bool CJGeoCreator::pointIsVisible(helperCluster* cluster,
	const TopoDS_Shape& currentShape,
	const TopoDS_Face& currentFace,
	const bgi::rtree<Value,bgi::rstar<25>>& voxelIndex,
	const std::vector<int>& originVoxels,
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

		int voxelInt = originVoxels[qResult[j].second];
		voxel* currentBoxel = VoxelLookup_[voxelInt];
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
			LookupValue otherLookup = cluster->getHelper(0)->getLookup(qResult2[k].second);

			TopoDS_Shape otherShape;
			if (std::get<3>(otherLookup)) { otherShape = std::get<4>(otherLookup); }
			else { otherShape = cluster->getHelper(0)->getObjectShape(std::get<0>(otherLookup), true); }

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

CJGeoCreator::CJGeoCreator(helperCluster* cluster, double vSize, bool mkFootprint)
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

	std::cout << "- Populate Grid" << std::endl;
	for (int i = 0; i < totalVoxels_; i++) {

		if (i % 1000 == 0)
		{
			std::cout.flush();
			std::cout << "\t" << i << " of " << totalVoxels_ << "\r";
		}

		addVoxel(i, cluster);
	}
	std::cout << "\t" << totalVoxels_ << " of " << totalVoxels_ << std::endl;
	std::cout << "- Exterior space growing" << std::endl;
	for (int i = 0; i < totalVoxels_; i++)
	{
		if (!VoxelLookup_[i]->getIsIntersecting())
		{
			exteriorVoxelsIdx_ = growExterior(i, 0, cluster);
			break;
		}
	}
	std::cout << std::endl;
	std::cout << "\tExterior space succesfully grown" << std::endl << std::endl;

	initializeBasic(cluster);

	if (mkFootprint) { makeFootprint(cluster); }

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
							currentBoxel->addInternalProduct(qResult[k]);
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


