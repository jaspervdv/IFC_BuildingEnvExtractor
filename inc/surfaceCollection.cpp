#include "surfaceCollection.h"
#include "helper.h"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>


EvaluationPoint::EvaluationPoint(const gp_Pnt& p)
{
	thePoint_ = p;
	evalEdge_ = BRepBuilderAPI_MakeEdge(
		p,
		gp_Pnt(p.X(), p.Y(), p.Z() + 1000)
	);
	evalLin_ = gp_Lin(p, gp_Dir(0, 0, 1000));
}


SurfaceGroup::SurfaceGroup(const TopoDS_Face& aFace)
{
	theFace_ = aFace;
	lllPoint_ = gp_Pnt(999999999, 999999999, 99999999);
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

	avHeight_ = helperFunctions::getAvFaceHeight(aFace);
	topHeight_ = helperFunctions::getTopFaceHeight(aFace);
}


bool SurfaceGroup::overlap(SurfaceGroup other) {

	bool inXD = false;
	bool inYD = false;

	if (llPoint_.X() <= other.llPoint_.X() && urPoint_.X() >= other.lllPoint_.X()) { inXD = true; }
	if (other.llPoint_.X() <= llPoint_.X() && other.urrPoint_.X() > llPoint_.X()) { inXD = true; }
	if (llPoint_.Y() <= other.llPoint_.Y() && urPoint_.Y() >= other.lllPoint_.Y()) { inYD = true; }
	if (other.llPoint_.Y() <= llPoint_.Y() && other.urrPoint_.Y() >= llPoint_.Y()) { inYD = true; }

	if (inXD && inYD)
	{
		return true;
	}
	return false;
}


void SurfaceGroup::projectFace() {
	
	theProjectedFace_ = helperFunctions::projectFaceFlat(theFace_, 0);

	theFlatFace_ = helperFunctions::projectFaceFlat(
		theFace_, 
		helperFunctions::getHighestPoint(theFace_).Z() //TODO: make this smarter, might not work with non-flat surfaces
	);
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


bool SurfaceGroup::testIsVisable(const std::vector<SurfaceGroup>& otherSurfaces, bool preFilter)
{
	TopoDS_Face currentFace = getFace();
	std::vector<EvaluationPoint*> currentGrid = getPointGrid();

	for (size_t i = 0; i < otherSurfaces.size(); i++)
	{

		SurfaceGroup otherGroup = otherSurfaces[i];

		if (preFilter)
		{
			if (!overlap(otherGroup)) { continue; }
		}

		TopoDS_Face otherFace = otherGroup.getFace();
		gp_Pnt otherLLLPoint = otherGroup.getLLLPoint();
		gp_Pnt otherURRPoint = otherGroup.getURRPoint();

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
			if (evalPoint.Z() - urrPoint_.Z() > 0.1) { continue; }

			intersector.Perform( //TODO: find a smarter way to proccess
				currentEvalPoint->getEvalLine(),
				-0,
				+INFINITE);

			if (intersector.NbPnt() > 0)
			{
				currentEvalPoint->setInvisible();
				continue;
			}

			for (TopExp_Explorer expl(otherFace, TopAbs_EDGE); expl.More(); expl.Next())
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


Edge::Edge(const TopoDS_Edge& edge)
{
	theEdge_ = edge;

	TopExp_Explorer vertexExplorer(edge, TopAbs_VERTEX);

	// Step 2: Get the start and end vertices of the edge.
	gp_Pnt startVertex, endVertex;
	if (vertexExplorer.More()) {
		startPoint_ = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
		vertexExplorer.Next();
	}
	if (vertexExplorer.More()) {
		endPoint_ = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
	}
}

gp_Pnt Edge::getStart(bool projected) const
{
	if (projected)
	{
		return gp_Pnt(startPoint_.X(), startPoint_.Y(), 0);
	}
	return startPoint_;
}

gp_Pnt Edge::getEnd(bool projected) const
{ 
	if (projected)
	{
		return gp_Pnt(endPoint_.X(), endPoint_.Y(), 0);
	}
	return endPoint_;
}

