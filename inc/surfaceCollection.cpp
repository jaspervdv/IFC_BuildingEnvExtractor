#include "surfaceCollection.h"
#include "helper.h"
#include "settingsCollection.h"

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
	theFaceCollection_.emplace_back(aFace);
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
	
	theProjectedFace_ = helperFunctions::projectFaceFlat(theFaceCollection_[0], 0);

	theFlatFace_ = helperFunctions::projectFaceFlat(
		theFaceCollection_[0],
		helperFunctions::getHighestPoint(theFaceCollection_[0]).Z() //TODO: make this smarter, might not work with non-flat surfaces
	);
}


void SurfaceGroup::populateGrid(double distance) 
{
	TopoDS_Face theFace = theFaceCollection_[0];
	pointGrid_.clear();

	double xRange = urrPoint_.X() - lllPoint_.X();
	double yRange = urrPoint_.Y() - lllPoint_.Y();

	double xDistance = distance;
	double yDistance = distance;

	int xSteps = ceil(xRange / xDistance);
	int ySteps = ceil(yRange / yDistance);

	xDistance = xRange / xSteps;
	yDistance = yRange / ySteps;

	int vertCount = helperFunctions::getPointCount(theFace);

	if (vertCount == 6 && helperFunctions::computeArea(theFace) < 1) // If Triangle
	{
		double x = 0;
		double y = 0;
		double z = 0;

		for (TopExp_Explorer expl(theFace, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);

			x += point.X();
			y += point.Y();
			z += point.Z();
		}

		gp_Pnt centerPoint = gp_Pnt(
			x / 6,
			y / 6,
			z / 6
		);

		pointGrid_.emplace_back(std::make_shared<EvaluationPoint>(centerPoint));

		double largestAngle = helperFunctions::computeLargestAngle(theFace);

		//TODO: finetune
		if (largestAngle > 2.22) //140 degrees
		{
			std::vector<gp_Pnt> uniquePointList = helperFunctions::getUniquePoints(theFace);

			for (size_t i = 0; i < uniquePointList.size(); i++)
			{
				gp_Pnt legPoint = uniquePointList[i];

				if (xSteps == 1) { xSteps = 2; }

				gp_Vec translationVec = gp_Vec(
					(legPoint.X() - centerPoint.X()) / xSteps,
					(legPoint.Y() - centerPoint.Y()) / xSteps,
					(legPoint.Z() - centerPoint.Z()) / xSteps
				);

				for (size_t j = 0; j < xSteps; j++)
				{
					pointGrid_.emplace_back(std::make_shared<EvaluationPoint>(centerPoint.Translated(translationVec * j))
					);
				}

			}
			if (pointGrid_.size() <= 5) { populateGrid(distance / 2); } //TODO: make smarter to avoid points only on wires
			return;
		}
	}

	// if not triangle
	IntCurvesFace_Intersector intersector(theFace, 0.0001);
	for (size_t i = 0; i <= xSteps; i++)
	{
		for (size_t j = 0; j <= ySteps; j++)
		{
			intersector.Perform(
				gp_Lin(
					gp_Pnt(lllPoint_.X() + xDistance * i, lllPoint_.Y() + yDistance * j, -1000),
					gp_Dir(0, 0, 1000)),
				-INFINITY,
				+INFINITY);

			if (intersector.NbPnt() == 1) {
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(intersector.Pnt(1));
				pointGrid_.emplace_back(evalPoint);
			}
		}
	}
	if (pointGrid_.size() <= 5) { populateGrid(distance / 2); } //TODO: make smarter to avoid points only on wires
	return;
}


bool SurfaceGroup::testIsVisable(const std::vector<SurfaceGroup>& otherSurfaces, bool preFilter)
{
	TopoDS_Face currentFace = getFaces()[0];
	std::vector<std::shared_ptr<EvaluationPoint>> currentGrid = getPointGrid();

	double precision = SettingsCollection::getInstance().precision();

	for (size_t i = 0; i < otherSurfaces.size(); i++)
	{

		SurfaceGroup otherGroup = otherSurfaces[i];

		if (preFilter) //TODO: spacial q
		{
			if (!overlap(otherGroup)) { continue; }
		}

		TopoDS_Face otherFace = otherGroup.getFaces()[0];
		gp_Pnt otherLLLPoint = otherGroup.getLLLPoint();
		gp_Pnt otherURRPoint = otherGroup.getURRPoint();

		if (currentFace.IsEqual(otherFace)) { continue; }

		IntCurvesFace_Intersector intersector(otherFace, precision);

		for (size_t k = 0; k < currentGrid.size(); k++)
		{
			std::shared_ptr<EvaluationPoint> currentEvalPoint = currentGrid[k];

			if (!currentEvalPoint->isVisible()) { continue; }

			// check if the projection line falls withing the bbox of the surface
			gp_Pnt evalPoint = currentEvalPoint->getPoint();
			if (evalPoint.X() - otherLLLPoint.X() < -0.1 || evalPoint.X() - otherURRPoint.X() > 0.1) { continue; }
			if (evalPoint.Y() - otherLLLPoint.Y() < -0.1 || evalPoint.Y() - otherURRPoint.Y() > 0.1) { continue; }
			if (evalPoint.Z() - urrPoint_.Z() > 0.1) { continue; }

			intersector.Perform( //TODO: find a smarter way to proccess
				currentEvalPoint->getEvalLine(),
				-0,
				+INFINITY);

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

