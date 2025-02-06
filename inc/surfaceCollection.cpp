#include "surfaceCollection.h"
#include "helper.h"
#include "settingsCollection.h"
#include "DebugUtils.h"

#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GCPnts_AbscissaPoint.hxx>

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

bool SurfaceGridPair::overlap(const SurfaceGridPair& other)
{
	bool inXD = false;
	bool inYD = false;

	if (lllPoint_.X() <= other.lllPoint_.X() && urrPoint_.X() >= other.lllPoint_.X()) { inXD = true; }
	if (other.lllPoint_.X() <= lllPoint_.X() && other.urrPoint_.X() > lllPoint_.X()) { inXD = true; }
	if (lllPoint_.Y() <= other.lllPoint_.Y() && urrPoint_.Y() >= other.lllPoint_.Y()) { inYD = true; }
	if (other.lllPoint_.Y() <= lllPoint_.Y() && other.urrPoint_.Y() >= lllPoint_.Y()) { inYD = true; }

	if (inXD && inYD)
	{
		return true;
	}
	return false;
}

SurfaceGridPair::SurfaceGridPair(const TopoDS_Face& theFace)
{
	theFace_ = theFace;
	lllPoint_ = gp_Pnt(999999999, 999999999, 99999999);
	urrPoint_ = gp_Pnt(-999999999, -999999999, -99999999);

	int vertCount = 0;
	TopExp_Explorer expl;
	for (expl.Init(theFace, TopAbs_VERTEX); expl.More(); expl.Next())
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

	avHeight_ = helperFunctions::getAverageZ(theFace);
	topHeight_ = helperFunctions::getHighestZ(theFace);
}

void SurfaceGridPair::populateGrid(double distance)
{
	pointGrid_.clear();

	double xRange = urrPoint_.X() - lllPoint_.X();
	double yRange = urrPoint_.Y() - lllPoint_.Y();

	double xDistance = distance;
	double yDistance = distance;

	int xSteps = ceil(xRange / xDistance);
	int ySteps = ceil(yRange / yDistance);

	xDistance = xRange / xSteps;
	yDistance = yRange / ySteps;

	int vertCount = vertCount_;

	if (vertCount == 6 && helperFunctions::computeArea(theFace_) < 1) // If Triangle
	{
		double x = 0;
		double y = 0;
		double z = 0;

		for (TopExp_Explorer expl(theFace_, TopAbs_VERTEX); expl.More(); expl.Next())
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

		double largestAngle = helperFunctions::computeLargestAngle(theFace_);

		//TODO: finetune
		if (largestAngle > 2.22) //140 degrees
		{
			std::vector<gp_Pnt> uniquePointList = helperFunctions::getUniquePoints(theFace_);

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
	IntCurvesFace_Intersector intersector(theFace_, 0.0001);

	// get points on wire (do not use offsetter due to instability issues)
	for (TopExp_Explorer expl(theFace_, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		BRepAdaptor_Curve curve(currentEdge);
		Standard_Real firstParam = curve.FirstParameter();
		Standard_Real lastParam = curve.LastParameter();
		Standard_Real edgeLength = GCPnts_AbscissaPoint::Length(curve, firstParam, lastParam);
		int stepCount = static_cast<int>(ceil(helperFunctions::getFirstPointShape(currentEdge).Distance(helperFunctions::getLastPointShape(currentEdge))/distance));
		double localDistance = edgeLength / stepCount;

		if (stepCount < 2)
		{
			stepCount = 2;
		}

		GCPnts_UniformAbscissa uniformAbscissa(curve, stepCount);

		if (!uniformAbscissa.IsDone()) { continue; }

		for (int i = 1; i <= uniformAbscissa.NbPoints(); ++i) {
			Standard_Real param = uniformAbscissa.Parameter(i);

			gp_Pnt point;
			gp_Vec tangent;
			curve.D1(param, point, tangent);

			tangent.Normalize();
			tangent = tangent * 0.005;

			gp_Vec perp1Vec(tangent.Y(), -tangent.X(), 0);
			gp_Vec perp2Vec(-tangent.Y(), tangent.X(), 0);

			gp_Pnt p1 = point.Translated(perp1Vec);
			gp_Pnt p2 = point.Translated(perp2Vec);
			p1.SetZ(-1000);
			p2.SetZ(-1000);

			intersector.Perform(
				gp_Lin(
					p1,
					gp_Dir(0, 0, 10000)),
				-INFINITY,
				+INFINITY);

			if (intersector.NbPnt() == 1) {
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(intersectionPoint);
				pointGrid_.emplace_back(evalPoint);
				continue;
			}

			intersector.Perform(
				gp_Lin(
					p2,
					gp_Dir(0, 0, 10000)),
				-INFINITY,
				+INFINITY);

			if (intersector.NbPnt() == 1) {
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(intersectionPoint);
				pointGrid_.emplace_back(evalPoint);
				continue;
			}
		}
	}

	// get points on face
	for (size_t i = 0; i <= xSteps; i++)
	{
		for (size_t j = 0; j <= ySteps; j++)
		{
			intersector.Perform(
				gp_Lin(
					gp_Pnt(lllPoint_.X() + xDistance * i, lllPoint_.Y() + yDistance * j, -1000),
					gp_Dir(0, 0, 10000)),
				-INFINITY,
				+INFINITY);

			if (intersector.NbPnt() == 1) {
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(intersectionPoint);
				pointGrid_.emplace_back(evalPoint);
			}
		}
	}
	if (pointGrid_.size() <= 5) { populateGrid(distance / 2); } //TODO: make smarter to avoid points only on wires
	return;
}

bool SurfaceGridPair::testIsVisable(const std::vector<std::shared_ptr<SurfaceGridPair>>& otherSurfaces, bool preFilter)
{
	double precision = SettingsCollection::getInstance().precision();
	
	if (!pointGrid_.size()) { populateGrid(SettingsCollection::getInstance().surfaceGridSize()); }

	for (const std::shared_ptr<SurfaceGridPair>& otherGroup : otherSurfaces)
	{
		if (preFilter) //TODO: spacial q
		{
			if (!overlap(*otherGroup)) { continue; }
		}

		TopoDS_Face otherFace = otherGroup->getFace();
		gp_Pnt otherLLLPoint = otherGroup->getLLLPoint();
		gp_Pnt otherURRPoint = otherGroup->getURRPoint();

		if (theFace_.IsEqual(otherFace)) { continue; }
		IntCurvesFace_Intersector intersector(otherFace, precision);

		for (const std::shared_ptr<EvaluationPoint>& currentEvalPoint: pointGrid_)
		{
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

	for (const std::shared_ptr<EvaluationPoint>& currentPoint : pointGrid_)
	{
		if (currentPoint->isVisible())
		{
			return true;
		}
	}
	visibility_ = false;
	return false;
}



void SurfaceGridPair::Merge(const std::vector<SurfaceGridPair>& otherPairList, const TopoDS_Face& theCompleteFace)
{
	lllPoint_ = gp_Pnt(999999999, 999999999, 99999999);
	urrPoint_ = gp_Pnt(-999999999, -999999999, -99999999);

	int vertCount = 0;
	TopExp_Explorer expl;
	for (expl.Init(theCompleteFace, TopAbs_VERTEX); expl.More(); expl.Next())
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

	avHeight_ = helperFunctions::getAverageZ(theCompleteFace);
	topHeight_ = helperFunctions::getHighestZ(theCompleteFace);

	theFace_ = theCompleteFace;

	for (const SurfaceGridPair& currentPair : otherPairList)
	{
		for (auto point : currentPair.pointGrid_)
		{
			pointGrid_.emplace_back(point);
		}
	}
}

RCollection::RCollection(const std::vector<TopoDS_Face>& theFaceColletion)
{
	theFaceCollection_ = theFaceColletion;
	helperFunctions::bBoxDiagonal(theFaceCollection_, &lllPoint_, &urrPoint_, 0, 0, 0);

	if (theFaceColletion.size() == 1)
	{
		theFlatFace_ = helperFunctions::projectFaceFlat(
			theFaceColletion[0],
			urrPoint_.Z()
		);
		return;
	}

	std::vector<TopoDS_Face> projectedFaces;

	for (const TopoDS_Face& currentFace : theFaceColletion)
	{
		projectedFaces.emplace_back(helperFunctions::projectFaceFlat(
			currentFace,
			urrPoint_.Z()
		));
	}

	std::vector<TopoDS_Face> flatFaceList = helperFunctions::planarFaces2Outline(projectedFaces);

	if (flatFaceList.size() < 1)
	{
		return;
	}
	theFlatFace_ = helperFunctions::planarFaces2Outline(flatFaceList)[0];
	return;
}


const TopoDS_Face RCollection::getProjectedFace() const
{
	return helperFunctions::projectFaceFlat(
		theFlatFace_,
		0
	);
}
