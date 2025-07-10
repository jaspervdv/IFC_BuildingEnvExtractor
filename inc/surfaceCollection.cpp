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
#include <Geom_TrimmedCurve.hxx>

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
	if (distance < 0.05) { return; }

	pointGrid_.clear();

	// get points on wire

	std::vector<gp_Pnt> wireGridList = helperFunctions::getPointGridOnWire(theFace_, distance);
	pointGrid_.reserve(wireGridList.size());
	for (const gp_Pnt& wirePoint : wireGridList)
	{
		std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(wirePoint);
		pointGrid_.emplace_back(evalPoint);
	}

	std::vector<gp_Pnt> faceGridList = helperFunctions::getPointGridOnSurface(theFace_, distance);
	for (const gp_Pnt& facePoint : faceGridList)
	{
		std::shared_ptr<EvaluationPoint> evalPoint = std::make_shared<EvaluationPoint>(facePoint);
		pointGrid_.emplace_back(evalPoint);
	}

	if (pointGrid_.size() <= 5) { populateGrid(distance / 2); } //TODO: refine this
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
		for (const std::shared_ptr<EvaluationPoint>& currentEvalPoint: pointGrid_)
		{
			if (!currentEvalPoint->isVisible()) { continue; }

			// check if the projection line falls withing the bbox of the surface
			gp_Pnt evalPoint = currentEvalPoint->getPoint();
			if (evalPoint.X() - otherLLLPoint.X() < -0.1 || evalPoint.X() - otherURRPoint.X() > 0.1) { continue; }
			if (evalPoint.Y() - otherLLLPoint.Y() < -0.1 || evalPoint.Y() - otherURRPoint.Y() > 0.1) { continue; }
			if (evalPoint.Z() - urrPoint_.Z() > 0.1) { continue; }

			gp_Pnt offsetPoint = currentEvalPoint->getPoint();
			offsetPoint.SetZ(offsetPoint.Z() + 1000);
			
			if (helperFunctions::LineShapeIntersection(otherFace, currentEvalPoint->getPoint(), offsetPoint))
			{
				currentEvalPoint->setInvisible();
				continue;
			}

			gp_Pnt projectedPoint = currentEvalPoint->getPoint();
			projectedPoint.SetZ(0);
			for (TopExp_Explorer expl(otherFace, TopAbs_EDGE); expl.More(); expl.Next())
			{
				TopoDS_Edge otherEdge = TopoDS::Edge(expl.Current());
				gp_Pnt p0 = helperFunctions::getFirstPointShape(otherEdge);
				gp_Pnt p1 = helperFunctions::getLastPointShape(otherEdge);
				p0.SetZ(0);
				p1.SetZ(0);

				if (p0.Distance(p1) < 1e-6) { continue; }
				if (helperFunctions::pointOnEdge(BRepBuilderAPI_MakeEdge(p0, p1), projectedPoint))
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
		TopoDS_Face flattenedFace = helperFunctions::projectFaceFlat(theFaceColletion[0], urrPoint_.Z() );
		theFlatFace_ = flattenedFace;
		return;
	}

	std::vector<TopoDS_Face> projectedFaces;

	for (const TopoDS_Face& currentFace : theFaceColletion)
	{
		TopoDS_Face flattenedFace = helperFunctions::projectFaceFlat(currentFace, urrPoint_.Z() );
		if (flattenedFace.IsNull()) {continue; }
		projectedFaces.emplace_back(flattenedFace);
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
