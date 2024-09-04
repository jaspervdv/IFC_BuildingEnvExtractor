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


ROSCollection::ROSCollection(std::vector< SurfaceGridPair> theGridPairList)
{
	theFaceCollection_ = theGridPairList;

	lllPoint_ = gp_Pnt(999999999, 999999999, 99999999);
	urrPoint_ = gp_Pnt(-999999999, -999999999, -99999999);

	for (SurfaceGridPair surfaceGridPair : theFaceCollection_)
	{
		TopoDS_Face currentFace = surfaceGridPair.getFace();
		for (TopExp_Explorer expl(currentFace, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt point = BRep_Tool::Pnt(vertex);

			if (point.X() > urrPoint_.X()) { urrPoint_.SetX(point.X()); }
			if (point.Y() > urrPoint_.Y()) { urrPoint_.SetY(point.Y()); }
			if (point.Z() > urrPoint_.Z()) { urrPoint_.SetZ(point.Z()); }
			if (point.X() < lllPoint_.X()) { lllPoint_.SetX(point.X()); }
			if (point.Y() < lllPoint_.Y()) { lllPoint_.SetY(point.Y()); }
			if (point.Z() < lllPoint_.Z()) { lllPoint_.SetZ(point.Z()); }
		}
	}
}

const std::vector<TopoDS_Face> ROSCollection::getFaces() const
{
	std::vector<TopoDS_Face> faceList;
	for (const SurfaceGridPair& currentPair : theFaceCollection_)
	{
		faceList.emplace_back(currentPair.getFace());
	}
	return faceList;
}

bool ROSCollection::overlap(ROSCollection other) {

	bool inXD = false;
	bool inYD = false;

	if (lllPoint_.X() <= other.lllPoint_.X() && urrPoint_.X() >= other.urrPoint_.X()) { inXD = true; }
	if (other.lllPoint_.X() <= lllPoint_.X() && other.urrPoint_.X() > urrPoint_.X()) { inXD = true; }
	if (lllPoint_.Y() <= other.lllPoint_.Y() && urrPoint_.Y() >= other.lllPoint_.Y()) { inYD = true; }
	if (other.lllPoint_.Y() <= lllPoint_.Y() && other.urrPoint_.Y() >= lllPoint_.Y()) { inYD = true; }

	if (inXD && inYD)
	{
		return true;
	}
	return false;
}

bool ROSCollection::testIsVisable(const std::vector<ROSCollection>& otherSurfaces, bool preFilter)
{
	for (SurfaceGridPair currentGridPair : theFaceCollection_)
	{
		bool isVisible = true;
		for (ROSCollection otherSurfaceGroup : otherSurfaces)
		{
			if (!currentGridPair.testIsVisable(otherSurfaceGroup.getFaceGridPairs()))
			{
				isVisible = false;
				break;
			}
		}
		if (isVisible) { return true; }
	}
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

bool SurfaceGridPair::overlap(SurfaceGridPair other)
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

	avHeight_ = helperFunctions::getAvFaceHeight(theFace);
	topHeight_ = helperFunctions::getTopFaceHeight(theFace);
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
					helperFunctions::printPoint(centerPoint.Translated(translationVec * j));
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

bool SurfaceGridPair::testIsVisable(const std::vector<SurfaceGridPair>& otherSurfaces, bool preFilter)
{
	double precision = SettingsCollection::getInstance().precision();

	for (size_t i = 0; i < otherSurfaces.size(); i++)
	{

		SurfaceGridPair otherGroup = otherSurfaces[i];

		if (preFilter) //TODO: spacial q
		{
			if (!overlap(otherGroup)) { continue; }
		}

		TopoDS_Face otherFace = otherGroup.getFace();
		gp_Pnt otherLLLPoint = otherGroup.getLLLPoint();
		gp_Pnt otherURRPoint = otherGroup.getURRPoint();

		if (theFace_.IsEqual(otherFace)) { continue; }

		IntCurvesFace_Intersector intersector(otherFace, precision);

		for (size_t k = 0; k < pointGrid_.size(); k++)
		{
			std::shared_ptr<EvaluationPoint> currentEvalPoint = pointGrid_[k];

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

	for (size_t i = 0; i < pointGrid_.size(); i++)
	{
		if (pointGrid_[i]->isVisible())
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

	avHeight_ = helperFunctions::getAvFaceHeight(theCompleteFace);
	topHeight_ = helperFunctions::getTopFaceHeight(theCompleteFace);

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
	helperFunctions::getBBoxPoints(theFaceCollection_, &lllPoint_, &urrPoint_);

	if (theFaceColletion.size() == 1)
	{
		TopoDS_Face flatFace = helperFunctions::projectFaceFlat(theFaceColletion[0], 0);

		theFlatFace_ = helperFunctions::projectFaceFlat(
			flatFace,
			urrPoint_.Z()
		);

		for (TopExp_Explorer WireExpl(theFaceColletion[0], TopAbs_WIRE); WireExpl.More(); WireExpl.Next())
		{
			TopoDS_Wire currentWire = TopoDS::Wire(WireExpl.Current());
			theWireCollection_.emplace_back(currentWire);
		}
		return;
	}

	std::vector<TopoDS_Edge> edgeList;
	for (TopoDS_Face currentFace : theFaceColletion)
	{
		for (TopExp_Explorer expl(currentFace, TopAbs_EDGE); expl.More(); expl.Next())
		{
			TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
			edgeList.emplace_back(currentEdge);
		}
	}

	std::vector<TopoDS_Edge> outerEdgeList;
	std::vector<TopoDS_Edge> outerEdgeListFlat;
	for (size_t i = 0; i < edgeList.size(); i++)
	{
		TopoDS_Edge currentEdge = edgeList[i];
		bool isUnique = true;

		gp_Pnt currentP1 = helperFunctions::getFirstPointShape(currentEdge);
		gp_Pnt currentP2 = helperFunctions::getLastPointShape(currentEdge);

		for (size_t j = 0; j < edgeList.size(); j++)
		{
			if (i == j) { continue; }

			TopoDS_Edge otherEdge = edgeList[j];

			gp_Pnt otherP1 = helperFunctions::getFirstPointShape(otherEdge);
			gp_Pnt otherP2 = helperFunctions::getLastPointShape(otherEdge);

			if (currentP1.IsEqual(otherP2, 1e-6) && currentP2.IsEqual(otherP1, 1e-6) ||
				currentP1.IsEqual(otherP1, 1e-6) && currentP2.IsEqual(otherP2, 1e-6))
			{
				isUnique = false;
				break;
			}
		}

		if (!isUnique)
		{
			continue;
		}

		gp_Pnt currentP1Flat(currentP1.X(), currentP1.Y(), 0);
		gp_Pnt currentP2Flat(currentP2.X(), currentP2.Y(), 0);
		outerEdgeList.emplace_back(currentEdge);
		outerEdgeListFlat.emplace_back(BRepBuilderAPI_MakeEdge(currentP1Flat, currentP2Flat));
	}

	//grow wires
	std::vector<TopoDS_Wire> wireList = helperFunctions::growWires(outerEdgeList);
	std::vector<TopoDS_Wire> cleanWireList = helperFunctions::cleanWires(wireList);
	if (!cleanWireList.size()) { cleanWireList = wireList; }
	theWireCollection_ = cleanWireList;

	std::vector<TopoDS_Wire> wireListFlat = helperFunctions::growWires(outerEdgeListFlat);
	std::vector<TopoDS_Wire> cleanWireListFlat = helperFunctions::cleanWires(wireListFlat);
	if (!cleanWireListFlat.size()) { cleanWireListFlat = wireListFlat; }
	std::vector<TopoDS_Face> cleanFaceList = helperFunctions::wireCluster2Faces(cleanWireListFlat);
	theFlatFace_ = helperFunctions::projectFaceFlat(
		cleanFaceList[0],
		urrPoint_.Z()
	);
}


const TopoDS_Face RCollection::getProjectedFace() const
{
	return helperFunctions::projectFaceFlat(
		theFlatFace_,
		0
	);
}
