#include "surfaceCollection.h"
#include "helper.h"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <BOPAlgo_Builder.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepExtrema_SelfIntersection.hxx>

EvaluationPoint::EvaluationPoint(const gp_Pnt& p)
{
	thePoint_ = p;
	evalEdge_ = BRepBuilderAPI_MakeEdge(
		p,
		gp_Pnt(p.X(), p.Y(), p.Z() + 1000)
	);
	evalLin_ = gp_Lin(p, gp_Dir(0, 0, 1000));
}


SurfaceGroup::SurfaceGroup(const TopoDS_Shape& theShape)
{
	// filter out surfaces that cannot be seen from the top (z normal component = 0)
	std::vector<TopoDS_Face> horizontalFaces;

	// TODO: make function
	for (TopExp_Explorer expl(theShape, TopAbs_FACE); expl.More(); expl.Next()) {
		// ignore if the z component of normal is 0 
		TopoDS_Face face = TopoDS::Face(expl.Current());

		GProp_GProps gprops;
		BRepGProp::SurfaceProperties(face, gprops); // Stores results in gprops
		double area = gprops.Mass();

		if (area < 0.01) { continue; }

		gp_Vec faceNormal = helperFunctions::computeFaceNormal(face);

		if (std::abs(faceNormal.Z()) < 0.001) { continue; }

		if (!helperFunctions::shapeValidity(face))
		{
			continue;
		}
		horizontalFaces.emplace_back(face);
	}

	std::vector<FaceGridPair> potentialFaces;
	for (size_t i = 0; i < horizontalFaces.size(); i++)
	{
		bool isVis = true;
		TopoDS_Face currentFace = horizontalFaces[i];

		// ignore lowest if identical projected points
		double height = helperFunctions::getAvFaceHeight(currentFace);
		int vertCount = helperFunctions::countVerts(currentFace);

		for (size_t j = 0; j < horizontalFaces.size(); j++)
		{
			if (i == j) { continue; }
			TopoDS_Face otherFace = horizontalFaces[j];

			double otherHeight = helperFunctions::getAvFaceHeight(otherFace);
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
				isVis = false;
				break;
			}
		}

		if (!isVis) { continue; }
		FaceGridPair currentPair = FaceGridPair(currentFace, 0.3);
		potentialFaces.emplace_back(currentPair);
	}

	for (size_t i = 0; i < potentialFaces.size(); i++)
	{
		bool isVis = true;
		FaceGridPair currentFaceGrid = potentialFaces[i];

		for (size_t j = 0; j < potentialFaces.size(); j++)
		{
			if (i == j)  {continue; }
			FaceGridPair otherFaceGrid = potentialFaces[j];
			if (!testIsVisable(currentFaceGrid, otherFaceGrid))
			{
				isVis = false;
				break;
			}
		}
		if (isVis)
		{
			std::vector<TopoDS_Edge> edgeList;
			//TODO: clean
			for (TopExp_Explorer edgeExp(currentFaceGrid.getFace(), TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
				edgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
			}

			std::vector<TopoDS_Wire> wireList = helperFunctions::growWires(edgeList);
			std::vector<TopoDS_Wire> cleanWireList = helperFunctions::cleanWires(wireList);
			std::vector<TopoDS_Face> cleanedFaceList = helperFunctions::wireCluster2Faces(cleanWireList);

			currentFaceGrid.replaceFace(cleanedFaceList[0]);
			theFaceCollection_.emplace_back(currentFaceGrid);
		}
	}
	return;
}


bool SurfaceGroup::overlap(SurfaceGroup other) {

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

bool SurfaceGroup::testIsVisable(FaceGridPair& evaluatedSurface, FaceGridPair& otherSurface)
{
	TopoDS_Face currentFace = evaluatedSurface.getFace();
	TopoDS_Face otherFace = otherSurface.getFace();
	if (currentFace.IsEqual(otherFace)) { return true; }

	std::vector<EvaluationPoint*> currentGrid = evaluatedSurface.getGrid();

	gp_Pnt otherLLLPoint = otherSurface.getLLLPoint();
	gp_Pnt otherURRPoint = otherSurface.getURRPoint();

	IntCurvesFace_Intersector intersector(otherFace, 1e-6);

	for (EvaluationPoint* currentEvalPoint : currentGrid)
	{
		if (!currentEvalPoint->isVisible()) { continue; }

		// check if the projection line falls withing the bbox of the surface
		gp_Pnt evalPoint = currentEvalPoint->getPoint();
		if (evalPoint.X() - otherLLLPoint.X() < -0.1 || evalPoint.X() - otherURRPoint.X() > 0.1) { continue; }
		if (evalPoint.Y() - otherLLLPoint.Y() < -0.1 || evalPoint.Y() - otherURRPoint.Y() > 0.1) { continue; }

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
	for (size_t i = 0; i < currentGrid.size(); i++) // TODO: improve to clean the surfaces
	{
		if (currentGrid[i]->isVisible())
		{
			return true;
		}
	}
	return false;
}


void SurfaceGroup::createOutLine()
{
	std::vector<TopoDS_Edge> completeEdgeList;
	// get the outer edges
	for (FaceGridPair facePair : theFaceCollection_) //TODO: checkSize
	{
		for (TopExp_Explorer edgeExp(facePair.getFace(), TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
			completeEdgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
		}
	}

	std::vector<TopoDS_Edge> OuterEdgeList;

	gp_Pnt lllPoint = gp_Pnt(999999999, 999999999, 99999999);
	gp_Pnt urrPoint = gp_Pnt(-999999999, -999999999, -99999999);

	for (TopoDS_Edge currentEdge : completeEdgeList)
	{
		gp_Pnt startPoint = helperFunctions::getFirstPointShape(currentEdge);
		gp_Pnt endPoint = helperFunctions::getLastPointShape(currentEdge);

		if (startPoint.X() > urrPoint.X()) { urrPoint.SetX(startPoint.X()); }
		if (startPoint.Y() > urrPoint.Y()) { urrPoint.SetY(startPoint.Y()); }
		if (startPoint.Z() > urrPoint.Z()) { urrPoint.SetZ(startPoint.Z()); }
		if (startPoint.X() < lllPoint.X()) { lllPoint.SetX(startPoint.X()); }
		if (startPoint.Y() < lllPoint.Y()) { lllPoint.SetY(startPoint.Y()); }
		if (startPoint.Z() < lllPoint.Z()) { lllPoint.SetZ(startPoint.Z()); }

		if (endPoint.X() > urrPoint.X()) { urrPoint.SetX(endPoint.X()); }
		if (endPoint.Y() > urrPoint.Y()) { urrPoint.SetY(endPoint.Y()); }
		if (endPoint.Z() > urrPoint.Z()) { urrPoint.SetZ(endPoint.Z()); }
		if (endPoint.X() < lllPoint.X()) { lllPoint.SetX(endPoint.X()); }
		if (endPoint.Y() < lllPoint.Y()) { lllPoint.SetY(endPoint.Y()); }
		if (endPoint.Z() < lllPoint.Z()) { lllPoint.SetZ(endPoint.Z()); }


		int count = 0;
		for (TopoDS_Edge otherEdge : completeEdgeList)
		{
			gp_Pnt otherStartPoint = helperFunctions::getFirstPointShape(otherEdge);
			gp_Pnt otherEndPoint = helperFunctions::getLastPointShape(otherEdge);

			if (startPoint.IsEqual(otherStartPoint, 0.0001) && endPoint.IsEqual(otherEndPoint, 0.0001) ||
				endPoint.IsEqual(otherStartPoint, 0.0001) && startPoint.IsEqual(otherEndPoint, 0.0001))
			{
				count++;
			}

		}

		if (count != 1) { continue; }
		OuterEdgeList.emplace_back(currentEdge);
	}
	std::vector<TopoDS_Wire> wireList = helperFunctions::growWires(OuterEdgeList);
	std::vector<TopoDS_Wire> cleanWireList = helperFunctions::cleanWires(wireList);

	wireList_ = cleanWireList;
	lllPoint_ = lllPoint;
	urrPoint_ = urrPoint;

	return;
}

void SurfaceGroup::projectFace() 
{	
	if (!wireList_.size()) { createOutLine(); }

	// merge the surfaces into 1;
	BOPAlgo_Builder faceBuilder;
	faceBuilder.SetFuzzyValue(1e-6);
	faceBuilder.SetParallelMode(Standard_False);

	if (theFaceCollection_.size() == 1)
	{
		TopoDS_Face currentFace = theFaceCollection_[0].getFace();

		theProjectedFace_ = helperFunctions::projectFaceFlat(currentFace, 0);

		gp_GTrsf trsf;
		trsf.SetVectorialPart(
			gp_Mat(
				1.0, 0.0, 0.0,
				0.0, 1.0, 0.0,
				0.0, 0.0, 0.0
			)
		);
		trsf.SetTranslationPart(gp_XYZ(0, 0, helperFunctions::getTopFaceHeight(currentFace)));

		theFlatFace_ = TopoDS::Face(BRepBuilderAPI_GTransform(theProjectedFace_, trsf, true).Shape());
		return;
	}

	double topHeight = 0;
	for (FaceGridPair facePair : theFaceCollection_)
	{
		TopoDS_Face currentFace = facePair.getFace();

		double topFace = helperFunctions::getTopFaceHeight(currentFace);

		if (topHeight < topFace) { topHeight = topFace; }

		TopoDS_Face projectedSubFace = helperFunctions::projectFaceFlat(currentFace, 0);

		faceBuilder.AddArgument(projectedSubFace);
	}
	
	faceBuilder.Perform();
	TopoDS_Shape mergedFacesShape = faceBuilder.Shape();

	std::vector<TopoDS_Edge> outerEdgeList;

	TopTools_IndexedDataMapOfShapeListOfShape mapOfEdgeToFaces;
	TopExp::MapShapesAndAncestors(mergedFacesShape, TopAbs_EDGE, TopAbs_FACE, mapOfEdgeToFaces);

	for (TopExp_Explorer edgeExplorer(mergedFacesShape, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
		TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
		const TopTools_ListOfShape& faces = mapOfEdgeToFaces.FindFromKey(edge);

		if (faces.Extent() == 1) {
			outerEdgeList.emplace_back(edge);
		}
	}

	theProjectedFace_ = helperFunctions::outerEdges2Shapes(outerEdgeList)[0]; //TODO: fix this

	gp_GTrsf trsf;
	trsf.SetVectorialPart(
		gp_Mat(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 0.0
		)
	);
	trsf.SetTranslationPart(gp_XYZ(0, 0, topHeight));

	theFlatFace_ = TopoDS::Face(BRepBuilderAPI_GTransform(theProjectedFace_, trsf, true).Shape());
	return;
}


bool SurfaceGroup::testIsVisable(const std::vector<SurfaceGroup>& otherSurfaces, bool preFilter)
{
	for (FaceGridPair currentSurface : theFaceCollection_)
	{
		TopoDS_Face currentFace = currentSurface.getFace();
		std::vector<EvaluationPoint*> currentGrid = currentSurface.getGrid();

		for (SurfaceGroup otherSurfaceGroup : otherSurfaces)
		{
			if (preFilter) //TODO: spacial q
			{
				if (!overlap(otherSurfaceGroup)) { continue; }
			}

			for (FaceGridPair otherSurface : otherSurfaceGroup.getSurfaceCollection())
			{
				if (!currentSurface.overlap(otherSurface)) { continue; }
				TopoDS_Face otherFace = otherSurface.getFace();
				if (currentFace.IsSame(otherFace)) { continue; }

				gp_Pnt otherLLLPoint = otherSurface.getLLLPoint();
				gp_Pnt otherURRPoint = otherSurface.getURRPoint();

				IntCurvesFace_Intersector intersector(otherFace, 1e-6);

				for (EvaluationPoint* currentEvalPoint : currentGrid)
				{
					if (!currentEvalPoint->isVisible()) { continue; }

					// check if the projection line falls withing the bbox of the surface
					gp_Pnt evalPoint = currentEvalPoint->getPoint();
					if (evalPoint.X() - otherLLLPoint.X() < -0.1 || evalPoint.X() - otherURRPoint.X() > 0.1) { continue; }
					if (evalPoint.Y() - otherLLLPoint.Y() < -0.1 || evalPoint.Y() - otherURRPoint.Y() > 0.1) { continue; }
					if (evalPoint.Z() - urrPoint_.Z() < 0.1) { continue; }

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
		}

		for (size_t i = 0; i < currentGrid.size(); i++) // TODO: improve to clean the surfaces
		{
			if (currentGrid[i]->isVisible())
			{
				return true;
			}
		}
	}
	visibility_ = false;
	return false;
}

bool SurfaceGroup::hasSummaryData()
{
	if (theFlatFace_.IsNull() || theProjectedFace_.IsNull() || wireList_.size() == 0)
	{
		return false;
	}
	return true;

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


void FaceGridPair::populatePointGrid(const TopoDS_Face& theFace, int vertCount, double res)
{
	pointGrid_.clear();

	double xRange = urrPoint_.X() - lllPoint_.X();
	double yRange = urrPoint_.Y() - lllPoint_.Y();

	double xDistance = res;
	double yDistance = res;

	int xSteps = ceil(xRange / xDistance);
	int ySteps = ceil(yRange / yDistance);

	xDistance = xRange / xSteps;
	yDistance = yRange / ySteps;

	IntCurvesFace_Intersector intersector(theFace, 0.0001);

	if (vertCount == 6) // If Triangle
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

		pointGrid_.emplace_back(
			new EvaluationPoint(centerPoint)
		);

		double largestAngle = helperFunctions::computeLargestAngle(theFace);


		//TODO: finetune
		if (largestAngle > 2.22 || helperFunctions::computeArea(theFace) < 1) //140 degrees
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
					pointGrid_.emplace_back(
						new EvaluationPoint(centerPoint.Translated(translationVec * j))
					);
				}

			}
			if (pointGrid_.size() <= 5) { populatePointGrid(theFace, vertCount, res / 2); } //TODO: make smarter to avoid points only on wires
			return;
		}
	}

	// if not triangle
	for (size_t i = 0; i <= xSteps; i++)
	{
		for (size_t j = 0; j <= ySteps; j++)
		{
			intersector.Perform(
				gp_Lin(
					gp_Pnt(lllPoint_.X() + xDistance * i, lllPoint_.Y() + yDistance * j, -1000),
					gp_Dir(0, 0, 1000)),
				-INFINITE,
				+INFINITE);

			if (intersector.NbPnt() == 1) {
				gp_Pnt intersectionPoint = intersector.Pnt(1);
				EvaluationPoint* evalPoint = new EvaluationPoint(intersector.Pnt(1)); //TODO: make smart
				pointGrid_.emplace_back(evalPoint);
			}
		}
	}
	if (pointGrid_.size() <= 5) { populatePointGrid(theFace, vertCount, res / 2); } //TODO: make smarter to avoid points only on wires
	return;
}

FaceGridPair::FaceGridPair(const TopoDS_Face& theFace, double res)
{
	gp_Pnt lllPoint = gp_Pnt(999999999, 999999999, 99999999);
	gp_Pnt urrPoint = gp_Pnt(-999999999, -999999999, -99999999);

	int vertCount = 0;
	TopExp_Explorer expl;
	for (expl.Init(theFace, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt point = BRep_Tool::Pnt(vertex);

		if (point.X() > urrPoint.X()) { urrPoint.SetX(point.X()); }
		if (point.Y() > urrPoint.Y()) { urrPoint.SetY(point.Y()); }
		if (point.Z() > urrPoint.Z()) { urrPoint.SetZ(point.Z()); }
		if (point.X() < lllPoint.X()) { lllPoint.SetX(point.X()); }
		if (point.Y() < lllPoint.Y()) { lllPoint.SetY(point.Y()); }
		if (point.Z() < lllPoint.Z()) { lllPoint.SetZ(point.Z()); }

		vertCount++;
	}

	urrPoint_ = urrPoint;
	lllPoint_ = lllPoint;
	face_ = theFace;

	populatePointGrid(theFace, vertCount, res);	
}

bool FaceGridPair::overlap(FaceGridPair otherFacePair)
{
	bool inXD = false;
	bool inYD = false;

	if (lllPoint_.X() <= otherFacePair.lllPoint_.X() && urrPoint_.X() >= otherFacePair.lllPoint_.X()) { inXD = true; }
	if (otherFacePair.lllPoint_.X() <= lllPoint_.X() && otherFacePair.urrPoint_.X() > lllPoint_.X()) { inXD = true; }
	if (lllPoint_.Y() <= otherFacePair.lllPoint_.Y() && urrPoint_.Y() >= otherFacePair.lllPoint_.Y()) { inYD = true; }
	if (otherFacePair.lllPoint_.Y() <= lllPoint_.Y() && otherFacePair.urrPoint_.Y() >= lllPoint_.Y()) { inYD = true; }

	if (inXD && inYD)
	{
		return true;
	}
	return false;
}
