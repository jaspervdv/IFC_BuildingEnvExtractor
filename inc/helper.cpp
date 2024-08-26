#include "helper.h"
#include "settingsCollection.h"
#include "stringManager.h"
#include "errorCollection.h"

#include <CJToKernel.h>
#include <iostream>
#include <string>
#include <filesystem>


#include <sys/stat.h>

#include <BOPAlgo_Splitter.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>
#include <GProp_GProps.hxx>
#include <IntTools_EdgeFace.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TopoDS.hxx>
#include <BRepTools.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>

#include <Geom_TrimmedCurve.hxx>
#include <gp_Lin.hxx>

#include <gp_Quaternion.hxx>


void helperFunctions::WriteToSTEP(const TopoDS_Solid& shape, const std::string& addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";

	STEPControl_Writer writer;

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}


void helperFunctions::WriteToSTEP(const TopoDS_Shape& shape, const std::string& addition) {
	std::string path = "C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/step" + addition + ".stp";
	STEPControl_Writer writer;

	/*TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
		writer.Transfer(expl.Current(), STEPControl_ManifoldSolidBrep);
	}

	IFSelect_ReturnStatus stat = writer.Write(path.c_str());*/

	writer.Transfer(shape, STEPControl_AsIs);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}


void helperFunctions::WriteToSTEP(const std::vector<TopoDS_Face>& shapeList, const std::string& addition)
{
	std::string path = "C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/step" + addition + ".stp";

	STEPControl_Writer writer;
	writer.SetTolerance(1e-100);

	for (TopoDS_Shape shape : shapeList)
	{
		writer.Transfer(shape, STEPControl_AsIs);
	}
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

}

void helperFunctions::WriteToSTEP(const std::vector<TopoDS_Shape>& shapeList, const std::string& addition)
{
	std::string path = "C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/step" + addition + ".stp";

	STEPControl_Writer writer;
	writer.SetTolerance(1e-100);

	for (TopoDS_Shape shape : shapeList)
	{
		writer.Transfer(shape, STEPControl_AsIs);
	}
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

}

void helperFunctions::WriteToTxt(const std::vector<TopoDS_Face>& shapeList, const std::string& addition)
{
	std::ofstream outputFile("C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/step" + addition + ".txt");

	for (TopoDS_Face shape: shapeList)
	{
		outputFile << "new" << std::endl;
		for (TopExp_Explorer expl(shape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			outputFile << p.X() << ", " << p.Y() << ", " << p.Z() << std::endl;
		}
	}

	outputFile.close();
}


void helperFunctions::WriteToTxt(const std::vector<TopoDS_Solid>& shapeList, const std::string& addition)
{
	std::ofstream outputFile("C:/Users/Jasper/Documents/1_projects/IFCEnvelopeExtraction/IFC_BuildingEnvExtractor/exports/step" + addition + ".txt");

	for (TopoDS_Solid shape : shapeList)
	{
		for (TopExp_Explorer explFace(shape, TopAbs_FACE); explFace.More(); explFace.Next())
		{
			outputFile << "new" << std::endl;
			for (TopExp_Explorer expl(TopoDS::Face(explFace.Current()), TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);

				outputFile << p.X() << ", " << p.Y() << ", " << p.Z() << std::endl;
			}
		}
	}

	outputFile.close();
}


void helperFunctions::printPoint(const gp_Pnt& p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << "\n";
}


void helperFunctions::printPoint(const gp_Pnt2d& p) {
	std::cout << p.X() << ", " << p.Y() << "\n";
}


void helperFunctions::printPoint(const BoostPoint3D& p) {
	std::cout << bg::get<0>(p) << ", " << bg::get<1>(p) << ", " << bg::get<2>(p) << "\n";
}


void helperFunctions::printPoint(const gp_Vec& p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << "\n";
}


void helperFunctions::printPoint(const gp_Vec2d& p) {
	std::cout << p.X() << ", " << p.Y() << "\n";
}


void helperFunctions::printPoints(const TopoDS_Wire& w)
{
	for (TopExp_Explorer expl(w, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		printPoint(p);
	}
}


void helperFunctions::printFaces(const TopoDS_Shape& shape)
{
	//std::cout << "Shape:" << std::endl;
	std::vector<TopoDS_Face> faceList;

	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		faceList.emplace_back(TopoDS::Face(expl.Current()));
	}

	for (size_t i = 0; i < faceList.size(); i++)
	{
		std::cout << "new" << std::endl;
		for (expl.Init(faceList[i], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			printPoint(p);
		}
	}
	//std::cout << std::endl;
}


std::vector<gp_Pnt> helperFunctions::getUniquePoints(const std::vector<gp_Pnt>& pointList) {
	std::vector<gp_Pnt> uniquePoints;
	for (size_t j = 0; j < pointList.size(); j++)
	{
		bool dub = false;

		for (size_t k = 0; k < uniquePoints.size(); k++)
		{
			if (pointList[j].IsEqual(uniquePoints[k], 0.001))
			{
				dub = true;
				break;
			}
		}
		if (!dub)
		{
			uniquePoints.emplace_back(pointList[j]);
		}
	}
	return uniquePoints;
}

std::vector<gp_Pnt> helperFunctions::getUniquePoints(const TopoDS_Shape& inputShape)
{
	std::vector<gp_Pnt> uniquePoints;
	for (TopExp_Explorer expl(inputShape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		bool dub = false;

		for (size_t k = 0; k < uniquePoints.size(); k++)
		{
			if (p.IsEqual(uniquePoints[k], 0.001))
			{
				dub = true;
				break;
			}
		}
		if (!dub)
		{
			uniquePoints.emplace_back(p);
		}
	}
	return uniquePoints;
}


int helperFunctions::getPointCount(const TopoDS_Shape& inputShape)
{
	int pCount = 0;
	for (TopExp_Explorer expl(inputShape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		pCount++;
	}
	return pCount;
}

gp_Pnt helperFunctions::rotatePointWorld(const gp_Pnt& p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}


TopoDS_Wire helperFunctions::reversedWire(const TopoDS_Wire& mainWire) {
	BRepBuilderAPI_MakeWire wireMaker;

	for (TopExp_Explorer wireExp(mainWire, TopAbs_EDGE); wireExp.More(); wireExp.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(wireExp.Current());
		gp_Pnt firstPoint = getFirstPointShape(edge);
		gp_Pnt secondPoint = getLastPointShape(edge);

		wireMaker.Add(BRepBuilderAPI_MakeEdge(secondPoint, firstPoint));
	}
	wireMaker.Build();

	if (wireMaker.IsDone()) { return wireMaker.Wire(); }
	return BRepBuilderAPI_MakeWire();
}


BoostPoint3D helperFunctions::rotatePointWorld(const BoostPoint3D& p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}


gp_Pnt helperFunctions::rotatePointPoint(const gp_Pnt& p, const gp_Pnt& anchorP, double angle)
{
	gp_Pnt translatedP = p.Translated(gp_Vec(-anchorP.X(), -anchorP.Y(), -anchorP.Z()));
	gp_Pnt rotatedP = helperFunctions::rotatePointWorld(translatedP, angle);
	return rotatedP.Translated(gp_Vec(anchorP.X(), anchorP.Y(), anchorP.Z()));
}


bool helperFunctions::bBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, double buffer) {

	bool isPSet = false;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = pointList[i];

		if (!isPSet)
		{
			isPSet = true;

			*lllPoint = point;
			*urrPoint = point;
		}
		else
		{
			if (point.X() < lllPoint->X()) { lllPoint->SetX(point.X() - buffer); }
			if (point.Y() < lllPoint->Y()) { lllPoint->SetY(point.Y() - buffer); }
			if (point.Z() < lllPoint->Z()) { lllPoint->SetZ(point.Z() - buffer); }

			if (point.X() > urrPoint->X()) { urrPoint->SetX(point.X() + buffer); }
			if (point.Y() > urrPoint->Y()) { urrPoint->SetY(point.Y() + buffer); }
			if (point.Z() > urrPoint->Z()) { urrPoint->SetZ(point.Z() + buffer); }
		}
	}
	if (lllPoint->IsEqual(*urrPoint, 0.01)) { return false; }
	return true;
}

bool helperFunctions::rotatedBBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint , double angle, double secondAngle) {

	bool isPSet = false;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = helperFunctions::rotatePointWorld(pointList[i], angle).Rotated(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0, 1, 0)), -secondAngle);

		if (!isPSet)
		{
			isPSet = true;

			*lllPoint = point;
			*urrPoint = point;
		}
		else
		{
			if (point.X() < lllPoint->X()) { lllPoint->SetX(point.X()); }
			if (point.Y() < lllPoint->Y()) { lllPoint->SetY(point.Y()); }
			if (point.Z() < lllPoint->Z()) { lllPoint->SetZ(point.Z()); }

			if (point.X() > urrPoint->X()) { urrPoint->SetX(point.X()); }
			if (point.Y() > urrPoint->Y()) { urrPoint->SetY(point.Y()); }
			if (point.Z() > urrPoint->Z()) { urrPoint->SetZ(point.Z()); }
		}
	}
	if (lllPoint->IsEqual(*urrPoint, 0.01)) { return false; }
	return true;
}


BoostPoint3D helperFunctions::Point3DOTB(const gp_Pnt& oP){
	return BoostPoint3D(oP.X(), oP.Y(), oP.Z());
}


gp_Pnt helperFunctions::Point3DBTO(const BoostPoint3D& oP) {
	return gp_Pnt(bg::get<0>(oP), bg::get<1>(oP), bg::get<2>(oP));
}


gp_Pnt helperFunctions::getLowestPoint(const TopoDS_Shape& shape, bool areaFilter)
{
	double lowestZ = 9999;
	gp_Pnt lowestP(0, 0, 0);

	TopExp_Explorer expl;

	TopoDS_Shape largestFace = shape;

	if (areaFilter)
	{
		GProp_GProps gprop;
		double area = 0;

		for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
		{
			BRepGProp::SurfaceProperties(TopoDS::Face(expl.Current()), gprop);
			if (area < gprop.Mass())
			{
				area = gprop.Mass();
				largestFace = TopoDS::Face(expl.Current());
			}
		}
	}

	for (expl.Init(largestFace, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() < lowestZ)
		{
			lowestZ = p.Z();
			lowestP = p;
		}
	}

	double sumX = 0;
	double sumY = 0;
	int aP = 0;

	for (expl.Init(largestFace, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() == lowestZ)
		{
			aP++;
			sumX += p.X();
			sumY += p.Y();
		}
	}
	return gp_Pnt(sumX / aP, sumY / aP, lowestZ);
}


gp_Pnt helperFunctions::getHighestPoint(const TopoDS_Shape& shape)
{
	double highestZ = -9999;
	gp_Pnt highestP(0, 0, 0);

	TopExp_Explorer expl;

	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() > highestZ)
		{
			highestZ = p.Z();
			highestP = p;
		}
	}

	double sumX = 0;
	double sumY = 0;
	int aP = 0;

	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() == highestZ)
		{
			aP++;
			sumX += p.X();
			sumY += p.Y();
		}
	}
	return gp_Pnt(sumX / aP, sumY / aP, highestZ);
}


std::optional<gp_Pnt> helperFunctions::getPointOnFace(const TopoDS_Face& theFace) 
{
	triangulateShape(theFace);

	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	for (int i = 1; i <= mesh.get()->NbTriangles(); i++) 
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		gp_Pnt middlePoint = gp_Pnt(
			(p1.X() + p2.X() + p3.X()) / 3,
			(p1.Y() + p2.Y() + p3.Y()) / 3,
			(p1.Z() + p2.Z() + p3.Z()) / 3
		);
		return middlePoint;
	}
	return std::nullopt;
}

std::vector<gp_Pnt> helperFunctions::getPointListOnFace(const TopoDS_Face& theFace)
{
	triangulateShape(theFace);

	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	std::vector<gp_Pnt> pointList;
	for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		gp_Pnt middlePoint = gp_Pnt(
			(p1.X() + p2.X() + p3.X()) / 3,
			(p1.Y() + p2.Y() + p3.Y()) / 3,
			(p1.Z() + p2.Z() + p3.Z()) / 3
		);
		pointList.emplace_back(middlePoint);
	}
	return pointList;
}


gp_Pnt helperFunctions::getFirstPointShape(const TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex startVertex;

	if (vertexExplorer.More()) {
		startVertex = TopoDS::Vertex(vertexExplorer.Current());
	}

	gp_Pnt startPoint = BRep_Tool::Pnt(startVertex);
	return startPoint;
}


gp_Pnt helperFunctions::getLastPointShape(const TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex endVertex;

	while (vertexExplorer.More()) {
		endVertex = TopoDS::Vertex(vertexExplorer.Current());
		vertexExplorer.Next();
	}
	return BRep_Tool::Pnt(endVertex);;
}


gp_Vec helperFunctions::computeFaceNormal(const TopoDS_Face& theFace)
{
	double precision = SettingsCollection::getInstance().precision();

	gp_Vec vec1;
	gp_Vec vec2;

	bool hasVec1 = false;
	bool hasVec2 = false;


	for (TopExp_Explorer vertexExp(theFace, TopAbs_EDGE); vertexExp.More(); vertexExp.Next()) {
		TopoDS_Edge edge = TopoDS::Edge(vertexExp.Current());

		gp_Pnt startpoint = getFirstPointShape(edge);
		gp_Pnt endpoint = getLastPointShape(edge);

		if (startpoint.IsEqual(endpoint, precision)) { continue; }

		if (!hasVec1)
		{
			hasVec1 = true;
			vec1 = gp_Vec(startpoint, endpoint);
			continue;
		}

		gp_Vec potentialVec2 = gp_Vec(startpoint, endpoint);

		if (potentialVec2.IsParallel(vec1, 1e-06)) { continue; }

		if (!hasVec2)
		{
			hasVec2 = true;
			vec2 = gp_Vec(startpoint, endpoint);
			break;
		}
	}
	//std::cout << "out" << std::endl;
	if (!hasVec1 || !hasVec2) { return gp_Vec(0, 0, 0); }

	gp_Vec normal = vec1.Crossed(vec2);
	normal.Normalize();
	return normal;
}

double helperFunctions::computeLargestAngle(const TopoDS_Face& theFace)
{
	double precision = SettingsCollection::getInstance().precision();
	std::vector<gp_Pnt> pointList = {};

	double biggestAngle = 0;
	for (TopExp_Explorer expl(theFace, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());

		gp_Pnt Anglepoint = helperFunctions::getFirstPointShape(currentEdge);
		gp_Pnt legPoint1 = helperFunctions::getLastPointShape(currentEdge);
		gp_Pnt legPoint2;

		bool leg2Found = false;
		for (TopExp_Explorer expl2(theFace, TopAbs_EDGE); expl2.More(); expl2.Next())
		{
			TopoDS_Edge otherEdge = TopoDS::Edge(expl2.Current());
			if (currentEdge.IsSame(otherEdge)) { continue; }

			gp_Pnt otherP1 = helperFunctions::getFirstPointShape(otherEdge);
			gp_Pnt otherP2 = helperFunctions::getLastPointShape(otherEdge);

			if (otherP1.IsEqual(Anglepoint, precision))
			{ 
				leg2Found = true;
				legPoint2 = otherP2; 
				break;
			}
			else if (otherP2.IsEqual(Anglepoint, precision))
			{ 
				leg2Found = false;
				legPoint2 = otherP1;
				break;
			}
		}

		gp_Vec v1 = gp_Vec(Anglepoint, legPoint1);
		gp_Vec v2 = gp_Vec(Anglepoint, legPoint2);

		double angle = v1.Angle(v2);

		if (angle > biggestAngle)
		{
			biggestAngle = angle;
		}

	}
	return biggestAngle;
}


bool helperFunctions::edgeEdgeOVerlapping(const TopoDS_Edge& currentEdge, const TopoDS_Edge& otherEdge)
{
	gp_Pnt cP0 = getFirstPointShape(currentEdge);
	gp_Pnt cP1 = getLastPointShape(currentEdge);
	gp_Pnt oP0 = getFirstPointShape(otherEdge);
	gp_Pnt oP1 = getLastPointShape(otherEdge);

	double precision = SettingsCollection::getInstance().precision();

	if (cP0.IsEqual(oP0, precision) && cP1.IsEqual(oP1, precision)) { return true; }
	if (cP1.IsEqual(oP0, precision) && cP0.IsEqual(oP1, precision)) { return true; }

	return false;
}


bool helperFunctions::faceFaceOverlapping(const TopoDS_Face& upperFace, const TopoDS_Face& lowerFace)
{
	// compute area
	double setPresicion = SettingsCollection::getInstance().precision();
	if (abs(computeArea(upperFace) - computeArea(lowerFace)) > setPresicion) { return false; }

	// align verts
	for (TopExp_Explorer currentVertExpl(upperFace, TopAbs_VERTEX); currentVertExpl.More(); currentVertExpl.Next())
	{
		TopoDS_Vertex currentVertex = TopoDS::Vertex(currentVertExpl.Current());
		gp_Pnt currentPoint = BRep_Tool::Pnt(currentVertex);

		bool vertFound = false;
		for (TopExp_Explorer otherVertExpl(lowerFace, TopAbs_VERTEX); otherVertExpl.More(); otherVertExpl.Next())
		{
			TopoDS_Vertex otherVertex = TopoDS::Vertex(otherVertExpl.Current());
			gp_Pnt otherPoint = BRep_Tool::Pnt(otherVertex);

			double hDistance = sqrt(pow(currentPoint.X() - otherPoint.X(), 2) + pow(currentPoint.Y() - otherPoint.Y(), 2));

			if (hDistance > setPresicion) { continue; }
			if (currentPoint.Z() < otherPoint.Z()) { continue; }

			vertFound = true;
			break;
		}

		if (!vertFound)
		{
			return false;
		}
	}
	return true;
}

std::vector<TopoDS_Face> helperFunctions::shape2FaceList(const TopoDS_Shape& shape)
{
	std::vector<TopoDS_Face> faceList;
	for (TopExp_Explorer expl(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		faceList.emplace_back(TopoDS::Face(expl.Current()));
	}
	return faceList;
}

std::vector<gp_Pnt> helperFunctions::shape2PointList(const TopoDS_Shape& shape)
{
	if (shape.IsNull()) { return {}; }

	std::vector<gp_Pnt> pointList;
	for (TopExp_Explorer expl(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		pointList.emplace_back(BRep_Tool::Pnt(vertex));
	}
	return pointList;
}


std::vector<IfcSchema::IfcProduct*> helperFunctions::getNestedProducts(IfcSchema::IfcProduct* product) {
	
	std::vector<IfcSchema::IfcProduct*> productList;

	if (product->Representation() == nullptr)
	{
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() == 0) { return {}; }

		for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {

#if defined(USE_IFC4) || defined(USE_IFC4x3)
			IfcSchema::IfcRelAggregates* aggregates = *et;
#else
			IfcSchema::IfcRelDecomposes* aggregates = *et;
#endif // USE_IFC4

			IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

			for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
				IfcSchema::IfcObjectDefinition* aggDef = *rt;
				productList.emplace_back(aggDef->as<IfcSchema::IfcProduct>());
			}
		}
	}
	else if (product->Representation() != nullptr)
	{
		productList.emplace_back(product);
	}
	return productList;
}

void helperFunctions::geoTransform(TopoDS_Shape* shape, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans)
{
	shape->Move(objectTrans.Inverted());
	shape->Move(geoTrans);
	shape->Move(objectTrans);
}


void helperFunctions::geoTransform(TopoDS_Face* shape, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans)
{
	shape->Move(objectTrans.Inverted());
	shape->Move(geoTrans);
	shape->Move(objectTrans);
}

void helperFunctions::geoTransform(gp_Pnt* point, const gp_Trsf& objectTrans, const gp_Trsf& geoTrans)
{
	point->Translate(objectTrans.Inverted().TranslationPart());
	gp_Pnt rotatedPoint = helperFunctions::rotatePointWorld(*point, geoTrans.GetRotation().GetRotationAngle());
	*point = rotatedPoint;
	point->Translate(geoTrans.TranslationPart());
	point->Translate(objectTrans.TranslationPart());
}

void helperFunctions::triangulateShape(const TopoDS_Shape& shape)
{
	auto hasTriangles = BRepTools::Triangulation(shape, 0.01);
	if (!hasTriangles) { auto mesh = BRepMesh_IncrementalMesh(shape, 0.01); }
}


double helperFunctions::tVolume(const gp_Pnt& p, const std::vector<gp_Pnt>& vertices) {
	BoostPoint3D p1(vertices[0].X() - vertices[1].X(), vertices[0].Y() - vertices[1].Y(), vertices[0].Z() - vertices[1].Z());
	BoostPoint3D p2(vertices[1].X() - p.X(), vertices[1].Y() - p.Y(), vertices[1].Z() - p.Z());
	BoostPoint3D p3(vertices[2].X() - p.X(), vertices[2].Y() - p.Y(), vertices[2].Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3)) / 6;
}


bool helperFunctions::triangleIntersecting(const std::vector<gp_Pnt>& line, const std::vector<gp_Pnt>& triangle)
{
	// check for potential intersection
	double left = tVolume(triangle[0], { triangle[2], line[0], line[1] });
	double right = tVolume(triangle[1], { triangle[2], line[0], line[1] });
	if (left > 0 && right > 0 || left < 0 && right < 0) { return false; }

	double left2 = tVolume(triangle[1], { triangle[0], line[0], line[1] });
	double right2 = tVolume(triangle[2], { triangle[0], line[0], line[1] });
	if (left2 > 0 && right2 > 0 || left2 < 0 && right2 < 0) { return false; }

	double left3 = tVolume(triangle[2], { triangle[1], line[0], line[1] });
	double right3 = tVolume(triangle[0], { triangle[1], line[0], line[1] });
	if (left3 > 0 && right3 > 0 || left3 < 0 && right3 < 0) { return false; }

	double leftFinal = tVolume(line[0], triangle);
	double rightFinal = tVolume(line[1], triangle);

	if (abs(leftFinal) < 1e-6 || abs(rightFinal) < 1e-6) { return false; } // if surfaces rest on eachother return 0
	if (leftFinal > 0 && rightFinal < 0 || rightFinal > 0 && leftFinal < 0) { return true; }

	return false;
}


bool helperFunctions::hasVolume(const bg::model::box<BoostPoint3D>& box)
{
	if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
		bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
		return false;
	}
	return true;
}


template<typename T1, typename T2>
inline bool helperFunctions::isInList(const T1& list, const T2& value)
{
	auto it = std::find(list.begin(), list.end(), value);
	if (it != list.end()) {
		return true;
	}
	return false;
}


double helperFunctions::getAvFaceHeight(const TopoDS_Face& face) {
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
	return totalZ / pCount;
}


double helperFunctions::getTopFaceHeight(const TopoDS_Face& face) {
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


gp_Vec helperFunctions::getDirEdge(const TopoDS_Edge& edge) {
	gp_Pnt startpoint = getFirstPointShape(edge);
	gp_Pnt endpoint = getLastPointShape(edge);

	if (startpoint.Distance(endpoint) == 0) { return gp_Vec(0, 0, 0); }

	gp_Vec vec = gp_Vec(startpoint, endpoint);
	return vec.Normalized();
}


double helperFunctions::computeArea(const TopoDS_Face& theFace)
{
	GProp_GProps gprops;
	BRepGProp::SurfaceProperties(theFace, gprops); // Stores results in gprops
	return gprops.Mass();
}

TopoDS_Wire helperFunctions::mergeWireOrientated(const TopoDS_Wire& baseWire, const TopoDS_Wire& mergingWire) {
	double precision = SettingsCollection::getInstance().precision();
	
	gp_Pnt connectionPoint1 = getFirstPointShape(baseWire);
	gp_Pnt connectionPoint2 = getLastPointShape(baseWire);

	gp_Pnt p1 = getFirstPointShape(mergingWire);
	gp_Pnt p2 = getLastPointShape(mergingWire);

	BRepBuilderAPI_MakeWire wiremaker;

	if (connectionPoint1.Distance(p2) < precision) // correct orentation placed in front
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
	if (connectionPoint1.Distance(p1) < precision) // wrong orentation placed in front
	{
		std::vector<TopoDS_Edge> tempEdgeList;
		for (TopExp_Explorer explorer(mergingWire, TopAbs_EDGE); explorer.More(); explorer.Next())
		{
			const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
			gp_Pnt beginPoint = helperFunctions::getFirstPointShape(edge);
			gp_Pnt endPoint = helperFunctions::getLastPointShape(edge);
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
	if (connectionPoint2.Distance(p1) < precision) // correct orentation placed after
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
	if (connectionPoint2.Distance(p2) < precision) // wrong orentation placed after
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
			gp_Pnt beginPoint = helperFunctions::getFirstPointShape(edge);
			gp_Pnt endPoint = helperFunctions::getLastPointShape(edge);
			tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(endPoint, beginPoint));
		}

		for (std::vector<TopoDS_Edge>::reverse_iterator i = tempEdgeList.rbegin();
			i != tempEdgeList.rend(); ++i) {

			wiremaker.Add(*i);
		}

		if (wiremaker.IsDone()) { return wiremaker.Wire(); }
	}
	return TopoDS_Wire();
}


TopoDS_Wire helperFunctions::closeWireOrientated(const TopoDS_Wire& baseWire) {
	gp_Pnt p1 = helperFunctions::getFirstPointShape(baseWire);
	gp_Pnt p2 = helperFunctions::getLastPointShape(baseWire);

	if (p1.Distance(p2) < SettingsCollection::getInstance().precision()) { return baseWire; }

	TopoDS_Wire closingWire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(p2, p1));

	return mergeWireOrientated(baseWire, closingWire);
}

std::vector<gp_Pnt> helperFunctions::getPointGridOnSurface(const TopoDS_Face& theface)
{
	double precision = SettingsCollection::getInstance().precision();
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	Handle(Geom_Surface) surface = BRep_Tool::Surface(theface);

	// greate points on grid over surface
	// get the uv bounds to create a point grid on the surface
	Standard_Real uMin, uMax, vMin, vMax;
	BRepTools::UVBounds(theface, BRepTools::OuterWire(theface), uMin, uMax, vMin, vMax);

	uMin = uMin + 0.05;
	uMax = uMax - 0.05;
	vMin = vMin + 0.05;
	vMax = vMax - 0.05;

	int numUPoints = static_cast<int>(ceil(abs(uMax - uMin) / settingsCollection.voxelSize()));
	int numVPoints = static_cast<int>(ceil(abs(vMax - vMin) / settingsCollection.voxelSize()));

	// set num of points if min/max rule is not met
	if (numUPoints < 2) { numUPoints = 2; }
	else if (numUPoints > 10) { numUPoints = 10; } // TODO: check if this max setting is smart
	if (numVPoints < 2) { numVPoints = 2; }
	else if (numVPoints > 10) { numVPoints = 10; }

	double uStep = (uMax - uMin) / (numUPoints - 1);
	double vStep = (vMax - vMin) / (numVPoints - 1);

	// create grid
	std::vector<gp_Pnt> gridPointList;
	for (int i = 0; i < numUPoints; ++i)
	{
		double u = uMin + i * uStep;

		for (int j = 0; j < numVPoints; ++j)
		{
			double v = vMin + j * vStep;
			gp_Pnt point;
			surface->D0(u, v, point);
			BRepClass_FaceClassifier faceClassifier(theface, point, precision);
			if (faceClassifier.State() != TopAbs_ON && faceClassifier.State() != TopAbs_IN) { continue; }
			gridPointList.emplace_back(point);
		}
	}
	return gridPointList;
}

std::vector<gp_Pnt> helperFunctions::getPointGridOnWire(const TopoDS_Face& theface)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	BRepOffsetAPI_MakeOffset offsetter(BRepTools::OuterWire(theface), GeomAbs_Intersection);
	
	if (helperFunctions::computeArea(theface) < 0.01) { return {}; }
	offsetter.Perform(-settingsCollection.precisionCoarse());


	if (!offsetter.IsDone()) { return {}; }
	const TopoDS_Shape offsettedFace = offsetter.Shape();
	if (offsettedFace.IsNull()){ return {};}

	std::vector<gp_Pnt> wirePointList;
	for (TopExp_Explorer expl(offsetter.Shape(), TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		
		BRepAdaptor_Curve curveAdaptor(currentEdge);
		
		double uStart = curveAdaptor.Curve().FirstParameter();
		double uEnd = curveAdaptor.Curve().LastParameter();
		int numUPoints = static_cast<int>(ceil(abs(uStart - uEnd)) / settingsCollection.voxelSize());

		if (numUPoints < 2) { numUPoints = 2; }
		else if (numUPoints > 10) { numUPoints = 10; }

		double uStep = abs(uStart - uEnd) / (numUPoints - 1);
		for (double u = uStart; u < uEnd; u += uStep) {
			gp_Pnt point;
			curveAdaptor.D0(u, point);
			wirePointList.emplace_back(point);
		}
	}
	return wirePointList;
}


int helperFunctions::invertDir(int dirIndx) {

	if (dirIndx % 2 == 1) { return dirIndx - 1; }
	else { return dirIndx + 1; }
}


bg::model::box <BoostPoint3D> helperFunctions::createBBox(const TopoDS_Shape& shape) {

	double buffer = 0.05;
	Bnd_Box boundingBox;
	BRepBndLib::Add(shape, boundingBox);

	if (boundingBox.IsVoid()) { return  bg::model::box < BoostPoint3D >(); }

	Standard_Real minX, minY, minZ, maxX, maxY, maxZ;
	boundingBox.Get(minX, minY, minZ, maxX, maxY, maxZ);
	return  bg::model::box < BoostPoint3D >(
		BoostPoint3D(minX - buffer, minY - buffer, minZ - buffer),
		BoostPoint3D(maxX + buffer, maxY + buffer, maxZ + buffer)
		);
}


bg::model::box <BoostPoint3D> helperFunctions::createBBox(const std::vector<TopoDS_Shape>& shape) {

	double buffer = 0.05;
	Bnd_Box boundingBox;
	for (size_t i = 0; i < shape.size(); i++)
	{
		BRepBndLib::Add(shape[i], boundingBox);
	}

	// Step 3: Get the bounds of the bounding box.
	Standard_Real minX, minY, minZ, maxX, maxY, maxZ;
	boundingBox.Get(minX, minY, minZ, maxX, maxY, maxZ);
	return  bg::model::box < BoostPoint3D >(
		BoostPoint3D(minX - buffer, minY - buffer, minZ - buffer),
		BoostPoint3D(maxX + buffer, maxY + buffer, maxZ + buffer)
		);
}


bg::model::box <BoostPoint3D>  helperFunctions::createBBox(const gp_Pnt& p1, const gp_Pnt& p2, double buffer) {

	// get proper order for the bbox
	gp_Pnt lll(
		std::min(p1.X(), p2.X()),
		std::min(p1.Y(), p2.Y()),
		std::min(p1.Z(), p2.Z())
	);

	gp_Pnt urr(
		std::max(p1.X(), p2.X()),
		std::max(p1.Y(), p2.Y()),
		std::max(p1.Z(), p2.Z())
	);

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

	BoostPoint3D boostlllPoint = BoostPoint3D(lll.X() - buffer, lll.Y() - buffer, lll.Z() - buffer);
	BoostPoint3D boosturrPoint = BoostPoint3D(urr.X() + buffer, urr.Y() + buffer, urr.Z() + buffer);

	bg::model::box <BoostPoint3D> box = bg::model::box < BoostPoint3D >(boostlllPoint, boosturrPoint);

	return box;
}


TopoDS_Shape helperFunctions::createBBOXOCCT(const gp_Pnt& lll, const gp_Pnt& urr, double buffer) {
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid outerbb;
	brepBuilder.MakeSolid(outerbb);

	gp_Pnt p0 = gp_Pnt(lll.X() - buffer, lll.Y() - buffer, lll.Z() - buffer);
	gp_Pnt p4 = gp_Pnt(urr.X() + buffer, urr.Y() + buffer, urr.Z() + buffer);

	gp_Pnt p1 = gp_Pnt(p0.X(), p4.Y(), p0.Z());
	gp_Pnt p2 = gp_Pnt(p4.X(), p4.Y(), p0.Z());
	gp_Pnt p3 = gp_Pnt(p4.X(), p0.Y(), p0.Z());
	gp_Pnt p5 = gp_Pnt(p0.X(), p4.Y(), p4.Z());
	gp_Pnt p6 = gp_Pnt(p0.X(), p0.Y(), p4.Z());
	gp_Pnt p7 = gp_Pnt(p4.X(), p0.Y(), p4.Z());

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
	brepBuilder.Add(outerbb, brepSewer.SewedShape());

	return outerbb;
}


TopoDS_Face helperFunctions::createHorizontalFace(double x, double y, double z) {
	
	gp_Pnt p0(-x, -y, z);
	gp_Pnt p1(-x, y, z);
	gp_Pnt p2(x, y, z);
	gp_Pnt p3(x, -y, z);

	return createPlanarFace(p0, p1, p2, p3);
}

TopoDS_Face helperFunctions::createHorizontalFace(const gp_Pnt& lll, const gp_Pnt& urr, double rotationAngle) {
	gp_Pnt p0 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), 0), -rotationAngle);
	gp_Pnt p1 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p2 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), 0), -rotationAngle);
	gp_Pnt p3 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), 0), -rotationAngle);

	return createPlanarFace(p0, p1, p2, p3);
}


TopoDS_Face helperFunctions::createPlanarFace(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3) {

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	return BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));
}


TopoDS_Face helperFunctions::projectFaceFlat(const TopoDS_Face& theFace, double height) {
	
	gp_GTrsf trsf;
	trsf.SetVectorialPart(
		gp_Mat(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 
			0.0, 0.0, 0.0
		)
	);
	trsf.SetTranslationPart(gp_XYZ(0, 0, height));

	TopoDS_Shape flatFace = BRepBuilderAPI_GTransform(theFace, trsf, true).Shape();
	return TopoDS::Face(flatFace);
}



std::optional<gp_Pnt> helperFunctions::linearLineIntersection(const gp_Pnt& sP1, const gp_Pnt& eP1, const gp_Pnt& sP2, const gp_Pnt& eP2, bool projected, double buffer) {

	double precision = SettingsCollection::getInstance().precision();

	gp_Pnt evalSP1 = sP1;
	gp_Pnt evalEP1 = eP1;
	gp_Pnt evalSP2 = sP2;
	gp_Pnt evalEP2 = eP2;

	if (projected)
	{
		evalSP1.SetZ(0);
		evalEP1.SetZ(0);
		evalSP2.SetZ(0);
		evalEP2.SetZ(0);
	}

	if (evalSP1.IsEqual(evalEP1, precision)) { return std::nullopt; }
	if (evalSP2.IsEqual(evalEP2, precision)) { return std::nullopt; }

	double z = 0; //Todo: make work in 3d
	if (!projected) { z = evalEP1.Z(); }

	gp_Vec v1(evalSP1, evalEP1);
	v1.Normalize();
	gp_Vec v2(evalSP2, evalEP2);
	v2.Normalize();

	if (v1.IsEqual(v2, precision, precision)) { return std::nullopt; }

	double x1 = evalSP1.X();
	double x2 = evalEP1.X();;
	double x3 = evalSP2.X();;
	double x4 = evalEP2.X();;

	double y1 = evalSP1.Y();
	double y2 = evalEP1.Y();
	double y3 = evalSP2.Y();
	double y4 = evalEP2.Y();

	double dom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

	if (abs(dom) == 0) { return std::nullopt; }

	double xI = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / (dom);
	double yI = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / (dom);

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
					return gp_Pnt(xI, yI, z);
				}
			}
		}
	}
	return std::nullopt;
}


std::optional<gp_Pnt> helperFunctions::linearLineIntersection(const Edge& edge1, const Edge& edge2, bool projected, double buffer) {
	gp_Pnt sP1 = edge1.getStart(false);
	gp_Pnt eP1 = edge1.getEnd(false);
	gp_Pnt sP2 = edge2.getStart(false);
	gp_Pnt eP2 = edge2.getEnd(false);

	return linearLineIntersection(sP1, eP1, sP2, eP2, projected, buffer);
}


std::optional<gp_Pnt> helperFunctions::linearLineIntersection(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, bool projected, double buffer) {
	return linearLineIntersection(
		helperFunctions::getFirstPointShape(edge1),
		helperFunctions::getLastPointShape(edge1),
		helperFunctions::getFirstPointShape(edge2),
		helperFunctions::getLastPointShape(edge2),
		projected,
		buffer
	);
}

bool helperFunctions::isOverlappingCompletely(const SurfaceGroup& evalFace, const SurfaceGroup& otherFace) {
	std::vector<std::shared_ptr<EvaluationPoint>> evalGrid = evalFace.getPointGrid();
	std::vector<std::shared_ptr<EvaluationPoint>> otherGrid = otherFace.getPointGrid();
	if (evalGrid.size() != otherGrid.size()) { return false; }

	double precision = SettingsCollection::getInstance().precision();
	for (size_t i = 0; i < evalGrid.size(); i++)
	{
		if (!evalGrid[i]->getPoint().IsEqual(otherGrid[i]->getPoint(), precision)) { return false; }
	}
	return true;
}

std::vector<nlohmann::json> helperFunctions::collectPropertyValues(std::string objectId, IfcParse::IfcFile* ifcFile)
{
	std::vector<nlohmann::json> attributesList;

	IfcSchema::IfcRelDefinesByProperties::list::ptr relDefList = ifcFile->instances_by_type <IfcSchema::IfcRelDefinesByProperties>();

	for (auto reldefIt = relDefList->begin(); reldefIt != relDefList->end(); reldefIt++)
	{
		IfcSchema::IfcRelDefinesByProperties* relDefItem = *reldefIt;
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcObjectDefinition::list::ptr relatedObjectList = relDefItem->RelatedObjects();
#else
		IfcSchema::IfcObject::list::ptr relatedObjectList = relDefItem->RelatedObjects();
#endif


		bool match = false;
		for (auto objectIt = relatedObjectList->begin(); objectIt != relatedObjectList->end(); objectIt++)
		{
			if ((*objectIt)->GlobalId() != objectId) { continue; }
			match = true;
		}

		if (!match) { continue; }
#if defined(USE_IFC4) || defined(USE_IFC4x3)
		IfcSchema::IfcPropertySetDefinitionSelect* propertyDef = relDefItem->RelatingPropertyDefinition();
#else
		IfcSchema::IfcPropertySetDefinition* propertyDef = relDefItem->RelatingPropertyDefinition();
#endif

		if (propertyDef->data().type()->name() != "IfcPropertySet") { continue; }
		IfcSchema::IfcPropertySet* propertySet = relDefItem->RelatingPropertyDefinition()->as<IfcSchema::IfcPropertySet>();
		IfcSchema::IfcProperty::list::ptr propertyList = propertySet->HasProperties();
		for (auto propertyIt = propertyList->begin(); propertyIt != propertyList->end(); propertyIt++)
		{
			IfcSchema::IfcPropertySingleValue* propertyItem = (*propertyIt)->as<IfcSchema::IfcPropertySingleValue>();

			IfcSchema::IfcValue* ifcValue = propertyItem->NominalValue();

			std::string propertyIdName = ifcValue->data().type()->name();

			if (propertyIdName == "IfcIdentifier")
			{
				IfcSchema::IfcIdentifier* propertyValueContainer = ifcValue->as<IfcSchema::IfcIdentifier>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator std::string();
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcText")
			{
				IfcSchema::IfcText* propertyValueContainer = ifcValue->as<IfcSchema::IfcText>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator std::string();
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcLabel")
			{
				IfcSchema::IfcLabel* propertyValueContainer = ifcValue->as<IfcSchema::IfcLabel>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator std::string();
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcLengthMeasure")
			{
				IfcSchema::IfcLengthMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcLengthMeasure>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = {
					{CJObjectEnum::getString(CJObjectID::jsonValue), propertyValueContainer->operator double() },
					{CJObjectEnum::getString(CJObjectID::jsonUom) , UnitStringEnum::getString(UnitStringID::meter) } //TODO: update to unit?
				};
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcAreaMeasure")
			{
				IfcSchema::IfcAreaMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcAreaMeasure>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = { 
					{CJObjectEnum::getString(CJObjectID::jsonValue), propertyValueContainer->operator double() },
					{CJObjectEnum::getString(CJObjectID::jsonUom) , UnitStringEnum::getString(UnitStringID::sqrMeter) } //TODO: update to set unit?
				};
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcReal")
			{
				IfcSchema::IfcReal* propertyValueContainer = ifcValue->as<IfcSchema::IfcReal>();				
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator double();
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcPowerMeasure")
			{
				IfcSchema::IfcPowerMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcPowerMeasure>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator double();
				attributesList.emplace_back(attributeItem);
			}
			else if (propertyIdName == "IfcThermalTransmittanceMeasure")
			{
				IfcSchema::IfcThermalTransmittanceMeasure* propertyValueContainer = ifcValue->as<IfcSchema::IfcThermalTransmittanceMeasure>();
				nlohmann::json attributeItem;
				attributeItem[propertyItem->Name()] = propertyValueContainer->operator double();
				attributesList.emplace_back(attributeItem);
			}
			else
			{
				ErrorCollection::getInstance().addError(errorID::missingProptery, propertyIdName);
			}
		}
	}
	return attributesList;
}
