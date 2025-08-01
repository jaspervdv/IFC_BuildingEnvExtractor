#include "helper.h"
#include "settingsCollection.h"
#include "stringManager.h"
#include "errorCollection.h"
#include "DebugUtils.h"

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
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangle.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <TopExp.hxx>
#include <BRep_Tool.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Wire.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Standard_Type.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <ProjLib_ProjectOnPlane.hxx>
#include <GeomAPI.hxx>
#include <ShapeFix_Edge.hxx>
#include <ShapeFix_Wire.hxx>
#include <BRepTools_ReShape.hxx>
#include <GCPnts_AbscissaPoint.hxx>

#include <Prs3d_ShapeTool.hxx>

#include <Geom_TrimmedCurve.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>

#include <gp_Quaternion.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

template double helperFunctions::getLowestZ<TopoDS_Face>(const TopoDS_Face& shape);
template double helperFunctions::getLowestZ<TopoDS_Shell>(const TopoDS_Shell& shape);
template double helperFunctions::getLowestZ<TopoDS_Solid>(const TopoDS_Solid& shape);
template double helperFunctions::getLowestZ<TopoDS_Shape>(const TopoDS_Shape& shape);

template double helperFunctions::getAverageZ<TopoDS_Face>(const TopoDS_Face& shape);
template double helperFunctions::getAverageZ<TopoDS_Shell>(const TopoDS_Shell& shape);
template double helperFunctions::getAverageZ<TopoDS_Solid>(const TopoDS_Solid& shape);
template double helperFunctions::getAverageZ<TopoDS_Shape>(const TopoDS_Shape& shape);

template double helperFunctions::getHighestZ<TopoDS_Face>(const TopoDS_Face& shape);
template double helperFunctions::getHighestZ<TopoDS_Shell>(const TopoDS_Shell& shape);
template double helperFunctions::getHighestZ<TopoDS_Solid>(const TopoDS_Solid& shape);
template double helperFunctions::getHighestZ<TopoDS_Shape>(const TopoDS_Shape& shape);

template double helperFunctions::getHighestZ<TopoDS_Face>(const std::vector<TopoDS_Face>& faceList);
template double helperFunctions::getHighestZ<TopoDS_Shell>(const std::vector<TopoDS_Shell>& faceList);
template double helperFunctions::getHighestZ<TopoDS_Solid>(const std::vector<TopoDS_Solid>& faceList);
template double helperFunctions::getHighestZ<TopoDS_Shape>(const std::vector<TopoDS_Shape>& faceList);

template void helperFunctions::bBoxDiagonal<TopoDS_Face>(const std::vector<TopoDS_Face>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Shell>(const std::vector<TopoDS_Shell>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Solid>(const std::vector<TopoDS_Solid>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Shape>(const std::vector<TopoDS_Shape>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);

template void helperFunctions::bBoxDiagonal<TopoDS_Face>(const TopoDS_Face& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Shell>(const TopoDS_Shell& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Solid>(const TopoDS_Solid& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);
template void helperFunctions::bBoxDiagonal<TopoDS_Shape>(const TopoDS_Shape& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle);

template gp_Vec helperFunctions::computeFaceNormal(const TopoDS_Face& theFace);
template gp_Vec helperFunctions::computeFaceNormal(const TopoDS_Wire& theFace);

template void helperFunctions::writeToSTEP<TopoDS_Face>(const std::vector<TopoDS_Face>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Shell>(const std::vector<TopoDS_Shell>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Solid>(const std::vector<TopoDS_Solid>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Shape>(const std::vector<TopoDS_Shape>& theShapeList, const std::string& targetPath);

template void helperFunctions::writeToSTEP<TopoDS_Face>(const std::vector<std::vector<TopoDS_Face>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Shell>(const std::vector<std::vector<TopoDS_Shell>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Solid>(const std::vector<std::vector<TopoDS_Solid>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToSTEP<TopoDS_Shape>(const std::vector<std::vector<TopoDS_Shape>>& theShapeList, const std::string& targetPath);

template void helperFunctions::writeToOBJ<TopoDS_Face>(const TopoDS_Face& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shell>(const TopoDS_Shell& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Solid>(const TopoDS_Solid& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shape>(const TopoDS_Shape& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Compound>(const TopoDS_Compound& theShapeList, const std::string& targetPath);

template void helperFunctions::writeToOBJ<TopoDS_Face>(const std::vector<TopoDS_Face>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shell>(const std::vector<TopoDS_Shell>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Solid>(const std::vector<TopoDS_Solid>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shape>(const std::vector<TopoDS_Shape>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Compound>(const std::vector<TopoDS_Compound>& theShapeList, const std::string& targetPath);

template void helperFunctions::writeToOBJ<TopoDS_Face>(const std::vector<std::vector<TopoDS_Face>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shell>(const std::vector<std::vector<TopoDS_Shell>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Solid>(const std::vector<std::vector<TopoDS_Solid>>& theShapeList, const std::string& targetPath);
template void helperFunctions::writeToOBJ<TopoDS_Shape>(const std::vector<std::vector<TopoDS_Shape>>& theShapeList, const std::string& targetPath);

struct gp_XYZ_Hash {
	std::size_t operator()(const gp_XYZ& p) const {
		auto round = [](double theVal) -> long long {
			return static_cast<long long>(std::round(theVal * SettingsCollection::getInstance().spatialTolerance()));
		};
		std::size_t hx = std::hash<long long>()(round(p.X()));
		std::size_t hy = std::hash<long long>()(round(p.Y()));
		std::size_t hz = std::hash<long long>()(round(p.Z()));
		return hx ^ (hy << 1) ^ (hz << 2);
	}
};

struct gp_XYZ_Equal {
	bool operator()(const gp_XYZ& a, const gp_XYZ& b) const {
		return a.IsEqual(b, SettingsCollection::getInstance().spatialTolerance());
	}
};

BoostPoint3D helperFunctions::Point3DOTB(const gp_Pnt& oP) {
	return BoostPoint3D(oP.X(), oP.Y(), oP.Z());
}

gp_Pnt helperFunctions::Point3DBTO(const BoostPoint3D& oP) {
	return gp_Pnt(bg::get<0>(oP), bg::get<1>(oP), bg::get<2>(oP));
}

std::vector<gp_Pnt> helperFunctions::getUniquePoints(const std::vector<gp_Pnt>& pointList)  //TODO: check where used
{
	std::cout << pointList.size() << std::endl;
	std::vector<gp_Pnt> uniquePoints;
	for (const gp_Pnt& currentPoint : pointList)
	{
		bool dub = false;
		for (const gp_Pnt& uniquePoint : uniquePoints)
		{
			if (currentPoint.IsEqual(uniquePoint, 0.001))
			{
				dub = true;
				break;
			}
		}
		if (!dub)
		{
			uniquePoints.emplace_back(currentPoint);
		}
	}
	return uniquePoints;
}

std::vector<gp_Pnt> helperFunctions::getUniquePoints(const TopoDS_Shape& inputShape) //TODO: check for triangles
{
	std::vector<gp_Pnt> uniquePoints;
	TopTools_IndexedMapOfShape vertexMap;
	TopExp::MapShapes(inputShape, TopAbs_VERTEX, vertexMap);

	for (int i = 1; i <= vertexMap.Extent(); ++i) {
		TopoDS_Vertex vertex = TopoDS::Vertex(vertexMap(i));
		uniquePoints.emplace_back(BRep_Tool::Pnt(vertex));
	}
	return uniquePoints;
}

std::vector<gp_Pnt> helperFunctions::getPoints(const TopoDS_Shape& shape)
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

int helperFunctions::getPointCount(const TopoDS_Shape& inputShape)
{
	TopTools_IndexedMapOfShape vertexMap;
	TopExp::MapShapes(inputShape, TopAbs_VERTEX, vertexMap);
	return vertexMap.Extent();
}

gp_Pnt helperFunctions::rotatePointWorld(const gp_Pnt& p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

BoostPoint3D helperFunctions::rotatePointWorld(const BoostPoint3D& p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

gp_Pnt helperFunctions::rotatePointPoint(const gp_Pnt& p, const gp_Pnt& anchorP, const double& angle)
{
	gp_Pnt translatedP = p.Translated(gp_Vec(-anchorP.X(), -anchorP.Y(), -anchorP.Z()));
	gp_Pnt rotatedP = helperFunctions::rotatePointWorld(translatedP, angle);
	return rotatedP.Translated(gp_Vec(anchorP.X(), anchorP.Y(), anchorP.Z()));
}

std::vector<gp_Pnt> helperFunctions::getPointGridOnSurface(const TopoDS_Face& theface, const double& resolution)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precision = settingsCollection.spatialTolerance();
	int minSurfacePoints = settingsCollection.minGridPointCount(); 

	Handle(Geom_Surface) surface = BRep_Tool::Surface(theface);

	// greate points on grid over surface
	// get the uv bounds to create a point grid on the surface
	Standard_Real uMin, uMax, vMin, vMax;
	BRepTools::UVBounds(theface, BRepTools::OuterWire(theface), uMin, uMax, vMin, vMax);

	int numUPoints = static_cast<int>(ceil(abs(uMax - uMin) / resolution));
	int numVPoints = static_cast<int>(ceil(abs(vMax - vMin) / resolution));

	// set num of points if min/max rule is not met
	if (numUPoints <= minSurfacePoints) { numUPoints = minSurfacePoints; }
	if (numVPoints <= minSurfacePoints) { numVPoints = minSurfacePoints; }

	double uStep = (uMax - uMin) / (numUPoints - 1);
	double vStep = (vMax - vMin) / (numVPoints - 1);

	std::vector<gp_Pnt> gridPointList;

	if (getPointCount(theface) == 3)
	{
		double x = 0;
		double y = 0;
		double z = 0;

		for (TopExp_Explorer expl(theface, TopAbs_VERTEX); expl.More(); expl.Next())
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

		gridPointList.emplace_back(centerPoint);

		double smallestAngle = helperFunctions::computeSmallestAngle(theface);

		if (smallestAngle < settingsCollection.thinTriangleAngle()) //10 degrees
		{
			std::vector<gp_Pnt> uniquePointList = helperFunctions::getUniquePoints(theface);

			for (size_t i = 0; i < uniquePointList.size(); i++)
			{
				gp_Pnt legPoint = uniquePointList[i];

				if (numUPoints == 1) { numUPoints = 2; } //TODO: finetune

				gp_Vec translationVec = gp_Vec(
					(legPoint.X() - centerPoint.X()) / numUPoints,
					(legPoint.Y() - centerPoint.Y()) / numUPoints,
					(legPoint.Z() - centerPoint.Z()) / numUPoints
				);

				for (int j = 0; j < numUPoints; j++)
				{
					gridPointList.emplace_back(centerPoint.Translated(translationVec * j));
				}

			}
			return gridPointList;
		}
	}

	// create grid
	int currentStep = 0;
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

			bool notOnWire = true;
			for (TopExp_Explorer expl(theface, TopAbs_WIRE); expl.More(); expl.Next())
			{
				TopoDS_Wire currentWire = TopoDS::Wire(expl.Current());
				if (helperFunctions::pointOnWire(currentWire, point, precision * 10))
				{
					notOnWire = false;
					break;
				}
			}
			if (!notOnWire)
			{
				continue;
			}
			gridPointList.emplace_back(point);
		}
	}
	return gridPointList;
}

std::vector<gp_Pnt> helperFunctions::getPointGridOnWire(const TopoDS_Face& theface, const double& resolution)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();
	double precision = settingsCollection.spatialTolerance();

	//if (helperFunctions::computeArea(theface) < 0.001) { return {}; }

	std::vector<gp_Pnt> wirePointList;

	TopoDS_Face faceLocalCopy = theface;
	for (TopExp_Explorer exp(theface, TopAbs_EDGE); exp.More(); exp.Next()) {
		const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
		Standard_Real first, last;
		Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
		if (curve.IsNull()) {
			return {};  // Skip degenerated edges
		}
		if (!curve->IsKind(STANDARD_TYPE(Geom_Line))) {

			std::vector<TopoDS_Face> tesselatedFace = TessellateFace(theface);
			if (tesselatedFace.size() != 1)
			{
				return {};
			}
			faceLocalCopy = tesselatedFace[0];
		}
	}

	SettingsCollection::getInstance().getWireOffsetterMutex()->lock();
	BRepOffsetAPI_MakeOffset offsetter(BRepTools::OuterWire(faceLocalCopy), GeomAbs_Intersection);
	offsetter.Perform(-precision * 10);
	SettingsCollection::getInstance().getWireOffsetterMutex()->unlock();

	if (!offsetter.IsDone()) { return {}; }
	const TopoDS_Shape offsettedFace = offsetter.Shape();
	if (offsettedFace.IsNull()) { return {}; }

	for (TopExp_Explorer expl(offsetter.Shape(), TopAbs_EDGE); expl.More(); expl.Next())
	{
		const TopoDS_Edge& currentEdge = TopoDS::Edge(expl.Current());
		BRepAdaptor_Curve curveAdaptor(currentEdge);

		double uStart = curveAdaptor.Curve().FirstParameter();
		double uEnd = curveAdaptor.Curve().LastParameter();
		int numUPoints = static_cast<int>(ceil(abs(uStart - uEnd)) / resolution);

		if (numUPoints < 2) { numUPoints = 2; }
		else if (numUPoints > 10) { numUPoints = 10; }

		double uStep = abs(uStart - uEnd) / (numUPoints - 1);
		bool t = false;
		for (double u = uStart; u < uEnd; u += uStep) {
			gp_Pnt point;
			curveAdaptor.D0(u, point);
			wirePointList.emplace_back(point);
			t = true;
		}
	}
	return wirePointList;
}

bool helperFunctions::pointIsSame(const BoostPoint3D& lp, const BoostPoint3D& rp)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	if (abs(lp.get<0>() - rp.get<0>()) > precision ) { return false; }
	if (abs(lp.get<1>() - rp.get<1>()) > precision ) { return false; }
	if (abs(lp.get<2>() - rp.get<2>()) > precision ) { return false; }
	return true;

}


template<typename T>
void helperFunctions::bBoxDiagonal(const std::vector<T>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle)
{
	for (const T& theShape : theShapeList) { bBoxDiagonal(theShape, lllPoint, urrPoint, buffer, angle, secondAngle); }
}

template<typename T>
void helperFunctions::bBoxDiagonal(const T& theShape, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle)
{
	if (lllPoint->Distance(*urrPoint) < SettingsCollection::getInstance().spatialTolerance() && 
		lllPoint->Distance(gp_Pnt(0,0,0)) < SettingsCollection::getInstance().spatialTolerance() &&
		urrPoint->Distance(gp_Pnt(0, 0, 0)) < SettingsCollection::getInstance().spatialTolerance())
	{
		lllPoint->SetCoord(9999999, 9999999, 9999999);
		urrPoint->SetCoord(-9999999, -9999999, -9999999);
	}

	//TODO: implement rotation
	for (TopExp_Explorer expl(theShape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt point = helperFunctions::rotatePointWorld(BRep_Tool::Pnt(vertex), angle).Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -secondAngle);

		if (point.X() > urrPoint->X()) { urrPoint->SetX(point.X()); }
		if (point.Y() > urrPoint->Y()) { urrPoint->SetY(point.Y()); }
		if (point.Z() > urrPoint->Z()) { urrPoint->SetZ(point.Z()); }

		if (point.X() < lllPoint->X()) { lllPoint->SetX(point.X()); }
		if (point.Y() < lllPoint->Y()) { lllPoint->SetY(point.Y()); }
		if (point.Z() < lllPoint->Z()) { lllPoint->SetZ(point.Z()); }
	}

	applyBuffer(lllPoint, urrPoint, buffer);
	return;
}

bool helperFunctions::bBoxDiagonal(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle)
{
	*lllPoint = helperFunctions::rotatePointWorld(pointList[0], angle).Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -secondAngle);
	*urrPoint = helperFunctions::rotatePointWorld(pointList[0], angle).Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -secondAngle);

	for (size_t i = 1; i < pointList.size(); i++)
	{
		gp_Pnt point = helperFunctions::rotatePointWorld(pointList[i], angle).Rotated(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -secondAngle);

		if (point.X() < lllPoint->X()) { lllPoint->SetX(point.X()); }
		if (point.Y() < lllPoint->Y()) { lllPoint->SetY(point.Y()); }
		if (point.Z() < lllPoint->Z()) { lllPoint->SetZ(point.Z()); }

		if (point.X() > urrPoint->X()) { urrPoint->SetX(point.X()); }
		if (point.Y() > urrPoint->Y()) { urrPoint->SetY(point.Y()); }
		if (point.Z() > urrPoint->Z()) { urrPoint->SetZ(point.Z()); }
	}
	if (lllPoint->IsEqual(*urrPoint, 0.01)) { return false; }

	applyBuffer(lllPoint, urrPoint, buffer);
	return true;
}

void helperFunctions::bBoxOrientated(const std::vector<gp_Pnt>& pointList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, double* rotationAngle, const double buffer)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance();

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	int maxIt = 15; //TODO: maybe add this to the settings collection
	double smallestDistance = lllPoint->Distance(*urrPoint);

	for (size_t i = 0; i < maxIt; i++)
	{
		std::tuple<gp_Pnt, gp_Pnt, double> left;
		std::tuple<gp_Pnt, gp_Pnt, double> right;

		gp_Pnt leftLllPoint;
		gp_Pnt leftUrrPoint;
		gp_Pnt rghtLllPoint;
		gp_Pnt rghtUrrPoint;

		bBoxDiagonal(pointList, &leftLllPoint, &leftUrrPoint, 0, *rotationAngle - angle);
		bBoxDiagonal(pointList, &rghtLllPoint, &rghtUrrPoint, 0, *rotationAngle + angle);

		double leftDistance = leftLllPoint.Distance(leftUrrPoint);
		double rghtDistance = rghtLllPoint.Distance(rghtUrrPoint);

		if (leftDistance > rghtDistance && smallestDistance > rghtDistance)
		{
			*rotationAngle = *rotationAngle + angle;
			smallestDistance = rghtDistance;
			*lllPoint = rghtLllPoint;
			*urrPoint = rghtUrrPoint;
		}
		else if (smallestDistance > leftDistance)
		{
			*rotationAngle = *rotationAngle - angle;
			smallestDistance = leftDistance;
			*lllPoint = leftLllPoint;
			*urrPoint = leftUrrPoint;
		}
		angle = angle / 2;
	}
	applyBuffer(lllPoint, urrPoint, buffer);
	return;
}

bg::model::box <BoostPoint3D> helperFunctions::createBBox(const TopoDS_Shape& shape, double buffer)
{
	return createBBox(std::vector<TopoDS_Shape>{shape}, buffer);
}

bg::model::box <BoostPoint3D> helperFunctions::createBBox(const std::vector<TopoDS_Shape>& shape, double buffer) 
{
	Bnd_Box boundingBox;
	for (size_t i = 0; i < shape.size(); i++)
	{
		BRepBndLib::Add(shape[i], boundingBox);
	}

	if (boundingBox.IsVoid()) { return {}; }

	Standard_Real minX, minY, minZ, maxX, maxY, maxZ;
	boundingBox.Get(minX, minY, minZ, maxX, maxY, maxZ);

	return  bg::model::box < BoostPoint3D >(
		BoostPoint3D(minX - buffer, minY - buffer, minZ - buffer),
		BoostPoint3D(maxX + buffer, maxY + buffer, maxZ + buffer)
		);
}

bg::model::box <BoostPoint3D> helperFunctions::createBBox(const std::vector<gp_Pnt>& pointList, double buffer)
{
	gp_Pnt lll;
	gp_Pnt urr;

	bBoxDiagonal(pointList, &lll, &urr, 0);
	return  bg::model::box < BoostPoint3D >(
		BoostPoint3D(lll.X() - buffer, lll.Y() - buffer, lll.Z() - buffer),
		BoostPoint3D(urr.X() + buffer, urr.Y() + buffer, urr.Z() + buffer)
		);
}

bg::model::box <BoostPoint3D>  helperFunctions::createBBox(const gp_Pnt& p1, const gp_Pnt& p2, double buffer) 
{
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

	BoostPoint3D boostlllPoint = BoostPoint3D(lll.X() - buffer, lll.Y() - buffer, lll.Z() - buffer);
	BoostPoint3D boosturrPoint = BoostPoint3D(urr.X() + buffer, urr.Y() + buffer, urr.Z() + buffer);
	bg::model::box <BoostPoint3D> box = bg::model::box < BoostPoint3D >(boostlllPoint, boosturrPoint);

	return box;
}

TopoDS_Shape helperFunctions::createBBOXOCCT(const gp_Pnt& lll, const gp_Pnt& urr, double buffer, double horizontalAngle, double verticalAngle) 
{
	if (abs(urr.X() - lll.X()) < SettingsCollection::getInstance().spatialTolerance()) { return TopoDS_Solid(); }
	if (abs(urr.Y() - lll.Y()) < SettingsCollection::getInstance().spatialTolerance()) { return TopoDS_Solid(); }
	if (abs(urr.Z() - lll.Z()) < SettingsCollection::getInstance().spatialTolerance()) { return TopoDS_Solid(); }
	
	gp_Ax1 vertRotation(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));

	BRep_Builder brepBuilder;
	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidbox;
	brepBuilder.MakeSolid(solidbox);

	gp_Pnt p0(helperFunctions::rotatePointWorld(lll.Rotated(vertRotation, verticalAngle), -horizontalAngle));
	gp_Pnt p1 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);
	gp_Pnt p2 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);
	gp_Pnt p3 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);

	gp_Pnt p4(helperFunctions::rotatePointWorld(urr.Rotated(vertRotation, verticalAngle), -horizontalAngle));
	gp_Pnt p5 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);
	gp_Pnt p6 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);
	gp_Pnt p7 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()).Rotated(vertRotation, verticalAngle), -horizontalAngle);

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

	BRepBuilderAPI_Sewing brepSewer;
	for (size_t k = 0; k < faceList.size(); k++) { brepSewer.Add(faceList[k]); }
	brepSewer.Perform();
	brepBuilder.Add(solidbox, brepSewer.SewedShape());

	return solidbox;
}

TopoDS_Shape helperFunctions::boxSimplefyShape(const TopoDS_Shape& shape)
{
	double angularTolerance = SettingsCollection::getInstance().angularTolerance();
	double precision = SettingsCollection::getInstance().spatialTolerance();
	// get the vectors of the shape
	std::vector<gp_Pnt> pointList = getPoints(shape);
	gp_Vec hVector = getShapedir(pointList, true);
	gp_Vec vVector = getShapedir(pointList, false);

	if (hVector.Magnitude()  < precision) { hVector = gp_Vec(1, 0, 0); }
	if (vVector.Magnitude()  < precision) { vVector = gp_Vec(0, 0, 1); }

	// compute rotation around z axis
	gp_Pnt p1 = gp_Pnt(0, 0, 0);
	gp_Pnt p2 = p1.Translated(hVector);

	double angleFlat = 0;
	// apply rotation around z axis if required
	if (abs(p1.Y() - p2.Y()) > angularTolerance)
	{
		double os = abs(p1.Y() - p2.Y()) / p1.Distance(p2);
		angleFlat = asin(os);

		gp_Pnt tempP = helperFunctions::rotatePointPoint(p2, p1, angleFlat);

		// mirror the rotation if incorrect
		if (Abs(p1.X() - tempP.X()) > angularTolerance &&
			Abs(p1.Y() - tempP.Y()) > angularTolerance)
		{
			angleFlat = -angleFlat;
		}
	}

	// rotate the box around the x axis to correctly place the roation axis for the x rotation
	gp_Pnt p3 = gp_Pnt(0, 0, 0);
	gp_Pnt p4 = helperFunctions::rotatePointPoint(p3.Translated(vVector), p3, angleFlat);
	if (abs(p3.X() - p4.X()) < angularTolerance)
	{
		p3 = helperFunctions::rotatePointWorld(p3, M_PI / 2.0);
		p4 = helperFunctions::rotatePointWorld(p4, M_PI / 2.0);
		angleFlat += M_PI / 2.0;
	}

	// compute vertical rotation
	double angleVert = acos(abs(p4.Z() - p3.Z()) / p3.Distance(p4));
	p4.Rotate(gp_Ax1(p3, gp_Vec(0,1,0)), angleVert);
	// mirror the rotation if incorrect
	if (abs(p3.X() - p4.X()) < angularTolerance)
	{
		angleVert = -angleVert;
		p4.Rotate(gp_Ax1(p3, gp_Vec(0, 1, 0)), 2 * angleVert);
	}

	gp_Pnt lllPoint;
	gp_Pnt urrPoint;
	helperFunctions::bBoxDiagonal(pointList, &lllPoint, &urrPoint, 0, angleFlat, angleVert);
	if (lllPoint.IsEqual(urrPoint, SettingsCollection::getInstance().spatialTolerance())) { return TopoDS_Shape(); }
	TopoDS_Shape boxShape = helperFunctions::createBBOXOCCT(lllPoint, urrPoint, 0.0, angleFlat, angleVert);
	helperFunctions::triangulateShape(boxShape);
	return boxShape;
}

void helperFunctions::applyBuffer(gp_Pnt* lllPoint, gp_Pnt* urrPoint, double buffer)
{
	urrPoint->SetX(urrPoint->X() + buffer);
	urrPoint->SetY(urrPoint->Y() + buffer);
	urrPoint->SetZ(urrPoint->Z() + buffer);
	lllPoint->SetX(lllPoint->X() - buffer);
	lllPoint->SetY(lllPoint->Y() - buffer);
	lllPoint->SetZ(lllPoint->Z() - buffer);
	return;
}


template<typename T>
double helperFunctions::getLowestZ(const T& shape)
{
	double lowestZ = 999999999;
	for (TopExp_Explorer expl(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() < lowestZ) { lowestZ = p.Z(); }
	}
	return lowestZ;
}

template<typename T>
double helperFunctions::getHighestZ(const T& shape)
{
	double highestZ = -999999999;
	for (TopExp_Explorer expl(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		if (p.Z() > highestZ) { highestZ = p.Z(); }
	}
	return highestZ;
}

template<typename T>
double helperFunctions::getHighestZ(const std::vector<T>& faceList) {
	double maxHeight = -999999;

	for (const T& currentFace : faceList)
	{
		double currentHeight = getHighestZ(currentFace);
		if (currentHeight > maxHeight) { maxHeight = currentHeight; }
	}
	return maxHeight;
}

template<typename T>
double helperFunctions::getAverageZ(const T& shape) {
	double totalZ = 0;
	int pCount = 0;
	for (TopExp_Explorer expl(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		totalZ += p.Z();
		pCount++;
	}
	return totalZ / pCount;
}


gp_Pnt helperFunctions::getTriangleCenter(const opencascade::handle<Poly_Triangulation>& mesh, const Poly_Triangle& theTriangle, const TopLoc_Location& loc) {
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

std::optional<gp_Pnt> helperFunctions::getPointOnFace(const TopoDS_Face& theFace) 
{
	triangulateShape(theFace);
	TopLoc_Location loc;
	opencascade::handle<Poly_Triangulation> mesh = BRep_Tool::Triangulation(theFace, loc);

	if (mesh.IsNull()) { return std::nullopt; }
	for (int i = 1; i <= mesh.get()->NbTriangles(); i++) 
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);
		gp_Pnt point = getTriangleCenter(mesh, theTriangle, loc);

		bool isEdge = false;
		for (TopExp_Explorer expl(theFace, TopAbs_WIRE); expl.More(); expl.Next())
		{
			TopoDS_Wire currentWire = TopoDS::Wire(expl.Current());

			if (helperFunctions::pointOnWire(currentWire, point))
			{
				isEdge = true;
				break;
			}
		}

		if (isEdge) { continue; }
		return point;
	}
	return std::nullopt;
}

std::vector<gp_Pnt> helperFunctions::getPointListOnFace(const TopoDS_Face& theFace)
{
	triangulateShape(theFace);

	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);
	if (mesh.IsNull()) { return {}; }
	
	std::vector<gp_Pnt> pointList;
	for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);
		pointList.emplace_back(getTriangleCenter(mesh, theTriangle, loc));
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

bool helperFunctions::pointOnShape(const TopoDS_Shape& shape, const gp_Pnt& thePoint, double precision)
{
	if (precision == 0.0) { precision = SettingsCollection::getInstance().spatialTolerance(); }

	for (TopExp_Explorer faceExpl(shape, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());
		if (pointOnFace(currentFace, thePoint, precision)) { return true; }
	}
	return false;
}

bool helperFunctions::pointOnFace(const TopoDS_Face& theFace, const gp_Pnt& thePoint, double precision)
{
	if (precision == 0.0) { precision = SettingsCollection::getInstance().spatialTolerance(); }

	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	if (mesh.IsNull())
	{
		helperFunctions::triangulateShape(theFace);
		mesh = BRep_Tool::Triangulation(theFace, loc);
	}
	if (mesh.IsNull()) { return false; }

	for (int j = 1; j <= mesh.get()->NbTriangles(); j++) //TODO: if large num indx?
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		double baseArea = computeArea(p1, p2, p3);

		double area1 = computeArea(thePoint, p1, p2);
		if (area1 - baseArea > precision) { continue; }

		double area2 = computeArea(thePoint, p1, p3);
		if (area2 - baseArea > precision) { continue; }

		double area3 = computeArea(thePoint, p2, p3);
		if (area3 - baseArea > precision) { continue; }

		if (abs(baseArea - (area1 + area2 + area3)) > precision * baseArea) { continue; }
		return true;
	}
	return false;
}

bool helperFunctions::pointOnFace(const std::vector<TopoDS_Face>& theFace, const gp_Pnt& thePoint, double precision)
{
	for (const TopoDS_Face& currentFace : theFace)
	{
		if (pointOnFace(currentFace, thePoint, precision))
		{
			return true;
		}
	}
	return false;
}

bool helperFunctions::pointOnWire(const TopoDS_Face& theFace, const gp_Pnt& thePoint, double precision)
{
	for (TopExp_Explorer expl(theFace, TopAbs_WIRE); expl.More(); expl.Next())
	{
		TopoDS_Wire currentWire = TopoDS::Wire(expl.Current());
		if (pointOnWire(currentWire, thePoint, precision))
		{
			return true;
		}
	}
	return false;
}

bool helperFunctions::pointOnWire(const TopoDS_Wire& theWire, const gp_Pnt& thePoint, double precision)
{
	for (TopExp_Explorer currentExpl(theWire, TopAbs_EDGE); currentExpl.More(); currentExpl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(currentExpl.Current());
		if (pointOnEdge(currentEdge, thePoint, precision)) { return true; }
	}
	return false;
}

bool helperFunctions::pointOnEdge(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, double precision)
{
	if (precision == 0.0) { precision = SettingsCollection::getInstance().spatialTolerance(); }

	gp_Pnt p1 = getFirstPointShape(theEdge);
	gp_Pnt p2 = getLastPointShape(theEdge);

	double baseDistance = p1.Distance(p2);

	if (abs(baseDistance - (p1.Distance(thePoint) + p2.Distance(thePoint))) < precision) { return true; }
	return false;
}


gp_Vec helperFunctions::computeEdgeDir(const TopoDS_Edge& theEdge) //TODO: make this smarter
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	gp_Pnt startpoint = getFirstPointShape(theEdge);
	gp_Pnt endpoint = getLastPointShape(theEdge);

	if (startpoint.IsEqual(endpoint, precision)) { return gp_Vec(0, 0, 0); }
	return gp_Vec(startpoint, endpoint).Normalized();
}

template<typename T>
gp_Vec helperFunctions::computeFaceNormal(const T& theFace) //TODO: check if triangle based would be usefull 
{
	if (theFace.IsNull()) { return gp_Vec(0, 0, 0); }

	double precision = SettingsCollection::getInstance().spatialTolerance();

	gp_Vec vec1;
	gp_Vec vec2;

	TopExp_Explorer edgeExpl(theFace, TopAbs_EDGE);

	bool found = false;
	for (edgeExpl; edgeExpl.More(); edgeExpl.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(edgeExpl.Current());
		vec1 = computeEdgeDir(edge);
		if (vec1.Magnitude() < precision) { continue; }
		found = true;
		break;
	}
	if (!found) { return gp_Vec(0, 0, 0); }

	for (edgeExpl; edgeExpl.More(); edgeExpl.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(edgeExpl.Current());
		vec2 = computeEdgeDir(edge);
		if (vec2.Magnitude() < precision) { continue; }
		if (vec2.IsParallel(vec1, precision)) { continue; }

		gp_Vec normal = vec1.Crossed(vec2);
		normal.Normalize();
		return normal;
	}
	return gp_Vec(0, 0, 0);
}

TopoDS_Wire helperFunctions::reversedWire(const TopoDS_Wire& mainWire) {  //TODO: check where used
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

double helperFunctions::computeLargestAngle(const TopoDS_Face& theFace)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	std::vector<gp_Pnt> pointList = getUniquePoints(theFace);
	if (pointList.size() != 3) { std::cout << "largest angle only works for triangles\n"; }
	
	gp_Vec v01(pointList[0], pointList[1]);
	gp_Vec v10 = v01.Reversed();
	gp_Vec v12(pointList[1], pointList[2]);
	gp_Vec v21 = v12.Reversed();
	gp_Vec v20(pointList[2], pointList[0]);
	gp_Vec v02 = v20.Reversed();

	double angle0 = v20.Angle(v10);
	double angle1 = v01.Angle(v21);
	double angle2 = v02.Angle(v12);

	return std::max({ angle0, angle1, angle2 });
}

double helperFunctions::computeSmallestAngle(const TopoDS_Face& theFace)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	std::vector<gp_Pnt> pointList = getUniquePoints(theFace);
	if (pointList.size() != 3) { std::cout << "smallest angle only works for triangles\n"; }

	gp_Vec v01(pointList[0], pointList[1]);
	gp_Vec v10 = v01.Reversed();
	gp_Vec v12(pointList[1], pointList[2]);
	gp_Vec v21 = v12.Reversed();
	gp_Vec v20(pointList[2], pointList[0]);
	gp_Vec v02 = v20.Reversed();

	double angle0 = v20.Angle(v10);
	double angle1 = v01.Angle(v21);
	double angle2 = v02.Angle(v12);

	return std::min({ angle0, angle1, angle2 });
}

gp_Vec helperFunctions::getShapedir(const std::vector<gp_Pnt>& pointList, bool isHorizontal)
{
	std::vector<std::pair<gp_Vec, int>> vecCountMap;
	double precision = SettingsCollection::getInstance().spatialTolerance();

	// compute median lenght of all edges
	std::vector<double> distances;
	for (size_t i = 0; i < pointList.size(); i += 2)
	{
		gp_Pnt p1 = pointList[i];
		gp_Pnt p2 = pointList[i + 1];

		distances.emplace_back(p1.Distance(p2));
	}
	std::sort(distances.begin(), distances.end());

	double medianDistance;
	if (distances.size() % 2 != 0)
	{
		medianDistance = distances[distances.size() / 2];
	}
	else
	{
		medianDistance = distances[(distances.size() - 1) / 2 + (distances.size() + 1) / 2] / 2;
	}
	double minDistance = medianDistance * 0.05;

	while (true)
	{
		for (size_t i = 0; i < pointList.size(); i += 2)
		{
			gp_Pnt p1 = pointList[i];
			gp_Pnt p2 = pointList[i + 1];

			if (isHorizontal)
			{
				p1.SetZ(0);
				p2.SetZ(0);
			}

			double distance = p1.Distance(p2);
			if (distance < minDistance) { continue; }
			if (distance < precision) { continue; }
			gp_Vec vec = gp_Vec(p1, p2);

			if (!isHorizontal)
			{
				if (abs(vec.Z()) < 0.001) {
					continue;
				}
			}

			bool vFound = false;
			for (auto& vecPair : vecCountMap)
			{
				if (vecPair.first.IsParallel(vec, precision))
				{
					vecPair.second += 1;
					vFound = true;
					break;
				}
			}
			if (vFound) { continue; }
			vecCountMap.emplace_back(std::pair<gp_Vec, int>(vec, 1));
		}
		if (!vecCountMap.empty()) { break; }
		if (minDistance < precision) { return gp_Vec(); }
		minDistance = 0;
	}
	
	std::pair<gp_Vec, int> RotationVecPair = *vecCountMap.begin();
	for (auto& vecPair : vecCountMap)
	{
		if (RotationVecPair.second < vecPair.second)
		{
			RotationVecPair = vecPair;
		}
	}
	return RotationVecPair.first.Normalized();
}

bool helperFunctions::wireIsForwards(const TopoDS_Face& theFace, const TopoDS_Wire& theWire)
{
	double area = 0.0;
	for (TopExp_Explorer exp(theWire, TopAbs_EDGE); exp.More(); exp.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Standard_Real f, l;

		Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(edge, theFace, f, l);
		if (curve2d.IsNull()) continue;

		gp_Pnt2d p1 = curve2d->Value(f);
		gp_Pnt2d p2;

		const int n = 10;
		for (int i = 1; i <= n; ++i)
		{
			Standard_Real t = f + (l - f) * i / n;
			p2 = curve2d->Value(t);
			area += (p2.X() - p1.X()) * (p2.Y() + p1.Y());
			p1 = p2;
		}
	}
	return (area > 0);
}


bool helperFunctions::shareEdge(const TopoDS_Face& theFace, const TopoDS_Face& theotherFace)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	for (TopExp_Explorer currentExpl(theFace, TopAbs_EDGE); currentExpl.More(); currentExpl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(currentExpl.Current());
		for (TopExp_Explorer otherExpl(theotherFace, TopAbs_EDGE); otherExpl.More(); otherExpl.Next())
		{
			TopoDS_Edge otherEdge = TopoDS::Edge(otherExpl.Current());
			if (edgeEdgeOVerlapping(currentEdge, otherEdge)) { return true; }
		}
	}
	return false;
}

bool helperFunctions::edgeEdgeOVerlapping(const TopoDS_Edge& currentEdge, const TopoDS_Edge& otherEdge)
{
	gp_Pnt cP0 = getFirstPointShape(currentEdge);
	gp_Pnt cP1 = getLastPointShape(currentEdge);
	gp_Pnt oP0 = getFirstPointShape(otherEdge);
	gp_Pnt oP1 = getLastPointShape(otherEdge);

	double precision = SettingsCollection::getInstance().spatialTolerance();

	// check if edges are parallel
	gp_Vec currentVec = gp_Vec(cP0, cP1);
	gp_Vec otherVec = gp_Vec(oP0, oP1);

	if (currentVec.Magnitude() < precision || otherVec.Magnitude() < precision) { return false; }

	if (!currentVec.IsParallel(otherVec, precision)) { return false; }

	// check if edges are identical
	if (cP0.IsEqual(oP0, precision) && cP1.IsEqual(oP1, precision) ||
		cP1.IsEqual(oP0, precision) && cP0.IsEqual(oP1, precision))
	{
		return true;
	}

	// if the distance between 3 points of the edges is the same as the full length of one edge. the edges are overlapping
	double currentFullDistance = cP0.Distance(cP1);
	if (abs(currentFullDistance - (cP0.Distance(oP0) + oP0.Distance(cP1))) < precision && cP0.Distance(oP0) > precision && oP0.Distance(cP1) > precision ||
		abs(currentFullDistance - (cP0.Distance(oP1) + oP1.Distance(cP1))) < precision && cP0.Distance(oP1) > precision && oP1.Distance(cP1) > precision)
	{
		return true;
	}
	double otherFullDistance = oP0.Distance(oP1);
	if (abs(otherFullDistance - (oP0.Distance(cP0) + cP0.Distance(oP1))) < precision && oP0.Distance(cP0) > precision && cP0.Distance(oP1) > precision  ||
		abs(otherFullDistance - (oP0.Distance(cP1) + cP1.Distance(oP1))) < precision && oP0.Distance(cP1) > precision && cP1.Distance(oP1) > precision)
	{
		return true;
	}
	return false;
}

bool helperFunctions::faceFaceOverlapping(const TopoDS_Face& upperFace, const TopoDS_Face& lowerFace)
{
	// compute area
	double setPresicion = SettingsCollection::getInstance().spatialTolerance();
	if (abs(computeArea(upperFace) - computeArea(lowerFace)) > setPresicion) { return false; }

	// align verts
	for (TopExp_Explorer currentVertExpl(upperFace, TopAbs_VERTEX); currentVertExpl.More(); currentVertExpl.Next())
	{
		gp_Pnt currentPoint = BRep_Tool::Pnt(TopoDS::Vertex(currentVertExpl.Current()));

		bool vertFound = false;
		for (TopExp_Explorer otherVertExpl(lowerFace, TopAbs_VERTEX); otherVertExpl.More(); otherVertExpl.Next())
		{
			gp_Pnt otherPoint = BRep_Tool::Pnt(TopoDS::Vertex(otherVertExpl.Current()));

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

bool helperFunctions::coplanarOverlapping(const TopoDS_Face& leftFace, const TopoDS_Face& rightFace)
{
	// check if endpoint of wire is on face

	for (TopExp_Explorer currentExpl(leftFace, TopAbs_EDGE); currentExpl.More(); currentExpl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(currentExpl.Current());
		 
		gp_Pnt p0 = helperFunctions::getFirstPointShape(currentEdge);
		gp_Pnt p1 = helperFunctions::getLastPointShape(currentEdge);
		if (pointOnShape(rightFace, p0)) { return true; }
		if (pointOnShape(rightFace, p1)) { return true; }
	}

	for (TopExp_Explorer currentExpl(rightFace, TopAbs_EDGE); currentExpl.More(); currentExpl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(currentExpl.Current());

		gp_Pnt p0 = helperFunctions::getFirstPointShape(currentEdge);
		gp_Pnt p1 = helperFunctions::getLastPointShape(currentEdge);
		if (pointOnShape(leftFace, p0)) { return true; }
		if (pointOnShape(leftFace, p1)) { return true; }
	}

	// check if any mesh point is on another face

	for (const gp_Pnt& currentPoint : getPointListOnFace(leftFace))
	{
		if (pointOnShape(rightFace, currentPoint)) { return true; }
	}

	for (const gp_Pnt& currentPoint : getPointListOnFace(rightFace))
	{
		if (pointOnShape(leftFace, currentPoint)) { return true; }
	}

	// check if wire edges intersect

	//TODO: implement



	return false;


}

bool helperFunctions::surfaceIsIncapsulated(const TopoDS_Face& innerSurface, const TopoDS_Face& outerSurface)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	for (TopExp_Explorer explorer(innerSurface, TopAbs_VERTEX); explorer.More(); explorer.Next())
	{
		const TopoDS_Vertex& vertex = TopoDS::Vertex(explorer.Current());
		gp_Pnt currentPoint = BRep_Tool::Pnt(vertex);

		if (!pointOnFace(outerSurface, currentPoint) && !pointOnWire(outerSurface, currentPoint))
		{
			return false;
		}
	}

	std::vector<gp_Pnt> pointList = getPointListOnFace(innerSurface);
	for (const gp_Pnt& currentPoint : pointList)
	{
		if (!pointOnFace(outerSurface, currentPoint))
		{
			return false;
		}
	}
	return true;
}

double helperFunctions::tVolume(const gp_Pnt& p, const std::vector<gp_Pnt>& vertices) {
	const gp_Pnt& vert0 = vertices[0];
	const gp_Pnt& vert1 = vertices[1];
	const gp_Pnt& vert2 = vertices[2];

	BoostPoint3D p1(vert0.X() - vert1.X(), vert0.Y() - vert1.Y(), vert0.Z() - vert1.Z());
	BoostPoint3D p2(vert1.X() - p.X(), vert1.Y() - p.Y(), vert1.Z() - p.Z());
	BoostPoint3D p3(vert2.X() - p.X(), vert2.Y() - p.Y(), vert2.Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3)) / 6;
}

bool helperFunctions::triangleIntersecting(const std::vector<gp_Pnt>& line, const std::vector<gp_Pnt>& triangle)
{
	const gp_Pnt& lineStart = line[0];
	const gp_Pnt& lineEnd = line[1];

	for (size_t i = 0; i < 3; i++)
	{
		int ip = (i + 1) % 3;
		int ipp = (i + 2) % 3;

		if (
			hasSameSign(
				tVolume(triangle[i], { triangle[ipp], lineStart, lineEnd }),
				tVolume(triangle[ip], { triangle[ipp], lineStart, lineEnd }))
			) { return false; }
	}

	double leftFinal = tVolume(lineStart, triangle);
	double rightFinal = tVolume(lineEnd, triangle);

	double precision = SettingsCollection::getInstance().spatialTolerance();

	if (abs(leftFinal) < precision || abs(rightFinal) < precision) { return false; } // if surfaces rest on eachother return 0
	if (!hasSameSign(leftFinal, rightFinal)) { return true; }
	return false;
}

bool helperFunctions::hasSameSign(const double& leftDouble, const double& rightDouble)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	const bool leftIsZero = std::abs(leftDouble) < precision;
	const bool rightIsZero = std::abs(rightDouble) < precision;

	if (leftIsZero || rightIsZero) { return false; }
	if (leftDouble > 0 && rightDouble > 0 || leftDouble < 0 && rightDouble < 0) { return true; }
	return false;
}


std::optional<gp_Pnt> helperFunctions::linearLineIntersection(const gp_Pnt& sP1, const gp_Pnt& eP1, const gp_Pnt& sP2, const gp_Pnt& eP2, bool projected, double buffer) {

	double precision = SettingsCollection::getInstance().spatialTolerance();

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

bool helperFunctions::LineShapeIntersection(const TopoDS_Shape& theShape, const gp_Pnt& lP1, const gp_Pnt& lP2)
{
	for (TopExp_Explorer faceExpl(theShape, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());

		if (LineShapeIntersection(currentFace, lP1, lP2)) { return true; }
	}
	return false;
}

bool helperFunctions::LineShapeIntersection(const TopoDS_Face& theFace, const gp_Pnt& lP1, const gp_Pnt& lp2, bool inZdir)
{
	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	if (mesh.IsNull())
	{
		helperFunctions::triangulateShape(theFace);
		mesh = BRep_Tool::Triangulation(theFace, loc);
	}
	if (mesh.IsNull()) { return false; }

	for (int j = 1; j <= mesh.get()->NbTriangles(); j++) //TODO: if large num indx?
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);
		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		if (inZdir)
		{
			gp_Pnt lll;
			gp_Pnt urr;
			bBoxDiagonal({ p1, p2, p3 }, &lll, &urr);

			if (lP1.X() > urr.X() || lP1.X() < lll.X()) { continue; }
			if (lP1.Y() > urr.Y() || lP1.Y() < lll.Y()) { continue; }
		}
		

		if (helperFunctions::triangleIntersecting({ lP1, lp2 }, {p1, p2, p3}))
		{
			return true;
		}
	}
	return false;
}

TopoDS_Wire helperFunctions::mergeWireOrientated(const TopoDS_Wire& baseWire, const TopoDS_Wire& mergingWire) {
	double precision = SettingsCollection::getInstance().spatialTolerance();
	
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

std::vector<TopoDS_Face> helperFunctions::mergeFaces(const std::vector<TopoDS_Face>& theFaceList) //TODO: inify with planeroutline
{
	if (theFaceList.size() == 1) { return theFaceList; }
	double precision = SettingsCollection::getInstance().spatialTolerance();

	std::vector<gp_Vec> faceNormalList;
	std::vector<TopoDS_Face> faceCopyList;
	for (const TopoDS_Face surfacePair : theFaceList)
	{
		gp_Vec currentVec = helperFunctions::computeFaceNormal(surfacePair);
		if (currentVec.Magnitude() < precision) { continue; }
		faceNormalList.emplace_back(currentVec);
		faceCopyList.emplace_back(surfacePair);
	}

	std::vector<int> evalList(faceNormalList.size(), 0);
	std::vector<TopoDS_Face> cleanedFaceCollection;
	bool hasMergedFaces = false;

	for (size_t i = 0; i < faceNormalList.size(); i++) //TODO: indexing?
	{
		if (evalList[i] == 1) { continue; }
		std::vector<TopoDS_Face> mergingPairList;

		TopoDS_Face currentFace = faceCopyList[i];
		mergingPairList.emplace_back(currentFace);
		evalList[i] = 1;

		while (true)
		{
			int originalMergeSize = mergingPairList.size();

			for (size_t j = 0; j < faceNormalList.size(); j++)
			{
				if (j == i) { continue; }
				if (evalList[j] == 1) { continue; }

				TopoDS_Face otherFace = faceCopyList[j];
				if (!faceNormalList[i].IsParallel(faceNormalList[j], precision)) { continue; }

				// find if the surface shares edge with any of the to merge faces
				bool toMerge = false;
				for (const TopoDS_Face& mergingFace : mergingPairList)
				{
					if (shareEdge(otherFace, mergingFace))
					{
						toMerge = true;
						break;
					}
					if (coplanarOverlapping(otherFace, mergingFace))
					{
						toMerge = true;
						break;
					}
				}

				if (!toMerge) { continue; }
				mergingPairList.emplace_back(otherFace);
				evalList[j] = 1;

			}
			if (originalMergeSize == mergingPairList.size()) { break; }
		}
		
		if (mergingPairList.size() == 1)
		{
			cleanedFaceCollection.emplace_back(currentFace);
			continue;
		}

		std::vector<TopoDS_Face> mergedFaceList = planarFaces2Outline(mergingPairList);

		if (mergedFaceList.empty())
		{
			for (const TopoDS_Face& mergedFace : mergingPairList)
			{
				cleanedFaceCollection.emplace_back(mergedFace);
			}
			continue;
		}

		for (TopoDS_Face mergedFace : mergedFaceList)
		{
			if (fixFace(&mergedFace))
			{
				cleanedFaceCollection.emplace_back(mergedFace);
			}	
		}
	}
	return cleanedFaceCollection;
}

std::vector<TopoDS_Face> helperFunctions::mergeCoFaces(const std::vector<TopoDS_Face>& theFaceList)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	if (!theFaceList.size()) { return theFaceList; }

	TopTools_ListOfShape toolList;
	for (const TopoDS_Face& currentFace : theFaceList)
	{
		toolList.Append(currentFace);
	}

	BRepAlgoAPI_Fuse fuser;
	fuser.SetArguments(toolList);
	fuser.SetTools(toolList);
	fuser.SetFuzzyValue(precision);
	fuser.Build();

	TopoDS_Shape mergedShape = fuser.Shape();

	Bnd_Box boundingBox;
	BRepBndLib::Add(mergedShape, boundingBox);
	Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;

	Handle(Geom_Surface) geomSurface = BRep_Tool::Surface(theFaceList[0]);
	if (geomSurface.IsNull()) { return theFaceList; }
	Handle(Geom_Plane) currentGeoPlane = Handle(Geom_Plane)::DownCast(geomSurface);
	if (currentGeoPlane.IsNull()) { return theFaceList; }

	TopoDS_Face largerFace = BRepBuilderAPI_MakeFace(currentGeoPlane, -1000, 1000, -1000, 1000, Precision::Confusion());
	gp_Pnt basePoint = helperFunctions::getFirstPointShape(largerFace);

	BRepAlgoAPI_Splitter splitter;
	splitter.SetFuzzyValue(precision);
	TopTools_ListOfShape splitterToolList;
	TopTools_ListOfShape argumentList;

	argumentList.Append(largerFace);
	splitter.SetArguments(argumentList);

	splitterToolList.Append(fuser.Shape());
	splitter.SetTools(splitterToolList);
	splitter.Build();

	splitterToolList.Clear();
	for (TopExp_Explorer faceExpl(splitter.Shape(), TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());

		std::optional<gp_Pnt> optionalPoint = helperFunctions::getPointOnFace(currentFace);

		if (optionalPoint == std::nullopt) { continue; }
		gp_Pnt currentPoint = *optionalPoint;

		bool isFound = false;
		for (const TopoDS_Face originalFace : theFaceList)
		{
			if (pointOnShape(originalFace, currentPoint))
			{
				isFound = true;
				break;
			}
		}
		if (isFound) { continue; }

		splitterToolList.Append(currentFace);
	}

	std::vector<TopoDS_Face> mergedFaces;
	splitter.SetTools(splitterToolList);
	splitter.Build();
	for (TopExp_Explorer faceExpl(splitter.Shape(), TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());

		std::optional<gp_Pnt> optionalPoint = helperFunctions::getPointOnFace(currentFace);
		if (optionalPoint == std::nullopt) { continue; }
		gp_Pnt currentPoint = *optionalPoint;

		bool isFound = false;
		for (const TopoDS_Face originalFace : theFaceList)
		{
			if (pointOnShape(originalFace, currentPoint))
			{
				isFound = true;
				break;
			}
		}
		if (!isFound) { continue; }
		mergedFaces.emplace_back(currentFace);
	}
	return mergedFaces;
}

TopoDS_Wire helperFunctions::closeWireOrientated(const TopoDS_Wire& baseWire) {
	gp_Pnt p1 = helperFunctions::getFirstPointShape(baseWire);
	gp_Pnt p2 = helperFunctions::getLastPointShape(baseWire);

	if (p1.Distance(p2) < SettingsCollection::getInstance().spatialTolerance()) { return baseWire; }

	TopoDS_Wire closingWire = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(p2, p1));

	return mergeWireOrientated(baseWire, closingWire);
}


TopoDS_Face helperFunctions::createHorizontalFace(double x, double y, double z) {
	
	gp_Pnt p0(-x, -y, z);
	gp_Pnt p1(-x, y, z);
	gp_Pnt p2(x, y, z);
	gp_Pnt p3(x, -y, z);

	return createPlanarFace(p0, p1, p2, p3);
}

TopoDS_Face helperFunctions::createHorizontalFace(const gp_Pnt& lll, const gp_Pnt& urr, double rotationAngle, double z) {
	gp_Pnt p0 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), z), rotationAngle);
	gp_Pnt p1 = helperFunctions::rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), z), rotationAngle);
	gp_Pnt p2 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), z), rotationAngle);
	gp_Pnt p3 = helperFunctions::rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), z), rotationAngle);

	return createPlanarFace(p0, p1, p2, p3);
}

TopoDS_Face helperFunctions::createPlanarFace(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3) {

	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
	TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p0);

	return BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2, edge3));
}

TopoDS_Face helperFunctions::createPlanarFace(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2)
{
	TopoDS_Edge edge0 = BRepBuilderAPI_MakeEdge(p0, p1);
	TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
	TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p0);

	return BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(edge0, edge1, edge2));
}

TopoDS_Face helperFunctions::projectFaceFlat(const TopoDS_Face& theFace, double height) {

	double precision = SettingsCollection::getInstance().spatialTolerance();
	double angularTolerance = SettingsCollection::getInstance().angularTolerance();
	if (theFace.IsNull()) { return TopoDS_Face(); }
	// check if face is flat
	gp_Vec faceNormal = computeFaceNormal(theFace);
	if (abs(faceNormal.Z()) < angularTolerance) { return TopoDS_Face(); }

	TopoDS_Face flatFace;
	if (abs(faceNormal.X()) < angularTolerance && abs(faceNormal.Y()) < angularTolerance)
	{
		gp_Trsf trsf;
		double faceHeight = getLowestZ(theFace);
		if (abs(height - faceHeight) < precision) { return theFace; }
		trsf.SetTranslationPart(gp_XYZ(0, 0, height - faceHeight));
		BRepBuilderAPI_Transform transformer(theFace, trsf);
		if (!transformer.IsDone()) {  return TopoDS_Face(); }
		flatFace = TopoDS::Face(transformer.Shape());
	}
	else
	{
		gp_Pnt p0 = getFirstPointShape(theFace);
		p0.SetZ(height);
		gp_Vec normal(0, 0, 1);
		Handle(Geom_Plane) plane = new Geom_Plane(p0, normal);

		TopoDS_Wire outerWire = BRepTools::OuterWire(theFace);
		if (outerWire.IsNull()) { return TopoDS_Face(); }
		if (!outerWire.Closed()) { return TopoDS_Face(); }

		TopoDS_Wire flattenedWire = projectWireFlat(outerWire, height);
		if (flattenedWire.IsNull()) { return TopoDS_Face(); }
		flattenedWire.Orientation(TopAbs_FORWARD);

		BRepBuilderAPI_MakeFace faceMaker(plane, flattenedWire, precision);
		for (TopExp_Explorer expl(theFace, TopAbs_WIRE); expl.More(); expl.Next())
		{
			TopoDS_Wire currentWire = TopoDS::Wire(expl.Current());
			if (currentWire.IsEqual(outerWire)) { continue; }
			TopoDS_Wire currentFlatWire = projectWireFlat(currentWire, height);
			currentFlatWire.Orientation(TopAbs_REVERSED);
			BRepBuilderAPI_MakeFace faceMaker2(currentFlatWire);
			if (!faceMaker2.IsDone()) { continue; }
			TopoDS_Face innerFace = faceMaker2.Face();
			if (innerFace.IsNull()) { continue; }
			if (computeArea(innerFace) < 0.001) { continue; }
			faceMaker.Add(currentFlatWire);
		}

		if (!faceMaker.IsDone())
		{
			return TopoDS_Face();
		}
		flatFace = faceMaker.Face();
	}
	fixFace(&flatFace);
	return flatFace;
}

TopoDS_Wire helperFunctions::projectWireFlat(const TopoDS_Wire& theWire, double height)
{
	BRepBuilderAPI_MakeWire builder;
	double precision = SettingsCollection::getInstance().spatialTolerance();
	for (BRepTools_WireExplorer expl(theWire); expl.More(); expl.Next()) {

		TopoDS_Edge edge = TopoDS::Edge(expl.Current());
		gp_Pnt p0 = helperFunctions::getFirstPointShape(edge);
		gp_Pnt p1 = helperFunctions::getLastPointShape(edge);

		p0.SetZ(height);
		p1.SetZ(height);

		if (p0.IsEqual(p1, precision))
		{
			continue;
		}

		BRepBuilderAPI_MakeEdge edgeMaker(p0, p1);
		if (!edgeMaker.IsDone())
		{
			continue;
		}
		builder.Add(edgeMaker.Edge());
	}

	if (!builder.IsDone())
	{
		return TopoDS_Wire();
	}

	TopoDS_Wire flattenedWire = builder.Wire();

	if (theWire.Closed() != flattenedWire.Closed())
	{
		return TopoDS_Wire();
	}

	return flattenedWire;
}

std::vector<TopoDS_Face> helperFunctions::TessellateFace(const TopoDS_Face& theFace, bool knownIsFlat)
{
	if (!knownIsFlat)
	{
		if (!isFlat(theFace)) {
			return TriangulateFace(theFace);
		}
	}

	double precision = SettingsCollection::getInstance().spatialTolerance();
	gp_Pnt p0 = getFirstPointShape(theFace);
	gp_Vec normal = computeFaceNormal(theFace);
	if (normal.Magnitude() < precision) { return {}; }

	Handle(Geom_Plane) plane = new Geom_Plane(p0, normal);
	TopoDS_Wire outerWire = BRepTools::OuterWire(theFace);
	if (outerWire.IsNull()) { return {}; }
	if (!outerWire.Closed()) { return {}; }
	TopoDS_Wire outerWireClean = outerWire;
	outerWireClean.Orientation(TopAbs_FORWARD);
	TopoDS_Wire outerstraightWire = replaceCurves(outerWire);
	TopoDS_Wire outerCleanedWire = cleanWire(outerstraightWire);
	BRepBuilderAPI_MakeFace faceMaker(plane, outerCleanedWire, precision);
	for (TopExp_Explorer expl(theFace, TopAbs_WIRE); expl.More(); expl.Next())
	{
		TopoDS_Wire currentWire = TopoDS::Wire(expl.Current());
		if (currentWire.IsEqual(outerWire)) { continue; }
		currentWire.Orientation(TopAbs_REVERSED);

		TopoDS_Wire currentStraightWire = replaceCurves(currentWire);
		if (currentStraightWire.IsNull()) { continue; }
		if (!currentStraightWire.Closed()) { continue; }
		TopoDS_Wire currentCleanWire = cleanWire(currentStraightWire);

		BRepBuilderAPI_MakeFace faceMaker2(currentCleanWire);
		TopoDS_Face innerFace = faceMaker2.Face();
		if (innerFace.IsNull()) { continue; }
		if (computeArea(innerFace) < 0.001) { continue; }
		faceMaker.Add(currentCleanWire);
	}
	TopoDS_Face currentFace = faceMaker.Face();

	if (!fixFace(&currentFace))
	{
		return { theFace };
	}
	return { currentFace };
}

bool helperFunctions::fixFace(TopoDS_Face* theFace)
{
	BRepCheck_Analyzer analyzer(*theFace);
	if (analyzer.IsValid()) // no need to fix
	{
		return true;
	}

	ShapeFix_Face faceFixer(*theFace);
	if (wireCount(*theFace) > 1)
	{
		faceFixer.FixOrientation(); // fixes the innerwire invalid issue
	}
	faceFixer.FixIntersectingWires();
	faceFixer.SetPrecision(SettingsCollection::getInstance().spatialTolerance());
	faceFixer.Perform();
	TopoDS_Face fixedFace = faceFixer.Face();

	BRepCheck_Analyzer cleanAnalyzer(fixedFace);
	if (!cleanAnalyzer.IsValid())
	{
		return false;
	}
	if (fixedFace.IsNull()) { return false; }

	*theFace = fixedFace;
	return true;
}

std::vector<TopoDS_Face> helperFunctions::TessellateFace(const std::vector<TopoDS_Face>& theFaceList, bool knownIsFlat)
{
	std::vector<TopoDS_Face> outputList;
	outputList.reserve(theFaceList.size());
	for (const TopoDS_Face& currentFace : theFaceList)
	{
		std::vector<TopoDS_Face> cleanedFaceList = TessellateFace(currentFace, knownIsFlat);

		if (cleanedFaceList.empty())
		{
			outputList.emplace_back(currentFace);
			continue;
		}

		for (const TopoDS_Face& cleanedFace : cleanedFaceList)
		{
			outputList.emplace_back(cleanedFace);
		}
	}
	return outputList;
}

std::vector<TopoDS_Face> helperFunctions::TriangulateFace(const TopoDS_Face& theFace)
{
	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	if (mesh.IsNull())
	{
		helperFunctions::triangulateShape(theFace);
		mesh = BRep_Tool::Triangulation(theFace, loc);
	}
	if (mesh.IsNull()) { return {}; }

	std::vector<TopoDS_Face> triangleFaceList;
	for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);

		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		TopoDS_Face triangleFace = createPlanarFace(p1, p2, p3);

		if (triangleFace.IsNull()) { continue; }
		triangleFaceList.emplace_back(triangleFace);
	}

	std::vector<TopoDS_Face> collapsedTriangles = mergeFaces(triangleFaceList);
	return collapsedTriangles;
}

TopoDS_Wire helperFunctions::wipeWireClean(const TopoDS_Wire& theWire)
{
	BRepBuilderAPI_MakeWire wireMaker;
	for (BRepTools_WireExplorer expl(theWire); expl.More(); expl.Next()) {
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());

		TopoDS_Edge reversedEdge = BRepBuilderAPI_MakeEdge(helperFunctions::getFirstPointShape(currentEdge), helperFunctions::getLastPointShape(currentEdge));

		wireMaker.Add(reversedEdge);
	}
	TopoDS_Wire newWire = wireMaker.Wire();
	return newWire;
}

TopoDS_Wire helperFunctions::projectWireOnPlane(const TopoDS_Wire& wire, const Handle(Geom_Plane)& plane)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	const Handle(Geom_Surface) surface = Handle(Geom_Surface)::DownCast(plane);

	BRepBuilderAPI_MakeWire wireBuilder;
	BRep_Builder builder;

	for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(exp.Current());

		Standard_Real f, l;
		Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, f, l);
		if (curve3d.IsNull()) continue;

		// Project to 2D on the plane
		Handle(Geom2d_Curve) curve2d = GeomAPI::To2d(curve3d, plane->Pln());
		if (curve2d.IsNull()) continue;

		// Create new edge
		TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(curve3d, f, l);
		builder.UpdateEdge(newEdge, curve2d, surface, TopLoc_Location(), precision);
		newEdge.Orientation(edge.Orientation()); // preserve orientation

		wireBuilder.Add(newEdge);
	}

	return wireBuilder.Wire();
}


std::vector<TopoDS_Wire> helperFunctions::growWires(const std::vector<TopoDS_Edge>& edgeList) {
	std::vector<TopoDS_Wire> wireCollection;
	std::vector<TopoDS_Wire> wireCollectionClosed;
	std::vector<TopoDS_Edge> tempEdgeList;

	std::vector<TopoDS_Edge> cleanEdgelist;
	for (const TopoDS_Edge& currentEdge : replaceCurves(edgeList))
	{
		cleanEdgelist.emplace_back(currentEdge);
	}

	//BRepBuilderAPI_MakeWire wireMaker;
	bool loopFound = false;

	TopoDS_Edge currentEdge = cleanEdgelist[0];
	std::vector<int> evaluated(cleanEdgelist.size());
	evaluated[0] = 1;

	gp_Pnt originPoint = helperFunctions::getFirstPointShape(cleanEdgelist[0]); // the original point of the original edge
	gp_Pnt extendingPoint = helperFunctions::getLastPointShape(cleanEdgelist[0]); // the point from which will be extended

	tempEdgeList.emplace_back(currentEdge);

	double precision = SettingsCollection::getInstance().spatialTolerance();
	bool isReversed = false;
	while (true)
	{
		bool hasStapped = false; // true if a stap is found in the while iteration
		bool closed = false; // true if the extensionpoint meets the originpoint

		for (size_t i = 0; i < cleanEdgelist.size(); i++)
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges
			TopoDS_Edge otherEdge = cleanEdgelist[i];

			gp_Pnt p1 = helperFunctions::getFirstPointShape(otherEdge);
			gp_Pnt p2 = helperFunctions::getLastPointShape(otherEdge);

			if (p1.IsEqual(extendingPoint, precision)) // check if edge is neighbour
			{
				extendingPoint = p2;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				else { tempEdgeList.emplace_back(otherEdge); }
				break;
			}
			else if (p2.IsEqual(extendingPoint, precision)) // check if reversed edge is neighbour 
			{
				extendingPoint = p1;
				evaluated[i] = 1;
				hasStapped = true;
				if (isReversed) { tempEdgeList.insert(tempEdgeList.begin(), otherEdge); }
				else { tempEdgeList.emplace_back(BRepBuilderAPI_MakeEdge(p2, p1).Edge()); }
				break;
			}
			else if (extendingPoint.IsEqual(originPoint, precision)) // check if a closed loop is found if no new neighbour is there
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


		for (size_t i = 0; i < cleanEdgelist.size(); i++) // search next unused edge to create new wire
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges

			originPoint = helperFunctions::getFirstPointShape(cleanEdgelist[i]); // the original point of the original edge
			extendingPoint = helperFunctions::getLastPointShape(cleanEdgelist[i]); // the point from which will be extended
			tempEdgeList.emplace_back(cleanEdgelist[i]);

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

	if (wireCollection.size() != 0) {

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

			gp_Pnt startpoint = helperFunctions::getFirstPointShape(currentWire);
			gp_Pnt endpoint = helperFunctions::getLastPointShape(currentWire);

			for (int i = 0; i < wireCollection.size(); i++)
			{
				TopoDS_Wire otherwire = wireCollection[i];
				gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(otherwire);
				gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(otherwire);

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

			if (idxMatch != -1)
			{
				currentWire = helperFunctions::mergeWireOrientated(currentWire, BRepBuilderAPI_MakeWire(connectionEdge));
				currentWire = helperFunctions::mergeWireOrientated(currentWire, wireCollection[idxMatch]);
				wireCollection.erase(wireCollection.begin() + idxMatch);
				stepped = true;
			}

			if (!stepped)
			{
				wireCollectionClosed.emplace_back(helperFunctions::closeWireOrientated(currentWire));
				if (wireCollection.size() == 0) { break; }

				currentWireIdx++;
				currentWire = wireCollection[0];
				wireCollection.erase(wireCollection.begin());
			}
		}
	}

	for (const TopoDS_Wire& currentWire : wireCollectionClosed)
	{
		
		 
	}

	



	return wireCollectionClosed;
}

std::vector<TopoDS_Wire> helperFunctions::cleanWires(const std::vector<TopoDS_Wire>& wireList) {

	std::vector<TopoDS_Wire> cleanedWires;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		TopoDS_Wire cleanWire = helperFunctions::cleanWire(wireList[i]);
		if (cleanWire.IsNull()) { return {}; }
		cleanedWires.emplace_back(cleanWire);
	}
	return cleanedWires;
}

TopoDS_Wire helperFunctions::cleanWire(const TopoDS_Wire& wire) {
	TopTools_IndexedDataMapOfShapeListOfShape vertexToEdges;
	TopExp::MapShapesAndAncestors(wire, TopAbs_VERTEX, TopAbs_EDGE, vertexToEdges);

	if (!wire.Closed())
	{
		return wire;
	}

	std::vector<TopoDS_Edge> allEdges;
	for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next()) {
		TopoDS_Edge currentEdge = TopoDS::Edge(exp.Current());
		if (currentEdge.Orientation() == TopAbs_INTERNAL) { continue; }
		allEdges.emplace_back(currentEdge);
	}

	double precision = SettingsCollection::getInstance().spatialTolerance();

	TopTools_MapOfShape visited;
	BRepBuilderAPI_MakeWire wireMaker;
	for (const TopoDS_Edge& currentEdge : allEdges)
	{
		if (visited.Contains(currentEdge)) { continue; }
		visited.Add(currentEdge);

		std::vector<TopoDS_Edge> colinGroup{ currentEdge };
		gp_Vec currentDir = helperFunctions::computeEdgeDir(currentEdge);

		if (currentDir.Magnitude() < precision) { continue; }

		// grow forwards
		TopoDS_Vertex firstVertex = TopExp::FirstVertex(currentEdge, true);

		while (true)
		{
			const TopTools_ListOfShape& adjEdges = vertexToEdges.FindFromKey(firstVertex);
			bool found = false;
			for (const TopoDS_Shape& potentialShape : adjEdges) {
				TopoDS_Edge potentialEdge = TopoDS::Edge(potentialShape);
				if (visited.Contains(potentialEdge)) { continue; }
				
				gp_Vec otherDir = helperFunctions::computeEdgeDir(potentialEdge);
				if (otherDir.Magnitude() < precision) 
				{ 
					visited.Add(potentialEdge);
					continue; 
				}

				if (!currentDir.IsParallel(otherDir, precision)) { continue; }
				colinGroup.insert(colinGroup.begin(), potentialEdge);
				visited.Add(potentialEdge);
				firstVertex = TopExp::FirstVertex(potentialEdge, true);
				found = true;
			}
			if (!found)
			{
				break;
			}
		}

		// grow backwards
		TopoDS_Vertex lastVertex = TopExp::LastVertex(currentEdge, true);
		while (true)
		{
			const TopTools_ListOfShape& adjEdges = vertexToEdges.FindFromKey(lastVertex);
			bool found = false;
			for (const TopoDS_Shape& potentialShape : adjEdges) {
				TopoDS_Edge potentialEdge = TopoDS::Edge(potentialShape);
				if (visited.Contains(potentialEdge)) { continue; }
				gp_Vec otherDir = helperFunctions::computeEdgeDir(potentialEdge);
				if (otherDir.Magnitude() < precision)
				{
					visited.Add(potentialEdge);
					continue;
				}

				if (!currentDir.IsParallel(otherDir, precision)) { continue; }
				colinGroup.emplace_back(potentialEdge);
				visited.Add(potentialEdge);
				lastVertex = TopExp::LastVertex(potentialEdge, true);
				found = true;
			}
			if (!found)
			{
				break;
			}
		}

		if (colinGroup.size() == 1)
		{
			wireMaker.Add(currentEdge);
			continue;
		}

		gp_Pnt p1 = BRep_Tool::Pnt(TopExp::FirstVertex(colinGroup.front(), true));
		gp_Pnt p2 = BRep_Tool::Pnt(TopExp::LastVertex(colinGroup.back(), true));

		if (p1.IsEqual(p2, precision)){continue;}
		TopoDS_Edge mergedEdge = BRepBuilderAPI_MakeEdge(p1,p2);	
		wireMaker.Add(mergedEdge);
	}

	if (!wireMaker.IsDone())
	{
		return wire;
	}
	TopoDS_Wire finalWire = wireMaker.Wire();
	if (finalWire.Closed())
	{
		return finalWire;
	}
	return wire;

}

TopoDS_Face helperFunctions::wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList) {

	double precision = SettingsCollection::getInstance().spatialTolerance();

	BRepBuilderAPI_MakeFace faceBuilder;
	std::vector<TopoDS_Face> faceList;
	gp_Vec normal = computeFaceNormal(wireList[0]);

	if (normal.Magnitude() < precision) { return TopoDS_Face(); }

	std::vector<gp_Pnt> pointList = getUniquePoints(wireList[0]);
	gp_Pnt originPoint = pointList[0];
	gp_Vec castingVector = gp_Vec(originPoint, pointList[1]);

	for (const TopoDS_Wire& currentWire : wireList)
	{
		if (helperFunctions::getUniquePoints(currentWire).size() < 3) { continue; }
		faceBuilder = BRepBuilderAPI_MakeFace(
			gp_Pln(originPoint, normal),
			currentWire
		);
		if (faceBuilder.Error() == BRepBuilderAPI_FaceDone) { faceList.emplace_back(faceBuilder.Face()); }
	}

	if (!faceList.size()) { return TopoDS_Face(); }
	if (faceList.size() == 1) { return faceList[0]; }

	// test which surfaces are inner loops
	std::vector<double> areaList;
	std::vector<TopoDS_Face> correctedFaceList;
	for (const TopoDS_Face& currentFace : faceList)
	{
		double area = computeArea(currentFace);
		if (area < 0.001) { continue; }
		areaList.emplace_back(area);
		correctedFaceList.emplace_back(currentFace);
	}

	std::vector<TopoDS_Face> orderedFootprintList;
	std::vector<double> orderedAreaList;
	std::vector<int> ordered(areaList.size());
	for (size_t i = 0; i < areaList.size(); i++)
	{
		double evalArea = 0;
		int evalIdx = -1;
		for (int j = 0; j < areaList.size(); j++)
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

	TopoDS_Face clippedFace = orderedFootprintList[0];
	for (size_t i = 1; i < orderedFootprintList.size(); i++)
	{
		for (TopExp_Explorer expl(orderedFootprintList[i], TopAbs_WIRE); expl.More(); expl.Next())
		{
			TopoDS_Wire voidWire = TopoDS::Wire(expl.Current());
			voidWire = TopoDS::Wire(expl.Current().Reversed()); 
			try
			{
				BRepBuilderAPI_MakeFace merger = BRepBuilderAPI_MakeFace(clippedFace, voidWire);
				clippedFace = merger.Face();
			}
			catch (const std::exception&)
			{
				continue;
			}	
			break;
		}
	}
	return clippedFace;
}

std::vector<TopoDS_Face> helperFunctions::planarFaces2Outline(const std::vector<TopoDS_Face>& planarFaces, const TopoDS_Face& boundingFace)
{
	std::vector<TopoDS_Shape> faceClusterList = planarFaces2Cluster(planarFaces);
	if (faceClusterList.empty()) { return {}; }

	double precisionCoarse = SettingsCollection::getInstance().spatialTolerance(); //TODO: this has been uncoarsened 
	std::vector<TopoDS_Face> outputFaceList;

	const TopoDS_Shape& faceCluster = faceClusterList[0];

	// split section face with the merged splitting faces
	BRepAlgoAPI_Splitter splitter;
	splitter.SetFuzzyValue(precisionCoarse);
	TopTools_ListOfShape toolList;
	TopTools_ListOfShape argumentList;

	argumentList.Append(boundingFace);
	splitter.SetArguments(argumentList);
	toolList.Append(faceCluster);

	splitter.SetTools(toolList);
	splitter.Build();

	if (!splitter.IsDone())
	{
		//TODO: add error
		return {};
	}

	gp_Pnt p0 = helperFunctions::getFirstPointShape(boundingFace);
	std::vector<TopoDS_Face> outerInvFaceList;
	std::vector<TopoDS_Face> innerFaces;

	for (TopExp_Explorer faceExpl(splitter.Shape(), TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());
		// ignore extremely small surfaces
		if (helperFunctions::computeArea(currentFace) < 0.005) { continue; }
		std::optional<gp_Pnt> optionalPoint = getPointOnFace(currentFace);
		if (optionalPoint == std::nullopt) { continue; }
		gp_Pnt pointOnFace = *optionalPoint;

		if (pointOnShape(currentFace, p0))
		{
			for (const TopoDS_Face invertedFace : invertFace(currentFace))
			{
				if (invertedFace.IsNull()) { continue; }
				outerInvFaceList.emplace_back(invertedFace);
			}
			continue;
		}

		if (!pointOnShape(faceCluster, pointOnFace))
		{
			innerFaces.emplace_back(currentFace);
		}
	}

	if (outerInvFaceList.empty())
	{
		//TODO: find out how to avoid this case
		return {};
	}

	for (const TopoDS_Face& currentOuterFace : outerInvFaceList)
	{
		gp_Vec currentNormal = computeFaceNormal(currentOuterFace);
		if (currentNormal.Magnitude() < SettingsCollection::getInstance().spatialTolerance()) { continue; }

		BRepAlgoAPI_Splitter cleanSplitter;
		cleanSplitter.SetFuzzyValue(precisionCoarse);
		TopTools_ListOfShape cleanToolList;
		TopTools_ListOfShape cleanArgumentList;

		cleanArgumentList.Append(currentOuterFace);
		cleanSplitter.SetArguments(cleanArgumentList);

		for (size_t i = 0; i < innerFaces.size(); i++)
		{
			std::optional<gp_Pnt> optionalPoint = getPointOnFace(innerFaces[i]);
			if (optionalPoint == std::nullopt) { continue; }
			gp_Pnt pointOnFace = *optionalPoint;
			if (!pointOnShape(currentOuterFace, pointOnFace)) { continue; }

			cleanToolList.Append(innerFaces[i]);
		}

		if (cleanToolList.IsEmpty() )
		{
			outputFaceList.emplace_back(currentOuterFace);
			continue;
		}

		cleanSplitter.SetTools(cleanToolList);
		cleanSplitter.Build();

		if (!cleanSplitter.IsDone())
		{
			//TODO: add error
			continue;
		}

		TopoDS_Shape test = cleanSplitter.Shape();

		if (test.IsNull()) { continue; }

		for (TopExp_Explorer faceExpl(test, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
		{
			TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());
			if (!fixFace(&currentFace)) {
				//TODO: add error
				//TODO: find fixes
			}
			std::optional<gp_Pnt> optionalPoint = getPointOnFace(currentFace);
			if (optionalPoint == std::nullopt) { continue; }
			gp_Pnt pointOnFace = *optionalPoint;
			if (!pointOnShape(faceCluster, pointOnFace)) { continue; }
			outputFaceList.emplace_back(currentFace);
		}
	}

	std::vector<TopoDS_Face> cleanedOutputFaceList;
	for (const TopoDS_Face& currentFace : outputFaceList)
	{
		std::vector<TopoDS_Face> cleanedFaceList = TessellateFace(currentFace);
		for (TopoDS_Face& cleanedFace : cleanedFaceList)
		{
			fixFace(&cleanedFace);
			helperFunctions::triangulateShape(cleanedFace, true);
			cleanedOutputFaceList.emplace_back(cleanedFace);
		}
	}
	return cleanedOutputFaceList;
}

std::vector<TopoDS_Face> helperFunctions::planarFaces2Outline(const std::vector<TopoDS_Face>& planarFaces)
{
	if (planarFaces.empty()) { return {}; }
	if (planarFaces.size() == 1) { return planarFaces; }

	gp_Trsf transform;
	double precision = SettingsCollection::getInstance().spatialTolerance();

	gp_Vec clusterNormal = computeFaceNormal(planarFaces[0]);
	gp_Vec horizontalNormal = gp_Vec(0, 0, 1);

	std::vector<std::pair<double, TopoDS_Face>> flattenedAreaFaceList;
	flattenedAreaFaceList.reserve(planarFaces.size());

	if (!clusterNormal.IsParallel(horizontalNormal, precision))
	{
		std::optional<gp_Pnt> optionalbasePoint = helperFunctions::getPointOnFace(planarFaces[0]);
		if (optionalbasePoint == std::nullopt) { return {}; }

		gp_Vec normalCrossProduct = clusterNormal ^ horizontalNormal;
		gp_Ax1 rotationAxis(*optionalbasePoint, normalCrossProduct);
		Standard_Real rotationAngle = clusterNormal.AngleWithRef(horizontalNormal, rotationAxis.Direction());

		transform.SetRotation(rotationAxis, rotationAngle);

		for (const TopoDS_Face& face : planarFaces) {
			BRepBuilderAPI_Transform transformer(face, transform);
			if (transformer.IsDone()) {
				TopoDS_Face dubface = TopoDS::Face(transformer.Shape());
				double area = computeArea(dubface);
				flattenedAreaFaceList.emplace_back(std::pair(area,dubface));
			}
		}
	}
	else
	{
		for (const TopoDS_Face& face : planarFaces) {
			double area = computeArea(face);
			flattenedAreaFaceList.emplace_back(area, face);
		}
	}

	std::sort(flattenedAreaFaceList.begin(), flattenedAreaFaceList.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	std::vector<TopoDS_Face> flattenedFaceList;
	flattenedFaceList.reserve(flattenedAreaFaceList.size());
	for (size_t i = 0; i < flattenedAreaFaceList.size(); i++) //TODO: find a better way to filter out overlap
	{
		TopoDS_Face currentFace = flattenedAreaFaceList[i].second;

		if (i + 1 == flattenedAreaFaceList.size())
		{
			flattenedFaceList.emplace_back(currentFace);
			break;
		}

		bool useFace = true;
		for (size_t j = i + 1; j < flattenedAreaFaceList.size(); j++)
		{
			if (surfaceIsIncapsulated(currentFace, flattenedAreaFaceList[j].second))
			{
				useFace = false;
				break;
			}
		}

		if (useFace)
		{
			flattenedFaceList.emplace_back(currentFace);
		}
	}

	if (flattenedFaceList.size() == 1) { return flattenedFaceList; }

	// use the storey approach? 
	gp_Pnt lll;
	gp_Pnt urr;
	// create plane on which the projection has to be made
	helperFunctions::bBoxDiagonal(flattenedFaceList, &lll, &urr);

	if (abs(lll.Z() - urr.Z()) > SettingsCollection::getInstance().spatialTolerance())
	{
		return {};
	}

	gp_Pnt p0 = gp_Pnt(lll.X() - 10, lll.Y() - 10, urr.Z());
	gp_Pnt p1 = gp_Pnt(urr.X() + 10, urr.Y() + 10, urr.Z());
	TopoDS_Face boundingPlane = helperFunctions::createHorizontalFace(p0, p1, 0, urr.Z());

	std::vector<TopoDS_Face> outlinedFaceList = planarFaces2Outline(flattenedFaceList, boundingPlane);
	transform.Invert();

	std::vector<TopoDS_Face> orientedFaces;
	orientedFaces.reserve(outlinedFaceList.size());

	if (!clusterNormal.IsParallel(horizontalNormal, precision))
	{
		for (const TopoDS_Face& outlinedFace : outlinedFaceList)
		{
			BRepBuilderAPI_Transform transformer(outlinedFace, transform);
			orientedFaces.emplace_back(TopoDS::Face(transformer.Shape()));
		}
	}
	else
	{
		orientedFaces = outlinedFaceList;
	}
	return orientedFaces;
}

std::vector<TopoDS_Shape> helperFunctions::planarFaces2Cluster(const std::vector<TopoDS_Face>& planarFaces)
{
	double precisionCoarse = SettingsCollection::getInstance().spatialTolerance(); //TODO: this has been uncoarsened 

	std::vector<TopoDS_Shape> clusteredShapeList;
	FaceComplex faceComplex;
	faceComplex.faceList_ = planarFaces;
	std::vector<FaceComplex> faceComplexList = { faceComplex };

	for (const FaceComplex& faceComplex : faceComplexList)
	{
		// merge the faces
		BRepAlgoAPI_Fuse fuser;
		TopTools_ListOfShape mergeList;
		fuser.SetFuzzyValue(precisionCoarse);
		for (const TopoDS_Face splitFace : faceComplex.faceList_)
		{
			mergeList.Append(splitFace);
		}
		fuser.SetArguments(mergeList);
		fuser.SetTools(mergeList);
		fuser.Build();

		if (!fuser.IsDone())
		{
			//TODO: add error
			continue;
		}
		TopoDS_Shape fusedShape = fuser.Shape();

		if (!fusedShape.IsNull())
		{
			clusteredShapeList.emplace_back(fusedShape);
			continue;
		}
	}
	return clusteredShapeList;
}

TopoDS_Face helperFunctions::getOuterFace(const TopoDS_Shape& splitShape, const TopoDS_Face& originalFace)
{
	// find the outer face
	TopoDS_Face outerFace;
	gp_Pnt p0 = helperFunctions::getFirstPointShape(originalFace);
	for (TopExp_Explorer faceExpl(splitShape, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());
		bool isFound = false;

		for (TopExp_Explorer vertExpl(currentFace, TopAbs_VERTEX); vertExpl.More(); vertExpl.Next()) {
			TopoDS_Vertex currentVertex = TopoDS::Vertex(vertExpl.Current());
			gp_Pnt currentPoint = BRep_Tool::Pnt(currentVertex);

			if (currentPoint.IsEqual(p0, SettingsCollection::getInstance().spatialTolerance()))
			{
				outerFace = currentFace;
				isFound = true;
				break;
			}
		}
		if (isFound) { break; }
	}
	return outerFace;
}

std::vector<TopoDS_Face> helperFunctions::invertFace(const TopoDS_Face& inputFace)
{
	gp_Pnt p0 = helperFunctions::getFirstPointShape(inputFace);
	std::vector<TopoDS_Face> mergedFaceList;
	for (TopExp_Explorer WireExpl(inputFace, TopAbs_WIRE); WireExpl.More(); WireExpl.Next())
	{
		TopoDS_Wire currentWire = TopoDS::Wire(WireExpl.Current());
		bool isInner = true;

		for (TopExp_Explorer vertExpl(currentWire, TopAbs_VERTEX); vertExpl.More(); vertExpl.Next()) {
			TopoDS_Vertex currentVertex = TopoDS::Vertex(vertExpl.Current());
			gp_Pnt currentPoint = BRep_Tool::Pnt(currentVertex);

			if (currentPoint.IsEqual(p0, SettingsCollection::getInstance().spatialTolerance()))
			{
				isInner = false;
				break;
			}
		}
		if (!isInner) { continue; }

		if (!currentWire.Closed())
		{
			continue;
		}

		if (currentWire.Orientation() == TopAbs_REVERSED)
		{
			currentWire.Reverse();
		}

		TopoDS_Face innerFace = BRepBuilderAPI_MakeFace(currentWire);

		BRepCheck_Analyzer check(innerFace);
		if (!check.IsValid()) {
			//TODO: add error
		}

		if (innerFace.IsNull()) { continue; }
		mergedFaceList.emplace_back(innerFace);
	}
	return mergedFaceList;
}

double helperFunctions::getObjectZOffset(IfcSchema::IfcObjectPlacement* objectPlacement, bool deepOnly)
{
	double offset = 0;
	if (objectPlacement->data().type()->name() != "IfcLocalPlacement") { return 0.0; }
	IfcSchema::IfcLocalPlacement* storeyLocalPlacement = objectPlacement->as<IfcSchema::IfcLocalPlacement>();
	IfcSchema::IfcObjectPlacement* localObjectPlacement = storeyLocalPlacement->PlacementRelTo();

	if (!deepOnly && localObjectPlacement != nullptr || localObjectPlacement == nullptr)
	{
		if (storeyLocalPlacement->RelativePlacement()->data().type()->name() != "IfcAxis2Placement3D") { return 0.0; }
		IfcSchema::IfcAxis2Placement3D* axisPlacement = storeyLocalPlacement->RelativePlacement()->as<IfcSchema::IfcAxis2Placement3D>();

#if defined(USE_IFC4x3)
		offset = axisPlacement->Location()->as<IfcSchema::IfcCartesianPoint>()->Coordinates()[2];
#else
		try
		{
			std::vector<double> coord = axisPlacement->Location()->Coordinates();

			if (coord.size() >= 3)
			{
				offset = axisPlacement->Location()->Coordinates()[2];
			}
		}
		catch (const std::exception&)
		{

		}

#endif // DEBUG
	}
	if (localObjectPlacement == nullptr)
	{
		return offset;
	}
	return offset + getObjectZOffset(localObjectPlacement, deepOnly);
}

bool helperFunctions::hasGlassMaterial(const IfcSchema::IfcProduct* ifcProduct)
{
	IfcSchema::IfcRelAssociates::list::ptr associations = ifcProduct->HasAssociations();
	for (IfcSchema::IfcRelAssociates::list::it it = associations->begin(); it != associations->end(); ++it)
	{
		IfcSchema::IfcRelAssociates* IfcRelAssociates = *it;
		if (IfcRelAssociates->data().type()->name() != "IfcRelAssociatesMaterial") { continue; }

		IfcSchema::IfcRelAssociatesMaterial* MaterialAss = IfcRelAssociates->as<IfcSchema::IfcRelAssociatesMaterial>();
		if (MaterialAss->data().type()->name() != "IfcRelAssociatesMaterial") { continue; }

		IfcSchema::IfcMaterialSelect* relMaterial = MaterialAss->RelatingMaterial();
		if (relMaterial->data().type()->name() != "IfcMaterial") { continue; }

		IfcSchema::IfcMaterial* ifcMaterial = relMaterial->as<IfcSchema::IfcMaterial>();
		std::string materialName = boost::to_upper_copy(ifcMaterial->Name());

		if (materialName.find("GLASS") != std::string::npos || 
			materialName.find("GLAZED") != std::string::npos)
		{
			return true;
		}
		 //TODO: implement ifc4x3
#if defined(USE_IFC4x3)
		return false;
#elif defined(USE_IFC2x3) || defined(USE_IFC4) 

		// if material name is not glass or glazed search for render properties transparency
		IfcSchema::IfcMaterialDefinitionRepresentation::list::ptr materialRepresentation = ifcMaterial->HasRepresentation();
		IfcSchema::IfcStyledRepresentation* currentStyleRep = nullptr;

		for (auto propertyIt = materialRepresentation->begin(); propertyIt != materialRepresentation->end(); ++propertyIt)
		{
			bool found = false;
			IfcSchema::IfcMaterialDefinitionRepresentation* currentMaterialRepresenation = *propertyIt;

			IfcSchema::IfcRepresentation::list::ptr representationList = currentMaterialRepresenation->Representations();

			for (auto repIt = representationList->begin(); repIt != representationList->end(); ++repIt)
			{
				IfcSchema::IfcRepresentation* currentRep = *repIt;
				if (currentRep->data().type()->name() == "IfcStyledRepresentation")
				{
					currentStyleRep = currentRep->as<IfcSchema::IfcStyledRepresentation>();
					found = true;
					break;
				}
			}
			if (found) { break; }
		}

		// find via object material
		if (currentStyleRep != nullptr) {

			IfcSchema::IfcRepresentationItem::list::ptr representationList = currentStyleRep->Items();
			for (auto propertyIt = representationList->begin(); propertyIt != representationList->end(); ++propertyIt)
			{
				IfcSchema::IfcRepresentationItem* currentItem = *propertyIt;
				if (currentItem->data().type()->name() != "IfcStyledItem") { continue; }
				IfcSchema::IfcStyledItem* currentStyledItem = currentItem->as<IfcSchema::IfcStyledItem>();

#if defined(USE_IFC2x3) 
				IfcSchema::IfcPresentationStyleAssignment::list::ptr currenStyleAssList = currentStyledItem->Styles();
#elif defined(USE_IFC4) 
				IfcSchema::IfcStyleAssignmentSelect::list::ptr currenStyleAssList = currentStyledItem->Styles();
#endif

				for (auto currenStyleAssIt = currenStyleAssList->begin(); currenStyleAssIt != currenStyleAssList->end(); ++currenStyleAssIt)
				{
#if defined(USE_IFC2x3) 
					IfcSchema::IfcPresentationStyleAssignment* currentStyleAss = *currenStyleAssIt;
#elif defined(USE_IFC4) 
					IfcSchema::IfcStyleAssignmentSelect* currentStyleAss = *currenStyleAssIt;
#endif				
					if (currentStyleAss->data().type()->name() != "IfcPresentationStyleAssignment") { continue; }
					IfcSchema::IfcPresentationStyleAssignment* currentAss = currentStyleAss->as<IfcSchema::IfcPresentationStyleAssignment>();
					IfcSchema::IfcPresentationStyleSelect::list::ptr styleSelectList = currentAss->Styles();

					for (auto styleSelectIt = styleSelectList->begin(); styleSelectIt != styleSelectList->end(); ++styleSelectIt)
					{
						IfcSchema::IfcPresentationStyleSelect* currentStyleSelect = *styleSelectIt;
						if (currentStyleSelect->data().type()->name() != "IfcSurfaceStyle") { continue; }
						IfcSchema::IfcSurfaceStyle* currentStyle = currentStyleSelect->as<IfcSchema::IfcSurfaceStyle>();
						IfcSchema::IfcSurfaceStyleElementSelect::list::ptr elementSurfList = currentStyle->Styles();

						for (auto styleElementIt = elementSurfList->begin(); styleElementIt != elementSurfList->end(); ++styleElementIt)
						{
							IfcSchema::IfcSurfaceStyleElementSelect* currentElemStyle = *styleElementIt;
							if (currentElemStyle->data().type()->name() != "IfcSurfaceStyleRendering") { continue; }
							IfcSchema::IfcSurfaceStyleRendering* currentRenderStyle = currentElemStyle->as< IfcSchema::IfcSurfaceStyleRendering>();
							if (currentRenderStyle->Transparency() > 0.25) { return true; }
						}
					}
				}
			}
		}

		// find via geometry material
		if (currentStyleRep == nullptr) {
			IfcSchema::IfcProductRepresentation* currentProductRep = ifcProduct->Representation();
			IfcSchema::IfcRepresentation::list::ptr currentRepList = currentProductRep->Representations();
			
			std::vector< IfcSchema::IfcRepresentation*> representationList;
			for (auto repIt = currentRepList->begin(); repIt != currentRepList->end(); ++repIt)
			{
				IfcSchema::IfcRepresentation* currentRep = *repIt;
				if (!currentRep->RepresentationIdentifier()) { continue;  }
				if (currentRep->RepresentationIdentifier().get() != "Body") { continue; }
				if (currentRep->RepresentationType().get() == "MappedRepresentation") // repesentation is used as container
				{
					IfcSchema::IfcRepresentationItem::list::ptr representationSubItemList = currentRep->Items();

					for (auto represenetationSubIt = representationSubItemList->begin(); represenetationSubIt != representationSubItemList->end(); ++represenetationSubIt)
					{
						IfcSchema::IfcRepresentationItem* subRepresentationItem = *represenetationSubIt;
						if (subRepresentationItem->data().type()->name() != "IfcMappedItem") { continue; }
						IfcSchema::IfcMappedItem* currentMappedItem = subRepresentationItem->as<IfcSchema::IfcMappedItem>();
						IfcSchema::IfcRepresentationMap* currentRepMap = currentMappedItem->MappingSource();
						IfcSchema::IfcRepresentation* subRep = currentRepMap->MappedRepresentation();
						representationList.emplace_back(subRep);
					}
					continue;
				}
				representationList.emplace_back(currentRep);
			}


			for (IfcSchema::IfcRepresentation* currentRep : representationList)
			{
				IfcSchema::IfcRepresentationItem::list::ptr representationSubItemList = currentRep->Items();

				for (auto RepresentationSubItemIt = representationSubItemList->begin(); RepresentationSubItemIt != representationSubItemList->end(); ++RepresentationSubItemIt)
				{
					IfcSchema::IfcRepresentationItem* RepresentationSubItem = *RepresentationSubItemIt;
					IfcSchema::IfcStyledItem::list::ptr StyledItemList = RepresentationSubItem->StyledByItem();

					for (auto styledItemIt = StyledItemList->begin(); styledItemIt != StyledItemList->end(); ++styledItemIt)
					{
						IfcSchema::IfcStyledItem* styledItem = *styledItemIt;

#if defined(USE_IFC2x3)
						IfcSchema::IfcPresentationStyleAssignment::list::ptr currenStyleAssList = styledItem->Styles();
#elif defined(USE_IFC4)
						IfcSchema::IfcStyleAssignmentSelect::list::ptr currenStyleAssList = styledItem->Styles();
#endif

						for (auto currenStyleAssIt = currenStyleAssList->begin(); currenStyleAssIt != currenStyleAssList->end(); ++currenStyleAssIt)
						{
#if defined(USE_IFC2x3)
							IfcSchema::IfcPresentationStyleAssignment* currentStyleAss = *currenStyleAssIt;
#elif defined(USE_IFC4)
							IfcSchema::IfcStyleAssignmentSelect* currentStyleAss = *currenStyleAssIt;
#endif
							if (currentStyleAss->data().type()->name() != "IfcSurfaceStyle") { continue; }

							IfcSchema::IfcSurfaceStyle* currentStyle = currentStyleAss->as<IfcSchema::IfcSurfaceStyle>();
							IfcSchema::IfcSurfaceStyleElementSelect::list::ptr elementSurfList = currentStyle->Styles();

							for (auto styleElementIt = elementSurfList->begin(); styleElementIt != elementSurfList->end(); ++styleElementIt)
							{
								IfcSchema::IfcSurfaceStyleElementSelect* currentElemStyle = *styleElementIt;
								if (currentElemStyle->data().type()->name() != "IfcSurfaceStyleRendering") { continue; }
								IfcSchema::IfcSurfaceStyleRendering* currentRenderStyle = currentElemStyle->as< IfcSchema::IfcSurfaceStyleRendering>();
								if (currentRenderStyle->Transparency() > 0.25) { return true; }
							}
						}
					}
				}
			}
		}
#endif
	}
	return false;
}


void helperFunctions::writeToSTEP(const TopoDS_Shape& theShape, const std::string& targetPath)
{
	std::stringstream buffer;
	std::streambuf* originalBuffer = std::cout.rdbuf(buffer.rdbuf());

	STEPControl_Writer writer;
	writer.Transfer(theShape, STEPControl_AsIs);
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());

	std::cout.rdbuf(originalBuffer);
	return;
}

template <typename T>
void helperFunctions::writeToSTEP(const std::vector<T>& theShapeList, const std::string& targetPath)
{
	std::stringstream buffer;
	std::streambuf* originalBuffer = std::cout.rdbuf(buffer.rdbuf());

	STEPControl_Writer writer;
	for (const T& shape : theShapeList) { writer.Transfer(shape, STEPControl_AsIs); }
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());

	std::cout.rdbuf(originalBuffer);
	return;
}

template <typename T>
void helperFunctions::writeToSTEP(const std::vector<std::vector<T>>& theShapeList, const std::string& targetPath)
{
	std::stringstream buffer;
	std::streambuf* originalBuffer = std::cout.rdbuf(buffer.rdbuf());

	STEPControl_Writer writer;
	for (const auto& nestedList : theShapeList)
	{
		for (const T& shape : nestedList) { writer.Transfer(shape, STEPControl_AsIs); }
	}
	
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());

	std::cout.rdbuf(originalBuffer);
	return;
}

template <typename T>
void helperFunctions::writeToOBJ(const T& theShape, const std::string& targetPath)
{
	std::vector<T> shapeList = { theShape };
	writeToOBJ(shapeList, targetPath);
	return;
}

template <typename T>
void helperFunctions::writeToOBJ(const std::vector<T>& theShapeList, const std::string& targetPath)
{
	std::ofstream objFile(targetPath);
	int vertIdxOffset = 1;
	std::vector<std::vector<int>> nestedTriangleIndx;
	std::unordered_map<gp_XYZ, int, gp_XYZ_Hash, gp_XYZ_Equal> vertMap;

	for (const T& theShape : theShapeList)
	{
		for (TopExp_Explorer exp(theShape, TopAbs_FACE); exp.More(); exp.Next()) {
			TopoDS_Face face = TopoDS::Face(exp.Current());
			TopLoc_Location loc;

			auto mesh = BRep_Tool::Triangulation(face, loc);

			if (mesh.IsNull())
			{
				triangulateShape(face);
				mesh = BRep_Tool::Triangulation(face, loc);
			}

			for (int j = 1; j <= mesh.get()->NbTriangles(); j++)
			{
				const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

				std::vector<int> triangleIndx = {};
				for (size_t i = 1; i <= 3; i++)
				{
					gp_XYZ xyz = mesh->Nodes().Value(theTriangle(i)).Transformed(loc).Coord();

					if (vertMap.find(xyz) != vertMap.end())
					{
						triangleIndx.emplace_back(vertMap[xyz]);
						continue;
					}

					objFile << "v " << xyz.X() << " " << xyz.Y() << " " << xyz.Z() << "\n";
					triangleIndx.emplace_back(vertIdxOffset);
					vertMap[xyz] = vertIdxOffset;
					vertIdxOffset++;
				}

				nestedTriangleIndx.emplace_back(triangleIndx);
			}
		}
	}

	for (const auto& triangleList : nestedTriangleIndx)
	{
		objFile << "f";
		for (int indx : triangleList)
		{
			objFile << " " << indx;
		}
		objFile << "\n";
	}
	objFile.close();
	return;
}

template <typename T>
void helperFunctions::writeToOBJ(const std::vector<std::vector<T>>& theShapeList, const std::string& targetPath)
{
	std::vector<T> flattenedSurfList;
	for (const auto& surfList : theShapeList)
	{
		for (const auto& theSurf : surfList)
		{
			flattenedSurfList.emplace_back(theSurf);
		}
	}
	writeToOBJ(flattenedSurfList, targetPath);
	return;
}

double helperFunctions::computeArea(const TopoDS_Shape& theShape)
{
	GProp_GProps gprops;
	BRepGProp::SurfaceProperties(theShape, gprops);
	return gprops.Mass();
}


double helperFunctions::computeArea(const TopoDS_Face& theFace)
{
	GProp_GProps gprops;
	BRepGProp::SurfaceProperties(theFace, gprops);
	return gprops.Mass();
}

void helperFunctions::triangulateShape(const TopoDS_Shape& shape, bool force)
{
	for (TopExp_Explorer faceExpl(shape, TopAbs_FACE); faceExpl.More(); faceExpl.Next())
	{
		TopoDS_Face currentFace = TopoDS::Face(faceExpl.Current());

		if (currentFace.IsNull() || BRep_Tool::Surface(currentFace).IsNull()) {
			continue;
		}

		TopLoc_Location loc;
		Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(currentFace, loc);

		if (!triangulation.IsNull()) {
			if (!force)
			{
				continue;  // No triangulation present, skip to the next face
			}
			BRepTools::Clean(currentFace);
		}

		std::vector<gp_Pnt> uniquePointList =  helperFunctions::getUniquePoints(currentFace);

		if (uniquePointList.size() < 3) { continue; }
		if (uniquePointList.size() == 3)
		{
			gp_Trsf inverseLoc = loc.Transformation().Inverted();
			Handle(Poly_Triangulation) triangulation = new Poly_Triangulation(3, 1, Standard_False);

			TColgp_Array1OfPnt nodes(1, 3);
			nodes.SetValue(1, uniquePointList[0].Transformed(inverseLoc));
			nodes.SetValue(2, uniquePointList[1].Transformed(inverseLoc));
			nodes.SetValue(3, uniquePointList[2].Transformed(inverseLoc));
			triangulation->ChangeNodes() = nodes;

			Poly_Array1OfTriangle triangles(1, 1);  // One triangle at index 1
			triangles.SetValue(1, Poly_Triangle(1, 2, 3));
			triangulation->ChangeTriangles() = triangles;

			BRep_Builder builder;
			builder.UpdateFace(currentFace, triangulation);
			continue;
		}
		double area = computeArea(currentFace);

		for (size_t i = 1; i <= 3; i++)
		{
			double refinement = 1 / i;
			double deflection = 0.01 * refinement;
			double triangleCount = area / (0.5 * deflection * deflection * std::sqrt(3.0));
			if (std::isinf(triangleCount))
			{
				break;
			}

			BRepTools::Clean(currentFace);
			BRepMesh_IncrementalMesh(currentFace, deflection, Standard_False, 0.5 * refinement, Standard_True);
			TopLoc_Location locLocal;
			Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(currentFace, loc);
			if (!BRep_Tool::Triangulation(currentFace, locLocal).IsNull()) {
				break;
			}
		}
	}
	return;
}

TopoDS_Wire helperFunctions::CurveToCompound(const TopoDS_Edge& theEdge)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	double stepLenght = 0.3;

	Standard_Real first, last;
	Handle(Geom_Curve) curve = BRep_Tool::Curve(theEdge, first, last);
	if (curve.IsNull()) return {};
	GeomAdaptor_Curve adaptorCurve(curve, first, last);
	double curveLength = GCPnts_AbscissaPoint::Length(adaptorCurve, first, last);

	int splitSteps = std::ceil(curveLength / stepLenght);
	if (splitSteps < 2) { splitSteps = 3; }

	GCPnts_UniformAbscissa abscissa(adaptorCurve, splitSteps, first, last);
	if (!abscissa.IsDone()) return {};


	BRepBuilderAPI_MakeWire builder;
	for (int i = 1; i < abscissa.NbPoints(); ++i) {
		gp_Pnt p1 = adaptorCurve.Value(abscissa.Parameter(i));
		gp_Pnt p2 = adaptorCurve.Value(abscissa.Parameter(i + 1));

		if (p1.IsEqual(p2, precision))
		{ continue;
		}

		TopoDS_Edge segment = BRepBuilderAPI_MakeEdge(p1, p2);
		builder.Add(segment);
	}

	builder.Build();
	if (builder.IsDone())
	{
		return builder.Wire();
	}
	return {};
}

TopoDS_Wire helperFunctions::replaceCurves(const TopoDS_Wire& theWire)
{
	std::vector<TopoDS_Edge> fixedEdges;
	double precision = SettingsCollection::getInstance().spatialTolerance();
	bool isEdited = false;

	for (BRepTools_WireExplorer expl(theWire); expl.More(); expl.Next()) {
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());

		if (isStraight(currentEdge)) //TODO: make this smarter
		{
			gp_Pnt p1 = getFirstPointShape(currentEdge);
			gp_Pnt p2 = getLastPointShape(currentEdge);

			if (p1.IsEqual(p2, precision))
			{
				continue;
			}

			TopoDS_Edge segment = BRepBuilderAPI_MakeEdge(p1, p2);

			fixedEdges.emplace_back(segment);
			continue;
		}

		isEdited = true;
		TopoDS_Wire straightCurve = CurveToCompound(currentEdge);

		// if the wire is incorrectly ordered the order is reversed
		std::vector<TopoDS_Edge> straightEdgeList;
		for (BRepTools_WireExplorer expl2(straightCurve); expl2.More(); expl2.Next()) {
			TopoDS_Edge straightenedEdge = TopoDS::Edge(expl2.Current());
			straightEdgeList.emplace_back(straightenedEdge);
		}

		if (fixedEdges.empty())
		{
			fixedEdges.insert(std::end(fixedEdges), std::begin(straightEdgeList), std::end(straightEdgeList));
			continue;
		}

		TopoDS_Edge lastEdge = fixedEdges.back();
		TopoDS_Edge	firstEdge = fixedEdges[0];
		TopoDS_Edge connectingEdge = straightEdgeList[0];
		gp_Pnt lp1 = helperFunctions::getFirstPointShape(lastEdge);
		gp_Pnt lp2 = helperFunctions::getLastPointShape(lastEdge);
		gp_Pnt fp1 = helperFunctions::getFirstPointShape(firstEdge);
		gp_Pnt fp2 = helperFunctions::getLastPointShape(firstEdge);
		gp_Pnt sp1 = helperFunctions::getFirstPointShape(connectingEdge);
		gp_Pnt sp2 = helperFunctions::getLastPointShape(connectingEdge);

		if (!lp1.IsEqual(sp1, precision) && !lp1.IsEqual(sp2, precision) &&
			!lp2.IsEqual(sp1, precision) && !lp2.IsEqual(sp2, precision) &&
			!fp1.IsEqual(sp1, precision) && !fp1.IsEqual(sp2, precision) &&
			!fp2.IsEqual(sp1, precision) && !fp2.IsEqual(sp2, precision)
			)
		{
			std::reverse(straightEdgeList.begin(), straightEdgeList.end());
		}
		fixedEdges.insert(std::end(fixedEdges), std::begin(straightEdgeList), std::end(straightEdgeList));
	}
	if (!isEdited) {return theWire; }

	BRepBuilderAPI_MakeWire builder;
	for (const TopoDS_Edge& fixedEdge : fixedEdges)
	{
		builder.Add(fixedEdge);
	}

	builder.Build();
	if (!builder.IsDone())
	{
		return theWire;
	}

	TopoDS_Wire cleanedWire = builder.Wire();

	if (!cleanedWire.Closed())
	{
		//std::cout << "hit" << std::endl;
		//DebugUtils::printPoints(cleanedWire);
		return theWire;
	}

	return cleanedWire;
}

std::vector<TopoDS_Edge> helperFunctions::replaceCurves(const std::vector<TopoDS_Edge>& theEdgeList)
{
	std::vector<TopoDS_Edge> fixedEdges;
	for (const TopoDS_Edge& currentEdge : theEdgeList)
	{
		if (isStraight(currentEdge))
		{
			fixedEdges.emplace_back(currentEdge);
			continue;
		}

		TopoDS_Wire straightCurve = CurveToCompound(currentEdge);

		for (TopExp_Explorer expl2(straightCurve, TopAbs_EDGE); expl2.More(); expl2.Next()) {
			TopoDS_Edge straightenedEdge = TopoDS::Edge(expl2.Current());
			fixedEdges.emplace_back(straightenedEdge);
		}
	}
	return fixedEdges;
}

bool helperFunctions::isStraight(const TopoDS_Edge& theEdge)
{
	Standard_Real first, last;
	Handle(Geom_Curve) curve = BRep_Tool::Curve(theEdge, first, last);

	if (curve.IsNull()) {
		return false;
	}

	Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(curve);
	return !line.IsNull();
}

bool helperFunctions::isStraight(const TopoDS_Wire& theWire)
{
	for (TopExp_Explorer explorer(theWire, TopAbs_EDGE); explorer.More(); explorer.Next())
	{
		const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());

		if (!isStraight(edge))
		{
			return false;
		}
	}
	return true;
}

bool helperFunctions::hasVolume(const bg::model::box<BoostPoint3D>& bbox)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();

	const auto& t1 = bbox.min_corner();
	const auto& t2 = bbox.max_corner();
	if (abs(t1.get<0>() - t2.get<0>()) < precision &&
		abs(t1.get<1>() - t2.get<1>()) < precision &&
		abs(t1.get<2>() - t2.get<2>()) < precision)
	{
		return false;
	}
	return true;
}

bool helperFunctions::isSame(const bg::model::box<BoostPoint3D>& bboxL, const bg::model::box<BoostPoint3D>& bboxR)
{
	BoostPoint3D minPointL = bboxL.min_corner();
	BoostPoint3D maxPointL = bboxL.max_corner();
	BoostPoint3D minPointR = bboxR.min_corner();
	BoostPoint3D maxPointR = bboxR.max_corner();

	if (!pointIsSame(minPointL, minPointR)) { return false; }
	if (!pointIsSame(maxPointL, maxPointR)) { return false; }
	return true;
}

bool helperFunctions::isSame(const TopoDS_Face& faceL, const TopoDS_Face& faceR)
{
	double precision = SettingsCollection::getInstance().spatialTolerance();
	double areaTolerance = SettingsCollection::getInstance().areaTolerance();

	if (abs(computeArea(faceL) - computeArea(faceR)) > areaTolerance)
	{
		return false;
	}
	std::vector<gp_Pnt> uniqueLPoints = getUniquePoints(faceL);
	std::vector<gp_Pnt> uniqueRPoints = getUniquePoints(faceR);

	if (uniqueLPoints.size() != uniqueRPoints.size())
	{
		return false;
	}

	for (const gp_Pnt& uniqueLPoint : uniqueLPoints)
	{
		bool pointFound = false;
		for (const gp_Pnt& uniqueRPoint : uniqueRPoints)
		{
			if (uniqueLPoint.IsEqual(uniqueRPoint, precision))
			{
				pointFound = true;
				break;
			}
		}
		if (!pointFound)
		{
			return false;
		}
	}
	return true;
}

double helperFunctions::computeArea(const gp_Pnt& p0, const gp_Pnt& p1, const gp_Pnt& p2)
{
	gp_Vec v01(p0, p1);
	gp_Vec v02(p0, p2);
	gp_Vec cross = v01.Crossed(v02);
	return 0.5 * cross.Magnitude();
}

int helperFunctions::wireCount(const TopoDS_Face& theFace)
{
	int count = 0;
	for (TopExp_Explorer WireExpl(theFace, TopAbs_WIRE); WireExpl.More(); WireExpl.Next())
	{
		count++;
	}
	return count;
}

std::vector<TopoDS_Face> helperFunctions::removeDubFaces(const std::vector<TopoDS_Face>& inputFaceList, bool fullProcessing)
{
	std::vector<TopoDS_Face> cleanedFaceList;
	std::vector<double> areaList;

	double minArea = SettingsCollection::getInstance().areaTolerance();

	bgi::rtree<Value, bgi::rstar<25>> spatialIndex;
	for (const TopoDS_Face& currentFace : inputFaceList)
	{
		if (computeArea(currentFace) < minArea) { continue; }

		std::vector<Value> qResult;
		qResult.clear();
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		spatialIndex.query(bgi::intersects(
			bbox), std::back_inserter(qResult));

		bool isDub = false;
		for (const auto&  [otherBox, faceIndx] : qResult)
		{
			const TopoDS_Face& otherFace = cleanedFaceList[faceIndx];
			if (!isSame(currentFace, otherFace)) {continue; }
			isDub = true;
			break;

		}
		if (isDub) { continue; }

		spatialIndex.insert(std::make_pair(bbox, cleanedFaceList.size()));
		cleanedFaceList.emplace_back(currentFace);
		areaList.emplace_back(computeArea(currentFace));
	}
	if (!fullProcessing) { return cleanedFaceList; }

	std::vector<TopoDS_Face> filteredFaceList;
	for (size_t i = 0; i < cleanedFaceList.size(); i++)
	{
		const TopoDS_Face& currentFace = cleanedFaceList[i];
		double currentArea = areaList[i];

		std::vector<Value> qResult;
		qResult.clear();
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace, 0.0);
		spatialIndex.query(bgi::intersects(
			bbox), std::back_inserter(qResult));

		bool isSurrounded = false;
		for (const auto& [otherBox, faceIndx] : qResult)
		{
			if (i == faceIndx) { continue; }
			double otherArea = areaList[faceIndx];
			TopoDS_Face otherFace = cleanedFaceList[faceIndx];

			if (currentArea > otherArea) { continue; }
			isSurrounded = true;
			for (TopExp_Explorer expl(currentFace, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				if (!pointOnShape(otherFace, p))
				{
					isSurrounded = false;
					break;
				}
			}
			if (isSurrounded) { 
				break; 
			}
		}
		if (isSurrounded) { continue; }
		filteredFaceList.emplace_back(currentFace);
	}
	return filteredFaceList;
}

std::vector<TopoDS_Face> helperFunctions::getUniqueFaces(const std::vector<TopoDS_Face>& inputFaceList)
{
	std::vector<TopoDS_Face> cleanedFaceList;
	double areaTolerance = SettingsCollection::getInstance().areaTolerance();
	bgi::rtree<Value, bgi::rstar<25>> spatialIndex;
	for (const TopoDS_Face& currentFace : inputFaceList)
	{
		if (computeArea(currentFace) < areaTolerance) { continue; }
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		spatialIndex.insert(std::make_pair(bbox, spatialIndex.size()));
	}

	for (const TopoDS_Face& currentFace : inputFaceList)
	{
		std::vector<Value> qResult;
		qResult.clear();
		bg::model::box <BoostPoint3D> bbox = helperFunctions::createBBox(currentFace);
		spatialIndex.query(bgi::intersects(
			bbox), std::back_inserter(qResult));

		bool isDub = false;
		for (const auto& [otherBox, faceIndx] : qResult)
		{
			const TopoDS_Face& otherFace = inputFaceList[faceIndx];;
			if (currentFace.IsSame(otherFace)) { continue; }
			if (!isSame(currentFace, otherFace)) { continue; }
			isDub = true;
			break;

		}
		if (isDub) { continue; }
		cleanedFaceList.emplace_back(currentFace);

	}
	return cleanedFaceList;
}

TopoDS_Face helperFunctions::plane2Face(const Handle(Geom_Plane)& geoPlane, const double& planeSize)
{
	const gp_Pln& currentPlane = geoPlane->Pln();
	const gp_Pnt& currentOrigin = currentPlane.Location();
	const gp_Dir& currentNormal = currentPlane.Axis().Direction();
	Handle(Geom_Plane) geomPlane = new Geom_Plane(currentPlane);

	Standard_Real UMin = -planeSize;
	Standard_Real UMax = planeSize;
	Standard_Real VMin = -planeSize;
	Standard_Real VMax = planeSize;

	TopoDS_Face occtFace = BRepBuilderAPI_MakeFace(geomPlane, UMin, UMax, VMin, VMax, Precision::Confusion());
	return occtFace;
}

bool helperFunctions::isFlat(const TopoDS_Face& theFace)
{
	TopLoc_Location loc;
	auto mesh = BRep_Tool::Triangulation(theFace, loc);

	if (mesh.IsNull())
	{
		helperFunctions::triangulateShape(theFace);
		mesh = BRep_Tool::Triangulation(theFace, loc);
	}
	if (mesh.IsNull())
	{
		return true;
	}

	gp_Vec tstNormal(0, 0, 0);
	double precision = SettingsCollection::getInstance().spatialTolerance();
	double angularTolerance = SettingsCollection::getInstance().angularTolerance();
	for (int i = 1; i <= mesh.get()->NbTriangles(); i++)
	{
		const Poly_Triangle& theTriangle = mesh->Triangles().Value(i);
		gp_Pnt p1 = mesh->Nodes().Value(theTriangle(1)).Transformed(loc);
		gp_Pnt p2 = mesh->Nodes().Value(theTriangle(2)).Transformed(loc);
		gp_Pnt p3 = mesh->Nodes().Value(theTriangle(3)).Transformed(loc);

		gp_Vec v1(p1, p2);
		gp_Vec v2(p1, p3);

		gp_Vec localNormal = v1.Crossed(v2);
		if (localNormal.Magnitude() < precision)
		{
			continue; 
		}

		if (tstNormal.Magnitude() < precision)
		{
			tstNormal = v1.Crossed(v2);
			continue;
		}

		if (!tstNormal.IsParallel(localNormal, angularTolerance))
		{
			return false;
		}
	}
	return true;
}
