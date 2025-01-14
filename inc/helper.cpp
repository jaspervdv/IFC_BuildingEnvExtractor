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

std::vector<gp_Pnt> helperFunctions::getPointGridOnSurface(const TopoDS_Face& theface)
{
	SettingsCollection& settingsCollection = SettingsCollection::getInstance(); 
	double precision = settingsCollection.precision();

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
	if (offsettedFace.IsNull()) { return {}; }

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


template<typename T>
void helperFunctions::bBoxDiagonal(const std::vector<T>& theShapeList, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle)
{
	for (const T& theShape : theShapeList) { bBoxDiagonal(theShape, lllPoint, urrPoint, buffer, angle, secondAngle); }
}

template<typename T>
void helperFunctions::bBoxDiagonal(const T& theShape, gp_Pnt* lllPoint, gp_Pnt* urrPoint, const double buffer, const double angle, const double secondAngle)
{
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
	if (abs(urr.X() - lll.X()) < SettingsCollection::getInstance().precision()) { return TopoDS_Solid(); }
	if (abs(urr.Y() - lll.Y()) < SettingsCollection::getInstance().precision()) { return TopoDS_Solid(); }
	if (abs(urr.Z() - lll.Z()) < SettingsCollection::getInstance().precision()) { return TopoDS_Solid(); }
	
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
	// get the vectors of the shape
	std::vector<gp_Pnt> pointList = getPoints(shape);
	gp_Vec hVector = getShapedir(pointList, true);
	gp_Vec vVector = getShapedir(pointList, false);

	// compute horizontal rotation
	gp_Pnt p1 = gp_Pnt(0, 0, 0);
	gp_Pnt p2 = p1.Translated(hVector);

	double angleFlat = 0;
	if (abs(p1.Y() - p2.Y()) > 0.00001)
	{
		double os = abs(p1.Y() - p2.Y()) / p1.Distance(p2);
		angleFlat = asin(os);

		gp_Pnt tempP = helperFunctions::rotatePointPoint(p2, p1, angleFlat);

		if (Abs(p1.X() - tempP.X()) > 0.01 && 
			Abs(p1.Y() - tempP.Y()) > 0.01)
		{
			angleFlat = -angleFlat;
		}
	}

	// compute vertical rotation
	gp_Pnt p3 = gp_Pnt(0, 0, 0);
	gp_Pnt p4 = helperFunctions::rotatePointPoint(p3.Translated(vVector), p3, angleFlat);

	if (abs(p3.X() - p4.X()) < 0.01)
	{
		p3 = helperFunctions::rotatePointWorld(p3, M_PI / 2.0);
		p4 = helperFunctions::rotatePointWorld(p4, M_PI / 2.0);
		angleFlat += M_PI / 2.0;
	}
	double angleVert = acos(abs(p4.Z() - p3.Z()) / p3.Distance(p4));

	gp_Pnt lllPoint;
	gp_Pnt urrPoint;
	helperFunctions::bBoxDiagonal(pointList, &lllPoint, &urrPoint, 0, angleFlat, angleVert);
	if (lllPoint.IsEqual(urrPoint, SettingsCollection::getInstance().precision())) { return TopoDS_Shape(); }

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
		return getTriangleCenter(mesh, theTriangle, loc);
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


gp_Vec helperFunctions::computeEdgeDir(const TopoDS_Edge& theEdge)
{
	double precision = SettingsCollection::getInstance().precision();
	gp_Pnt startpoint = getFirstPointShape(theEdge);
	gp_Pnt endpoint = getLastPointShape(theEdge);

	if (startpoint.IsEqual(endpoint, precision)) { return gp_Vec(0, 0, 0); }
	return gp_Vec(startpoint, endpoint).Normalized();
}

gp_Vec helperFunctions::computeFaceNormal(const TopoDS_Face& theFace) //TODO: check if triangle based would be usefull 
{
	double precision = SettingsCollection::getInstance().precision();

	gp_Vec vec1;
	gp_Vec vec2;

	TopExp_Explorer edgeExpl(theFace, TopAbs_EDGE);
	for (edgeExpl; edgeExpl.More(); edgeExpl.Next())
	{
		TopoDS_Edge edge = TopoDS::Edge(edgeExpl.Current());
		vec1 = computeEdgeDir(edge);
		if (vec1.Magnitude() < precision) { continue; }
		break;
	}
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
	double precision = SettingsCollection::getInstance().precision();

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

gp_Vec helperFunctions::getShapedir(const std::vector<gp_Pnt>& pointList, bool isHorizontal)
{
	std::vector<std::pair<gp_Vec, int>> vecCountMap;
	double precision = SettingsCollection::getInstance().precision();
	for (size_t i = 0; i < pointList.size(); i += 2)
	{
		gp_Pnt p1 = pointList[i];
		gp_Pnt p2 = pointList[i + 1];

		if (isHorizontal)
		{
			p1.SetZ(0);
			p2.SetZ(0);
		}

		if (p1.Distance(p2) < precision) { continue; }
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
	std::pair<gp_Vec, int> RotationVecPair = *vecCountMap.begin();
	for (auto& vecPair : vecCountMap)
	{
		if (RotationVecPair.second < vecPair.second)
		{
			RotationVecPair = vecPair;
		}
	}
	return RotationVecPair.first;
}


bool helperFunctions::shareEdge(const TopoDS_Face& theFace, const TopoDS_Face& theotherFace)
{
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

	double precision = SettingsCollection::getInstance().precision();

	// check if edges are parallel
	gp_Vec currentVec = gp_Vec(cP0, cP1);
	gp_Vec otherVec = gp_Vec(oP0, oP1);
	if (!currentVec.IsParallel(otherVec, precision)) { return false; }

	// if the distance between 3 points of the edges is the same as the full length of one edge. the edges are overlapping
	double currentFullDistance = cP0.Distance(cP1);
	if (abs(currentFullDistance - (cP0.Distance(oP0) + oP0.Distance(cP1))) < precision ||
		abs(currentFullDistance - (cP0.Distance(oP1) + oP1.Distance(cP1))) < precision)
	{
		return true;
	}
	double otherFullDistance = oP0.Distance(oP1);
	if (abs(otherFullDistance - (oP0.Distance(cP0) + cP0.Distance(oP1))) < precision ||
		abs(otherFullDistance - (oP0.Distance(cP1) + cP1.Distance(oP1))) < precision)
	{
		return true;
	}
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

double helperFunctions::tVolume(const gp_Pnt& p, const std::vector<gp_Pnt>& vertices) {
	gp_Pnt vert0 = vertices[0];
	gp_Pnt vert1 = vertices[1];
	gp_Pnt vert2 = vertices[2];

	BoostPoint3D p1(vert0.X() - vert1.X(), vert0.Y() - vert1.Y(), vert0.Z() - vert1.Z());
	BoostPoint3D p2(vert1.X() - p.X(), vert1.Y() - p.Y(), vert1.Z() - p.Z());
	BoostPoint3D p3(vert2.X() - p.X(), vert2.Y() - p.Y(), vert2.Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3)) / 6;
}

bool helperFunctions::triangleIntersecting(const std::vector<gp_Pnt>& line, const std::vector<gp_Pnt>& triangle)
{
	gp_Pnt lineStart = line[0];
	gp_Pnt lineEnd = line[1];

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

	if (abs(leftFinal) < 1e-6 || abs(rightFinal) < 1e-6) { return false; } // if surfaces rest on eachother return 0
	if (!hasSameSign(leftFinal, rightFinal)) { return true; }
	return false;
}

bool helperFunctions::hasSameSign(const double& leftDouble, const double& rightDouble)
{
	if (leftDouble > 0 && rightDouble > 0 || leftDouble < 0 && rightDouble < 0) { return true; }
	return false;
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

std::vector<TopoDS_Face> helperFunctions::mergeFaces(const std::vector<TopoDS_Face>& theFaceList)
{
	double lowPrecision = SettingsCollection::getInstance().precisionCoarse();

	std::vector<gp_Vec> faceNormalList;
	for (const TopoDS_Face surfacePair : theFaceList)
	{
		faceNormalList.emplace_back(helperFunctions::computeFaceNormal(surfacePair));
	}

	if (theFaceList.size() != faceNormalList.size())
	{
		//TODO: add error output
		return {};
	}

	std::vector<int> evalList(faceNormalList.size(), 0);
	std::vector<TopoDS_Face> cleanedFaceCollection;
	bool hasMergedFaces = false;

	for (size_t i = 0; i < faceNormalList.size(); i++) //TODO: indexing?
	{
		if (evalList[i] == 1) { continue; }
		std::vector<TopoDS_Face> mergingPairList;

		TopoDS_Face currentFace = theFaceList[i];
		mergingPairList.emplace_back(currentFace);
		evalList[i] = 1;

		while (true)
		{
			int originalMergeSize = mergingPairList.size();

			for (size_t j = 0; j < faceNormalList.size(); j++)
			{
				if (j == i) { continue; }
				if (evalList[j] == 1) { continue; }

				TopoDS_Face otherFace = theFaceList[j];
				if (!faceNormalList[i].IsParallel(faceNormalList[j], 1e-6)) { continue; }

				// find if the surface shares edge with any of the to merge faces
				bool toMerge = false;
				for (const TopoDS_Face& mergingFace : mergingPairList)
				{
					if (helperFunctions::shareEdge(otherFace, mergingFace))
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

		std::vector<TopoDS_Face> mergedFaceList = mergeCoFaces(mergingPairList);
		for (const TopoDS_Face& mergedFace : mergedFaceList) { cleanedFaceCollection.emplace_back(mergedFace); }
	}
	return cleanedFaceCollection;
}

std::vector<TopoDS_Face> helperFunctions::mergeCoFaces(const std::vector<TopoDS_Face>& theFaceList)
{
	double precision = SettingsCollection::getInstance().precision();

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
			BRepExtrema_DistShapeShape distanceCalc1(
				originalFace,
				BRepBuilderAPI_MakeVertex(currentPoint)
			);
			distanceCalc1.Perform();

			if (distanceCalc1.Value() < precision)
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
			BRepExtrema_DistShapeShape distanceCalc1(
				originalFace,
				BRepBuilderAPI_MakeVertex(currentPoint)
			);
			distanceCalc1.Perform();

			if (distanceCalc1.Value() < precision)
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

	if (p1.Distance(p2) < SettingsCollection::getInstance().precision()) { return baseWire; }

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

std::vector<TopoDS_Face> helperFunctions::outerEdges2Shapes(const std::vector<TopoDS_Edge>& edgeList)
{
	std::vector<TopoDS_Wire> wireList = growWires(edgeList);
	std::vector<TopoDS_Wire> cleanWireList = cleanWires(wireList);
	std::vector<TopoDS_Face> cleanedFaceList = wireCluster2Faces(cleanWireList);
	return cleanedFaceList;
}

std::vector<TopoDS_Wire> helperFunctions::growWires(const std::vector<TopoDS_Edge>& edgeList) {
	std::vector<TopoDS_Wire> wireCollection;
	std::vector<TopoDS_Wire> wireCollectionClosed;
	std::vector<TopoDS_Edge> tempEdgeList;

	//BRepBuilderAPI_MakeWire wireMaker;
	bool loopFound = false;

	TopoDS_Edge currentEdge = edgeList[0];
	std::vector<int> evaluated(edgeList.size());
	evaluated[0] = 1;

	gp_Pnt originPoint = helperFunctions::getFirstPointShape(edgeList[0]); // the original point of the original edge
	gp_Pnt extendingPoint = helperFunctions::getLastPointShape(edgeList[0]); // the point from which will be extended

	tempEdgeList.emplace_back(currentEdge);

	double precision = SettingsCollection::getInstance().precision();
	bool isReversed = false;
	while (true)
	{
		bool hasStapped = false; // true if a stap is found in the while iteration
		bool closed = false; // true if the extensionpoint meets the originpoint

		for (size_t i = 0; i < edgeList.size(); i++)
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges
			TopoDS_Edge otherEdge = edgeList[i];

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


		for (size_t i = 0; i < edgeList.size(); i++) // search next unused edge to create new wire
		{
			if (evaluated[i] != 0) { continue; } // pass over evaluated edges

			originPoint = helperFunctions::getFirstPointShape(edgeList[i]); // the original point of the original edge
			extendingPoint = helperFunctions::getLastPointShape(edgeList[i]); // the point from which will be extended
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

	BRepBuilderAPI_MakeWire wireMaker;
	std::vector<TopoDS_Edge> orderedEdgeList;
	for (TopExp_Explorer edgeExp(wire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
		orderedEdgeList.emplace_back(TopoDS::Edge(edgeExp.Current()));
	}

	std::vector<int> merged(orderedEdgeList.size());

	double precision = SettingsCollection::getInstance().precision();

	bool ttt = false;
	gp_Pnt connection = helperFunctions::getFirstPointShape(orderedEdgeList[0]);
	gp_Vec startingVec = helperFunctions::computeEdgeDir(orderedEdgeList[0]);
	if (startingVec.IsParallel(helperFunctions::computeEdgeDir(orderedEdgeList.back()), precision)) //get another startpoint
	{
		ttt = true;
		for (size_t i = 1; i < orderedEdgeList.size(); i++)
		{
			if (!startingVec.IsParallel(helperFunctions::computeEdgeDir(orderedEdgeList[i]), precision))
			{
				std::rotate(orderedEdgeList.begin(), orderedEdgeList.begin() + i, orderedEdgeList.end());
				break;
			}
		}
	}
	connection = helperFunctions::getFirstPointShape(orderedEdgeList[0]);

	for (size_t i = 0; i < orderedEdgeList.size(); i++) // merge parralel 
	{
		if (merged[i] == 1) { continue; }
		merged[i] = 1;

		TopoDS_Edge currentEdge = orderedEdgeList[i];
		gp_Vec currentVec = helperFunctions::computeEdgeDir(currentEdge);

		if (currentVec.Magnitude() == 0)
		{
			merged[i] = 1;
			continue;
		}

		gp_Pnt endPoint = helperFunctions::getLastPointShape(currentEdge);

		for (size_t j = i + 1; j < orderedEdgeList.size(); j++)
		{
			if (merged[j] == 1) { continue; }

			TopoDS_Edge otherEdge = orderedEdgeList[j];
			gp_Vec otherVec = helperFunctions::computeEdgeDir(otherEdge);

			if (otherVec.Magnitude() == 0)
			{
				merged[j] = 1;
				continue;
			}

			if (currentVec.IsParallel(otherVec, 0.01)) {
				merged[j] = 1;
				continue;
			}

			if (j + 1 == orderedEdgeList.size())
			{
				endPoint = helperFunctions::getFirstPointShape(otherEdge);
				break;
			}

			endPoint = helperFunctions::getFirstPointShape(otherEdge);

			gp_Vec endPointVec = gp_Vec(connection, endPoint);

			if (endPointVec.Magnitude() == 0) { break; }
			else if (!currentVec.IsParallel(endPointVec, 0.01)) { endPoint = helperFunctions::getLastPointShape(otherEdge); }

			break;
		}

		if (connection.IsEqual(endPoint, precision)) { continue; }

		wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, endPoint));
		connection = endPoint;
	}

	if (connection.IsEqual(helperFunctions::getFirstPointShape(orderedEdgeList[0]), precision))
	{
		TopoDS_Wire finalWire = wireMaker.Wire();
		return finalWire;
	}

	gp_Pnt finalPoint = helperFunctions::getFirstPointShape(orderedEdgeList[0]);


	wireMaker.Add(BRepBuilderAPI_MakeEdge(connection, finalPoint));
	TopoDS_Wire finalWire = wireMaker.Wire();
	return finalWire;
}

std::vector<TopoDS_Face> helperFunctions::wireCluster2Faces(const std::vector<TopoDS_Wire>& wireList) {
	BRepBuilderAPI_MakeFace faceBuilder;
	std::vector<TopoDS_Face> faceList;
	gp_Vec normal;
	gp_Pnt originPoint;
	gp_Vec castingVector;

	for (size_t i = 0; i < wireList.size(); i++)
	{
		if (i == 0) // use the first face to compute normal dir 
		{
			while (i < wireList.size())
			{
				gp_Vec vec1;
				gp_Vec vec2;
				for (TopExp_Explorer edgeExp(wireList[i], TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
					TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());

					originPoint = helperFunctions::getFirstPointShape(edge);
					gp_Pnt endpoint = helperFunctions::getLastPointShape(edge);
					vec1 = gp_Vec(originPoint, endpoint);
					vec2 = gp_Vec(originPoint, endpoint);

					while (vec1.IsParallel(vec2, 0.0001))
					{
						edgeExp.Next();
						if (!edgeExp.More()) {
							break;
						}
						edge = TopoDS::Edge(edgeExp.Current());
						gp_Pnt otherStartpoint = helperFunctions::getFirstPointShape(edge);
						gp_Pnt otherEndpoint = helperFunctions::getLastPointShape(edge);
						vec2 = gp_Vec(otherStartpoint, otherEndpoint);
					}
					break;
				}

				if (vec1.IsParallel(vec2, 0.0001))
				{
					i++;
					continue;
				}

				normal = vec1.Crossed(vec2);
				normal.Normalize();
				castingVector = vec1;
				break;
			}
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
	double precision = SettingsCollection::getInstance().precision();

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

				TopoDS_Edge evalEdge = BRepBuilderAPI_MakeEdge(
					p, gp_Pnt(
						p.X() + abs(castingVector.X()) * 10000,
						p.Y() + abs(castingVector.Y()) * 10000,
						p.Z() + abs(castingVector.Z()) * 10000
					));

				int intersectionCount = 0;
				TopExp_Explorer edgeExplorer(orderedFootprintList[i], TopAbs_EDGE);
				for (; edgeExplorer.More(); edgeExplorer.Next()) {
					const TopoDS_Edge& currentEdge = TopoDS::Edge(edgeExplorer.Current());

					BRepExtrema_DistShapeShape distanceCalc1(evalEdge, currentEdge); //TODO: speed up with linear intersection function?
					distanceCalc1.Perform();

					if (distanceCalc1.Value() < precision) { intersectionCount++; }
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
				ErrorCollection::getInstance().addError(ErrorID::propertyNotImplemented, propertyIdName);
			}
		}
	}
	return attributesList;
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
		offset = axisPlacement->Location()->Coordinates()[2];
#endif // DEBUG
	}
	if (localObjectPlacement == nullptr)
	{
		return offset;
	}
	return offset + getObjectZOffset(localObjectPlacement, deepOnly);
}


double helperFunctions::computeArea(const TopoDS_Face& theFace)
{
	GProp_GProps gprops;
	BRepGProp::SurfaceProperties(theFace, gprops);
	return gprops.Mass();
}

void helperFunctions::triangulateShape(const TopoDS_Shape& shape)
{
	auto hasTriangles = BRepTools::Triangulation(shape, 0.01);
	if (!hasTriangles) { auto mesh = BRepMesh_IncrementalMesh(shape, 0.01); }
}

bool helperFunctions::hasVolume(const bg::model::box<BoostPoint3D>& bbox)
{
	auto t1 = bbox.min_corner();
	auto t2 = bbox.max_corner();
	if (abs(t1.get<0>() - t2.get<0>()) < 1e-6 &&
		abs(t1.get<1>() - t2.get<1>()) < 1e-6 &&
		abs(t1.get<2>() - t2.get<2>()) < 1e-6)
	{
		return false;
	}
	return true;
}
