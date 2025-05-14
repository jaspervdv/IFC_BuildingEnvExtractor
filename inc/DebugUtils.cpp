#include "DebugUtils.h"

#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <BRepTools_WireExplorer.hxx>

#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Face.hxx>
#include <BRepCheck_Wire.hxx>

template void DebugUtils::printFaces<TopoDS_Face>(const std::vector<TopoDS_Face>& shape);
template void DebugUtils::printFaces<TopoDS_Shell>(const std::vector<TopoDS_Shell>& shape);
template void DebugUtils::printFaces<TopoDS_Solid>(const std::vector<TopoDS_Solid>& shape);
template void DebugUtils::printFaces<TopoDS_Shape>(const std::vector<TopoDS_Shape>& shape);

template void DebugUtils::WriteToSTEP<TopoDS_Face>(const TopoDS_Face& shape, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shell>(const TopoDS_Shell& shape, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Solid>(const TopoDS_Solid& shape, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shape>(const TopoDS_Shape& shape, const std::string& targetPath);

template void DebugUtils::WriteToSTEP<TopoDS_Face>(const std::vector<TopoDS_Face>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shell>(const std::vector<TopoDS_Shell>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Solid>(const std::vector<TopoDS_Solid>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shape>(const std::vector<TopoDS_Shape>& shapeList, const std::string& targetPath);

template void DebugUtils::WriteToSTEP<TopoDS_Face>(const std::vector<std::vector<TopoDS_Face>>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shell>(const std::vector<std::vector<TopoDS_Shell>>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Solid>(const std::vector<std::vector<TopoDS_Solid>>& shapeList, const std::string& targetPath);
template void DebugUtils::WriteToSTEP<TopoDS_Shape>(const std::vector<std::vector<TopoDS_Shape>>& shapeList, const std::string& targetPath);

template std::string DebugUtils::pointToString3D<gp_Pnt>(const gp_Pnt& currentPoint);
template std::string DebugUtils::pointToString3D<gp_Vec>(const gp_Vec& currentPoint);

template std::string DebugUtils::pointToString2D<gp_Pnt2d>(const gp_Pnt2d& currentPoint);
template std::string DebugUtils::pointToString2D<gp_Vec2d>(const gp_Vec2d& currentPoint);

template<typename T>
std::string DebugUtils::pointToString3D(const T& currentPoint)
{
	return
		std::to_string(currentPoint.X()) + ", " +
		std::to_string(currentPoint.Y()) + ", " +
		std::to_string(currentPoint.Z()) + "\n";
}

template<typename T>
std::string DebugUtils::pointToString2D(const T& currentPoint)
{
	return
		std::to_string(currentPoint.X()) + ", " +
		std::to_string(currentPoint.Y()) + "\n";
}

std::string DebugUtils::faceToString(const TopoDS_Face& currentFace)
{
	std::string currentString = "new\n";
	for (TopExp_Explorer WireExpl(currentFace, TopAbs_WIRE); WireExpl.More(); WireExpl.Next())
	{
		TopoDS_Wire currentWire = TopoDS::Wire(WireExpl.Current());
		for (BRepTools_WireExplorer expl(currentWire); expl.More(); expl.Next()) {
			TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());

			for (TopExp_Explorer expl(currentEdge, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				currentString += pointToString3D(p);
			}
		}
	}

	return currentString;
}

void DebugUtils::printPoint(const gp_Pnt& p) {
	std::cout << pointToString3D(p);
}

void DebugUtils::printPoint(const gp_Pnt2d& p) {
	std::cout << pointToString2D(p);
}

void DebugUtils::printPoint(const std::vector<gp_Pnt>& pList) {
	for (const auto& p : pList)
	{
		printPoint(p);
	}
}

void DebugUtils::printPoint(const BoostPoint3D& p) {
	std::cout << bg::get<0>(p) << ", " << bg::get<1>(p) << ", " << bg::get<2>(p) << "\n";
}

void DebugUtils::printPoint(const gp_Vec& p) {
	std::cout << pointToString3D(p);
}

void DebugUtils::printPoint(const gp_Vec2d& p) {
	std::cout << pointToString2D(p);
}

void DebugUtils::printPoints(const TopoDS_Wire& w)
{
	for (TopExp_Explorer expl(w, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		printPoint(p);
	}
}

template<typename T>
void DebugUtils::printFaces(const std::vector<T>& shape)
{
	for (const TopoDS_Shape& currentShape : shape) { printFaces(currentShape); }
}

void DebugUtils::printFaces(const TopoDS_Shape& shape)
{
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		std::cout << faceToString(TopoDS::Face(expl.Current()));
	}
}

void DebugUtils::WriteToTxt(const std::vector<TopoDS_Face>& shapeList, const std::string& pathString)
{
	std::ofstream outputFile(pathString);

	for (TopoDS_Face shape : shapeList)
	{
		outputFile << faceToString(shape);
	}
	outputFile.close();
}

void DebugUtils::WriteToTxt(const std::vector<TopoDS_Shape>& shapeList, const std::string& pathString)
{
	std::ofstream outputFile(pathString);

	for (TopoDS_Shape shape : shapeList)
	{
		for (TopExp_Explorer explFace(shape, TopAbs_FACE); explFace.More(); explFace.Next())
		{
			outputFile << faceToString(TopoDS::Face(explFace.Current()));
		}
	}
	outputFile.close();
}

void DebugUtils::outPutFaceError(const TopoDS_Face& theShape)
{
	BRepCheck_Analyzer analyzer(theShape);

	if (!analyzer.IsValid()) {
		std::cout << "Face is invalid...\n";
		BRepCheck_Face faceCheck(theShape);

		// Get list of specific errors
		for (const auto& status : faceCheck.Status())
		{
			std::cout << "Face Status: " << status << " : " << checkStatusToString(status) << "\n";
		}
	}
	else
	{
		std::cout << "Face is valid\n";
	}
	std::cout << "Face precision: " << BRep_Tool::Tolerance(theShape) << "\n";
	for (TopExp_Explorer exp(theShape, TopAbs_WIRE); exp.More(); exp.Next()) {
		TopoDS_Wire wire = TopoDS::Wire(exp.Current());
		outputWireError(wire);
	}
	return;
}

void DebugUtils::outputWireError(const TopoDS_Wire& theShape)
{
	BRepCheck_Analyzer analyzer(theShape);

	if (!analyzer.IsValid()) {
		std::cout << "Wire is invalid..." << std::endl;
		BRepCheck_Wire wireCheck(theShape);

		for (const auto& status : wireCheck.Status())
		{
			std::cout << "Wire Status: " << status << std::endl;
		}
	}
	else
	{
		std::cout << "Wire is valid" << std::endl;
	}

	for (TopExp_Explorer expl(theShape, TopAbs_EDGE); expl.More(); expl.Next())
	{
		TopoDS_Edge currentEdge = TopoDS::Edge(expl.Current());
		std::cout << "Edge precision: " << BRep_Tool::Tolerance(currentEdge) << "\n";
	}
	return;
}

std::string DebugUtils::checkStatusToString(const BRepCheck_Status& statusCode)
{
	switch (statusCode) {
	case BRepCheck_NoError:
		return "No error";
	case BRepCheck_InvalidPointOnCurve:
		return "Invalid point on curve";
	case BRepCheck_InvalidPointOnCurveOnSurface:
		return "Invalid point on curve on surface";
	case BRepCheck_InvalidPointOnSurface:
		return "Invalid point on surface";
	case BRepCheck_No3DCurve:
		return "No 3D curve";
	case BRepCheck_Multiple3DCurve:
		return "Multiple 3D curves";
	case BRepCheck_Invalid3DCurve:
		return "Invalid 3D curve";
	case BRepCheck_NoCurveOnSurface:
		return "No curve on surface";
	case BRepCheck_InvalidCurveOnSurface:
		return "Invalid curve on surface";
	case BRepCheck_InvalidCurveOnClosedSurface:
		return "Invalid curve on closed surface";
	case BRepCheck_InvalidSameRangeFlag:
		return "Invalid same range flag";
	case BRepCheck_InvalidSameParameterFlag:
		return "Invalid same parameter flag";
	case BRepCheck_InvalidDegeneratedFlag:
		return "Invalid degenerated flag";
	case BRepCheck_FreeEdge:
		return "Free edge";
	case BRepCheck_InvalidMultiConnexity:
		return "Invalid multi-connexity";
	case BRepCheck_InvalidRange:
		return "Invalid range";
	case BRepCheck_EmptyWire:
		return "Empty wire";
	case BRepCheck_RedundantEdge:
		return "Redundant edge";
	case BRepCheck_SelfIntersectingWire:
		return "Self-intersecting wire";
	case BRepCheck_NoSurface:
		return "No surface";
	case BRepCheck_InvalidWire:
		return "Invalid wire";
	case BRepCheck_RedundantWire:
		return "Redundant wire";
	case BRepCheck_IntersectingWires:
		return "Intersecting wires";
	case BRepCheck_InvalidImbricationOfWires:
		return "Invalid imbrication of wires";
	case BRepCheck_EmptyShell:
		return "Empty shell";
	case BRepCheck_RedundantFace:
		return "Redundant face";
	case BRepCheck_InvalidImbricationOfShells:
		return "Invalid imbrication of shells";
	case BRepCheck_UnorientableShape:
		return "Unorientable shape";
	case BRepCheck_NotClosed:
		return "Not closed";
	case BRepCheck_NotConnected:
		return "Not connected";
	case BRepCheck_SubshapeNotInShape:
		return "Subshape not in shape";
	case BRepCheck_BadOrientation:
		return "Bad orientation";
	case BRepCheck_BadOrientationOfSubshape:
		return "Bad orientation of subshape";
	case BRepCheck_InvalidPolygonOnTriangulation:
		return "Invalid polygon on triangulation";
	case BRepCheck_InvalidToleranceValue:
		return "Invalid tolerance value";
	case BRepCheck_EnclosedRegion:
		return "Enclosed region";
	case BRepCheck_CheckFail:
		return "Check failed";
	default:
		return "Unknown status";
	}
}

template<typename T>
void DebugUtils::WriteToSTEP(const T& shape, const std::string& targetPath) {
	STEPControl_Writer writer;
	writer.Transfer(shape, STEPControl_AsIs);
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());
}

template<typename T>
void DebugUtils::WriteToSTEP(const std::vector<T>& shapeList, const std::string& targetPath) {
	STEPControl_Writer writer;
	for (const T& shape : shapeList) { writer.Transfer(shape, STEPControl_AsIs); }
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());
}

template<typename T>
void DebugUtils::WriteToSTEP(const std::vector<std::vector<T>>& shapeList, const std::string& targetPath) {
	STEPControl_Writer writer;
	for (const std::vector<T>& nestedShape : shapeList)
	{
		for (const T& shape : nestedShape)
		{
			writer.Transfer(shape, STEPControl_AsIs);
		}
	}
	IFSelect_ReturnStatus stat = writer.Write(targetPath.c_str());
}