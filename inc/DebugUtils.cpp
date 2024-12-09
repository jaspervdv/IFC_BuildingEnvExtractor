#include "DebugUtils.h"

#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>

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
	for (TopExp_Explorer expl(currentFace, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);

		currentString += pointToString3D(p);
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