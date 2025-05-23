#include "helper.h"

#include <string>
#include <vector>
#include <TopoDS.hxx>
#include <BRepCheck_Status.hxx>

#ifndef DEBUGUTILS_DEBUGUTILS_H
#define DEBUGUTILS_DEBUGUTILS_H

struct DebugUtils {
	/// Print point to console
	template<typename T>
	static std::string pointToString3D(const T& currentPoint);

	template<typename T>
	static std::string pointToString2D(const T& currentPoint);

	static std::string faceToString(const TopoDS_Face& currentFace);

	static void printPoint(const gp_Pnt& p);
	static void printPoint(const gp_Pnt2d& p);
	static void printPoint(const std::vector<gp_Pnt>& pList);
	static void printPoint(const BoostPoint3D& p);
	static void printPoint(const gp_Vec& p);
	static void printPoint(const gp_Vec2d& p);
	static void printPoints(const TopoDS_Wire& w);

	/// Print points of the faces to console
	template<typename T>
	static void printFaces(const std::vector<T>& shape);
	static void printFaces(const TopoDS_Shape& shape);

	/// Output shape to step file
	template<typename T>
	static void WriteToSTEP(const T& shape, const std::string& targetPath);

	template<typename T>
	static void WriteToSTEP(const std::vector<T>& shapeList, const std::string& targetPath);

	template<typename T>
	static void WriteToSTEP(const std::vector<std::vector<T>>& shapeList, const std::string& targetPath);

	/// Ouptu shape to txt file
	static void WriteToTxt(const std::vector<TopoDS_Face>& shapeList, const std::string& pathString);
	static void WriteToTxt(const std::vector<TopoDS_Shape>& shapeList, const std::string& pathString);

	/// prints the issues that a face has
	static void outPutFaceError(const TopoDS_Face& theShape);
	/// prints the issues that a wire has
	static void outputWireError(const TopoDS_Wire& theShape);
	/// convert brepcheck_status to cout
	static std::string checkStatusToString(const BRepCheck_Status& statusCode);

	/// prints the shapetype a shape has
	static void outPutShapeType(const TopoDS_Shape& theShape);
	/// convert shapeEnum to cout
	static std::string shapeTypeToString(const TopAbs_ShapeEnum& shapeEnum);
};

#endif // DEBUGUTILS_DEBUGUTILS_H
