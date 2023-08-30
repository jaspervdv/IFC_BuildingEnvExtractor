#include "helper.h"
#include "floorProcessor.h"

#include <CJToKernel.h>

#include <iostream>
#include <string>
#include <filesystem>
#include <direct.h>

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



void WriteToSTEP(TopoDS_Solid shape, std::string addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";

	STEPControl_Writer writer;

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}

void WriteToSTEP(TopoDS_Shape shape, std::string addition) {
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

void printPoint(gp_Pnt p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << "\n";
}

void printPoint(gp_Pnt2d p) {
	std::cout << p.X() << ", " << p.Y() << "\n";
}

void printPoint(BoostPoint3D p) {
	std::cout << bg::get<0>(p) << ", " << bg::get<1>(p) << ", " << bg::get<2>(p) << "\n";
}

void printPoint(gp_Vec p) {
	std::cout << p.X() << ", " << p.Y() << ", " << p.Z() << "\n";
}

void printPoint(gp_Vec2d p) {
	std::cout << p.X() << ", " << p.Y() << "\n";
}

void printFaces(TopoDS_Shape shape)
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


void addTimeToJSON(nlohmann::json* j, std::string valueName, std::chrono::steady_clock::time_point startTime, std::chrono::steady_clock::time_point endTime)
{

	double duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
	if (duration < 5) { (*j)[valueName + " (ms)"] = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count(); }
	else { (*j)[valueName + " (s)"] = duration; }
}

gp_Pnt rotatePointWorld(gp_Pnt p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}


TopoDS_Wire reversedWire(const TopoDS_Wire& mainWire) {
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

BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}


gp_Pnt rotatePointPoint(gp_Pnt& p, gp_Pnt& anchorP, double angle)
{
	gp_Pnt translatedP = p.Translated(gp_Vec(-anchorP.X(), -anchorP.Y(), -anchorP.Z()));
	gp_Pnt rotatedP = rotatePointWorld(translatedP, angle);
	return rotatedP.Translated(gp_Vec(anchorP.X(), anchorP.Y(), anchorP.Z()));

}
std::tuple<gp_Pnt, gp_Pnt, double> rotatedBBoxDiagonal(std::vector<gp_Pnt> pointList, double angle, double secondAngle = 0) {
	
	bool isPSet = false;
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = rotatePointWorld(pointList[i], angle).Rotated(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0, 1, 0)), -secondAngle);

		if (!isPSet)
		{
			isPSet = true;

			lllPoint = point;
			urrPoint = point;
		}
		else
		{
			if (point.X() < lllPoint.X()) { lllPoint.SetX(point.X()); }
			if (point.Y() < lllPoint.Y()) { lllPoint.SetY(point.Y()); }
			if (point.Z() < lllPoint.Z()) { lllPoint.SetZ(point.Z()); }

			if (point.X() > urrPoint.X()) { urrPoint.SetX(point.X()); }
			if (point.Y() > urrPoint.Y()) { urrPoint.SetY(point.Y()); }
			if (point.Z() > urrPoint.Z()) { urrPoint.SetZ(point.Z()); }
		}
	}

	// compute diagonal distance
	double distance = lllPoint.Distance(urrPoint);

	return std::make_tuple( lllPoint, urrPoint, distance );
}

BoostPoint3D Point3DOTB(gp_Pnt oP){
	return BoostPoint3D(oP.X(), oP.Y(), oP.Z());
}

gp_Pnt Point3DBTO(BoostPoint3D oP) {
	return gp_Pnt(bg::get<0>(oP), bg::get<1>(oP), bg::get<2>(oP));
}

gp_Pnt getLowestPoint(TopoDS_Shape shape, bool areaFilter)
{
	double lowestZ = 9999;
	gp_Pnt lowestP(0, 0, 0);

	TopExp_Explorer expl;

	TopoDS_Shape largestFace = shape;

	if (areaFilter)
	{
		GProp_GProps gprop;
		largestFace;
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

gp_Pnt getHighestPoint(TopoDS_Shape shape)
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

gp_Pnt getPointOnFace(TopoDS_Face theFace) {

	theFace.Reverse();
	int maxGuesses = 10;

	gp_Pnt lll(999999, 999999, 999999);
	gp_Pnt urr(-999999, -999999, -999999);

	for (TopExp_Explorer vertexExp(theFace, TopAbs_VERTEX); vertexExp.More(); vertexExp.Next()) {
		TopoDS_Vertex vertex = TopoDS::Vertex(vertexExp.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		if (p.X() < lll.X()) { lll.SetX(p.X()); }
		if (p.Y() < lll.Y()) { lll.SetY(p.Y()); }
		if (p.Z() < lll.Z()) { lll.SetZ(p.Z()); }
		if (p.X() > urr.X()) { urr.SetX(p.X()); }
		if (p.Y() > urr.Y()) { urr.SetY(p.Y()); }
		if (p.Z() > urr.Z()) { urr.SetZ(p.Z()); }
	}
	//std::cout << "t" << std::endl;
	// Generate a random point on the face
	std::random_device rd;
	std::mt19937 gen(rd());

	std::uniform_real_distribution<> xDistr(lll.X(), urr.X());
	std::uniform_real_distribution<> yDistr(lll.Y(), urr.Y());
	std::uniform_real_distribution<> zDistr(lll.Z(), urr.Z());

	gp_Pnt randomPoint;
	bool isOnFace = false;
	int itt = 0;
	while (!isOnFace) {
		randomPoint.SetXYZ(gp_XYZ(xDistr(gen), yDistr(gen), zDistr(gen)));
		
		BRepExtrema_DistShapeShape distanceCalc(theFace, BRepBuilderAPI_MakeVertex(randomPoint));
		distanceCalc.Perform();

		randomPoint = distanceCalc.PointOnShape1(1);
		isOnFace = true;

		for (TopExp_Explorer wireExp(theFace, TopAbs_WIRE); wireExp.More(); wireExp.Next()) {
			TopoDS_Wire theWire = TopoDS::Wire(wireExp.Current());

			BRepExtrema_DistShapeShape distanceWireCalc(theWire, BRepBuilderAPI_MakeVertex(randomPoint));
			distanceWireCalc.Perform();

			if (distanceWireCalc.Value() < 0.01)
			{
				isOnFace = false;
			}
		}
		if (itt == maxGuesses) { break; } // TODO: prevent hitting this
		itt++;
	}

	return randomPoint;
}

gp_Pnt& getFirstPointShape(const TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex startVertex;

	if (vertexExplorer.More()) {
		startVertex = TopoDS::Vertex(vertexExplorer.Current());
	}

	gp_Pnt startPoint = BRep_Tool::Pnt(startVertex);
	return startPoint;
}


gp_Pnt& getLastPointShape(const TopoDS_Shape& shape) {
	TopExp_Explorer vertexExplorer(shape, TopAbs_VERTEX);
	TopoDS_Vertex endVertex;

	while (vertexExplorer.More()) {
		endVertex = TopoDS::Vertex(vertexExplorer.Current());
		vertexExplorer.Next();
	}
	return BRep_Tool::Pnt(endVertex);;
}


gp_Vec computeFaceNormal(const TopoDS_Face& theFace)
{
	gp_Vec vec1;
	gp_Vec vec2;

	bool hasVec1 = false;
	bool hasVec2 = false;


	for (TopExp_Explorer vertexExp(theFace, TopAbs_EDGE); vertexExp.More(); vertexExp.Next()) {
		TopoDS_Edge edge = TopoDS::Edge(vertexExp.Current());

		gp_Pnt startpoint = getFirstPointShape(edge);
		gp_Pnt endpoint = getLastPointShape(edge);

		if (startpoint.IsEqual(endpoint, 1e-6)) { continue; }

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

std::vector<TopoDS_Face> getRoomFootprint(TopoDS_Shape shape)
{
	TopExp_Explorer expl;
	std::vector<TopoDS_Face> faceList;
	double lowestZ = 9999;
	int lowestFaceIndx = -1;

	for (expl.Init(shape, TopAbs_FACE); expl.More(); expl.Next()) { faceList.emplace_back(TopoDS::Face(expl.Current())); }

	// get horizontal faces
	std::vector<TopoDS_Face> horizontalFaceList;
	std::vector<double> horizontalFaceHeight;
	double totalHeight = 0;

	for (int j = 0; j < faceList.size(); j++)
	{
		double z = 0;
		int c = 0;
		bool flat = true;

		for (expl.Init(faceList[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);

			if (c == 0)
			{
				z = p.Z();
				c++;
				continue;
			}

			if (z > p.Z() + 0.01 || z < p.Z() - 0.01)
			{
				flat = false;
				break;
			}
		}

		if (flat)
		{
			horizontalFaceList.emplace_back(faceList[j]);
			horizontalFaceHeight.emplace_back(z);
			totalHeight += z;
		}
	}

	double averageHeigth = totalHeight / horizontalFaceList.size();

	std::vector<TopoDS_Face> footprint;
	for (size_t i = 0; i < horizontalFaceHeight.size(); i++)
	{
		if (horizontalFaceHeight[i] < averageHeigth)
		{
			footprint.emplace_back(horizontalFaceList[i]);
		}
	}

	return footprint;
}

std::vector<IfcSchema::IfcProduct*> getNestedProducts(IfcSchema::IfcProduct* product) {
	
	std::vector<IfcSchema::IfcProduct*> productList;

	if (!product->hasRepresentation())
	{
#ifdef USE_IFC4
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() == 0) { return {}; }

		for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {

#ifdef USE_IFC4
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
	else if (product->hasRepresentation())
	{
		productList.emplace_back(product);
	}

	return productList;
}

bool testSolid(TopoDS_Shape shape) 
{
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next())
	{
		return true;
	}
	return false;
}

double tVolume(gp_Pnt p, const std::vector<gp_Pnt> vertices) {
	BoostPoint3D p1(vertices[0].X() - vertices[1].X(), vertices[0].Y() - vertices[1].Y(), vertices[0].Z() - vertices[1].Z());
	BoostPoint3D p2(vertices[1].X() - p.X(), vertices[1].Y() - p.Y(), vertices[1].Z() - p.Z());
	BoostPoint3D p3(vertices[2].X() - p.X(), vertices[2].Y() - p.Y(), vertices[2].Z() - p.Z());

	return bg::dot_product(p1, bg::cross_product(p2, p3)) / 6;
}

bool triangleIntersecting(const std::vector<gp_Pnt> line, const std::vector<gp_Pnt> triangle)
{
	// check for potential intersection
	double left = tVolume(triangle[0], { triangle[2], line[0], line[1] });
	double right = tVolume(triangle[1], { triangle[2], line[0], line[1] });

	double left2 = tVolume(triangle[1], { triangle[0], line[0], line[1] });
	double right2 = tVolume(triangle[2], { triangle[0], line[0], line[1] });

	double left3 = tVolume(triangle[2], { triangle[1], line[0], line[1] });
	double right3 = tVolume(triangle[0], { triangle[1], line[0], line[1] });


	if (left > 0 && right > 0 || left < 0 && right < 0) { return false; }
	if (left2 > 0 && right2 > 0 || left2 < 0 && right2 < 0) { return false; }
	if (left3 > 0 && right3 > 0 || left3 < 0 && right3 < 0) { return false; }

	double leftFinal = tVolume(line[0], triangle);
	double rightFinal = tVolume(line[1], triangle);

	if (leftFinal > 0 && rightFinal < 0 || rightFinal > 0 && leftFinal < 0) { return true; }
	return false;
}

std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> checkConnection(TopoDS_Shape roomShape, IfcSchema::IfcSpace* room, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList) {
	TopExp_Explorer expl;

	// get rel space boundaries
	BRepExtrema_DistShapeShape distanceMeasurer;
	distanceMeasurer.LoadS1(roomShape);

	std::vector<IfcSchema::IfcRelSpaceBoundary*> boundaryList;
	std::vector<IfcSchema::IfcDoor*> connectedDoors; 
	std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> newqProductList;

	for (size_t j = 0; j < qProductList.size(); j++)
	{
		bool isConnected = true;

		// find objects that are close
		IfcSchema::IfcProduct* qProduct = std::get<0>(qProductList[j]);
		TopoDS_Shape qShape = std::get<1>(qProductList[j]);

		distanceMeasurer.LoadS2(qShape);

		distanceMeasurer.Perform();

		double distance = distanceMeasurer.Value();

		if (distance > 0.5)
		{
			isConnected = false;
		}
		else if (distance < 0.0001) // if distance is 0 it is known it is connected
		{
			isConnected = true;


		}
		else if (distance < 0.5)
		{
			int counter = 0;

			double x1 = 0;
			double x2 = 0;
			double y1 = 0;
			double y2 = 0;
			double z1 = 0;
			double z2 = 0;

			for (size_t k = 1; k < distanceMeasurer.NbSolution(); k++)
			{
				gp_Pnt p1 = distanceMeasurer.PointOnShape1(k);
				gp_Pnt p2 = distanceMeasurer.PointOnShape2(k);

				x1 += p1.X();
				x2 += p2.X();
				y1 += p1.Y();
				y2 += p2.Y();
				z1 += p1.Z();
				z2 += p2.Z();

				counter++;
			}

			gp_Pnt p1n(x1 / counter, y1 / counter, z1 / counter);
			gp_Pnt p2n(x2 / counter, y2 / counter, z2 / counter);

			if (isnan(p1n.X()) || isnan(p1n.Y()) || isnan(p1n.Z()) ||
				isnan(p2n.X()) || isnan(p2n.Y()) || isnan(p2n.Z()))
			{
				isConnected = false;
			}
			else
			{
				TopoDS_Edge shortestEdge = BRepBuilderAPI_MakeEdge(p1n, p2n);

				for (size_t k = 0; k < qProductList.size(); k++)
				{
					if (j == k) { continue; }

					IfcSchema::IfcProduct* matchingProduct = std::get<0>(qProductList[k]);

					IntTools_EdgeFace intersector;
					intersector.SetEdge(shortestEdge);

					for (expl.Init(std::get<1>(qProductList[k]), TopAbs_FACE); expl.More(); expl.Next()) {
						intersector.SetFace(TopoDS::Face(expl.Current()));
						intersector.Perform();

						if (intersector.CommonParts().Size() > 0) {
							isConnected = false;
							break;
						}
					}
					if (!isConnected)
					{
						break;
					}
				}
			}
		}
		if (isConnected)
		{
			newqProductList.emplace_back(qProductList[j]);
		}
	}
	return newqProductList;
}


std::vector<IfcSchema::IfcRelSpaceBoundary*> makeSpaceBoundary(IfcSchema::IfcSpace* room, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList) {
	
	std::vector<IfcSchema::IfcRelSpaceBoundary*> boundaryList;

	for (size_t j = 0; j < qProductList.size(); j++)
	{
		bool isConnected = true;

		// find objects that are close
		IfcSchema::IfcProduct* qProduct = std::get<0>(qProductList[j]);


		IfcSchema::IfcRelSpaceBoundary* boundary = new IfcSchema::IfcRelSpaceBoundary(
			IfcParse::IfcGlobalId(),
			0,
			std::string(""),
			std::string(""),
			room,
			qProduct->as<IfcSchema::IfcElement>(),
			0,
			IfcSchema::IfcPhysicalOrVirtualEnum::IfcPhysicalOrVirtual_PHYSICAL,
			IfcSchema::IfcInternalOrExternalEnum::IfcInternalOrExternal_INTERNAL
		);

		boundaryList.emplace_back(boundary);

	}
	return boundaryList;
}

helper::helper(std::string path) {

	findSchema(path);

	IfcParse::IfcFile*  sourceFile = new IfcParse::IfcFile(path);

	bool good = sourceFile->good();

	if (!good) 
	{ 
		std::cout << "Unable to parse .ifc file" << std::endl; 
	}
	else { 
		std::cout << "- Valid IFC file found" << std::endl; 
		std::cout << std::endl;
		
		file_ = sourceFile;

		kernel_ = new IfcGeom::Kernel(file_);

		helper::setUnits(file_);
	}
}


bool findSchema(std::string path, bool quiet) {
	std::ifstream infile(path);
	std::string line;

	while (std::getline(infile, line))
	{
		if (line[0] == '#') {
			if (!quiet)
			{
				std::cout << "[Warning] No valid ifc scheme found" << std::endl;
			}
			infile.close();
			return false; 
		}
		if (line.find("FILE_SCHEMA(('IFC4'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC2X3'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC2X3" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA (('IFC2X3'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC2X3" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x1'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x1" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x2'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x2" << std::endl;
			}
			break;
		}
		else if (line.find("FILE_SCHEMA(('IFC4x3'))") != std::string::npos) {
			if (!quiet)
			{
				std::cout << "- Valid scheme found: IFC4x3" << std::endl;
			}	
			break;
		}
	}
	infile.close();
	return true;
}


std::vector<gp_Pnt> helper::getAllPoints(IfcSchema::IfcProduct::list::ptr products)
{
	std::vector<gp_Pnt> pointList;
	for (auto it = products->begin(); it != products->end(); ++it) {

		IfcSchema::IfcProduct* product = *it;
		std::string typeName = product->data().type()->name();

		if (!product->hasRepresentation()) { continue; }

		if (typeName != "IfcRoof" && 
			typeName != "IfcSlab" && 
			typeName != "IfcWall" &&
			typeName != "IfcWallStandardCase") //TODO: remove
		{
			continue; 
		}

		std::vector<gp_Pnt> temp = getObjectPoints(product);

		if (temp.size() > 1);
		{
			for (size_t i = 0; i < temp.size(); i++) { 
				pointList.emplace_back(temp[i]); 
			}
		}
		temp.clear();

	}

	return pointList;
}

template<typename T>
std::vector<gp_Pnt> helper::getAllTypePoints(T &typePtr)
{
	std::vector<gp_Pnt> pointList;
	for (auto it = typePtr->begin(); it != typePtr->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;
		std::vector<gp_Pnt> temp = getObjectPoints(product);

		if (temp.size() > 1);
		{
			for (size_t i = 0; i < temp.size(); i++) {
				pointList.emplace_back(std::move(temp[i]));
			}
		}
		temp.clear();
	}
	return pointList;
}

bool helper::hasSetUnits() {
	if (!length_ || !area_ || !volume_){ return false; }
	else { return true;  }
}


void helper::setUnits(IfcParse::IfcFile *file) {
	double length = 0;
	double area = 0;
	double volume = 0;
	
	IfcSchema::IfcUnitAssignment::list::ptr presentUnits = file->instances_by_type<IfcSchema::IfcUnitAssignment>();
	if (presentUnits.get()->size() == 0) {
		std::cout << "[Error] No unit assignment has been found" << std::endl;
		return;
	}
	else if (presentUnits.get()->size() > 1)
	{
		std::cout << "[Error] Multiple unit assignments have been found" << std::endl;
		return;
	}


	for (IfcSchema::IfcUnitAssignment::list::it it = presentUnits->begin(); it != presentUnits->end(); ++it)
	{
		const IfcSchema::IfcUnitAssignment* itUnits = *it;
		auto units = itUnits->Units();

		for (auto et = units.get()->begin(); et != units.get()->end(); et++) {
			auto unit = *et;

			if (unit->declaration().type() == 902 || unit->declaration().type() == 765) // select the IfcSIUnit
			{
				std::string unitType = unit->data().getArgument(1)->toString();
				std::string SiUnitBase = unit->data().getArgument(3)->toString();
				std::string SiUnitAdd = unit->data().getArgument(2)->toString();

				if (unitType == ".LENGTHUNIT.")
				{
					if (SiUnitBase == ".METRE.") { length = 1; }
					if (SiUnitAdd == ".MILLI.") { length = length / 1000; }
				}
				else if (unitType == ".AREAUNIT.")
				{
					if (SiUnitBase == ".SQUARE_METRE.") { area = 1; }
					if (SiUnitAdd == ".MILLI.") { area = area / pow(1000, 2); }
				}
				if (unitType == ".VOLUMEUNIT.")
				{
					if (SiUnitBase == ".CUBIC_METRE.") { volume = 1; }
					if (SiUnitAdd == ".MILLI.") { volume = volume / pow(1000, 3); }
				}
			}
		}
	}

	// check if units have been found
	std::cout << "[INFO] found units:" << std::endl;
	if (!length)
	{
		std::cout << "[Error] SI unit for length cannot be found!" << std::endl;
		return;
	}
	else if (length == 1) { std::cout << "- Lenght in metre" << std::endl; }
	else if (length == 0.001) { std::cout << "- Lenght in millimetre" << std::endl; }

	if (!area)
	{
		std::cout << "[Error] SI unit for area cannot be found!" << std::endl;
		return;
	}
	else if (area == 1) { std::cout << "- Area in square metre" << std::endl; }
	else if (area == 0.000001) { std::cout << "- Area in square millimetre" << std::endl; }

	if (!volume)
	{
		std::cout << "[Warning] SI unit for volume cannot be found!" << std::endl;
		std::cout << "[Warning] SI unit for volume is set to cubic metre!" << std::endl;
		volume = 1;
	}
	else if (volume == 1) { std::cout << "- Volume in cubic metre" << std::endl; }
	else if (volume == 0.000000001) { std::cout << "- Volume in cubic millimetre" << std::endl; }

	std::cout << std::endl;

	//internalize the data
	length_ = length;
	area_ = area;
	volume_ = volume;
}

void helper::internalizeGeo()
{
	std::cout << "[INFO] Internalizing Geometry of Construction Model\n" << std::endl;
	int faceCount = 0;
	// get products
	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	objectCount = products->size();
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;
		if (product->data().type()->name() == "IfcBuildingElementProxy") { proxyCount++; }
		objectCount ++;
	}

	if (proxyCount > 0)
	{
		hasProxy = true;
	}
	if (proxyCount/objectCount >= maxProxyP)
	{
		hasLotProxy = true;
	} 

	// get a point to translate the model to
	IfcSchema::IfcSlab::list::ptr slabList = file_->instances_by_type<IfcSchema::IfcSlab>();

	gp_Pnt lllPointSite;
	bool hasSitePoint = false;

	for (auto it = slabList->begin(); it != slabList->end(); ++it) {
		IfcSchema::IfcSlab* slab = *it;

		TopoDS_Shape siteShape = getObjectShape(slab, false, false);
		for (TopExp_Explorer vertexExplorer(siteShape, TopAbs_VERTEX); vertexExplorer.More(); vertexExplorer.Next())
		{
			const TopoDS_Vertex& vertex = TopoDS::Vertex(vertexExplorer.Current());
			gp_Pnt vertexPoint = BRep_Tool::Pnt(vertex);

			if (!hasSitePoint)
			{
				lllPointSite = vertexPoint;
				hasSitePoint = true;
			}

			if (vertexPoint.X() < lllPointSite.X()) { lllPointSite.SetX(vertexPoint.X());}
			if (vertexPoint.Y() < lllPointSite.Y()) { lllPointSite.SetY(vertexPoint.Y());}
			if (vertexPoint.Z() < lllPointSite.Z()) { lllPointSite.SetZ(vertexPoint.Z());}

		}
		break;
	}
	objectTranslation_.SetTranslationPart(gp_Vec(-lllPointSite.X(), -lllPointSite.Y(), 0));

	//std::cout << "in" << std::endl;
	auto startTime = std::chrono::high_resolution_clock::now();
	std::vector<gp_Pnt> pointListWall = getAllTypePoints<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
	//std::cout << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
	std::vector<gp_Pnt> pointListWallSt = getAllTypePoints<IfcSchema::IfcWallStandardCase::list::ptr>(file_->instances_by_type<IfcSchema::IfcWallStandardCase>());
	//std::cout << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
	std::vector<gp_Pnt> pointListRoof = getAllTypePoints<IfcSchema::IfcRoof::list::ptr>(file_->instances_by_type<IfcSchema::IfcRoof>());
	//std::cout << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
	std::vector<gp_Pnt> pointLisSlab = getAllTypePoints<IfcSchema::IfcSlab::list::ptr>(file_->instances_by_type<IfcSchema::IfcSlab>());
	//std::cout << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
	std::vector<gp_Pnt> pointListWindow = getAllTypePoints<IfcSchema::IfcWindow::list::ptr>(file_->instances_by_type<IfcSchema::IfcWindow>());
	//std::cout << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
	//std::cout << "out" << std::endl;

	std::vector<gp_Pnt> pointList;
	pointList.reserve(pointListWall.size() + pointListWallSt.size() + pointListRoof.size() + pointLisSlab.size() + pointListWindow.size());

	pointList.insert(pointList.end(), pointListWall.begin(), pointListWall.end());
	pointList.insert(pointList.end(), pointListWallSt.begin(), pointListWallSt.end());
	pointList.insert(pointList.end(), pointListRoof.begin(), pointListRoof.end());
	pointList.insert(pointList.end(), pointLisSlab.begin(), pointLisSlab.end());
	pointList.insert(pointList.end(), pointListWindow.begin(), pointListWindow.end());

	// approximate smalles bbox
	double angle = 22.5 * (M_PI / 180);
	double rotation = 0;
	int maxIt = 15;

	// set data for a bbox
	auto base = rotatedBBoxDiagonal(pointList, rotation);
	gp_Pnt lllPoint = std::get<0>(base);
	gp_Pnt urrPoint = std::get<1>(base);
	double smallestDistance = std::get<2>(base);


	for (size_t i = 0; i < maxIt; i++)
	{
		std::tuple<gp_Pnt, gp_Pnt, double> left;
		std::tuple<gp_Pnt, gp_Pnt, double> right;

		left = rotatedBBoxDiagonal(pointList, rotation - angle);
		right = rotatedBBoxDiagonal(pointList, rotation + angle);
	
		if (std::get<2>(left) > std::get<2>(right) && smallestDistance > std::get<2>(right))
		{ 
			rotation = rotation + angle; 
			smallestDistance = std::get<2>(right);
			lllPoint = std::get<0>(right);
			urrPoint = std::get<1>(right);
		}
		else  if (smallestDistance > std::get<2>(left))
		{ 
			rotation = rotation - angle;
			smallestDistance = std::get<2>(left);
			lllPoint = std::get<0>(left);
			urrPoint = std::get<1>(left);
		}
		
		//make angle smaller
		angle = angle / 2;
	}

	lllPoint_ = lllPoint;
	urrPoint_ = urrPoint;
	originRot_ = rotation;
	hasGeo = true;
}

void helper::internalizeGeo(double angle, gp_Trsf objectTranslation) {
	std::cout << "Internalizing Geometry\n" << std::endl;
	originRot_ = angle;
	objectTranslation_ = objectTranslation;

	std::vector<gp_Pnt> pointListWall = getAllTypePoints<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
	std::vector<gp_Pnt> pointListWallSt = getAllTypePoints<IfcSchema::IfcWallStandardCase::list::ptr>(file_->instances_by_type<IfcSchema::IfcWallStandardCase>());
	std::vector<gp_Pnt> pointListRoof = getAllTypePoints<IfcSchema::IfcRoof::list::ptr>(file_->instances_by_type<IfcSchema::IfcRoof>());
	std::vector<gp_Pnt> pointListWindow = getAllTypePoints<IfcSchema::IfcWindow::list::ptr>(file_->instances_by_type<IfcSchema::IfcWindow>());

	std::vector<gp_Pnt> pointList;
	pointList.reserve(pointListWall.size() + pointListWallSt.size() + pointListRoof.size() + pointListWindow.size());

	pointList.insert(pointList.end(), pointListWall.begin(), pointListWall.end());
	pointList.insert(pointList.end(), pointListWallSt.begin(), pointListWallSt.end());
	pointList.insert(pointList.end(), pointListRoof.begin(), pointListRoof.end());
	pointList.insert(pointList.end(), pointListWindow.begin(), pointListWindow.end());

	auto bbox = rotatedBBoxDiagonal(pointList, angle);

	lllPoint_ = std::get<0>(bbox);
	urrPoint_ = std::get<1>(bbox);


	hasGeo = true;
}

void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location

	if (!hasIndex_)
	{
		std::cout << "- Create Spatial Index" << std::endl;
		if (!useCustomFull)
		{
			// add the floorslabs to the rtree
			auto startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcSlab::list::ptr>(file_->instances_by_type<IfcSchema::IfcSlab>());
			std::cout << "\tIfcSlab objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcRoof::list::ptr>(file_->instances_by_type<IfcSchema::IfcRoof>());
			std::cout << "\tIfcRoof objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the walls to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
			std::cout << "\tIfcWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcWallStandardCase::list::ptr>(file_->instances_by_type<IfcSchema::IfcWallStandardCase>());
			std::cout << "\tIfcWallStandardCase objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcCovering::list::ptr>(file_->instances_by_type<IfcSchema::IfcCovering>());
			std::cout << "\tIfcCovering objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the columns to the rtree TODO sweeps
			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcColumn::list::ptr>(file_->instances_by_type<IfcSchema::IfcColumn>());
			std::cout << "\tIfcColumn objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the beams to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcBeam::list::ptr>(file_->instances_by_type<IfcSchema::IfcBeam>());
			std::cout << "\tIfcBeam objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add the curtain walls to the rtree
			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcCurtainWall>());
			std::cout << "\tIfcCurtainWall objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcPlate::list::ptr>(file_->instances_by_type<IfcSchema::IfcPlate>());
			std::cout << "\tIfcPlate objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcMember::list::ptr>(file_->instances_by_type<IfcSchema::IfcMember>());
			std::cout << "\tIfcMember objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			// add doors to the rtree (for the appartment detection)
			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
			std::cout << "\tIfcDoor objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			startTime = std::chrono::high_resolution_clock::now();
			addObjectToIndex<IfcSchema::IfcWindow::list::ptr>(file_->instances_by_type<IfcSchema::IfcWindow>());
			std::cout << "\tIfcWindow objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

			if (useProxy)
			{
				startTime = std::chrono::high_resolution_clock::now();
				addObjectToIndex<IfcSchema::IfcBuildingElementProxy::list::ptr>(file_->instances_by_type<IfcSchema::IfcBuildingElementProxy>());
				std::cout << "\tIfcBuildingElementProxy objects finished in: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;
			}

		}
		if (useCustom)
		{
			IfcSchema::IfcProduct::list::ptr productList = file_->instances_by_type<IfcSchema::IfcProduct>();
			
			for (auto it = roomBoundingObjects_->begin(); it != roomBoundingObjects_->end(); ++it) {
				
				auto startTime = std::chrono::high_resolution_clock::now();

				IfcSchema::IfcProduct::list::ptr selectedlist(new IfcSchema::IfcProduct::list);

				for (auto et = productList->begin(); et != productList->end(); ++et)
				{
					IfcSchema::IfcProduct* product = *et;

					if (*it == boost::to_upper_copy<std::string>(product->data().type()->name()))
					{
						selectedlist.get()->push(product);
					}
				}

				std::cout << "\t" + *it + " objects finished in : " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "s" << std::endl;

				addObjectToIndex<IfcSchema::IfcProduct::list::ptr>(selectedlist);
			}
		}
		std::cout << std::endl;
		// find valid voids
		applyVoids();
		hasIndex_ = true;
	}

	// =================================================================================================
	
	if (!hasCIndex_)
	{
		// connectivity objects indexing
		addObjectToCIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
		addObjectToCIndex<IfcSchema::IfcStair::list::ptr>(file_->instances_by_type<IfcSchema::IfcStair>());

		hasCIndex_ = true;
	}

	// =================================================================================================
	
	if (!hasRIndex_)
	{
		// room objects indexing
		addObjectToRIndex<IfcSchema::IfcSpace::list::ptr>(file_->instances_by_type<IfcSchema::IfcSpace>());

		hasRIndex_ = true;
	}
	
	// =================================================================================================

}

void helper::indexRooms()
{
	if (!hasRIndex_)
	{
		// room objects indexing
		addObjectToRIndex<IfcSchema::IfcSpace::list::ptr>(file_->instances_by_type<IfcSchema::IfcSpace>());

		hasRIndex_ = true;
	}
}

void helper::indexConnectiveShapes()
{
	if (!hasCIndex_)
	{
		// connectivity objects indexing
		addObjectToCIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
		addObjectToCIndex<IfcSchema::IfcStair::list::ptr>(file_->instances_by_type<IfcSchema::IfcStair>());

		hasCIndex_ = true;
	}
}


bg::model::box < BoostPoint3D > helper::makeObjectBox(IfcSchema::IfcProduct* product)
{
	std::vector<gp_Pnt> productVert = getObjectPoints(product);
	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({0,0,0}, {0,0,0}); }

	// only outputs 2 corners of the three needed corners!
	auto box = rotatedBBoxDiagonal(productVert, originRot_);

	BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(box));
	BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(box));

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}

bg::model::box < BoostPoint3D > helper::makeObjectBox(const std::vector<IfcSchema::IfcProduct*> products)
{
	std::vector<gp_Pnt> productVert;

	for (size_t i = 0; i < products.size(); i++)
	{
		std::vector<gp_Pnt> singleVerts = getObjectPoints(products[i]);

		for (size_t j = 0; j < singleVerts.size(); j++)
		{
			productVert.emplace_back(singleVerts[j]);
		}
	}

	if (!productVert.size() > 1) { return bg::model::box < BoostPoint3D >({ 0,0,0 }, { 0,0,0 }); }

	// only outputs 2 corners of the three needed corners!
	auto box = rotatedBBoxDiagonal(productVert, originRot_);

	BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(box));
	BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(box));

	return bg::model::box < BoostPoint3D >(boostlllpoint, boosturrpoint);
}

TopoDS_Solid helper::makeSolidBox(gp_Pnt lll, gp_Pnt urr, double angle, double extraAngle)
{
	gp_Ax1 vertRotation(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));

	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidbox;
	brepBuilder.MakeSolid(solidbox);

	gp_Pnt p0(rotatePointWorld(lll.Rotated(vertRotation, extraAngle), -angle));
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()).Rotated(vertRotation, extraAngle), -angle);

	gp_Pnt p4(rotatePointWorld(urr.Rotated(vertRotation, extraAngle), -angle));
	gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);
	gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()).Rotated(vertRotation, extraAngle), -angle);

	bool sameXCoordinate = true;
	bool sameYCoordinate = true;
	bool sameZCoordinate = true;

	double xCoord = p0.X();
	double yCoord = p0.Y();
	double zCoord = p0.Z();
	for (const auto& p : { p1, p2, p3, p4, p5, p6, p7 }) {
		if (abs(p.X() - xCoord) > 0.1) { sameXCoordinate = false; }
		if (abs(p.Y() - yCoord) > 0.1) { sameYCoordinate = false; }
		if (abs(p.Z() - zCoord) > 0.1) { sameZCoordinate = false; }

		if (!sameXCoordinate && !sameYCoordinate && !sameZCoordinate) { break; }
	}

	if (sameXCoordinate || sameYCoordinate || sameZCoordinate)
	{
		return TopoDS_Solid(); //TODO: find a solution for this issue 
	}


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
	brepBuilder.Add(solidbox, brepSewer.SewedShape());

	return solidbox;
}

void helper::correctRooms() 
{
	if (!hasRIndex_)
	{
		indexRooms();
	}

	// get all rooms in a cluser
	std::vector<roomLookupValue> roomValues = getFullRLookup();

	// compute volume of every room
	std::vector<double> roomVolume;
	GProp_GProps gprop;

	// get point inside of each room
	for (size_t i = 0; i < roomValues.size(); i++)
	{
		BRepGProp::VolumeProperties(std::get<1>(roomValues[i]), gprop);
		roomVolume.emplace_back(gprop.Mass());

		// check if center of mass lies inside of room
		gp_Pnt potentialCenter = gprop.CentreOfMass();
		gp_Pnt potentialCenterHigh = potentialCenter;
		potentialCenterHigh.SetZ(potentialCenterHigh.Z() + 1000);
		std::vector<gp_Pnt> lowLine = { potentialCenter,  potentialCenterHigh };

		auto faceTriangles = triangulateShape(&std::get<1>(roomValues[i]));

		if (faceTriangles.size() == 0)
		{
			roomCenterPoints_.emplace_back(gprop.CentreOfMass());
			continue;
		}

		int iCount = 0;
		for (size_t l = 0; l < faceTriangles.size(); l++)
		{
			if (triangleIntersecting(lowLine, faceTriangles[l]))
			{
				iCount++;
			}
		}

		if (iCount % 2 != 0) {
			roomCenterPoints_.emplace_back(gprop.CentreOfMass());
			continue;
		}

		// if not create replacement for the potential point
		double distance = 999999;
		gp_Pnt replacementPoint = { 0,0,0 };


		for (size_t j = 0; j < faceTriangles.size(); j++)
		{

			bool isFlat = true;

			if (faceTriangles[j].size() != 3)
			{
				continue;
			}

			for (size_t k = 1; k < faceTriangles[j].size(); k++)
			{

				// only select flat triangles
				if (faceTriangles[j][k].Z() + 0.01 < faceTriangles[j][0].Z() || 
					faceTriangles[j][k].Z() - 0.01 > faceTriangles[j][0].Z())
				{
					isFlat = false;
					break;
				}
			}

			if (!isFlat)
			{
				continue;
			}

			// compute centerpoint of the triangle
			double sumX = 0;
			double sumY = 0;
			double sumZ = 0;

			for (size_t k = 0; k < faceTriangles[j].size(); k++)
			{
				sumX += faceTriangles[j][k].X();
				sumY += faceTriangles[j][k].Y();
				sumZ += faceTriangles[j][k].Z();
			}

			gp_Pnt potentialReplacementPoint = { sumX / 3, sumY / 3, sumZ / 3 };

			double pToPDistance = potentialReplacementPoint.Distance(potentialCenter);
			if (pToPDistance < distance)
			{
				distance = pToPDistance;
				replacementPoint = potentialReplacementPoint;
			}

		}

		if (replacementPoint.Z() < potentialCenter.Z()) { replacementPoint.SetZ(replacementPoint.Z() + 0.10); }
		if (replacementPoint.Z() > potentialCenter.Z()) { replacementPoint.SetZ(replacementPoint.Z() - 0.10); }
		
		roomCenterPoints_.emplace_back(replacementPoint);
	}

	// check which room is complex
	for (size_t i = 0; i < roomValues.size(); i++)
	{
		// create bounding box around roomshape
		TopoDS_Shape roomShape = std::get<1>(roomValues[i]);
		std::vector<gp_Pnt> cornerPoints;

		gp_Pnt lll(9999, 9999, 9999);
		gp_Pnt urr(-9999, -9999, -9999);

		TopExp_Explorer expl;
		for (expl.Init(roomShape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			cornerPoints.emplace_back(p);
		}

		for (size_t j = 0; j < cornerPoints.size(); j++)
		{
			auto currentCorner = rotatePointWorld(cornerPoints[j], originRot_);
			if (urr.X() < currentCorner.X()) { urr.SetX(currentCorner.X()); }
			if (urr.Y() < currentCorner.Y()) { urr.SetY(currentCorner.Y()); }
			if (urr.Z() < currentCorner.Z()) { urr.SetZ(currentCorner.Z()); }
			if (lll.X() > currentCorner.X()) { lll.SetX(currentCorner.X()); }
			if (lll.Y() > currentCorner.Y()) { lll.SetY(currentCorner.Y()); }
			if (lll.Z() > currentCorner.Z()) { lll.SetZ(currentCorner.Z()); }
		}
		// check if other rooms fall inside evaluating room
		boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lll), Point3DOTB(urr));

		std::vector<Value> qResult;
		qResult.clear();
		getRoomIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

		if (qResult.size() == 0) { continue; }

		bool complex = false;

		BRepClass3d_SolidClassifier insideChecker;
		insideChecker.Load(roomShape);

		for (size_t j = 0; j < qResult.size(); j++)
		{
			if (i == qResult[j].second) { continue; } // ignore self

			TopoDS_Shape matchingRoom = std::get<1>(roomValues[qResult[j].second]);
			for (expl.Init(matchingRoom, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				insideChecker.Perform(p, 0.0001);
				if (!insideChecker.State() || insideChecker.IsOnAFace())
				{
					if (roomVolume[i] > roomVolume[qResult[j].second])
					{
						complex = true;
						continue;
					}
				}
				if (complex)
				{
					complex = false;
					break;
				}
			}
			if (complex) {
				std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_COMPLEX);
				break;
			}
		}
		if (!complex)
		{
			std::get<0>(roomValues[i])->setCompositionType(IfcSchema::IfcElementCompositionEnum::IfcElementComposition_ELEMENT);
			gp_Pnt centerOfMass = gprop.CentreOfMass();
		}
	}
}

std::map<std::string, std::string> helper::getProjectInformation()
{
	


	return std::map<std::string, std::string>();
}

std::map<std::string, std::string> helper::getBuildingInformation()
{
	std::map<std::string, std::string> dictionary;

	IfcSchema::IfcBuilding::list::ptr buildingList = file_->instances_by_type<IfcSchema::IfcBuilding>();
	
	for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
		IfcSchema::IfcBuilding* building = *it;
		
		if (building->hasDescription()) { dictionary.emplace("Description", building->Description()); }
		if (building->hasObjectType()) { dictionary.emplace("ObjectType", building->ObjectType()); }
		if (building->hasName()) { dictionary.emplace("Name", building->Name()); }
		if (building->hasLongName()) { dictionary.emplace("Long Name", building->LongName()); }
	}

	IfcSchema::IfcRelDefinesByProperties::list::ptr propteriesRels = file_->instances_by_type<IfcSchema::IfcRelDefinesByProperties>();

	for (auto it = propteriesRels->begin(); it != propteriesRels->end(); ++it) {
		IfcSchema::IfcRelDefinesByProperties* propteriesRel = *it;

		auto relatedObjects = propteriesRel->RelatedObjects();
		bool isBuilding = false;

		for (auto et = relatedObjects->begin(); et != relatedObjects->end(); ++et) {

			auto* relatedObject = *et;
			if (std::string(relatedObject->data().type()->name().c_str()) != "IfcBuilding") { break; }

			isBuilding = true;
			break;
		}

		if (isBuilding)
		{
			IfcSchema::IfcPropertySet* te = propteriesRel->RelatingPropertyDefinition()->as<IfcSchema::IfcPropertySet>();
			if (propteriesRel->RelatingPropertyDefinition()->data().type()->name() != "IfcPropertySet") { continue; }
			auto relAssociations = propteriesRel->RelatingPropertyDefinition()->data().getArgument(4)->operator IfcEntityList::ptr();

			for (auto et = relAssociations->begin(); et != relAssociations->end(); ++et) {
				auto* relatedObject = *et;
				IfcSchema::IfcPropertySingleValue* propertyValue = relatedObject->as<IfcSchema::IfcPropertySingleValue>();

				int dataType = propertyValue->NominalValue()->data().getArgument(0)->type();


				if (dataType == 3) // If Bool
				{
					if (propertyValue->NominalValue()->data().getArgument(0)->toString() == ".T.")
					{
						dictionary.emplace(propertyValue->Name().c_str(), "True");
					}
					else {
						dictionary.emplace(propertyValue->Name().c_str(), "False");
					}
				}
				else if (dataType == 4) // If Area
				{
					std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();

					if (stringValue[stringValue.size() - 1] == '.') { stringValue += "0"; }

					if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue + " m^2"); } // TODO: get unit

				}
				else if (dataType == 5) // If String
				{
					std::string stringValue = propertyValue->NominalValue()->data().getArgument(0)->toString();
					stringValue = stringValue.substr(1, stringValue.size() - 2);
					if (stringValue.size() != 0) { dictionary.emplace(propertyValue->Name().c_str(), stringValue); }
					continue;
				}

				if (propertyValue->NominalValue()->data().getArgument(0)->toString().size() != 0)
				{
					dictionary.emplace(propertyValue->Name().c_str(), propertyValue->NominalValue()->data().getArgument(0)->toString());
				}
			}
		}
	}

	return dictionary;
}

std::string helper::getBuildingName()
{
	IfcSchema::IfcBuilding::list::ptr buildingList = file_->instances_by_type<IfcSchema::IfcBuilding>();

	for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
		IfcSchema::IfcBuilding* building = *it;

		if (building->hasName())
		{
			return building->Name();
		}

		return "";
	}

}

std::string helper::getBuildingLongName()
{
	IfcSchema::IfcBuilding::list::ptr buildingList = file_->instances_by_type<IfcSchema::IfcBuilding>();

	for (auto it = buildingList->begin(); it != buildingList->end(); ++it) {
		IfcSchema::IfcBuilding* building = *it;
		if (building->hasLongName())
		{
			return building->LongName();
		}

		return "";
	}
}

std::string helper::getProjectName()
{
	IfcSchema::IfcProject::list::ptr projectList = file_->instances_by_type<IfcSchema::IfcProject>();
	for (auto it = projectList->begin(); it != projectList->end(); ++it) {
		IfcSchema::IfcProject* project = *it;

		if (project->hasName())
		{
			return project->Name();
		}
		return "";
	}


}

std::list<std::string> helper::getObjectTypes() {

	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();

	std::list<std::string> pList;

	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		pList.emplace_back(boost::to_upper_copy<std::string>(product->data().type()->name()));
	}

	pList.sort();
	pList.unique();

	return pList;
}

std::vector<std::vector<gp_Pnt>> helper::triangulateProduct(IfcSchema::IfcProduct* product)
{
	return triangulateShape(&getObjectShape(product, true));
}

std::vector<std::vector<gp_Pnt>> triangulateShape(TopoDS_Shape* shape) {
	
	std::vector<TopoDS_Face> faceList;

	TopExp_Explorer expl;
	for (expl.Init(*shape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face face = TopoDS::Face(expl.Current());
		faceList.emplace_back(face);
	}

	std::vector<std::vector<gp_Pnt>> triangleMeshList;

	for (size_t i = 0; i < faceList.size(); i++)
	{
		BRepMesh_IncrementalMesh faceMesh = BRepMesh_IncrementalMesh(faceList[i], 0.004);

		TopLoc_Location loc;
		auto mesh = BRep_Tool::Triangulation(faceList[i], loc);
		const gp_Trsf& trsf = loc.Transformation();

		if (mesh.IsNull()) //TODO warning
		{
			continue;
		}

		for (size_t j = 1; j <= mesh.get()->NbTriangles(); j++)
		{
			const Poly_Triangle& theTriangle = mesh->Triangles().Value(j);

			std::vector<gp_Pnt> trianglePoints;
			for (size_t k = 1; k <= 3; k++)
			{
				gp_Pnt p = mesh->Nodes().Value(theTriangle(k)).Transformed(trsf);
				trianglePoints.emplace_back(p);
			}
			triangleMeshList.emplace_back(trianglePoints);
		}

	}

	return triangleMeshList;

}

template <typename T>
void helper::addObjectToIndex(T object) {
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;
		bg::model::box <BoostPoint3D> box = makeObjectBox(product);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			std::cout << "Failed: " + product->data().toString() << std::endl;
			continue;
		}

		bool dub = false;
		for (size_t i = 0; i < productLookup_.size(); i++)
		{
			if (std::get<0>(productLookup_[i])->data().id() == product->data().id())
			{
				dub = true;
				break;
			}
		}

		if (dub) { continue; }

		TopoDS_Shape shape = getObjectShape(product);
		TopoDS_Shape cbbox;
		bool hasCBBox = false;
		bool matchFound = false;

		std::string productType = product->data().type()->name();

		if (productType == "IfcDoor" || productType == "IfcWindow")
		{
			// get potential nesting objects
			std::vector<Value> qResult;
			qResult.clear();
			index_.query(bgi::intersects(box), std::back_inserter(qResult));

			BRepExtrema_DistShapeShape distanceMeasurer;

			gp_Pnt lll = Point3DBTO(box.min_corner());
			gp_Pnt urr = Point3DBTO(box.max_corner());

			gp_Pnt mP = gp_Pnt((lll.X() + urr.X()) / 2, (lll.Y() + urr.Y()) / 2, (lll.Z() + urr.Z()) / 2);
			distanceMeasurer.LoadS1(BRepBuilderAPI_MakeVertex(mP));

			for (size_t i = 0; i < qResult.size(); i++)
			{
				LookupValue lookup = productLookup_[qResult[i].second];
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);
				std::string qProductType = qProduct->data().type()->name();

				if (qProductType != "IfcWall" &&
					qProductType != "IfcWallStandardCase" &&
					qProductType != "IfcRoof" &&
					qProductType != "IfcSlab")
				{
					continue;
				}

				auto search = adjustedshapeLookup_.find(qProduct->data().id());
				if (search == adjustedshapeLookup_.end()) { continue; }
				TopoDS_Shape qUntrimmedShape = search->second;

				TopExp_Explorer expl;
				for (expl.Init(qUntrimmedShape, TopAbs_SOLID); expl.More(); expl.Next()) {
					// get distance to isolated actual object
					distanceMeasurer.LoadS2(expl.Current());
					distanceMeasurer.Perform();
					double distance = distanceMeasurer.Value();
					if (!distanceMeasurer.InnerSolution()) { continue; }
					if (distance > 0.2) { continue; }

					matchFound = true;
					break;
				}
			}
			//if (matchFound) { continue; }

			// if no void was found
			// find longest horizontal and vertical edge
			std::vector<gp_Pnt> pointList;
			std::vector<gp_Pnt> horizontalMaxEdge;
			std::vector<gp_Pnt> verticalMaxEdge;

			double maxHDistance = 0;
			double maxVDistance = 0;

			TopExp_Explorer expl;
			for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				pointList.emplace_back(p);
			}
			for (size_t i = 0; i < pointList.size(); i += 2)
			{
				gp_Pnt p1 = pointList[i];
				gp_Pnt p2 = pointList[i + 1];

				double vDistance = abs(p1.Z() - p2.Z());

				if (maxVDistance < vDistance)
				{
					maxVDistance = vDistance;
					verticalMaxEdge = { p1, p2 };
				}

				p1.SetZ(0);
				p2.SetZ(0);

				double hDistance = p1.Distance(p2);

				if (hDistance > maxHDistance)
				{
					maxHDistance = hDistance;
					horizontalMaxEdge = { p1, p2 };
				}
			}

			// compute horizontal rotaion
			gp_Pnt p1 = horizontalMaxEdge[0];
			gp_Pnt p2 = horizontalMaxEdge[1];

			double angleFlat = 0;

			if (abs(p1.Y() - p2.Y()) > 0.00001)
			{
				double os = abs(p1.Y() - p2.Y()) / p1.Distance(p2);
				angleFlat = asin(os);

				gp_Pnt tempP = rotatePointPoint(p2, p1, angleFlat);

				if (Abs(p1.X() - tempP.X()) > 0.01 && Abs(p1.Y() - tempP.Y()) > 0.01)
				{
					angleFlat = -angleFlat;
				}
			}

			// compute vertical rotation
			gp_Pnt p3 = verticalMaxEdge[0];
			gp_Pnt p4 = rotatePointPoint(verticalMaxEdge[1], p3, angleFlat);

			bool isRotated = false;
			if (abs(p3.X() - p4.X()) < 0.01)
			{
				p3 = rotatePointWorld(p3, M_PI / 2.0);
				p4 = rotatePointWorld(p4, M_PI / 2.0);
				angleFlat += M_PI / 2.0;
				isRotated = true;
			}

			double angleVert = acos(abs(p4.Z() - p3.Z()) / p3.Distance(p4));
			gp_Pnt tempPoint = p4.Rotated(gp_Ax1(p3, gp_Dir(0, 1, 0)), angleVert);
			if (Abs(p3.X() - tempPoint.X()) > 0.01 && Abs(p3.Z() - tempPoint.Z()) > 0.01)
			{
				angleVert = -angleVert;
			}

			auto base = rotatedBBoxDiagonal(pointList, angleFlat, angleVert);

			gp_Pnt lllPoint = std::get<0>(base);
			gp_Pnt urrPoint = std::get<1>(base);

			if (lllPoint.IsEqual(urrPoint, 1e-6)) { continue; }

			hasCBBox = true;
			cbbox = makeSolidBox(lllPoint, urrPoint, angleFlat, angleVert);
			if (cbbox.IsNull()) { hasCBBox = false; }

			//if (isRotated)
			//{
			//	gp_Trsf transform;
			//	transform.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), -M_PI / 2.0);
			//	angleFlat -= M_PI / 2.0;
			//	BRepBuilderAPI_Transform brep_transform(cbbox, transform);
			//	cbbox = brep_transform.Shape();
			//}

			//if (angleVert != 0)
			//{
			//	gp_Trsf transform;
			//	transform.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), angleVert);
			//	BRepBuilderAPI_Transform brep_transform(cbbox, transform);
			//	cbbox = brep_transform.Shape();
			//}


		}
		index_.insert(std::make_pair(box, (int)index_.size()));
		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);
		auto lookup = std::make_tuple(*it, triangleMeshList, shape, hasCBBox, cbbox);
		productLookup_.emplace_back(lookup);
	}
}

template <typename T>
void helper::addObjectToCIndex(T object) {
	// add doors to the rtree (for the appartment detection)

	std::map < int, int >  processed;

	for (auto it = object->begin(); it != object->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		std::vector<IfcSchema::IfcProduct*> productList = getNestedProducts(product);

		bg::model::box <BoostPoint3D> box = makeObjectBox(productList);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}

		gp_Pnt groundObjectPoint = { 0, 0,9999 };
		gp_Pnt topObjectPoint = {0, 0, -9999};
		bool isDoor = false;

		if (product->data().type()->name() == "IfcDoor") { isDoor = true; }

		for (size_t i = 0; i < productList.size(); i++)
		{
			TopoDS_Shape productShape = getObjectShape(productList[i]);
			gp_Pnt potGround = getLowestPoint(productShape, isDoor);
			gp_Pnt potTop = getHighestPoint(productShape);

			if (!isDoor)
			{
				if (potGround.Z() < groundObjectPoint.Z()) { groundObjectPoint = potGround; }
				if (potTop.Z() > topObjectPoint.Z()) { topObjectPoint = potTop; }
			}
			if (isDoor)
			{
				if (potGround.Z() < groundObjectPoint.Z()) { groundObjectPoint = potGround; }
				if (potTop.Z() > topObjectPoint.Z()) { topObjectPoint = potTop; }
			}
		}

		std::vector<gp_Pnt> connectingPoints = { groundObjectPoint, topObjectPoint };

		cIndex_.insert(std::make_pair(box, (int)cIndex_.size()));
		std::vector<roomObject*>* space = new std::vector<roomObject*>;
		auto lookup = std::make_tuple(*it, connectingPoints, space);
		connectivityLookup_.emplace_back(lookup);
	}
}

template<typename T>
void helper::addObjectToRIndex(T object){
	// add doors to the rtree (for the appartment detection)
	for (auto it = object->begin(); it != object->end(); ++it) {
		bg::model::box <BoostPoint3D> box = makeObjectBox(*it);
		if (bg::get<bg::min_corner, 0>(box) == bg::get<bg::max_corner, 0>(box) &&
			bg::get<bg::min_corner, 1>(box) == bg::get<bg::max_corner, 1>(box)) {
			continue;
		}
		rIndex_.insert(std::make_pair(box, (int)rIndex_.size()));
		TopoDS_Shape spaceShape = getObjectShape(*it);
		auto lookup = std::make_tuple(*it, spaceShape);
		roomLookup_.emplace_back(lookup);
	}
}



IfcSchema::IfcOwnerHistory* helper::getHistory()
{
	IfcSchema::IfcOwnerHistory* ownerHistory;
	IfcSchema::IfcOwnerHistory::list::ptr ownerHistories = file_->instances_by_type<IfcSchema::IfcOwnerHistory>();

	if (ownerHistories.get()->size() != 1) { std::cout << "[Warning] multiple owners objects found!" << std::endl; }

	return *ownerHistories.get()->begin();
}


std::vector<gp_Pnt> helper::getObjectPoints(IfcSchema::IfcProduct* product, bool sortEdges, bool simple)
{
	std::vector<gp_Pnt> pointList;
	TopoDS_Shape productShape = getObjectShape(product, simple);

	TopExp_Explorer expl;
	for (expl.Init(productShape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		pointList.emplace_back(p);
	}

	if (!sortEdges) { return pointList; }

	std::vector<gp_Pnt> pointListSmall;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		if (i%2 == 0) { pointListSmall.emplace_back(pointList[i]); }
	}

	return pointListSmall;

}

std::vector<gp_Pnt> helper::getObjectPoints(TopoDS_Shape shape, bool sortEdges)
{
	std::vector<gp_Pnt> pointList;
	TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_VERTEX); expl.More(); expl.Next())
	{
		const TopoDS_Vertex& vertex = TopoDS::Vertex(expl.Current());
		pointList.emplace_back(BRep_Tool::Pnt(vertex));
	}

	if (!sortEdges) { return pointList; }

	std::vector<gp_Pnt> pointListSmall;
	for (size_t i = 0; i < pointList.size(); i++)
	{
		if (i % 2 == 0) { pointListSmall.emplace_back(pointList[i]); }
	}

	return pointListSmall;
}


std::vector<TopoDS_Face> helper::getObjectFaces(IfcSchema::IfcProduct* product, bool simple)
{
	std::vector<TopoDS_Face> faceList;

	if (!product->hasRepresentation()) { return {}; }

	TopoDS_Shape rShape = getObjectShape(product, simple);

	TopExp_Explorer expl;
	for (expl.Init(rShape, TopAbs_FACE); expl.More(); expl.Next())
	{
		TopoDS_Face face = TopoDS::Face(expl.Current());
		faceList.emplace_back(face);
	}

	return faceList;
}

TopoDS_Shape helper::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted, bool memorize)
{
	std::string objectType = product->data().type()->name();
	if (objectType == "IfcFastener")
	{
		return {};
	}

	// filter with lookup
	if (objectType != "IfcWall" &&
		objectType != "IfcWallStandardCase" &&
		objectType != "IfcRoof" &&
		objectType != "IfcSlab" )
	{
		adjusted = false;
	}

	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->data().id());
		if (search != shapeLookup_.end())
		{
			return search->second;
		}
	}

	if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->data().id());
		if (search != adjustedshapeLookup_.end())
		{
			return search->second;
		}
	}

	IfcSchema::IfcRepresentation* ifc_representation = 0;
	bool hasbody = false;

	if (product->hasRepresentation())
	{
		IfcSchema::IfcProductRepresentation* prodrep = product->Representation();
		IfcSchema::IfcRepresentation::list::ptr reps = prodrep->Representations();

		for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
			IfcSchema::IfcRepresentation* rep = *it;
			if (rep->RepresentationIdentifier() == "Body") {
				ifc_representation = rep;
				hasbody = true;
				break;
			}
		}
	}

	bool hasHoles = false;

	if (objectType == "IfcWall" ||
		objectType == "IfcWallStandardCase" ||
		objectType == "IfcRoof" ||
		objectType == "IfcSlab") {
		hasHoles = true;
	}

	if (!hasbody)
	{
#ifdef USE_IFC4
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() > 0)
		{
			if (decomposedProducts->size() == 0) { return { }; }

			BRep_Builder builder;
			TopoDS_Compound collection;
			builder.MakeCompound(collection);

			BRep_Builder builderSimple;
			TopoDS_Compound collectionSimple;
			builderSimple.MakeCompound(collectionSimple);

			for (auto et = decomposedProducts->begin(); et != decomposedProducts->end(); ++et) {
#ifdef USE_IFC4
				IfcSchema::IfcRelAggregates* aggregates = *et;
#else
				IfcSchema::IfcRelDecomposes* aggregates = *et;
#endif // USE_IFC4
				IfcSchema::IfcObjectDefinition::list::ptr aggDef = aggregates->RelatedObjects();

				for (auto rt = aggDef->begin(); rt != aggDef->end(); ++rt) {
					IfcSchema::IfcObjectDefinition* aggDef = *rt;

					IfcSchema::IfcProduct* addprod = aggDef->as<IfcSchema::IfcProduct>();

					TopoDS_Shape addshape = getObjectShape(addprod, false, false);
					TopoDS_Shape addshapeSimple = getObjectShape(addprod, true, false);
					builder.Add(collection, addshape);
					builderSimple.Add(collectionSimple, addshapeSimple);
				}
			}

			if (memorize)
			{
				shapeLookup_[product->data().id()] = collection;
			}

			if (hasHoles)
			{
				if (memorize)
				{
					adjustedshapeLookup_[product->data().id()] = collectionSimple;
				}
			}

			if (adjusted)
			{
				return collectionSimple;
			}

			return collection;
		}
		return {};
	}

	TopoDS_Compound comp;
	TopoDS_Compound simpleComp;
	gp_Trsf placement;
	gp_Trsf trsf;

	IfcGeom::IteratorSettings settings;
	kernel_->convert_placement(product->ObjectPlacement(), trsf);
	IfcGeom::BRepElement<double, double>* brep = kernel_->convert(settings, ifc_representation, product);

	if (brep == nullptr) { return {}; } //TODO: find manner to aquire data in another manner
	
	kernel_->convert_placement(ifc_representation, placement);

	comp = brep->geometry().as_compound();
	comp.Move(trsf * placement); // location in global space
	comp.Move(objectTranslation_);

	if (hasHoles)
	{
		settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true);
		brep = kernel_->convert(settings, ifc_representation, product);
		kernel_->convert_placement(ifc_representation, placement);

		simpleComp = brep->geometry().as_compound();
		simpleComp.Move(trsf * placement); // location in global space
		simpleComp.Move(objectTranslation_);
	}

	if (memorize)
	{
		shapeLookup_[product->data().id()] = comp;
	}


	if (hasHoles)
	{
		if (memorize)
		{
			adjustedshapeLookup_[product->data().id()] = simpleComp;
		}
	}

	if (adjusted)
	{
		return simpleComp;
	}
	
	return comp;
}

void helper::updateShapeLookup(IfcSchema::IfcProduct* product, TopoDS_Shape shape, bool adjusted)
{
	if (!product->hasRepresentation()) { return; }

	// filter with lookup
	if (!adjusted)
	{
		auto search = shapeLookup_.find(product->data().id());
		if (search == shapeLookup_.end())
		{
			return;
		}

		shapeLookup_[product->data().id()] = shape;
	}

	if (adjusted)
	{
		auto search = adjustedshapeLookup_.find(product->data().id());
		if (search == adjustedshapeLookup_.end())
		{
			return;
		}

		adjustedshapeLookup_[product->data().id()] = shape;
	}
}

void helper::updateIndex(IfcSchema::IfcProduct* product, TopoDS_Shape shape) {
	
	auto base = rotatedBBoxDiagonal(getObjectPoints(product), 0);
	gp_Pnt lllPoint = std::get<0>(base);
	gp_Pnt urrPoint = std::get<1>(base);

	std::vector<Value> qResult;
	boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lllPoint), Point3DOTB(urrPoint));

	index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

	for (size_t i = 0; i < qResult.size(); i++)
	{
		LookupValue lookup = getLookup(qResult[i].second);
		IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);

		if (qProduct->data().id() != product->data().id())
		{
			continue;
		}

		std::vector<std::vector<gp_Pnt>> triangleMeshList = triangulateProduct(product);

		updateLookupTriangle(triangleMeshList, qResult[i].second);
	}
}

void helper::applyVoids()
{
	voidShapeAdjust<IfcSchema::IfcWallStandardCase::list::ptr>(file_->instances_by_type<IfcSchema::IfcWallStandardCase>());
	voidShapeAdjust<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
}

template <typename T>
void helper::voidShapeAdjust(T products) 
{
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* wallProduct = *it;
		TopoDS_Shape untrimmedWallShape = getObjectShape(wallProduct, true);
		TopExp_Explorer expl;

		// get the voids
		IfcSchema::IfcElement* wallElement = wallProduct->as<IfcSchema::IfcElement>();
		IfcSchema::IfcRelVoidsElement::list::ptr voidElementList = wallElement->HasOpenings();
		std::vector<TopoDS_Shape> validVoidShapes;
		double voidCount = voidElementList->size();

		// find if the voids are filled or not
		for (auto et = voidElementList->begin(); et != voidElementList->end(); ++et) {
			IfcSchema::IfcRelVoidsElement* voidElement = *et;
			IfcSchema::IfcFeatureElementSubtraction* openingElement = voidElement->RelatedOpeningElement();
			TopoDS_Shape substractionShape = getObjectShape(openingElement);
			
			auto base = rotatedBBoxDiagonal(getObjectPoints(openingElement), originRot_);
			gp_Pnt lllPoint = std::get<0>(base);
			gp_Pnt urrPoint = std::get<1>(base);

			std::vector<Value> qResult;
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lllPoint), Point3DOTB(urrPoint));

			index_.query(bgi::intersects(qBox), std::back_inserter(qResult));

			if (qResult.size() == 0)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}

			BRepClass3d_SolidClassifier insideChecker;
			insideChecker.Load(substractionShape);

			bool inter = false;

			for (size_t i = 0; i < qResult.size(); i++)
			{
				LookupValue lookup = getLookup(qResult[i].second);
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);

				if (qProduct->data().type()->name() != "IfcWindow" &&
					qProduct->data().type()->name() != "IfcDoor" &&
					qProduct->data().type()->name() != "IfcColumn")
				{
					continue;
				}

				TopoDS_Shape qShape = getObjectShape(qProduct);

				TopExp_Explorer expl2;
				for (expl2.Init(qShape, TopAbs_VERTEX); expl2.More(); expl2.Next()) {

					insideChecker.Perform(BRep_Tool::Pnt(TopoDS::Vertex(expl2.Current())), 0.01);

					if (insideChecker.State() || insideChecker.IsOnAFace()) {
						inter = true;
						break;
					}
				}

				if (inter)
				{
					break;
				}
			}

			if (inter == false)
			{
				validVoidShapes.emplace_back(substractionShape);
				continue;
			}		
		}

		if (validVoidShapes.size() == 0 )
		{
			continue;
		}
		else if (validVoidShapes.size() == voidCount)
		{
			TopoDS_Shape finalShape = getObjectShape(wallProduct, false);
			updateShapeLookup(wallProduct, finalShape, true);
			updateIndex(wallProduct, finalShape);
		}

		else if (validVoidShapes.size() > 0 && voidElementList->size() > 0) {
			// get a basepoint of the wall
			std::vector<gp_Pnt> pList;;
			for (expl.Init(untrimmedWallShape, TopAbs_VERTEX); expl.More(); expl.Next()) {
				pList.emplace_back(BRep_Tool::Pnt(TopoDS::Vertex(expl.Current())));
			}

			// bool out voidShape
			BOPAlgo_Splitter aSplitter;
			TopTools_ListOfShape aLSObjects;
			aLSObjects.Append(untrimmedWallShape);
			TopTools_ListOfShape aLSTools;

			TopExp_Explorer expl;
			for (size_t i = 0; i < validVoidShapes.size(); i++)
			{
				aLSTools.Append(validVoidShapes[i]);
			}

			aLSTools.Reverse();
			aSplitter.SetArguments(aLSObjects);
			aSplitter.SetTools(aLSTools);
			aSplitter.SetRunParallel(Standard_True);
			aSplitter.SetNonDestructive(Standard_True);

			aSplitter.Perform();

			const TopoDS_Shape& aResult = aSplitter.Shape(); // result of the operation

			TopoDS_Shape finalShape;
			std::vector<TopoDS_Solid> solids;
			for (expl.Init(aResult, TopAbs_SOLID); expl.More(); expl.Next()) {
				solids.emplace_back(TopoDS::Solid(expl.Current()));
			}

			for (size_t i = 0; i < pList.size(); i++)
			{
				int count = 0;

				for (size_t j = 0; j < solids.size(); j++)
				{
					for (expl.Init(solids[j], TopAbs_VERTEX); expl.More(); expl.Next()) {
						gp_Pnt evalPoint = BRep_Tool::Pnt(TopoDS::Vertex(expl.Current()));
						if (pList[i].X() == evalPoint.X() &&
							pList[i].Y() == evalPoint.Y() &&
							pList[i].Z() == evalPoint.Z() )
						{
							finalShape = solids[j];
							count ++;
							break;
						}
					}

				}

				if (count == 1)
				{
					break;
				}
			}
			updateShapeLookup(wallProduct, finalShape, true);
			updateIndex(wallProduct, finalShape);
		}
	}
}

void helper::whipeObject(IfcSchema::IfcProduct* product)
{
	file_->removeEntity(product);

	// TODO check dependencies!
}

void helper::wipeObject(helper* data, int id)
{
	//TODO find the nested indx located in the file
	//TODO find if the nested idnx have other dependencies 
	//TODO remove objects at indx that do not have dependencies
}

void helper::writeToFile(std::string path)
{
	std::ofstream storageFile;
	storageFile.open(path);
	std::cout << "[INFO] Exporting file " << fileName_ << std::endl;
	storageFile << *file_;
	std::cout << "[INFO] Exported succesfully" << std::endl;
	storageFile.close();
}

void helperCluster::internaliseData()
{
	bool debug = false;

	if (!helperList[0].getDepending())
	{
		helperList[0].internalizeGeo();

		lllPoint_ = helperList[0].getLllPoint();
		urrPoint_ = helperList[0].getUrrPoint();
		originRot_ = helperList[0].getRotation();

		if (debug)
		{
			std::cout << "cluster:" << std::endl;
			printPoint(lllPoint_);
			printPoint(urrPoint_);
			std::cout << originRot_ << std::endl;
		}

		return;
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (helper.getIsConstruct())
		{
			helper.internalizeGeo();
			lllPoint_ = helper.getLllPoint();
			urrPoint_ = helper.getUrrPoint();
			originRot_ = helper.getRotation();
			objectTranslation_ = helper.getObjectTranslation();
		}
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (!helper.getHasGeo())
		{
			helper.internalizeGeo(originRot_, objectTranslation_);

			// update bbox if needed
			gp_Pnt addLllPoint = helper.getLllPoint();
			gp_Pnt addUrrPoint = helper.getUrrPoint();

			if (addLllPoint.X() < lllPoint_.X()) { lllPoint_.SetX(addLllPoint.X()); }
			if (addLllPoint.Y() < lllPoint_.Y()) { lllPoint_.SetY(addLllPoint.Y()); }
			if (addLllPoint.Z() < lllPoint_.Z()) { lllPoint_.SetZ(addLllPoint.Z()); }

			if (addUrrPoint.X() > urrPoint_.X()) { urrPoint_.SetX(addUrrPoint.X()); }
			if (addUrrPoint.Y() > urrPoint_.Y()) { urrPoint_.SetY(addUrrPoint.Y()); }
			if (addUrrPoint.Z() > urrPoint_.Z()) { urrPoint_.SetZ(addUrrPoint.Z()); }
		}

	}

	hasBbox_ = true;

	if (debug)
	{
		std::cout << "cluster:" << std::endl;
		printPoint(lllPoint_);
		printPoint(urrPoint_);
		std::cout << originRot_ << std::endl;
	}
}

void helperCluster::appendHelper(helper data)
{
	helperList.emplace_back(data);
	size_++;
}


void helperCluster::setUseProxy(bool b) {
	for (size_t i = 0; i < helperList.size(); i++)
	{
		helperList[i].setUseProxy(b);
	}
}

std::list<std::string> helperCluster::getObjectList() {
	
	std::list<std::string> typeListLong;

	for (size_t i = 0; i < size_; i++)
	{	
		std::list<std::string> typeList = helperList[i].getObjectTypes();

		for (auto it = typeList.begin(); it != typeList.end(); ++it) {
			typeListLong.emplace_back(*it);
		}
	}

	typeListLong.sort();
	typeListLong.unique();

	return typeListLong;
}

bool IOManager::yesNoQuestion()
{
	std::string cont = "";

	while (true)
	{
		std::getline(std::cin, cont);
		if (cont == "Y" || cont == "y") { return true; }
		if (cont == "N" || cont == "n") { return false; }
	}
}

int IOManager::numQuestion(int n, bool lower)
{
	while (true)
	{
		bool validInput = true;
		std::cout << "Num: ";

		std::string stringNum = "";
		std::getline(std::cin, stringNum);

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (!std::isdigit(stringNum[i]))
			{
				validInput = false;
			}
		}

		if (validInput)
		{
			int intNum = std::stoi(stringNum) - 1;
			if (!lower)
			{
				if (n >= intNum + 1) {
					return intNum;
				}
			}
			else if (lower)
			{
				if (n >= intNum + 1 && intNum >= 0) {
					return intNum;
				}
			}
		}
		std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
	}
}

bool IOManager::getTargetPathList()
{
	std::cout << "Enter filepath of the JSON or IFC input file" << std::endl;
	std::cout << "[INFO] If multifile seperate by enter" << std::endl;
	std::cout << "[INFO] Finish by empty line + enter" << std::endl;

	while (true)
	{
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		if (singlepath.size() == 0 && inputPathList_.size() == 0)
		{
			std::cout << "[INFO] No filepath has been supplied" << std::endl;
			std::cout << "Enter filepath of the JSON or IFC input file (if multiplefile sperate path with enter):" << std::endl;
			continue;
		}
		else if (singlepath.size() == 0)
		{
			break;
		}
		if (!hasExtension(singlepath, "ifc") && !hasExtension(singlepath, "json"))
		{
			std::cout << "[INFO] No valid filepath has been supplied" << std::endl;
			std::cout << "Enter filepath of the JSON or IFC input file (if multiplefile sperate path with enter):" << std::endl;
			continue;
		}

		if (hasExtension(singlepath, "json"))
		{
			isJsonInput_ = true;
			if (inputPathList_.size() > 1)
			{
				return false;
			}
		}

		inputPathList_.emplace_back(singlepath);
	}

	if (inputPathList_.size() > 0) { return true; }
	return false;
}


bool IOManager::getOutputPathList() {

	while (true)
	{
		std::cout << "Enter target folderpath of the CityJSON file" << std::endl;
		std::cout << "Path: ";
		std::string singlepath = "";
		std::getline(std::cin, singlepath);

		struct stat info;
		if (stat(singlepath.c_str(), &info) != 0)
		{
			std::cout << "[INFO] Folder path " << singlepath << " does not exist." << std::endl;
			std::cout << "do you want to create a new folder (Y/N) " << std::endl;

			if (yesNoQuestion())
			{
				std::cout << "[INFO] Export folder is created at: " << singlepath << std::endl;
				int status = mkdir(singlepath.c_str());
				break;
			}
			continue;
		}
		outputFolderPath_ = singlepath;
		return true;
	}

}

bool IOManager::getUseDefaultSettings()
{
	std::cout << "Use default process/export settings? (Y/N):";
	if (!yesNoQuestion()) { return false; } 

	// get default export path
	size_t pos = inputPathList_[0].find_last_of("\\/");
	outputFolderPath_ = (std::string) inputPathList_[0].substr(0, pos) + "/" + "_exports";
	struct stat info;
	if (stat(outputFolderPath_.c_str(), &info) != 0)
	{
		std::cout << "[INFO] Export folder is created at: " << outputFolderPath_ << std::endl;
		int status = mkdir(outputFolderPath_.c_str());
	}
	return true;
}

std::string IOManager::getFileName(const std::string& stringPath)
{
	std::vector<std::string> segments;
	boost::split(segments, stringPath, boost::is_any_of("/, \\"));
	std::string filePath = segments[segments.size() - 1];
	return filePath.substr(0, filePath.size() - 4);
}

bool IOManager::getDesiredLoD()
{
	make00_ = false;
	make02_ = false;
	make10_ = false;
	make12_ = false;
	make13_ = false;
	make22_ = false;
	make32_ = false;

	std::cout << "Please select the desired output LoD" << std::endl;
	std::cout << "Can be a single string (e.g. 123 -> LoD0.1, 0.2, 1.0)" << std::endl;
	std::cout << "1. LoD0.0" << std::endl;
	std::cout << "2. LoD0.2" << std::endl;
	std::cout << "3. LoD1.0" << std::endl;
	std::cout << "4. LoD1.2" << std::endl;
	std::cout << "5. LoD1.3" << std::endl;
	std::cout << "6. LoD2.2" << std::endl;
	std::cout << "7. LoD3.2" << std::endl;
	std::cout << "8. All" << std::endl;

	while (true)
	{
		bool validInput = true;
		std::cout << "Num: ";

		std::string stringNum = "";
		std::getline(std::cin, stringNum);

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (!std::isdigit(stringNum[i]))
			{
				validInput = false;
			}
		}

		if (!validInput)
		{
			std::cout << "\n [INFO] Please enter a valid number! \n" << std::endl;
			continue;
		}

		for (size_t i = 0; i < stringNum.size(); i++)
		{
			if (stringNum[i] == '1') { make00_ = true; }
			if (stringNum[i] == '2') { make02_ = true; }
			if (stringNum[i] == '3') { make10_ = true; }
			if (stringNum[i] == '4') { make12_ = true; }
			if (stringNum[i] == '5') { make13_ = true; }
			if (stringNum[i] == '6') { make22_ = true; }
			if (stringNum[i] == '7') { make32_ = true; }
			if (stringNum[i] == '8') { 
				make00_ = true;
				make02_ = true;
				make10_ = true;
				make12_ = true;
				make13_ = true;
				make22_ = true;
				make32_ = true;
			}
		}
		return true;
	}
	return false;
}

bool IOManager::getBoudingRules()
{
	std::cout << "Please select a desired rulset for space bounding objects" << std::endl;
	std::cout << "1. Default room bounding objects" << std::endl;
	std::cout << "2. Default room bounding objects + IfcBuildingElementProxy objects" << std::endl;
	std::cout << "3. Default room bounding objects + custom object selection" << std::endl;
	std::cout << "4. custom object selection" << std::endl;

	while (true)
	{
		int ruleNum = numQuestion(3) + 1;

		if (ruleNum == 1)
		{
			return true;
		}

		if (ruleNum == 2)
		{
			useProxy_ = true;
			return true;
		}

		if (ruleNum == 3 || ruleNum == 4)
		{
			if (ruleNum == 3) { useDefaultDiv_ = false; }

			std::cout << std::endl;
			std::cout << "Please enter the desired IfcTypes" << std::endl;
			std::cout << "[INFO] Not case sensitive" << std::endl;
			std::cout << "[INFO] Seperate type by enter" << std::endl;
			std::cout << "[INFO] Finish by empty line + enter" << std::endl;

			while (true)
			{
				std::cout << "IfcType: ";

				std::string singlepath = "";

				std::getline(std::cin, singlepath);

				if (singlepath.size() == 0 && addDivObjects_.size() == 0)
				{
					std::cout << "[INFO] No type has been supplied" << std::endl;
					continue;
				}
				else if (singlepath.size() == 0)
				{
					std::cout << std::endl;
					return true;
				}
				else if (boost::to_upper_copy<std::string>(singlepath.substr(0, 3)) == "IFC") {

					std::string potentialType = boost::to_upper_copy<std::string>(singlepath);

					if (DevObjectsOptions_.find(potentialType) == DevObjectsOptions_.end())
					{
						std::cout << "[INFO] Type is not an IfcType" << std::endl;
						continue;
					}

					if (addDivObjects_.find(potentialType) != addDivObjects_.end())
					{
						std::cout << "[INFO] Type is already used as space bounding object" << std::endl;
						continue;
					}

					addDivObjects_.insert(potentialType);

				}
				else
				{
					std::cout << "[INFO] No valid type has been supplied" << std::endl;
					continue;
				}
			}
		}
		std::cout << "[INFO] No valid option was chosen" << std::endl;
	}
	return false;
}

bool IOManager::getVoxelSize()
{
	// ask user for desired voxel dimensions

	std::string stringXYSize = "";
	std::string stringZSize = "";

	while (true)
	{
		std::cout << "Enter voxel XY dimenion (double):";
		std::getline(std::cin, stringXYSize);

		char* end = nullptr;
		double val = strtod(stringXYSize.c_str(), &end);

		if (end != stringXYSize.c_str() && *end == '\0' && val != HUGE_VAL)
		{
			voxelSize_ = val;
			break;
		}

		std::cout << "[INFO] No valid number has been supplied" << std::endl;
	}
}

bool IOManager::getFootprintElev()
{
	std::string elev = "";
	while (true)
	{
		std::cout << "Please enter the footprint elevation: ";
		std::getline(std::cin, elev);

		double dEleve;
		auto iElev = std::istringstream(elev);
		iElev >> dEleve;

		if (!iElev.fail() && iElev.eof())
		{
			footprintElevation_ = dEleve;
			return true;
		}

		std::cout << "[INFO] No valid elevation has been supplied" << std::endl;
	}
	return false;
}

bool IOManager::getUserValues()
{
	// output targets

	std::cout << std::endl;
	if (getUseDefaultSettings()) { 
		printSummary();
		return true; 
	}

	std::cout << std::endl;
	if (!getOutputPathList()) { return false; }

	std::cout << std::endl;
	std::cout << "Create report file (Y/N): ";
	writeReport_ = yesNoQuestion();

	std::cout << std::endl;
	if (!getDesiredLoD()) { return false; }

	std::cout << std::endl;
	std::cout << "Customize space bounding objects? (Y/N): ";
	if (yesNoQuestion()) {
		if (!getBoudingRules()) { return false; }
	}

	std::cout << std::endl;
	std::cout << "Desired voxel size? : ";
	if (!getVoxelSize()) { return false; } 

	if (make02_)
	{
		std::cout << std::endl;
		getFootprintElev();
	}
	return true;
}

bool IOManager::getJSONValues()
{
	std::ifstream f(inputPathList_[0]);
	nlohmann::json json = nlohmann::json::parse(f);

	inputPathList_.clear();

	if (!json.contains("Filepaths"))
	{
		std::cout << "JSON file does not contain Filpaths Entry" << std::endl;
		return false;
	}

	nlohmann::json filePaths = json["Filepaths"];

	if (!filePaths.contains("Input"))
	{
		std::cout << "JSON file does not contain Input Filpath Entry" << std::endl;
		return false;
	}
	if (!filePaths.contains("Output"))
	{
		std::cout << "JSON file does not contain Output Filpath Entry" << std::endl;
		return false;
	}

	nlohmann::json inputPaths = filePaths["Input"];
	for (size_t i = 0; i < inputPaths.size(); i++) { inputPathList_.emplace_back(inputPaths[i]); }
	outputFolderPath_ = filePaths["Output"];

	if (json.contains("Output report"))
	{
		if (json["Output report"] == 0) { writeReport_ = false; }
	}

	if (json.contains("LoD output"))
	{
		nlohmann::json lodList = json["LoD output"];
		make00_ = false;
		make02_ = false;
		make10_ = false;
		make12_ = false;
		make13_ = false;
		make22_ = false;
		make32_ = false;

		for (size_t i = 0; i < lodList.size(); i++)
		{
			if (lodList[i] == 0.0) { make00_ = true; }
			else if (lodList[i] == 0.2) { make02_ = true; }
			else if (lodList[i] == 1.0) { make10_ = true; }
			else if (lodList[i] == 1.2) { make12_ = true; }
			else if (lodList[i] == 1.3) { make13_ = true; }
			else if (lodList[i] == 2.2) { make22_ = true; }
			else if (lodList[i] == 3.2) { make32_ = true; }
		}
	}

	if (make02_)
	{
		if (json.contains("Footprint elevation"))
		{
			footprintElevation_ = json["Footprint elevation"];
		}
	}

	if (json.contains("voxelSize"))
	{
		nlohmann::json voxelData = json["voxelSize"];

		if (voxelData.contains("xy")) { voxelSize_ = voxelData["xy"]; }
	}

	if (json.contains("Ignore Proxy"))
	{
		if (json["Ignore Proxy"] == 1)
		{
			useProxy_ = true;
		}
	}
	return true;
}

bool IOManager::hasExtension(const std::vector<std::string>& stringList, const std::string& ext)
{
	for (size_t i = 0; i < stringList.size(); i++)
	{
		if (!hasExtension(stringList[i], ext)) { return false; }
	}
	return true;
}

bool IOManager::hasExtension(const std::string& string, const std::string& ext)
{
	std::string substring = boost::to_lower_copy<std::string>(string.substr(string.find_last_of(".") + 1));
	if (substring == ext) { return true; }
	return false;
}

void IOManager::printSummary()
{
	std::cout << "=============================================================" << std::endl;
	std::cout << "[INFO] Used settings: " << std::endl;

	std::cout << "- Input File(s):" << std::endl;
	for (size_t i = 0; i < inputPathList_.size(); i++) { std::cout << "    " << inputPathList_[i] << std::endl; }
	std::cout << "- Output Folder:" << std::endl;
	std::cout << "    " << outputFolderPath_ << std::endl;
	std::cout << "- Create Report:" << std::endl;
	if (writeReport_) { std::cout << "    yes" << std::endl; }
	else { std::cout << "    no" << std::endl; }
	std::cout << "- LoD export enabled:" << std::endl;
	std::cout << "    " << getLoDEnabled() << std::endl;
	std::cout << "- Space dividing objects: " << std::endl;
	if (useDefaultDiv_)
	{
		for (auto it = divObjects_.begin(); it != divObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	}
	if (useProxy_)
	{
		std::cout << "    IFCBUILDINGELEMENTPROXY" << std::endl;
	}
	for (auto it = addDivObjects_.begin(); it != addDivObjects_.end(); ++it) { std::cout << "    " << boost::to_upper_copy(*it) << std::endl; }
	std::cout << "- Voxel size:" << std::endl;
	std::cout << "    " << voxelSize_ << std::endl;

	if (make02_)
	{
		std::cout << "- footprint Elevation:" << std::endl;
		std::cout << "    " << footprintElevation_ << std::endl;
	}


	std::cout << "=============================================================" << std::endl;
}

std::string IOManager::getLoDEnabled()
{
	std::string summaryString = "";

	if (make00_) { summaryString += ", 0.0"; }
	if (make02_) { summaryString += ", 0.2"; }
	if (make12_) { summaryString += ", 1.2"; }
	if (make13_) { summaryString += ", 1.3"; }
	if (make22_) { summaryString += ", 2.2"; }
	if (make32_) { summaryString += ", 3.2"; }

	summaryString.erase(0, 2);

	return summaryString;
}

bool IOManager::init(const std::vector<std::string>& inputPathList, bool silent)
{
	auto internalizingTime = std::chrono::high_resolution_clock::now(); // Time Collection Starts
	timeTotal = 0;

	bool isSilent_ = silent;

	// find if input has IFC format
	bool isIFC = true;

	if (!isSilent_)
	{
		std::wcout << "============================================================= \n" << std::endl;
		std::cout << "		IFC_BuildingEnvExtractor" << std::endl;
		std::cout << "    Experimental building envelope extractor/approximation\n" << std::endl;
		std::wcout << "=============================================================" << std::endl;
		std::cout << std::endl;
	}

	bool hasDirectInterface = false;
	if (inputPathList.size() == 0)
	{
		getTargetPathList();
		hasDirectInterface = true;
	}
	else if (hasExtension(inputPathList, "ifc")) // If all files are IFC copy the path list and ask user for info
	{ 
		inputPathList_ = inputPathList; 
		if (!isSilent_)
		{
			std::cout << "[INFO] Input file path(s):" << std::endl;
			for (size_t i = 0; i < inputPathList_.size(); i++) { std::cout << inputPathList_[i] << std::endl; }
			std::cout << std::endl;
		}
		hasDirectInterface = true;
	} 
	else if (hasExtension(inputPathList, "json")) {
		isJsonInput_ = true;
		inputPathList_ = inputPathList;
	}
	if (!isJsonInput_)
	{
		if (!getUserValues()) { return false; } // attempt to ask user for data		 
	}
	else
	{
		if (inputPathList.size() > 1) { return false; }
		isJsonInput_ = true;
		if (!getJSONValues()) { return false; }  // attempt to get data from json
	}

	if (!isSilent_)
	{
		std::cout << std::endl;
		printSummary();
	}

	// init helpers
	for (size_t i = 0; i < inputPathList_.size(); i++)
	{
		std::string currentPath = inputPathList_[i];

		std::cout << "[INFO] Parsing file " << currentPath << std::endl;
		helper h = helper(currentPath);

		if (inputPathList_.size() > 1) {
			h.setDepending(true);

			// set the construction model
			if (i == 0) { h.setIsConstruct(true); }
		}
		else { h.setIsConstruct(true); }
		if (!h.hasSetUnits()) { return 0; }
		h.setName(getFileName(currentPath));
		h.setfootprintLvl(footprintElevation_);
		//h->setRoomBoundingObjects(, !useDefaultDiv_, );

		hCluster_.appendHelper(h);
	}
	hCluster_.setUseProxy(useProxy_);
	timeTotal = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - internalizingTime).count();
	return true;
}

bool IOManager::run()
{
	hCluster_.internaliseData();
	for (int i = 0; i < hCluster_.getSize(); i++) { hCluster_.getHelper(i)->indexGeo(); }
	return true;
}
