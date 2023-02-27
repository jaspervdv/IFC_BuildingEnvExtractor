#include "helper.h"


void WriteToSTEP(TopoDS_Solid shape, std::string addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";

	STEPControl_Writer writer;

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
	IFSelect_ReturnStatus stat = writer.Write(path.c_str());

	//std::cout << "stat: " << stat << std::endl;
}

void WriteToSTEP(TopoDS_Shape shape, std::string addition) {
	std::string path = "D:/Documents/Uni/Thesis/sources/Models/exports/step" + addition + ".stp";
	STEPControl_Writer writer;

	/*TopExp_Explorer expl;
	for (expl.Init(shape, TopAbs_SOLID); expl.More(); expl.Next()) {
		writer.Transfer(expl.Current(), STEPControl_ManifoldSolidBrep);
	}

	IFSelect_ReturnStatus stat = writer.Write(path.c_str());*/

	writer.Transfer(shape, STEPControl_ManifoldSolidBrep);
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
		std::cout << "Start of Face" << std::endl;
		for (expl.Init(faceList[i], TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			printPoint(p);
		}
		std::cout << "End of Face" << std::endl;
		std::cout << std::endl;
	}
	//std::cout << std::endl;
}


gp_Pnt rotatePointWorld(gp_Pnt p, double angle) {
	double pX = p.X();
	double pY = p.Y();
	double pZ = p.Z();

	return gp_Pnt(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

BoostPoint3D rotatePointWorld(BoostPoint3D p, double angle) {
	double pX = bg::get<0>(p);
	double pY = bg::get<1>(p);
	double pZ = bg::get<2>(p);

	return BoostPoint3D(pX * cos(angle) - pY * sin(angle), pY * cos(angle) + pX * sin(angle), pZ);
}

std::tuple<gp_Pnt, gp_Pnt, double> rotatedBBoxDiagonal(std::vector<gp_Pnt> pointList, double angle) {
	
	bool isPSet = false;
	gp_Pnt lllPoint;
	gp_Pnt urrPoint;

	for (size_t i = 0; i < pointList.size(); i++)
	{
		gp_Pnt point = rotatePointWorld(pointList[i], angle);

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
			typeName != "IfcWallStandardCase" &&
			typeName != "IfcWall")
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
		std::cout << "[Error] SI unit for volume cannot be found!" << std::endl;
		return;
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
	// get products
	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	objectCount = products->size();
	for (auto it = products->begin(); it != products->end(); ++it) {
		IfcSchema::IfcProduct* product = *it;

		if (product->data().type()->name() == "IfcBuildingElementProxy")
		{
			proxyCount++;
		}
	}

	if (proxyCount > 0)
	{
		hasProxy = true;
	}
	if (proxyCount/objectCount >= maxProxyP)
	{
		hasLotProxy = true;
	} 
	std::vector<gp_Pnt> pointList = getAllPoints(products);

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

void helper::internalizeGeo(double angle) {
	std::cout << "Internalizing Geometry\n" << std::endl;

	IfcSchema::IfcProduct::list::ptr products = file_->instances_by_type<IfcSchema::IfcProduct>();
	std::vector<gp_Pnt> pointList = getAllPoints(products);

	auto bbox = rotatedBBoxDiagonal(pointList, angle);

	lllPoint_ = std::get<0>(bbox);
	urrPoint_ = std::get<1>(bbox);
	originRot_ = angle;

	hasGeo = true;
}

void helper::indexGeo()
{
	// this indexing is done based on the rotated bboxes of the objects
	// the bbox does thus comply with the model bbox but not with the actual objects original location

	if (!hasIndex_)
	{
		if (!useCustomFull)
		{
			// add the floorslabs to the rtree
			addObjectToIndex<IfcSchema::IfcSlab::list::ptr>(file_->instances_by_type<IfcSchema::IfcSlab>());
			addObjectToIndex<IfcSchema::IfcRoof::list::ptr>(file_->instances_by_type<IfcSchema::IfcRoof>());

			// add the walls to the rtree
			addObjectToIndex<IfcSchema::IfcWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcWall>());
			addObjectToIndex<IfcSchema::IfcCovering::list::ptr>(file_->instances_by_type<IfcSchema::IfcCovering>());

			// add the columns to the rtree TODO sweeps
			addObjectToIndex<IfcSchema::IfcColumn::list::ptr>(file_->instances_by_type<IfcSchema::IfcColumn>());

			// add the beams to the rtree
			addObjectToIndex<IfcSchema::IfcBeam::list::ptr>(file_->instances_by_type<IfcSchema::IfcBeam>());

			// add the curtain walls to the rtree
			addObjectToIndex<IfcSchema::IfcCurtainWall::list::ptr>(file_->instances_by_type<IfcSchema::IfcCurtainWall>());
			addObjectToIndex<IfcSchema::IfcPlate::list::ptr>(file_->instances_by_type<IfcSchema::IfcPlate>());
			addObjectToIndex<IfcSchema::IfcMember::list::ptr>(file_->instances_by_type<IfcSchema::IfcMember>());

			// add doors to the rtree (for the appartment detection)
			addObjectToIndex<IfcSchema::IfcDoor::list::ptr>(file_->instances_by_type<IfcSchema::IfcDoor>());
			addObjectToIndex<IfcSchema::IfcWindow::list::ptr>(file_->instances_by_type<IfcSchema::IfcWindow>());

			if (useProxy)
			{
				addObjectToIndex<IfcSchema::IfcBuildingElementProxy::list::ptr>(file_->instances_by_type<IfcSchema::IfcBuildingElementProxy>());
			}
		}
		if (useCustom)
		{
			IfcSchema::IfcProduct::list::ptr productList = file_->instances_by_type<IfcSchema::IfcProduct>();
			
			for (auto it = roomBoundingObjects_->begin(); it != roomBoundingObjects_->end(); ++it) {
				
				IfcSchema::IfcProduct::list::ptr selectedlist(new IfcSchema::IfcProduct::list);

				for (auto et = productList->begin(); et != productList->end(); ++et)
				{
					IfcSchema::IfcProduct* product = *et;

					if (*it == boost::to_upper_copy<std::string>(product->data().type()->name()))
					{
						selectedlist.get()->push(product);
					}
				}
				addObjectToIndex<IfcSchema::IfcProduct::list::ptr>(selectedlist);
			}
		}

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

TopoDS_Solid helper::makeSolidBox(gp_Pnt lll, gp_Pnt urr, double angle)
{
	BRep_Builder brepBuilder;
	BRepBuilderAPI_Sewing brepSewer;

	TopoDS_Shell shell;
	brepBuilder.MakeShell(shell);
	TopoDS_Solid solidbox;
	brepBuilder.MakeSolid(solidbox);

	gp_Pnt p0(rotatePointWorld(lll, -angle));
	gp_Pnt p1 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), lll.Z()), -angle);
	gp_Pnt p2 = rotatePointWorld(gp_Pnt(urr.X(), urr.Y(), lll.Z()), -angle);
	gp_Pnt p3 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), lll.Z()), -angle);

	gp_Pnt p4(rotatePointWorld(urr, -angle));
	gp_Pnt p5 = rotatePointWorld(gp_Pnt(lll.X(), urr.Y(), urr.Z()), -angle);
	gp_Pnt p6 = rotatePointWorld(gp_Pnt(lll.X(), lll.Y(), urr.Z()), -angle);
	gp_Pnt p7 = rotatePointWorld(gp_Pnt(urr.X(), lll.Y(), urr.Z()), -angle);

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

			distanceMeasurer.LoadS1(BRepBuilderAPI_MakeVertex(mP)); //TODO: check for outliers
			//distanceMeasurer.LoadS1(shape); //TODO: check for outliers

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

			if (!matchFound)
			{
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

				// get rotation in xy plane
				gp_Pnt bp(0, 0, 0);

				double bpDistance0 = bp.Distance(horizontalMaxEdge[0]);
				double bpDistance1 = bp.Distance(horizontalMaxEdge[1]);

				if (bpDistance0 > bpDistance1)
				{
					std::reverse(horizontalMaxEdge.begin(), horizontalMaxEdge.end());
				}

				gp_Pnt p1 = horizontalMaxEdge[0];
				gp_Pnt p2 = horizontalMaxEdge[1];

				double angle = 0;

				if (abs(p1.Y() - p2.Y()) > 0.00001)
				{
					double os = (p1.Y() - p2.Y()) / p1.Distance(p2);
					angle = asin(os);
				}

				auto base = rotatedBBoxDiagonal(pointList, angle);
				auto base2 = rotatedBBoxDiagonal(pointList, -angle);

				gp_Pnt lllPoint = std::get<0>(base);
				gp_Pnt urrPoint = std::get<1>(base);
				double rot = angle;

				if (lllPoint.Distance(urrPoint) > std::get<0>(base2).Distance(std::get<1>(base2)))
				{
					lllPoint = std::get<0>(base2);
					urrPoint = std::get<1>(base2);

					rot = -angle;
				}

				hasCBBox = true;
				cbbox = makeSolidBox(lllPoint, urrPoint, rot);
			}
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

	//std::cout << product->data().toString() << std::endl;
	if (!product->hasRepresentation()) { 
		std::vector<IfcSchema::IfcProduct*> productList;

#ifdef USE_IFC4
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() == 0) { return { gp_Pnt(0.0, 0.0, 0.0) }; }
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

		for (size_t i = 0; i < productList.size(); i++)
		{
			TopoDS_Shape rShape = getObjectShape(productList[i]);
			TopExp_Explorer expl;
			for (expl.Init(rShape, TopAbs_VERTEX); expl.More(); expl.Next())
			{
				TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
				gp_Pnt p = BRep_Tool::Pnt(vertex);
				pointList.emplace_back(p);
			}
		}
	}
	else {

		//std::cout << product->data().toString() << std::endl;

		TopoDS_Shape rShape = getObjectShape(product, simple);

		TopExp_Explorer expl;
		for (expl.Init(rShape, TopAbs_VERTEX); expl.More(); expl.Next())
		{
			TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
			gp_Pnt p = BRep_Tool::Pnt(vertex);
			pointList.emplace_back(p);
		}
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
		TopoDS_Vertex vertex = TopoDS::Vertex(expl.Current());
		gp_Pnt p = BRep_Tool::Pnt(vertex);
		pointList.emplace_back(p);
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

TopoDS_Shape helper::getObjectShape(IfcSchema::IfcProduct* product, bool adjusted)
{
	if (product->data().type()->name() == "IfcFastener")
	{
		return {};
	}


	// filter with lookup
	if (product->data().type()->name() != "IfcWall" &&
		product->data().type()->name() != "IfcWallStandardCase" &&
		product->data().type()->name() != "IfcRoof" &&
		product->data().type()->name() != "IfcSlab" )
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

	if (!product->hasRepresentation()) {


#ifdef USE_IFC4
		IfcSchema::IfcRelAggregates::list::ptr decomposedProducts = product->IsDecomposedBy();
#else
		IfcSchema::IfcRelDecomposes::list::ptr decomposedProducts = product->IsDecomposedBy();
#endif // USE_IFC4

		if (decomposedProducts->size() == 0) { return { }; }

		BRep_Builder builder;
		TopoDS_Compound collection;
		builder.MakeCompound(collection);

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

				TopoDS_Shape addshape = getObjectShape(addprod);
				builder.Add(collection, addshape);
			}
		}
		return collection;
	}

	int id = product->data().id();

	bool hasHoles = false;
	bool isFloor = false;

	if (product->data().type()->name() == "IfcWall" ||
		product->data().type()->name() == "IfcWallStandardCase" ||
		product->data().type()->name() == "IfcRoof" ||
		product->data().type()->name() == "IfcSlab") {
		hasHoles = true;
	}
	if (product->data().type()->name() == "IfcSlab") { isFloor = true; }


	TopoDS_Compound comp;
	TopoDS_Compound simpleComp;

	IfcSchema::IfcRepresentation* ifc_representation = 0;

	IfcSchema::IfcProductRepresentation* prodrep = product->Representation();
	IfcSchema::IfcRepresentation::list::ptr reps = prodrep->Representations();

	for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
		IfcSchema::IfcRepresentation* rep = *it;
		if (rep->RepresentationIdentifier() == "Body") {
			ifc_representation = rep;
			break;
		}
	}

	if (ifc_representation == 0)
	{
		for (IfcSchema::IfcRepresentation::list::it it = reps->begin(); it != reps->end(); ++it) {
			IfcSchema::IfcRepresentation* rep = *it;

			if (rep->RepresentationIdentifier() == "Annotation") {
				ifc_representation = rep;
				break;
			}
		}
	}


	IfcGeom::IteratorSettings settings;
	// TODO: monitor the effect of doing this
	//if (isFloor) { settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true); }

	if (!ifc_representation)
	{
		return {};
	}

	if (ifc_representation->RepresentationIdentifier() == "Annotation")
	{

		gp_Trsf placement;
		gp_Trsf trsf;

		settings.set(settings.INCLUDE_CURVES, true);

		kernel_->convert_placement(product->ObjectPlacement(), trsf);
		IfcGeom::BRepElement<double, double>* brep = kernel_->convert(settings, ifc_representation, product);
		kernel_->convert_placement(ifc_representation, placement);
		//comp = brep->geometry().as_compound();


		/*gp_Trsf trsf;
		kernel_->convert_placement(product->ObjectPlacement(), trsf);

		IfcSchema::IfcRepresentationItem::list::ptr representationItems = ifc_representation->Items();

		for (auto it = representationItems->begin(); it != representationItems->end(); ++it)
		{
			IfcSchema::IfcRepresentationItem* representationItem = *it;

			if (representationItem->data().type()->name() == "IfcTextLiteralWithExtent") { continue; }

		}

		*/
		//std::cout << representationItems->data().toString() << std::endl;

		// data is never deleted, can be used later as internalized data
		//IfcGeom::IfcRepresentationShapeItems ob(kernel_->convert(representationItems));



	}
	else if (ifc_representation->RepresentationIdentifier() == "Body")
	{
		gp_Trsf placement;
		gp_Trsf trsf;

		kernel_->convert_placement(product->ObjectPlacement(), trsf);
		IfcGeom::BRepElement<double, double>* brep = kernel_->convert(settings, ifc_representation, product);
		kernel_->convert_placement(ifc_representation, placement);

		comp = brep->geometry().as_compound();
		comp.Move(trsf * placement); // location in global space


		if (hasHoles)
		{
			settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, true);
			brep = kernel_->convert(settings, ifc_representation, product);
			kernel_->convert_placement(ifc_representation, placement);

			simpleComp = brep->geometry().as_compound();
			simpleComp.Move(trsf * placement); // location in global space
		}
	}

	shapeLookup_[product->data().id()] = comp;

	if (hasHoles)
	{
		adjustedshapeLookup_[product->data().id()] = simpleComp;
		
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

			//printFaces(substractionShape);
			
			auto base = rotatedBBoxDiagonal(getObjectPoints(openingElement), 0);
			gp_Pnt lllPoint = std::get<0>(base);
			gp_Pnt urrPoint = std::get<1>(base);

			//printPoint(lllPoint);
			//printPoint(urrPoint);

			std::vector<Value> qResult;
			boost::geometry::model::box<BoostPoint3D> qBox = bg::model::box<BoostPoint3D>(Point3DOTB(lllPoint), Point3DOTB(urrPoint));

			//printPoint(lllPoint);
			//printPoint(urrPoint);

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

	if (!helperList[0]->getDepending())
	{
		helperList[0]->internalizeGeo();

		lllPoint_ = helperList[0]->getLllPoint();
		urrPoint_ = helperList[0]->getUrrPoint();
		originRot_ = helperList[0]->getRotation();

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

		if (helper->getIsConstruct())
		{
			helper->internalizeGeo();
			lllPoint_ = helper->getLllPoint();
			urrPoint_ = helper->getUrrPoint();
			originRot_ = helper->getRotation();
		}
	}

	for (size_t i = 0; i < size_; i++)
	{
		auto helper = helperList[i];

		if (!helper->getHasGeo())
		{
			helper->internalizeGeo(originRot_);

			// update bbox if needed
			gp_Pnt addLllPoint = helper->getLllPoint();
			gp_Pnt addUrrPoint = helper->getUrrPoint();

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

void helperCluster::appendHelper(helper* data)
{
	helperList.emplace_back(data);
	size_++;
}

void helperCluster::updateConnections(TopoDS_Shape room, roomObject* rObject, boost::geometry::model::box<BoostPoint3D> qBox, std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> connectedObjects)
{

	int cSize = size_;

	for (size_t i = 0; i < cSize; i++)
	{
		if (!helperList[i]->hasClookup())
		{
			helperList[i]->indexConnectiveShapes();
		}
	}

	std::vector<IfcSchema::IfcProduct*> connectedDoors;

	for (size_t i = 0; i < connectedObjects.size(); i++)
	{
		if (std::get<0>(connectedObjects[i])->data().type()->name() == "IfcDoor")
		{
			connectedDoors.emplace_back(std::get<0>(connectedObjects[i]));
		}
	}


	int doorCount = 0;
	int stairCount = 0;

	// find connectivity data
	// Get lowest Face
	std::vector<TopoDS_Face> roomFootPrintList = getRoomFootprint(room);

	TopoDS_Vertex lDoorP;

	std::vector<Value> qResult;
	TopExp_Explorer expl;

	for (int j = 0; j < cSize; j++)
	{
		qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
		helperList[j]->getConnectivityIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

		for (size_t k = 0; k < qResult.size(); k++)
		{
			ConnectLookupValue lookup = helperList[j]->getCLookup(qResult[k].second);
			
			if (std::get<0>(lookup)->data().type()->name() == "IfcDoor") { 

				for (size_t i = 0; i < connectedDoors.size(); i++)
				{
					IfcSchema::IfcProduct* qProduct = connectedDoors[i];
					if (std::get<0>(lookup)->data().id() == qProduct->data().id())
					{
						//doorCount++;
						std::get<2>(lookup)->emplace_back(rObject);

						if (std::get<2>(lookup)->size() == 2)
						{
							bool found = false;
							auto connection = rObject->getConnections();

							for (size_t l = 0; l < connection.size(); l++)
							{
								if (connection[l] == std::get<2>(lookup)[0][0])
								{
									found = true;
									break;
								}
							}

							if (!found)
							{
								doorCount++;
								std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
								std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
							}
						}
						else
						{
							doorCount++;
						}
						
						break;
					}					
				}
			}

			if (std::get<0>(lookup)->data().type()->name() == "IfcStair") {
				gp_Trsf stairOffset;

				gp_Pnt groundObjectPoint = std::get<1>(lookup)[0];
				groundObjectPoint.SetZ(groundObjectPoint.Z() + 0.5);
				gp_Pnt groundObjectPointHigh = groundObjectPoint;
				groundObjectPointHigh.SetZ(groundObjectPoint.Z() + 1000);

				std::vector<gp_Pnt> lowLine = { groundObjectPoint,  groundObjectPointHigh };
				auto faceTriangles = triangulateShape(&room);

				int iCount = 0;
				for (size_t l = 0; l < faceTriangles.size(); l++)
				{
					if (triangleIntersecting(lowLine, faceTriangles[l]))
					{
						iCount ++;
					}
				}

				if (iCount % 2 != 0) {
					std::get<2>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<2>(lookup)->size() == 2)
					{
						std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
						std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
					}

					continue;
				}

				gp_Pnt topObjectPoint = std::get<1>(lookup)[1];
				topObjectPoint.SetZ(topObjectPoint.Z() + .5);
				gp_Pnt topObjectPointHigh = topObjectPoint;
				topObjectPointHigh.SetZ(topObjectPoint.Z() + 1000);

				std::vector<gp_Pnt> highLine = { topObjectPoint,  topObjectPointHigh };

				iCount = 0;
				for (size_t l = 0; l < faceTriangles.size(); l++)
				{
					if (triangleIntersecting(highLine, faceTriangles[l]))
					{
						iCount++;
					}
				}

				if (iCount % 2 != 0) {
					std::get<2>(lookup)->emplace_back(rObject);
					stairCount++;
					if (std::get<2>(lookup)->size() == 2)
					{
						std::get<2>(lookup)[0][0]->addConnection(std::get<2>(lookup)[0][1]);
						std::get<2>(lookup)[0][1]->addConnection(std::get<2>(lookup)[0][0]);
					}
					continue;
				}
			}
		}
	}
	rObject->setDoorCount(doorCount);
	rObject->getSelf()->setDescription("Has " + std::to_string(doorCount) + " unique doors and " + std::to_string(stairCount) + " unique stairs. Connected to : ");
}

std::vector<roomObject*> helperCluster::createGraphData()
{

	double scaler = 10;

	std::vector<roomObject*> roomObjectList;

	for (size_t i = 0; i < size_; i++)
	{
		helper* h = helperList[i];

		if (!h->hasIndex())
		{
			h->indexGeo();
		}

		IfcSchema::IfcSpace::list::ptr spaceList =  h->getSourceFile()->instances_by_type<IfcSchema::IfcSpace>();

		for (auto it = spaceList->begin(); it != spaceList->end(); ++it) {

			IfcSchema::IfcSpace* currentSpace = *it;
			roomObject* rObject = new roomObject(currentSpace, roomObjectList.size());

			TopoDS_Shape roomShape = h->getObjectShape(currentSpace);
			std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList;

			auto roomFootprint = getRoomFootprint(roomShape);

			double area = 0;

			GProp_GProps gprop;
			for (size_t j = 0; j < roomFootprint.size(); j++)
			{
				BRepGProp::SurfaceProperties(roomFootprint[j], gprop);
				area += gprop.Mass();
			}

			rObject->setArea(area);

			auto base = rotatedBBoxDiagonal(h->getObjectPoints(currentSpace), originRot_);
			BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(base));
			BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(base));

			double dx = (bg::get<0>(boosturrpoint) - bg::get<0>(boostlllpoint)) / scaler;
			double dy = (bg::get<1>(boosturrpoint) - bg::get<1>(boostlllpoint)) / scaler;
			double dz = (bg::get<2>(boosturrpoint) - bg::get<2>(boostlllpoint)) / scaler;

			BoostPoint3D scaledlllpoint;
			BoostPoint3D scaledurrpoint;

			boost::geometry::strategy::transform::translate_transformer<double, 3, 3> translate(dx, dy, 2 * dz);
			bg::transform(boosturrpoint, scaledurrpoint, translate);
			boost::geometry::strategy::transform::translate_transformer<double, 3, 3> translate2(-dx, -dy, -2 * dz);
			bg::transform(boostlllpoint, scaledlllpoint, translate2);

			auto qBox = bg::model::box < BoostPoint3D >(scaledlllpoint, scaledurrpoint);

			std::vector<Value> qResult;

			qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
			h->getIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

			if (qResult.size() == 0) { continue; }

			for (size_t k = 0; k < qResult.size(); k++)
			{
				LookupValue lookup = h->getLookup(qResult[k].second);
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);
				TopoDS_Shape shape;

				if (std::get<3>(lookup))
				{
					qProductList.emplace_back(std::make_tuple(qProduct, std::get<4>(lookup)));
				}
				else {
					qProductList.emplace_back(std::make_tuple(qProduct, h->getObjectShape(std::get<0>(lookup), false)));
				}
			}

			auto connectedObjects = checkConnection(roomShape, currentSpace, qProductList);
			updateConnections(roomShape, rObject, qBox, connectedObjects);
			roomObjectList.emplace_back(rObject);
		}
	}
	return roomObjectList;
}

std::vector<roomObject*> helperCluster::createGraph(std::vector<roomObject*> rObjectList) {
	// update data to outside 
	int cSize = size_;

	// create outside object
	roomObject* outsideObject = new roomObject(nullptr, rObjectList.size());
	outsideObject->setIsOutSide();

	//roomObjectList.emplace_back(outsideObject);
	rObjectList.insert(rObjectList.begin(), outsideObject);

	for (size_t i = 0; i < cSize; i++)
	{
		std::vector<ConnectLookupValue> lookup = helperList[i]->getFullClookup();

		for (size_t j = 0; j < lookup.size(); j++)
		{
			if (std::get<2>(lookup[j])->size() == 1)
			{
				std::vector<gp_Pnt> doorPointList = helperList[i]->getObjectPoints(std::get<0>(lookup[j]));

				double lowZ = 99999999999;
				for (size_t k = 0; k < doorPointList.size(); k++)
				{
					double pZ = doorPointList[k].Z();
					if (pZ < lowZ) { lowZ = pZ; }
				}

				if (lowZ < 2 && lowZ > -0.5) //TODO door height needs to be smarter
				{
					std::get<2>(lookup[j])[0][0]->addConnection(outsideObject);
				}

			}
		}
	}

	// Make sections
	int counter = 0;

	std::vector<roomObject*> bufferList;

	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		roomObject* currentRoom = rObjectList[i];

		if (!currentRoom->isInside())
		{
			currentRoom->setSNum(-2);
			continue;
		}

		if (currentRoom->getConnections().size() != 1) { continue; } // always start isolated
		if (currentRoom->getSNum() != -1) { continue; }

		bufferList.emplace_back(currentRoom);
		currentRoom->setSNum(counter);

		double totalAreaApartment = 0;

		std::vector<roomObject*> waitingList;
		std::vector<roomObject*> currentApp;
		std::vector<roomObject*> recurseList;

		// growing of an appartement
		while (bufferList.size() > 0)
		{
			std::vector<roomObject*> tempBufferList;
			tempBufferList.clear();

			for (size_t j = 0; j < bufferList.size(); j++)
			{
				roomObject* evaluatedRoom = bufferList[j];
				std::vector<roomObject*> connections = evaluatedRoom->getConnections();

				if (evaluatedRoom->getDoorCount() >= hallwayNum_)
				{
					waitingList.emplace_back(evaluatedRoom);
					continue;
				}
				currentApp.emplace_back(evaluatedRoom);

				totalAreaApartment += evaluatedRoom->getArea();

				for (size_t k = 0; k < connections.size(); k++)
				{
					if (connections[k]->getSNum() == -1 && connections[k]->isInside())
					{
						
						connections[k]->setSNum(counter);
						tempBufferList.emplace_back(connections[k]);
					}
					else if (connections[k]->getSNum() >= 0 && connections[k]->getSNum() != counter)
					{
						recurseList.emplace_back(connections[k]);
					}
				}
			}

			if (tempBufferList.size() == 0)
			{
				if (totalAreaApartment < minArea_ ||
					currentApp.size() < minRoom_)
				{
					for (size_t j = 0; j < waitingList.size(); j++)
					{
						roomObject* evaluatedRoom = waitingList[j];
						currentApp.emplace_back(evaluatedRoom);
						totalAreaApartment += evaluatedRoom->getArea();
						std::vector<roomObject*> connections = evaluatedRoom->getConnections();

						for (size_t k = 0; k < connections.size(); k++)
						{
							if (connections[k]->getSNum() == -1 && connections[k]->isInside())
							{
								connections[k]->setSNum(counter);
								tempBufferList.emplace_back(connections[k]);
							}
							else if (connections[k]->getSNum() >= 0 && connections[k]->getSNum() != counter)
							{
								recurseList.emplace_back(connections[k]);
							}
						}
					}
				}
				else if (totalAreaApartment > minArea_)
				{
					for (size_t j = 0; j < waitingList.size(); j++)
					{
						waitingList[j]->setSNum(-1);
					}
				}
				waitingList.clear();
			}

			if (tempBufferList.size() == 0)
			{
				if (recurseList.size() != 0) {
					for (size_t j = 0; j < currentApp.size(); j++)
					{
						currentApp[j]->setSNum(recurseList[0]->getSNum());
					}
				}
				recurseList.clear();
			}

			
			bufferList.clear();
			bufferList = tempBufferList;
		}
		currentApp.clear();
		counter++;
		totalAreaApartment = 0;
	}

	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		roomObject* currentRoom = rObjectList[i];
		if (currentRoom->isInside())
		{
			currentRoom->getSelf()->setDescription(currentRoom->getSelf()->Description() + "apartment: " + std::to_string(currentRoom->getSNum()));
		}

	}


	return rObjectList;
}

void helperCluster::writeGraph(std::string path, std::vector<roomObject*> rObjectList) {
	
	// output the graph data
	std::string p = path;

	std::ofstream storageFile;
	storageFile.open(path);

	storageFile << "_pointList_" << std::endl;
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		gp_Pnt p = rObjectList[i]->getPoint();
		storageFile << p.X() << ", " << p.X() << ", " << p.Z() << std::endl;
	}
	storageFile << "_name_" << std::endl;
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		if (rObjectList[i]->isInside())
		{
			if (rObjectList[i]->getSelf()->hasLongName()) { storageFile << rObjectList[i]->getSelf()->LongName() << std::endl; }
			else { storageFile << rObjectList[i]->getSelf()->Name() << std::endl; }
		}
		else
		{
			storageFile << "Outside" << std::endl;
		}
	}

	storageFile << "_area_" << std::endl;
	storageFile << 100 << std::endl;
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		if (rObjectList[i]->isInside())
		{
			storageFile << rObjectList[i]->getArea() << std::endl;
		}
	}

	storageFile << "_connection_" << std::endl;
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		auto connections = rObjectList[i]->getConnections();

		for (size_t j = 0; j < connections.size(); j++)
		{
			if (rObjectList[i]->getIdx() + 1 >= rObjectList.size())
			{
				storageFile << 0 << ", " << connections[j]->getIdx() + 1 << std::endl;
			}
			else if (connections[j]->getIdx() + 1 >= rObjectList.size())
			{
				storageFile << rObjectList[i]->getIdx() + 1 << ", " << 0 << std::endl;
			}
			else
			{
				storageFile << rObjectList[i]->getIdx() + 1 << ", " << connections[j]->getIdx() + 1 << std::endl;
			}

		}
	}

	storageFile << "_sections_" << std::endl;
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		storageFile << rObjectList[i]->getSNum() << std::endl;
	}
}

void helperCluster::updateRoomCData(std::vector<roomObject*> rObjectList)
{
	for (size_t i = 0; i < rObjectList.size(); i++)
	{
		roomObject* rObject = rObjectList[i];
		auto connections = rObject->getConnections();
		std::string description = "";
		for (size_t j = 0; j < connections.size(); j++) { 
			description = description + connections[j]->getSelf()->Name() + ", "; 
		}

		if (rObject->getSelf()->hasDescription())
		{
			rObject->getSelf()->setDescription(rObject->getSelf()->Description() + description);
		}
		else
		{
			rObject->getSelf()->setDescription(description);
		}
	}
}

void helperCluster::determineRoomBoundaries() {
	// remove all rell space boundaries
	for (size_t i = 0; i < size_; i++)
	{
		IfcSchema::IfcRelSpaceBoundary::list::ptr rSBList = helperList[i]->getSourceFile()->instances_by_type<IfcSchema::IfcRelSpaceBoundary>();

		for (IfcSchema::IfcRelSpaceBoundary::list::it it = rSBList->begin(); it != rSBList->end(); ++it)
		{
			IfcSchema::IfcRelSpaceBoundary* rSB = *it;
			helperList[i]->getSourceFile()->removeEntity(rSB);
		}
	}
	
	for (size_t i = 0; i < size_; i++)
	{
		helper* h = helperList[i];

		std::cout << "File " << i + 1 << " of " << size_ << std::endl;
		IfcSchema::IfcSpace::list::ptr spaceList = (h->getSourceFile()->instances_by_type<IfcSchema::IfcSpace>());

		int spaceListSize = spaceList->size();

		if (spaceListSize == 0)
		{
			std::cout << "No IfcSpace objects found" << std::endl;
			continue;
		}

		if (!h->hasIndex())
		{
			h->indexGeo();
		}


		double scaler = 10;

		std::cout << std::endl;

		int counter = 0;
		for (auto it = spaceList->begin(); it != spaceList->end(); ++it) {

			std::cout.flush();
			std::cout << counter << " of " << spaceListSize << "\r";
			counter++;

			IfcSchema::IfcSpace* currentSpace = *it;
			TopoDS_Shape spaceShape = h->getObjectShape(currentSpace);
			std::vector<std::tuple<IfcSchema::IfcProduct*, TopoDS_Shape>> qProductList;

			auto base = rotatedBBoxDiagonal(h->getObjectPoints(spaceShape), originRot_);
			BoostPoint3D boostlllpoint = Point3DOTB(std::get<0>(base));
			BoostPoint3D boosturrpoint = Point3DOTB(std::get<1>(base));

			double dx = (bg::get<0>(boosturrpoint) - bg::get<0>(boostlllpoint)) / scaler;
			double dy = (bg::get<1>(boosturrpoint) - bg::get<1>(boostlllpoint)) / scaler;
			double dz = (bg::get<2>(boosturrpoint) - bg::get<2>(boostlllpoint)) / scaler;

			BoostPoint3D scaledlllpoint;
			BoostPoint3D scaledurrpoint;

			boost::geometry::strategy::transform::translate_transformer<double, 3, 3> translate(dx, dy, dz);
			bg::transform(boosturrpoint, scaledurrpoint, translate);
			boost::geometry::strategy::transform::translate_transformer<double, 3, 3> translate2(-dx, -dy, -dz);
			bg::transform(boostlllpoint, scaledlllpoint, translate2);
			
			auto qBox = bg::model::box < BoostPoint3D >(scaledlllpoint, scaledurrpoint);

			std::vector<Value> qResult;

			qResult.clear(); // no clue why, but avoids a random indexing issue that can occur
			h->getIndexPointer()->query(bgi::intersects(qBox), std::back_inserter(qResult));

			if (qResult.size() == 0) { continue; }

			for (size_t k = 0; k < qResult.size(); k++)
			{
				LookupValue lookup = h->getLookup(qResult[k].second);
				IfcSchema::IfcProduct* qProduct = std::get<0>(lookup);
				TopoDS_Shape shape;

				if (std::get<3>(lookup))
				{
					qProductList.emplace_back(std::make_tuple(qProduct, std::get<4>(lookup)));
				}
				else {
					qProductList.emplace_back(std::make_tuple(qProduct, h->getObjectShape(std::get<0>(lookup), false)));
				}
			}

			auto connectedObjects = checkConnection(spaceShape, currentSpace, qProductList);
			std::vector<IfcSchema::IfcRelSpaceBoundary*> boundaryList = makeSpaceBoundary(currentSpace, connectedObjects);

			for (size_t j = 0; j < boundaryList.size(); j++)
			{
				h->getSourceFile()->addEntity(boundaryList[j]);
			}

		}
		std::cout << counter << " of " << spaceListSize << std::endl;
	}
}

void helperCluster::setUseProxy(bool b) {
	for (size_t i = 0; i < helperList.size(); i++)
	{
		helperList[i]->setUseProxy(b);
	}
}

std::list<std::string> helperCluster::getObjectList() {
	
	std::list<std::string> typeListLong;

	for (size_t i = 0; i < size_; i++)
	{	
		std::list<std::string> typeList = helperList[i]->getObjectTypes();

		for (auto it = typeList.begin(); it != typeList.end(); ++it) {
			typeListLong.emplace_back(*it);
		}
	}

	typeListLong.sort();
	typeListLong.unique();

	return typeListLong;
}