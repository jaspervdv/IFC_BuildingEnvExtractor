#include "helper.h"

#include <memory>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

#include <ifcparse/IfcFile.h>
#include <ifcgeom/IfcGeom.h>
#include <ifcgeom/IfcGeomRepresentation.h>
#include <ifcgeom_schema_agnostic/kernel.h>

#include <tuple>

#ifndef FLOORPROCESSOR_HELPER_H
#define FLOORPROCESSOR_HELPER_H

class floorProcessor {

private:

	/// collects an edge by begin and enpoint and the distance between the two
	struct DistancePair
	{
		gp_Pnt p1;
		gp_Pnt p2;
		double distance;
	};

	struct FloorStruct
	{
		// stored data		
		bool isFlat_;
		bool isSmall_;
		TopoDS_Face face_;
		double elevation_;
		double topElevation_;

		// check if data is stored
		bool hasFlatness = false;
		bool hasSmallness = false;
		bool hasFace = false;
		bool hasElevation = false;
		bool hasGroup = false;

		explicit FloorStruct(TopoDS_Face face, double area = 0);
	};

	struct FloorGroupStruct {
		bool assigned_ = false;
		bool isFlat_;
		int mergeNum_ = -1;
		FloorGroupStruct* merger_ = nullptr;
		double topElevation_ = -9999;
		double elevation_ = 9999;

		std::vector<FloorStruct*> floors_;

		explicit FloorGroupStruct() {};
		explicit FloorGroupStruct(FloorStruct* floor);

		void markMerger(FloorStruct* targetFloor, FloorStruct* mergingFloor);

		// Add a floor to the stuct and update all the varables
		void addFloor(FloorStruct* floor);
		
		// Merge a group into the struct and update all the varables 
		void mergeGroup(FloorGroupStruct* group);

	};

	static double getMedian(std::vector<double> l);
	
	// returns a vector filled with the top faces of the present floorslab objects
	static std::vector<TopoDS_Shape> getSlabShapes(helper* data);
	static std::vector<TopoDS_Face> getSlabFaces(helper* data);
	static std::vector<TopoDS_Face> getSlabFaces(std::vector <TopoDS_Shape> shapes);

	static TopoDS_Face getTopFace(TopoDS_Shape shape);

	// returns a vector of the areas of the faces
	static std::vector<double> getFaceAreas(std::vector<TopoDS_Face> faces);

	static void updateFloorGroup(std::vector<FloorGroupStruct>* floorGroups);

public:

	static std::vector<double> getStoreyElevations(helper* data);
	static std::vector<double> getStoreyElevations(std::vector<helper*> data);

	static std::vector<double> computeFloorElevations(helper* data);
	static std::vector<double> computeFloorElevations(std::vector<helper*> data);

	static bool compareElevations(std::vector<double> elevations, std::vector<double> floors);

	static void processStoreys(std::vector<helper*> data, std::vector<double> elevations, bool useOriginal);

	// removes all the storey data from the file
	static void cleanStoreys(helper* data);

	// adds new storeys based on the inputted vector elevations
	static void createStoreys(helper* data, std::vector<double> floorStoreys);

	static void sortObjects(helper* data);
	static void sortObjects(helper* data, IfcSchema::IfcBuildingStorey::list::ptr storeys);
	static void sortObjects(helper* data, IfcSchema::IfcProduct::list::ptr products);

	// TODO make private
	static void printLevels(std::vector<double> levels);
};

#endif // FLOORPROCESSOR_HELPER_H