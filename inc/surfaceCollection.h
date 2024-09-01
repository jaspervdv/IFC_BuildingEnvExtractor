#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Lin.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Wire.hxx>
#include <memory>


#ifndef EVALUATIONPOINT_EVALUATIONPOINT_H
#define EVALUATIONPOINT_EVALUATIONPOINT_H

class EvaluationPoint {
private:
	gp_Pnt thePoint_;

	TopoDS_Edge evalEdge_;
	gp_Lin evalLin_;

	bool isVisible_ = true;

public:
	EvaluationPoint(const gp_Pnt& p);
	bool isVisible() { return isVisible_; }
	void setInvisible() { isVisible_ = false; }

	const gp_Pnt getPoint() { return thePoint_; }
	const TopoDS_Edge& getEvalEdge() { return evalEdge_; }
	const gp_Lin& getEvalLine() { return evalLin_; }
};
#endif // EVALUATIONPOINT_EVALUATIONPOINT_H


#ifndef SURFACEGROUP_SURFACEGROUP_H
#define SURFACEGROUP_SURFACEGROUP_H

// contains the data of a single TopoDS_Face with its matching grid
class SurfaceGridPair {
private: 
	TopoDS_Face theFace_;

	// bounding box 3D data
	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// information about the face
	double topHeight_;
	double avHeight_;
	int vertCount_ = 0;

	// usability information
	bool visibility_ = true;
	bool isSmall_ = false;

	std::vector<std::shared_ptr<EvaluationPoint>> pointGrid_;
	bool overlap(SurfaceGridPair other);

public:
	SurfaceGridPair(const TopoDS_Face& theFace);

	const TopoDS_Face getFace() const { return theFace_; }

	const gp_Pnt getLLLPoint() { return lllPoint_; }
	const gp_Pnt getURRPoint() { return urrPoint_; }

	double getAvHeight() { return avHeight_; }
	double getTopHeight() { return topHeight_; }

	int getVertCount() { return vertCount_; }

	bool isVisible() const { return visibility_; }
	void setIsHidden() { visibility_ = false; }

	void populateGrid(double distance);

	bool testIsVisable(const std::vector<SurfaceGridPair>& otherSurfaces, bool preFilter = false);
};

// collection of the visible roof surfaces per IfcObject or IfcObject set
class ROSCollection {
private:
	// the faces
	std::vector<SurfaceGridPair> theFaceCollection_;

	// the wire surrounding the faceCollection
	std::vector<TopoDS_Wire> theWireCollection_;

	// the flat face geometry of the complete face collection for LoD13
	TopoDS_Face theFlatFace_;

	// bounding box 3D data
	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	// usability information
	bool visibility_ = true;
	bool isSmall_ = false;

	bool overlap(ROSCollection other);

public:
	ROSCollection(std::vector< SurfaceGridPair> theGridPairList);

	const std::vector< SurfaceGridPair> getFaceGridPairs() { return theFaceCollection_; }
	const std::vector<TopoDS_Wire> getWires() { return theWireCollection_; }

	const std::vector<TopoDS_Face> getFaces();
	const TopoDS_Face& getFlatFace() const { return theFlatFace_; }

	const TopoDS_Face getProjectedFace() const;

	const gp_Pnt getLLLPoint() { return lllPoint_; }
	const gp_Pnt getURRPoint() { return urrPoint_; }

	bool isVisible() { return visibility_; }
	bool testIsVisable(const std::vector<ROSCollection>& otherSurfaces, bool preFilter = false);

	void setIsHidden() { visibility_ = false; }
	void projectFace();

	void update();
};
#endif // SURFACEGROUP_SURFACEGROUP_H


#ifndef EDGE_EDGE_H
#define EDGE_EDGE_H

class Edge {
private:
	TopoDS_Edge theEdge_;
	gp_Pnt startPoint_;
	gp_Pnt endPoint_;

	bool hasEdgeEval_ = false;
	bool isOuter_ = false;

public:
	explicit Edge(const TopoDS_Edge& edge);

	const TopoDS_Edge& getEdge() const { return theEdge_; }

	TopoDS_Edge* getEdgePtr() { return &theEdge_; }

	gp_Pnt getStart(bool projected) const;

	gp_Pnt getEnd(bool projected) const;
};
#endif // EDGE_EDGE_H