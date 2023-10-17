#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Lin.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Vec.hxx>


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

class SurfaceGroup {
private:
	TopoDS_Face theFace_;
	TopoDS_Face theFlatFace_;
	TopoDS_Face theProjectedFace_;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;
	gp_Pnt2d llPoint_;
	gp_Pnt2d urPoint_;

	double topHeight_;
	double avHeight_;

	bool visibility_ = true;
	bool isSmall_ = false;
	int vertCount_ = 0;

	std::vector<EvaluationPoint*> pointGrid_;
	bool overlap(SurfaceGroup* other);

public:
	SurfaceGroup(const TopoDS_Face& aFace);

	const TopoDS_Face& getFace() { return theFace_; }
	const TopoDS_Face& getFlatFace() { return theFlatFace_; }
	const TopoDS_Face& getProjectedFace() { return theProjectedFace_; }
	TopoDS_Face* getProjectedFacePtr() { return &theProjectedFace_; }

	const gp_Pnt getLLLPoint() { return lllPoint_; }
	const gp_Pnt getURRPoint() { return urrPoint_; }

	const gp_Pnt2d getLLPoint() { return llPoint_; }
	const gp_Pnt2d getURPoint() { return urPoint_; }

	double getAvHeight() { return avHeight_; }
	double getTopHeight() { return topHeight_; }

	std::vector<EvaluationPoint*>& getPointGrid() { return pointGrid_; }
	bool isVisible() { return visibility_; }
	bool testIsVisable(const std::vector<SurfaceGroup*>& otherSurfaces, bool preFilter = false);
	int getVertCount() { return vertCount_; }

	void setIsHidden() { visibility_ = false; }
	void projectFace();

	void populateGrid(double distance);
};
#endif // SURFACEGROUP_SURFACEGROUP_H


#ifndef EDGE_EDGE_H
#define EDGE_EDGE_H

class Edge {
private:
	TopoDS_Edge* theEdge_;
	gp_Pnt startPoint_;
	gp_Pnt endPoint_;

	bool hasEdgeEval_ = false;
	bool isOuter_ = false;

public:
	explicit Edge(const TopoDS_Edge& edge);

	TopoDS_Edge* getEdge() { return theEdge_; }

	gp_Pnt getStart() { return startPoint_; }

	gp_Pnt getEnd() { return endPoint_; }

	gp_Pnt getProjectedStart() { return gp_Pnt(startPoint_.X(), startPoint_.Y(), 0); }

	gp_Pnt getProjectedEnd() { return gp_Pnt(endPoint_.X(), endPoint_.Y(), 0); }
};
#endif // EDGE_EDGE_H