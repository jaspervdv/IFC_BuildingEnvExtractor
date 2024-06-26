#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Lin.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
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


#ifndef FACEGRIDPAIR_FACEGRIDPAIR_H
#define FACEGRIDPAIR_FACEGRIDPAIR_H

class FaceGridPair {
private:
	TopoDS_Face face_;
	std::vector<EvaluationPoint*> pointGrid_;

	gp_Pnt lllPoint_;
	gp_Pnt urrPoint_;

	void populatePointGrid(
		const TopoDS_Face& theFace, 
		int vertCount,
		double res
	);

public:
	FaceGridPair(const TopoDS_Face& theFace, double res);
	FaceGridPair(const TopoDS_Face& theFace, std::vector<EvaluationPoint*>& pointGrid); //TODO: implement

	TopoDS_Face getFace() const { return face_; }
	void replaceFace(const TopoDS_Face& theFace) {face_ = theFace; }
	std::vector<EvaluationPoint*> getGrid() const { return pointGrid_; }
	gp_Pnt getLLLPoint() const { return lllPoint_; }
	gp_Pnt getURRPoint() const { return urrPoint_; }

	bool overlap(FaceGridPair otherFacePair);
};
#endif // FACEGRIDPAIR_FACEGRIDPAIR_H


#ifndef SURFACEGROUP_SURFACEGROUP_H
#define SURFACEGROUP_SURFACEGROUP_H

/// class that contains the top surfaces of a shape or collection of shapes
class SurfaceGroup {
private:
	// the face
	std::vector<FaceGridPair> theFaceCollection_; //TODO: make this function with complex faces

	// the flat face geometry LoD13
	TopoDS_Face theFlatFace_;

	// the flat face at ground level
	TopoDS_Face theProjectedFace_;

	// the wire the surrounds the surfaceGroup
	std::vector<TopoDS_Wire> wireList_;

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

	bool overlap(SurfaceGroup other);

	bool testIsVisable(FaceGridPair& evaluatedSurface, FaceGridPair& otherSurface);

public:
	explicit SurfaceGroup(const TopoDS_Shape& theShape);

	//const std::vector<TopoDS_Face>& getFaces() const { return theFaceCollection_; }
	const std::vector<FaceGridPair> getSurfaceCollection() const { return theFaceCollection_; }
	const TopoDS_Face& getFace(int indx) { return theFaceCollection_[indx].getFace(); }
	const TopoDS_Face& getFlatFace() const { return theFlatFace_; }
	const TopoDS_Face& getProjectedFace() const { return theProjectedFace_; }
	TopoDS_Face* getProjectedFacePtr() { return &theProjectedFace_; }

	std::vector<TopoDS_Wire> getWireList() { return wireList_; };

	const gp_Pnt getLLLPoint() const { return lllPoint_; }
	const gp_Pnt getURRPoint() const { return urrPoint_; }

	double getAvHeight() { return avHeight_; }
	double getTopHeight() { return topHeight_; }

	const std::vector<EvaluationPoint*>& getPointGrid(int indx) { return theFaceCollection_[indx].getGrid(); }
	bool isVisible() { return visibility_; }

	bool testIsVisable(const std::vector<SurfaceGroup>& otherSurfaces, bool preFilter = false);
	int getVertCount() { return vertCount_; }

	bool hasSummaryData();

	void setIsHidden() { visibility_ = false; }

	void createOutLine();
	void projectFace();

	void populateGrid(double distance);
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