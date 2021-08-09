#ifndef _VORONOI_DIAGRAM_GENERATOR_H_
#define _VORONOI_DIAGRAM_GENERATOR_H_

#include "../src/RBTree.h"
#include "../src/CircleEventQueue.h"
#include "../src/BeachLine.h"
#include "Diagram.h"
#include <vector>
// #include <chrono>
#include <ctime>
// #include <iostream>

struct BoundingBox {
	double xL;
	double xR;
	double yB;
	double yT;

	BoundingBox() {};
	BoundingBox(double xmin, double xmax, double ymin, double ymax) :
		xL(xmin), xR(xmax), yB(ymin), yT(ymax) {};
};

class VoronoiDiagramGenerator {
public:
	VoronoiDiagramGenerator() : circleEventQueue(nullptr), siteEventQueue(nullptr), beachLine(nullptr) {};
	~VoronoiDiagramGenerator() {};

	Diagram* compute(std::vector<Point2>& sites, BoundingBox bbox);
	void get_centroid();
	Diagram* relax();
	void do_voronoi(int n);
	void do_voronoi(float* a, int n);
	void do_voronoi(float* a,float* centers,int n);
	void do_voronoi(float* a,float* centers,float* myPos,float* myVCenter,int n, float d = 500);
private:
	Diagram* diagram;
	CircleEventQueue* circleEventQueue;
	std::vector<Point2*>* siteEventQueue;
	BoundingBox	boundingBox;

	void printBeachLine();

	//BeachLine
	RBTree<BeachSection>* beachLine;
	treeNode<BeachSection>* addBeachSection(Site* site);
	inline void detachBeachSection(treeNode<BeachSection>* section);
	void removeBeachSection(treeNode<BeachSection>* section);
	double leftBreakpoint(treeNode<BeachSection>* section, double directrix);
	double rightBreakpoint(treeNode<BeachSection>* section, double directrix);

	//some specific example implementaiton functions
	void genRandomSites(std::vector<Point2>& sites, BoundingBox& bbox, unsigned int dimension, unsigned int numSites);
	void genSites(std::vector<Point2>& sites, BoundingBox& bbox, unsigned int dimension, unsigned int numSites,float* pos);
};

inline void VoronoiDiagramGenerator::detachBeachSection(treeNode<BeachSection>* section) {
	circleEventQueue->removeCircleEvent(section);
	beachLine->removeNode(section);
}

#endif