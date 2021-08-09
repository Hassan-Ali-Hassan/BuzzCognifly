#include "../include/VoronoiDiagramGenerator.h"
#include "../include/Vector2.h"
#include "Epsilon.h"
#include <algorithm>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

void VoronoiDiagramGenerator::printBeachLine() {
	treeNode<BeachSection>* section = beachLine->getFirst(beachLine->getRoot());

	while (section) {
		cout << section->data.site->p << endl;
		section = section->next;
	}
	if(section) cout << section->data.site->p << endl;
	cout << endl << endl;
}

bool pointComparator(Point2* a, Point2* b) {
	double r = b->y - a->y;
	if (r < 0) return true;
	else if (r == 0) {
		if (b->x - a->x < 0) return true;
		else return false;
	}
	else return false;
}

Diagram* VoronoiDiagramGenerator::compute(std::vector<Point2>& sites, BoundingBox bbox) {
	siteEventQueue = new std::vector<Point2*>();
	boundingBox = bbox;

	for (size_t i = 0; i < sites.size(); ++i) {
		//sanitize sites by quantizing to integer multiple of epsilon
		sites[i].x = round(sites[i].x / EPSILON)*EPSILON;
		sites[i].y = round(sites[i].y / EPSILON)*EPSILON;

		siteEventQueue->push_back(&(sites[i]));
	}

	diagram = new Diagram();
	circleEventQueue = new CircleEventQueue();
	beachLine = new RBTree<BeachSection>();

	// Initialize site event queue
	std::sort(siteEventQueue->begin(), siteEventQueue->end(), pointComparator);

	// process queue
	Point2* site = siteEventQueue->empty() ? nullptr : siteEventQueue->back();
	if (!siteEventQueue->empty()) siteEventQueue->pop_back();
	treeNode<CircleEvent>* circle;

	// main loop
	for (;;) {
		// figure out whether to handle a site or circle event
		// for this we find out if there is a site event and if it is
		// 'earlier' than the circle event
		circle = circleEventQueue->firstEvent;

		// add beach section
		if (site && (!circle || site->y < circle->data.y || (site->y == circle->data.y && site->x < circle->data.x))) {
			// first create cell for new site
			Cell* cell = diagram->createCell(*site);
			// then create a beachsection for that site
			addBeachSection(&cell->site);

			site = siteEventQueue->empty() ? nullptr : siteEventQueue->back();
			if (!siteEventQueue->empty()) siteEventQueue->pop_back();
		}

		// remove beach section
		else if (circle)
			removeBeachSection(circle->data.beachSection);

		// all done, quit
		else
			break;
	}

	// wrapping-up:
	//   connect dangling edges to bounding box
	//   cut edges as per bounding box
	//   discard edges completely outside bounding box
	//   discard edges which are point-like
	diagram->clipEdges(boundingBox);

	//   add missing edges in order to close open cells
	diagram->closeCells(boundingBox);

	diagram->finalize();

	delete circleEventQueue;
	circleEventQueue = nullptr;

	delete siteEventQueue;
	siteEventQueue = nullptr;

	delete beachLine;
	beachLine = nullptr;

	return diagram;
}

bool halfEdgesCW(HalfEdge* e1, HalfEdge* e2) {
	return e1->angle < e2->angle;
}

Diagram* VoronoiDiagramGenerator::relax() {
	std::vector<Point2> sites;
	std::vector<Point2> verts;
	std::vector<Vector2> vectors;
	//replace each site with its cell's centroid:
	//    subdivide the cell into adjacent triangles
	//    find those triangles' centroids (by averaging corners) 
	//    and areas (by computing vector cross product magnitude)
	//    combine the triangles' centroids through weighted average
	//	  to get the whole cell's centroid
	for (Cell* c : diagram->cells) {
		size_t edgeCount = c->halfEdges.size();
		verts.resize(edgeCount);
		vectors.resize(edgeCount);

		for (size_t i = 0; i < edgeCount; ++i) {
			verts[i] = *c->halfEdges[i]->startPoint();
			vectors[i] = *c->halfEdges[i]->startPoint() - verts[0];
		}

		Point2 centroid(0.0, 0.0);
		double totalArea = 0.0;
		for (size_t i = 1; i < edgeCount-1; ++i) {
			double area = (vectors[i+1].x*vectors[i].y - vectors[i+1].y*vectors[i].x)/2;
			totalArea += area;
			centroid.x += area*(verts[0].x + verts[i].x + verts[i + 1].x) / 3;
			centroid.y += area*(verts[0].y + verts[i].y + verts[i + 1].y) / 3;
		}
		centroid.x /= totalArea;
		centroid.y /= totalArea;
		sites.push_back(centroid);
	}

	//then recompute the diagram using the cells' centroids
	compute(sites, boundingBox);

	return diagram;
}

void VoronoiDiagramGenerator::get_centroid() {
	std::vector<Point2> sites;
	std::vector<Point2> verts;
	std::vector<Vector2> vectors;
	//replace each site with its cell's centroid:
	//    subdivide the cell into adjacent triangles
	//    find those triangles' centroids (by averaging corners) 
	//    and areas (by computing vector cross product magnitude)
	//    combine the triangles' centroids through weighted average
	//	  to get the whole cell's centroid
	for (Cell* c : diagram->cells) {
		size_t edgeCount = c->halfEdges.size();
		verts.resize(edgeCount);
		vectors.resize(edgeCount);

		for (size_t i = 0; i < edgeCount; ++i) {
			verts[i] = *c->halfEdges[i]->startPoint();
			vectors[i] = *c->halfEdges[i]->startPoint() - verts[0];
		}

		Point2 centroid(0.0, 0.0);
		double totalArea = 0.0;
		for (size_t i = 1; i < edgeCount-1; ++i) {
			double area = (vectors[i+1].x*vectors[i].y - vectors[i+1].y*vectors[i].x)/2;
			totalArea += area;
			centroid.x += area*(verts[0].x + verts[i].x + verts[i + 1].x) / 3;
			centroid.y += area*(verts[0].y + verts[i].y + verts[i + 1].y) / 3;
		}
		centroid.x /= totalArea;
		centroid.y /= totalArea;
		// std::cout<<"the centroid is\t"<<centroid<<std::endl;
		c->cg = centroid;
	}
}

bool sitesOrdered(const Point2& s1, const Point2& s2) {
	if (s1.y < s2.y)
		return true;
	if (s1.y == s2.y && s1.x < s2.x)
		return true;

	return false;
}

void VoronoiDiagramGenerator::genRandomSites(std::vector<Point2>& sites, BoundingBox& bbox, unsigned int dimension, unsigned int numSites) {
	bbox = BoundingBox(0, dimension, dimension, 0);
	std::vector<Point2> tmpSites;

	tmpSites.reserve(numSites);
	sites.reserve(numSites);

	Point2 s;
	clock_t t = std::clock();
	// std::cout<<t<<std::endl;
	srand(t);
	// srand(50);
	for (unsigned int i = 0; i < numSites; ++i) {
		s.x = 1 + (rand() / (double)RAND_MAX)*(dimension - 2);
		s.y = 1 + (rand() / (double)RAND_MAX)*(dimension - 2);
		tmpSites.push_back(s);
	}

	//remove any duplicates that exist
	std::sort(tmpSites.begin(), tmpSites.end(), sitesOrdered);
	sites.push_back(tmpSites[0]);
	for (Point2& s : tmpSites) {
		if (s != sites.back()) sites.push_back(s);
	}
}

void VoronoiDiagramGenerator::genSites(std::vector<Point2>& sites, BoundingBox& bbox, unsigned int dimension, unsigned int numSites,float* pos) {
	bbox = BoundingBox(0, dimension, dimension, 0);
	// bbox = BoundingBox(-dimension, dimension, -dimension, dimension);
	std::vector<Point2> tmpSites;

	tmpSites.reserve(numSites);
	sites.reserve(numSites);

	Point2 s;
	
	for (unsigned int i = 0; i < numSites; ++i) {
		s.x = pos[i*2];
		s.y = pos[i*2+1];;
		tmpSites.push_back(s);
	}

	//remove any duplicates that exist
	std::sort(tmpSites.begin(), tmpSites.end(), sitesOrdered);
	sites.push_back(tmpSites[0]);
	for (Point2& s : tmpSites) {
		if (s != sites.back()) sites.push_back(s);
	}
}
void VoronoiDiagramGenerator::do_voronoi(int n)
{
	// std::cout<<"hey man  !!!"<<std::endl;
	Diagram* diagram = nullptr;
	std::vector<Point2>* sites;
	BoundingBox bbox;

	sites = new std::vector<Point2>();
	genRandomSites(*sites, bbox, 100, n);
	diagram = compute(*sites, bbox);
	get_centroid();
	diagram->printDiagram();

	delete diagram;
	delete sites;
}

void VoronoiDiagramGenerator::do_voronoi(float* a,int n)
{
	//this function takes as an input an array that contains 
	//the x and y coordinates for some points(x1,y1,x2,y2 and
	// so on)and does the whole voronoi sequence.
	Diagram* diagram = nullptr;
	std::vector<Point2>* sites;
	BoundingBox bbox;

	sites = new std::vector<Point2>();
	genSites(*sites, bbox, 100, n, a);
	diagram = compute(*sites, bbox);
	get_centroid();
	diagram->printDiagram();

	delete diagram;
	delete sites;
}

void VoronoiDiagramGenerator::do_voronoi(float* a,float* centers,int n)
{
	//this function takes as an input an array that contains 
	//the x and y coordinates for some points(x1,y1,x2,y2 and
	// so on)and does the whole voronoi sequence. The centroids
	// of the voronoi cells are stored in the vector "centers", 
	// which is passed as a pointer to an array to fill the array.
	Diagram* diagram = nullptr;
	std::vector<Point2>* sites;
	BoundingBox bbox;

	sites = new std::vector<Point2>();
	genSites(*sites, bbox, 100, n, a);
	diagram = compute(*sites, bbox);
	get_centroid();
	int i = 0;
	for (Cell* c : diagram->cells)
	{
		centers[2*i] = c->cg.x;
		centers[2*i+1] = c->cg.y;
		printf("centroid(%i):\t %f \t %f\n",i,centers[2*i],centers[2*i+1]);
		i += 1;
	}
	delete diagram;
	delete sites;
}

void VoronoiDiagramGenerator::do_voronoi(float* a,float* centers,float* myPos,float* myVCenter,int n,float dimension)
{
	//this function takes as an input an array that contains 
	//the x and y coordinates for some points(x1,y1,x2,y2 and
	// so on)and does the whole voronoi sequence. This function
	// returns all the centers of the voronoi cells, as well as
	// the specific center of the specific agent #i.
	Diagram* diagram = nullptr;
	std::vector<Point2>* sites;
	BoundingBox bbox;

	sites = new std::vector<Point2>();
	genSites(*sites, bbox, dimension, n, a);
	diagram = compute(*sites, bbox);
	get_centroid();
	int i = 0;
	for (Cell* c : diagram->cells)
	{
		centers[2*i] = c->cg.x;
		centers[2*i+1] = c->cg.y;
		// printf("centroid(%i):\t %f \t %f\n",i,centers[2*i],centers[2*i+1]);
		i += 1;
	}

	int counter = 0;
	for(int j = 0; j < n; j++) //now looping over all cells, and seeing which sites have smaller y coordinate than the current one, this will dictate which is the index of the current agent in the final list so we can return the right vornoi centroid
	{
		if(myPos[1]>a[2*j+1])
		{
			//printf("the counter for [%f,%f] is %i\n", myPos[2*counter],myPos[2*counter+1],counter);
			counter++;
		}
	}

	// i = 0;
	// float ddd = 0;
	// for (Cell* c : diagram->cells)
	// {
	// 	ddd = sqrt(  (myPos[0]-c->site.p.x)*(myPos[0]-c->site.p.x) + (myPos[1]-c->site.p.y)*(myPos[1]-c->site.p.y) );
	// 	printf("distance is  %f\n:",ddd);
	// 	if(ddd < 0.1) //which means we've found the right cell corresponding to local position
	// 	{
	// 		myVCenter[0] = c->cg.x;
	// 		myVCenter[1] = c->cg.y;

	// 		printf("pos(%i):\t %f \t %f\t centroid:\t %f\t %f\n",i,c->site.p.x-225,c->site.p.y-225,myVCenter[0]-225,myVCenter[1]-225);
	// 		i += 1;
	// 		break;
	// 	}
	// 	// printf("pos(%i):\t %f \t %f\t centroid:\t %f\t %f\n",i,c->site.p.x,c->site.p.y,c->cg.x,c->cg.y);
	// 	// printf("pos(%i):\t %f \t %f\t centroid:\t %f\t %f\n",i,c->site.p.x-225,c->site.p.y-225,c->cg.x-225,c->cg.y-225);
	// }

	// printf("the counter for [%f,%f] is %i\n", myPos[2*counter],myPos[2*counter+1],counter);
	myVCenter[0] = centers[2*counter];
	myVCenter[1] = centers[2*counter+1];
	delete diagram;
	delete sites;
}