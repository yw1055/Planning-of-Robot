/*
This code was developed by Tansel Uras (turas@usc.edu) at USC.
The code is hosted at 'http://idm-lab.org/anyangle'.
If you use this code in your research, please  cite our SoCS paper:

T. Uras and S. Koenig,  2015. An Empirical Comparison of Any-Angle Path-Planning Algorithms. In: Proceedings of the 8th Annual Symposium on Combinatorial
Search. Code available at: http://idm-lab.org/anyangle

Bibtex:
@inproceedings{uras:15,
  author = "T. Uras and S. Koenig",
  title = "An Empirical Comparison of Any-Angle Path-Planning Algorithms",
  booktitle = {Proceedings of the 8th Annual Symposium on Combinatorial Search},
  year = "2015",
  note = "Code available at: http://idm-lab.org/anyangle",
}
*/

#ifndef Batch_Informed_Tree_H
#define Batch_Informed_Tree_H

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <GL/glut.h>	
#include <GLFW/glfw3.h>
#include <cstdio>
#include <fstream>

using namespace std;

// Vertices are placed at the corners of cells.
// Given a map of size n x m (n x m cells, n+1 x m+1 corners):
// Top left cell is (0,0), top left corner is (0,0), and corner (x,y) is located at the top left corner of cell (x,y).
//
// We add a frame of obstacles around the map but don't add any more corners,
// resulting in n+2 x m+2 cells, n+1 x m+1 corners
//
// Now, corner x,y is located at the bottom right corner of cell x,y, and every corner is surrounded by cells
// so we don't have to do any out-of-map-bounds checks
//
// 'cells' store the blockage information of cells (only used for LOS checks after initialization)
// 'cornerIds' assign an id for each corner that has at least one neighboring cell without obstacles
// 'cornerLocs' is the reverse mapping of cornerIds: for each cornerId, it stores its location


struct states{
  float x, y, theta;
};
struct points{
  float x, y;
};

class Batch_Informed_Tree_Algorithms {

public:

#ifdef ANY_ANGLE_RUNNING_IN_HOG
    //Batch_Informed_Tree_Algorithms(const vector<int >);                // Constructs the class using a map given as a MapEnvironment in HOG
#endif
	//Batch_Informed_Tree_Algorithms(const std::vector<bool> &bits, const int width, const int height);
	//Batch_Informed_Tree_Algorithms();
	//~Batch_Informed_Tree_Algorithms();

	void FindDubinsPath(const states from, const states to);
	void FindPointsPaht(const points from, const points to);
	float EuclideanDistance(float x1,float y1, float x2, float y2)  {
		  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));}
private:
   int map_x,map_y;
   float start_x, start_y, end_x, end_y, infinity;
};

#endif
