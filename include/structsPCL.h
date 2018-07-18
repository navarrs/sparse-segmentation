#ifndef STRUCTSPCL_H
#define STRUCTSPCL_H

#define THRESH_ERROR  -3.2
#define GROUND_LABEL 4
#define PI 3.14159265

struct point_XYZIRL { 
	float x;  // Coordinate x 
	float y;  // Coordinate y
	float z;  // Coordinate z
	float i;  // intensity
	float r;  // range
	float l;  // label
	float n;    // Line number
}; 

#endif
