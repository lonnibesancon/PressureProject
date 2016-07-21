#ifndef INTERACTIONMODE
#define INTERACTIONMODE


// Data manipulation
#define dataTangible  			1 
#define dataTouch  				2 
//#define dataHybrid  			3 

//Plane manipulation
#define planeTouch  			11 
#define planeTangible  			12 
//#define planeHybrid  			13 


// Plane + data
#define dataPlaneTouch   		21 
#define dataPlaneTangible  		22 
#define dataPlaneHybrid 		23
#define dataTouchTangible 		24


//Seeding point interaction
#define seedPointTangible  		31 
#define seedPointTouch			32
#define seedPointHybrid			33


#define ftle 		1
#define ironProt	2
#define head		3
#define velocity	4


#define RATE_CONTROL		1
#define RATE_CONTROL_SIMPLE	11
#define SPEED_CONTROL		2
#define PRESSURE_CONTROL	3
#define SLIDER_CONTROL		4
#define NO_CONTROL			0

#define MAXPRECISION		3.0
#define MINPRECISION		0.5

#define MINEUCLIDEAN		0.005
#define MAXEUCLIDEAN		10.0
#define MINEUCLIDEANRATE	0.05
#define MAXEUCLIDEANRATE	30

#define MINANGULAR			0.00005
#define MAXANGULAR			0.15
#define MINANGULARRATE		0.5
#define MAXANGULARRATE		5.0


#define thresholdRST  300.0 


#endif