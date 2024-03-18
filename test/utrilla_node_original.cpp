
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>

using namespace std;
// #include "acado_common.h"
// #include "acado_auxiliary_functions.h"

#include <stdio.h>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <common_msgs/Controls.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/console.h"
#include "utrilla_mpc/utrilla_wrapper.h"

using namespace utrilla_mpc;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1       /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */


double prediccion[NX*(N+1)];

double u_prediccion[NU*N];

bool acado_inicializado = false;
bool acado_preparado = false;

vector< double > k;
vector< double > s;


double interpolate( vector<double> &xData, vector<double> &yData, double x, bool extrapolate )
{
   int size = xData.size();

   int i = 0;                                                                  // find left end of interval for interpolation
   if ( x >= xData[size - 2] )                                                 // special case: beyond right end
   {
      i = size - 2;
   }
   else
   {
      while ( x > xData[i+1] ) i++;
   }
   double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
   if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
   {
      if ( x < xL ) yR = yL;
      if ( x > xR ) yL = yR;
   }

   double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

   return yL + dydx * ( x - xL );                                              // linear interpolation
}

bool readDataFromFile( const char* fileName, vector< double > & data )
{
	ifstream file( fileName );
	string line;

	if ( file.is_open() )
	{
        //cout << "bien" << endl;
		while( getline(file, line) )
		{
			istringstream linestream( line );
			double number;

			while( linestream >> number )
			{
				data.push_back( number );
			}
		}

		file.close();
	}
	else{
        //cout << "mal" << endl;
        return false;
    }
		

	return true;
}







/* A template for testing of the solver. */
void callback(const std_msgs::Float32MultiArray::ConstPtr& msg, ros::Publisher& pub){
	
	/* Some temporary variables. */
	int i;
	
	
	acado_timer t;
	acado_tic( &t );

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));

	//Initialize x0
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = msg->data[i];
	// if (acadoVariables.x0[3]<0.5){
	// 	acadoVariables.x0[3]=3;
	// }
	// if (acadoVariables.x0[6]==0){
	// 	acadoVariables.x0[6]=0.1;
	// }
	// cout << acadoVariables.x0[6] << endl;
	

	

	/* Initialize the solver. */
	if( !acado_inicializado ) {
		acado_initializeSolver();
		acado_inicializado = true;
	}

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = prediccion[ i ];
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = u_prediccion[ i ];

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;
	
	
	for (i=0; i<51;++i) acadoVariables.od[i]= interpolate(s,k,acadoVariables.x[i*NX],true);

	/* Prepare first step */
	if(!acado_preparado) {
		acado_preparationStep();
		acado_preparado = true;
	}

	/* Perform the feedback step. */
	acado_feedbackStep( );

	/* Apply the new control immediately to the process, first NU components. */
	common_msgs::Controls cmd;
	cmd.accelerator = acadoVariables.x[6];
	cmd.steering = (acadoVariables.x[7]*180)/3.14159265358979323846;
	printf("cmd: %f, delta: %f\n", cmd.accelerator, cmd.steering );
	pub.publish(cmd);

	real_t te = acado_toc( &t );
	printf("Elapsed time:   %f\n\n", te);
	

	/* Optional: shift the initialization (look at acado_common.h). */
	acado_shiftStates(2, 0, 0); 
	acado_shiftControls( 0 ); 

	for (i = 0; i < NX * (N + 1); ++i) prediccion[i] = acadoVariables.x[i];
	for (i = 0; i < NU * N; ++i) u_prediccion[i] = acadoVariables.u[i];

	/* Prepare for the next step. */
	acado_preparationStep();
}

int main(int argc, char **argv){

	for (int i = 0; i < NX*(N+1); ++i) prediccion[i] = 0.01;
	for (int i = 0; i < NU*N; ++i) u_prediccion[i] = 0.0;

	if (readDataFromFile("/home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/test/k_smooth.txt", k) == false)
	{
		cout << "Cannot read measurements 1" << endl;
		// return EXIT_FAILURE;
	}
	if (readDataFromFile("/home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/test/s.txt", s) == false)
	{
		cout << "Cannot read measurements 2" << endl;
		// return EXIT_FAILURE;
	}

    ros::init(argc,argv,"utrilla_mpc");
    ros::NodeHandle n;

    
    ros::Publisher pub = n.advertise<common_msgs::Controls>("controls_mpc", 1000);
    ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("/curvilineas/x", 1000,
         boost::bind(callback, _1, boost::ref(pub)));
   
    
    ros::spin();

    return 0;
}