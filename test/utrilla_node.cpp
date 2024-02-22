
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
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/console.h"
#include "utrilla_mpc/utrilla_wrapper.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   800       /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
// ACADOvariables acadoVariables;
// ACADOworkspace acadoWorkspace;


// void callback(const sensor_msgs::PointCloud2::ConstPtr& msg, 
//             ros::Publisher& pub){

//     std_msgs::String s;
//     s.data = "hola mundo";
//     cout << "a"<< endl;
//     pub.publish(s);
// }



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
        cout << "bien" << endl;
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
        cout << "mal" << endl;
        return false;
    }
		

	return true;
}







/* A template for testing of the solver. */
void callback(const std_msgs::Float32MultiArray::ConstPtr& msg, ros::Publisher& pub){

	cout << msg->data[0] << endl;

	/* Some temporary variables. */
	int    i, iter;
	utrilla_mpc::acado_timer t;

	// Reset all solver memory
	memset(&utrilla_mpc::acadoWorkspace, 0, sizeof( utrilla_mpc::acadoWorkspace ));
	memset(&utrilla_mpc::acadoVariables, 0, sizeof( utrilla_mpc::acadoVariables ));

	vector< double > k;
	vector< double > s;
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

	/* Initialize the solver. */
	utrilla_mpc::acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  utrilla_mpc::acadoVariables.x[ i ] = 0.01;
	for (i = 0; i < NU * N; ++i)  utrilla_mpc::acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  utrilla_mpc::acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  utrilla_mpc::acadoVariables.yN[ i ] = 0.0;
	
	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) utrilla_mpc::acadoVariables.x0[ i ] = 0.0;
#endif
	utrilla_mpc::acadoVariables.x0[0]=10;
	utrilla_mpc::acadoVariables.x0[3]=5;
	utrilla_mpc::acadoVariables.x0[6]=0.1;
	// if( VERBOSE ) acado_printHeader();
	
	for (i=0; i<51;++i) utrilla_mpc::acadoVariables.od[i]= interpolate(s,k,utrilla_mpc::acadoVariables.x[i*NX],true);

	/* Prepare first step */
	utrilla_mpc::acado_preparationStep();

	/* Get the time before start of the loop. */
	utrilla_mpc::acado_tic( &t );

	double sim_states[NUM_STEPS*NX];

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{		
		/* Perform the feedback step. */
		utrilla_mpc::acado_feedbackStep( );
		
		/* Apply the new control immediately to the process, first NU components. */

		if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, utrilla_mpc::acado_getKKT() );

		/* Optional: shift the initialization (look at acado_common.h). */
        utrilla_mpc::acado_shiftStates(2, 0, 0); 
		utrilla_mpc::acado_shiftControls( 0 ); 

		/* Prepare for the next step. */
		utrilla_mpc::acado_preparationStep();

		for (i = 0; i < NX; ++i) utrilla_mpc::acadoVariables.x0[ i ] = utrilla_mpc::acadoVariables.x[ i ];
		for (i=0; i<51;++i) utrilla_mpc::acadoVariables.od[i]= interpolate(s,k,utrilla_mpc::acadoVariables.x[i*NX],true);
		//cout << acadoVariables.od[0] << " " << acadoVariables.x[NX] << "\n";
		for (i = 0; i < NX; ++i) sim_states[i+iter*NX]= utrilla_mpc::acadoVariables.x0[ i ];
	}
	/* Read the elapsed time. */
	utrilla_mpc::real_t te = utrilla_mpc::acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	//acado_printDifferentialVariables();
	//acado_printControlVariables();

	for(i=0; i<NUM_STEPS*NX; ++i) cout << sim_states[i] << " ";
	cout << "\n";
    // return 0;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"utrilla_mpc");
    ros::NodeHandle n;

    
    ros::Publisher pub = n.advertise<std_msgs::String>("/string_topic", 1000);
    ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("/curvilineas/x", 1000,
         boost::bind(callback, _1, boost::ref(pub)));
   
    
    ros::spin();

    return 0;
}