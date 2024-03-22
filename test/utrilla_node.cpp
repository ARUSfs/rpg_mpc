
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
#include <common_msgs/Trajectory.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
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
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */


double prediccion[NX*(N+1)];

double u_prediccion[NU*N];

bool acado_inicializado = false;
bool acado_preparado = false;
bool primera_iter=true;

double i_mu;

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
void callback(const std_msgs::Float32MultiArray::ConstPtr& msg, ros::Publisher& pub, ros::Publisher& pub_pred, ros::Publisher& pub_pred_v){
	
	/* Some temporary variables. */
	int i;

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
	memset(&acadoVariables, 0, sizeof( acadoVariables ));

	//Initialize x0
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = msg->data[i];
	// for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.0;
	// acadoVariables.x0[1]=prediccion[1];
	// acadoVariables.x0[3]=prediccion[3];
	// acadoVariables.x0[6]=prediccion[6];
	// acadoVariables.x0[7]=prediccion[7];
	double v=acadoVariables.x0[3];
	double mu0=acadoVariables.x0[1];
	cout << "v_diff:  " << prediccion[3]-v << " |  mu_diff:  " << prediccion[1]-mu0 << "\n";
	cout << "v:   " << v << "\n\n";
	// cout << "v_0: " << acadoVariables.x0[3] << "   mu0: " << acadoVariables.x0[1] << "  s0: " << acadoVariables.x0[0] <<"\n";


	
	// i_mu=i_mu+0.2*acadoVariables.x0[1];
	// if (i_mu*i_mu > 10) i_mu=0;

	
	if (acadoVariables.x0[3]<2.0){
		common_msgs::Controls cmd;
		cmd.accelerator = 0.2;
		cmd.steering = 0.0;//-0.2*acadoVariables.x0[1]-0.2*i_mu;
		printf("cmd: %f, delta: %f\n", cmd.accelerator, cmd.steering );
		cout << "v_real " << acadoVariables.x0[3] << "\n";
		cout << "i_mu " << i_mu << "\n";
		pub.publish(cmd);
		primera_iter=true;
	}
	else{
		/* Initialize the solver. */
	if( !acado_inicializado ) {
		acado_initializeSolver();
		acado_inicializado = true;
	}
	// acado_initializeSolver();
	if( primera_iter ) {
		acadoVariables.x0[5] = 0.0;
		for (i = 0; i < (N+1); ++i) {
			for (int j=0; j<NX;++j) prediccion[i*NX +j]=acadoVariables.x0[j];
		}
		primera_iter = false;
	}
	// cout << "Prediccion \n";
	// for(i=0; i<N+1;++i) {
	// 	for (int j=0; j<NX;++j) cout << prediccion[i*NX+j] << " ";
	// 	cout << "\n";
	// }
	// acadoVariables.x0[0]=prediccion[0];
	// acadoVariables.x0[1]=prediccion[1];
	// acadoVariables.x0[2]=prediccion[2];
	// acadoVariables.x0[3]=prediccion[3];
	// acadoVariables.x0[4]=prediccion[4];
	// acadoVariables.x0[5]=prediccion[5];
	// acadoVariables.x0[6]=prediccion[6];
	// acadoVariables.x0[7]=prediccion[7];

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = prediccion[ i ];
	// for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.01;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = u_prediccion[ i ];
	// for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;
	
	
	for (i=0; i<NX+1;++i) acadoVariables.od[i]= interpolate(s,k,acadoVariables.x[i*NX],true);
	// for (i=0; i<NX+1;++i) acadoVariables.od[i]= 0.0;
	// cout << "k: " << acadoVariables.od[0] << " s: " << acadoVariables.x0[0] << "\n";
	// cout << "v_0: " << acadoVariables.x0[3] << "   mu0: " << acadoVariables.x0[1] <<"\n";

	/* Prepare first step */
	// if(!acado_preparado) {
	// 	acado_preparationStep();
	// 	acado_preparado = true;
	// }
	acado_preparationStep();

	/* Perform the feedback step. */
	acado_feedbackStep( );

	for(i=1; i<1; ++i){
	acado_preparationStep();
	acado_feedbackStep();
	}
	if( VERBOSE ) printf("\tKKT Tolerance = %.3e\n\n", acado_getKKT() );

	// acado_printDifferentialVariables();
	/* Optional: shift the initialization (look at acado_common.h). */
	acado_shiftStates(2, 0, 0); 
	acado_shiftControls( 0 ); 

	/* Apply the new control immediately to the process, first NU components. */
	common_msgs::Controls cmd;
	cmd.accelerator = acadoVariables.x[4];
	// if (v>8.5) cmd.accelerator=0.001;
	// double delta = 0.7*acadoVariables.x[5]/acadoVariables.x0[3]*1.53+0.3*acadoVariables.x[7];
	double delta = acadoVariables.x[5];
	cmd.steering = 180*delta/3.14159265358979323846;

	if (cmd.accelerator*cmd.accelerator < 1) {
		printf("cmd: %f, delta: %f\n", cmd.accelerator, cmd.steering );
		// cout << "v_pred" << acadoVariables.x[3] << "   mu_pred: " << acadoVariables.x[1] << "\n";
		// cout << "s_pred: " << acadoVariables.x[0] << "\n";
		pub.publish(cmd);	
	}
	else{
		cmd.accelerator = 0.0;
		cmd.steering = 0.0; 
		pub.publish(cmd);
	}
	for (i = 0; i < NX * (N + 1); ++i) prediccion[i] = acadoVariables.x[i];
	for (i = 0; i < NU * N; ++i) u_prediccion[i] = acadoVariables.u[i];

	std_msgs::Float32MultiArray pred;
	std_msgs::Float32 pred_v;
	pred_v.data=prediccion[3];
	pred.data.resize(2*N+2);
	for (i=0; i<N+1; ++i){
		pred.data.push_back(prediccion[i*NX]);
		pred.data.push_back(prediccion[i*NX+1]);
	}
	pub_pred.publish(pred);
	pub_pred_v.publish(pred_v);

	// acado_printDifferentialVariables();
	// acado_printControlVariables();
	/* Prepare for the next step. */
	acado_preparationStep();
	cout << "---\n";
	}


	
}

int main(int argc, char **argv){

	for (int i = 0; i < NX*(N+1); ++i) prediccion[i] = 0.0;
	for (int i = 0; i < N+1; ++i) {
		prediccion[i*NX+3] = 2;
		prediccion[i*NX+4] = 0.2;
		prediccion[i*NX]= 3;
	}
	for (int i = 0; i < NU*N; ++i) u_prediccion[i] = 0.0;

	// i_mu=0;

	if (readDataFromFile("/home/juv/workspaces/mpc_ws/src/DRIVERLESS/src/control/rpg_mpc/test/k_new.txt", k) == false)
	{
		cout << "Cannot read measurements 1" << endl;
		// return EXIT_FAILURE;
	}
	if (readDataFromFile("/home/juv/workspaces/mpc_ws/src/DRIVERLESS/src/control/rpg_mpc/test/s_new.txt", s) == false)
	{
		cout << "Cannot read measurements 2" << endl;
		// return EXIT_FAILURE;
	}

    ros::init(argc,argv,"utrilla_mpc");
    ros::NodeHandle n;

    
    ros::Publisher pub = n.advertise<common_msgs::Controls>("controls_mpc", 1);
	ros::Publisher pub_pred = n.advertise<std_msgs::Float32MultiArray>("pred_ruta",1);
	ros::Publisher pub_pred_v = n.advertise<std_msgs::Float32>("pred_v",1);

    ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("/curvilineas/x", 1000,
         boost::bind(callback, _1, boost::ref(pub), boost::ref(pub_pred), boost::ref(pub_pred_v)));
	    
    ros::spin();

    return 0;
}
