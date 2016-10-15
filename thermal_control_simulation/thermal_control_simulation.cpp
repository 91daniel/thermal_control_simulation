// thermal_control_simulation.cpp : Defines the entry point for the console application.
//
#include <iostream>
#include <chrono>
#include <fstream>
#include "libraries/thermal_control/HeatModel.h"
/* Do not create default /logs folder.
!Need another specified folder in the config file instead! */
#ifndef ELPP_NO_DEFAULT_LOG_FILE
#define ELPP_NO_DEFAULT_LOG_FILE
#endif
#include "libraries/logging/easylogging++.h"
INITIALIZE_EASYLOGGINGPP

using namespace std;


int main() {
	// Configure Loggers
	el::Configurations conf_defaultLogger("libraries/logging/infoLogger.conf");
	el::Configurations conf_dataLogger("libraries/logging/dataLogger.conf");
	el::Loggers::reconfigureLogger("default", conf_defaultLogger); // Default logger
	el::Loggers::reconfigureLogger("data", conf_dataLogger); // Logging data to file

	CLOG(INFO, "default") << "Start thermal control simulation";

	heatModel* heatModeler = new heatModel();
	heatModeler->setUpHeatModel(15.0);

	double force[133] = { 25.127
		,25.184
		,24.636
		,24.673
		,24.227
		,24.365
		,24.278
		,23.936
		,23.398
		,23.52
		,22.633
		,22.365
		,22.298
		,22.258
		,22.255
		,22.715
		,22.716
		,22.661
		,22.715
		,24.114
		,24.77
		,26.483
		,28.315
		,30.21
		,31.909
		,34.319
		,38.775
		,42.973
		,44.386
		,37.692
		,26.061
		,23.189
		,13.329
		,6.558
		,3.932
		,2.784
		,2.674
		,1.934
		, 1.81
		,1.689
		,3.794
		,11.022
		,14.526
		,16.728
		,18.289
		,14.45
		,10.089
		,10.223
		,9.768
		,7.162
		, 4.92
		,4.023
		,1.999
		,1.848
		,1.926
		,1.869
		,1.833
		,  1.8
		,1.276
		,1.307
		,3.435
		,9.547
		,12.964
		,14.644
		,15.265
		,15.129
		,12.192
		,10.035
		, 9.36
		,7.407
		,6.034
		,4.473
		,4.374
		,3.757
		,1.736
		,1.732
		,1.793
		,1.778
		, 1.77
		,1.729
		,4.637
		,10.084
		,12.148
		,12.808
		,12.68
		,11.63
		,9.606
		,8.315
		,6.844
		,0.049
		,0.038
		,0.033
		,0.033
		,0.033
		,0.033
		,0.033
		,0.032
		,0.031
		, 0.03
		,0.029
		,0.029
		,0.029
		,0.029
		,0.028
		,0.028
		,0.027
		,0.025
		,0.024
		,0.022
		,0.021
		, 0.02
		,0.019
		,0.018
		,0.018
		,0.017
		,0.017
		,0.016
		,0.016
		,0.016
		,0.016
		,0.017
		,0.017
		,0.017
		,0.018
		,0.019
		, 0.02
		, 0.02 };

	double torque[133] = { 0.012,
		0.012,
		0.012,
		0.012,
		0.012,
		0.012,
		0.012,
		0.012,
		0.012,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.013,
		0.014,
		0.014,
		0.014,
		0.014,
		0.02,
		0.02,
		0.021,
		0.022,
		0.022,
		0.027,
		0.026,
		0.025,
		0.025,
		0.018,
		0.017,
		0.016,
		0.015,
		0.014,
		0.013,
		0.013,
		0.012,
		0.008,
		0.008,
		0.008,
		0.016,
		0.011,
		0.013,
		0.014,
		0.015,
		0.016,
		0.016,
		0.016,
		0.016,
		0.015,
		0.015,
		0.015,
		0.015,
		0.01,
		0.008,
		0.013,
		0.008,
		0.008,
		0.008,
		0.008,
		0.009,
		0.017,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.015,
		0.014,
		0.014,
		0.014,
		0.013,
		0.009,
		0.009,
		0.009,
		0.009,
		0.011,
		0.012,
		0.013,
		0.014,
		0.019,
		0.007,
		0.005,
		0.005,
		0.005,
		0.005,
		0.004,
		0.004,
		0.004,
		0.004,
		0.004,
		0.004,
		0.004,
		0.003,
		0.003,
		0.003,
		0.003,
		0.003,
		0.003,
		0.002,
		0.002,
		0.002,
		0.002,
		0.002,
		0.002,
		0.002,
		0.002,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001,
		0.001 };

	/*	double force[133] = {14.026,
	13.945,
	13.866,
	13.864,
	13.484,
	13.529,
	13.529,
	13.506,
	13.503,
	13.926,
	13.374,
	13.317,
	13.379,
	13.745,
	13.256,
	13.341,
	13.335,
	13.323,
	13.727,
	13.927,
	14.049,
	14.448,
	15.402,
	14.452,
	12.539,
	10.438,
	9.732,
	8.709,
	7.117,
	5.574,
	4.164,
	1.937,
	1.028,
	0.61,
	0.539,
	0.474,
	0.423,
	0.385,
	0.351,
	0.35,
	4.653,
	7.183,
	8.627,
	9.11,
	6.92,
	4.539,
	2.907,
	1.898,
	1.265,
	1.216,
	1.165,
	1.145,
	0.737,
	0.606,
	0.541,
	0.494,
	0.455,
	0.425,
	0.401,
	1.185,
	4.787,
	6.965,
	8.405,
	8.968,
	7.249,
	4.971,
	2.761,
	1.806,
	1.156,
	1.044,
	0.975,
	0.555,
	0.524,
	0.51,
	0.465,
	0.423,
	0.389,
	0.36,
	0.342,
	0.343,
	3.61,
	5.857,
	7.812,
	7.938,
	6.483,
	4.172,
	2.776,
	1.754,
	1.268,
	1.157,
	1.114,
	1.06,
	1.007,
	0.005,
	0.035,
	0.049,
	0.054,
	0.056,
	0.055,
	0.054,
	0.052,
	0.05,
	0.048,
	0.047,
	0.049,
	0.049,
	0.049,
	0.05,
	0.051,
	0.052,
	0.053,
	0.054,
	0.055,
	0.056,
	0.058,
	0.059,
	0.061,
	0.063,
	0.065,
	0.067,
	0.068,
	0.07,
	0.072,
	0.073,
	0.074,
	0.076,
	0.076,
	0.077,
	0.078,
	0.078,
	0.078,
	0.078,
	0.078};

	double torque[243] = {0.007,
	0.007,
	0.007,
	0.007,
	0.007,
	0.007,
	0.007,
	0.007,
	0.007,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.008,
	0.009,
	0.009,
	0.009,
	0.009,
	0.014,
	0.014,
	0.013,
	0.012,
	0.012,
	0.007,
	0.006,
	0.006,
	0.005,
	0.005,
	0.005,
	0.005,
	0.004,
	0.004,
	0.004,
	0.004,
	0.004,
	0.004,
	0.004,
	0.005,
	0.005,
	0.005,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.005,
	0.005,
	0.005,
	0.005,
	0.005,
	0.005,
	0.005,
	0.005,
	0.006,
	0.006,
	0.006,
	0.007,
	0.007,
	0.007,
	0.007,
	0.006,
	0.006,
	0.006,
	0.006,
	0.006,
	0.005,
	0.005,
	0.005,
	0.005,
	0.008,
	0.01,
	0.011,
	0.012,
	0.013,
	0.013,
	0.013,
	0.013,
	0.014,
	0.014,
	0.006,
	0.004,
	0.004,
	0.004,
	0.003,
	0.003,
	0.003,
	0.003,
	0.003,
	0.003,
	0.002,
	0.002,
	0.002,
	0.002,
	0.002,
	0.002,
	0.002,
	0.002,
	0.002,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001,
	0.001};*/

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();
	double distance = 0, time = 0;

	for (int i = 0; i< 127; i++)
	{
		distance += 0.1;
		time += 0.2;
		CLOG(INFO, "data") << heatModeler->updateHeatModel(force[i], torque[i], distance, time);
	}
	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end - start;
	CLOG(INFO, "default") << "It took " << elapsed_seconds.count() << " microseconds.";

	delete heatModeler;
	return 0;
}
