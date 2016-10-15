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

	std::vector<double> force;
	std::vector<double> torque;
	// Read file
	fstream file("FTData1.txt", ios::in);
	string line;
	int ctr(0);
	while (getline(file, line))
	{
		std::stringstream   linestream(line);
		std::string         value;

		while (getline(linestream, value, ';'))
		{
			if (ctr == 0) {
				force.push_back(stod(value));
			}
			else {
				torque.push_back(stod(value));
			}
			ctr += 1;
		}

		ctr = 0;
		std::cout << "Line Finished" << std::endl;
	}

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

