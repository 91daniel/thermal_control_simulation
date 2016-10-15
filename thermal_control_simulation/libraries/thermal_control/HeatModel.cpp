/*
 * heatModel.cpp
 *
 *  Created on: Aug 5, 2016
 *      Author: tom
 */

#include "heatModel.h"
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif 
#include <math.h>
#include <string.h>
#include <chrono>
#include <ctime>
 //#include <sys/time.h>
#include <iostream>
#include <fstream>

using namespace std;
heatModel::heatModel() {
	// Times
	m_startTime = 0.0; // Absolute time
	m_previousTime = 0.0; // Relative to start
	m_currentTime = 0.0; // Relative to start

	// Define constants
	m_ro = 1800;
	m_cp = 1260;
	m_K = 0.6;
	m_omega = 2*M_PI*(1000.f / 60.f);
	m_alpha = m_K / (m_ro*m_cp); // thermal diffusivity


	m_A = 0.68; // Force calibration factor
	m_B = 41.0; // Torque calibration factor
	m_drillRadius = 0.9e-3; // Radius in m
	m_drillArea = M_PI*(m_drillRadius *m_drillRadius ); //area of drill bit
	m_lateralDistFromTrajectory = m_drillRadius + 0.5e-3; //1.4 mm: distance(radius) to point source 0.5 mm + 0.9 mm(radius)

	m_deltaR = m_drillRadius / (NUMBER_POINTS_ON_DISK-1);
	memset(&m_temperatureAtPoint[0],0.0,NUMBER_ASSESSMENT_POINTS*NUMBER_TIME_POINTS*sizeof(double)); // The calculated temperature at the equivalent position in m_assessmentDistances
	memset(&m_pointsOnDisk[0],0.0,NUMBER_POINTS_ON_DISK*sizeof(double)); // Points at differing radii
	memset(&m_assessmentDistances[0],0.0,NUMBER_ASSESSMENT_POINTS*sizeof(double)); // Points along the trajectory
	m_previousPosition = 0;
	m_pointCounter = 0;
}

heatModel::~heatModel() {
	// TODO Auto-generated destructor stub
}

void heatModel::setUpHeatModel(double trajectoryLength)
{
	// Windows (chrono, ctime)
	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now();
	std::time_t start_time = std::chrono::system_clock::to_time_t(start);
	m_startTime = start_time;

	// Linux (sys/time)
	//	timeval currentTime;
	//	gettimeofday(&currentTime, NULL);
	//	m_startTime = currentTime.tv_sec + (currentTime.tv_usec / 1000000.0);
	m_previousTime = 0;
	m_previousPosition = 0;
	memset(&m_temperatureAtPoint[0],0.0,NUMBER_ASSESSMENT_POINTS*NUMBER_TIME_POINTS*sizeof(double)); // The calculated temperature at the equivalent position in m_assessmentDistances
	memset(&m_pointsOnDisk[0],0.0,NUMBER_POINTS_ON_DISK*sizeof(double)); // Points at differing radii
	memset(&m_assessmentDistances[0],0.0,NUMBER_ASSESSMENT_POINTS*sizeof(double)); // Points along the trajectory
	for (int i = 0; i< NUMBER_ASSESSMENT_POINTS; i++)
	{
		m_assessmentDistances[i] = ((double) (i)*trajectoryLength/((double)(NUMBER_ASSESSMENT_POINTS)-1))*1.0e-3;
	}
	for (int i = 0; i< NUMBER_POINTS_ON_DISK; i++)
	{
		m_pointsOnDisk[i] = (double) (i)*m_drillRadius/((double)(NUMBER_POINTS_ON_DISK)-1);
	}

	m_pointCounter = 0;
}

double heatModel::updateHeatModel(double force, double torque, double currentPosition, double time) //Note time is just for debugging
{
	m_pointCounter++;
	// Get the time
	// Windows (chrono, ctime)
	//std::chrono::time_point<std::chrono::system_clock> current_time_point;
	//current_time_point = std::chrono::system_clock::now();
	//std::time_t current_time = std::chrono::system_clock::to_time_t(current_time_point);
	//m_currentTime = current_time;

	// Linux (sys/time)
	//	timeval currentTime;
	//	gettimeofday(&currentTime, NULL);
	//	m_currentTime = time;//currentTime.tv_sec + (currentTime.tv_usec / 1000000.0) - m_startTime; // Replace this with time to debug
	
	m_currentTime = time;

	double currentPositionInMeters = currentPosition*1.0e-3;
	// Calculate energy
	double deltaT = m_currentTime - m_previousTime; //0.2s
	double v = ((currentPositionInMeters-m_previousPosition)/deltaT); // feed rate in meter

	torque = torque < 0 ? 0:torque;
	force = force < 0 ? 0:force;
	double energy = torque*deltaT*m_omega*m_A + force*v*deltaT*m_B;  //Energy function
	double energyN = energy/m_drillArea;  //Normalize with area of drill bit(necessary for disk source model to use for both diameters)
	double maxTemp = 0.0;

	for (int a = 0; a <  NUMBER_ASSESSMENT_POINTS; a++) // Integrate for each defined point along x axis, this stays (this is looped every time the function is called)
	{
		for (int b = 0; b < NUMBER_POINTS_ON_DISK; b++)// Integrate over disk radius, this stays
		{
			for (int d = 0; d < NUMBER_TIME_POINTS-m_pointCounter; d++)
			{
				// changed j0 to _j0 (Windows gave a compiling error telling me to change that...)
				double besselPart = _j0(m_lateralDistFromTrajectory*m_pointsOnDisk[b] /(2.0*m_alpha*(((double) (d+1))*deltaT)));
				double energyPart = (energyN*m_pointsOnDisk[b] * m_deltaR) / (4.0 * m_ro*m_cp*sqrt(M_PI)*pow(m_alpha*(((double) (d+1))*deltaT), 1.5));
				double exponentialPart = exp(-(pow(m_lateralDistFromTrajectory,2.0) + pow(m_pointsOnDisk[b],2.0) + pow(m_assessmentDistances[a] - currentPositionInMeters,2.0)) / (4.0 * m_alpha*((double) (d+1)*deltaT)));
				double currentResult = isnan(energyPart*exponentialPart*besselPart) ? 0 : energyPart*exponentialPart*besselPart;
				m_temperatureAtPoint[a][d+m_pointCounter] += currentResult;
			}
		}

		maxTemp = m_temperatureAtPoint[a][m_pointCounter-1] > maxTemp ? m_temperatureAtPoint[a][m_pointCounter-1] : maxTemp;
	}

	m_previousTime = m_currentTime;
	m_previousPosition = currentPositionInMeters;
	maxTemp+=25;
	return maxTemp;
}
