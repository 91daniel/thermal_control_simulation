/*
 * heatModel.h
 *
 *  Created on: Aug 5, 2016
 *      Author: tom
 */

#ifndef HEATMODEL_H_
#define HEATMODEL_H_

#define NUMBER_ASSESSMENT_POINTS 21
#define NUMBER_TIME_POINTS 500
#define NUMBER_POINTS_ON_DISK 11


class heatModel {
public:
	heatModel();
	virtual ~heatModel();

	void setUpHeatModel(double trajectoryLength);
	double updateHeatModel(double force, double torque, double currentPosition,double time);
private:

	// Times
	double m_startTime; // Absolute time
	double m_previousTime; // Relative to start
	double m_currentTime; // Relative to start

	// Constant parameters
	double m_omega;
	double m_ro; // density
	double m_cp; // specific heat
	double m_K;   // thermal conductivity
	double m_alpha; // thermal diffusivity

	double m_deltaR; // disk step size

	double m_A;
	double m_B;
	double m_drillRadius;
	double m_drillArea;
	double m_lateralDistFromTrajectory;

	double m_pointsOnDisk[NUMBER_POINTS_ON_DISK]; //Points on radius of disk
	double m_assessmentDistances[NUMBER_ASSESSMENT_POINTS]; // Points along the trajectory at which we assess the temperature

	// Parameter to be updated
	double m_temperatureAtPoint[NUMBER_ASSESSMENT_POINTS][NUMBER_TIME_POINTS]; // The modeled temperature at the points/distances in assessmentDistances
	double m_previousPosition;

	int m_pointCounter;

};

#endif /* HEATMODEL_H_ */
