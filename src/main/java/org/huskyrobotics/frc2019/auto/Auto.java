package org.huskyrobotics.frc2018.auto;
import edu.wpi.first.wpilibj.Ultrasonic;

import java.lang.Math;
import org.huskyrobotics.frc2018.auto.FalconPathPlanner;

public class Auto {
	double seconds = 15.0; //the amount of seconds to run bot
	double step = 0.1; //the period of time between each change of speed (smaller step is more accurate)
	double robotWidth = 0.1; //distance in feet between wheels
	//seconds divided by step must be an integer
	double[][] empty = new double[][]{
		{0, 0},
	}; 
	double[][] l2l = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	double[][] l2r = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	double[][] m2l = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	double[][] m2r = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	double[][] r2l = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	double[][] r2r = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 
	Ultrasonic lUltra = new Ultrasonic(1,1);
	Ultrasonic rUltra = new Ultrasonic(1,1);
	public void doAuto(int startLoc, int end) { // for startLoc 0 = left, 1 = middle, 2 = right; for end 0 = left, 1 = right
		FalconPathPlanner path = new FalconPathPlanner(empty);
		switch (startLoc) {
		case 0: driveOffHab();
			if(end == 0) {
				path = new FalconPathPlanner(l2l);
			} else {
				path = new FalconPathPlanner(l2r);
			}
			break;
		case 1: if(end == 0) {
				path = new FalconPathPlanner(m2l);
			} else {
				path = new FalconPathPlanner(m2r);
			}
			break;
		case 2:	driveOffHab();
			if(end == 0) {
				path = new FalconPathPlanner(r2l);
			} else {
				path = new FalconPathPlanner(r2r);
			}
			break;
		}
		path.calculate(seconds, step, robotWidth);
		path.driveBot(seconds, step);
		//TODO place thing
	}
		void allignRobot(int placeholdDis2, int placeholdDis1) {
		int disBetSen = 2;
		double degrees = 0;
		if(placeholdDis1 > placeholdDis2 ) {
			degrees = Math.toDegrees(Math.atan((placeholdDis1-placeholdDis2)/(disBetSen))); //calculates angle by doing the inverse tan of the difference between the two sensor outputs over distance between the two sensors
		} else if(placeholdDis1 < placeholdDis2) {
			degrees = Math.toDegrees(Math.atan((placeholdDis2-placeholdDis1)/(disBetSen))); 
		}
		//TODO turn robot by degrees
	}
	void driveOffHab() {
		//TODO drive bot forward by x dis to get bot off hab
		allignRobot((int) lUltra.getRangeInches(), (int) rUltra.getRangeInches());
		//TODO drive robot back by placeholdDis1 - Desired dis away from hab
	}
	
	
}

