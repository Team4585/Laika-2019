package mainPack;
import java.lang.Math;
import lineSmoother.FalconPathPlanner;

public class Auto {
	static double seconds = 15.0; //the amount of seconds to run bot
	static double step = 0.1; //the period of time between each change of speed (smaller step is more accurate)
	static double robotWidth = 0.2; //distance in feet between wheels
	//seconds divided by step must be an integer
	public static void doAuto(int startLoc, int end) { // for startLoc 0 = left, 1 = middle, 2 = right; for end 0 = left, 1 = right
		switch (startLoc) {
		case 0: driveOffHab();
			if(end == 0) {
				l2l();
			} else {
				l2r();
			}
			break;
		case 1: if(end == 0) {
				m2l();
			} else {
				m2r();
			}
			break;
		case 2:	driveOffHab();
			if(end == 0) {
				r2l();
			} else {
				r2r();
			}
			break;
		}
		//TODO adjust bot and place thing
	}
		static void allignRobot(int placeholdDis2, int placeholdDis1) {
		int disBetSen = 2;
		double degrees = 0;
		if(placeholdDis1 > placeholdDis2 ) {
			degrees = Math.toDegrees(Math.atan((placeholdDis1-placeholdDis2)/(disBetSen))); //calculates angle by doing the inverse tan of the difference between the two sensor outputs over distance between the two sensors
		} else if(placeholdDis1 < placeholdDis2) {
			degrees = Math.toDegrees(Math.atan((placeholdDis2-placeholdDis1)/(disBetSen))); 
		}
		//TODO turn robot by degrees
	}
	static void driveOffHab() {
		//TODO drive bot forward by x dis to get bot off hab
		allignRobot(0, 0);
		//TODO drive robot back by placeholdDis1 - Desired dis away from hab
	}
	
	static void driveBot() {
		for(int i = 0; i < (seconds * step); i++) {
			//set rightmotor to ((this.smoothRightVelocity[i][1]));
			//set rightmotor to ((this.smoothLeftVelocity[i][1]));
			//sleep(step*1000)
		}
	}

	static void l2l() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
	static void m2l() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
	static void r2l() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
	static void l2r() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
	static void m2r() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
	static void r2r() {
		double[][] waypoints = new double[][]{
			{1, 1},
			{5, 1},
			{9, 12},
			{12, 9},
			{15, 6},
			{19, 12}
			//TODO make path
		}; 

		FalconPathPlanner path = new FalconPathPlanner(waypoints);
		path.calculate(seconds, step, robotWidth);
	}
}
