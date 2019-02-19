package org.huskyrobotics.frc2019.FalconAuto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;


import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;

public class Trajectories {

        public static HashMap<String, Pose2d> locations = new HashMap<String, Pose2d>();

        private static void genLocs() {
		locations.put("habL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684), Rotation2dKt.getDegree(180)));
		locations.put("habM", new Pose2d(LengthKt.getFeet(5.181), LengthKt.getFeet(13.379), Rotation2dKt.getDegree(180)));
		locations.put("habR", new Pose2d(LengthKt.getFeet(5.141), LengthKt.getFeet(9.508), Rotation2dKt.getDegree(180)));
		locations.put("loadingL", new Pose2d(LengthKt.getFeet(1.286), LengthKt.getFeet(25.021), Rotation2dKt.getDegree(180.0)));
		locations.put("loadingR", new Pose2d(LengthKt.getFeet(1.325), LengthKt.getFeet(2.336), Rotation2dKt.getDegree(180.0)));
		locations.put("cargoL1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d)));
		locations.put("cargoL2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d))); 
		locations.put("cargoL3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(17.235), Rotation2dKt.getDegree(-90d))); 
		locations.put("cargoML", new Pose2d(LengthKt.getFeet(17.101), LengthKt.getFeet(14.338), Rotation2dKt.getDegree(180)));
		locations.put("cargoMR", new Pose2d(LengthKt.getFeet(17.066), LengthKt.getFeet(12.653), Rotation2dKt.getDegree(180)));
		locations.put("cargoR1", new Pose2d(LengthKt.getFeet(21.565), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d))); 
		locations.put("cargoR2", new Pose2d(LengthKt.getFeet(23.532), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d))); 
		locations.put("cargoR3", new Pose2d(LengthKt.getFeet(25.277), LengthKt.getFeet(9.898), Rotation2dKt.getDegree(90d))); 
		locations.put("depotL", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(20.517), Rotation2dKt.getDegree(0)));
		locations.put("depotR", new Pose2d(LengthKt.getFeet(5.203), LengthKt.getFeet(6.107), Rotation2dKt.getDegree(0)));

	}


	public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> generatedTrajectories = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
	public static List<String> grabs = new ArrayList<String>(Arrays.asList("habR", "habM", "habL", "loadingL", "loadingR", "depotL", "depotR"));
	public static List<String> puts = new ArrayList<String>(Arrays.asList("cargoL1", "cargoL2", "cargoL3", "cargoML", "cargoMR", "cargoR1", "cargoR2", "cargoR3",
			"rocketL1", "rocketR1"));

    public static Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
    public static Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
  
    public static Velocity<Length> kDefaultVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0.3));
    public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(0.3));

    private static List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(
        // This limits our centripetal acceleration to 3 feet per second per second
        new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(0.3))),
        // This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
        new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(0.3)))
    );

    public static TimedTrajectory<Pose2dWithCurvature> Hatch;


    public static void generateAllTrajectories() {
		generateAllTrajectories(true);
	}
    public static void generateAllTrajectories(Boolean isReal){  
        System.out.println("Generating ALL trajectories");
		genLocs();
		double startTime = 0;
		if (isReal){
            startTime = Timer.getFPGATimestamp();
        }
        generatedTrajectories.put("Hatch", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                    new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                    new Pose2d(LengthKt.getFeet(7), LengthKt.getFeet(17), Rotation2dKt.getDegree(0))
            )),
            false       
            ));
            
        generatedTrajectories.put("leftFLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(17.475), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(21.627), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
        )),
        false       
        ));

        generatedTrajectories.put("leftMLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(20.686), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(23.544), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
            )),
            false       
            ));

        generatedTrajectories.put("leftBLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(22.782), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(25.283), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
            )),
            false       
            ));

        generatedTrajectories.put("leftForwardLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(17.208), LengthKt.getFeet(14.286), Rotation2dKt.getDegree(0))
            )),
            false       
            ));

        generatedTrajectories.put("leftForwardRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(17.208), LengthKt.getFeet(12.601), Rotation2dKt.getDegree(0))
            )),
            false       
            ));

        generatedTrajectories.put("leftFRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
            Arrays.asList(
                new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(18), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
                new Pose2d(LengthKt.getFeet(21.627), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
            )),
            false       
            ));

        generatedTrajectories.put("leftMRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(23.366), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
        )),
        false       
        ));
        
        generatedTrajectories.put("leftBRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(21.472), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(24.971), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
        )),
        false       
        ));

        generatedTrajectories.put("rightFLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(17.475), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(21.627), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
        )),
        false       
        ));

        generatedTrajectories.put("rightMLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(20.686), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(23.544), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
        )),
        false       
        ));

        generatedTrajectories.put("rightBLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(22.782), LengthKt.getFeet(17), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(25.283), LengthKt.getFeet(17), Rotation2dKt.getDegree(-90))
        )),
        false       
        ));

        generatedTrajectories.put("rightForwardLCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(17.208), LengthKt.getFeet(14.286), Rotation2dKt.getDegree(0))
        )),
        false       
        ));

        generatedTrajectories.put("rightForwardRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(17.208), LengthKt.getFeet(12.601), Rotation2dKt.getDegree(0))
        )),
        false       
        ));

        generatedTrajectories.put("rightFRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(18), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(21.627), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
        )),
        false       
        ));

        generatedTrajectories.put("rightMRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(23.366), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
        )),
        false       
        ));

        generatedTrajectories.put("rightBorderRCargoBay", generateTrajectory(new  ArrayList<Pose2d>(
        Arrays.asList(
            new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(21.472), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
            new Pose2d(LengthKt.getFeet(24.971), LengthKt.getFeet(10), Rotation2dKt.getDegree(90))
        )),
        false       
        ));

    System.out.println("Out of first round of generation");
		double now = 0;
		if (isReal) {
			now = Timer.getFPGATimestamp();
			System.out.println("Trajectories generated in " + (now - startTime) + " seconds!");
		}
    }
    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, boolean reversed){
        return generateTrajectory(waypoints, constraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, false);
      }
    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, 
                                List<TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed){
        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
          waypoints,
          constraints_,
          startVelocity,
          endVelocity,
          maxVelocity,
          maxAcceleration,
          reversed
    );
  }
}