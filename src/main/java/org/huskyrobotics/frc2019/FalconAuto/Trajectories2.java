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
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.DifferentialDriveDynamicsConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;

public class Trajectories2 {
    public static FalconDrive m_Drive = FalconDrive.getInstance();
	public static HashMap<String, Pose2d> locations = new HashMap<String, Pose2d>();
	public static Velocity<Length> MaxSpeed = VelocityKt.getVelocity(LengthKt.getFeet(12.5));

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

	public static final Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
	public static final Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

	public static final Velocity<Length> kDefaultVelocity = VelocityKt.getVelocity(LengthKt.getFeet(5));
	public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));

    @SuppressWarnings("unused")
	private static final ArrayList<Pose2d> forward20ftSrc = new ArrayList<Pose2d>(Arrays.asList(
			new Pose2d(LengthKt.getFeet(20), LengthKt.getFeet(5), Rotation2dKt.getDegree(0)),
			new Pose2d(LengthKt.getFeet(35), LengthKt.getFeet(5), Rotation2dKt.getDegree(0))));
	public static TimedTrajectory<Pose2dWithCurvature> forward20Feet;

	private static List<TimingConstraint<Pose2dWithCurvature>> kConstraints = Arrays.asList(
			new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(10))),
			new DifferentialDriveDynamicsConstraint(m_Drive.getDifferentialDrive(), 12 /* volts */),
			// This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
            new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(2.0)))
            );

	public static void generateAllTrajectories() {
		generateAllTrajectories(true);
	}

	/**
	 * Generate all trajectories
	 * @param isReal
	 */
	public static void generateAllTrajectories(boolean isReal) {

		System.out.println("Generating ALL trajectories");
		genLocs();
		double startTime = 0;
		if (isReal)
			startTime = Timer.getFPGATimestamp();
		/*
		generatedTrajectories.put("habL to cargoML", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("habL"), locations.get("cargoML"))), true));
		generatedTrajectories.put("habM to cargoMR", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoMR"))), true));
		generatedTrajectories.put("habM to cargoML", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("habM"), locations.get("cargoML"))), true));
		generatedTrajectories.put("habR to cargoMR", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("habR"), locations.get("cargoMR"))), true));

		generatedTrajectories.put("cargoML to loadingL", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoML"), locations.get("loadingL"))), false));
		generatedTrajectories.put("cargoMR to loadingR", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoMR"), locations.get("loadingR"))), false));

		generatedTrajectories.put("cargoL1 to depotL", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoL1"), locations.get("depotL"))), true));
		generatedTrajectories.put("cargoR1 to depotR", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("cargoR1"), locations.get("depotR"))), true));

		generatedTrajectories.put("depotL to cargoL1", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotL"), locations.get("cargoL1"))), false));
		generatedTrajectories.put("depotR to cargoR1", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("depotR"), locations.get("cargoR1"))), false));
		*/
		generatedTrajectories.put("test", generateTrajectory(new ArrayList<Pose2d>(Arrays.asList(locations.get("loadingL"),
				new Pose2d(LengthKt.getFeet(15.9), LengthKt.getFeet(22.011), Rotation2dKt.getDegree(165)),
				new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)))),
				kConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, true));
		generatedTrajectories.put("test1", generateTrajectory(Arrays.asList(new Pose2d(LengthKt.getFeet(21.646), LengthKt.getFeet(19.223), Rotation2dKt.getDegree(-90)),
				locations.get("cargoL1")), false));

		System.out.println("Out of first round of generation");
		double now = 0;
		if (isReal) {
			now = Timer.getFPGATimestamp();
			System.out.println("Trajectories generated in " + (now - startTime) + " seconds!");
		}
	}

	/**
	* Generate a trajectory from a list of waypoints in low gear
	* @param waypoints to follow
	* @param reversed if the path is reversed
	* @return
	*/
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, boolean reversed) {
		return generateTrajectory(waypoints, kConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, reversed);
	}

	/**
	 * Generate a trajectory from scratch. Given a list of waypoints, constriants, velocities and accelerations, a trajectory will
	 * be generated using the default trajectory generator.
	 * @param waypoints to follow
	 * @param constraints_ to respect (slow zone, acceleration, etc)
	 * @param startVelocity to start at
	 * @param endVelocity to end at
	 * @param maxVelocity that will never be exceded
	 * @param maxAcceleration for the trajectory
	 * @param reversed for if the path should be reversed (flipped)
	 */
	public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints,
			List<? extends TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed) {
		return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
				waypoints,
				constraints_,
				startVelocity,
				endVelocity,
				maxVelocity,
				maxAcceleration,
				reversed);
	}

}