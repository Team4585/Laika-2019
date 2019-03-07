package org.huskyrobotics.frc2019.autonomous;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

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
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

public class Trajectories {
    public static HashMap<Pose2d[], TimedTrajectory<Pose2dWithCurvature>> generatedTrajectories = new HashMap<Pose2d[], TimedTrajectory<Pose2dWithCurvature>>();

    public static Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
    public static Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
  
    public static Velocity<Length> kDefaultVelocity = VelocityKt.getVelocity(LengthKt.getFeet(2));
    public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(4));

    private static List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(
        // This limits our centripetal acceleration to 3 feet per second per second
        new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(3))),
        // This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
        new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(2.0)))
    );

    public static TimedTrajectory<Pose2dWithCurvature> Hatch;

    public static void generateAllTrajectories(){  
        Hatch = generateTrajectory(
            Arrays.asList(
                    new Pose2d(LengthKt.getFeet(2), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
                    new Pose2d(LengthKt.getFeet(17), LengthKt.getFeet(25), Rotation2dKt.getDegree(0))
            ),
            false
    );  
        
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