/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.commands.Auto;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.huskyrobotics.frc2019.FalconAuto.*;
import org.huskyrobotics.frc2019.commands.Turn;
import org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff.FalconDrive;

import edu.wpi.first.wpilibj.command.WaitCommand;

public class CargoRoutine extends AutoCommandGroup {
    private ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
    private static FalconDrive m_Drive = FalconDrive.getInstance();
  
    /**
     * 1 cargo hard coded Auto (We might not get Vision in at SDR, so it's a backup)
     * @param startPos The starting position (L, M, or R)
     * @param side to target (L or R)
     */
    public CargoRoutine(char startPos, char side) {
      String Start = "hab" + startPos;
  
      /* Get a trajectory to move to the cargo ship */
      TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedTrajectories.get(Start + " to " + "cargoM" + side); //Generated using the Locs map in Trajectories.java
      trajects.add(traject);
      this.addSequential(m_Drive.followTrajectory(traject, true)); //drive to goal
      /*this.addSequential(new Turn(Trajectories.locations.get("cargo" + side + '1').component2()));
      //this.addParallel(new VisionTest());
      this.addSequential(new WaitCommand(0.5));
      //this.addSequential(new ReleaseCargo());
  
      Start = "CargoM" + side;

      traject = Trajectories.generatedTrajectories.get(Start + " to " + "Depot" + side);
      this.addSequential(m_Drive.followTrajectory(traject));
      this.addSequential(new Turn(Trajectories.locations.get("cargo" + side + '1').component2()));
      //this.addSequential(new GrabCargo());
      this.addSequential(new Turn(Trajectories.locations.get("Depot" + side + '1').component2()));
*/
    }

  }
