/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2019.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.huskyrobotics.lib.drivers.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.Gyro;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.wrappers.FalconMotor;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class Drive extends Subsystem {
    public static Drive instance;
    public static Localization localization;
    public static DifferentialDrive differentialDrive;
    public static DCMotorTransmission dcMotorTransmission;
  
    private final TalonSRX masterRight, masterLeft;
    private final VictorSPX slaveRight, slaveLeft;

    public Drive(){
       masterRight = TalonSRXFactory.createDefaultTalon(0);
       slaveRight = VictorSPXFactory.createPermanentSlaveVictorSPX(0, 0); //I'm not 100% this will work

       masterLeft = TalonSRXFactory.createDefaultTalon(1);
       slaveLeft = VictorSPXFactory.createPermanentSlaveVictorSPX(1, 1);




    }

    public TalonSRX getLeftTalon(){
        return masterLeft;
    }

    public TalonSRX getRightTalon(){
        return masterRight;
    }



    @NotNull
    @Override
    public FalconMotor<Length> getLeftMotor() {
        return masterLeft;
    }

    @NotNull
    @Override
    public FalconMotor<Length> getRightMotor() {
        return masterRight;
    }

    @NotNull
    @Override
    public TrajectoryTracker getTrajectoryTracker() {
        return null;
    }

    @Override
    protected void initDefaultCommand() {
    }

    public static synchronized Drive getInstance(){
        if(instance == null){
            instance = new Drive();
        }

        return instance;
    }
}