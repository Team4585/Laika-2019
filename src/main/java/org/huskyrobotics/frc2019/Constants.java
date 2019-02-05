package org.huskyrobotics.frc2019;

import edu.wpi.first.wpilibj.Solenoid;


public class Constants {
    public static final double kLooperDt = 0.01;
    /* Physical Constants */

    //Wheels
    public static final double kWheelTrackWidth = 0; //change when Drivetrain is built
    public static final double kWheelDiameter = 6; //change if we end up using 6" wheels
    public static final double kWheelRadius = kWheelDiameter / 2.0;
    public static final double kTrackScrubFactor = 0; //Tune this with the drivetrain
    /*Scrub Radius is defined as the amount of friction resisting the turning motion
      The best way to overcome friction from the wheels is to have a center of gravity
      closer to the middle of the drivetrain*/
    public static final double kNativeUnitModel = 0;
    //Dynamics we can calculate if we want
    /*These would include things like intertia (linear and angular), 
      velocity in terms of rad/s, and acceleration in terms of rad/s^2*/
    
    //Geometry
    public static final double kCenterToFrontBumper = 0; //This needs to be done by dividing the total length of the robot include bumpers divided by 2
    public static final double kCenterToBackBumper = 0; //Same calculation as c_CenterToFrontBumper
    public static final double kCenterToSideBumper = 0; //This is done by taking the width of the robot including bumpers divided by 2

    /* CONTROL LOOP GAINS */
    //Motor Controller Config
    public static final int kTimeoutMs = 100;
    public static final int drivePIDIdx = 0;
    public static final int kLGvelocityConstant = 10; //We need to tune this to find our velocity
    public static final int kHGvelocityConstant = 10;
    //Gearing and mechanical gains
    /*This is where we put our robot velocity and angular velocity, as well as any required pathing constants*/

    //PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0; //Tune 
    public static final double kDriveLowGearVelocityKi = 0.0; //Tune 
    public static final double kDriveLowGearVelocityKd = 0.0; //Tune 
    public static final double kDriveLowGearVelocityKf = 0.0; //Tune 
    public static final int kDriveLowGearVelocityIZone = 0; //Tune 
    public static final double kDriveVoltageRampRate = 0.0; //Tune 

    //PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveHighGearVelocityKp = 0; //Tune
    public static final double kDriveHighGearVelocityKi = 0.0; //Tune
    public static final double kDriveHighGearVelocityKd = 0.0; //Tune
    public static final double kDriveHighGearVelocityKf = 0.0; //Tune
    public static final int kDriveHighGearVelocityIZone = 0; //Tune

    public static final int kShifterSolenoidID = 12;
    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId <= 4) {
            // These solenoids are on PCM 1, wired 1-4 to 7-4.
            return new Solenoid(1, 8 - solenoidId);
        } else if (solenoidId <= 8) {
            // These solenoids are on PCM 0, wired 5-8 to 0-3.
            return new Solenoid(0, solenoidId - 5);
        } else if (solenoidId <= 12) {
            // These solenoids are on PCM 0, wired 9-12 to 7-4.
            return new Solenoid(0, 16 - solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }







    public static final int kWillToLive = 0;
}