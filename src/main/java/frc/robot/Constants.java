package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;


public class Constants {
    
    /* Physical Constants */

    //Wheels
    public static final double c_WheelTrackWidth = 0; //change when Drivetrain is built
    public static final double c_WheelDiameter = 4; //change if we end up using 6" wheels
    public static final double c_WheelRadius = c_WheelDiameter / 2.0;
    public static final double c_TrackScrubFactor = 0; //Tune this with the drivetrain
    /*Scrub Radius is defined as the amount of friction resisting the turning motion
      The best way to overcome friction from the wheels is to have a center of gravity
      closer to the middle of the drivetrain*/

    //Dynamics we can calculate if we want
    /*These would include things like intertia (linear and angular), 
      velocity in terms of rad/s, and acceleration in terms of rad/s^2*/
    
    //Geometry
    public static final double c_CenterToFrontBumper = 0; //This needs to be done by dividing the total length of the robot include bumpers divided by 2
    public static final double c_CenterToBackBumper = 0; //Same calculation as c_CenterToFrontBumper
    public static final double c_CenterToSideBumper = 0; //This is done by taking the width of the robot including bumpers divided by 2

    /* CONTROL LOOP GAINS */

    //Gearing and mechanical gains
    /*This is where we put our robot velocity and angular velocity, as well as any required pathing constants*/

    //PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double c_DriveLowGearVelocityKp = 0; //Tune 
    public static final double c_DriveLowGearVelocityKi = 0.0; //Tune 
    public static final double c_DriveLowGearVelocityKd = 0.0; //Tune 
    public static final double c_DriveLowGearVelocityKf = 0.0; //Tune 
    public static final int c_DriveLowGearVelocityIZone = 0; //Tune 
    public static final double c_DriveVoltageRampRate = 0.0; //Tune 

    //PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double c_DriveHighGearVelocityKp = 0; //Tune
    public static final double c_DriveHighGearVelocityKi = 0.0; //Tune
    public static final double c_DriveHighGearVelocityKd = 0.0; //Tune
    public static final double c_DriveHighGearVelocityKf = 0.0; //Tune
    public static final int c_DriveHighGearVelocityIZone = 0; //Tune

    public static final int c_ShifterSolenoidID = 12;
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
}