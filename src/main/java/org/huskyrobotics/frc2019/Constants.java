package org.huskyrobotics.frc2019;

import edu.wpi.first.wpilibj.Solenoid;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;

public class Constants {

    public class controls{
        public static final double kLooperDt = 0.01;
    }
    public class path{

    }
    public class physical{
        
    }

    public static class drivetrain{
            /** Drivetrain width in feet */
    public static final double wheel_base = 24/12; // TODO
    public static final double left_wheel_effective_diameter = 6; // units are in inches, TODO tune this!
    public static final double right_wheel_effective_diameter = 6; // units are in inches, TODO tune this!

    public static final Length left_radius = LengthKt.getInch(3);
    public static final Length right_radius = LengthKt.getInch(3);

    //Gearing and mechanical gains
    /*This is where we put our robot velocity and angular velocity, as well as any required pathing constants*/

    // Set speeds for teleop
    public static final double kMaxforwardHigh = 0; // Feet per second forward velocity
    public static final double kMaxturnHigh = 0; // Max turn speed in degrees per second
    public static final double kMaxforwardLow = 0; // Feet per second forward velocity
    public static final double kMaxturnLow = 0; // Max turn speed in degrees per second
    
    //Encoder resolution stuff for our Quad encoders
    public static final double kPPR = 4096;
    public static final NativeUnit kDriveSensorUnitsPerRotation = NativeUnitKt.getSTU(4096);
    public static final NativeUnitLengthModel kLeftNativeunitLengthmodel = new NativeUnitLengthModel(kDriveSensorUnitsPerRotation, left_radius);
    public static final NativeUnitLengthModel kRightNativeunitLengthmodel = new NativeUnitLengthModel(kDriveSensorUnitsPerRotation, right_radius);


    //Stuff for pathfinding
      public static final double kleftstatickv = 0.0; //TODO TUNE THIS! the voltage required to get the robot moving/overcome static friction
      public static final double krightstatickv = 0.0; //TODO TUNE THIS! the voltage required to get the robot moving/overcome static friction


    /* CONTROL LOOP GAINS */
    //Motor Controller Config
    public static final int kTimeoutMs = 100;
    public static final int drivePIDIdx = 0;
    public static final int kLGvelocityConstant = 10; //We need to tune this to find our velocity
    public static final int kHGvelocityConstant = 10;

    //PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    
    //Left Side of the above controls
    public static final double kLGleftKp = 5; //TODO Tune
    public static final double kLGleftKi = 0; //TODO Tune
    public static final double kLGleftKd = 3; //TODO Tune
    public static final double kLGleftKf = 20; //TODO Tune
    public static final double kLGleftIzone = 0; //TODO Tune
    public static final double kLGleftIntMax = 0; //TODO Tune


    //Right Side of the above Controls
    public static final double kLGrightKp = 10; //TODO Tune
    public static final double kLGrightKi = 0; //TODO Tune
    public static final double kLGrightKd = 3; //TODO Tune
    public static final double kLGrightKf = 20; //TODO Tune
    public static final double kLGrightIzone = 0; //TODO Tune
    public static final double kLGrightIntMax = 0; //TODO Tune

    //PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    
    //Left Side of the above controls
    public static final double kHGleftKp = 7; //TODO Tune
    public static final double kHGleftKi = 0; //TODO Tune
    public static final double kHGleftKd = 4; //TODO Tune
    public static final double kHGleftKf = 20; //TODO Tune
    public static final double kHGleftIzone = 0; //TODO Tune
    public static final double kHGleftIntMax = 0; //TODO Tune


    //Right Side of the above Controls
    public static final double kHGrightKp = 12; //TODO Tune
    public static final double kHGrightKi = 0; //TODO Tune
    public static final double kHGrightKd = 4; //TODO Tune
    public static final double kHGrightKf = 20; //TODO Tune
    public static final double kHGrightIzone = 0; //TODO Tune
    public static final double kHGrightIntMax = 0; //TODO Tune


    }
    public static class PivotArm{
        public static final NativeUnit kDriveSensorUnitsPerRotation = NativeUnitKt.getSTU(4096);
        public static final NativeUnitRotationModel kArmNativeunitRotationmodel = new NativeUnitRotationModel(kDriveSensorUnitsPerRotation);
    }
    public class pathfinder {
        public static final double gyro_correct_kp = 0.2;
      }

    public class limelight {
        public static final double kpGain = 0.0;
    }

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