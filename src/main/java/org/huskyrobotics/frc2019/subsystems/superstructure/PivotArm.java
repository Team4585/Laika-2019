package org.huskyrobotics.frc2019.subsystems.superstructure;

//import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.subsystems.superstructure.PivotArmObject.EncoderMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.ghrobotics.lib.mathematics.twodim.*;

//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class PivotArm extends Subsystem {
      private double m_targetAngle;
      private double m_currentAngle;
      private PivotArmObject Pivot;
      private static PivotArm m_instance;
      
      //private AnalogPotentiometer m_potent;
      //variables used for PID
      private final double kP = 1;//Speed Constant
      private final double kI = 0.001;//Speed up constant
      private final double kD = 1;//Slow down constant
      private final int kTimeoutMs = 100;
      private final int kF = 1;
      
      public void initDefaultCommand() 
	{
        //setDefaultCommand(new UseDrivetrain());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}
      public synchronized static PivotArm getInstance() {
            if (m_instance == null) m_instance = new PivotArm();
            return m_instance;
          }
      private PivotArm(){
            Pivot = new PivotArmObject(RobotMap.kPivotMaster, EncoderMode.QuadEncoder);
            Pivot.getPivot().configClosedloopRamp(0.4, 10);
      }
      public void setArmAxis(double input) {
            if(input > 0.1) {
                  goUp();
            } else if(input < -0.1) {
                  goDown();
            } else {
                  Pivot.stop();
            }
      }
	public void setIsClimbActive (boolean input) {
		if (input) {
			setTarget(0);
		}
	}
      //To be called by Robot.java. Will move the arm towards the target position.
      public void periodic() {
            Pivot.getDegree();
      }

      public double getCurrentAngle() {
            return m_currentAngle;
      }

      public void setTarget(double angle) {
            m_targetAngle = angle;
      }

      public void goUp() {
            setTarget(90);
      }
      public void goDown() {
            setTarget(0);
      }
}