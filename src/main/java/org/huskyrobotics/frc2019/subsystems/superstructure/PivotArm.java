package org.huskyrobotics.frc2019.subsystems.superstructure;

//import org.huskyrobotics.frc2019.subsystems.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class PivotArm extends Subsystem {
      private double m_targetAngle;
      private double m_currentAngle;

      private TalonSRX m_motor;
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

      public PivotArm(int motorPort, int sensorPort) {
            m_motor = new TalonSRX(motorPort);
            m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
            m_motor.setSensorPhase(true);
            m_motor.configNominalOutputForward(0, kTimeoutMs);
            m_motor.configNominalOutputReverse(0, kTimeoutMs);
            m_motor.configPeakOutputForward(1, kTimeoutMs);
            m_motor.configPeakOutputReverse(-1, kTimeoutMs);
            
            m_motor.selectProfileSlot(0, 0);
            m_motor.config_kF(0, kF, kTimeoutMs);
            m_motor.config_kP(0, kP, kTimeoutMs);
            m_motor.config_kI(0, kI, kTimeoutMs);
            m_motor.config_kD(0, kD, kTimeoutMs);
            m_motor.config_IntegralZone(0, 100, kTimeoutMs);
      }
      public void setArmAxis(double input) {
            if(input > 0.1) {
                  goUp();
            } else if(input < -0.1) {
                  goDown();
            } else {
                  stop();
            }
      }
	public void setIsClimbActive (boolean input) {
		if (input) {
			setTarget(0);
		}
	}
      //To be called by Robot.java. Will move the arm towards the target position.
      public void periodic() {
            calculateAngle();
            m_motor.set(ControlMode.Position, m_targetAngle);
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
      public void stop() {
            setTarget(m_currentAngle);
            resetEncoder();
      }
      //Used to calculate the current angle of the arm
      private void calculateAngle() {
            m_currentAngle = m_motor.getSelectedSensorPosition()/360;
      }
      //Used when target is reached or when it should stop for any reason
      public void resetEncoder() {
            m_motor.setSelectedSensorPosition(0, 0, kTimeoutMs);
      }
}
 
