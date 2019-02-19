package org.huskyrobotics.frc2019.subsystems.superstructure;

//import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.RobotMap;
import org.huskyrobotics.frc2019.subsystems.superstructure.PivotArmObject.EncoderMode;

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
      private final int kIzone = 0;
      private final int kMaxI = 0;
      
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
            Pivot.getPivot().configClosedloopRamp(0.4, kTimeoutMs);
            Pivot.setClosedLoopGains(kP, kI, kD, kF, kIzone, kMaxI);
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
			setTarget(90);
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
            setTarget(m_targetAngle);
      }
      public void goDown() {
            setTarget(0);
      }
}