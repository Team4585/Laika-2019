
package org.huskyrobotics.frc2019.subsystems.superstructure;

//import org.huskyrobotics.frc2019.subsystems.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class PivotArm extends Subsystem {
      private double m_targetAngle;
      private double m_currentAngle;

      private VictorSPX m_motor;
      private AnalogPotentiometer m_potent;

      private final int c_P = 1;
      private final int c_I = 1;
      private final int c_D = 1;
      private int integral = 0;
      private double previousError, error = 0;
      private final double c_timeConstant = 0.02;
      private double PIDOutput = 0;

      public void initDefaultCommand() 
	{
        //setDefaultCommand(new UseDrivetrain());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
	}

      public PivotArm(int motorPort, int sensorPort) {
		m_motor = new VictorSPX(motorPort);
            m_potent = new AnalogPotentiometer(sensorPort, 360, 0);
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
            resetPID();
      }

      public void periodic() {
            calculateAngle();
            calculatePID();
            
            m_motor.set(ControlMode.PercentOutput, PIDOutput);
      }
      private void calculateAngle() {
            m_currentAngle = m_potent.get();
      }
      private void calculatePID() {
            previousError = error;
            error = m_targetAngle - m_currentAngle;
            integral += (error*c_timeConstant);
            var derivative = (error - previousError) / c_timeConstant;
            PIDOutput = c_P*error + c_I*integral + c_D*derivative;
      }
      public void resetPID() {
            integral = 0;
            previousError = 0;
            error = 0;
      }
}
