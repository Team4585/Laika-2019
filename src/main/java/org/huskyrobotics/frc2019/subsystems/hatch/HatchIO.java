package org.huskyrobotics.frc2019.subsystems.hatch;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchIO extends Subsystem {
    public void initDefaultCommand() 
    {
      //setDefaultCommand(new UseDrivetrain());
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
    
    private double m_targetAngle;
    private double m_currentAngle;

    private TalonSRX m_motor;
    //variables used for PID
    private final double kP = 1;//Speed Constant
    private final double kI = 0.001;//Speed up constant
    private final double kD = 1;//Slow down constant
    private final int kTimeoutMs = 100;
    private final int kF = 1;

    public HatchIO(int MotorPort) {
        m_motor = new TalonSRX(MotorPort);
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
    /*
     * Method to toggle the Hatch mechanism motors between open and closed
     */
    public void setHatchToggle () {
        if(m_targetAngle < 45) {
            setTarget(90);
        } else if(m_targetAngle >= 45) {
            setTarget(0);
        }
    }

    /**
     * Sets the target angle
     * @param angle the target angle
     */
    public void setTarget(double angle) {
        m_targetAngle = angle;
    }

    /**
     * Gets the current angle
     * @return the current angle
     */
    public double getCurrentAngle() {
        return m_currentAngle;
    }

    /**
     * Moves the hatch to its target position and updates its current angle
     */
    public void periodic() {
          calculateAngle();
          m_motor.set(ControlMode.Position, m_targetAngle);
    }

    /**
     * Used to calculate the current angle of the arm
     */
    private void calculateAngle() {
        m_currentAngle = m_motor.getSelectedSensorPosition()/360;
    }
}
