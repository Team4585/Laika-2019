package org.huskyrobotics.frc2019.subsystems.cargo;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.huskyrobotics.frc2019.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class CargoIO extends Subsystem{
	@Override
    public void initDefaultCommand() 
	{
        // setDefaultCommand(new UseDrivetrain());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
      }
	private VictorSPX m_motor;
	      //private AnalogPotentiometer m_potent;
      //variables used for PID
      private final double kP = 1;//Speed Constant
      private final double kI = 0.001;//Speed up constant
      private final double kD = 1;//Slow down constant
      private final int kTimeoutMs = 100;
      private final int kF = 1;
	//private SomeSensor _CheckSensor;
	public CargoIO (int MotorPort) {
		m_motor = new VictorSPX(MotorPort);
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

	public void intake (double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
		SmartDashboard.putNumber("Intake Speed", speed);
	}
	public void periodic(){
		if(Robot.m_Oi.GetCargoIn() == true){
			intake(0.5);
		}
		if(Robot.m_Oi.GetCargoOut() == true){
			intake(-1);
		}
	}

	public void stop () {
		m_motor.set(ControlMode.PercentOutput, 0);
	}
	@SuppressWarnings("unused")
	private void init() {
		m_motor.set(ControlMode.PercentOutput, 0);
	}
}