package org.huskyrobotics.frc2019.subsystems.cargo;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class CargoIO {
	private VictorSPX m_motor;
	private DigitalInput m_limitSwitch;

	private double maxSpeed = 1.0;

	//private SomeSensor _CheckSensor;
	public CargoIO (int MotorPort, int SensorPort) {
		m_motor = new VictorSPX(MotorPort);
		m_limitSwitch = new DigitalInput(SensorPort);
	}

	public void toggle () {
		if (hasBall()) {
			output();
		} else {
			intake();
		}
	}

	public void intake () {
		m_motor.set(ControlMode.PercentOutput, maxSpeed);
	}

	public void output () {
		m_motor.set(ControlMode.PercentOutput, -maxSpeed);
	}

	public void stop () {
		m_motor.set(ControlMode.PercentOutput, 0);
	}

	private void init() {
		m_motor.set(ControlMode.PercentOutput, 0);
	}
	private boolean hasBall() {
		return (m_limitSwitch.get());
	}
}