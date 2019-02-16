package org.huskyrobotics.frc2019.subsystems.cargo;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class CargoIO {
	private VictorSPX m_motor;

	private double maxSpeed = 1.0;

	//private SomeSensor _CheckSensor;
	public CargoIO (int MotorPort) {
		m_motor = new VictorSPX(MotorPort);
	}

	/**
	 * Sets the speed of the rollers based on the user input
	 * @param input user-controlled input
	 */
	public void setCargoAxis (double input) {
		if(Math.abs(input) > 0.1) {
			m_motor.set(ControlMode.PercentOutput, input);
		} else {
			m_motor.set(ControlMode.PercentOutput, 0);
		}
	}

	/**
	 * Sets the intake speed to max
	 */
	public void intake () {
		m_motor.set(ControlMode.PercentOutput, maxSpeed);
	}

	/**
	 * Sets the output speed to max
	 */
	public void output () {
		m_motor.set(ControlMode.PercentOutput, -maxSpeed);
	}

	/**
	 * Completly stops the rollers
	 */
	public void stop () {
		m_motor.set(ControlMode.PercentOutput, 0);
	}


	private void init() {
		m_motor.set(ControlMode.PercentOutput, 0);
	}
}
