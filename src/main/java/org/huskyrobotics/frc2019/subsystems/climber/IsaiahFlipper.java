package org.huskyrobotics.frc2019.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IsaiahFlipper {
    private TalonSRX m_winchMotor;
    private Solenoid m_sol;

    private boolean m_solActive;

    private boolean m_controlActive = false;

    public IsaiahFlipper (int winchMotorPort, int solenoidChannel) {
        m_winchMotor = new TalonSRX(winchMotorPort);
        m_sol = new Solenoid(solenoidChannel);
    }
    //releases the winch rope
    public void setWinchAxis(double input) {
        if(m_controlActive) {
            if(Math.abs(input) > 0.1) {
                m_winchMotor.set(ControlMode.PercentOutput, input);
            } else {
                m_winchMotor.set(ControlMode.PercentOutput, 0);
            }
        }
    }
	public void setActive (boolean input) {
		m_controlActive = !input;
	}
    public void periodic() {
        if(m_controlActive) {
            clamp(true);
        }
        m_sol.set(m_solActive);
    }

    //Clamps onto the platform so winch can pull robot up.
    //True for clamped, false for released/
    public void clamp(boolean clamp) {
        m_solActive = clamp;
    }
    public void toggleClamp() {
        m_solActive = !m_solActive;
    }

    public boolean getClamped() {
        return(m_solActive);
    }
    //stops the winch
    public void stopWinch() {
        m_winchMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
