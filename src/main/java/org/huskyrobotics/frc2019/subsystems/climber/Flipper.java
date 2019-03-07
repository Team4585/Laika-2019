package org.huskyrobotics.frc2019.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Flipper {
    private TalonSRX m_winchMotor;
    private Solenoid m_sol;

    private boolean m_solActive;
    private boolean m_IsClampActive;

public class Flipper {

    // the channel of the solenoids(should be the same)
    private int c_channel = 0;
    private int c_winchMotorPort = 0;
    private int c_deviceNumber = 66666;
    // initially set to false untill winch is the correct radius
    private boolean m_status = false;
    private Clamp c_clamp = new Clamp(c_channel, m_status);
    private Winch c_winch = new Winch(c_winchMotorPort, c_deviceNumber);

    public void autoInit() {

    public static enum EncoderMode {
        None, QuadEncoder, CTRE_MagEncoder_Relative, CTRE_MagEncoder_Absolute
    }

    public Flipper (int winchMotorPort, int solenoidChannel, EncoderMode mode) {
        m_sol = new Solenoid(solenoidChannel);
        
        m_winchMotor = new TalonSRX(winchMotorPort);

        if(mode == EncoderMode.QuadEncoder) {
            m_winchMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
            m_winchMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 10);
        }
        //I set the peak output to 75% so we don't slam our robot into itself?
        m_winchMotor.configPeakOutputForward(+0.75, 10);
        m_winchMotor.configPeakOutputReverse(-0.75, 10);
    }
    //releases the winch rope
    public void setWinchAxis(double input) {
		if(Math.abs(input) > 0.1) {
			m_winchMotor.set(ControlMode.Position, input);
		} else {
			m_winchMotor.set(ControlMode.Position, 0);
		}
        
    }
    public void setIsClimbActive(boolean input) {
        clamp(input);
        m_IsClampActive = input;
        SmartDashboard.putBoolean("Is the clamp Active?", m_IsClampActive);
    }

    //Clamps onto the platform so winch can pull robot up.
    //True for clamped, false for released/
    public void clamp(boolean clamp) {
        if(clamp = true){
        m_sol.set(clamp);
        m_solActive = clamp;
        }else{
        m_solActive = false;
        }
    }

    public boolean getClamped() {
        return(m_solActive);
    }
    //stops the winch
    public void stopWinch() {
        m_winchMotor.setNeutralMode(NeutralMode.Coast);
        m_winchMotor.set(ControlMode.PercentOutput, 0.0);
    }
    public void setClosedLoopGains(double kp, double ki, double kd, double kf, double maxIntegral) {
        m_winchMotor.config_kP(0, kp, 10);
        m_winchMotor.config_kI(0, ki, 10);
        m_winchMotor.config_kD(0, kd, 10);
        m_winchMotor.config_kF(0, kf, 10);
        m_winchMotor.configMaxIntegralAccumulator(0, maxIntegral, 0);
      }
    

}