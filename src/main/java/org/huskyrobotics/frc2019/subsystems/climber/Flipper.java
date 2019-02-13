package org.huskyrobotics.frc2019.subsystems.climber;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.huskyrobotics.frc2019.subsystems.*;

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

    }

    public void doAuto() {

    }

    public void teleopInit() {

    }

    public void doTeleop() {

    }

    /**
     * Extends the clamps so they can clamp on.
     */
    public void extendArms() {

    }

    public class Clamp {
        private int m_channel;
        private boolean m_status;

        private Solenoid m_sol = new Solenoid(m_channel);

        public Clamp(int m_channel, boolean m_status) {
            this.m_channel = m_channel;
            this.m_status = m_status;
        }

        /**
         * Turns the piston either on or off.
         */
        public void setStatus() {
            // turns the output on, extending the piston. See
            // http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Solenoid.html#set(boolean)
            m_sol.set(m_status);
        }
    }

    /**
     * Clamps onto the platform so winch can pull robot up.
     */
    public void clamp() {
        // calls m_sol.set(m_status) in Clamp class extending/detracting the piston.
        c_clamp.setStatus();
    }
    // documentation for winch: http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_victor_s_p_x.html#a24abd61c6efb94078a83eacefd53b67d
    public class Winch {
        private int c_deviceNumber;
        private int c_winchMotorPort;

        VictorSPX m_moter = new VictorSPX(c_deviceNumber);
        public Winch(int c_winchMotorPort, int c_deviceNumber) {
            this.c_winchMotorPort = c_winchMotorPort;
            this.c_deviceNumber = c_deviceNumber;
        }

        /**
         * possible method to get current location of winch
         */
        public void getWinchLoc() {
        }

        /**
         * releases the winch rope
         */
        public void extendWinch() {
            m_moter.set(ControlMode.PercentOutput, 1.0);
        }

        /**
         * retracts the winch rope
         */
        public void retractWinch() {
            m_moter.set(ControlMode.PercentOutput, -1.0);
        }

        /**
         * stops the winch
         */
        public void stopWinch() {
            m_moter.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * Pulls the main part of the robot onto the platform.
     */
    public void winch() {
        c_winch.retractWinch();
        c_winch.stopWinch();
        c_winch.extendWinch();
        c_winch.getWinchLoc();
    }
}