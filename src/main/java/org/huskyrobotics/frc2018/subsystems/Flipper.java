package org.huskyrobotics.frc2018.subsystems;
import edu.wpi.first.wpilibj.Solenoid;

public class Flipper implements HuskySubsystem {

    // the channel of the solenoids(should be the same)
    private int m_channel = 0;
    // initially set to false untill winch is the correct radius
    private boolean m_status = false;
    private Clamp clamp = new Clamp(m_channel, m_status);
    private Winch winch = new Winch();

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
        clamp.setStatus();
    }

    public class Winch {

        private int winchMotorPort; // motor port variable
        private double m_currentLoc;
        private boolean m_status;

        /**
         * possible method to get current location of winch
         */
        public String getWinchLoc() {
            return null;
        }

        /**
         * releases the winch rope
         */
        public void extendWinch() {
        }

        /**
         * retracts the winch rope
         */
        public void retractWinch() {
        }
    }

    /**
     * Pulls the main part of the robot onto the platform.
     */
    public void winch() {

    }
}
