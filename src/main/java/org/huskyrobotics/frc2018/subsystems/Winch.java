package org.huskyrobotics.frc2018.subsystems;

/**
 * controls the winch part of the flipper
 */
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
