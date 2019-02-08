package org.huskyrobotics.lib.util;
/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double m_LeftMotor;
    protected double m_RightMotor;
    protected boolean m_BrakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        m_LeftMotor = left;
        m_RightMotor = right;
        m_BrakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return m_LeftMotor;
    }

    public double getRight() {
        return m_RightMotor;
    }

    public boolean getBrakeMode() {
        return m_BrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + m_LeftMotor + ", R: " + m_RightMotor + (m_BrakeMode ? ", BRAKE" : "");
    }
}