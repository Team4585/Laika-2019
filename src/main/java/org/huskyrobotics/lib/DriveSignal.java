package org.huskyrobotics.lib;
/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

/**
 * A drivetrain command consisting of the left, right motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal {
    protected Velocity<Length> m_LeftMotor;
    protected Velocity<Length> m_RightMotor;
    protected double m_LeftPercent;
    protected double m_RightPercent;
    protected boolean m_BrakeMode;

    public DriveSignal(Velocity<Length> left, Velocity<Length> right) {
        this(left, right, false);
    }

    public DriveSignal(double leftPercent, double rightPercent) {
        this(leftPercent, rightPercent, false);
    }

    public DriveSignal(double leftPercent, double rightPercent, boolean mBrakeMode) {
        this(zeroSpeed, zeroSpeed, leftPercent, rightPercent, false);
    }

    public DriveSignal(Velocity<Length> left, Velocity<Length> right, boolean brakeMode) {
        this(left, right, 0, 0, false);
    }

    public DriveSignal(Velocity<Length> left, Velocity<Length> right, double leftPercent, double rightPercent, boolean brakeMode) {
        m_LeftMotor = left;
        m_RightMotor = right;
        m_LeftPercent = leftPercent;
        m_RightPercent = rightPercent;
        m_BrakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(0f)));
    public static DriveSignal BRAKE = new DriveSignal(VelocityKt.getVelocity(LengthKt.getFeet(0f)), VelocityKt.getVelocity(LengthKt.getFeet(0f)), true);
    private static final Velocity<Length> zeroSpeed = VelocityKt.getVelocity(LengthKt.getFeet(0f));

    public Velocity<Length> getLeft() {
        return m_LeftMotor;
    }

    public Velocity<Length> getRight() {
        return m_RightMotor;
    }

    public double getLeftPercent() {
        return m_LeftPercent;
    }

    public double getRightPercent() {
        return m_RightPercent;
    }

    public boolean getBrakeMode() {
        return m_BrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + m_LeftMotor + ", R: " + m_RightMotor + (m_BrakeMode ? ", BRAKE" : "");
    }
}