package org.huskyrobotics.lib.drivers;
//254 Talon Maintenance that is :ok_hand:
import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonSRXUtil {
    // Checks the specified error code for issues.
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}
