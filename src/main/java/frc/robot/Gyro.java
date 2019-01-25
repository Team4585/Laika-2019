package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.*;
public class Gyro {

    public Gyro(){

        //Pigeon connected to the can
        PigeonIMU _pigeon = new PigeonIMU(0); /* example Pigeon with device ID 0 */

        //connected to Talon SRX
        TalonSRX _talon2 = new TalonSRX(2); /* Talon SRX on CAN bus with device ID 2*/
        PigeonIMU _pigeon = new PigeonIMU(_talon2); /* Pigeon is plugged into Talon 2*/

        //
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        _pigeon.GetGeneralStatus(genStatus);

        //retrieve yaw
        double [] ypr = new double[3];
        _pigeon.GetYawPitchRoll(ypr);
        System.out.println("Yaw:" + ypr[0]);

        _pigeon.SetYaw(newAngle);
        _pigeon.SetFusedHeading(newAngle);

        //calibrating modes
        _pigeon.EnterCalibrationMode(CalibrationMode.Temperature);





/* this si what I originally did

        public double() {

        return
*/
        }
    }
}

