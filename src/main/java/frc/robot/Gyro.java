

public class Gyro {

    public Gyroscope(){

        //Pigeon connected to the can
        PigeonImu _pigeon = new PigeonImu(0); /* example Pigeon with device ID 0 */

        //connected to Talon SRX
        CANTalon _talon2 = new CANTalon(2); /* Talon SRX on CAN bus with device ID 2*/
        PigeonImu _pigeon = new PigeonImu(_talon2); /* Pigeon is plugged into Talon 2*/

        //
        PigeonImu.GeneralStatus genStatus = new PigeonImu.GeneralStatus();
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

