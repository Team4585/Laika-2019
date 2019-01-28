package org.huskyrobotics.frc2019;


import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.smartdashboard.*;
public class Gyro {

    public Gyro(){
        //inits Pigeon
        PigeonIMU m_pigeon;
        TalonSRX m_talon2 = new TalonSRX(2); /* Talon SRX on CAN bus with device ID 2*/

        Boolean isPigeonOnCAN = true; //Just a clever way of checking whether or not we're connected to the second talon or the CAN bus

        if(isPigeonOnCAN){
            /* Pigeon is on CANBus (powered from ~12V, and has a device ID of zero) */
            m_pigeon = new PigeonIMU(4);             // Change ID accordingly
            SmartDashboard.putNumber("CAN Pigeon", m_pigeon.getDeviceID()); 
        }else{
            /* Pigeon is ribbon cabled to the specified CANTalon. */
            m_pigeon = new PigeonIMU(m_talon2);   // Change Talon Accordingly
            SmartDashboard.putNumber("Pigeon over Ribbon", m_pigeon.getDeviceID());
        }

        //
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        m_pigeon.getGeneralStatus(genStatus);

        //retrieve yaw
        double [] ypr = new double[3];
        m_pigeon.getYawPitchRoll(ypr);
        System.out.println("Yaw:" + ypr[0]);

        /*m_pigeon.SetYaw(newAngle);          //I'm not entirely sure where this came from on the documentation, so I'm commenting it out until I determine where newAngle came
        m_pigeon.SetFusedHeading(newAngle);*/ 

        //calibrating modes
        m_pigeon.enterCalibrationMode(CalibrationMode.Temperature);





/* this si what I originally did

        public double() {

        return
        }
        */
    }
}

