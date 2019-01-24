/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;

public class CargoIO implements HuskySubsystem
{
	private Encoder _IntakeEncoder;

	//private SomeSensor _CheckSensor;
	public CargoIO (int MotorPort, int SensorPort)
	{
		//set encoder
		//set sensor
	}

	public void onDeactivate ()
	{
		_IntakeEncoder.reset();
	}

	public void Intake ()
	{
		//Set encoder
	}
	public void Outtake ()
	{
		//set encoder
	}
	public void Toggle ()
	{
		if (/*Sensor senses ball*/)
		{
			  Outtake ();
		}
		else
		{
			  Intake ();
		}
	}
}
