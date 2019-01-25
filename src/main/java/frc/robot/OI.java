/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class OI {
  private int HELMSTICKPORT;
  private int WEAPONSTICKPORT;
  
  /*
  Move Forward/Backward           | Axis 1                         | Helm           | Done
  Turn Right/Left                 | Twist                          | Helm           | Done
  Move Cargo Grabber Up/Down      | Axis 1                         | Weapons        | Done
  Move Panel Grabber Up/Down      | Axis 1                         | Weapons        | Done
  Switch from Cargo to Panel      | Trigger                        | Weapons        | Done
  Start/Stop Cargo Intake         | Button 5                       | Weapons        | Done
  Push Hatch Panels               | Button 6                       | Weapons        | Done
  Start Climb                     | Button 12                      | Weapons        | Done
  Switch To Televised             | Button 11                      | Helm/Weapons   | Done
  */
  
  private Joystick _HelmStick;
  private Joystick _WeaponStick;
  
  private boolean isCargoActive;
  
  public OI ()
  {
    _HelmStick = new Joystick(HELMSTICKPORT);
    _WeaponStick = new Joystick(WEAPONSTICKPORT);
  } 
  //These functions will be used by the 
  public boolean GetCargoActive ()
  {
    if (_WeaponStick.getTriggerPressed()) 
    {
      isCargoActive = !isCargoActive;
    }
    return (isCargoActive);
  }
  public double GetCargoAxis ()
  {
    if (isCargoActive)
    {
      return (_WeaponStick.getY(GenericHID.Hand.kRight));
    }
    return (0);
  }
  public double GetPanelAxis ()
  {
    if (!isCargoActive)
    {
      return (_WeaponStick.getY(GenericHID.Hand.kRight));
    }
    return (0);
  }
  public double GetRobotForward ()
  {
    return (_HelmStick.getY(GenericHID.Hand.kRight));
  }
  public double GetRobotTwist ()
  {
    return (_HelmStick.getTwist());
  }
  public boolean GetCargoToggle ()
  {
    return _WeaponStick.getRawButton(5);
  }
  public boolean GetHatchPush ()
  {
    return _WeaponStick.getRawButton(6);
  }
  public boolean GetClimb ()
  {
    return _WeaponStick.getRawButton(12);
  }
  public boolean GetTelevisedSwitch ()
  {
    return _WeaponStick.getRawButton(11);
  }
}
