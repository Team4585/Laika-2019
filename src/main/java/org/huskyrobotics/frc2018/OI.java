/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.huskyrobotics.frc2018;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;

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
  
  private Joystick m_HelmStick;
  private Joystick m_WeaponStick;
  
  private boolean isCargoActive;
  
  public OI ()
  {
    m_HelmStick = new Joystick(HELMSTICKPORT);
    m_WeaponStick = new Joystick(WEAPONSTICKPORT);
  } 
  //These functions will be used by the 
  public boolean GetCargoActive ()
  {
    if (m_WeaponStick.getTriggerPressed()) 
    {
      isCargoActive = !isCargoActive;
    }
    return (isCargoActive);
  }
  public double GetCargoAxis ()
  {
    if (isCargoActive)
    {
      return (m_WeaponStick.getY(GenericHID.Hand.kRight));
    }
    return (0);
  }
  public double GetPanelAxis ()
  {
    if (!isCargoActive)
    {
      return (m_WeaponStick.getY(GenericHID.Hand.kRight));
    }
    return (0);
  }
  public double GetRobotForward ()
  {
    return (m_HelmStick.getY(GenericHID.Hand.kRight));
  }
  public double GetRobotTwist ()
  {
    return (m_HelmStick.getTwist());
  }
  public boolean GetCargoToggle ()
  {
    return m_WeaponStick.getRawButton(5);
  }
  public boolean GetHatchPush ()
  {
    return m_WeaponStick.getRawButton(6);
  }
  public boolean GetClimb ()
  {
    return m_WeaponStick.getRawButton(12);
  }
  public boolean GetTelevisedSwitch ()
  {
    return m_WeaponStick.getRawButton(11);
  }
}
