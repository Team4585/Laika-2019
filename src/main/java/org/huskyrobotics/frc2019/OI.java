//this class is used to get driver input
package org.huskyrobotics.frc2019;

import edu.wpi.first.wpilibj.Joystick;

import java.util.HashMap;

public class OI {
  private int HELMSTICKPORT;
  private int WEAPONSTICKPORT;
  
  private Joystick m_HelmStick;
  private Joystick m_WeaponStick;
  
  private HashMap<String, Integer> controlsH = new HashMap<String, Integer>();//holds info for helm driver's mapping
  controlsH.put("RobotForward", 1);
  controlsH.put("RobotTwist", 0);
  
  private HashMap<String, Integer> controlsW = new HashMap<String, Integer>();//holds info for weapon driver's mapping
  controlsW.put("ArmAxis", 1);
  controlsW.put("CargoActivate", 0);
  controlsW.put("HatchPush", 1);
  controlsW.put("Climb", 2);
  controlsW.put("TeleopSwitch", 7);
  
  public OI (int khsp, int kwsp) {
    HELMSTICKPORT = khsp;
    WEAPONSTICKPORT = kwsp;
    m_HelmStick = new Joystick(HELMSTICKPORT);
    m_WeaponStick = new Joystick(WEAPONSTICKPORT);

    controlsH = new HashMap<String, Integer>();
    controlsH.put("RobotForward", 1);
    controlsH.put("RobotTwist", 0);
    controlsW = new HashMap<String, Integer>();
    controlsW.put("ArmAxis", 1);
    controlsW.put("CargoActivate", 0);
    controlsW.put("HatchPush", 1);
    controlsW.put("Climb", 2);
    controlsW.put("TeleopSwitch", 7);
  } 
  //These functions will be used by robot.java to get the input
  public double GetRobotForward () {//The value used for robot motors moving forward. Should be put into Drive
    return m_HelmStick.getRawAxis(controlsH.get("RobotForward"));
  }
  public double GetRobotTwist () {//The value used for robot motors twisting. Should be put into Drive
    return m_HelmStick.getRawAxis(controlsH.get("RobotTwist"));
  }
  public double GetArmAxis () {//The value used for moving the arm up and down. Should be put into PivotArm
    return m_WeaponStick.getRawAxis(controlsW.get("ArmAxis"));
  }
  public boolean GetCargoActivate () {//The value used for controlling the cargo intake. Should be put into CargoIO
    return m_WeaponStick.getRawButton(controlsW.get("ArmAxis"));
  }
  public boolean GetHatchPush () {//The value used for pushing out hatch panels. Should be put into HatchIO
    return m_WeaponStick.getRawButton(controlsW.get("HatchPush"));
  }
  public boolean GetClimb () {//The value used to activate climbing. Should be put into whatever the climbing thing is
    return m_WeaponStick.getRawButton(controlsW.get("Climb"));
  }
  public boolean GetTeleopSwitch () {//The value used to switch to teleop incase auto fails. Should be put somewhere? ¯\_(ツ)_/¯
    return m_WeaponStick.getRawButton(controlsW.get("TeleopSwitch"));
  }
}
