package org.huskyrobotics.frc2019;
import edu.wpi.first.wpilibj.Joystick;
import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
  private Joystick m_HelmStick;
  private Joystick m_WeaponStick;

  private HashMap[] helmControls;
  private HashMap<String, Integer> controlsHar;//holds info for Harold's driver mapping
  private HashMap<String, Integer> controlsMan;//holds info for Manvith's driver mapping

  private HashMap[] weaponsControls;
  private HashMap<String, Integer> controlsMic;//holds info for Micaela's driver mapping
  private HashMap<String, Integer> controlsAhn;//holds info for Ahnaff's driver mapping

  SendableChooser<Integer> m_helm = new SendableChooser<>();
  SendableChooser<Integer> m_weapon = new SendableChooser<>();

  private boolean isClimbActive = false;
  
  public OI (int HELMSTICKPORT, int WEAPONSTICKPORT) {
    m_HelmStick = new Joystick(HELMSTICKPORT);
    m_WeaponStick = new Joystick(WEAPONSTICKPORT);

    m_helm.addOption("Harold", 0);
    m_helm.addOption("Manvith", 1);

    m_weapon.addOption("Micaela", 0);
    m_weapon.addOption("Ahnaff", 1);

    SmartDashboard.putData("Helm Driver", m_helm);
    SmartDashboard.putData("Weapons Officer", m_weapon);

    controlsHar = new HashMap<String, Integer>();
    controlsHar.put("RobotForward", 1);
    controlsHar.put("RobotTwist", 0);
    controlsHar.put("TeleopSwitch", 8);

    controlsMan = new HashMap<String, Integer>();
    controlsMan.put("RobotForward", 1);
    controlsMan.put("RobotTwist", 0);
    controlsMan.put("TeleopSwitch", 8);

    helmControls = new HashMap[] {controlsHar, controlsMan};

    controlsMic = new HashMap<String, Integer>();
    controlsMic.put("ArmAxis", 5);
    controlsMic.put("WinchAxis", 0);
    controlsMic.put("CargoAxis", 3);
    controlsMic.put("WinchIn", 4);
    controlsMic.put("WinchOut", 2);
    controlsMic.put("CargoIn", 0);
    controlsMic.put("CargoOut", 0);
    controlsMic.put("HatchPush", 1);
    controlsMic.put("ToggleClimb", 3);

    controlsAhn = new HashMap<String, Integer>();
    controlsAhn.put("ArmAxis", 2);
    controlsAhn.put("WinchAxis", 2);
    controlsAhn.put("CargoAxis", 0);
    controlsAhn.put("WinchIn", 0);
    controlsAhn.put("WinchOut", 0);
    controlsAhn.put("CargoIn", 3);
    controlsAhn.put("CargoOut", 4);
    controlsAhn.put("HatchPush", 12);
    controlsAhn.put("ToggleClimb", 1);

    weaponsControls = new HashMap[] {controlsMic, controlsAhn};
  }
  public void periodic () {
    isClimbActive = m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("ToggleClimb"));
  }

  //These functions will be used by robot.java to get the input
  public double GetRobotForward () {//The value used for robot motors moving forward. Should be put into Drive
    return m_HelmStick.getRawAxis((int) helmControls[m_helm.getSelected()].get("RobotForward"));
  }
  public double GetRobotTwist () {//The value used for robot motors twisting. Should be put into Drive
    return m_HelmStick.getRawAxis((int) helmControls[m_helm.getSelected()].get("RobotTwist"));
  }
  public boolean GetTeleopSwitch () {//The value used to switch to teleop incase auto fails. Should be put somewhere? ¯\_(ツ)_/¯
    return m_HelmStick.getRawButton((int) helmControls[m_helm.getSelected()].get("TeleopSwitch"));
  }


  public double GetArmAxis () {//The value used for moving the arm up and down. Should be put into PivotArm
    return m_WeaponStick.getRawAxis((int) weaponsControls[m_weapon.getSelected()].get("ArmAxis"));
  }
  public double GetCargoAxis () {//The value used for controlling the cargo intake. Should be put into CargoIO
    if(!isClimbActive) {
      if ((int) weaponsControls[m_weapon.getSelected()].get("CargoIn") > 0) {
        if (m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("CargoIn"))) {
          return (0.5);
        } else if (m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("CargoOut"))) {
          return (-0.5);
        } else {
          return (0);
        }
      }
      if ((int) weaponsControls[m_weapon.getSelected()].get("CargoAxis") > 0) {
        return (m_WeaponStick.getRawAxis((int) weaponsControls[m_weapon.getSelected()].get("CargoAxis")));
      }
    }
    return(0);
  }
  public double GetWinchAxis () {
    if(isClimbActive) {
      if ((int) weaponsControls[m_weapon.getSelected()].get("WinchIn") > 0) {
        if (m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("WinchIn"))) {
          return (0.5);
        } else if (m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("WinchOut"))) {
          return (-0.5);
        } else {
          return (0);
        }
      }
      if ((int) weaponsControls[m_weapon.getSelected()].get("WinchAxis") > 0) {
        return (m_WeaponStick.getRawAxis((int) weaponsControls[m_weapon.getSelected()].get("WinchAxis")));
      }
    }
    return(0);
  }
  public boolean GetHatchPush () {//The value used for pushing out hatch panels. Should be put into HatchIO
    return m_WeaponStick.getRawButton((int) weaponsControls[m_weapon.getSelected()].get("HatchPush"));
  }
  public boolean GetIsClimbActive () {//The value used to activate the clamps. Should be put into Flipper and PivotArm
    return(isClimbActive);
  }
}

