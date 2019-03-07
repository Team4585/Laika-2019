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
    controlsMic.put("HatchToggle", 1);
    controlsMic.put("ToggleClimb", 3);

    controlsAhn = new HashMap<String, Integer>();
    controlsAhn.put("ArmAxis", 2);
    controlsAhn.put("WinchAxis", 2);
    controlsAhn.put("CargoAxis", 0);
    controlsAhn.put("WinchIn", 0);
    controlsAhn.put("WinchOut", 0);
    controlsAhn.put("CargoIn", 3);
    controlsAhn.put("CargoOut", 4);
    controlsAhn.put("HatchToggle", 12);
    controlsAhn.put("ToggleClimb", 1);

    weaponsControls = new HashMap[] {controlsMic, controlsAhn};
  }

    controlsW.put("ArmAxis", 1);
    controlsW.put("CargoIn", 1);
    controlsW.put("CargoOut", 2);
    //controlsW.put("HatchPush", 1);
    controlsW.put("Climb", 3);
    controlsW.put("TeleopSwitch", 7);
    controlsW.put("ArmRotate", 5);
  } 

  //These functions will be used by robot.java to get the input
  public double getRobotForward () {//The value used for robot motors moving forward. Should be put into Drive
    return m_HelmStick.getRawAxis((int) helmControls[m_helm.getSelected()].get("RobotForward"));
  }
  public double getRobotTwist () {//The value used for robot motors twisting. Should be put into Drive
    return m_HelmStick.getRawAxis((int) helmControls[m_helm.getSelected()].get("RobotTwist"));
  }
  public boolean getTeleopSwitch () {//The value used to switch to teleop incase auto fails. Should be put somewhere? ¯\_(ツ)_/¯
    return m_HelmStick.getRawButton((int) helmControls[m_helm.getSelected()].get("TeleopSwitch"));
  }
  public boolean GetCargoIn () {//The value used for controlling the cargo intake. Should be put into CargoIO
    return m_WeaponStick.getRawButton(controlsW.get("CargoIn"));
  }
  public boolean GetCargoOut() {
    return m_WeaponStick.getRawButton(controlsW.get("CargoOut"));

  public double getArmAxis () {//The value used for moving the arm up and down. Should be put into PivotArm
    return m_WeaponStick.getRawAxis((int) weaponsControls[m_weapon.getSelected()].get("ArmAxis"));

  }
  public double getCargoAxis () {//The value used for controlling the cargo intake. Should be put into CargoIO
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
    return(0);
  }
  public double getWinchAxis () {
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
    return(0);
  }
  public boolean getHatchToggle () {//The value used for pushing out hatch panels. Should be put into HatchIO
    return m_WeaponStick.getRawButtonReleased((int) weaponsControls[m_weapon.getSelected()].get("HatchToggle"));
  }
  public boolean getRotate(){
    return m_WeaponStick.getRawButton(controlsW.get("ArmRotate"));
  }
}

