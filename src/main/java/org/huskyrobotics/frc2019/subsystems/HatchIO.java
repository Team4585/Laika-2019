package org.huskyrobotics.frc2019.subsystems;

public class HatchIO implements HuskySubsystem {

    public int HatchMotorPort, HatchSensorPort;

    public HatchIO(int HatchMotorPort, int HatchSensorPort) {
        this.HatchMotorPort = HatchMotorPort;
        this.HatchSensorPort = HatchSensorPort;
    }

    @Override
    public void autoInit() {

    }

    @Override
    public void doAuto() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void doTeleop() {

    }

    public void release() {
        
    }

}