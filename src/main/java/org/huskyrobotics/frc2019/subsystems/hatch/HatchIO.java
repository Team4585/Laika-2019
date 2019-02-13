package org.huskyrobotics.frc2019.subsystems.hatch;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class HatchIO {
    private VictorSPX[] m_linearActuators;
    private double m_outSpeed = 1.0;
    private double m_inSpeed = 1.0;

    public HatchIO(int[] ActuatorPorts) {
        m_linearActuators = new VictorSPX[] {new VictorSPX(ActuatorPorts[0]), new VictorSPX(ActuatorPorts[1]), new VictorSPX(ActuatorPorts[2]), new VictorSPX(ActuatorPorts[3])};
    }

    public void setHatchPush (boolean input) {
        if(input) {
            release();
        } else {
            reset();
        }
    }

    public void release () {
        SetActuatorsOut();
    }
    public void reset () {
        SetActuatorsIn();
    }

    private void init () {
		ResetActuators();
    }

    private void SetActuatorsIn() {
        for (VictorSPX actuator : m_linearActuators) { 
            actuator.set(ControlMode.PercentOutput, m_inSpeed);
        } 
    }
    private void SetActuatorsOut() {
        for (VictorSPX actuator : m_linearActuators) { 
            actuator.set(ControlMode.PercentOutput, m_outSpeed);
        } 
    }
    private void ResetActuators() {
        for (VictorSPX actuator : m_linearActuators) { 
            actuator.set(ControlMode.PercentOutput, 0);
        } 
    }
}
