package org.huskyrobotics.frc2019.subsystems.hatch;

import org.huskyrobotics.frc2019.subsystems.*;

public class HatchIO implements HuskySubsystem {
    
	private NidecBrushless[] m_linearActuators;
    private double m_outSpeed = 1.0;
    private double m_inSpeed = 1.0;
    private boolean m_IsActive = false;

    public HatchIO(int[] actuatorPortPWMs, int[] actuatorPortDIOs) {
        m_linearActuator = {new NidecBrushless(actuatorPortPWM[0], actuatorPortDIO[0]), new NidecBrushless(actuatorPortPWM[10], actuatorPortDIO[1]), new NidecBrushless(actuatorPortPWM[2], actuatorPortDIO[2]), new NidecBrushless(actuatorPortPWM[3], actuatorPortDIO[3])};
    }
    
    public void release () {
        m_IsActive = true;
    }
    
	public void reset () {
        for (NidecBrushless actuator : m_linearActuators) { 
            actuator.reset();
        }
	}
	
    private void init () {
		m_IsActive = false;
		reset();
    }
    
    private void do () {
        for (NidecBrushless actuator : m_linearActuators) { 
            if (m_IsActive) {
                actuator.set(m_outSpeed);
            } else {
                actuator.set(m_inSpeed);
            }
        } 
    }
    
    @Override
    public void autoInit() {
        init();
    }

    @Override
    public void doAuto() {
        do();
    }

    @Override
    public void teleopInit() {
        init();
    }

    @Override
    public void doTeleop() {
        do();
    }
}
