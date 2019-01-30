package org.huskyrobotics.frc2018.subsystems;
public class PID{
    private final double c_ProportionalConst = 0;
    private double m_IntegralGain = 0;
    private double m_DerivativeGain = 0;

    public PID(){
    }

   /**
    * Sets integral gain based on error
    * @param error the distance from the target
    */
    private void setIntegralGain(double error){
        m_IntegralGain = 0;
    }

    /**
     * Sets derivative gain based on error
     * @param error the distance from the target
     */
    private void setDerivativeGain(double error){
        m_DerivativeGain = 0;
    }

    /**
     * Gets the final velocity after PID calculation
     * @return the final velocity
     */
    public double getVelocity(){
        return 0;
    }


}