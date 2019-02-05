
package org.huskyrobotics.frc2019.subsystems.superstructure;

import org.huskyrobotics.frc2019.subsystems.*;

public class PivotArm implements HuskySubsystem
{
      private double TargetAngle;
      private double CurrentAngle;

      private int m_SensorPort = 0;
      private double m_MotorPort = 0;

      public PivotArm(int MotorPort, int SensorPort) {
            m_SensorPort = SensorPort;
            m_MotorPort = MotorPort;
      }

      /**
       * @return the currentAngle
       */
      public double getCurrentAngle() {
            return CurrentAngle;
      }

      public void SetTarget(double angle) {
            TargetAngle = angle;
      }
  
      public void onDeactivate () {
            SetTarget(0);
      }
      
      public void doAuto () {
            doTask();
      }
      public void doTeleop () {
            doTask();
      }
      private void doTask () {
            //calculate PID and go to target position
      }

      public void autoInit() {

      }

      public void teleopInit() {

      }
}
