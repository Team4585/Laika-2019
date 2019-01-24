
package frc.robot.subsystems;

public class PivotArm implements HuskySubsystem
{
      private double TargetAngle;
      private double CurrentAngle;

      public PivotArm (int MotorPort, int SensorPort) {
             //set motor
             //set sensor
      }
  
      public void SetTarget (double angle) {
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
