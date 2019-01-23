
package frc.robot.subsystems;

public class PivotArm extends HuskySubsystem
{
      private double TargetAngle;

      public PivotArm (int MotorPort, int SensorPort)
      {
             //set motor
             //set sensor
      }
  
      public void SetTarget (double angle)
      {
            TargetAngle = angle;
      }
  
      public void onDeactivate ()
      {
            SetTarget();
      }
      
      public void doAuto ()
      {
            //move to target position
      }
      public void doTeleop ()
      {
            //move to target position
      }
}
