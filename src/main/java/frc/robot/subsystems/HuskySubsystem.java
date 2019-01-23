package frc.team4585.robot;

public interface HuskySubsystem {
	
	public bool Active;
	
	public void onDeactivate();
	
	public void autoInit();
	
	public void doAuto();
	
	public void teleopInit();
	
	public void doTeleop();
	
}
