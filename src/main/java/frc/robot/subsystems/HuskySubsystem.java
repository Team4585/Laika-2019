package frc.team4585.robot;

public interface HuskySubsystem {
	
	private bool Active;
	
	public void setActive(bool a)
	{
		Active = a;
		if (a == false)
		{
			onDeactivate();
		}
	}
	
	public void onDeactivate();
	
	public void autoInit();
	
	public void doAuto();
	
	public void teleopInit();
	
	public void doTeleop();
	
}
