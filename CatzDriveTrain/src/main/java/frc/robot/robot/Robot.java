package frc.robot.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Mechanisms.CatzDriveTrain;

public class Robot extends TimedRobot 
{
  public static CatzDriveTrain driveTrain;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

  public static PowerDistribution pdp;

  double left;
  double right;

  @Override
  public void robotInit() 
  {
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    pdp = new PowerDistribution();

    driveTrain = new CatzDriveTrain();

  }

  @Override
  public void robotPeriodic() 
  {

  }

  @Override
  public void autonomousInit() 
  {

  }

  @Override
  public void autonomousPeriodic() 
  {

  }

  @Override
  public void teleopInit() 
  {
    driveTrain.setToBrakeMode();

    if(xboxAux.getRightBumper())
    {
      driveTrain.arcadeDrive((xboxAux.getLeftY()), xboxAux.getRightX());
    }
    else
    {
      left = xboxDrv.getLeftY();
      right = xboxDrv.getRightX();

      driveTrain.arcadeDrive(left, right);
    }

    if (xboxDrv.getLeftBumper()) 
    {
      driveTrain.shiftToHighGear();
    } 
    else if (xboxDrv.getRightBumper()) 
    {
      driveTrain.shiftToLowGear();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() 
  {
    driveTrain.setToCoastMode();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
