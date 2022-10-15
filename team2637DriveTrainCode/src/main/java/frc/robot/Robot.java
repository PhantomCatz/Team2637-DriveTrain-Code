package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.*;


public class Robot extends TimedRobot 
{
  public static CatzDriveTrain driveTrain;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static PowerDistribution pdp;

  double left;
  double right;

  boolean firstTele = true;

  @Override
  public void robotInit() 
  {

    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    pdp = new PowerDistribution();

    driveTrain = new CatzDriveTrain();

  }
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() 
  {
    driveTrain.setToBrakeMode();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {

    if(firstTele == true)
    {
      driveTrain.instantiateDifferentialDrive();
      firstTele = false;
    }
    
    driveTrain.setToBrakeMode();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
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

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() 
  {
    driveTrain.setToCoastMode();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
