package frc.robot.Mechanisms;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class CatzDriveTrain
{
    // Motor Definitions
    public final int     DRVTRAIN_LT_FRNT_MC_CAN_ID    = 1;
    public final int     DRVTRAIN_LT_BACK_MC_CAN_ID    = 2;
    public final int     DRVTRAIN_RT_FRNT_MC_CAN_ID    = 3;
    public final int     DRVTRAIN_RT_BACK_MC_CAN_ID    = 4;

    public final int     DRVTRAIN_LT_FRNT_MC_PDP_PORT  = 14;
    public final int     DRVTRAIN_LT_BACK_MC_PDP_PORT  = 15;
    public final int     DRVTRAIN_RT_FRNT_MC_PDP_PORT  = 0;
    public final int     DRVTRAIN_RT_BACK_MC_PDP_PORT  = 1;

    private final int    CURRENT_LIMIT_AMPS            = 55;
    private final int    CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;

    private final boolean ENABLE_CURRENT_LIMIT         = true;

    public WPI_TalonFX drvTrainMtrCtrlLTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlLTBack;
    public WPI_TalonFX drvTrainMtrCtrlRTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlRTBack;

    private SupplyCurrentLimitConfiguration drvTrainCurrentLimit;

    // Drive Definitions
    public final int DRVTRAIN_LT = 0;
    public final int DRVTRAIN_RT = 1;

    private DifferentialDrive drvTrainDifferentialDrive;

    private MotorControllerGroup drvTrainLT;
    private MotorControllerGroup drvTrainRT;

    // Gearbox and Encoder Definitions
    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;

    private final int DRVTRAIN_GEARSHIFT_LO_PCM_PORT = 4;
    private final int DRVTRAIN_GEARSHIFT_HI_PCM_PORT = 3;

    public final int DRVTRAIN_GEAR_LO                = 0;
    public final int DRVTRAIN_GEAR_HI                = 1;

    private final double LOW_GEAR_RATIO              = 25.0 / 3.0;
    private final double HIGH_GEAR_RATIO             = 857.0 / 50.0;

    private final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV = 2048.0;
    private final double DRVTRAIN_WHEEL_RADIUS               = 2;
    private final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);

    public final double  DRVTRAIN_ENC_COUNTS_TO_INCHES_LO     = (1/LOW_GEAR_RATIO) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV);
    public final double  DRVTRAIN_ENC_COUNTS_TO_INCHES_HI     = (1/HIGH_GEAR_RATIO) * DRVTRAIN_WHEEL_CIRCUMFERENCE * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV);
    
    public double currentEncCountsToInches = 0.0;
    public int    currentDrvTrainGear;
    private DoubleSolenoid gearShiftSolenoid;

    // Slew rate
    private SlewRateLimiter filter;
    public boolean inSlewRate = false;
    private final double SLEW_RATE_FACTOR = 1.0;
    
    public CatzDriveTrain()
    {
        drvTrainMtrCtrlLTFrnt = new WPI_TalonFX(DRVTRAIN_LT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlLTBack = new WPI_TalonFX(DRVTRAIN_LT_BACK_MC_CAN_ID);
        drvTrainMtrCtrlRTFrnt = new WPI_TalonFX(DRVTRAIN_RT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlRTBack = new WPI_TalonFX(DRVTRAIN_RT_BACK_MC_CAN_ID);

        drvTrainMtrCtrlLTFrnt.configFactoryDefault();
        drvTrainMtrCtrlLTBack.configFactoryDefault();
        drvTrainMtrCtrlRTFrnt.configFactoryDefault();
        drvTrainMtrCtrlRTBack.configFactoryDefault();

        drvTrainCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        drvTrainMtrCtrlLTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlLTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        
        drvTrainMtrCtrlLTBack.follow(drvTrainMtrCtrlLTFrnt);
        drvTrainMtrCtrlRTBack.follow(drvTrainMtrCtrlRTFrnt);

        drvTrainMtrCtrlRTFrnt.setInverted(true);
        drvTrainMtrCtrlRTBack.setInverted(true);

        drvTrainLT = new MotorControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTBack);
        drvTrainRT = new MotorControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTBack);

        gearShiftSolenoid = new DoubleSolenoid(PCM_TYPE, DRVTRAIN_GEARSHIFT_LO_PCM_PORT, DRVTRAIN_GEARSHIFT_HI_PCM_PORT);
    }

    // drive methods
    public void arcadeDrive(double power, double rotation)
    {
        if(inSlewRate == true)
        {
            drvTrainDifferentialDrive.arcadeDrive(filter.calculate(-power), rotation );

        }
        else
        {
            drvTrainDifferentialDrive.arcadeDrive(-power, rotation);
        }

    }

    //----------------------------------------- Motor config and Status Methods --------------------------------------------------------------------------

    public void setToBrakeMode()
    {
        drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Brake);
    }

    public void setToCoastMode()
    {
        drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Coast);
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Coast);
        drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Coast);
        drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Coast);
    }

    // ----------------------------------------Gear shift methods---------------------------------------
   
    public void shiftToLowGear()
    {
        gearShiftSolenoid.set(Value.kForward);
        
        currentDrvTrainGear      = DRVTRAIN_GEAR_HI;
        currentEncCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_HI;
    } 

    public void shiftToHighGear()
    {
        gearShiftSolenoid.set(Value.kReverse);

        currentDrvTrainGear      = DRVTRAIN_GEAR_LO;
        currentEncCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_LO;
    }

}
