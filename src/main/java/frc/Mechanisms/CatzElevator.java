package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.*;
import frc.robot.Robot;

public class CatzElevator {
    private Thread elevatorThread;

    private int elevatorState;
    private final static int ELEVATOR_STATE_IDLE               = 0;
    private final static int ELEVATOR_STATE_ELEVATOR_SCORE_TOP = 1;
    private final static int ELEVATOR_STATE_ELEVATOR_SCORE_MID = 2;
    private final static int ELEVATOR_STATE_ELEVATOR_SCORE_LOW = 3;
    private final static int ELEVATOR_STATE_ELEVATOR_STOW      = 4;
    
    /*----------------------------------------------------------------------------------------------
    * 
    *  Pivot Definitions
    *
    *---------------------------------------------------------------------------------------------*/
    private WPI_TalonFX elevatorPivotMtrLT;
    private WPI_TalonFX elevatorPivotMtrRT;

    private final int ELEVATOR_PIVOT_LT_MC_ID  = 22;
    private final int ELEVATOR_PIVOT_RT_MC_ID  = 20;

    private static boolean elevatorPivotDone = false;

    private static boolean elevatorPivotManualMode = false;

    private SupplyCurrentLimitConfiguration pivotMtrCurrentLimit;
    private final double  ELEVATOR_PIVOT_CURRENT_LIMIT                 = 60.0;
    private final double  ELEVATOR_PIVOT_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  ELEVATOR_PIVOT_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ELEVATOR_PIVOT_ENABLE_CURRENT_LIMIT          = true;

    private final double ELEVATOR_PIVOT_VERSA_STAGE_1_RATIO = 3.0;
    private final double ELEVATOR_PIVOT_VERSA_STAGE_2_RATIO = 10.0;
    private final double ELEVATOR_PIVOT_VERSA_FINAL_RATIO   = ELEVATOR_PIVOT_VERSA_STAGE_1_RATIO * ELEVATOR_PIVOT_VERSA_STAGE_2_RATIO;

    private final double ELEVATOR_PIVOT_PINION_GEAR = 16.0;    //TBD - Moving to chain
    private final double ELEVATOR_PIVOT_SPUR_GEAR   = 84.0;
    private final double ELEVATOR_PIVOT_GEAR_RATIO  = ELEVATOR_PIVOT_SPUR_GEAR / ELEVATOR_PIVOT_PINION_GEAR;

    private final double ELEVATOR_PIVOT_FINAL_RATIO   = ELEVATOR_PIVOT_VERSA_FINAL_RATIO * ELEVATOR_PIVOT_GEAR_RATIO;
    private final double ELEVATOR_PIVOT_CNTS_TO_DEGREES = 360.0 / 2048.0;   //TBD Finish; add gear ratio;  Should be CNTS_PER_DEG

    private final double ELEV_PIVOT_SCORE_ANGLE_HIGH_DEG = 40.0;
    private final double ELEV_PIVOT_SCORE_ANGLE_MID_DEG  = 40.0;
    private final double ELEV_PIVOT_SCORE_ANGLE_LOW_DEG  = 40.0;

    private final double ELEV_PIVOT_SCORE_ANGLE_HIGH_CNTS = ELEV_PIVOT_SCORE_ANGLE_HIGH_DEG * ELEVATOR_PIVOT_CNTS_TO_DEGREES; //TBD - define by angle; add units to name

    private final double ELEVATOR_PIVOT_SCORE_HIGH_POS = 2083.0; //to be fixed
    private final double ELEVATOR_PIVOT_SCORE_MID_POS  = 2160.0; //to be fixed
    private final double ELEVATOR_PIVOT_SCORE_LOW_POS  = 2150.0; //to be fixed
    private final double ELEVATOR_PIVOT_STOWED_POS     = 2568.0;

    private final int ELEVATOR_PIVOT_ENCODER_CAN_ID = 9;

    private CANCoder pivotEnc;
    private DigitalInput MagEncPWMInput;    //TBD

    private static double pivotTargetAngle  = 0.0;
    private static double pivotCurrentAngle = 0.0; //TBD How should we handle initialization?

    private double pivotDistanceToTarget = pivotTargetAngle - pivotCurrentAngle; //TBD is this init necessary?

    static public double PID_ELEVATOR_UP_EMPTY_KP   = 1.25; // needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_EMPTY_KI   = 0.00; // needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_EMPTY_KD   = 0.00; // needs to be fixed to certain value

    static public double PID_ELEVATOR_DOWN_EMPTY_KP = 1.25; // needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_EMPTY_KI = 0.00; // needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_EMPTY_KD = 0.00; // needs to be fixed to certain value

    static public double PID_ELEVATOR_UP_CONE_KP    = 1.25; // needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_CONE_KI    = 0.00; // needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_CONE_KD    = 0.00; // needs to be fixed to certain value

    static public double PID_ELEVATOR_DOWN_CONE_KP  = 1.25; // needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_CONE_KI  = 0.00; // needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_CONE_KD  = 0.00; // needs to be fixed to certain value

    private boolean indexingCone = false;

    private final int ELEVATOR_PIVOT_SCORE_LOW_CMD  = 0;
    private final int ELEVATOR_PIVOT_SCORE_MID_CMD  = 1;
    private final int ELEVATOR_PIVOT_SCORE_HIGH_CMD = 2;
    private final int ELEVATOR_PIVOT_STOWED_CMD     = 3;

    private int currentPivotCmd = ELEVATOR_PIVOT_STOWED_CMD;    //to be fixed

    private double wheelOffset;    //TBD

    public double testMaxPower;    //TBD - used without init?  Can we delete?

    /*----------------------------------------------------------------------------------------------
    * 
    *  Extend/Retract Definitions
    *
    *---------------------------------------------------------------------------------------------*/
    public static CANSparkMax elevatorSpoolMC;

    private final int ELEVATOR_SPOOL_MC_CAN_ID        = 21;

    private final int ELEVATOR_SPOOL_MC_CURRENT_LIMIT = 80; //TBD

    private final double ELEVATOR_SPOOL_MTR_STOP =  0.0;

    private SparkMaxLimitSwitch elevatorLimitSwitchResetBottom;
    private SparkMaxLimitSwitch elevatorLimitSwitchScoringTop;
    private DigitalInput        elevatorLimitSwitchScoringMid;

    private final int ELEVATOR_LIMIT_SWITCH_MID_DIO_PORT = 5; // TBD

    private final double   ELEV_SPOOL_VERSA_STAGE_1_RATIO = 4.0;
    private final double   ELEV_SPOOL_VERSA_STAGE_2_RATIO = 5.0;
    private final double   ELEV_SPOOL_VERSA_RATIO         = ELEV_SPOOL_VERSA_STAGE_2_RATIO / ELEV_SPOOL_VERSA_STAGE_1_RATIO;

    private final double   ELEV_SPOOL_SPROCKET_1          = 16.0;     //TBD - Confirm
    private final double   ELEV_SPOOL_SPROCKET_2          = 22.0;     //TBD - Confirm
    private final double   ELEV_SPOOL_SPROCKET_RATIO      = ELEV_SPOOL_SPROCKET_1 / ELEV_SPOOL_SPROCKET_2;

    private final double   ELEV_SPOOL_FINAL_RATIO         = ELEV_SPOOL_VERSA_RATIO * ELEV_SPOOL_SPROCKET_RATIO;

    private final double ELEVATOR_SPOOL_GEAR_1_SIZE   = 12.0;     //Neo pinin TBD - confirm size
    private final double ELEVATOR_SPOOL_GEAR_2_SIZE   = 30.0;     //on spool shaft TBD - confirm size
    private final double ELEVATOR_SPOOL_GEAR_RATIO    = ELEVATOR_SPOOL_GEAR_2_SIZE / ELEVATOR_SPOOL_GEAR_1_SIZE;
    private final double ELEVATOR_SPOOL_DIAMETER      = 1.25;     //check diameter/radius
    private final double ELEVATOR_SPOOL_CIRCUMFERENCE = Math.PI * ELEVATOR_SPOOL_DIAMETER;
    private final double NEO_CNTS_PER_REVOLUTION  = 42.0;
    private final double ELEVATOR_SPOOL_CNT_TO_INCHES = ELEVATOR_SPOOL_GEAR_RATIO * ELEVATOR_SPOOL_CIRCUMFERENCE * (1/NEO_CNTS_PER_REVOLUTION);//test later 
    //private final double ELEVATOR_SPOOL_CNT_TO_INCHES = ELEV_SPOOL_FINAL_RATIO * ELEV_SPOOL_CIRCUMFERENCE * (1.0 / NEO_CNTS_PER_REVOLUTION);  //TBD INCORRECT CHK UNITS 

    private SparkMaxPIDController elevatorSpoolPid;
    private final double PID_ELEVATOR_SPOOL_KP = 0.25;//TBD
    private final double PID_ELEVATOR_SPOOL_KI = 0.00;
    private final double PID_ELEVATOR_SPOOL_KD = 0.00;

    private double ELEVATOR_SPOOL_TOP_MAX_RANGE = 75.0;    // Inches
    private double ELEVATOR_SPOOL_BOT_MAX_RANGE = 0.0;     // Inches

    private static RelativeEncoder elevatorSpoolEncoder;

    private final double ELEVATOR_SPOOL_ENCODER_STARTING_POS = 0.0;

    private final double ELEVATOR_SPOOL_SCORE_TOP_POS = 74.0;   // Inches
    private final double ELEVATOR_SPOOL_SCORE_MID_POS = 50.0;   // Inches
    private final double ELEVATOR_SPOOL_SCORE_LOW_POS = 26.0;   // Inches
    private final double ELEVATOR_SPOOL_STOWED_POS    = 40.0;   // Inches

    private static boolean elevatorSpoolDone = false;

    private static boolean elevatorSpoolManualMode = false;

    private double elevatorCarriagePosInch;


    
    public CatzElevator() 
    {
        /************************************************************************************
         * spool
         ************************************************************************************/
        elevatorSpoolMC = new CANSparkMax(ELEVATOR_SPOOL_MC_CAN_ID, MotorType.kBrushless);
        elevatorSpoolMC.restoreFactoryDefaults();

        elevatorSpoolMC.setSmartCurrentLimit(ELEVATOR_SPOOL_MC_CURRENT_LIMIT);

        elevatorLimitSwitchResetBottom  = elevatorSpoolMC.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        elevatorLimitSwitchScoringTop = elevatorSpoolMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        

        elevatorLimitSwitchScoringMid = new DigitalInput(ELEVATOR_LIMIT_SWITCH_MID_DIO_PORT);

        elevatorSpoolEncoder = elevatorSpoolMC.getEncoder();
        elevatorSpoolEncoder.setPositionConversionFactor(ELEVATOR_SPOOL_CNT_TO_INCHES);
        elevatorSpoolEncoder.setPosition(ELEVATOR_SPOOL_ENCODER_STARTING_POS);

        elevatorSpoolPid = elevatorSpoolMC.getPIDController();

        elevatorSpoolPid.setP(PID_ELEVATOR_SPOOL_KP);
        elevatorSpoolPid.setI(PID_ELEVATOR_SPOOL_KI);
        elevatorSpoolPid.setD(PID_ELEVATOR_SPOOL_KD);



        /************************************************************************************
         * pivot
         ************************************************************************************/
        elevatorPivotMtrLT = new WPI_TalonFX(ELEVATOR_PIVOT_LT_MC_ID);
        elevatorPivotMtrRT = new WPI_TalonFX(ELEVATOR_PIVOT_RT_MC_ID);
        
       // elevatorPivotMtrLT.configFactoryDefault();
        //elevatorPivotMtrRT.configFactoryDefault();

        elevatorPivotMtrRT.follow(elevatorPivotMtrLT);
        elevatorPivotMtrRT.setInverted(true);

        pivotMtrCurrentLimit = new SupplyCurrentLimitConfiguration(ELEVATOR_PIVOT_ENABLE_CURRENT_LIMIT, ELEVATOR_PIVOT_CURRENT_LIMIT,
                ELEVATOR_PIVOT_CURRENT_LIMIT_TRIGGER_AMPS, ELEVATOR_PIVOT_CURRENT_LIMIT_TIMEOUT_SECONDS);
        elevatorPivotMtrLT.configSupplyCurrentLimit(pivotMtrCurrentLimit);
        elevatorPivotMtrRT.configSupplyCurrentLimit(pivotMtrCurrentLimit);

        pivotEnc = new CANCoder(ELEVATOR_PIVOT_ENCODER_CAN_ID);
        wheelOffset = 0.0;// TBD

        //make these constants, they are for testing rn only
       elevatorPivotMtrLT.configReverseSoftLimitThreshold(2065.0);
       elevatorPivotMtrLT.configReverseSoftLimitEnable(true);

       elevatorPivotMtrLT.configForwardSoftLimitThreshold(2568.0);
       elevatorPivotMtrLT.configForwardSoftLimitEnable(true);

        elevatorPivotMtrLT.config_kP(0, PID_ELEVATOR_UP_EMPTY_KP);   //TBD does empty really mean empty?
        elevatorPivotMtrLT.config_kI(0, PID_ELEVATOR_UP_EMPTY_KI);
        elevatorPivotMtrLT.config_kD(0, PID_ELEVATOR_UP_EMPTY_KD);

        elevatorPivotMtrLT.config_kP(1, PID_ELEVATOR_DOWN_EMPTY_KP);
        elevatorPivotMtrLT.config_kI(1, PID_ELEVATOR_DOWN_EMPTY_KI);
        elevatorPivotMtrLT.config_kD(1, PID_ELEVATOR_DOWN_EMPTY_KD);

        elevatorPivotMtrLT.config_kP(2, PID_ELEVATOR_UP_CONE_KP);
        elevatorPivotMtrLT.config_kI(2, PID_ELEVATOR_UP_CONE_KI);
        elevatorPivotMtrLT.config_kD(2, PID_ELEVATOR_UP_CONE_KD);
        
        elevatorPivotMtrLT.config_kP(3, PID_ELEVATOR_DOWN_CONE_KP);
        elevatorPivotMtrLT.config_kI(3, PID_ELEVATOR_DOWN_CONE_KI);
        elevatorPivotMtrLT.config_kD(3, PID_ELEVATOR_DOWN_CONE_KD);

        setBrakeMode();

        startElevatorThread();
    }



    public void startElevatorThread()
    {
        elevatorThread = new Thread(() ->
        {
            while(true)
            {
                getCarriagePositionInch();   //TBD Why here?

                switch (elevatorState)
                {
                    case ELEVATOR_STATE_IDLE:
                        
                    break;

                    case ELEVATOR_STATE_ELEVATOR_SCORE_TOP:
                        elevatorSpoolDone = false;
                        elevatorPivotDone = false;

                        if(!elevatorSpoolDone)  //TBD - don't use negative logic
                        {
                            elevatorSpoolPid.setReference(ELEVATOR_SPOOL_SCORE_TOP_POS, CANSparkMax.ControlType.kPosition);
                            if(elevatorLimitSwitchScoringTop.isPressed())
                            {
                                elevatorSpoolDone = true;
                            }
                        }
                        if(!elevatorPivotDone)
                        {
                            elevatorPivotHighScoringPosition();
                            elevatorPivotDone = true;
                        }

                        elevatorState = ELEVATOR_STATE_IDLE;
                    break;

                    case ELEVATOR_STATE_ELEVATOR_SCORE_MID:
                        elevatorSpoolPid.setReference(ELEVATOR_SPOOL_SCORE_MID_POS, CANSparkMax.ControlType.kPosition);
                        elevatorPivotMidScoringPosition();

                        elevatorState = ELEVATOR_STATE_IDLE;
                    break;

                    case ELEVATOR_STATE_ELEVATOR_SCORE_LOW:
                        elevatorSpoolDone = false;
                        elevatorPivotDone = false;
                        if(!elevatorSpoolDone)
                        {
                            elevatorSpoolPid.setReference(ELEVATOR_SPOOL_SCORE_LOW_POS, CANSparkMax.ControlType.kPosition);
                            
                            if(elevatorLimitSwitchResetBottom.isPressed() || getCarriagePositionInch() <= ELEVATOR_SPOOL_BOT_MAX_RANGE )
                            {
                                if(elevatorLimitSwitchResetBottom.isPressed())
                                {
                                    elevatorSpoolEncoder.setPosition(0.0);

                                }
                                elevatorSpoolDone = true;
                            }
                        }
                        if(!elevatorPivotDone)
                        {
                            elevatorPivotLowScoringPosition();
                            elevatorPivotDone = true;
                        }
                         
                        elevatorState = ELEVATOR_STATE_IDLE;
                    break;

                    case ELEVATOR_STATE_ELEVATOR_STOW:
                        elevatorSpoolPid.setReference(ELEVATOR_SPOOL_STOWED_POS, CANSparkMax.ControlType.kPosition);
                        elevatorPivotStowedPosition();

                        elevatorState = ELEVATOR_STATE_IDLE;
                    break;

                    default:
                        elevatorState = ELEVATOR_STATE_IDLE;
                    break;
                }
            }
        });
    }


    
    public void setBrakeMode()
    {
        elevatorSpoolMC.setIdleMode(IdleMode.kBrake);
        elevatorPivotMtrLT.setNeutralMode(NeutralMode.Coast);
        elevatorPivotMtrRT.setNeutralMode(NeutralMode.Coast);
    }

    public void elevatorScoreTop()
    {
        elevatorState = ELEVATOR_STATE_ELEVATOR_SCORE_TOP;
    }

    public void elevatorScoreMid()
    {
        elevatorState = ELEVATOR_STATE_ELEVATOR_SCORE_MID;
    }

    public void elevatorScoreLow()
    {
        elevatorState = ELEVATOR_STATE_ELEVATOR_SCORE_LOW;
    }



    /************************************************************************************************************************************************
    * 
    * Elevator spool code
    * 
    ************************************************************************************************************************************************/
    public void manualExtension(double xboxInput)
    {
        xboxInput *= -1.0;
        double mtrPwr;
        elevatorState = ELEVATOR_STATE_IDLE;

        if(Math.abs(xboxInput) > 0.2)
        {
            elevatorSpoolManualMode = true;
        } 
        else if(elevatorSpoolManualMode)
        {
            elevatorSpoolMC.set(0.0);
            elevatorSpoolManualMode = false;
        }

        mtrPwr = xboxInput;
        System.out.println("xboxinput" + xboxInput);

        if(elevatorSpoolManualMode)
        {
            if(xboxInput >= 0.2)//add a constant
            {
                if(getCarriagePositionInch() >= ELEVATOR_SPOOL_TOP_MAX_RANGE || elevatorLimitSwitchScoringTop.isPressed())
                {
                    mtrPwr = ELEVATOR_SPOOL_MTR_STOP;
                } 
            } 
            else if(xboxInput <= -0.2)
            {
                mtrPwr *= 0.3;
                if(elevatorLimitSwitchResetBottom.isPressed() || getCarriagePositionInch() <= ELEVATOR_SPOOL_BOT_MAX_RANGE )
                {
                    mtrPwr = ELEVATOR_SPOOL_MTR_STOP;
                    System.out.println("Bot Limit Elevator Spool" + getCarriagePositionInch());

                    if(elevatorLimitSwitchResetBottom.isPressed())
                    {
                        elevatorSpoolEncoder.setPosition(0.0);

                    }
                }
            }

            elevatorSpoolMC.set(mtrPwr);
        }
    }

    public void ignoreSpoolSoftLimit(boolean ignoresSoftLimits)
    {
        if (ignoresSoftLimits)
        {
            ELEVATOR_SPOOL_TOP_MAX_RANGE = 9999.0;
            ELEVATOR_SPOOL_BOT_MAX_RANGE = -9999.0;
        }
        else
        {
            ELEVATOR_SPOOL_TOP_MAX_RANGE = 75.0;// TBD, this is an approx value
            ELEVATOR_SPOOL_BOT_MAX_RANGE = 0.0;
        }
    }



    public double getCarriagePositionInch()
    {
        elevatorCarriagePosInch = elevatorSpoolEncoder.getPosition();
        return elevatorCarriagePosInch;
    }

    /************************************************************************************************************************************************
     * 
     * Elevator pivot code
     *
     ************************************************************************************************************************************************/
    public void elevatorPivotLowScoringPosition() {
        elevatorPivotManualMode = false;
        pivotCurrentAngle = pivotEnc.getPosition();

        if (indexingCone) {
            elevatorPivotMtrLT.selectProfileSlot(3, 0);
            System.out.println("Low Scoring with cone");
        } else {
            elevatorPivotMtrLT.selectProfileSlot(1, 0);
            System.out.println("Low Scoring without cone");
        }
        elevatorPivotMtrLT.set(ControlMode.Position, ELEVATOR_PIVOT_SCORE_LOW_POS);
        currentPivotCmd = ELEVATOR_PIVOT_SCORE_LOW_CMD;
    }

    public void elevatorPivotMidScoringPosition() {
        elevatorPivotManualMode = false;
        pivotCurrentAngle = pivotEnc.getPosition();

        if (indexingCone && currentPivotCmd == ELEVATOR_PIVOT_SCORE_HIGH_CMD) //TBD Check independantly //TBD instead of checking state, why not just check current position
        {
            elevatorPivotMtrLT.selectProfileSlot(3, 0);
            System.out.println("Mid Scoring with cone");
        } else if (indexingCone && currentPivotCmd == ELEVATOR_PIVOT_SCORE_LOW_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(2, 0);
            System.out.println("Mid Scoring with cone");
        } else if (currentPivotCmd == ELEVATOR_PIVOT_SCORE_HIGH_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(1, 0);
            System.out.println("Mid Scoring without cone");
        } else if (currentPivotCmd == ELEVATOR_PIVOT_SCORE_LOW_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(0, 0);
            System.out.println("Mid Scoring without cone");
        }
        elevatorPivotMtrLT.set(ControlMode.Position, ELEVATOR_PIVOT_SCORE_MID_POS);
        currentPivotCmd = ELEVATOR_PIVOT_SCORE_MID_CMD;
    }

    public void elevatorPivotStowedPosition() {
        elevatorPivotManualMode = false;
        pivotCurrentAngle = pivotEnc.getPosition();

        if (indexingCone && currentPivotCmd == ELEVATOR_PIVOT_SCORE_HIGH_CMD) //TBD Check independantly //TBD instead of checking state, why not just check current position
        {
            elevatorPivotMtrLT.selectProfileSlot(3, 0);
            System.out.println("Mid Scoring with cone");
        } else if (indexingCone && currentPivotCmd == ELEVATOR_PIVOT_SCORE_LOW_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(2, 0);
            System.out.println("Mid Scoring with cone");
        } else if (currentPivotCmd == ELEVATOR_PIVOT_SCORE_HIGH_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(1, 0);
            System.out.println("Mid Scoring without cone");
        } else if (currentPivotCmd == ELEVATOR_PIVOT_SCORE_LOW_CMD) 
        {
            elevatorPivotMtrLT.selectProfileSlot(0, 0);
            System.out.println("Mid Scoring without cone");
        }
        elevatorPivotMtrLT.set(ControlMode.Position, ELEVATOR_PIVOT_STOWED_POS);
        currentPivotCmd = ELEVATOR_PIVOT_STOWED_CMD;
    }

    public void elevatorPivotHighScoringPosition() {
        elevatorPivotManualMode = false;
        pivotCurrentAngle = pivotEnc.getPosition();

        if (indexingCone) {
            elevatorPivotMtrLT.selectProfileSlot(2, 0);
            System.out.println("High Scoring with cone");
        } else {
            elevatorPivotMtrLT.selectProfileSlot(0, 0);
            System.out.println("HI Scoring without cone");

        }
        elevatorPivotMtrLT.set(ControlMode.Position, ELEVATOR_PIVOT_SCORE_HIGH_POS);
        System.out.println("HI Scoring controlmode set");

        currentPivotCmd = ELEVATOR_PIVOT_SCORE_HIGH_CMD;
    }

    public void manualPivotControl(double power) {
        if(Math.abs(power) > 0.0)
        {
            elevatorPivotManualMode = true;
        } 
        else if(elevatorPivotManualMode)
        {
            elevatorPivotManualMode = false;
            elevatorPivotMtrLT.set(0.0);
        }

        if(elevatorPivotManualMode == true)
        {
            elevatorPivotMtrLT.set(power);
        }
    }



    /*----------------------------------------------------------------------------------------------
    * 
    *  Smart Dashboard
    *
    *---------------------------------------------------------------------------------------------*/
    public void smartDashboardElevator()
    {
        SmartDashboard.putBoolean("Bottom Limitswitch", elevatorLimitSwitchResetBottom.isPressed());
        SmartDashboard.putBoolean("Top Limitswitch", elevatorLimitSwitchScoringTop.isPressed());
        SmartDashboard.putBoolean("Middle Limitswitch", elevatorLimitSwitchScoringMid.get());

        SmartDashboard.putNumber("spool current position inches", elevatorCarriagePosInch);
    }

    public void smartDashboardElevator_DEBUG()
    {
        
    }

    /*----------------------------------------------------------------------------------------------
    * 
    *  pivot testing, left while merging. clean it later
    *
    *---------------------------------------------------------------------------------------------*/
    public void getEncPuts()
    {   
        if(elevatorPivotMtrLT.get()>testMaxPower)
        testMaxPower = elevatorPivotMtrLT.get();

       SmartDashboard.putNumber("LTclosedError", elevatorPivotMtrLT.getClosedLoopError());
       SmartDashboard.putNumber("EncRawValue", pivotEnc.getPosition());
       SmartDashboard.putNumber("EncRawValue", pivotEnc.getAbsolutePosition());
       SmartDashboard.putNumber("SelectedSensorPosition", elevatorPivotMtrLT.getSelectedSensorPosition());
       SmartDashboard.putNumber("testMaxPower", testMaxPower);      
    }

    public void testElevator()
    {
        elevatorPivotMtrLT.selectProfileSlot(0, 0);
        elevatorPivotMtrLT.set(ControlMode.Position,2200.0);
    }
}