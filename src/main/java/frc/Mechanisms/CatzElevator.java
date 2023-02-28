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
    private final static int ELEV_STATE_IDLE                 = 0;
    private final static int ELEV_STATE_ELEVATOR_MOVE_TO_TOP = 1;
    private final static int ELEV_STATE_ELEVATOR_MOVE_TO_MID = 2;
    private final static int ELEV_STATE_ELEVATOR_STOW        = 3;

    private static boolean elevatorSpoolDone = false;
    private static boolean elevatorPivotDone = false;

    private static boolean elevatorPivotMalualMode = false;

    public static CANSparkMax elevatorSpoolMC;

    private WPI_TalonFX elevPivotMtrLT;
    private WPI_TalonFX elevPivotMtrRT;

    private final int ELEVATOR_SPOOL_MC_CAN_ID = 21;
    private final int ELEVATOR_PIVOT_LT_MC_ID  = 22;
    private final int ELEVATOR_PIVOT_RT_MC_ID  = 20;

    private final int ELEV_SPOOL_MC_CURRENT_LIMIT = 60; //TBD
    private SupplyCurrentLimitConfiguration pivotMtrCurrentLimit;
    private final int     ELEV_PIV_CURRENT_LIMIT        = 30;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final static double ELEVATOR_SPOOL_EXTEND_MTR_POWER  =  1.0;
    private final static double ELEVATOR_SPOOL_RETRACT_MTR_POWER = -0.5;
    private final static double ELEVATOR_SPOOL_MTR_STOP          =  0.0;

    private SparkMaxLimitSwitch elevLimitSwitchStowedPos;
    private SparkMaxLimitSwitch elevLimitSwitchScoringTop;
    private DigitalInput elevLimitSwitchScoringMid;

    private final int ELEV_SCORING_MID_DIO_PORT = 5; // TBD

    private final double ELEV_SPOOL_GEAR_1_SIZE   = 12.0;     //Neo pinin TBD - confirm size
    private final double ELEV_SPOOL_GEAR_2_SIZE   = 30.0;     //on spool shaft TBD - confirm size
    private final double ELEV_SPOOL_GEAR_RATIO    = ELEV_SPOOL_GEAR_2_SIZE / ELEV_SPOOL_GEAR_1_SIZE;
    private final double ELEV_SPOOL_DIAMETER      = 1.25;     //check diameter/radius
    private final double ELEV_SPOOL_CIRCUMFERENCE = Math.PI * ELEV_SPOOL_DIAMETER;
    private final double NEO_CNTS_PER_REVOLUTION  = 42.0;
    private final double ELEV_SPOOL_CNT_TO_INCHES = ELEV_SPOOL_GEAR_RATIO * ELEV_SPOOL_CIRCUMFERENCE * (1/NEO_CNTS_PER_REVOLUTION);//test later 

    private SparkMaxPIDController elevSpoolPid;
    private final double PID_ELEVATOR_SPOOL_KP = 0.00;
    private final double PID_ELEVATOR_SPOOL_KI = 0.00;
    private final double PID_ELEVATOR_SPOOL_KD = 0.00;


    private final double ELEV_PIVOT_VERSA_STAGE_1_RATIO = 3.0;
    private final double ELEV_PIVOT_VERSA_STAGE_2_RATIO = 10.0;
    private final double ELEV_PIVOT_VERSA_FINAL_RATIO   = ELEV_PIVOT_VERSA_STAGE_1_RATIO * ELEV_PIVOT_VERSA_STAGE_2_RATIO;

    private final double ELEV_PIVOT_PINION_GEAR = 16.0;
    private final double ELEV_PIVOT_SPUR_GEAR   = 84.0;
    private final double ELEV_PIVOT_GEAR_RATIO  = ELEV_PIVOT_SPUR_GEAR / ELEV_PIVOT_PINION_GEAR;

    private final double ELEV_PIVOT_FINAL_RATIO   = ELEV_PIVOT_VERSA_FINAL_RATIO * ELEV_PIVOT_GEAR_RATIO;
    private final double ELEV_PIV_CNTS_TO_DEGREES = 360.0 / 2048.0;//add gear ratio

    private final double ELEV_SPOOL_TOP_MAX_RANGE = 36.0;// TBD
    private final double ELEV_SPOOL_BOT_MAX_RANGE = 0.0;

    private static RelativeEncoder elevSpoolEncoder;

    private final double ELEV_SPOOL_ENCODER_STARTING_POS = 0.0;

    private double elevSpoolScoreTopPos = 10.0; //TBD - probably use ELEV_TOP_MAX_RANGE as score pos
    private double elevSpoolScoreMidPos = 5.0;
    private double elevSpoolScoreBotPos = 0.0;

    private final double ELEV_PIVOT_SCORE_HIGH_POS = 2023.0; //to be fixed
    private final double ELEV_PIVOT_SCORE_MID_POS  = 2500.0; //to be fixed
    private final double ELEV_PIVOT_SCORE_LOW_POS  = 2500.0; //to be fixed

    private double currentSpoolPos;
    private double elevCarriagePosInch;

    private static double pivotTargetAngle    = 0.0;
    private static double pivotCurrentAngleLT = 0.0;
    // private static double pivotCurrentAngleRT;

    private double pivotDistanceToTarget = pivotTargetAngle - pivotCurrentAngleLT;

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

    private int currentPivotCmd = 2;//to be fixed

    private final int ELEV_PIV_SCORE_LOW_CMD = 0;
    private final int ELEV_PIV_SCORE_MID_CMD = 1;
    private final int ELEV_PIV_SCORE_HIGH_CMD = 2;

    private double wheelOffset;

    private CANCoder pivotEnc;
    private DigitalInput MagEncPWMInput;
    private final int ELEV_PIV_ENCODER_SEN_ID = 9;
    public double testMaxPower;


    
    public CatzElevator() {
        /************************************************************************************
         * spool
         ************************************************************************************/
        elevatorSpoolMC = new CANSparkMax(ELEVATOR_SPOOL_MC_CAN_ID, MotorType.kBrushless);
        elevatorSpoolMC.restoreFactoryDefaults();

        elevatorSpoolMC.setSmartCurrentLimit(ELEV_SPOOL_MC_CURRENT_LIMIT);

        elevLimitSwitchStowedPos = elevatorSpoolMC.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        elevLimitSwitchScoringTop = elevatorSpoolMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        elevLimitSwitchScoringMid = new DigitalInput(ELEV_SCORING_MID_DIO_PORT);

        elevSpoolEncoder = elevatorSpoolMC.getEncoder();
        elevSpoolEncoder.setPositionConversionFactor(ELEV_SPOOL_CNT_TO_INCHES);
        elevSpoolEncoder.setPosition(ELEV_SPOOL_ENCODER_STARTING_POS);

        elevSpoolPid = elevatorSpoolMC.getPIDController();

        elevSpoolPid.setP(PID_ELEVATOR_SPOOL_KP);
        elevSpoolPid.setI(PID_ELEVATOR_SPOOL_KI);
        elevSpoolPid.setD(PID_ELEVATOR_SPOOL_KD);



        /************************************************************************************
         * pivot
         ************************************************************************************/
        elevPivotMtrLT = new WPI_TalonFX(ELEVATOR_PIVOT_LT_MC_ID);
        elevPivotMtrRT = new WPI_TalonFX(ELEVATOR_PIVOT_RT_MC_ID);
        
        elevPivotMtrLT.configFactoryDefault();
        elevPivotMtrRT.configFactoryDefault();

        elevPivotMtrRT.follow(elevPivotMtrLT);
        elevPivotMtrRT.setInverted(true);

        pivotMtrCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, ELEV_PIV_CURRENT_LIMIT,
                CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
        elevPivotMtrLT.configSupplyCurrentLimit(pivotMtrCurrentLimit);
        elevPivotMtrRT.configSupplyCurrentLimit(pivotMtrCurrentLimit);

        pivotEnc = new CANCoder(ELEV_PIV_ENCODER_SEN_ID); // CANCoder
        wheelOffset = 0.0;// TBD
        //make these constants, they are for testing rn only
        elevPivotMtrLT.configReverseSoftLimitThreshold(1990.0);
       elevPivotMtrLT.configReverseSoftLimitEnable(true);

       elevPivotMtrLT.configForwardSoftLimitThreshold(2600.0);
       elevPivotMtrLT.configForwardSoftLimitEnable(true);

        elevPivotMtrLT.config_kP(0, PID_ELEVATOR_UP_EMPTY_KP);
       // elevPivotMtrRT.config_kP(0, PID_ELEVATOR_UP_EMPTY_KP);
        elevPivotMtrLT.config_kI(0, PID_ELEVATOR_UP_EMPTY_KI);
        //elevPivotMtrRT.config_kI(0, PID_ELEVATOR_UP_EMPTY_KI);
        elevPivotMtrLT.config_kD(0, PID_ELEVATOR_UP_EMPTY_KD);
        //elevPivotMtrRT.config_kD(0, PID_ELEVATOR_UP_EMPTY_KD);

        elevPivotMtrLT.config_kP(1, PID_ELEVATOR_DOWN_EMPTY_KP);
        //elevPivotMtrRT.config_kP(1, PID_ELEVATOR_DOWN_EMPTY_KP);
        elevPivotMtrLT.config_kI(1, PID_ELEVATOR_DOWN_EMPTY_KI);
        //elevPivotMtrRT.config_kI(1, PID_ELEVATOR_DOWN_EMPTY_KI);
        elevPivotMtrLT.config_kD(1, PID_ELEVATOR_DOWN_EMPTY_KD);
        //elevPivotMtrRT.config_kD(1, PID_ELEVATOR_DOWN_EMPTY_KD);

        elevPivotMtrLT.config_kP(2, PID_ELEVATOR_UP_CONE_KP);
        //elevPivotMtrRT.config_kP(2, PID_ELEVATOR_UP_CONE_KP);
        elevPivotMtrLT.config_kI(2, PID_ELEVATOR_UP_CONE_KI);
        //elevPivotMtrRT.config_kI(2, PID_ELEVATOR_UP_CONE_KI);
        elevPivotMtrLT.config_kD(2, PID_ELEVATOR_UP_CONE_KD);
        //elevPivotMtrRT.config_kD(2, PID_ELEVATOR_UP_CONE_KD);
        
        elevPivotMtrLT.config_kP(3, PID_ELEVATOR_DOWN_CONE_KP);
        //elevPivotMtrRT.config_kP(3, PID_ELEVATOR_DOWN_CONE_KP);
        elevPivotMtrLT.config_kI(3, PID_ELEVATOR_DOWN_CONE_KI);
        //elevPivotMtrRT.config_kI(3, PID_ELEVATOR_DOWN_CONE_KI);
        elevPivotMtrLT.config_kD(3, PID_ELEVATOR_DOWN_CONE_KD);
        //elevPivotMtrRT.config_kD(3, PID_ELEVATOR_DOWN_CONE_KD);

        setBrakeMode();
    }



    public void startElevatorThread()
    {
        elevatorThread = new Thread(() ->
        {
            //extension state - intakeOut
            while(true)
            {
                getSpoolMtrPositionInch();
                currentSpoolPos = elevCarriagePosInch;

                switch (elevatorState)
                {
                    case ELEV_STATE_IDLE:
                        
                    break;

                    case ELEV_STATE_ELEVATOR_MOVE_TO_TOP:
                        if(!elevatorSpoolDone && !elevatorPivotDone)
                        {
                            if(!elevLimitSwitchScoringTop.isPressed())
                            {
                                elevatorSpoolMC.set(ELEVATOR_SPOOL_EXTEND_MTR_POWER);
                            } 
                            else
                            {
                                elevatorSpoolMC.set(ELEVATOR_SPOOL_MTR_STOP);
                                elevatorSpoolDone = true;
                            }

                            
                        }
                    break;

                    case ELEV_STATE_ELEVATOR_MOVE_TO_MID:
                        if(!elevatorSpoolDone && !elevatorPivotDone)
                        {
                            elevSpoolPid.setReference(elevSpoolScoreMidPos, CANSparkMax.ControlType.kPosition);
                            if(elevLimitSwitchScoringMid.get())
                            {
                                elevSpoolScoreMidPos = elevSpoolEncoder.getPosition();
                                elevatorSpoolDone = true;
                            }
                        }
                    break;

                    case ELEV_STATE_ELEVATOR_STOW:
                        if(!elevatorSpoolDone && !elevatorPivotDone)
                        {
                            elevSpoolPid.setReference(elevSpoolScoreBotPos, CANSparkMax.ControlType.kPosition);
                            if(elevLimitSwitchStowedPos.isPressed())
                            {
                                elevSpoolScoreMidPos = elevSpoolEncoder.getPosition();
                                elevatorSpoolDone = true;
                            }
                        }
                    break;

                    default:
                        elevatorState = ELEV_STATE_IDLE;
                    break;
                }
            }
        });
    }


    
    public void setBrakeMode()
    {
        elevatorSpoolMC.setIdleMode(IdleMode.kBrake);
        elevPivotMtrLT.setNeutralMode(NeutralMode.Brake);
        elevPivotMtrRT.setNeutralMode(NeutralMode.Brake);
    }



    /************************************************************************************************************************************************
    * 
    * Elevator spool code
    * 
    ************************************************************************************************************************************************/
    public void spoolTopScorePos()
    {
        if(currentSpoolPos < ELEV_SPOOL_TOP_MAX_RANGE)
        {
            elevatorSpoolMC.set(0.3);
        }
    }



    public void spoolMidScorePos()//TBD - set the position value to inches
    {
        if(!elevLimitSwitchScoringMid.get())
        {
            if(currentSpoolPos < 0.5)
            {
                elevatorSpoolMC.set(0.3);
            }
            else if(currentSpoolPos > 0.5)
            {
                elevatorSpoolMC.set(-0.3);
            }
        }
    }



    public void spoolStowedPos()
    {
        if(currentSpoolPos < 0.0)
        {
            elevatorSpoolMC.set(-0.3);
        }
    }



    public void manualExtension(double xboxInput)
    {
        double mtrPwr;
        elevatorState = ELEV_STATE_IDLE;

        mtrPwr = xboxInput;

        if(xboxInput >= 0.2)//add a constant
        {
            if(getSpoolMtrPositionInch() >= ELEV_SPOOL_TOP_MAX_RANGE || elevLimitSwitchScoringTop.isPressed())
            {
                mtrPwr = 0.0;
            } 
        } 
        else if(xboxInput <= -0.2)
        {
            if(elevLimitSwitchStowedPos.isPressed())
            {
                mtrPwr = 0.0;
                elevSpoolEncoder.setPosition(0.0);
            }
        } 
        else
        {
            mtrPwr = 0.0;
        }

        elevatorSpoolMC.set(mtrPwr);
    }



    public double getSpoolMtrPositionInch()
    {
        elevCarriagePosInch = elevSpoolEncoder.getPosition();
        return elevCarriagePosInch;
    }

    /************************************************************************************************************************************************
     * 
     * Elevator pivot code
     *
     ************************************************************************************************************************************************/
    public void lowScoringPosition() {
        elevatorPivotMalualMode = false;
        pivotCurrentAngleLT = pivotEnc.getPosition();

        if (indexingCone) {
            elevPivotMtrLT.selectProfileSlot(3, 0);
            // elevPivotMtrRT.selectProfileSlot(3, 0);
            System.out.println("Low Scoring with cone");
        } else {
            elevPivotMtrLT.selectProfileSlot(1, 0);
            // elevPivotMtrRT.selectProfileSlot(1, 0);
            System.out.println("Low Scoring without cone");
        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_LOW_POS);
        // elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_LOW_POS);
        currentPivotCmd = ELEV_PIV_SCORE_LOW_CMD;
    }

    public void midScoringPosition() {
        elevatorPivotMalualMode = false;
        pivotCurrentAngleLT = pivotEnc.getPosition();

        if (indexingCone && currentPivotCmd == ELEV_PIV_SCORE_HIGH_CMD) {
            elevPivotMtrLT.selectProfileSlot(3, 0);
            // elevPivotMtrRT.selectProfileSlot(3, 0);
            System.out.println("Mid Scoring with cone");
        } else if (indexingCone && currentPivotCmd == ELEV_PIV_SCORE_LOW_CMD) {
            elevPivotMtrLT.selectProfileSlot(2, 0);
            // elevPivotMtrRT.selectProfileSlot(2, 0);
            System.out.println("Mid Scoring with cone");
        } else if (currentPivotCmd == ELEV_PIV_SCORE_HIGH_CMD) {
            elevPivotMtrLT.selectProfileSlot(1, 0);
            // elevPivotMtrRT.selectProfileSlot(1, 0);
            System.out.println("Mid Scoring without cone");
        } else if (currentPivotCmd == ELEV_PIV_SCORE_LOW_CMD) {
            elevPivotMtrLT.selectProfileSlot(0, 0);
            // elevPivotMtrRT.selectProfileSlot(0, 0);
            System.out.println("Mid Scoring without cone");
        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_MID_POS);
        // elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_MID_POS);
        currentPivotCmd = ELEV_PIV_SCORE_MID_CMD;
    }

    public void highScoringPosition() {
        elevatorPivotMalualMode = false;
        pivotCurrentAngleLT = pivotEnc.getPosition();

        if (indexingCone) {
            elevPivotMtrLT.selectProfileSlot(2, 0);
            // elevPivotMtrRT.selectProfileSlot(2, 0);
            System.out.println("High Scoring with cone");
        } else {
            elevPivotMtrLT.selectProfileSlot(0, 0);
            // elevPivotMtrRT.selectProfileSlot(0, 0);
            System.out.println("HI Scoring without cone");

        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_HIGH_POS);
        System.out.println("HI Scoring controlmode set");

        // elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_HIGH_POS);
        currentPivotCmd = ELEV_PIV_SCORE_HIGH_CMD;
    }

    public void manualPivotControl(double power) {
        if(Math.abs(power) > 0.0)
        {
            elevatorPivotMalualMode = true;
        }
        if(elevatorPivotMalualMode == true)
        {
            elevPivotMtrLT.set(power);
        }
        // elevPivotMtrRT.set(power);
    }



    /*----------------------------------------------------------------------------------------------
    * 
    *  Smart Dashboard
    *
    *---------------------------------------------------------------------------------------------*/
    public void smartDashboardElevator()
    {
        SmartDashboard.putBoolean("Bottom Limitswitch", elevLimitSwitchStowedPos.isPressed());//set variable for limitswitch
        SmartDashboard.putBoolean("Top Limitswitch", elevLimitSwitchScoringTop.isPressed());//set variable for limitswitch
        SmartDashboard.putBoolean("Middle Limitswitch", elevLimitSwitchScoringMid.get());//set variable for limitswitch

        SmartDashboard.putNumber("spool current position inches", elevCarriagePosInch);
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
        if(elevPivotMtrLT.get()>testMaxPower)
        testMaxPower = elevPivotMtrLT.get();

       SmartDashboard.putNumber("LTclosedError", elevPivotMtrLT.getClosedLoopError());
       SmartDashboard.putNumber("EncRawValue", pivotEnc.getPosition());
       SmartDashboard.putNumber("EncRawValue", pivotEnc.getAbsolutePosition());
       SmartDashboard.putNumber("SelectedSensorPosition", elevPivotMtrLT.getSelectedSensorPosition());
       SmartDashboard.putNumber("testMaxPower", testMaxPower);

      // SmartDashboard.putString("ControlMode", ring) elevPivotMtrLT.getControlMode());
      
    }

    public void testElevator()
    {
        elevPivotMtrLT.selectProfileSlot(0, 0);
        elevPivotMtrLT.set(ControlMode.Position,2200.0);
    }
}
