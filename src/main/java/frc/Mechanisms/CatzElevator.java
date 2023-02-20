package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.*;
import frc.robot.Robot;

public class CatzElevator {
    private Thread elevatorThread;

    private int elevSpoolstate;
    private final static int ELEV_STATE_IDLE                    = 0;
    private final static int ELEV_STATE_ELEVATOR_MOVE_TO_POS    = 1;

    public static CANSparkMax elevatorSpoolMC;

    private WPI_TalonFX elevPivotMtrLT;
    private WPI_TalonFX elevPivotMtrRT;

    private final int ELEVATOR_SPOOL_MC_CAN_ID = 21;
    private final int LT_PIVOT_MOTOR_ID = 22;
    private final int RT_PIVOT_MOTOR_ID = 20;

    private final int ELEV_SPOOL_MC_CURRENT_LIMIT = 60; //TBD
    private SupplyCurrentLimitConfiguration EncCurrentLimit;
    private final int     ELEV_PIV_CURRENT_LIMIT      = 30;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private SparkMaxLimitSwitch elevLimitSwitchStowedPos;
    private SparkMaxLimitSwitch elevLimitSwitchScoringTop;
    private DigitalInput elevLimitSwitchScoringMid;

    private final int ELEV_SCORING_TOP_DIO_PORT = 5; // TBD

    private final double ELEV_SPOOL_GEAR_1_SIZE   = 12.0;     //Neo pinin TBD - confirm size
    private final double ELEV_SPOOL_GEAR_2_SIZE   = 30.0;     //on spool shaft TBD - confirm size
    private final double ELEV_SPOOL_GEAR_RATIO    = ELEV_SPOOL_GEAR_2_SIZE / ELEV_SPOOL_GEAR_1_SIZE;
    private final double ELEV_SPOOL_DIAMETER      = 1.25;     //check diameter/radius
    private final double ELEV_SPOOL_CIRCUMFERENCE = Math.PI * ELEV_SPOOL_DIAMETER;
    private final double NEO_CNTS_PER_REVOLUTION  = 42.0;
    private final double ELEV_SPOOL_CNT_TO_INCHES = ELEV_SPOOL_GEAR_RATIO * ELEV_SPOOL_CIRCUMFERENCE * (1/NEO_CNTS_PER_REVOLUTION);//test later 

    private final double ELEV_PIV_CNTS_TO_DEGREES = 360/2048;//add gear ratio

    private final double ELEV_TOP_MAX_RANGE = 0.0;// TBD
    private final double ELEV_BOT_MAX_RANGE = -0.0;// TBD

    private static RelativeEncoder elevSpoolEncoder;

    private final double ELEV_SPOOL_ENCODER_STARTING_POS = 0.0;

    private final double ELEV_SPOOL_SCORE_TOP_POS = 0.0; //TBD
    private final double ELEV_SPOOL_SCORE_MID_POS = 0.0;

    private final double ELEV_PIVOT_SCORE_HIGH_POS  = 0.0;
    private final double ELEV_PIVOT_SCORE_MID_POS   = 0.0;
    private final double ELEV_PIVOT_SCORE_LOW_POS   = 0.0;

    private double currentSpoolPos;
    private double elevCarriagePosInch;

    private static double pivotTargetAngle;
    private static double pivotCurrentAngleLT;
    //private static double pivotCurrentAngleRT;

    private double pivotDistanceToTarget = pivotTargetAngle - pivotCurrentAngleLT;

    static public double PID_ELEVATOR_UP_EMPTY_KP   = 0.00; //needs to be fixed to certain value
	static public double PID_ELEVATOR_UP_EMPTY_KI   = 0.00; //needs to be fixed to certain value
	static public double PID_ELEVATOR_UP_EMPTY_KD   = 0.00; //needs to be fixed to certain value

    static public double PID_ELEVATOR_DOWN_EMPTY_KP = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_EMPTY_KI = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_EMPTY_KD = 0.00; //needs to be fixed to certain value

    static public double PID_ELEVATOR_UP_CONE_KP    = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_CONE_KI    = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_UP_CONE_KD    = 0.00; //needs to be fixed to certain value

    static public double PID_ELEVATOR_DOWN_CONE_KP  = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_CONE_KI  = 0.00; //needs to be fixed to certain value
    static public double PID_ELEVATOR_DOWN_CONE_KD  = 0.00; //needs to be fixed to certain value

    private final int INDEX_MATERIAL_NONE = 0;
    private final int INDEX_MATERIAL_CONE = 1;
    private int indexingMaterial = INDEX_MATERIAL_NONE;

    private double elevSpoolMtrPwr;

    private int currentPivotPos;

    private final int ELEV_PIV_SCORE_LOW_POS  = 0;
    private final int ELEV_PIV_SCORE_MID_POS  = 1;
    private final int ELEV_PIV_SCORE_HIGH_POS = 2;



    public CatzElevator()
    {
        elevatorSpoolMC = new CANSparkMax(ELEVATOR_SPOOL_MC_CAN_ID, MotorType.kBrushless);
        elevPivotMtrLT = new WPI_TalonFX(LT_PIVOT_MOTOR_ID);
        elevPivotMtrRT = new WPI_TalonFX(RT_PIVOT_MOTOR_ID);

        elevatorSpoolMC.restoreFactoryDefaults();
        elevPivotMtrLT.configFactoryDefault();
        elevPivotMtrRT.configFactoryDefault();

        setBrakeMode();

        elevatorSpoolMC.setSmartCurrentLimit(ELEV_SPOOL_MC_CURRENT_LIMIT);
        elevPivotMtrLT.configSupplyCurrentLimit(EncCurrentLimit);
        elevPivotMtrRT.configSupplyCurrentLimit(EncCurrentLimit);

        elevLimitSwitchStowedPos = elevatorSpoolMC.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        elevLimitSwitchScoringTop = elevatorSpoolMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        //elevLimitSwitchStowedPos = new DigitalInput(ELEV_STOWED_POS_DIO_PORT);
        //elevLimitSwitchScoringTop = new DigitalInput(ELEV_SCORING_MID_DIO_PORT);
        elevLimitSwitchScoringMid = new DigitalInput(ELEV_SCORING_TOP_DIO_PORT);

        elevSpoolEncoder = elevatorSpoolMC.getEncoder();
        elevSpoolEncoder.setPositionConversionFactor(ELEV_SPOOL_CNT_TO_INCHES);
        
        elevSpoolEncoder.setPosition(ELEV_SPOOL_ENCODER_STARTING_POS);
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

                switch (elevSpoolstate)
                {
                    case ELEV_STATE_IDLE:
                    
                    break;

                    case ELEV_STATE_ELEVATOR_MOVE_TO_POS:
                    
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
        if(currentSpoolPos < ELEV_TOP_MAX_RANGE)
        {
            elevatorSpoolMC.set(0.3);
        }
    }



    public void spoolMidScorePos()//TBD - set the position value to inches
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



    public void spoolStowedPos()
    {
        if(currentSpoolPos < 0.0)
        {
            elevatorSpoolMC.set(-0.3);
        }
    }



    public void startExtension(double mtrPwr)
    {
        elevSpoolMtrPwr = mtrPwr;

        if(elevSpoolMtrPwr > 0.0)
        {
            if(getSpoolMtrPositionInch() >= ELEV_TOP_MAX_RANGE || elevLimitSwitchScoringTop.isPressed())
            {
                elevSpoolMtrPwr = 0.0;
            }
        } 
        else if(elevSpoolMtrPwr < 0.0)
        {
            if(elevLimitSwitchStowedPos.isPressed())
            {
                elevSpoolMtrPwr = 0.0;
                elevSpoolEncoder.setPosition(0.0);
            }
        }
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
    public void lowScoringPosition()
    {
        if (indexingMaterial == INDEX_MATERIAL_CONE)
        {
            elevPivotMtrLT.selectProfileSlot(3, 0);
            elevPivotMtrRT.selectProfileSlot(3, 0);
        }
        else
        {
            elevPivotMtrLT.selectProfileSlot(1, 0);
            elevPivotMtrRT.selectProfileSlot(1, 0);
        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_LOW_POS);
        elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_LOW_POS);
        currentPivotPos = 0;
    }

    public void midScoringPosition()
    {
        if (indexingMaterial == INDEX_MATERIAL_CONE && ELEV_PIVOT_SCORE_HIGH_POS == currentPivotPos)
        {
            elevPivotMtrLT.selectProfileSlot(3, 0);
            elevPivotMtrRT.selectProfileSlot(3, 0);
        }
        else if (indexingMaterial == INDEX_MATERIAL_CONE && ELEV_PIVOT_SCORE_LOW_POS == currentPivotPos)
        {
            elevPivotMtrLT.selectProfileSlot(2, 0);
            elevPivotMtrRT.selectProfileSlot(2, 0);
        }
        else if (currentPivotPos == ELEV_PIVOT_SCORE_HIGH_POS)
        {
            elevPivotMtrLT.selectProfileSlot(1, 0);
            elevPivotMtrRT.selectProfileSlot(1, 0);
        }
        else if (currentPivotPos == ELEV_PIVOT_SCORE_LOW_POS)
        {
            elevPivotMtrLT.selectProfileSlot(0, 0);
            elevPivotMtrRT.selectProfileSlot(0, 0);
        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_MID_POS);
        elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_MID_POS);
        currentPivotPos = 1;
    }

    public void highScoringPosition()
    {
        if (indexingMaterial == INDEX_MATERIAL_CONE)
        {
            elevPivotMtrLT.selectProfileSlot(2, 0);
            elevPivotMtrRT.selectProfileSlot(2, 0);
        }
        else
        {
            elevPivotMtrLT.selectProfileSlot(0, 0);
            elevPivotMtrRT.selectProfileSlot(0, 0);
        }
        elevPivotMtrLT.set(ControlMode.Position, ELEV_PIVOT_SCORE_HIGH_POS);
        elevPivotMtrRT.set(ControlMode.Position, ELEV_PIVOT_SCORE_HIGH_POS);
        currentPivotPos = 2;
    }

    public void manualPivotControl(double power)
    {
        elevPivotMtrLT.set(power);
        elevPivotMtrRT.set(power);
    }
}
