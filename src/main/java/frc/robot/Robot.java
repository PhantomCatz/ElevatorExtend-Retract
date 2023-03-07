// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.Autonomous.CatzAutonomous;
import frc.Autonomous.CatzAutonomousPaths;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Mechanisms.CatzBalance;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzElevator;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzClaw;
import frc.Mechanisms.CatzRGB;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //----------------------------------------------------------------------------------------------
  //  Shared Libraries & Utilities
  //----------------------------------------------------------------------------------------------
  public static CatzConstants       constants;

  public static DataCollection      dataCollection;
  public ArrayList<CatzLog>         dataArrayList;

  //----------------------------------------------------------------------------------------------
  //  Shared Robot Components (e.g. not mechanism specific)
  //----------------------------------------------------------------------------------------------
  //PDH
  //Camera
  //Limelight???
  

  public static AHRS                navX;

  public final int PH_CAN_ID = 1;
  public PneumaticHub pneumaticHub;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

  private XboxController xboxDrv;
  private XboxController xboxAux;

  public static CatzRGB catzRGB;

  public static boolean robotDisabled = false;
  public static boolean coneOnboard = false;
  public static boolean cubeOnboard = false;
  public static boolean cubeScoringReady = false;
  public static boolean coneScoringReady = false;
  public static boolean inAuton = false;
  public static boolean autobalancing = false;
  public static boolean noGamePiece = false;
  

  //----------------------------------------------------------------------------------------------
  //  Autonomous
  //----------------------------------------------------------------------------------------------
  public static CatzAutonomous      auton;
  public static CatzBalance         balance;
  public static CatzAutonomousPaths paths;

  //----------------------------------------------------------------------------------------------
  //  Mechanisms
  //----------------------------------------------------------------------------------------------
  public static CatzDrivetrain      drivetrain;
  public static CatzIntake          intake;
  public static CatzElevator        elevator;
  public static CatzClaw            claw;


  public static Timer               currentTime;      //TBD - what is this intended for?


  // put into mechanism classes 
  private final boolean DEPLOYED = true;
  private final boolean STOWED   = false;
  public boolean elevatorState = STOWED;
  public boolean intakeState   = STOWED;

  /*
   * For autobalancing
  */
  private final double OFFSET_DELAY = 0.5;    // put into mechanism classes


  /*-----------------------------------------------------------------------------------------
  *  
  *  robotXxx
  *
  *----------------------------------------------------------------------------------------*/
  /*-----------------------------------------------------------------------------------------
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  *----------------------------------------------------------------------------------------*/
  @Override
  public void robotInit()
  {
    //----------------------------------------------------------------------------------------------
    //  Shared Libraries & Utilities
    //----------------------------------------------------------------------------------------------
    constants      = new CatzConstants();

    dataCollection = new DataCollection();
    dataArrayList  = new ArrayList<CatzLog>();
    dataCollection.dataCollectionInit(dataArrayList);

    //----------------------------------------------------------------------------------------------
    //  Shared Robot Components (e.g. not mechanism specific)
    //----------------------------------------------------------------------------------------------
    //PDH
    
    pneumaticHub = new PneumaticHub(PH_CAN_ID);
    navX = new AHRS();
    navX.reset();
    navX.setAngleAdjustment(-navX.getYaw());  //TBD - What is this for?


    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    //----------------------------------------------------------------------------------------------
    //  Autonomous
    //----------------------------------------------------------------------------------------------
    // auton          = new CatzAutonomous();
    // balance        = new CatzBalance();
    // paths          = new CatzAutonomousPaths();

    //----------------------------------------------------------------------------------------------
    //  Mechanisms
    //----------------------------------------------------------------------------------------------
    drivetrain     = new CatzDrivetrain();
    // intake         = new CatzIntake();
    elevator       = new CatzElevator();
    claw           = new CatzClaw();


    currentTime = new Timer();
  }


  /*-----------------------------------------------------------------------------------------
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   *----------------------------------------------------------------------------------------*/
  @Override
  public void robotPeriodic()
  {
    //drivetrain.updateShuffleboard();

    //For each mechanism - debug and comp
    SmartDashboard.putNumber("NavX", navX.getAngle());
    elevator.smartDashboardElevator();
    elevator.smartDashboardElevator_DEBUG();

    //drivetrain.testAngle();

    
  //   if(!(isDisabled()))
  //   {
  //     robotDisabled = false;
  //     if(isAutonomous())
  //     {
  //       if(balance.startBalance)
  //       {
  //         autobalancing = true;
  //       }
  //       inAuton = true;
  //     }
  //     else
  //     {

  //     }

      
      
  //   }
  //   else
  //   {
  //     robotDisabled = true;
  //   }
    
  //   catzRGB.LEDWork();
   }


  /*-----------------------------------------------------------------------------------------
  *  
  *  autonomousXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
  *----------------------------------------------------------------------------------------*/
  @Override
  public void autonomousInit()
  {
    //drivetrain.setBrakeMode();

    currentTime.reset();
    currentTime.start();

    dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_STRAIGHT);  //TBD Pull from shuffleboard
    dataCollection.startDataCollection();                             //TBD where do we want to do this since also used in teleop?  robotInit???

    //drivetrain.initializeOffsets();     //TBD - Nicholas Update
    Timer.delay(OFFSET_DELAY);

    paths.determinePath();

    paths.stop();                   //TBD - move into autonomous file
    
  }


  /*-----------------------------------------------------------------------------------------
  *
  *  This function is called periodically during autonomous.
  *
  *----------------------------------------------------------------------------------------*/
  @Override
  public void autonomousPeriodic()
  {
    // drivetrain.drive(0.0, 0.5, navX.getAngle());
    
    // drivetrain.dataCollection();  
  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  teleopXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when teleop is enabled.
  *----------------------------------------------------------------------------------------*/
  @Override
  public void teleopInit() 
  {

    //drivetrain.setBrakeMode();      //TBD

    currentTime.reset();            //TBD
    currentTime.start();

    dataCollection.setLogDataID(dataCollection.LOG_ID_INTAKE);
    dataCollection.startDataCollection();
  }


  /*-----------------------------------------------------------------------------------------
  *
  *  This function is called periodically during operator control.
  *
  *----------------------------------------------------------------------------------------*/
  @Override
  public void teleopPeriodic()
  {
    //----------------------------------------------------------------------------------------------
    //  Drivetrain
    //----------------------------------------------------------------------------------------------
    drivetrain.cmdProcSwerve(xboxDrv.getLeftX(), xboxDrv.getLeftY(), xboxDrv.getRightX(), navX.getAngle());

    if(xboxDrv.getStartButtonPressed())
    {
      zeroGyro();
    }

    //----------------------------------------------------------------------------------------------
    //  Intake
    //----------------------------------------------------------------------------------------------
    //TBD - move logic to class method; this removes multiple gets and duplicate logic

    // intake.cmdProcDeploy(xboxDrv.getLeftStickButton());
    // intake.cmdProcStow  (xboxDrv.getRightStickButton());

    // intake.cmdProcRollerIn (xboxDrv.getRightTriggerAxis());
    // intake.cmdProcRollerOut(xboxDrv.getLeftTriggerAxis());


    // if(xboxDrv.getLeftStickButton())
    // {
    //   if(elevatorState == DEPLOYED)
    //   {
    //     //flash colors indicating that you can't deploy
    //   }
    //   else
    //   {
    //     intake.intakePivotDeploy();
    //     intakeState = DEPLOYED;
    //   }
    // }


    // else if(xboxDrv.getRightStickButton())
    // {
    //   intake.intakePivotStow();
    //   intakeState = STOWED;
    // }

    
    //leftTrigger-->control rollerOut
    // if (xboxDrv.getLeftTriggerAxis() > 0.2)
    // { 
    //   intake.intakeRollerOut();     
    // }

    // //rightTrigger-->control rollerIn
    // if (xboxDrv.getLeftTriggerAxis() > 0.2)     //TBD - COPY / PASTE Error
    // {
    //   intake.intakeRollerIn();      
    // }


    //----------------------------------------------------------------------------------------------
    //  Elevator - Automated
    //----------------------------------------------------------------------------------------------
    
    // elevator.cmdProcGoToPositionScoreLow (xboxAux.getAButton());
    // elevator.cmdProcGoToPositionScoreMid (xboxAux.getBButton());
    // elevator.cmdProcGoToPositionScoreHigh(xboxAux.getYButton());

    //TBD HOW DO WE GO TO STOW POSITION???

    if(xboxAux.getAButton())
    {
      if(intakeState == DEPLOYED)
      {
        //stow intake
        intakeState = STOWED;
      }
      // elevator.spoolStowedPos();
      // elevator.elevatorPivotLowScoringPosition();
      elevator.elevatorScoreLow();

      //go to LOW scoring position
    }
    else if(xboxAux.getBButton())
    {
      if(intakeState == DEPLOYED)
      {
        //stow intake
        intakeState = STOWED;
      }
      // elevator.spoolMidScorePos();
      // elevator.elevatorPivotMidScoringPosition();
      elevator.elevatorScoreMid();

      //go to MID scoring position 
    }
    else if(xboxAux.getYButton())
    {
      if(intakeState == DEPLOYED)
      {
        //stow intake
        intakeState = STOWED;
      }
      // elevator.spoolTopScorePos();
      // elevator.elevatorPivotHighScoringPosition();
      elevator.elevatorScoreTop();

      //go to HIGH scoring position
    }
    else if(xboxAux.getXButton())
    {
      if(intakeState == DEPLOYED)
      {
        //stow intake
        intakeState = STOWED;
      }
      // elevator.spoolTopScorePos();
      // elevator.elevatorPivotHighScoringPosition();
      elevator.elevatorStow();

      //go to HIGH scoring position
    }
    


    //----------------------------------------------------------------------------------------------
    //  Elevator - Manual
    //----------------------------------------------------------------------------------------------
    

    elevator.manualExtension(xboxAux.getRightY());

    elevator.ignoreSpoolSoftLimit(xboxAux.getRightStickButton());
    
    if(xboxAux.getPOV() == DPAD_UP)
    {
      if(intakeState == DEPLOYED)
      {
        //stow intake
        intakeState = STOWED;
      }
      elevator.manualPivotControl(-0.2);
      //while pressed, deploy elevator
    
    }
    else if(xboxAux.getPOV() == DPAD_DN)
    {
      //while pressed, pivot elevator
      elevator.manualPivotControl(0.1);
    }
    else
    {
      elevator.manualPivotControl(0.0);
    }


    //----------------------------------------------------------------------------------------------
    //  Claw
    //----------------------------------------------------------------------------------------------
    claw.procCmdClaw(xboxAux.getXButton(),      //Open Claw
                     xboxAux.getXButton() );    //lose Claw
  

    //----------------------------------------------------------------------------------------------
    //  Indexer
    //----------------------------------------------------------------------------------------------
    if(xboxAux.getPOV() == DPAD_LT)
    {
      //indexer manual index
    }        
    
  }   //End of teleopPeriodic()


  /*-----------------------------------------------------------------------------------------
  *  
  *  disabledXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    currentTime.stop();
    drivetrain.setCoastMode();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  testXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit()
  {
    drivetrain.initializeOffsets();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  simulateXxx
  *
  *----------------------------------------------------------------------------------------*/
 /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}



  /*-----------------------------------------------------------------------------------------
  *  
  *  TBD - MOVE THIS TO APPROPRIATE CLASS
  *
  *----------------------------------------------------------------------------------------*/

    public void zeroGyro()
    {
      navX.setAngleAdjustment(-navX.getYaw());
    }
}