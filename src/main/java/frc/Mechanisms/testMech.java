package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class testMech 
{
    private WPI_TalonFX ltPivot; 
    private WPI_TalonFX rtPivot;

    private final int RT_PIVOT_MOTOR_ID = 20;
    private final int LT_PIVOT_MOTOR_ID = 22;

    private WPI_TalonFX intakePivotMotor;
    private final int INTAKE_PIVOT_MC_ID        = 10; 

    public testMech()
    {
        ltPivot = new WPI_TalonFX(LT_PIVOT_MOTOR_ID);
        rtPivot = new WPI_TalonFX(RT_PIVOT_MOTOR_ID);

        ltPivot.configFactoryDefault();
        rtPivot.configFactoryDefault();

        ltPivot.setNeutralMode(NeutralMode.Brake);
        rtPivot.setNeutralMode(NeutralMode.Brake);

        intakePivotMotor = new WPI_TalonFX(INTAKE_PIVOT_MC_ID);

        intakePivotMotor.configFactoryDefault();

        intakePivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void elevatorMovePow(double power)
    {
        ltPivot.set(-power); //one of them will be negative 
        rtPivot.set( power);
    }

    public void intakeMovePow(double power)
    {
        intakePivotMotor.set(power);
    }
}
