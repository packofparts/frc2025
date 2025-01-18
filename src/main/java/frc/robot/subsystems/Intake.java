package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import poplib.smart_dashboard.TunableNumber;
import poplib.subsytems.pivot.SparkPivot;

public class Intake extends SparkPivot{
    private static Intake instance;

    public static Intake getInstance() {

        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private SparkMax rollerMotor;
    private Intake() {
      super(Constants.Intake.PIVOT_MOTOR_CONFIG, Constants.Intake.FF_CONFIG, false, "intake");

      rollerMotor = Constants.Intake.ROLLER_MOTOR_CONFIG.createSparkMax();
    }

    public void runRollerMotors() {
      rollerMotor.set(Constants.Intake.ROLLER_MOTOR_SPEED);
    }

    public void stopRollerMotors() {
      rollerMotor.set(0.0);
    }

    public Command moveWristUp() {
      return super.moveWrist(Constants.Intake.ARM_UP_SETPOINT, Constants.Intake.ERROR);
    }
    
    public Command moveWristDown() {
      return super.moveWrist(Constants.Intake.ARM_DOWN_SETPOINT, Constants.Intake.ERROR);
    }

    @Override
    public void periodic() {
      super.periodic();
    }
}