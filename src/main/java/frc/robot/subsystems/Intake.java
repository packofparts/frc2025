package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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

    public Command runIntake() {
      Elevator elevator = Elevator.getInstance();
      Manipulator manipulator = Manipulator.getInstance();
      return elevator.moveElevator(0.0).
      alongWith(moveWristDown()).
      andThen(runRollerMotors()).
      alongWith(manipulator.run()).
      until(manipulator::coralIn).
      andThen(manipulator.stop()).
      alongWith(stopRollerMotors()).
      andThen(moveWristUp());
    }

    public Command runRollerMotors() {
      return runOnce(() -> {
        rollerMotor.set(Constants.Intake.ROLLER_MOTOR_SPEED);
      });
    }

    public Command stopRollerMotors() {
      return runOnce(() -> {
        rollerMotor.set(0.0);
      });
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