package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import poplib.smart_dashboard.TunableNumber;
import poplib.subsytems.pivot.SparkPivot;

public class Intake extends SparkPivot{
    private static Intake instance;

    private SparkMax pivotMotor;
    private SparkMax rollerMotor;
    private TunableNumber setpoint;
    private double error;
    private PIDController pidController;
    public static Intake getInstance() {

        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private Intake() {
      super(Constants.Intake.PIVOT_MOTOR_CONFIG, 1, )

      rollerMotor = Constants.Intake.ROLLER_MOTOR_CONFIG.createSparkMax();
      error = setpoint.get() - pivotMotor.getEncoder().getPosition();
      pidController = new PIDController(0,0,0);
    }
    

    public void setSetpoint(double sP) {
      setpoint.setDefault(sP);
    }

  public boolean reachedSetPoint() {
    if (Math.abs(error) < 0.1) {
        return true;
    }
    else {
      return false;
    }
  }

    public Command runIntake() {
      return run(() -> {
        rollerMotor.set(Constants.Intake.ROLLER_MOTOR_SPEED);
      } ).until(this::reachedSetPoint)
      .andThen(() -> {rollerMotor.set(Constants.Intake.ROLLER_MOTOR_STOP_SPEED);});
    }

    public Command rotateIntake(double setpoint) {
      return run(() -> {
        setSetpoint(setpoint);
      }).until(() -> reachedSetPoint());
    }

    @Override
    public void periodic() {
      pivotMotor.set(pidController.calculate(pivotMotor.getEncoder().getPosition()));
    }
}