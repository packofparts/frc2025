package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
    SparkMax pivotMotor;
    SparkMax rollerMotor;
    private TunableNumber setpoint;
    private double error;
    private PIDController pidController;

    private Intake() {
      pivotMotor = new SparkMax(0, null);
      rollerMotor = new SparkMax(0, null);
      error = setpoint.get() - pivotMotor.getEncoder().getPosition();
      pidController = new PIDController(0,0,0);
    }
    

    public void setSetpoint(double sP) {
      setpoint.setDefault(sP);
    }

  public boolean reachedSetPoint() {
    if (Math.abs(error) < 0.1) {
        return true;
    }else return false;
}

    public Command runIntake() {
      return run(() -> {
        rollerMotor.set(1);
      } ).until(this::reachedSetPoint)
      .andThen(() -> {rollerMotor.set(0);});
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