package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;

import poplib.subsytems.pivot.SparkPivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AlphaIntake extends SparkPivot {
    private final SparkMax spin;

    private static AlphaIntake instance;

    public static AlphaIntake getInstance() {
        if (instance == null) {
            instance = new AlphaIntake();
        }

        return instance;
    }

    private AlphaIntake() {
        super(Constants.Intake.PIVOT, null, Constants.Intake.GEAR_RATIO, Constants.Intake.FF, Constants.Intake.ENCODER, Constants.Intake.TUNING_MODE, "intake");
        super.setpoint.setDefault(Constants.Intake.SETPOINTS.IDLE.getSetpoint());
        spin = Constants.Intake.SPIN.createSparkMax();
    }

    public Command run() {
        return runOnce(() -> {
            spin.set(Constants.Intake.SPEED);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            spin.set(0.0);
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            spin.set(-1 * Constants.Intake.SPEED);
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        super.log();
    }

}
