package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import poplib.subsytems.pivot.SparkPivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Intake extends SparkPivot {
    private final SparkMax spin;

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private Intake() {
        super(Constants.Intake.PIVOT, null, Constants.Intake.GEAR_RATIO, Constants.Intake.ff, Constants.Intake.ENCODER, Constants.Intake.TUNING_MODE, "intake");
        super.leadMotor.getEncoder().setPosition(31.0);
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
