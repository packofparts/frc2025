package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final SparkMax motor;
    private final SparkMax motor2;

    private static Indexer instance;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }

        return instance;
    }

    private Indexer() {
        motor = Constants.Indexer.MOTOR.createSparkMax();
        motor2 = Constants.Indexer.MOTOR2.createSparkMax();
    }

    public Command run() {
        return runOnce(() -> {
            motor.set(Constants.Indexer.SPEED);
            motor2.set(Constants.Indexer.SPEED * 1.0);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.set(0.0);
            motor2.set(0.0);
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            motor.set(-1 * Constants.Indexer.SPEED);
            motor2.set(-1 * Constants.Indexer.SPEED * 0.1);
        });
    }

    @Override
    public void periodic() {}
}
