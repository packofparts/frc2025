package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import POPLib.Sensors.BeamBreak.BeamBreak;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final SparkMax motor;

    private static Indexer instance;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }

        return instance;
    }

    private Indexer() {
        motor = Constants.Indexer.MOTOR.createSparkMax();
    }

    public Command run() {
        return runOnce(() -> {
            System.out.println("Trying to run Indexer");
            motor.set(Constants.Indexer.SPEED);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.set(0.0);
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            motor.set(-1 * Constants.Indexer.SPEED);
        });
    }

    @Override
    public void periodic() {}
}
