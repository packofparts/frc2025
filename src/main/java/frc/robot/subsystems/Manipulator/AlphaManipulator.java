package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkMax;
import poplib.sensors.beam_break.BeamBreak;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlphaManipulator extends SubsystemBase {
    private final SparkMax motor;
    private final BeamBreak beamBreak;

    private static AlphaManipulator instance;

    public static AlphaManipulator getInstance() {
        if (instance == null) {
            instance = new AlphaManipulator();
        }

        return instance;
    }

    private AlphaManipulator() {
        motor = Constants.Manipulator.MOTOR.createSparkMax();
        beamBreak = Constants.Manipulator.BEAM_BREAK.createBeamBreak();
    }

    public boolean coralIn() {
        return beamBreak.isBlocked();
    }

    public Command run() {
        return runOnce(() -> {
            motor.set(Constants.Manipulator.SPEED);
        });
    }

    public Command intakeCoral() {
        return run(() -> {
            motor.set(Constants.Manipulator.SPEED);
        }).until(beamBreak.getBlockedSupplier()).andThen(stop());
    }

    public Command stop() {
        return runOnce(() -> {
            motor.set(0.0);
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            motor.set(-0.05);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral In", coralIn());
    }
}
