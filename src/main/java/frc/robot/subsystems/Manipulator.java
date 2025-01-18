package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import POPLib.Sensors.BeamBreak.BeamBreak;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
    private final SparkMax motor;
    private final BeamBreak beamBreak;

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }

    private Manipulator() {
        motor = Constants.Manipualtor.MOTOR.createSparkMax();
        beamBreak = Constants.Manipualtor.BEAM_BREAK.createBeamBreak();
    }

    public boolean coralIn() {
        return beamBreak.isBlocked();
    }

    public Command run() {
        return runOnce(() -> {
            motor.set(Constants.Manipualtor.SPEED);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.set(0.0);
        });
    }

    public Command reverse() {
        return runOnce(() -> {
            motor.set(-1 * Constants.Manipualtor.SPEED);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral In", coralIn());
    }
}
