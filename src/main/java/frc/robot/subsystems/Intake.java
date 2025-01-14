package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
    

    private Intake() {
      pivotMotor = new SparkMax(0, null);
      rollerMotor = new SparkMax(0, null);
    }

    public void runPivot() {
      pivotMotor.set(1);
    }

    public void reversePivot() {
      pivotMotor.set(-1);
    }

    public void stopPivot() {
      pivotMotor.set(0);
    }

    public void runRoller() {
      rollerMotor.set(1);
    }

    public void reverseMotor() {
      rollerMotor.set(-1);
    }

    public void stopRoller() {
      rollerMotor.set(0);
    }

    @Override
    public void periodic() {
    }
}
