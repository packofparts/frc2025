package frc.robot.subsystems.Intake;

import frc.robot.AlphaConstants;
import poplib.subsytems.pivot.SparkPivot;

public class AlphaIntake extends BaseIntake {
    private static AlphaIntake instance;

    public static AlphaIntake getInstance() {
        if (instance == null) {
            instance = new AlphaIntake();
        }

        return instance;
    }

    private AlphaIntake() {
        super(AlphaConstants.Intake.SPIN.createSparkMax(), 
        new SparkPivot(AlphaConstants.Intake.PIVOT, 
        null, 
        AlphaConstants.Intake.GEAR_RATIO, 
        AlphaConstants.Intake.FF, 
        AlphaConstants.Intake.ENCODER, 
        AlphaConstants.Intake.TUNING_MODE, 
        "Alpha Intake"), 
        AlphaConstants.Intake.SPEED, 
        AlphaConstants.Intake.MAX_ERROR);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}