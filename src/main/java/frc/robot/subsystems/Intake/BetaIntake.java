package frc.robot.subsystems.Intake;

import frc.robot.BetaConstants;
import frc.robot.util.TalonPivot;

public class BetaIntake extends BaseIntake {
    private static BetaIntake instance;

    public static BetaIntake getInstance() {
        if (instance == null) {
            instance = new BetaIntake();
        }

        return instance;
    }

    private BetaIntake() {
        super(BetaConstants.Intake.SPIN.createTalon(),
         new TalonPivot(BetaConstants.Intake.PIVOT, 
         null, 
         BetaConstants.Intake.GEAR_RATIO, 
         BetaConstants.Intake.FF, 
         BetaConstants.Intake.ENCODER, 
         BetaConstants.Intake.TUNING_MODE, 
         "Beta Intake"), 
         BetaConstants.Intake.SPEED,
         BetaConstants.Intake.MAX_ERROR);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}