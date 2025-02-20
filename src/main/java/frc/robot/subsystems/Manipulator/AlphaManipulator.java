package frc.robot.subsystems.Manipulator;

import frc.robot.AlphaConstants;
import frc.robot.util.TalonPivot;

public class AlphaManipulator extends BaseManipulator {
    private static AlphaManipulator instance;

    public static AlphaManipulator getInstance() {
        if (instance == null) {
            instance = new AlphaManipulator();
        }

        return instance;
    }

    private AlphaManipulator() {
        super(AlphaConstants.Intake.SPIN.createSparkMax(),
         new TalonPivot(AlphaConstants.Intake.PIVOT, 
         null, 
         AlphaConstants.Intake.GEAR_RATIO, 
         AlphaConstants.Intake.FF, 
         AlphaConstants.Intake.ENCODER, 
         AlphaConstants.Intake.TUNING_MODE, 
         "Alpha Manipulator"), 
         AlphaConstants.Intake.SPEED, 
         AlphaConstants.Manipulator.BEAM_BREAK);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
