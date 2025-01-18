package poplib.controllers.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxOI extends OI {
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    public static XboxOI instance = null;

    public static XboxOI getInstance() {
        if (instance == null) {
            instance = new XboxOI();
        }

        return instance;
    }

    public static final class OIConstants  {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;


        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public XboxOI() {
        driverController = new CommandXboxController(OIConstants.DRIVE_PORT);
        operatorController = new CommandXboxController(OIConstants.OPERATOR_PORT);
    }

    @Override
    public double getDriveTrainRotation() {
        return getRawAxis(OIConstants.DRIVE_ROTATE, driverController);
    }

    @Override
    public double getDriveTrainTranslationY() {
        return getRawAxis(OIConstants.DRIVE_TRANSLATION_Y, driverController);
    }

    @Override
    public double getDriveTrainTranslationX() {
        return getRawAxis(OIConstants.DRIVE_TRANSLATION_X, driverController);
    }

    @Override
    public CommandGenericHID getDriverController() {
        return driverController;
    }

    @Override
    public CommandGenericHID getOperatorController() {
        return operatorController;
    }
}
