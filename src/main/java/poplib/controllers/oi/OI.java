package poplib.controllers.oi;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * Abstract class for OI control
*/
abstract public class OI {
    public static final double DEADBAND = 0.1;

    public OI() { }

    public abstract double getDriveTrainRotation();

    public abstract double getDriveTrainTranslationY();

    public abstract double getDriveTrainTranslationX();

    public abstract CommandGenericHID getDriverController();
    public abstract CommandGenericHID getOperatorController();

    protected double getRawAxis(int id, CommandGenericHID controller) {
        return controller.getHID().getRawAxis(id) * -1;
    }

    public Trigger getDriverButton(int id) {
        return getDriverController().button(id);
    }

    public Trigger getDriverTrigger(int id) {
        return getDriverController().axisGreaterThan(id, 0.5);
    }

    public Trigger getOperatorButton(int id) {
        return getOperatorController().button(id);
    }

    public Trigger getOperatorrigger(int id) {
        return getOperatorController().axisGreaterThan(id, 0.5);
    }
}
