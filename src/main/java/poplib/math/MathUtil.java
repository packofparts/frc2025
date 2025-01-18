package poplib.math;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import poplib.sensors.absolute_encoder.AbsoluteEncoder;
import poplib.smart_dashboard.TunableNumber;


public class MathUtil {
    public static double clamp(double max, double min, double currVal) {
        if (currVal > max) {
            return max;
        } else if (currVal < min) {
            return min;
        }

        return currVal;
    } 

    public static double clamp(double max, double min, TunableNumber currVal) {
        return clamp(max, min, currVal.get());
    } 

    public static double getError(double target, double setpoint) {
        return Math.abs(target - setpoint);
    }

    public static double getError(SparkMax target, double setpoint) {
        return getError(target.getEncoder().getPosition(), setpoint);
    }

    public static double getError(SparkMax target, TunableNumber setpoint) {
        return getError(target.getEncoder().getPosition(), setpoint.get());
    }

    public static double getError(TalonFX target, double setpoint) {
        return getError(target.getPosition().getValueAsDouble(), setpoint);
    }

    public static double getError(TalonFX target, TunableNumber setpoint) {
        return getError(target.getPosition().getValueAsDouble(), setpoint.get());
    }

    public static double getError(SparkMax target, AbsoluteEncoder setpoint) {
        return getError(target.getEncoder().getPosition(), setpoint.getPosition().getDegrees());
    }

}
