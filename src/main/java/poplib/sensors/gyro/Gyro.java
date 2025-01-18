package poplib.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Defines an structure for Gyroscope classes to follow.
 */
public abstract class Gyro {
    public abstract Angle getNormalizedAngle();

    public abstract Rotation2d getNormalizedRotation2dAngle();

    public abstract void zeroGyro();

    public abstract void setAngle(Rotation2d newAngle);

    public abstract Angle getRoll();

    public abstract Angle getPitch();

    public abstract Angle getYaw();

    public abstract AngularVelocity getAngularVelo();

    public abstract Angle getLatencyCompensatedAngle();
}
