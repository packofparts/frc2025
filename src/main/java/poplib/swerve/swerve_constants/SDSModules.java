package poplib.swerve.swerve_constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public enum SDSModules {
    MK4i(
        (150.0 / 7.0), 
        6.12, 
        Units.MetersPerSecond.of(5.6), 
        Units.MetersPerSecondPerSecond.of(4), 
        Units.RadiansPerSecond.of(4 * Math.PI), 
        Units.RadiansPerSecondPerSecond.of(4)
    ),
    MK4(
        12.8, 
        6.12, 
        Units.MetersPerSecond.of(5.21208) , 
        Units.MetersPerSecondPerSecond.of(10),
        Units.RadiansPerSecond.of(4 * Math.PI),
        Units.RadiansPerSecondPerSecond.of(4)
    );

    public double angleGearRatio;
    public double driveGearRatio;
    public LinearVelocity maxSpeed;
    public LinearAcceleration maxAcceleration;
    public AngularVelocity maxAngularVelocity;
    public AngularAcceleration maxAngularAcceleration;

    private SDSModules(double angleGearRatio, double driveGearRatio, LinearVelocity maxSpeed, LinearAcceleration maxAcceleration, AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }
}