package poplib.swerve.swerve_modules;

import com.revrobotics.spark.SparkMax;
import poplib.smart_dashboard.PIDTuning;
import poplib.swerve.swerve_constants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleNeo extends SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;


    public SwerveModuleNeo(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveNeo();
        angleMotor = constants.getAngleNeo();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        angleMotor.getClosedLoopController().setReference(angleRadians.getRotations(), SparkMax.ControlType.kPosition);
        driveMotor.getClosedLoopController().setReference(velocityMPS, SparkMax.ControlType.kVelocity);
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
    }

    @Override
    protected Angle getAngle() {
        return Units.Rotations.of(angleMotor.getEncoder().getPosition());
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        angleMotor.getClosedLoopController().setReference(0.0, SparkMax.ControlType.kPosition);
        driveMotor.setVoltage(voltage);
    }

    @Override
    protected Voltage getDriveVoltage() {
        // TOOD: Verify correctness
        return Units.Volts.of(driveMotor.getBusVoltage());
    }

    @Override
    protected Angle getDriveAngle() {
        return Units.Rotations.of(driveMotor.getEncoder().getPosition());
    }

    @Override
    protected AngularVelocity getDriveAngularVelocity() {
        return Units.RotationsPerSecond.of(driveMotor.getEncoder().getVelocity());
    }
}
