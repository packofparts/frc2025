package poplib.swerve.swerve_modules;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import poplib.smart_dashboard.PIDTuning;
import poplib.swerve.swerve_constants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityDutyCycle drivePID;
    private VoltageOut driveVoltage;
    private PositionDutyCycle anglePID;
    private CoastOut coastOut;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityDutyCycle(0); 
        driveVoltage = new VoltageOut(0.0);
        anglePID = new PositionDutyCycle(lastAngle.getRotations()); 
        coastOut = new CoastOut();
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getRotations());
        lastAngle = getCanCoder();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS)); 
        angleMotor.setControl(anglePID.withPosition(angle.getRotations()));
    }

    @Override
    protected Angle getAngle() {
        return angleMotor.getPosition().getValue();
    }

    @Override
    protected Angle getDriveAngle() {
        return driveMotor.getPosition().getValue();
    }

    @Override
    protected AngularVelocity getDriveAngularVelocity() {
        return driveMotor.getVelocity().getValue();
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        angleMotor.setControl(anglePID.withPosition(0.0)); 
        driveMotor.setControl(driveVoltage.withOutput(voltage));
    }

    @Override
    protected Voltage getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }
}
