package poplib.swerve.swerve_modules;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import poplib.smart_dashboard.PIDTuning;
import poplib.swerve.swerve_constants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final VelocityDutyCycle drivePID;
    private final VoltageOut driveVoltage;

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);
        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        resetToAbsolute();

        drivePID = new VelocityDutyCycle(0); 
        driveVoltage = new VoltageOut(0.0);
        lastAngle = getPose().angle;
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
        lastAngle = getCanCoder();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS)); 
        anglePID.setReference(angleRadians.getRotations(), SparkMax.ControlType.kPosition);
    }

    @Override
    protected Angle getAngle() {
        return Units.Rotations.of(angleEncoder.getPosition());
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }


    @Override
    public void runSysIdRoutine(double voltage) {
        driveMotor.setControl(driveVoltage.withOutput(voltage)); 
        anglePID.setReference(0.0, SparkMax.ControlType.kPosition);    
    }


    @Override
    protected Voltage getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }


    @Override
    protected Angle getDriveAngle() {
        return driveMotor.getPosition().getValue();
    }


    @Override
    protected AngularVelocity getDriveAngularVelocity() {
        return driveMotor.getVelocity().getValue();
    }
}