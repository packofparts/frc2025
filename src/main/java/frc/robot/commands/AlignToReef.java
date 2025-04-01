// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToReef extends Command {
  private final Swerve drive;
  private final Pose2d targetPose;
  
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private static final double POSITION_TOLERANCE = 0.02;  // Meters
  private static final double ANGLE_TOLERANCE = Math.toRadians(1.0);  // Radians

  private final CommandGenericHID driverController;

  /** Creates a new AlignToReef. */
  public AlignToReef(Swerve swerve, Pose2d targetPose, CommandGenericHID theDriverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = swerve;
    this.targetPose = targetPose;

    xController = new PIDController(1.0, 0.0, 0.0);
    yController = new PIDController(1.0, 0.0, 0.0);
    thetaController = new ProfiledPIDController(2.0, 0.0, 0.0, Constants.AutoAlign.constraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(ANGLE_TOLERANCE);

    this.driverController = theDriverController;

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset controllers
    xController.reset();
    yController.reset();
    thetaController.reset(drive.getOdomPose().getRotation().getRadians());
    
    // Log initial values for debugging
    SmartDashboard.putNumber("Target X", targetPose.getX());
    SmartDashboard.putNumber("Target Y", targetPose.getY());
    SmartDashboard.putNumber("Target Theta", targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drive.getOdomPose();

    double xVelo = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelo = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelo = thetaController.calculate(
        currentPose.getRotation().getRadians(), 
        targetPose.getRotation().getRadians()
    );

    // Convert from field-relative to robot-relative speeds
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xVelo, yVelo, thetaVelo,
      currentPose.getRotation()
    );

    drive.driveChassis(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveChassis(new ChassisSpeeds(0, 0, 0)); // Stop when done
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if the driver is giving input (adjust threshold if needed)
    boolean manualOverride = Math.abs(driverController.getRawAxis(XboxController.Axis.kLeftX.value)) > 0.25 || // Left/Right
    Math.abs(driverController.getRawAxis(XboxController.Axis.kLeftY.value)) > 0.25 || // Forward/Backward
    Math.abs(driverController.getRawAxis(XboxController.Axis.kRightX.value)) > 0.25;   // Rotation

    boolean reachedSetpoints = xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal();
    return reachedSetpoints || manualOverride;
  }
}