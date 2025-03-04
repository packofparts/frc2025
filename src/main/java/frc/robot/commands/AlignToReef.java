// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AlignToReef extends Command {
  private final Swerve drive;
  private final Pose2d targetPose;
  
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private static final double POSITION_TOLERANCE = 0.05;  // Meters
  private static final double ANGLE_TOLERANCE = Math.toRadians(2.0);  // Radians

  /** Creates a new AlignToReef. */
  public AlignToReef(Swerve swerve, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = swerve;
    this.targetPose = targetPose;

    xController = new PIDController(1.0, 0.0, 0.0);
    yController = new PIDController(1.0, 0.0, 0.0);
    thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, Constants.AutoAlign.constraints);

    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    thetaController.setTolerance(ANGLE_TOLERANCE);

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = this.drive.getOdomPose();  // Get the robot's current position

    double xVelo = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelo = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelo = thetaController.calculate(
        currentPose.getRotation().getRadians(), 
        targetPose.getRotation().getRadians()
    );

    drive.driveRobotOriented(new Translation2d(xVelo, yVelo), thetaVelo);
    // drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelo, yVelo, thetaVelo, drive.getGyroRotation()));
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

    drive.driveRobotOriented(new Translation2d(xVelo, yVelo), thetaVelo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveRobotOriented(new Translation2d(0.0, 0.0), 0.0); // Stop the robot when finished
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal();
  }
}