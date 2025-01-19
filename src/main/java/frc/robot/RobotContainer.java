// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import poplib.swerve.commands.SysIdSwerve;
import poplib.swerve.commands.TeleopSwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Swerve swerve;
  Elevator elevator;
  Indexer indexer;
  Intake intake;
  Manipulator manipulator;
  OI oi;

  public RobotContainer() {
    // swerve = Swerve.getInstance();
    oi = XboxOI.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    indexer = Indexer.getInstance();
    intake = Intake.getInstance();
    // swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));
    configureBindings();
  }


  private void configureBindings() {
    // SysIdSwerve sys = new SysIdSwerve(swerve);

    // oi.getDriverButton(XboxController.Button.kA.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
    // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

    oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.reZero());
    oi.getDriverButton(XboxController.Button.kB.value).onTrue(new ParallelCommandGroup(
      manipulator.run(),
      indexer.run(),
      intake.run()
    )).onFalse(new ParallelCommandGroup(
      manipulator.stop(),
      indexer.stop(),
      intake.stop()
    ));

    oi.getDriverButton(XboxController.Button.kX.value).onTrue(new ParallelCommandGroup(
      intake.reverse()
    )).onFalse(new ParallelCommandGroup(
      intake.stop()
    ));
    
    oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown(0.5)).onFalse(elevator.stop());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
