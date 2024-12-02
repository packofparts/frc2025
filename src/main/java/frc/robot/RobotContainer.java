// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve;
import POPLib.Controllers.OI.OI;
import POPLib.Controllers.OI.XboxOI;
import POPLib.Swerve.Commands.TeleopSwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Swerve swerve;
  OI oi;

  public RobotContainer() {
    swerve = Swerve.getInstance();
    oi = XboxOI.getInstance();
    swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));
    configureBindings();
  }


  private void configureBindings() {
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
