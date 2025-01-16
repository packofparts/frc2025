// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import POPLib.Controllers.OI.OI;
import POPLib.Controllers.OI.XboxOI;
import POPLib.Swerve.Commands.SysIdSwerve;
import POPLib.Swerve.Commands.TeleopSwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Swerve swerve;
  // Elevator elevator;
  OI oi;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerve = Swerve.getInstance();
    oi = XboxOI.getInstance();
    // elevator = Elevator.getInstance();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.addOption("line_2meters", makeAuto("line_2meters"));
    swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));
    configureBindings();
  }


  private void configureBindings() {
    SysIdSwerve sys = new SysIdSwerve(swerve);

    oi.getDriverButton(XboxController.Button.kA.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
    oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

    // oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.reZero());
    // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown()).onFalse(elevator.stop());
  }

  private Command makeAuto(String path) {
    return new PathPlannerAuto(path);
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
