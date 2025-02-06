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
import poplib.swerve.commands.TeleopSwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final Swerve swerve;
    // public final Elevator elevator;
    // public final Indexer indexer;
    // public final Intake intake;
    // public final Manipulator manipulator;
    public final OI oi;
    // public final SendableChooser<Command> scoring;
  
    private boolean intaking;

  public RobotContainer() {
    // swerve = Swerve.getInstance();
    swerve = Swerve.getInstance();
    oi = XboxOI.getInstance();
    // elevator = Elevator.getInstance();
    // manipulator = Manipulator.getInstance();
    // indexer = Indexer.getInstance();
    // intake = Intake.getInstance();
    // scoringPos = new TunableNumber("Elevator Scoring Position", 0, true);
    // swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));
    // configureBindings();
  }

    public boolean getIntaking() {
        return intaking;
    }

    public Command thing() {
        return swerve.moveToPoseVision(null);
    }


    // private void configureBindings() {
    //     // SysIdSwerve sys = new SysIdSwerve(swerve);
    //     // SysIdSwerve sys = new SysIdSwerve(swerve);

    // oi.getDriverButton(XboxController.Button.kA.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
    // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

    // oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.reZero());
    // oi.getDriverButton(XboxController.Button.kB.value).onTrue(new ParallelCommandGroup(
    //   manipulator.run(),
    //   indexer.run(),
    //   intake.run()
    // )).onFalse(new ParallelCommandGroup(
    //   manipulator.stop(),
    //   indexer.stop(),
    //   intake.stop()
    // ));

    // oi.getDriverButton(XboxController.Button.kX.value).onTrue(new ParallelCommandGroup(
    //   intake.reverse()
    // )).onFalse(new ParallelCommandGroup(
    //   intake.stop()
    // ));
    
    // oi.getDriverButton(XboxController.Button.kX.value).onTrue(elevator.moveDown(0.5)).onFalse(elevator.stop());

    // }

  // public Command intakeCoral() {
  //   Elevator elevator = Elevator.getInstance();
  //   Indexer indexer = Indexer.getInstance();
  //   Manipulator manipulator = Manipulator.getInstance();
  //   return elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint(), Constants.Elevator.MAX_ERROR).
  //   alongWith(intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR)).
  //   andThen(intake.run()).
  //   alongWith(indexer.run()).
  //   alongWith(manipulator.run()).
  //   until(manipulator::coralIn).
  //   andThen(manipulator.stop()).
  //   alongWith(indexer.stop()).
  //   alongWith(intake.stop()).
  //   andThen(intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(), Constants.Intake.MAX_ERROR));
  // }

  // public Command scoreGamepiece() {
  //     double setpoint;
  //     switch ((int) scoringPos.get()) {
  //         case 1:
  //             setpoint = Constants.Elevator.SETPOINTS.L1.getSetpoint();
  //             break;
  //         case 2:
  //             setpoint = Constants.Elevator.SETPOINTS.L2.getSetpoint();
  //             break;
  //         case 3:
  //             setpoint = Constants.Elevator.SETPOINTS.L3.getSetpoint();
  //             break;
  //         default:
  //             System.out.println("please give an actual scoring location");
  //             return intake.stop();  // basically i just want to return a command that does nothing
  //     }
  //     return elevator.moveElevator(setpoint, Constants.Elevator.MAX_ERROR).
  //     andThen(manipulator.run()).until(manipulator::coralIn).andThen(manipulator.stop()).
  //     andThen(elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint(), Constants.Elevator.MAX_ERROR));
  // }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return null;
    }
}
