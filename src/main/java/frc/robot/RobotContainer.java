// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.AlphaElevator;
import frc.robot.subsystems.Indexer.AlphaIndexer;
import frc.robot.subsystems.Intake.AlphaIntake;
import frc.robot.subsystems.Manipulator.AlphaManipulator;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import poplib.swerve.commands.TeleopSwerveDrive;
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
    public final AlphaElevator elevator;
    public final AlphaIndexer indexer;
    public final AlphaIntake intake;
    public final AlphaManipulator manipulator;
    public final OI oi;
    public final SendableChooser<Command> scoring;

    private boolean intaking;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        oi = XboxOI.getInstance();
        elevator = frc.robot.subsystems.Elevator.AlphaElevator.getInstance();
        manipulator = AlphaManipulator.getInstance();
        indexer = AlphaIndexer.getInstance();
        intake = AlphaIntake.getInstance();

        scoring = new SendableChooser<>();
        scoring.addOption("L1", elevatorScore(Constants.Elevator.SETPOINTS.L1));
        scoring.addOption("L2", elevatorScore(Constants.Elevator.SETPOINTS.L2));
        scoring.addOption("L3", elevatorScore(Constants.Elevator.SETPOINTS.L3));
        SmartDashboard.putData(scoring);

        swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));

        intaking = false;
        configureBindings();
    }

    public boolean getIntaking() {
        return intaking;
    }


    private void configureBindings() {
        // SysIdSwerve sys = new SysIdSwerve(swerve);
        // SysIdSwerve sys = new SysIdSwerve(swerve);

        // oi.getDriverButton(XboxController.Button.kA.value).whileTrue(sys.sysIdQuasistatic(Direction.kForward));
        // oi.getDriverButton(XboxController.Button.kB.value).whileTrue(sys.sysIdDynamic(Direction.kForward));

        oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.reZero());

        oi.getDriverButton(XboxController.Button.kB.value).onTrue(tobleIntake());

        oi.getDriverButton(XboxController.Button.kX.value).onTrue(new ParallelCommandGroup(
            // intake.reverse(),
            // indexer.reverse(),
            manipulator.run()
        )).onFalse(new ParallelCommandGroup(
            intake.stop(),
            indexer.stop(),
            manipulator.stop()
        ));

        oi.getDriverButton(XboxController.Button.kA.value).onTrue(elevatorScore(Constants.Elevator.SETPOINTS.L2));
    }

    public Command tobleIntake() {
        return new InstantCommand(() -> {
            (!intaking ? startIntaking() : stopIntaking()).schedule();
            intaking = !intaking;
        });
    }

    public Command startIntaking() {
       return new SequentialCommandGroup(
            intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR),
            new ParallelCommandGroup(
                manipulator.run(),
                indexer.run(),
                intake.run()
            )
        ); 
    }

    public Command stopIntaking() {
       return intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(), Constants.Intake.MAX_ERROR).
            andThen(
                new ParallelCommandGroup(
                    manipulator.stop(),
                    indexer.stop(),
                    intake.stop()
                )
        );
    }

    public Command elevatorScore(Constants.Elevator.SETPOINTS setpoint) {
        return new InstantCommand(() -> System.out.println("Setpoint Moving: " + setpoint.getSetpoint())).
            andThen(elevator.moveElevator(setpoint.getSetpoint()).
            andThen(manipulator.run()).
            andThen(new WaitCommand(1.0)).
            andThen(manipulator.stop()).
            andThen(elevator.moveElevator(Constants.Elevator.SETPOINTS.IDLE.getSetpoint()))
        );
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
