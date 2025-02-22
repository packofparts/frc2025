// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
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
    public Swerve swerve;

    public Elevator elevator;
    public Indexer indexer;
    public Intake intake;
    public Manipulator manipulator;
    
    public OI oi;
    public SendableChooser<Command> scoring;

    private boolean intaking;

    public RobotContainer() {
        // swerve = Swerve.getInstance();
        oi = XboxOI.getInstance();


        // elevator = Elevator.getInstance();
        // manipulator = Manipulator.getInstance();
        indexer = Indexer.getInstance();
        intake = Intake.getInstance();

        // scoring = new SendableChooser<>();
        // scoring.addOption("L1", elevatorScore(AlphaConstants.Elevator.SETPOINTS.L1));
        // scoring.addOption("L2", elevatorScore(AlphaConstants.Elevator.SETPOINTS.L2));
        // scoring.addOption("L3", elevatorScore(AlphaConstants.Elevator.SETPOINTS.L3));
        // SmartDashboard.putData(scoring);

        // swerve.setDefaultCommand(new TeleopSwerveDrive(swerve, oi));

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

        oi.getDriverButton(XboxController.Button.kY.value).onTrue(tobleIntake());

        // oi.getDriverButton(XboxController.Button.kB.value).onTrue(elevator.moveUp(0.4)).onFalse(elevator.stop());
        // oi.getDriverButton(XboxController.Button.kY.value).onTrue(elevator.moveDown(0.4)).onFalse(elevator.stop());

        // oi.getDriverButton(XboxController.Button.kX.value).onTrue(new ParallelCommandGroup(
        //     // intake.reverse(),
        //     // indexer.reverse(),
        //     manipulator.run()
        // )).onFalse(new ParallelCommandGroup(
        //     intake.stop(),
        //     indexer.stop(),
        //     manipulator.stop()
        // ));

        // oi.getDriverButton(XboxController.Button.kA.value).onTrue(elevatorScore(AlphaConstants.Elevator.SETPOINTS.L2));
    }

    public Command tobleIntake() {
        return new InstantCommand(() -> {
            (!intaking ? startIntaking() : stopIntaking()).schedule();
            intaking = !intaking;
        });
    }

    public Command startIntaking() {
    //    return new SequentialCommandGroup(
            // intake.moveWrist(Constants.Intake.SETPOINTS.CORAL_PICKUP.getSetpoint(), Constants.Intake.MAX_ERROR),
            return new ParallelCommandGroup(
                // manipulator.run(),
                indexer.run(),
                intake.run()
            );
        // );
    }

    public Command stopIntaking() {
    //    return intake.moveWrist(Constants.Intake.SETPOINTS.IDLE.getSetpoint(),Constants.Intake.MAX_ERROR).
    //         andThen(
                return new ParallelCommandGroup(
                    // manipulator.stop(),
                    indexer.stop(),
                    intake.stop()
                );
        // );
    }

    // public Command elevatorScore(AlphaConstants.Elevator.SETPOINTS setpoint) {
    //     return new InstantCommand(() -> System.out.println("Setpoint Moving: " + setpoint.getSetpoint())).
    //         andThen(elevator.moveElevator(setpoint.getSetpoint()).
    //         andThen(manipulator.run()).
    //         andThen(new WaitCommand(1.0)).
    //         andThen(manipulator.stop()).
    //         andThen(elevator.moveElevator(AlphaConstants.Elevator.SETPOINTS.IDLE.getSetpoint()))
    //     );
    // }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
