package frc.robot.subsystems.ObjectiveTracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import org.littletonrobotics.frc2023.subsystems.leds.Leds;
// import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.ObjectiveTracker.ReefPoseSelectorIO.ReefPoseSelectorIOInputs;

public class ObjectiveTracker extends SubsystemBase{

  private final ReefPoseSelectorIO selectorIO;
  private final ReefPoseSelectorIOInputs selectorInputs = new ReefPoseSelectorIOInputs();
  public final Objective objective = new Objective();

  public static class Objective {
    public ReefLevel reefLevel;
    public ReefBranch reefBranch;
    public ReefLocation reefLocation;

    public Objective(
        ReefLevel reefLevel,
        ReefBranch reefBranch,
        ReefLocation reefLocation) {
      this.reefLevel = reefLevel;
      this.reefBranch = reefBranch;
      this.reefLocation = reefLocation;
    }

    public Objective() {
      this(ReefLevel.ONE, ReefBranch.A, ReefLocation.MIDDLE);
    }
  }

  public ObjectiveTracker(ReefPoseSelectorIO selectorIO) {
    System.out.println("[Init] Creating ObjectiveTracker");
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("ReefPoseSelector", selectorInputs);

    int reefsLocation = (int) selectorInputs.selectedReefPose; //between 0 to 143 or is -1
    // Read updates from selector
    if (reefsLocation != -1) {
      int reef = reefsLocation / 48; //(reef 0: left, 1: middle, 2: right)
      int reefLocation = reefsLocation % 48; //position within the reef
      int reefLevel = reefLocation / 12;
      int reefBranch = reefLocation % 12; //A to K

      //test if there would be differences between different alliances
      this.objective.reefLevel = ReefLevel.values()[reefLocation / 12];
      this.objective.reefBranch = ReefBranch.values()[reefLocation % 12];
      if (reefLocation % 3 == 0) {
        objective.reefLocation = ReefLocation.LEFT;
      } else if (reefLocation % 3 == 1) {
        objective.reefLocation = ReefLocation.LEFT;
      } else {
        objective.reefLocation = ReefLocation.LEFT;
      }
    }

    selectorIO.setSelected(reefsLocation);

    // Send current node as text
    {
      String text = "";
      switch (objective.reefLevel) {
        case ONE: 
          text += "L1, "; 
          break;
        case TWO: 
          text += "L2, "; 
          break;
        case THREE: 
          text += "L3, "; 
          break;
        case FOUR: 
          text += "L4, "; 
          break;
      }
      SmartDashboard.putString("Selected Level", text);
    }

    // Log state
    Logger.getInstance().recordOutput("ObjectiveTracker/ReefLevel", objective.reefLevel);
    Logger.getInstance().recordOutput("ObjectiveTracker/ReefBranch", objective.reefBranch);
    Logger.getInstance().recordOutput("ObjectiveTracker/ReefPos", objective.reefLocation);
  }

  /** Shifts the selected pos in the selector by one position. */
  public void shiftPos(Direction direction) {
    switch (direction) {
      case UP:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          switch(objective.reefLevel) {
            case ONE: objective.reefLevel = ReefLevel.TWO; 
            case TWO: objective.reefLevel = ReefLevel.THREE;
            case THREE: objective.reefLevel = ReefLevel.FOUR;
            case FOUR: {};
          }
        }
        else {
          switch(objective.reefLevel) {
            case ONE: {}; 
            case TWO: objective.reefLevel = ReefLevel.TWO;
            case THREE: objective.reefLevel = ReefLevel.THREE;
            case FOUR: objective.reefLevel = ReefLevel.FOUR;
          }
        }
        break;

      case DOWN:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          switch(objective.reefLevel) {
            case ONE: {}; 
            case TWO: objective.reefLevel = ReefLevel.TWO;
            case THREE: objective.reefLevel = ReefLevel.THREE;
            case FOUR: objective.reefLevel = ReefLevel.FOUR;
          }
        }
        else {
          switch(objective.reefLevel) {
            case ONE: objective.reefLevel = ReefLevel.TWO; 
            case TWO: objective.reefLevel = ReefLevel.THREE;
            case THREE: objective.reefLevel = ReefLevel.FOUR;
            case FOUR: {};
          }
        }
        break;

      case LEFT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          switch(objective.reefBranch) {
            case A: {};
            case B: objective.reefBranch = ReefBranch.A; 
            case C: objective.reefBranch = ReefBranch.B; 
            case D: objective.reefBranch = ReefBranch.C; 
            case E: objective.reefBranch = ReefBranch.D; 
            case F: objective.reefBranch = ReefBranch.E; 
            case G: objective.reefBranch = ReefBranch.F; 
            case H: objective.reefBranch = ReefBranch.G; 
            case I: objective.reefBranch = ReefBranch.H; 
            case J: objective.reefBranch = ReefBranch.I; 
            case K: objective.reefBranch = ReefBranch.J;

          }
        } else {
          switch(objective.reefBranch) {
            case A: objective.reefBranch = ReefBranch.B; 
            case B: objective.reefBranch = ReefBranch.C; 
            case C: objective.reefBranch = ReefBranch.D; 
            case D: objective.reefBranch = ReefBranch.E; 
            case E: objective.reefBranch = ReefBranch.F; 
            case F: objective.reefBranch = ReefBranch.G; 
            case G: objective.reefBranch = ReefBranch.H; 
            case H: objective.reefBranch = ReefBranch.I; 
            case I: objective.reefBranch = ReefBranch.J; 
            case J: objective.reefBranch = ReefBranch.K; 
            case K: {};
          }
        }
        break;

      case RIGHT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          switch(objective.reefBranch) {
            case A: objective.reefBranch = ReefBranch.B; 
            case B: objective.reefBranch = ReefBranch.C; 
            case C: objective.reefBranch = ReefBranch.D; 
            case D: objective.reefBranch = ReefBranch.E; 
            case E: objective.reefBranch = ReefBranch.F; 
            case F: objective.reefBranch = ReefBranch.G; 
            case G: objective.reefBranch = ReefBranch.H; 
            case H: objective.reefBranch = ReefBranch.I; 
            case I: objective.reefBranch = ReefBranch.J; 
            case J: objective.reefBranch = ReefBranch.K; 
            case K: {};
          }
        } else {
          switch(objective.reefBranch) {
            case A: {};
            case B: objective.reefBranch = ReefBranch.A; 
            case C: objective.reefBranch = ReefBranch.B; 
            case D: objective.reefBranch = ReefBranch.C; 
            case E: objective.reefBranch = ReefBranch.D; 
            case F: objective.reefBranch = ReefBranch.E; 
            case G: objective.reefBranch = ReefBranch.F; 
            case H: objective.reefBranch = ReefBranch.G; 
            case I: objective.reefBranch = ReefBranch.H; 
            case J: objective.reefBranch = ReefBranch.I; 
            case K: objective.reefBranch = ReefBranch.J;
          }
        }
        break;
    }
  }

  /** Command factory to shift the selected position in the selector by one position. */
  public Command shiftPosCommand(Direction direction) {
    return new InstantCommand(() -> shiftPos(direction))
        .andThen(
            Commands.waitSeconds(0.3),
            Commands.repeatingSequence(
                new InstantCommand(() -> shiftPos(direction)), new WaitCommand(0.1)))
        .ignoringDisable(true);
  }

  public static enum ReefLevel {
    ONE, TWO, THREE, FOUR
  }

  public static enum ReefBranch {
    A, B, C, D, E, F, G, H, I, J, K
  }

  public static enum ReefLocation {
    LEFT, MIDDLE, RIGHT
  }

}
