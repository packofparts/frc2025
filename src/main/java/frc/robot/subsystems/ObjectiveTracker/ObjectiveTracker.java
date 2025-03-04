package frc.robot.subsystems.ObjectiveTracker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ObjectiveTracker.ReefPoseSelectorIO.ReefPoseSelectorIOInputs;

public class ObjectiveTracker extends SubsystemBase{

  private final ReefPoseSelectorIO selectorIO;
  private final ReefPoseSelectorIOInputs selectorInputs = new ReefPoseSelectorIOInputs();
  public final Objective objective = new Objective();

  public static class Objective {
    public ReefLevel reefLevel;
    public ReefBranch reefBranch;

    public Objective(
        ReefLevel reefLevel,
        ReefBranch reefBranch) {
      this.reefLevel = reefLevel;
      this.reefBranch = reefBranch;
    }

    public Objective() {
      this(ReefLevel.ONE, ReefBranch.A);
    }
  }

  public ObjectiveTracker(ReefPoseSelectorIO selectorIO) {
    System.out.println("[Init] Creating ObjectiveTracker");
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    SmartDashboard.putNumber("ReefPoseSelector", selectorInputs.selectedReefPose);

    int reefLocation = (int) selectorInputs.selectedReefPose; //between 0 to 47 or is -1
    // Read updates from selector
    if (reefLocation != -1) {
      //TODO: test if there would be differences between different alliances bc not implemented
      this.objective.reefLevel = ReefLevel.values()[reefLocation / 12]; //0 to 3
      this.objective.reefBranch = ReefBranch.values()[reefLocation % 12]; //A to K
    }

    selectorIO.setSelected(reefLocation);

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
    SmartDashboard.putString("ReefLevel", objective.reefLevel.toString());
    SmartDashboard.putString("ReefBranch", objective.reefBranch.toString());
  }

  /** Shifts the selected pos in the selector by one position. */
  public void shiftPos(Direction direction) {
    switch (direction) {
      case UP:
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
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
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
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
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
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
            case L: objective.reefBranch = ReefBranch.K;
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
            case K: objective.reefBranch = ReefBranch.L;
            case L: {};
          }
        }
        break;

      case RIGHT:
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
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
            case K: objective.reefBranch = ReefBranch.L;
            case L: {};
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
            case L: objective.reefBranch = ReefBranch.K;
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
    A, B, C, D, E, F, G, H, I, J, K, L
  }

  public static enum Direction {
    LEFT,
    RIGHT,
    UP,
    DOWN
  }

}
