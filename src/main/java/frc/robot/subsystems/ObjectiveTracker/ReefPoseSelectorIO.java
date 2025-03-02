
package frc.robot.subsystems.ObjectiveTracker;

public interface ReefPoseSelectorIO {
  public static class ReefPoseSelectorIOInputs {
    public long selectedReefPose = -1;
    public long coralLevel = -1;
  }

  public default void updateInputs(ReefPoseSelectorIOInputs inputs) {}

  public default void setSelected(long selected) {}

  public default void setCoralLevel(long level) {}
}