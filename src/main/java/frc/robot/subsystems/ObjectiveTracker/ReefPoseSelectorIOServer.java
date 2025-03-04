package frc.robot.subsystems.ObjectiveTracker;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;

public class ReefPoseSelectorIOServer implements ReefPoseSelectorIO {
  private final IntegerPublisher reefPosePublisher; //change to string or character?
  private final IntegerSubscriber reefPoseSubscriber;
  private final IntegerPublisher timePublisher;
  private final BooleanPublisher isAutoPublisher;

  public ReefPoseSelectorIOServer() {
    System.out.println("[Init] Creating ReefPoseSelectorIOServer");

    // Create publisher and subscriber
    var table = NetworkTableInstance.getDefault().getTable("reefPoseSelector");
    reefPosePublisher = table.getIntegerTopic("reefPose_robot_to_dashboard").publish();
    reefPoseSubscriber = table.getIntegerTopic("reefPose_dashboard_to_robot").subscribe(-1);

    timePublisher = table.getIntegerTopic("match_time").publish();
    isAutoPublisher = table.getBooleanTopic("is_auto").publish();

    // Start server
    var app =
        Javalin.create(
            config -> {
              config.staticFiles.add(
                  Paths.get(
                          Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                          "reefPoseselector")
                      .toString(),
                  Location.EXTERNAL);
            });
    app.start(5800);
  }

  public void updateInputs(ReefPoseSelectorIOInputs inputs) {
    timePublisher.set((long) Math.ceil(Math.max(0.0, DriverStation.getMatchTime())));
    isAutoPublisher.set(DriverStation.isAutonomous());
    for (var value : reefPoseSubscriber.readQueueValues()) {
      inputs.selectedReefPose = value;
    }
  }

  public void setSelected(long selected) {
    reefPosePublisher.set(selected);
  }
}

