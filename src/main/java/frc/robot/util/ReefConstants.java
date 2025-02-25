package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefConstants {

    /*       Source 1
     * D               
     * R                 /\
     * I               K/  \J
     * V              L/    \I
     * E              /      \
     * R             |        |
     *              A|        |H
     *               |        |
     * S            B|        |G
     * T             |        |
     * A              \      /
     * T              C\    /F
     * I               D\  /E
     * O                 \/
     * N               
           Source 2
     */

    public enum AutonPoses{
        // reef poses
        A(new Pose2d(3.160, 4.190, new Rotation2d(0))),  // Positions based on pathplanner
        B(new Pose2d(3.160, 3.850, new Rotation2d(0))),   // Measured on blue side, but I think it should flip automatically for red
        C(new Pose2d(3.700, 2.950, new Rotation2d(Math.PI/3))),
        D(new Pose2d(3.980, 2.790, new Rotation2d(Math.PI/3))),
        E(new Pose2d(5.010, 2.790, new Rotation2d((2*Math.PI)/3))),
        F(new Pose2d(5.290, 2.950, new Rotation2d((2*Math.PI)/3))),
        G(new Pose2d(5.800, 3.860, new Rotation2d(Math.PI))),
        H(new Pose2d(5.800, 4.200, new Rotation2d(Math.PI))),
        I(new Pose2d(5.300, 5.090, new Rotation2d(-(2*Math.PI)/3))),
        J(new Pose2d(5.000, 5.260, new Rotation2d(-(2*Math.PI)/3))),
        K(new Pose2d(5.0, 6.0, new Rotation2d(-Math.PI/3))),
        L(new Pose2d(5.0, 6.0, new Rotation2d(-Math.PI/3))),

        // other poses
        // SOURCE1_LEFT(),
        SOURCE1_MIDDLE(new Pose2d(1.800, 6.320, new Rotation2d((2*Math.PI)/3))),
        // SOURCE1_RIGHT(),
        // SOURCE2_LEFT(),
        // SOURCE2_MIDDLE(),
        // SOURCE2_RIGHT(),
        // START_LEFT(),
        // START_MIDDLE(),
        START_RIGHT(new Pose2d(7.580, 5.000, new Rotation2d(Math.PI)));

        private final Pose2d pose;

        AutonPoses(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }
}