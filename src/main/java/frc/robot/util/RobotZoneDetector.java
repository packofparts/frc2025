package frc.robot.util;

import java.awt.geom.Path2D;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class RobotZoneDetector {
    
    private static final double[][][] BLUE_ZONES = {
        {{0.000,1.240}, {0.000,6.800}, {3.630,4.500}, {3.630,3.500}},
        {{0.000,1.240}, {3.630,3.500}, {4.500,3.000}, {4.500,0.000}, {1.700, 0.000}},
        {{4.500,0.000}, {4.500,3.000}, {5.350,3.500}, {8.200,1.400}, {8.200, 0.000}},
        {{8.200,1.400}, {5.350,3.500}, {5.350,4.500}, {8.200,6.200}},
        {{4.500,5.000}, {4.500,8.000}, {8.200, 8.000}, {8.200,6.200}, {5.350,4.500}},
        {{4.500,5.000}, {3.630,4.500}, {0.000,6.800}, {1.775,8.000}, {4.500,8.000}}
    };

    private static final double[][][] RED_ZONES = {
        {{17.530,1.240}, {13.920,3.500}, {13.920,4.500}, {17.530,6.800}},
        {{13.920,4.500}, {13.060,5.000}, {13.060,8.000}, {15.750,8.000}, {17.530,6.800}},
        {{13.060,5.000}, {12.200,4.500}, {9.340,6.200}, {9.340, 8.000}, {13.060,8.000}},
        {{12.200,4.500}, {12.200,3.500}, {9.340,1.400}, {9.340,6.200}},
        {{13.060,3.000}, {13.060,0.000}, {9.340,0.000}, {9.340,1.400}, {12.200,3.500}},
        {{13.060,0.000}, {13.060,3.000}, {13.920,3.500}, {17.530,1.240}, {15.750,0.000}}
    };
    
    public static int getZone(double x, double y, boolean isBlueSide) {
        double[][][] zones = isBlueSide ? BLUE_ZONES : RED_ZONES;
        
        for (int i = 0; i < zones.length; i++) {
            if (isPointInPolygon(x, y, zones[i])) {
                return i + 1; // Zone numbers are 1-based
            }
        }
        return -1; // Not in any zone
    }

    private static boolean isPointInPolygon(double x, double y, double[][] polygon) {
        Path2D.Double path = new Path2D.Double();
        path.moveTo(polygon[0][0], polygon[0][1]);
        for (int i = 1; i < polygon.length; i++) {
            path.lineTo(polygon[i][0], polygon[i][1]);
        }
        path.closePath();
        return path.contains(x, y);
    }

    public static void main(String[] args) {
        boolean isBlueSide = true; // Change this to false for red side
        double x = 4.0, y = 4.0; // Example point
        int zone = getZone(x, y, isBlueSide);
        System.out.println("Point (" + x + ", " + y + ") is in Zone " + (zone == -1 ? "None" : zone));
    }

    public static String getPathNameForZone(int zoneID, boolean isLeft){
        switch (zoneID){
            case 1: return isLeft ? "PathToA" : "PathToB";
            case 2: return isLeft ? "PathToC" : "PathToD";
            case 3: return isLeft ? "PathToE" : "PathToF";
            case 4: return isLeft ? "PathToG" : "PathToH";
            case 5: return isLeft ? "PathToI" : "PathToJ";
            case 6: return isLeft ? "PathToK" : "PathToL";
        }
        return null;
    }

    public static Pose2d getReefTag(int zoneID, boolean isBlueSide){
        switch (zoneID){
            case 1: return isBlueSide ? getTagPose(18): getTagPose(7);
            case 2: return isBlueSide ? getTagPose(17): getTagPose(8);
            case 3: return isBlueSide ? getTagPose(22): getTagPose(9);
            case 4: return isBlueSide ? getTagPose(21): getTagPose(10);
            case 5: return isBlueSide ? getTagPose(20): getTagPose(11);
            case 6: return isBlueSide ? getTagPose(19): getTagPose(6);
            case -1: return isBlueSide ? getTagPose(18): getTagPose(7); // robot is outside all zones, which shouldn't be happening on field
            default: return isBlueSide ? getTagPose(18): getTagPose(7); // this case should never occur, but just here for safety
        }
    }

    public static Pose2d getTagPose(int tagID){
        Optional<Pose3d> thing = Constants.AutoAlign.APRIL_TAG_FIELD.getTagPose(tagID);
        return thing.isPresent() ? thing.get().toPose2d() : null;
    }

    public static Pose2d getLeftAlignPose(int zoneID, boolean isBlue){
        switch (zoneID){
            case 1: return isBlue ? new Pose2d(3.149, 4.190, new Rotation2d(0.0)) : new Pose2d(14.394, 3.865, new Rotation2d(0.0));
            case 2: return isBlue ? new Pose2d(3.680, 2.950, new Rotation2d(Math.PI/3)) : new Pose2d(13.864, 5.096, new Rotation2d(Math.PI/3));
            case 3: return isBlue ? new Pose2d(5.010, 2.780, new Rotation2d((2*Math.PI)/3)) : new Pose2d(12.543, 5.275, new Rotation2d((2*Math.PI)/3));
            case 4: return isBlue ? new Pose2d(5.820, 3.860, new Rotation2d(Math.PI)) : new Pose2d(11.722, 4.185, new Rotation2d(Math.PI));
            case 5: return isBlue ? new Pose2d(5.305, 5.096, new Rotation2d(-(2*Math.PI)/3)) : new Pose2d(12.260, 2.954, new Rotation2d(-(2*Math.PI)/3));
            case 6: return isBlue ? new Pose2d(3.962, 5.260, new Rotation2d(-Math.PI/3)) : new Pose2d(13.580, 2.775, new Rotation2d(-Math.PI/3));
            case -1: return isBlue ? new Pose2d(3.149, 4.190, new Rotation2d(0.0)) : new Pose2d();
            default: return isBlue ? new Pose2d(3.149, 4.190, new Rotation2d(0.0)) : new Pose2d();
        }
    }

    public static Pose2d getRightAlignPose(int zoneID, boolean isBlue){
        switch (zoneID){
            case 1: return isBlue ? new Pose2d(3.149, 3.872, new Rotation2d(0.0)) : new Pose2d(14.394, 4.190, new Rotation2d(0.0));
            case 2: return isBlue ? new Pose2d(3.970, 2.790, new Rotation2d(Math.PI/3)) : new Pose2d(13.573, 5.267, new Rotation2d(Math.PI/3));
            case 3: return isBlue ? new Pose2d(5.298, 2.954, new Rotation2d((2*Math.PI)/3)) : new Pose2d(12.252, 5.096, new Rotation2d((2*Math.PI)/3));
            case 4: return isBlue ? new Pose2d(5.820, 4.178, new Rotation2d(Math.PI)) : new Pose2d(11.722, 3.860, new Rotation2d(Math.PI));
            case 5: return isBlue ? new Pose2d(5.014, 5.260, new Rotation2d(-(2*Math.PI)/3)) : new Pose2d(12.550, 2.790, new Rotation2d(-(2*Math.PI)/3));
            case 6: return isBlue ? new Pose2d(3.679, 5.096, new Rotation2d(-Math.PI/3)) : new Pose2d(13.870, 2.950, new Rotation2d(-Math.PI/3));
            case -1: return isBlue ? new Pose2d(3.149, 3.872, new Rotation2d(0.0)) : new Pose2d(14.394, 4.190, new Rotation2d(0.0));
            default: return isBlue ? new Pose2d(3.149, 3.872, new Rotation2d(0.0)) : new Pose2d(14.394, 4.190, new Rotation2d(0.0));
        }
    }
}