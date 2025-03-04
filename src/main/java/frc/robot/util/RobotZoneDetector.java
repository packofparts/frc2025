package frc.robot.util;

import java.awt.geom.Path2D;

import edu.wpi.first.math.geometry.Pose2d;
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
        return Constants.AutoAlign.APRIL_TAG_FIELD.getTagPose(tagID).get().toPose2d();
    }
}