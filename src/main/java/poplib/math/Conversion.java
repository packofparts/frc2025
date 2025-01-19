package poplib.math;

public class Conversion {
    public static double RPSToMPS(double rps, double circumference) {
        return RPSToMPS(rps, circumference, 1.0);
    }

    public static double RPSToMPS(double rps, double circumference, double gearRatio) {
        return rps * circumference;
    }

    public static double MPSToRPS(double velocity, double circumference){
        return MPSToRPS(velocity, circumference, 1.0);
    }

    public static double MPSToRPS(double velocity, double circumference, double gearRatio){
        return velocity / circumference * gearRatio;
    }

    public static double rotationsToM(double rotations, double circumference, double gearRatio) {
        double axleRotations = rotations / gearRatio;
        return axleRotations * circumference;
    }

    public static double rotationsToM(double rotations, double circumference) {
        return rotationsToM(rotations, circumference, 1.0);
    }
}
