package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Contains some basic utility functions that are used often. */
public class Util {

    public static double ONE_OVER_ROOT_TWO = 1.0 / Math.sqrt(2.0);
    public static double ONE_OVER_PI = 1.0 / Math.PI;
    public static double ONE_OVER_TAU = 1.0 / (2.0 * Math.PI);

    /** Prevent this class from being instantiated. */
    private Util() {}

    public static double sq(double x){
        return x*x;
    }

    /**
     * limit a value to a given range
     *
     * @param val - Value to check limits on
     * @param maxMagnitude - Magnitude to check limits with
     * @return Value with affected limit
     */
    public static double limit(double val, double maxMagnitude) {
        return limit(val, -maxMagnitude, maxMagnitude);
    }

    /**
     * limit a value to a given range
     *
     * @param val - Value to check limits on
     * @param min - Min value to check lower limit with
     * @param max - Max value to check upper limit with
     * @return Value with affected limit
     */
    public static double limit(double val, double min, double max) {
        return Math.min(max, Math.max(min, val));
    }

    /**
     * Checks if a value is within a given range
     *
     * @param val - Value to check
     * @param maxMagnitude - Magnitude for range to check
     * @return If the value is in the given range
     */
    public static boolean inRange(double val, double maxMagnitude) {
        return inRange(val, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if a value is within a given range
     *
     * @param val - Value to check
     * @param min - Min value on range
     * @param max - Max value on range
     * @return If the value is in the given range
     */
    public static boolean inRange(double val, double min, double max) {
        return val > min && val < max;
    }

    /**
     * Checks if a value is closer to a than b
     *
     * @param a - First point
     * @param b - Second point
     * @param val - Value to check
     * @return if a value is closer to a than b
     */
    public static boolean isCloser(double a, double b, double val) {
        return Math.abs(a - val) < Math.abs(b - val);
    }

    /**
     * Linear interpolation between a and b based on percentage alpha
     *
     * @param a - First point
     * @param b - Second value to check
     * @param alpha - Percentage between a and b of desired value
     * @return linear interpolation between a and b based on percentage alpha
     */
    public static double lerp(double a, double b, double alpha) {
        return a * (1.0 - alpha) + b * alpha;
    }

    /**
     * Opposite of linear interpolation for given val between a and b
     *
     * @param a - First point
     * @param b - Second value to check
     * @param alpha - Value between a and b of desired percentage
     * @return percentage that value is between a and b
     */
    public static double unlerp(double a, double b, double value) {
        return (value - a) / (b - a);
    }

    public static double remap(double min0, double max0, double value, double min1, double max1) {
        return lerp(min1, max1, unlerp(min0, max0, value));
    }

    public static double dist2(double x, double y) {
        return x * x + y * y;
    }

    public static double dist2(double x, double y, double z) {
        return x * x + y * y + z * z;
    }

    /**
     * 2x2 Matrix determinant
     *
     * @param v0 - First vector
     * @param v1 - Second vector
     * @return determinant of v0 and v1
     */
    public static double det(Translation2d v0, Translation2d v1) {
        return (v0.getX() * v1.getY()) - (v1.getX() * v0.getY());
    }

    /**
     * Checks if the point exists within the triangle defined by the starting vector v0 with sides determined by v1 and
     * v2
     *
     * @param detvv1 - The determinant of v and v1
     * @param detvv2 - The determinant of v and v2
     * @param detv0v1 - The determinant of v0 and v1
     * @param detv0v2 - The determinant of v0 and v2
     * @param detv1v2_inverted - 1 over the determinant of v1 and v2
     * @return if point is in triangle
     */
    public static boolean isInTriangle(
            double detvv1, double detvv2, double detv0v1, double detv0v2, double detv1v2_inverted) {
        double a = (detvv2 - detv0v2) * detv1v2_inverted;
        double b = -(detvv1 - detv0v1) * detv1v2_inverted;

        return a > 0.0 && b > 0.0 && (a + b) < 1.0;
    }

    /**
     * Checks if value x is between low and high
     *
     * @param facingAngle_rad angle the subject is facing
     * @param targetAngle_rad angle of the object
     * @param angleRange_rad range of angle
     * @return low < x < high
     */
    public static boolean isWithinAngle(double facingAngle_rad, double targetAngle_rad, double angleRange_rad) {
        double delta = (facingAngle_rad - targetAngle_rad + Math.PI * 3) % (2 * Math.PI) - Math.PI;
        return Math.abs(delta) < 0.5 * angleRange_rad;
    }

    /**
     * Checks if value x is between low and high inclusively
     *
     * @param x
     * @param low
     * @param high
     * @return low <= x <= high
     */
    public static boolean isWithinAngleInclusive(
            double facingAngle_rad, double targetAngle_rad, double angleRange_rad) {
        double delta = (facingAngle_rad - targetAngle_rad + Math.PI * 3) % (2 * Math.PI) - Math.PI;
        return Math.abs(delta) <= 0.5 * angleRange_rad;
    }

    /**
     * Checks if [x y] is within a arbitrary distance from [0 0]
     *
     * @param facingAngle_rad angle the subject is facing
     * @param targetAngle_rad angle of the object
     * @param angleRange range of angle
     * @return if [x y] is within a arbitrary distance from [0 0]
     */
    public static boolean isWithinDistance(double x, double y, double distance) {
        return (x * x + y * y) < (distance * distance);
    }

    /**
     * Checks if [x1 y1] is within a arbitrary distance from [x0 y0]
     *
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param distance
     * @return if [x1 y1] is within a arbitrary distance from [x0 y0]
     */
    public static boolean isWithinDistance(double x0, double y0, double x1, double y1, double distance) {
        return isWithinDistance(x1 - x0, y1 - y0, distance);
    }

    /**
     * Checks if [x y] is within a arbitrary distance from [0 0] inclusively
     *
     * @param x
     * @param y
     * @param distance
     * @return if [x y] is within a arbitrary distance from [0 0]
     */
    public static boolean isWithinDistanceInclusive(double x, double y, double distance) {
        return (x * x + y * y) <= (distance * distance);
    }

    /**
     * Checks if [x1 y1] is within a arbitrary distance from [x0 y0] inclusively
     *
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @param distance
     * @return if [x1 y1] is within a arbitrary distance from [x0 y0]
     */
    public static boolean isWithinDistanceInclusive(double x0, double y0, double x1, double y1, double distance) {
        return isWithinDistanceInclusive(x1 - x0, y1 - y0, distance);
    }

    public static double FPGATimeDelta_ms(long latest, long prev) {
        return 0.001 * (latest - prev);
    }

    public static double FPGATimeDelta_ms(long delta) {
        return 0.001 * (delta);
    }

    public static double sqInput(double x) {
        return Math.signum(x) * x * x;
    }

    public static double cubeInput(double x) {
        return x * x * x;
    }

    public static double rotsin(double x) {
        return Math.sin(Math.PI * 2.0 * x);
    }

    public static double rotcos(double x) {
        return Math.cos(Math.PI * 2.0 * x);
    }

    public static double fast_sigmoid(double x){
        return x / (1.0 + Math.abs(x));
    }

    public static double gaussian(double x, double stdev){
        return Math.exp(-sq(x) / (2.0 * sq(stdev)));
    }
}
