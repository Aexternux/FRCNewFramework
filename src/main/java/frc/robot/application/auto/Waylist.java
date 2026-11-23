package frc.robot.application.auto;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** A short, blendable list of poses used by one autonomous drive action. */
public class Waylist {
    private static final double TOLERANCE_m = 0.1;

    private final ArrayList<Pose2d> poses = new ArrayList<>();
    private final ArrayList<Double> tolerances_m = new ArrayList<>();
    private double pathProgress = 0.0;

    public Waylist(Pose2d pose, double tolerance_m) {
        append(pose, tolerance_m);
    }

    public Waylist append(Pose2d pose, double tolerance_m) {
        poses.add(pose);
        tolerances_m.add(tolerance_m);
        return this;
    }

    public Waylist transformAll(Function<Pose2d, Pose2d> transform) {
        for (int i = 0; i < poses.size(); i++) {
            poses.set(i, transform.apply(poses.get(i)));
        }
        return this;
    }

    public void reset() {
        pathProgress = 0.0;
    }

    public void updateBlended(Pose2d currentPose) {
        if (poses.size() < 2) return;

        int firstSegment = Math.min((int) pathProgress, poses.size() - 2);
        double bestProgress = pathProgress;
        double bestDistance_m = Double.POSITIVE_INFINITY;
        for (int segment = firstSegment; segment < Math.min(firstSegment + 2, poses.size() - 1); segment++) {
            Pose2d start = poses.get(segment);
            Pose2d end = poses.get(segment + 1);
            double dx = end.getX() - start.getX();
            double dy = end.getY() - start.getY();
            double lengthSquared = dx * dx + dy * dy;
            double t = lengthSquared < 1e-6 ? 0.0
                    : ((currentPose.getX() - start.getX()) * dx + (currentPose.getY() - start.getY()) * dy)
                            / lengthSquared;
            t = Math.max(0.0, Math.min(1.0, t));
            double distance_m = Math.hypot(currentPose.getX() - (start.getX() + t * dx),
                    currentPose.getY() - (start.getY() + t * dy));
            if (distance_m < bestDistance_m) {
                bestDistance_m = distance_m;
                bestProgress = segment + t;
            }
        }
        pathProgress = Math.max(pathProgress, bestProgress);
    }

    public Pose2d getBlendedTarget(Pose2d currentPose, double lookahead_m) {
        updateBlended(currentPose);
        if (poses.size() == 1) return poses.get(0);

        int segment = Math.min((int) pathProgress, poses.size() - 2);
        double segT = Math.min(Math.max(pathProgress - segment, 0.0), 1.0);
        double offPath_m = currentPose.getTranslation().getDistance(
                interpolate(poses.get(segment), poses.get(segment + 1), segT).getTranslation());
        double remaining_m = Math.max(lookahead_m - offPath_m, 0.0);

        while (segment < poses.size() - 2) {
            double length_m = poses.get(segment).getTranslation()
                    .getDistance(poses.get(segment + 1).getTranslation());
            double remainingInSeg_m = (1.0 - segT) * length_m;
            if (remaining_m <= remainingInSeg_m) {
                double t = segT + (length_m < 1e-6 ? 0.0 : remaining_m / length_m);
                return interpolate(poses.get(segment), poses.get(segment + 1), t);
            }
            remaining_m -= remainingInSeg_m;
            segment++;
            segT = 0.0;
        }
        Pose2d start = poses.get(segment);
        Pose2d end = poses.get(segment + 1);
        double length_m = start.getTranslation().getDistance(end.getTranslation());
        double t = length_m < 1e-9 ? 1.0 : Math.min(1.0, segT + remaining_m / length_m);
        return interpolate(start, end, t);
    }

    private static Pose2d interpolate(Pose2d start, Pose2d end, double t) {
        return new Pose2d(start.getX() + t * (end.getX() - start.getX()),
                start.getY() + t * (end.getY() - start.getY()),
                start.getRotation().interpolate(end.getRotation(), t));
    }

    public boolean isComplete(Pose2d currentPose) {
        return isComplete(currentPose, tolerances_m.get(tolerances_m.size() - 1));
    }

    /** Returns whether the final waypoint is within the supplied completion distance. */
    public boolean isComplete(Pose2d currentPose, double completionTolerance_m) {
        Pose2d finalPose = poses.get(poses.size() - 1);
        return currentPose.getTranslation().getDistance(finalPose.getTranslation())
                < completionTolerance_m;
    }

    /** Uses the final waypoint's configured translation tolerance and a supplied heading tolerance. */
    public boolean isCompleteWithHeading(Pose2d currentPose, double headingTolerance_rad) {
        return isComplete(currentPose)
                && Math.abs(poses.get(poses.size() - 1).getRotation()
                        .minus(currentPose.getRotation()).getRadians()) < headingTolerance_rad;
    }

    /** Returns whether both the final translation and heading have settled. */
    public boolean isComplete(Pose2d currentPose, double completionTolerance_m,
            double headingTolerance_rad) {
        Pose2d finalPose = poses.get(poses.size() - 1);
        return isComplete(currentPose, completionTolerance_m)
                && Math.abs(finalPose.getRotation().minus(currentPose.getRotation()).getRadians())
                        < headingTolerance_rad;
    }

    public double getPathProgress() {
        return pathProgress;
    }
}
