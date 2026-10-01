// Every subsystem lives in its own package under frc.robot.subsystems.
package frc.robot.subsystems.obstacles;

// java.io / java.nio give us file reading for the deploy-folder obstacle map.
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

// AdvantageKit's Logger: anything you recordOutput here shows up in AdvantageScope / the log.
import org.littletonrobotics.junction.Logger;

// WPILib geometry + the AprilTag field layout (used here only to get the field size).
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Filesystem.getDeployDirectory() points at src/main/deploy (which ships with the robot code).
import edu.wpi.first.wpilibj.Filesystem;

// Our framework base class. Every subsystem extends it and gets the periodic lifecycle below.
import frc.robot.subsystems.SubsystemBase;

// SubsystemBase is generic over an enum of "commands" (the states you can put the subsystem in).
// We declare that enum (Command) right below and pass it as the type parameter.
public class ObstacleAvoidance extends SubsystemBase<ObstacleAvoidance.Command> {

  // The set of states this subsystem understands. setCommand(...) switches between them and
  // getCommand() reads the current one. Here it's just an on/off switch.
  public enum Command {
    ENABLED,
    DISABLED
  }

  // Name of the map file we read out of the deploy folder.
  private static final String FILE = "obstacles.txt";

  // Tuning constants. The _unit suffix says what the number is measured in.
  // MAX_DECEL_mps2: how hard we assume the robot can brake. Bigger = lets you approach walls faster.
  // ROBOT_RADIUS_m + MARGIN_m: how far from a wall the robot is allowed to stop.
  private final double MAX_DECEL_mps2 = 8.0;
  private final double ROBOT_RADIUS_m = 0.45;
  private final double MARGIN_m = 0.10;

  // Singleton instance. There is only ever one of each subsystem in the whole robot.
  private static ObstacleAvoidance instance;

  // Every obstacle is a closed polygon = a list of corner points. This is the list of all of them.
  private final List<List<Translation2d>> polygons = new ArrayList<>();

  // Constructor is private so nobody can "new" a second copy. Only getInstance() creates it.
  private ObstacleAvoidance() {
    super("ObstacleAvoidance"); // the String is the log key: everything logs under "ObstacleAvoidance/...".
    addFieldPerimeter();         // add the field walls automatically.
    load();                      // then add whatever shapes were drawn in the editor.
    setCommand(Command.ENABLED); // start switched on.
  }

  // The singleton accessor. First call builds it; every later call returns the same object.
  public static ObstacleAvoidance getInstance() {
    if (instance == null) {
      instance = new ObstacleAvoidance();
    }
    return instance;
  }

  // Build the field boundary rectangle so the robot always avoids the perimeter walls.
  private void addFieldPerimeter() {
    // Load the season's tag map just to read the field dimensions from it.
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    double length_m = layout.getFieldLength();
    double width_m = layout.getFieldWidth();
    // Four corners, counter-clockwise, forming the field rectangle.
    List<Translation2d> perimeter = new ArrayList<>();
    perimeter.add(new Translation2d(0, 0));
    perimeter.add(new Translation2d(length_m, 0));
    perimeter.add(new Translation2d(length_m, width_m));
    perimeter.add(new Translation2d(0, width_m));
    polygons.add(perimeter);
  }

  // Read the obstacle map that the editor exported into src/main/deploy/obstacles.txt.
  private void load() {
    // getDeployDirectory() resolves to the deploy folder on both the robot and in sim.
    Path path = Filesystem.getDeployDirectory().toPath().resolve(FILE);
    if (!Files.exists(path)) {
      return; // no map: we still have the perimeter from addFieldPerimeter().
    }
    try {
      // The file format is: "POLYGON", then "x y" lines, then "END", repeated per shape.
      List<Translation2d> current = new ArrayList<>();
      for (String raw : Files.readAllLines(path)) {
        String line = raw.trim();
        // Skip blank lines and # comments.
        if (line.isEmpty() || line.startsWith("#")) {
          continue;
        }
        if (line.equalsIgnoreCase("POLYGON")) {
          current = new ArrayList<>(); // start collecting a new shape's points.
        } else if (line.equalsIgnoreCase("END")) {
          if (current.size() >= 2) {   // a real shape needs at least a couple points.
            polygons.add(current);
          }
        } else {
          // Otherwise it's an "x y" vertex line. Split on whitespace and parse the two numbers.
          String[] xy = line.split("\\s+");
          current.add(new Translation2d(Double.parseDouble(xy[0]), Double.parseDouble(xy[1])));
        }
      }
      System.out.println("Obstacles loaded: " + polygons.size());
    } catch (IOException | NumberFormatException e) {
      // A bad file shouldn't crash the robot; just log it and carry on with what we have.
      System.out.println("Obstacles load failed: " + e.getMessage());
    }
  }

  // The public API Drive calls: given the robot's pose and the speeds it WANTS to drive, return the
  // speeds it's ALLOWED to drive so it can always brake before any wall.
  public ChassisSpeeds constrain(Pose2d pose, ChassisSpeeds speeds) {
    // If we're switched off (or there's nothing to avoid), pass the request straight through.
    if (getCommand() == Command.DISABLED || polygons.isEmpty()) {
      return speeds;
    }
    // ChassisSpeeds are robot-relative (x = forward). Obstacles are in field coordinates, so rotate
    // the velocity by the robot's heading to get it into the field frame.
    Rotation2d heading = pose.getRotation();
    Translation2d v = new Translation2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(heading);
    double vx = v.getX();
    double vy = v.getY();
    Translation2d p = pose.getTranslation(); // robot's field position.

    // Check the robot against every edge of every shape. Polygon zero is the field perimeter;
    // every later polygon represents space the robot must stay outside of.
    for (int polygonIndex = 0; polygonIndex < polygons.size(); polygonIndex++) {
      List<Translation2d> poly = polygons.get(polygonIndex);
      boolean inside = isInsidePolygon(p, poly);
      boolean invalidSide = polygonIndex == 0 ? !inside : inside;
      if (invalidSide) {
        // Recover through the nearest boundary. Applying this rule to every edge would let the
        // far side of a polygon cancel the escape command and trap the robot at its center.
        Translation2d nearest = closestOnPolygon(p, poly);
        Translation2d toBoundary = nearest.minus(p);
        double distance_m = toBoundary.getNorm();
        if (distance_m > 1e-6) {
          double nx = toBoundary.getX() / distance_m;
          double ny = toBoundary.getY() / distance_m;
          double toward_mps = vx * nx + vy * ny;
          if (toward_mps < 0.0) {
            vx -= toward_mps * nx;
            vy -= toward_mps * ny;
          }
        }
        continue;
      }
      for (int i = 0; i < poly.size(); i++) {
        // Edge from vertex i to the next one (wrapping around to close the shape).
        Translation2d closest = closestOnSegment(p, poly.get(i), poly.get((i + 1) % poly.size()));
        Translation2d toWall = closest.minus(p); // vector from robot to the nearest point on the edge.
        double dist_m = toWall.getNorm();
        if (dist_m < 1e-6) {
          continue; // basically on top of the edge; skip to avoid dividing by ~0.
        }
        // Unit vector pointing at the wall.
        double nx = toWall.getX() / dist_m;
        double ny = toWall.getY() / dist_m;
        // How fast we're moving TOWARD this wall (dot product of velocity and the wall direction).
        double toward_mps = vx * nx + vy * ny;

        if (toward_mps <= 0) {
          continue; // moving away from / along the wall: no limit needed.
        }
        // Room left before the robot's edge reaches the wall.
        double free_m = dist_m - ROBOT_RADIUS_m - MARGIN_m;
        // Fastest we can go and still stop in that distance: v = sqrt(2 * a * d).
        double max_mps = free_m <= 0 ? 0 : Math.sqrt(2.0 * MAX_DECEL_mps2 * free_m);
        // If we're approaching faster than that, subtract off the excess toward-the-wall speed.
        // (Speed parallel to the wall is untouched, so you slide along it instead of stopping dead.)
        if (toward_mps > max_mps) {
          double excess = toward_mps - max_mps;
          vx -= excess * nx;
          vy -= excess * ny;
        }
      }
    }

    // Rotate the limited field-frame velocity back into robot-relative speeds for the drivetrain.
    Translation2d limited = new Translation2d(vx, vy).rotateBy(heading.unaryMinus());
    return new ChassisSpeeds(limited.getX(), limited.getY(), speeds.omegaRadiansPerSecond);
  }

  // Standard geometry helper: the closest point to p on the line segment a--b.
  private static Translation2d closestOnSegment(Translation2d p, Translation2d a, Translation2d b) {
    double dx = b.getX() - a.getX();
    double dy = b.getY() - a.getY();
    double len2 = dx * dx + dy * dy; // squared length of the segment.
    if (len2 < 1e-9) {
      return a; // a and b are the same point.
    }
    // t = how far along a--b the projection of p falls (0 = at a, 1 = at b).
    double t = ((p.getX() - a.getX()) * dx + (p.getY() - a.getY()) * dy) / len2;
    t = Math.max(0.0, Math.min(1.0, t)); // clamp so we stay on the segment, not the infinite line.
    return new Translation2d(a.getX() + t * dx, a.getY() + t * dy);
  }

  private static Translation2d closestOnPolygon(Translation2d point, List<Translation2d> polygon) {
    Translation2d nearest = polygon.get(0);
    double nearestDistanceSquared = Double.POSITIVE_INFINITY;
    for (int i = 0; i < polygon.size(); i++) {
      Translation2d candidate = closestOnSegment(point, polygon.get(i),
          polygon.get((i + 1) % polygon.size()));
      double distanceSquared = candidate.getDistance(point) * candidate.getDistance(point);
      if (distanceSquared < nearestDistanceSquared) {
        nearest = candidate;
        nearestDistanceSquared = distanceSquared;
      }
    }
    return nearest;
  }

  /** Ray-casting containment test. Boundary points count as inside for stable limiting. */
  private static boolean isInsidePolygon(Translation2d point, List<Translation2d> polygon) {
    boolean inside = false;
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      Translation2d a = polygon.get(i);
      Translation2d b = polygon.get(j);
      if (closestOnSegment(point, a, b).getDistance(point) < 1e-6) {
        return true;
      }
      boolean straddlesY = (a.getY() > point.getY()) != (b.getY() > point.getY());
      if (straddlesY) {
        double xAtY = (b.getX() - a.getX()) * (point.getY() - a.getY())
            / (b.getY() - a.getY()) + a.getX();
        if (point.getX() < xAtY) inside = !inside;
      }
    }
    return inside;
  }

  // ---- SubsystemBase lifecycle: these three run every loop, in this order, from periodic(). ----

  // inputPeriodic: read sensors / hardware here. We have none, so it's empty.
  @Override
  protected void inputPeriodic() {}

  // handle: run the state machine / do the work here. Our work happens inside constrain() (called by
  // Drive), so there's nothing to do each loop.
  @Override
  protected void handle() {}

  // outputPeriodic: send outputs and log here. We just publish the shapes for AdvantageScope.
  @Override
  protected void outputPeriodic() {
    // Turn every polygon into a list of poses that trace its outline, for the field view.
    List<Pose2d> outline = new ArrayList<>();
    for (List<Translation2d> poly : polygons) {
      for (int i = 0; i <= poly.size(); i++) { // <= so we return to the first point and close the loop.
        outline.add(new Pose2d(poly.get(i % poly.size()), new Rotation2d()));
      }
    }
    Logger.recordOutput("Obstacles/Outline", outline.toArray(new Pose2d[0]));
    Logger.recordOutput("Obstacles/Count", polygons.size());
  }
}
