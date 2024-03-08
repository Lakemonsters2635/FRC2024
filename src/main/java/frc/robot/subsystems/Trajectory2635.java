package frc.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;

public class Trajectory2635 {
  /**
   * Generates a trajectory from the given waypoints and config. This method uses clamped cubic
   * splines -- a method in which the initial pose, final pose, and interior waypoints are provided.
   * The headings are automatically determined at the interior points to ensure continuous
   * curvature.
   *
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   * @param config The configuration for the trajectory.
   * @return The generated trajectory.
   */
  public static Trajectory generateTrajectory(
      Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config) {
    var controlVectors =
      // james update
      // SplineHelper.getCubicControlVectorsFromWaypoints(
      getCubicControlVectorsFromWaypoints(
        start, interiorWaypoints.toArray(new Translation2d[0]), end);

    // Return the generated trajectory.
    return TrajectoryGenerator.generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config);
  }  
  
  /**
   * Returns 2 cubic control vectors from a set of exterior waypoints and interior translations.
   *
   * @param start The starting pose.
   * @param interiorWaypoints The interior waypoints.
   * @param end The ending pose.
   * @return 2 cubic control vectors.
   */
  public static Spline.ControlVector[] getCubicControlVectorsFromWaypoints(
      Pose2d start, Translation2d[] interiorWaypoints, Pose2d end) {
    // Generate control vectors from poses.
    Spline.ControlVector initialCV;
    Spline.ControlVector endCV;

    // Chooses a magnitude automatically that makes the splines look better.
    if (interiorWaypoints.length < 1) {
      double scalar = start.getTranslation().getDistance(end.getTranslation()) * 1.2;
      initialCV = getCubicControlVector(scalar, start);
      endCV = getCubicControlVector(scalar, end);
    } else {
      double scalar = start.getTranslation().getDistance(interiorWaypoints[0]) * 1.2;
      initialCV = getCubicControlVector(scalar, start);
      scalar =
          end.getTranslation().getDistance(interiorWaypoints[interiorWaypoints.length - 1]) * 1.2;
      endCV = getCubicControlVector(scalar, end);
    }
    return new Spline.ControlVector[] {initialCV, endCV};
  }  

  private static Spline.ControlVector getCubicControlVector(double scalar, Pose2d point) {
    return new Spline.ControlVector(
        // TODO: put a break-point here and swap sin/cos as needed and adjust the signs of
        //       these to make them make sense.  This is not right... but could be a quick fix hack.
        new double[] {point.getX(), scalar * point.getRotation().getCos()},
        new double[] {point.getY(), scalar * point.getRotation().getSin()});
  }

}