// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.lang.Math.PI;
//import static java.lang.Math.toRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean logging = true;
  public static final String robotType = "Keith Mini";
  public static final boolean isMini = true;

  public static final class DrivetrainConstants {

    public static final boolean ADD_TO_DASHBOARD = false;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = inchesToMeters(14.5);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = inchesToMeters(14.5);

    public static final double DRIVETRAIN_WHEEL_DIAMETER_METERS = inchesToMeters(6);

    public static final double DRIVETRAIN_WHEEL_GEAR_REDUCTION = 10;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * DRIVETRAIN_WHEEL_GEAR_REDUCTION
        * DRIVETRAIN_WHEEL_DIAMETER_METERS * PI ;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
        / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // Creating my kinematics object: track width of 27 inches
    public static DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(inchesToMeters(27.0));

    // Example differential drive wheel speeds: 2 meters per second
    // for the left side, 3 meters per second for the right side.
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(2.0, 3.0);

    // Convert to chassis speeds.
    ChassisSpeeds chassisSpeeds = KINEMATICS.toChassisSpeeds(wheelSpeeds);

    // Linear velocity
    double linearVelocity = chassisSpeeds.vxMetersPerSecond;

    // Angular velocity
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double DRIVE_kS = 0.6716;
    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double DRIVE_kV = 2.5913;
    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double DRIVE_kA = 0.19321;

    public static final double STEER_kP = 0.2;
    public static final double STEER_kI = 0.0;
    public static final double STEER_kD = 0.1;

    public static final double DRIVE_kP = 0.02;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;

  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 20;
    public static final double Y_RATE_LIMIT = 20;
    public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

    public static final double HEADING_MAX_VELOCITY = PI * 2;
    public static final double HEADING_MAX_ACCELERATION = PI * 2;

    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = degreesToRadians(1.5);

  }

  public static class VisionConstants {

    /**
     * Physical location of the camera on the robot, relative to the center of the
     * robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(-0.3425, 0.0, -0.233),
        new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }

  public static class AutoConstants {
    public static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static double THETA_kP = 5.0;
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;

    public static double X_kP = 16.0;
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 16.0;
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

  }
}
