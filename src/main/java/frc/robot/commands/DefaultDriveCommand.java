package frc.robot.commands;

import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class DefaultDriveCommand extends Command {
  private final DrivetrainSubsystem drivetrain;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);

  /**
   * Constructor
   * 
   * @param drivetrainSubsystem  drivetrain
   * @param robotAngleSupplier   supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters
   *                             per second
   * @param translationYSupplier supplier for translation Y component, in meters
   *                             per second
   * @param rotationSupplier     supplier for rotation component, in radians per
   *                             second
   */
  public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    var robotAngle = robotAngleSupplier.get();

    // Calculate field relative speeds
    var chassisSpeeds = drivetrain.getChassisSpeeds();
    var robotSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond * robotAngle.getCos() - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
        chassisSpeeds.vyMetersPerSecond * robotAngle.getCos() + chassisSpeeds.vxMetersPerSecond * robotAngle.getSin(),
        chassisSpeeds.omegaRadiansPerSecond);

    // Reset the slew rate limiters, in case the robot is already moving
    translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
    rotationRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    double xSpeed = translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
    double ySpeed = translateYRateLimiter.calculate(translationYSupplier.getAsDouble());
    double rotSpeed = rotationRateLimiter.calculate(rotationSupplier.getAsDouble());
    if (Math.abs(xSpeed) > 0.1 || Math.abs(ySpeed) > 0.1 || Math.abs(rotSpeed) > 0.) {
      logf("Drive xSpeed:%.2f ySpeed:%.2f rotSpeed:%.2f\n", xSpeed, ySpeed, rotSpeed);
    }
    drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, robotAngleSupplier.get()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

}
