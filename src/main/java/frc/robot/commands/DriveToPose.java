package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.PIDF;
import frc.robot.utilities.TunablePIDF;

/** Drive straight to a given pose. Position and angle are handled independently, such that if the
 *  robot has to drive a significant distance, the final angle is reached before translation
 *  completes. Conversely, short translations may complete before rotation completes. */
public class DriveToPose extends Command {
  private final Pose2d m_pose;
  private final DriveSubsystem m_drive;
  private final double m_squaredTranslationPositionToleranceMeters;
  private final double m_squaredTranslationVelocityToleranceMetersPerDt;

  private static final TunablePIDF translatingPIDF = new TunablePIDF("DriveToPose.translatingPIDF",
    DriveCommandConstants.kTranslatingPIDF);
  private static final TunablePIDF turningPIDF = new TunablePIDF("DriveToPose.turningPIDF",
    DriveCommandConstants.kTurningPIDF);

  private ProfiledPIDController m_xController = new ProfiledPIDController(
    translatingPIDF.get().p(),
    translatingPIDF.get().i(),
    translatingPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0), // Dynamically scaled.
    Constants.kDt);
  private ProfiledPIDController m_yController = new ProfiledPIDController(
    translatingPIDF.get().p(),
    translatingPIDF.get().i(),
    translatingPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0), // Dynamically scaled.
    Constants.kDt);
  private ProfiledPIDController m_angleController = new ProfiledPIDController(
    turningPIDF.get().p(),
    turningPIDF.get().i(),
    turningPIDF.get().d(),
    new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond,
      DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared),
    Constants.kDt);

  private static double square(double x) {
    return x * x;
  }

  public DriveToPose(Pose2d pose, DriveSubsystem drive,
      double translationPosToleranceMeters, double translationVelToleranceMetersPerSecond,
      double anglePosToleranceRadians, double angleVelToleranceRadiansPerSecond) {
    m_pose = pose;
    m_drive = drive;
    m_squaredTranslationPositionToleranceMeters = square(translationPosToleranceMeters);
    m_squaredTranslationVelocityToleranceMetersPerDt =
      square(Constants.kDt * translationVelToleranceMetersPerSecond);

    m_angleController.setTolerance(anglePosToleranceRadians,
      Constants.kDt * angleVelToleranceRadiansPerSecond);

    addRequirements(m_drive);
  }

  public DriveToPose(Pose2d pose, DriveSubsystem drive) {
    this(pose, drive,
      DriveCommandConstants.kDefaultTranslationPositionToleranceMeters,
      DriveCommandConstants.kDefaultTranslationVelocityToleranceMetersPerSecond,
      DriveCommandConstants.kDefaultAnglePositionToleranceRadians,
      DriveCommandConstants.kDefaultAngleVelocityToleranceRadiansPerSecond);
  }

  private Translation2d getDesiredTranslation() {
    return m_pose.getTranslation();
  }

  private Translation2d getTranslationDeviation(Pose2d robotPose) {
    return getDesiredTranslation().minus(robotPose.getTranslation());
  }

  private Rotation2d getDesiredRotation() {
    return m_pose.getRotation();
  }

  private Rotation2d getRotationDeviation(Pose2d robotPose) {
    Rotation2d currentRotation = robotPose.getRotation();
    Rotation2d desiredRotation = getDesiredRotation();
    Rotation2d rotationDeviation = currentRotation.minus(desiredRotation);
    return rotationDeviation;
  }

  /* Dynamically scale the x,y controller constraints such that the combined x,y component
   * movements combine to follow a straight line. Absent rescaling, the robot typically would
   * initially move at a 45 degree angle relative to the axes, then arc to horizontal or vertical
   * movement until reaching the final pose.
   *
   * A simpler strategy would be to scale the controller constraints once during command
   * initialization, but if the the angle relative to an axis were close to 0, the controller
   * would be incapable of significant movement along the other axis, and if the robot were to
   * deviate from the intended trajectory, the controller would be incapable of correction. */
  private void scaleXYConstraints(Pose2d robotPose, Translation2d translationDeviation) {
    Rotation2d translationAngle = translationDeviation.getAngle();
    double xFactor = translationAngle.getCos();
    m_xController.setConstraints(new TrapezoidProfile.Constraints(
      xFactor * DriveConstants.kMaxSpeedMetersPerSecond,
      xFactor * DriveConstants.kMaxAccelerationMetersPerSecondSquared
    ));
    double yFactor = translationAngle.getSin();
    m_yController.setConstraints(new TrapezoidProfile.Constraints(
      yFactor * DriveConstants.kMaxSpeedMetersPerSecond,
      yFactor * DriveConstants.kMaxAccelerationMetersPerSecondSquared
    ));
  }

  @Override
  public void initialize() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d translationDeviation = getTranslationDeviation(robotPose);
    Translation2d velocity = m_drive.getVelocity();

    scaleXYConstraints(robotPose, translationDeviation);
    m_xController.reset(
      translationDeviation.getX(),
      velocity.getX()
    );
    m_yController.reset(
      translationDeviation.getY(),
      velocity.getY()
    );
    m_angleController.reset(
      getRotationDeviation(robotPose).getRadians(),
      m_drive.getAngularVelocity()
    );
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();
    Translation2d translationDeviation = getTranslationDeviation(robotPose);
    Rotation2d rotationDeviation = getRotationDeviation(robotPose);

    updateConstants();

    scaleXYConstraints(robotPose, translationDeviation);
    double xVelocity = m_xController.calculate(translationDeviation.getX());
    double yVelocity = m_yController.calculate(translationDeviation.getY());
    double angleVelocity = m_angleController.calculate(rotationDeviation.getRadians());
    m_drive.drive(
      xVelocity,
      yVelocity,
      angleVelocity,
      true);
  }

  private void updateConstants() {
    if (translatingPIDF.hasChanged()) {
      PIDF pidf = translatingPIDF.get();
      m_xController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
    if (turningPIDF.hasChanged()) {
      PIDF pidf = turningPIDF.get();
      m_angleController.setPID(pidf.p(), pidf.i(), pidf.d());
    }
  }

  private boolean atSetpoint() {
    // Extract error values from the x,y controllers and combine them such that the tolerance
    // defines a circle rather than a square.
    double xPosError = m_xController.getPositionError();
    double yPosError = m_yController.getPositionError();
    if (square(xPosError) + square(yPosError) > m_squaredTranslationPositionToleranceMeters) {
      return false;
    }
    double xVelError = m_xController.getVelocityError();
    double yVelError = m_yController.getVelocityError();
    if (square(xVelError) + square(yVelError) > m_squaredTranslationVelocityToleranceMetersPerDt) {
      return false;
    }
    return m_angleController.atSetpoint();
  }

  @Override
  public boolean isFinished() {
    return atSetpoint();
  }
}