package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.PhotonBridge;

public class NavigationSub extends SubsystemBase {
  private final ADIS16470_IMU imu = new ADIS16470_IMU(IMUAxis.kX, IMUAxis.kZ, IMUAxis.kY);
  private final Field2d field = new Field2d();
  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private Pose2d poseSim = new Pose2d();
  public final PhotonBridge photon = new PhotonBridge();

  private final double baseReadingError = 0.2;
  private double allowedReadingError = baseReadingError;

  // Pose estimation class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator;

  public NavigationSub() {
    zeroHeading();
    initPathPlanner();
    
    SmartDashboard.putData("Field", field);
    
    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(imu.getAngle()),
        Subsystems.drive.getModulePositions(),
        new Pose2d());
        
    resetOdometry(new Pose2d(4.992, 2.826, Rotation2d.k180deg)); 
  }

  /**
   * Initializes PathPlanner.
   * 
   * Should be called once upon robot start.
   */
  private void initPathPlanner() {
    try {
      final var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> {
            Subsystems.drive.drive(speeds, false);
            System.out.println(feedforwards.toString());
          },
          new PPHolonomicDriveController(
              DriveConstants.AUTO_TRANSLATION_PID,
              DriveConstants.AUTO_ROTATION_PID),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
          },
          this, Subsystems.drive);
    } catch (Exception e) {
      System.err
          .println("DriveSub: Error: PathPlanner failed to initialize! Autos may not work properly. Stack trace:");
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  /**
   * Updates the robot's odometry.
   * 
   * This should be called every robot tick, in the periodic method.
   */
  private void updateOdometry() {
    poseEstimator.update(Rotation2d.fromDegrees(imu.getAngle()), Subsystems.drive.getModulePositions());

    for (final var cam : photon.cams) {
      cam.getEstimatedPose()
          .ifPresent((visionResult) -> {
            // Reject any egregiously incorrect vision pose estimates
            final var visionPose = visionResult.estimatedPose.toPose2d();
            SmartDashboard.putString("visionPose", visionPose.toString());
            final var currentPose = getPose();
            final var errorMeters = visionPose.getTranslation().getDistance(currentPose.getTranslation());
            SmartDashboard.putNumber("visionErr", errorMeters);
            SmartDashboard.putNumber("visionErrLim", allowedReadingError);
            if (errorMeters > allowedReadingError){
              allowedReadingError *= 2;
            } else {
              allowedReadingError = baseReadingError;
              poseEstimator.addVisionMeasurement(visionPose, visionResult.timestampSeconds);
            }
          });
    }

    field.setRobotPose(getPose());
  }

  /** @return the currently estimated pose of the robot. */
  public Pose2d getPose() {
    return RobotBase.isReal() ? poseEstimator.getEstimatedPosition() : poseSim;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (pose == null){ pose = new Pose2d(0,0, Rotation2d.kZero);} //path planner can pass null when an auto includes no move command

    if (RobotBase.isSimulation()) {
      simImuSetAngleYaw(pose.getRotation().getDegrees());
      poseSim = pose;
      return;
    }

    poseEstimator.resetPosition(Rotation2d.fromDegrees(imu.getAngle()), Subsystems.drive.getModulePositions(), pose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.reset();
  }

  /**
   * @return the robot's heading (direction the robot is pointing field rel)
   */
  public Angle getHeading() {
    return Radians.of(poseEstimator.getEstimatedPosition().getRotation().getRadians());
  }

  /** * @return the turn rate of the robot */
  public AngularVelocity getTurnRate() {
    return DegreesPerSecond.of(imu.getRate());
  }

  /**
   * @return the robot's heading as a {@link Rotation2d} (direction the robot is
   *         pointing field rel)
   */
  public Rotation2d getHeadingR2D() {
    return Rotation2d.fromRadians(getHeading().in(Radians));
  }

  /** @return the current robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getChassisSpeeds() {
    //if (Robot.isSimulation()) {
    //  return getDesiredChassisSpeeds();
    //} // modules are not sim'd correctly
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(Subsystems.drive.getModuleStates());
  }

  /** @return the desired robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(Subsystems.drive.getModuleDesiredStates());
  }

  /**
   * @return the current translational speed of the robot (angle irrelevant) (m/s)
   */
  public double getTranslationSpeed() {
    final var speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * @return the current translational speed of the robot (angle irrelevant) (m/s)
   */
  public double getTranslationAngle() {
    final var speeds = getChassisSpeeds();
    return Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    final var speeds = getDesiredChassisSpeeds();

    SmartDashboard.putNumber("simImuAngle", field.getRobotPose().getRotation().getDegrees());
    SmartDashboard.putNumber("simImuRate", Math.toDegrees(speeds.omegaRadiansPerSecond));

    simImuSetRateYaw(Math.toDegrees(speeds.omegaRadiansPerSecond));
    // imuSim.setGyroAngleZ(getHeading().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond
    // * 0.02)).getDegrees());
    simImuSetAngleYaw(field.getRobotPose().getRotation().getDegrees());
    poseSim = poseSim.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * 0.02,
            speeds.vyMetersPerSecond * 0.02,
            speeds.omegaRadiansPerSecond * 0.02));

    photon.simulationPeriodic(getPose());
  }

  private void simImuSetAngleYaw(double angle) {
    switch(imu.getYawAxis()) {
      case kX: imuSim.setGyroAngleX(angle); break;
      case kY: imuSim.setGyroAngleY(angle); break;
      case kZ: imuSim.setGyroAngleZ(angle); break;
      case kRoll: imuSim.setGyroAngleX(angle); break;
      case kPitch: imuSim.setGyroAngleY(angle); break;
      case kYaw: imuSim.setGyroAngleZ(angle); break;
    }
  }

  private void simImuSetRateYaw(double angle) {
    switch(imu.getYawAxis()) {
      case kX: imuSim.setGyroRateX(angle); break;
      case kY: imuSim.setGyroRateY(angle); break;
      case kZ: imuSim.setGyroRateZ(angle); break;
      case kRoll: imuSim.setGyroRateX(angle); break;
      case kPitch: imuSim.setGyroRateY(angle); break;
      case kYaw: imuSim.setGyroRateZ(angle); break;
    }
  }
}