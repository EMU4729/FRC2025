// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.PhotonBridge;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase {
  // Swerve Modules
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_FL);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.SWERVE_MODULE_FR);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_BL);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.SWERVE_MODULE_BR);


//  private final SimSwerveDrivetrain drivetrainSim = new SimSwerveDrivetrain(
//    DriveConstants.DRIVE_KINEMATICS,
//    DriveConstants.WHEEL_DIAMETER_INCHES / 2,
//    DriveConstants.MASS,
//    DriveConstants.MOI,
//    DriveConstants.WHEEL_FRICTION,
//    DriveConstants.MOTOR_NOISE
//);




  // The gyro sensor
  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  // Photon Bridge
  public final PhotonBridge photon = new PhotonBridge();

  // Field for robot viz
  private final Field2d field = new Field2d();

  private final ChassisSpeeds simErrorOffset;

  // Pose estimation class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
      getModulePositions(),
      new Pose2d());

  // Simulation Variables
  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private Pose2d poseSim = new Pose2d();
  
  //Angle PID
  private PIDController YawController = new PIDController(
    DriveConstants.YAW_kP,
    DriveConstants.YAW_kI,
    DriveConstants.YAW_kD);

    
  
    /** Creates a new DriveSubsystem. */
    public DriveSub() {
      zeroHeading();
      initPathPlanner();
  
      SmartDashboard.putData("Field", field);
      SmartDashboard.putData("FL Module", frontLeft);
      SmartDashboard.putData("FR Module", frontRight);
      SmartDashboard.putData("BL Module", backLeft);
      SmartDashboard.putData("BR Module", backRight);

      YawController.enableContinuousInput(-Math.PI, Math.PI);
      YawController.setTolerance(Math.toRadians((2))); //2 degree tolerance
  
      simErrorOffset = new ChassisSpeeds(0, 0, 0.1);
      
    }
  
    @Override
    public void periodic() {
      updateOdometry();
      field.setRobotPose(getPose());
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
            (speeds, feedforwards) -> drive(speeds, false),
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
            this);
      } catch (Exception e) {
        System.err
            .println("DriveSub: Error: PathPlanner failed to initialize! Autos may not work properly. Stack trace:");
        e.printStackTrace();
      }
    }
  
    /**
     * Updates the robot's odometry.
     * 
     * This should be called every robot tick, in the periodic method.
     */
    private void updateOdometry() {
      poseEstimator.update(getRotation2d(), getModulePositions());
  
      for (final var cam : photon.cams) {
        cam.getEstimatedPose()
            .ifPresent((visionResult) -> {
              // Reject any egregiously incorrect vision pose estimates
              final var visionPose = visionResult.estimatedPose.toPose2d();
              final var currentPose = getPose();
              final var errorMeters = visionPose.getTranslation().getDistance(currentPose.getTranslation());
              if (errorMeters > 1)
                return;
  
              poseEstimator.addVisionMeasurement(visionPose, visionResult.timestampSeconds);
            });
      }
  
      field.setRobotPose(getPose());
    }
  
  
    public void drive(double xSpeed, double ySpeed, Rotation2d targetYaw) {
      drive(new Translation2d(xSpeed, ySpeed), targetYaw);
  }

    /**
   * drives the robot
   * 
   * @param speeds        speeds to drive at
   * @param fieldRelative true if the provided speeds are field-relative, false if
   *                      they are robot-relative
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2d());
    }
    if(Robot.isSimulation()){
      double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond,2) + Math.pow(speeds.vxMetersPerSecond,2));
      speeds = speeds.plus(simErrorOffset.times(speed));
    }

        // Desaturate and set module states
        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_SPEED);
        setModuleStates(states);
  }

  
  /**
   * 
   * @param translation //movement of the robot along cartesian plane
   * @param targetYaw  //angle of movement of robot
   */
    public void drive(Translation2d translation, Rotation2d targetYaw) {
      double currentYaw = getRotation2d().getRadians();
      double targetYawRadians = targetYaw.getRadians();
      double omegaOutput = YawController.calculate(currentYaw,targetYawRadians);

      omegaOutput = MathUtil.clamp(
        omegaOutput,
        -Math.PI,
        Math.PI);

      ChassisSpeeds fieldRelativespeeds = new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      0
      );

      ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativespeeds.vxMetersPerSecond, 
        fieldRelativespeeds.vyMetersPerSecond,
        fieldRelativespeeds.omegaRadiansPerSecond,
        getRotation2d());
  
      drive(robotRelativeSpeeds,false);
  

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
    if (RobotBase.isSimulation()) {
      imuSim.setGyroAngleZ(pose.getRotation().getDegrees());
      poseSim = pose;
      return;
    }

    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the desired swerve module states.
   *
   * @param desiredStates The desired {@link SwerveModuleState}s.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** @return the modules' current {@link SwerveModuleState}s */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  /** @return the desired {@link SwerveModuleState}s of the modules */
  public SwerveModuleState[] getModuleDesiredStates() {
    return new SwerveModuleState[] {
        frontLeft.getDesiredState(),
        frontRight.getDesiredState(),
        backLeft.getDesiredState(),
        backRight.getDesiredState()
    };
  }

  /** @return the {@link SwerveModulePosition}s of the modules */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** reset turn motor pid I accumulation to 0 */
  public void resetIntegral() {
    frontLeft.resetIntegral();
    backLeft.resetIntegral();
    frontRight.resetIntegral();
    backRight.resetIntegral();

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.reset();
  }

  /** @return the robot's heading (deg) */
  public double getHeading() {
    return imu.getAngle() * (DriveConstants.GYRO_REVERSED ? -1 : 1);
  }

  /** * @return the turn rate of the robot (deg/s) */
  public double getTurnRate() {
    return imu.getRate() * (DriveConstants.GYRO_REVERSED ? -1 : 1);
  }

  /** @return the robot's heading as a {@link Rotation2d} */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /** @return the current robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /** @return the desired robot-relative {@link ChassisSpeeds} */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleDesiredStates());
  }

  /** @return the current translational speed of the robot (m/s) */
  public double getTranslationSpeed() {
    final var speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    final var speeds = getDesiredChassisSpeeds();

    imuSim.setGyroRateZ(Math.toDegrees(speeds.omegaRadiansPerSecond));
    // imuSim.setGyroAngleZ(getHeading().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond
    // * 0.02)).getDegrees());
    imuSim.setGyroAngleZ(field.getRobotPose().getRotation().getDegrees());
    poseSim = poseSim.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * 0.02,
            speeds.vyMetersPerSecond * 0.02,
            speeds.omegaRadiansPerSecond * 0.02));

    photon.simulationPeriodic(getPose());
  }
}