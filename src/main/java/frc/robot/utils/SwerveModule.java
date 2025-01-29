// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SwerveModuleDetails;

public class SwerveModule implements Sendable {
  private final SwerveModuleDetails details;

  private final TalonFX driveMotor;
  private final SparkMax turnMotor;

  private final AbsoluteEncoder turnEncoder;

  private final VelocityVoltage driveController;
  private final SparkClosedLoopController turnController;

  /** the module's desired state, <strong>relative to the module.</strong> */
  private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

  /**
   * Constructs a new SwerveModule for a MAX Swerve Module housing a Falcon
   * driving motor and a Neo 550 Turning Motor.
   * 
   * @param moduleDetails the details of the module
   */
  public SwerveModule(SwerveModuleDetails moduleDetails) {
    this.details = moduleDetails;

    // DRIVE MOTOR CONFIG
    driveMotor = new TalonFX(moduleDetails.driveCANID());
    final var driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVE_MOTOR_INVERTED;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // set the Gear Ratio used for encoder reads
    driveMotorConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;
    // enable and set Current Limiting to prevent brownouts
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    // set the PID Config for the drive motor
    driveMotorConfig.Slot0.kP = DriveConstants.DRIVE_P;
    driveMotorConfig.Slot0.kI = DriveConstants.DRIVE_I;
    driveMotorConfig.Slot0.kD = DriveConstants.DRIVE_D;

    driveMotor.getConfigurator().apply(driveMotorConfig);
    driveController = new VelocityVoltage(0).withFeedForward(DriveConstants.DRIVING_FF).withSlot(0);

    // TURNING MOTOR CONFIG
    turnMotor = new SparkMax(moduleDetails.steerCANID(), MotorType.kBrushless);
    turnController = turnMotor.getClosedLoopController();
    final var turnMotorConfig = new SparkMaxConfig();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    turnEncoder = turnMotor.getAbsoluteEncoder();

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnMotorConfig.absoluteEncoder
        .positionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Set the PID gains for the turning motor and
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint
    turnMotorConfig.closedLoop
        .pidf(
            DriveConstants.TURNING_P,
            DriveConstants.TURNING_I,
            DriveConstants.TURNING_D,
            DriveConstants.TURNING_FF)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // enable and set Current Limiting to prevent brownouts
    turnMotorConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    // Save the SPARK MAX configurations (Brownout protection).
    turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // --------------GO TO DEFAULTS--------------
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveMotor.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Desired Turn Angle (deg)",
        () -> getDesiredState().angle.getDegrees(),
        (newValue) -> {
        });
    builder.addDoubleProperty(
        "Desired Drive Speed (m/s)",
        () -> getDesiredState().speedMetersPerSecond,
        (newValue) -> {
        });

    builder.addDoubleProperty(
        "Current Turn Angle (deg)",
        () -> Math.toDegrees(getTurnAngle()),
        (newValue) -> {
        });
    builder.addDoubleProperty(
        "Current Drive Distance (m)",
        this::getDrivePosition,
        (newValue) -> {
        });

    builder.addDoubleProperty(
        "Current Turn Speed (deg/s)",
        () -> Math.toDegrees(getTurnVelocity()),
        (newValue) -> {
        });
    builder.addDoubleProperty(
        "Current Drive Speed (m/s)",
        this::getDriveVelocity,
        (newValue) -> {
        });
  }

  /** @return the module's drive wheel position (m) */
  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  /** @return the module's drive wheel velocity (m/s) */
  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble() * DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
  }

  /** @return the module's robot-relative turning angle (rad) */
  public double getTurnAngle() { 
    return getTurnAngle(true);
  }
  /** @return the module's turning angle (rad) */
  public double getTurnAngle(boolean robotRelative) {
    if(robotRelative){
      return turnEncoder.getPosition() - details.angularOffset().getRadians();
    }
    return turnEncoder.getPosition();
  }

  /** @return the module's robot-relative turning angle as a {@link Rotation2d} */
  public Rotation2d getTurnRotation2d() {
    return getTurnRotation2d(true);
  }
  /** @return the module's turning angle as a {@link Rotation2d} */
  public Rotation2d getTurnRotation2d(boolean robotRelative) {
    return new Rotation2d(getTurnAngle(robotRelative));
  }

  /** @return the module's turning velocity (rad/s) */
  public double getTurnVelocity() {
    return turnEncoder.getVelocity();
  }

  /** @return the module's current robot-relative state */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getTurnRotation2d());
  }

  /** @return the module's current robot-relative position */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getTurnRotation2d());
  }

  /**
   * sets the desired state of the module.
   * 
   * @param state the desired state, relative to the robot.
   */
  public void setDesiredState(SwerveModuleState state) {
    // convert from robot-relative angle to module-relative angle
    state.angle = state.angle.plus(details.angularOffset());

    // if the desired state's speed is low enough, we can just stop the motors to
    // prevent motor weirdness
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      desiredState = state;
      stop();
      return;
    }

    // TODO: Make a good SwerveModuleState optimizer
    // stop changing optimiser
    state = optimize(state, getTurnRotation2d(false));
    driveMotor.setControl(
        driveController.withVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE_METERS));
    turnController.setReference(state.angle.getRadians(), ControlType.kPosition);

    desiredState = state;
  }

  /**
   * @return the desired state of the module, relative to the
   *         robot.
   */
  public SwerveModuleState getDesiredState() {
    // we must convert to a robot-relative angle, since desiredState is relative to
    // the module.
    return new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle.minus(details.angularOffset()));
  }

  // private static final double intertia = 45;
  /** Optimises wheel angle, using an inertia based system */
  /*
   * public SwerveModuleState optimize(SwerveModuleState
   * desiredState, Rotation2d currentAngle) {
   * var turnSpeed = 0;//Subsystems.swerveDrive.getTurnRate();
   * double threshold = 0;//90 + MathUtil.clamp(turnSpeed *
   * intertia, -80, 80);
   * 
   * var delta = desiredState.angle.minus(currentAngle);
   * if (delta.getDegrees() > threshold || delta.getDegrees()
   * < 180-threshold) {
   * isFlipped = true;
   * return new SwerveModuleState(
   * -desiredState.speedMetersPerSecond,
   * desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)
   * ));
   * } else {
   * isFlipped = false;
   * return new
   * SwerveModuleState(desiredState.speedMetersPerSecond,
   * desiredState.angle);
   * }
   * }
   */

  private int lastLimit = 0;
  private boolean isFlipped = false;
  /**
   * Optimises the wheel pivot direction to reduce time spent turning
   * uses a moving threshold to reduce flip-floping when near the 90deg point
   */
  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d
    currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    double error = Math.abs(delta.getDegrees());
    int limit = lastLimit;
    System.out.println(limit +" "+ error +" "+ delta +" "+ desiredState.angle +" "+ currentAngle +" "+ isFlipped);
    
    // optimizes by inverting the turn if the module is more than the limit
    if (error < limit) {
      lastLimit = error < 20 ? 90 : 135; // release only when near the target
      //direction
      isFlipped = true;
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle);
    } else {
      isFlipped = false;
      lastLimit = error > 160 ? 90 : 45; // release only when near the inverted
      //target direction
      return new SwerveModuleState(
          desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.kPi));
    }
  }
  

  /** resets the drive encoder */
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  /** reset turn motor pid I accumulation to 0 */
  public void resetIntegral() {
    turnController.setIAccum(0);
  }

  /** stops both the drive and turning motors. */
  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
}