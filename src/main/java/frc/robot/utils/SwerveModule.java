// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SwerveModuleDetails;
import frc.robot.utils.motorsupplier.FalconMotorSupplier;
import frc.robot.utils.motorsupplier.SparkMotorSupplier;

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
    driveMotor = new FalconMotorSupplier(moduleDetails.driveCANID())
        .withBrake()
        .withEncoder(DriveConstants.DRIVE_GEAR_RATIO)
        .withPID(DriveConstants.DRIVE_P,
            DriveConstants.DRIVE_I,
            DriveConstants.DRIVE_D)
        .get();
    driveController = new VelocityVoltage(0).withFeedForward(DriveConstants.DRIVING_FF).withSlot(0);

    // TURNING MOTOR CONFIG
    turnMotor = new SparkMotorSupplier(moduleDetails.steerCANID())
        .withAbsEncoder(DriveConstants.TURNING_ENCODER_POSITION_FACTOR,
            DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR)
        .withPID(DriveConstants.TURNING_P,
            DriveConstants.TURNING_I,
            DriveConstants.TURNING_D,
            DriveConstants.TURNING_FF)
        .withPIDIZone(DriveConstants.TURNING_I_ZONE.in(Radians))
        .withPositionWrapping(DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT,
            DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT)
        .withBrake()
        .get();
    turnController = turnMotor.getClosedLoopController();
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    turnEncoder = turnMotor.getAbsoluteEncoder();

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
        () -> getTurnAngle().in(Degrees),
        (newValue) -> {
        });
    builder.addDoubleProperty(
        "Current Drive Distance (m)",
        () -> this.getDrivePosition().in(Meters),
        (newValue) -> {
        });

    builder.addDoubleProperty(
        "Current Turn Speed (deg/s)",
        () -> getTurnVelocity().in(DegreesPerSecond),
        (newValue) -> {
        });
    builder.addDoubleProperty(
        "Current Drive Speed (m/s)",
        () -> this.getDriveVelocity().in(MetersPerSecond),
        (newValue) -> {
        });
  }

  /** @return the module's drive wheel position (m) */
  public Distance getDrivePosition() {
    return Meters.of(driveMotor.getPosition().getValue().in(Radians) * DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
  }

  /** @return the module's drive wheel velocity (m/s) */
  public LinearVelocity getDriveVelocity() {
    return MetersPerSecond
        .of(driveMotor.getVelocity().getValue().in(RadiansPerSecond) * DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
  }

  /** @return the module's robot-relative turning angle (rad) */
  public Angle getTurnAngle() {
    return getTurnAngle(true);
  }

  /**
   * 
   * @param robotRelative angles returned rel to (True : the robot front CCW+,
   *                      False : The current module)
   * @return the angle of the module relative to either Module Frame or Robot
   *         Frame
   */
  public Angle getTurnAngle(boolean robotRelative) {
    if (robotRelative) {
      return Radians.of(turnEncoder.getPosition() - details.angularOffset().getRadians());
    }
    return Radians.of(turnEncoder.getPosition());
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
  public AngularVelocity getTurnVelocity() {
    return RadiansPerSecond.of(turnEncoder.getVelocity());
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

    state = optimize(state, getTurnRotation2d(false));
    driveMotor.setControl(
        driveController.withVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters)));
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
  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    double error = Math.abs(delta.getDegrees());
    int limit = lastLimit;
    /*
     * System.out
     * .println(limit + " " + error + " " + delta + " " + desiredState.angle + " " +
     * currentAngle + " " + isFlipped);
     */

    // optimizes by inverting the turn if the module is more than the limit
    if (error < limit) {
      lastLimit = error < 20 ? 90 : 135; // release only when near the target
      // direction
      isFlipped = true;
      return new SwerveModuleState(
          desiredState.speedMetersPerSecond,
          desiredState.angle);
    } else {
      isFlipped = false;
      lastLimit = error > 160 ? 90 : 45; // release only when near the inverted
      // target direction
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
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