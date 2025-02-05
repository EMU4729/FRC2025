
package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.RangeMath.AxesFit;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class DriveConstants {
  // NEO Motor Constants
  /** Free speed of the driving motor in rpm */
  public static final double FREE_SPEED_RPM = 6380;
  /** Distance between centers of left and right wheels on robot in meters */
  public static final double TRACK_WIDTH = 0.52;
  /** Distance between front and back wheel on robot in meters */
  public static final double WHEEL_BASE = 0.52;
  /** Drivebase radius in m (distance from center of robot to farthest module) */
  public static final double DRIVEBASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  /**
   * Max speed of robot in meters per second
   * 
   * Right now, this is just set to a bit below the maxswerve module's free speed.
   * It should probably be changed.
   */
  public static final double MAX_SPEED = 4.8; // TODO check this
  /** Max acceleration of robot in meters per second squared */
  public static final double MAX_ACCELERATION = 1; // TODO check this
  public static final double MAX_DECELERATION = 2; // TODO check this


/**
 * For Robot Simulation
 * 
 */
public static final double MASS = 50; // Robot mass (kg)
public static final double MOI = 5; // Moment of inertia (kg·m²)
public static final double WHEEL_FRICTION = 0.1; // Wheel friction
public static final double MOTOR_NOISE = 0.05; // Motor noise
public static final double WHEEL_DIAMETER_INCHES = 3;


  /**
   * Max angular speed of robot in radians per second
   * 
   * This is derived from the MAX_SPEED using the angular velocity formula v = ωr
   */
  public static final double MAX_ANGULAR_SPEED = MAX_SPEED / DRIVEBASE_RADIUS;
  /** Max angular acceleration of robot in radians per second squared */
  public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_SPEED / 60 * 15;
  public static final double MAX_ANGULAR_DECELERATION = MAX_ANGULAR_SPEED / 60 * 30;

  public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
      MAX_SPEED, MAX_ACCELERATION,
      MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);

  /** Direction slew rate in radians per second */
  public static final double DIRECTION_SLEW_RATE = 1.2;
  /** Magnitude slew rate in percent per second (1 = 100%) */
  public static final double MAGNITUDE_SLEW_RATE = 1.8;
  /** Rotational slew rate in percent per second (1 = 100%) */
  public static final double ROTATIONAL_SLEW_RATE = 8.0;

  /**
   * Gear ratio of the MAX Swerve Module driving motor (gear ratio upgrade kit
   * extra high speed 1)
   * 
   * note to self: read deeper and double check unintuitive science next time, Joel
   * 
   * @see https://www.revrobotics.com/rev-21-3005/ for gear ratios
   */
  public static final double DRIVE_GEAR_RATIO = 4.50;

  // Chassis configuration

  /** IMU Gyro Inversion */
  public static final boolean GYRO_REVERSED = false;

  // Module Constants
  /** Drive motor inversion. */
  public static final InvertedValue DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

  /**
   * Whether the turning encoder is inverted or not.
   * 
   * In the MAXSwerve module, this should be set to `true`, since the output shaft
   * rotates in the opposite direction of the steering motor.
   */
  public static final boolean TURNING_ENCODER_INVERTED = true;

  /** Constants for Angle for Drag */
  public static final double YAW_kP = 0.5; // Tune this

  public static final double YAW_kI = 0.0;
  
  public static final double YAW_kD = 0.01;


  // Calculations required for driving motor conversion factors and feed forward
  /** Free speed of the driving motor in rps */
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
  /** Wheel diameter in meters */
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
  /** Wheel circumference in meters */
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

  /** Turning encoder position factor */
  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  /** Turning encoder velocity factor */
  public static final double TURNING_ENCODER_VELOCITY_FACTOR = TURNING_ENCODER_POSITION_FACTOR / 60.0; // rad/s
  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

  // TODO tune PID
  public static final double DRIVE_P = 0.7;
  public static final double DRIVE_I = 0.05;
  public static final double DRIVE_D = 0.05;
  public static final double DRIVING_FF = 0;

  public static final double TURNING_P = 1.3;
  public static final double TURNING_I = 0.05;
  public static final double TURNING_D = 0.15;
  public static final double TURNING_FF = 0;
  public static final double TURNING_I_ZONE = Math.toRadians(15);

  // Auto Constants
  // TODO: Tune these
  /** Auto translation PID constants */
  public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(0.1, 0, 0);
  /** Auto rotation PID constants */
  public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(0.25, 0, 0);

  /*
   * public static DriveBaseFit PILOT_SETTINGS = DriveBaseFit(
   * 0, 1, 4, 0.1, true,
   * 0, 1, 4, 0.1, false,
   * 0, 1, 3, 0.1, false,
   * 0.85, 0.8);
   */
  public static DriveBaseFit PILOT_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.7).withPow(4).withDeadBand(0.1)
          .withLimiter(0.15).withBooster(1),
      new AxesFit().withPow(3).withDeadBand(0.1).withLimiter(0.15).inverted());

  /*
   * public static DriveBaseFit PILOT_DEMO_SETTINGS = DriveBaseFit.InitSwerveBot(
   * 0, 0.2, 2, 0.1, true,
   * 0, 0.2, 2, 0.1, false,
   * 0, 0.2, 2, 0.1, false,
   * 0.6, 1);
   */
  public static DriveBaseFit PILOT_DEMO_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(0, 0.2).withPow(2).withDeadBand(0.1).withLimiter(0.15),
      new AxesFit().withOutputMinMax(0, 0.2).withPow(2).withDeadBand(0.1).withLimiter(0.15).inverted());

  public static final SwerveModuleDetails SWERVE_MODULE_FL = new SwerveModuleDetails(
      1, // Drive motor CAN ID
      1, // Steer motor CAN ID
      Rotation2d.kCW_90deg, // offset relative to FL
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2) // location rel to centre
  );
  public static final SwerveModuleDetails SWERVE_MODULE_FR = new SwerveModuleDetails(
      2, // Drive motor CAN ID
      2, // Steer motor CAN ID
      Rotation2d.kZero, // offset relative to FL
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2) // location rel to centre
  );
  public static final SwerveModuleDetails SWERVE_MODULE_BL = new SwerveModuleDetails(
      3, // Drive motor CAN ID
      3, // Steer motor CAN ID
      Rotation2d.k180deg, // Offset rel to FL module
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2) // location rel to centre
  );
  public static final SwerveModuleDetails SWERVE_MODULE_BR = new SwerveModuleDetails(
      4, // Drive motor CAN ID
      4, // Steer motor CAN ID
      Rotation2d.kCCW_90deg, // Offset rel to FL module
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // location rel to centre
  );

  /** Swerve Kinematics */
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      SWERVE_MODULE_FL.location(),
      SWERVE_MODULE_FR.location(),
      SWERVE_MODULE_BL.location(),
      SWERVE_MODULE_BR.location());

  public static record SwerveModuleDetails(
      /** CAN ID for the module's Driving Motor */
      int driveCANID,
      /** CAN ID for the module's Steering Motor */
      int steerCANID,
      /**
       * Angular offset of the module around the robot's center, relative to FL
       * module.
       */
      Rotation2d angularOffset,
      /**
       * Location of module relative to robot center. Mainly for sim purposes.
       */
      Translation2d location) {
  }
}
