// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.classes.SwerveModule;
import frc.robot.commands.auto.AutoProvider;
import frc.robot.commands.teleop.TeleopDriveSwerve;
import frc.robot.constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase {
  // Swerve Modules
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_FL);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.SWERVE_MODULE_FR);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_BL);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.SWERVE_MODULE_BR);

  private final PIDController holdYawPid = new PIDController(0.8, 0, 0);

  /** Creates a new DriveSubsystem. */
  public DriveSub() {
    // SmartDashboard.putData("FL Module", frontLeft);
    // SmartDashboard.putData("FR Module", frontRight);
    // SmartDashboard.putData("BL Module", backLeft);
    // SmartDashboard.putData("BR Module", backRight);

    setupSmartDash();
  }

  @Override
  public void periodic() {
    // final var cmd = this.getCurrentCommand();
    // if (cmd != null) {
    // System.out.println(cmd.getName());
    // }
  }

  public void driveAtAngle(ChassisSpeeds speeds, boolean fieldRelative, Rotation2d yawAngle) {
    drive(speeds, fieldRelative);
    Rotation2d currentYaw = Subsystems.nav.getHeadingR2D();
    Rotation2d err = currentYaw.minus(yawAngle);
    err.getDegrees();
    speeds.omegaRadiansPerSecond = holdYawPid.calculate(err.getDegrees());
  }

  /**
   * drives the robot
   * 
   * @param speeds field-relative speeds to drive at
   */
  public void drive(ChassisSpeeds speeds) {
    drive(speeds, true);
    
  }

  /**
   * drives the robot
   * 
   * @param speeds        speeds to drive at
   * @param fieldRelative true if the provided speeds are field-relative, false if
   *                      they are robot-relative
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative) { // convert field rel speeds to robot rel
   
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, new Rotation2d(Subsystems.nav.getIMUHeading()));
    }
    // System.out.println(speeds.toString());

    if (Robot.isSimulation() && false) { // induce some amount of drift while moving in sim
      double speedMag = Subsystems.nav.getTranslationSpeed();
      Rotation2d speedDir = Rotation2d.fromRadians(Subsystems.nav.getTranslationAngle());
      // System.out.println(speedMag + " " + speedDir);
      speedDir = speedDir.plus(Rotation2d.fromDegrees(-45));
      speedMag *= Math.cos(speedDir.getRadians());
      // System.out.println(speedMag + " - " + speedDir);

      speeds = speeds.plus(new ChassisSpeeds(0, 0, 0.2 * speedMag));
    }

    final var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);
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
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  /** reset turn motor pid I accumulation to 0 */
  public void resetIntegral() {
    frontLeft.resetIntegral();
    frontRight.resetIntegral();
    backLeft.resetIntegral();
    backRight.resetIntegral();

  }

  public SequentialCommandGroup testFunction() {
    return new SequentialCommandGroup(
        frontLeft.testFunction(),
        frontRight.testFunction(),
        backLeft.testFunction(),
        backRight.testFunction());
  }

  public void setupSmartDash() {
    SmartDashboard.putData("Swerve Drive Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Front Right Angle", () -> frontRight.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity().in(MetersPerSecond),
            null);

        builder.addDoubleProperty("Back Left Angle", () -> backLeft.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Back Right Angle", () -> backRight.getTurnAngle().in(Radians), null);
        builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity().in(MetersPerSecond), null);

        builder.addDoubleProperty("Robot Angle",
            () -> Subsystems.nav != null ? Subsystems.nav.getHeading().in(Radians) : 0, null);
      }
    });
  }
}
