package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase {
    // Swerve Modules
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_FL);
    private final SwerveModule frontRight = new SwerveModule(DriveConstants.SWERVE_MODULE_FR);
    private final SwerveModule backLeft = new SwerveModule(DriveConstants.SWERVE_MODULE_BL);
    private final SwerveModule backRight = new SwerveModule(DriveConstants.SWERVE_MODULE_BR);

    // PID controller used for holding yaw in driveAtAngle
    private final PIDController holdYawPid = new PIDController(0.8, 0, 0);

    // Reference to NavigationSub for accessing heading information
    private final NavigationSub navigationSub;

    public DriveSub(NavigationSub navigationSub) {
        this.navigationSub = navigationSub;
        SmartDashboard.putData("FL Module", frontLeft);
        SmartDashboard.putData("FR Module", frontRight);
        SmartDashboard.putData("BL Module", backLeft);
        SmartDashboard.putData("BR Module", backRight);
    }

    /**
     * Drives the robot using field-relative speeds by default.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, true);
    }

    /**
     * Drives the robot with the option for field-relative control.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (fieldRelative) { // Convert field-relative speeds to robot-relative using current heading.
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, navigationSub.getHeadingR2D());
        }

        if (Robot.isSimulation()) { // Introduce a bit of simulated drift.
            double speedMag = getTranslationSpeed();
            Rotation2d speedDir = Rotation2d.fromRadians(getTranslationAngle());
            System.out.println(speedMag + "   " + speedDir);
            speedDir = speedDir.plus(Rotation2d.fromDegrees(-45));
            speedMag *= Math.cos(speedDir.getRadians());
            System.out.println(speedMag + " - " + speedDir);
            speeds = speeds.plus(new ChassisSpeeds(0, 0, 0.2 * speedMag));
        }

        SwerveModuleState[] states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /**
     * Drives the robot while holding a specific yaw angle.
     */
    public void driveAtAngle(ChassisSpeeds speeds, Rotation2d yawAngle) {
        Rotation2d currentYaw = navigationSub.getHeadingR2D();
        Rotation2d error = currentYaw.minus(yawAngle);
        speeds.omegaRadiansPerSecond = holdYawPid.calculate(error.getDegrees());
        drive(speeds);
    }

    /**
     * Commands the swerve modules to the desired states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** @return the current states of the swerve modules */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    /** @return the desired states of the swerve modules */
    public SwerveModuleState[] getModuleDesiredStates() {
        return new SwerveModuleState[] {
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState()
        };
    }

    /** @return the positions of the swerve modules */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    /** Resets the drive encoders to zero. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    /** Resets the integral accumulation of the turn motors. */
    public void resetIntegral() {
        frontLeft.resetIntegral();
        frontRight.resetIntegral();
        backLeft.resetIntegral();
        backRight.resetIntegral();
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

    /** @return the current robot-relative chassis speeds */
    public ChassisSpeeds getChassisSpeeds() {
        if (Robot.isSimulation()) {
            return getDesiredChassisSpeeds(); // In simulation, modules might not be updated correctly.
        }
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /** @return the desired robot-relative chassis speeds */
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleDesiredStates());
    }

    /** @return the current translational speed (m/s) */
    public double getTranslationSpeed() {
        final var speeds = getChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /** @return the current translational direction (radians) */
    public double getTranslationAngle() {
        final var speeds = getChassisSpeeds();
        return Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
    }
}
