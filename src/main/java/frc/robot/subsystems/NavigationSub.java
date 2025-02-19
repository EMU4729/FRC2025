package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.PhotonBridge;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class NavigationSub extends SubsystemBase {
    private final ADIS16470_IMU imu = new ADIS16470_IMU();
    private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d poseSim = new Pose2d();
    private final Field2d field = new Field2d();
    public final PhotonBridge photon = new PhotonBridge();

    // A supplier to obtain module positions (provided by DriveSub)
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;

    public NavigationSub(Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
        this.modulePositionsSupplier = modulePositionsSupplier;
        // Initialize the pose estimator with the current heading and module positions.
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
            modulePositionsSupplier.get(),
            new Pose2d());
        initPathPlanner();
        SmartDashboard.putData("Field", field);
    }

    private void initPathPlanner() {
        try {
            final var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                () -> {
                    // Provide a chassis speeds supplier if needed.
                    return null;
                },
                (speeds, feedforwards) -> {
                    // This lambda won't command drive outputs from navigation.
                },
                new PPHolonomicDriveController(
                    DriveConstants.AUTO_TRANSLATION_PID,
                    DriveConstants.AUTO_ROTATION_PID),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
                this);
        } catch (Exception e) {
            System.err.println("NavigationSub: Error: PathPlanner failed to initialize! Autos may not work properly. Stack trace:");
            e.printStackTrace();
        }
    }

    /**
     * Updates odometry using the current heading from the IMU and the module positions.
     * Also adds vision measurements if available.
     */
    public void updateOdometry() {
        poseEstimator.update(getHeadingR2D(), modulePositionsSupplier.get());

        // Process vision measurements from Photon Bridge
        for (final var cam : photon.cams) {
            cam.getEstimatedPose().ifPresent((visionResult) -> {
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

    /** @return the current estimated pose */
    public Pose2d getPose() {
        return RobotBase.isReal() ? poseEstimator.getEstimatedPosition() : poseSim;
    }

    /**
     * Resets odometry to a specified pose.
     */
    public void resetOdometry(Pose2d pose) {
        if (RobotBase.isSimulation()) {
            imuSim.setGyroAngleZ(pose.getRotation().getDegrees());
            poseSim = pose;
            return;
        }
        poseEstimator.resetPosition(getHeadingR2D(), modulePositionsSupplier.get(), pose);
    }

    /** @return the current heading in degrees */
    public double getHeading() {
        return imu.getAngle(IMUAxis.kZ) * (DriveConstants.GYRO_REVERSED ? -1 : 1);
    }

    /** @return the current heading as a Rotation2d */
    public Rotation2d getHeadingR2D() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /** @return the current turn rate in deg/s */
    public double getTurnRate() {
        return imu.getRate() * (DriveConstants.GYRO_REVERSED ? -1 : 1);
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        // In simulation, update the simulated gyro angle from the field visualization.
        imuSim.setGyroAngleZ(field.getRobotPose().getRotation().getDegrees());
        // For a complete simulation, you might integrate chassis speeds to update poseSim.
        // Here we simply leave it unchanged or update it with a dummy twist.
        poseSim = poseSim.exp(new Twist2d(0, 0, 0)); 
        photon.simulationPeriodic(getPose());
    }
}
