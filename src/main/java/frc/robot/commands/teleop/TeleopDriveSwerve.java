package frc.robot.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.Control.OI;
import frc.robot.classes.RangeMath.DriveBaseFit;
import frc.robot.commands.auto.AutoProvider;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.LibUpgrades.ClosedSlewRateLimiter;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  private final ClosedSlewRateLimiter xLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter yLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter rLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond),
      DriveConstants.MAX_ANGULAR_DECELERATION.in(RadiansPerSecondPerSecond));
  private Angle AngleAfterAuto;

  private Rotation2d targetYaw = new Rotation2d(0);

  //private Angle angleoffset = Rotation2d.k180deg;

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleop()) {
      return;
    }
   // AngleAfterAuto = AutoProvider.getAngleAfterAuto();
    //Angle offsetAngle = AngleAfterAuto.minus(Subsystems.nav.getIMUHeading());
   // Angle StartAngle = Subsystems.nav.getIMUHeading().plus(angleoffset);


    // if (OI.pilot.leftTrigger().getAsBoolean()) {
    // Subsystems.drive.setX();
    // return;
    // }

    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    boolean fieldRelative = !OI.pilot.getHID().getLeftBumperButton();

    final var control = settings.fitSwerve(
        -OI.pilot.getLeftY(),
        -OI.pilot.getLeftX(),
        -OI.pilot.getRightX(),
        booster,
        limiter);

    var x = control[0] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var y = control[1] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var r = control[2] * DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond);

    final var currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(Subsystems.nav.getChassisSpeeds(),
        new Rotation2d(Subsystems.nav.getIMUHeading()));

    // if we are on the red alliance and driving field-relative, we should invert
    // driver inputs since the field's origin is taken from the blue alliance
    if (fieldRelative) {
      final var onRedAlliance = DriverStation.getAlliance()
          .map(alliance -> alliance == Alliance.Red)
          .orElse(false);
      if (onRedAlliance) {
        x *= -1;
        y *= -1;
      }
    }

    x = xLimiter.calculate(x, currentSpeeds.vxMetersPerSecond);
    y = yLimiter.calculate(y, currentSpeeds.vyMetersPerSecond);
    r = rLimiter.calculate(r, currentSpeeds.omegaRadiansPerSecond);

    final var speeds = new ChassisSpeeds(x, y, r);
    if (r == 0) {
      Subsystems.drive.driveAtAngle(speeds, fieldRelative, targetYaw);
    } else {
      Subsystems.drive.drive(speeds, fieldRelative);
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}