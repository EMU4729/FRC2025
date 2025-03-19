package frc.robot.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.fasterxml.jackson.databind.ser.std.ClassSerializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.ClosedSlewRateLimiter;
import frc.robot.utils.SpeedRateLimiter;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  private final ClosedSlewRateLimiter xLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      -DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter yLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      -DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter rLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond),
      -DriveConstants.MAX_ANGULAR_DECELERATION.in(RadiansPerSecondPerSecond));

  private Rotation2d targetYaw = new Rotation2d(0);

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {

    //if (OI.pilot.leftTrigger().getAsBoolean()) {
    //  Subsystems.drive.setX();
    //  return;
    //}
    
    //System.out.println("tods");
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

    final var currentSpeeds = Subsystems.nav.getChassisSpeeds();

    x = xLimiter.calculate(x, currentSpeeds.vxMetersPerSecond);
    y = yLimiter.calculate(y, currentSpeeds.vyMetersPerSecond) ;
    r = rLimiter.calculate(r, currentSpeeds.omegaRadiansPerSecond);

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