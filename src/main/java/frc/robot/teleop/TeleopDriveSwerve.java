package frc.robot.teleop;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.SpeedRateLimiter;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  private final SlewRateLimiter xLimiter = new SpeedRateLimiter(DriveConstants.MAX_ACCELERATION,
                                                                -DriveConstants.MAX_DECELERATION, 0);
  private final SlewRateLimiter yLimiter = new SpeedRateLimiter(DriveConstants.MAX_ACCELERATION,
                                                                -DriveConstants.MAX_DECELERATION, 0);
  private final SlewRateLimiter rLimiter = new SpeedRateLimiter(DriveConstants.MAX_ANGULAR_ACCELERATION, 
                                                                -DriveConstants.MAX_ANGULAR_DECELERATION, 0);

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    boolean fieldRelative = !OI.pilot.getHID().getBackButton();

    final var control = settings.fitSwerve(
        -OI.pilot.getLeftY(),
        -OI.pilot.getLeftX(),
        -OI.pilot.getRightX(),
        booster,
        limiter);

    var x = control[0];
    var y = control[1];
    var r = control[2];

    x = xLimiter.calculate(x) * DriveConstants.MAX_SPEED;
    y = yLimiter.calculate(y) * DriveConstants.MAX_SPEED;
    r = rLimiter.calculate(r) * DriveConstants.MAX_ANGULAR_SPEED;

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
    Subsystems.drive.drive(speeds, fieldRelative);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
