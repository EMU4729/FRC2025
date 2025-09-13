package frc.robot.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.Control.OI;
import frc.robot.classes.RangeMath.DriveBaseFit;
import frc.robot.constants.DriveConstants;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    if (!DriverStation.isTeleop()) {
      return;
    }

    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    boolean fieldRelative = !OI.pilot.getHID().getLeftBumperButton();

    final var control = settings.fitSwerve(
        -OI.pilot.getLeftY(),
        -OI.pilot.getLeftX(),
        -OI.pilot.getRightX(),
        booster,
        limiter);

    var x = -control[0] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var y = -control[1] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var r = control[2] * DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond);

    final var speeds = new ChassisSpeeds(x, y, r);
    Subsystems.drive.drive(speeds, fieldRelative, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}