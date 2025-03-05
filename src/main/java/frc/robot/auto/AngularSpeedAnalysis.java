package frc.robot.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class AngularSpeedAnalysis extends Command {
  private final static double MODULE_FREE_SPEED = 5;
  private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.1);
  private double maxSpeed = 0;

  public AngularSpeedAnalysis() {
    addRequirements(Subsystems.drive);
  }

  private void setSpeed(double v) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(v, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(135)),
        new SwerveModuleState(v, Rotation2d.fromDegrees(45)),
    });
  }

  public void execute() {
    final var v = accelerationLimiter.calculate(MODULE_FREE_SPEED);
    setSpeed(v);

    final var chassisSpeeds = Subsystems.nav.getChassisSpeeds();
    final var speed = Math.abs(chassisSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Angular Speed", speed);
    if (speed > maxSpeed) {
      maxSpeed = speed;
      SmartDashboard.putNumber("Max Angular Speed", maxSpeed);
    }
  }

  public void end(boolean interrupted) {
    System.out.println("-------------------------------------------interupt");
    setSpeed(0);
    accelerationLimiter.reset(0);
  }
}
