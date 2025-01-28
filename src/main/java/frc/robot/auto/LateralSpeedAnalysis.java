package frc.robot.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class LateralSpeedAnalysis extends Command {
  private final static double MODULE_FREE_SPEED = 5;
  private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.1);
  private double maxSpeed = 0;

  public LateralSpeedAnalysis() {
    addRequirements(Subsystems.drive);
  }

  private void setSpeed(double v) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(v, Rotation2d.kZero),
        new SwerveModuleState(v, Rotation2d.kZero),
        new SwerveModuleState(v, Rotation2d.kZero),
        new SwerveModuleState(v, Rotation2d.kZero),
    });
  }

  public void execute() {
    final var v = accelerationLimiter.calculate(MODULE_FREE_SPEED);
    setSpeed(v);

    final var chassisSpeeds = Subsystems.drive.getChassisSpeeds();
    final var speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    SmartDashboard.putNumber("Lateral Speed", speed);
    if (speed > maxSpeed) {
      maxSpeed = speed;
      SmartDashboard.putNumber("Max Lateral Speed", maxSpeed);
    }
  }

  public void end(boolean interrupted) {
    setSpeed(0);
    accelerationLimiter.reset(0);
  }

}
