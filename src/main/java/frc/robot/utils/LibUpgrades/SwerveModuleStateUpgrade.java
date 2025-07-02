package frc.robot.utils.LibUpgrades;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModuleStateUpgrade extends SwerveModuleState {
  
  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleStateUpgrade() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleStateUpgrade(double speedMetersPerSecond, Rotation2d angle) {
    super(speedMetersPerSecond, angle);
  }

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speed The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleStateUpgrade(LinearVelocity speed, Rotation2d angle) {
    super(speed, angle);
  }

  public SwerveModuleStateUpgrade(LinearVelocity speed, Angle angle){
    super(speed.in(MetersPerSecond), new Rotation2d(angle));
  }

  public SwerveModuleStateUpgrade(double speedMetersPerSecond, Angle angle){
    super(speedMetersPerSecond, new Rotation2d(angle));
  }
  public SwerveModuleStateUpgrade(SwerveModuleState state){
    super(state.speedMetersPerSecond, state.angle);
  }
  public SwerveModuleStateUpgrade(SwerveModuleStateUpgrade state){
    super(state.speedMetersPerSecond, state.angle);
  }

  public void setSpeed(LinearVelocity speed){
    this.speedMetersPerSecond = speed.in(MetersPerSecond);
  }
  public LinearVelocity getSpeed(){
    return MetersPerSecond.of(this.speedMetersPerSecond);
  }
  public AngularVelocity getWheelSpeed(Distance circumference){
    return RotationsPerSecond.of(speedMetersPerSecond / circumference.in(Meters));
  }

  public void setAngle(Angle angle){
    this.angle = new Rotation2d(angle);
  }
  public Angle getAngle(){
    return Radians.of(this.angle.getRadians());
  }
  

  @Override
  public void optimize(Rotation2d currentAngle){
    //TODO impliment
  }
  public void optimize(Angle currentAngle){
    optimize(new Rotation2d(currentAngle));
  }
}
