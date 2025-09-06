package frc.robot.utils.LibUpgrades;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveModuleStateUpgrade extends SwerveModuleState {
  private boolean optimised_last_loop = false;
  
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
  public void optimize(Rotation2d angle){


    //throw new UnsupportedOperationException("Do Not Use : optimize(Angle currentAngle, AngularVelocity speed, AngularAcceleration maxAcceleration, AngularVelocity maxSpeed) is better");
  }

  public void optimize(Angle currentAngle, AngularVelocity currentVelocity, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration){
    //there is a big simplification here
    //we expect acceleration to be high
    //high enough to assume it is near infinite
    //is this right no, but the time difference is also almost nothing
    //so it is close enough

    currentVelocity = RadiansPerSecond.of(Math.copySign(maxVelocity.in(RadiansPerSecond), currentVelocity.in(RadiansPerSecond)));

    double d1 = currentAngle.minus(getAngle()).in(Radians);
    double d2 = currentAngle.minus(Radians.of(Math.PI)).minus(getAngle()).in(Radians);
    d1 += d1 < -Math.PI ? 2*Math.PI : (d1 > Math.PI ? -2*Math.PI : 0);
    d2 += d2 < -Math.PI ? 2*Math.PI : (d2 > Math.PI ? -2*Math.PI : 0);  
    
    double t1 = Math.abs(d1 / currentVelocity.in(RadiansPerSecond));
    double t2 = Math.abs(d2 / currentVelocity.in(RadiansPerSecond));

    double t_inv = 2*Math.abs(currentVelocity.in(RadiansPerSecond)) / maxAcceleration.in(RadiansPerSecondPerSecond);

    if (Math.signum(d1) == Math.signum(currentVelocity.in(RadiansPerSecond))){
      t2 += t_inv;
    } else {
      t1 += t_inv;
    }

    if(optimised_last_loop){
      t1 *= 1.05;
    } else {
      t2 *= 1.05;
    }

    if(t1 > t2){
      optimised_last_loop = true;
      speedMetersPerSecond *= -1;
      angle = angle.plus(Rotation2d.k180deg);
    } else {
      optimised_last_loop = false;
    }
    System.out.println(d1+" "+d2+" "+t1+" "+t2+" "+optimised_last_loop);
  }
}
