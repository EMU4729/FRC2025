package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSub extends SubsystemBase {
  private final TalonFX motor = ClimberConstants.MOTOR_ID.get();
  private double speed = 0;

  public void up() {
    speed = 1;
  }

  public void down() {
    speed = -1;
  }

  public void stop() {
    speed = 0;
  }

  public Distance getDistance() {
    return Meters.of(motor.getPosition().getValue().in(Radians) * ClimberConstants.WHEEL_DIAMETER.in(Meters) * Math.PI);
  }

  public Command upCommand() {
    return this.startEnd(this::up, this::stop);
  }

  public Command downCommand() {
    return this.startEnd(this::down, this::stop);
  }

  @Override
  public void periodic() {
    final var distance = getDistance();
    if ((speed < 0 && distance.lt(ClimberConstants.LOWER_LIMIT))
        || (speed > 0 && distance.gt(ClimberConstants.UPPER_LIMIT))) {
      speed = 0;
    }
    motor.set(speed);
  }
}
