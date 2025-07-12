package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {
  private final TalonFX motor = CoralHolderConstants.MOTOR_ID.get();

  public CoralHolderSub() {
    setupSmartDash();
  }

  public void stop() {
    set(0);
  }

  public void forward() {
    set(CoralHolderConstants.THROTTLE);
  }

  public void reverse() {
    set(-CoralHolderConstants.THROTTLE);
  }

  public void set(double speed) {
    motor.set(speed);
  }

  public Command runCommand(double speed) {
    return this.startEnd(() -> set(speed), this::stop);
  }

  public Command manualOutCommand() {

    return this.startEnd(this::forward, this::stop);
  }

  public Command manualReverseCommand() {
    return this.startEnd(this::reverse, this::stop);

  }

  public Command timedInCommand() {
    return manualOutCommand().withTimeout(CoralHolderConstants.INTAKE_DURATION);
  }

  public double getTorqueCurrent(){
    return motor.getTorqueCurrent().getValueAsDouble();
  }

  public double getTorque() {
    final var power = motor.getTorqueCurrent().getValueAsDouble() * motor.getSupplyVoltage().getValueAsDouble();
    final var rps = motor.getRotorVelocity().getValueAsDouble();
    return (60 * power) / (2 * Math.PI * rps);
  }

  public Command runUntilLoadCommand() {
    return new SequentialCommandGroup(
      // set shooter speed
      this.runOnce(() -> this.set(0.4)),
      // wait so we don't detect initial current peak
      new WaitCommand(0.1),
      // wait until the current exceeds a threshold which indicates the coral is loaded
      new WaitUntilCommand(() -> this.getTorqueCurrent() > 25),
      // wait some more to make sure the coral is fully in the shooter
      new WaitCommand(0.3)
    ).finallyDo(this::stop);
  }

  private void setupSmartDash() {
    SmartDashboard.putData("Coral Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Duty Cycle", () -> motor.get(), null);
        builder.addDoubleProperty("Torque Current", () -> motor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Calculated Torque", () -> getTorque(), null);
        builder.addDoubleProperty("RPS", () -> motor.getRotorVelocity().getValueAsDouble(), null);
      }
    });
  }
  @Override
  public void periodic(){
    Double TorqueDifference = getTorqueCurrent() - getTorqueCurrent();
  }
}
