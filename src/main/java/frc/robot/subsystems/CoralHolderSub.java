package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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
    SmartDashboard.putData("Coral Sub", this);
  }

  public void stop() {
    set(0);
  }

  public void set(double speed) {
    motor.set(speed);
  }

  public double getTorqueCurrent() {
    return motor.getTorqueCurrent().getValueAsDouble();
  }

  public Command runCommand(double speed) {
    return this.startEnd(() -> set(speed), this::stop);
  }

  public Command runUntilLoadCommand() {
    return new SequentialCommandGroup(
        // set shooter speed
        this.runOnce(() -> this.set(0.4)),
        // wait so we don't detect initial current peak
        new WaitCommand(0.1),
        // wait until the current exceeds a threshold which indicates the coral is
        // loaded
        new WaitUntilCommand(() -> this.getTorqueCurrent() > 25),
        // wait some more to make sure the coral is fully in the shooter
        new WaitCommand(0.3)).finallyDo(this::stop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Duty Cycle", () -> motor.get(), null);
    builder.addDoubleProperty("Torque Current", () -> motor.getTorqueCurrent().getValueAsDouble(), null);
  }
}
