package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.utils.LEDs.FlashSolidLEDCommand;
import frc.robot.utils.LEDs.SolidLEDCommand;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {
  private final TalonFX motor = CoralHolderConstants.MOTOR_ID.get();
  private final DigitalInput limitSwitch = new DigitalInput(CoralHolderConstants.LIMIT_SW_ID);

  public CoralHolderSub() {
    setupSmartDash();
  }

  public void forward() {
    motor.set(1);
  }

  public void reverse() {
    motor.set(CoralHolderConstants.THROTTLE);
  }

  public void stop() {
    motor.set(0);
  }

  public Command manualOutCommand() {
    return this.startEnd(this::forward, this::stop);
  }

  public Command manualReverseCommand() {
    return this.startEnd(this::reverse, this::stop);
  }

  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command autoInCommand() {
    return new ParallelCommandGroup(
        manualOutCommand(),
        new SolidLEDCommand(Color.kOrange).withZone()).until(() -> hasCoral())
        .andThen(new SolidLEDCommand(Color.kGreen).withZone());
  }

  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command autoOutCommand() {
    return new ParallelCommandGroup(
        this.runOnce(() -> forward()) // run
            .andThen(new InstantCommand().until(() -> !hasCoral()).withTimeout(3)) // until no coral
            .andThen(new WaitCommand(1)) // ensure it's out
            .finallyDo(() -> stop()),

        new FlashSolidLEDCommand(Color.kBlue, 500).withZone());
  }

  private boolean simLimitSwitch = true;
  private int simHoldlimitState = 0;

  public boolean hasCoral() {
    if (Robot.isSimulation()) {
      if (simHoldlimitState < 0) {
        simLimitSwitch = !simLimitSwitch;
        simHoldlimitState = 75;
      } else {
        simHoldlimitState--;
      }

      return simLimitSwitch;
    }

    return !limitSwitch.get();
  }

  private void setupSmartDash() {
    SmartDashboard.putData("Coral Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Coral Loaded", () -> hasCoral(), null);
        builder.addDoubleProperty("Power", () -> motor.get(), null);
      }
    });
  }
}
