package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {
  private final TalonFX motor = CoralHolderConstants.MOTOR_ID.get();

  public CoralHolderSub() {
    setupSmartDash();
  }

  public void forward() {
    motor.set(CoralHolderConstants.THROTTLE);
  }

  public void reverse() {
    motor.set(-CoralHolderConstants.THROTTLE);
  }

  public void stop() {
    motor.set(0);
  }

  public Command manualOutCommand() {
    
    return this.startEnd(this::forward, this::stop).withTimeout(CoralHolderConstants.OUTTAKE_DURATION);
  }

  public Command manualReverseCommand() {
    return this.startEnd(this::reverse, this::stop);
    
  }

  public Command timedInCommand() {
    return manualOutCommand().withTimeout(CoralHolderConstants.INTAKE_DURATION);
  }



  private void setupSmartDash() {
    SmartDashboard.putData("Coral Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Power", () -> motor.get(), null);
      }
    });
  }
}
