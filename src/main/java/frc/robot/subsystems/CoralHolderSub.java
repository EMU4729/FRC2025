package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {
  private final TalonSRX motorLeft;
  private final TalonSRX motorRight;
  private final DigitalInput limitSwitch;

  private double leftThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? -1 : 1;
  private double rightThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? 1 : -1;

  public CoralHolderSub() {
    motorLeft = new TalonSRX(CoralHolderConstants.LEFT_CAN_ID);
    motorRight = new TalonSRX(CoralHolderConstants.RIGHT_CAN_ID);

    var config = new TalonSRXConfiguration();

    motorLeft.configAllSettings(config);
    motorRight.configAllSettings(config);

    motorRight.setInverted(InvertType.OpposeMaster);
    motorRight.follow(motorLeft);

    limitSwitch = new DigitalInput(CoralHolderConstants.LIMIT_SW_ID);
  }

  public void forward() {
    motorLeft.set(TalonSRXControlMode.PercentOutput, leftThrottleMultiplier * CoralHolderConstants.THROTTLE);
  }

  public Command forwardCommand() {
    return this.startEnd(this::forward, this::stop);
  }

  public void reverse() {
    motorLeft.set(TalonSRXControlMode.PercentOutput, rightThrottleMultiplier * CoralHolderConstants.THROTTLE);
  }

  public Command reverseCommand() {
    return this.startEnd(this::reverse, this::stop);
  }

  public void stop() {
    motorLeft.set(TalonSRXControlMode.PercentOutput, 0);
  }

  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command intakeCommand() {
    return reverseCommand().until(limitSwitch::get);
  }
}
