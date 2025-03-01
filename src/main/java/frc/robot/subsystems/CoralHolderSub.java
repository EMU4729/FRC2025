package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralHolderConstants;

public class CoralHolderSub extends SubsystemBase {
  private final WPI_VictorSPX motorLeft;
  private final WPI_VictorSPX motorRight;
  private final DigitalInput limitSwitch;

  private double leftThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? 1 : -1;
  private double rightThrottleMultiplier = CoralHolderConstants.INVERT_MOTORS ? -1 : 1;

  public CoralHolderSub() {
    motorLeft = new WPI_VictorSPX(CoralHolderConstants.LEFT_CAN_ID);
    motorLeft.setInverted(true);
    motorRight = new WPI_VictorSPX(CoralHolderConstants.RIGHT_CAN_ID);

    var config = new VictorSPXConfiguration();

    motorLeft.configAllSettings(config);
    motorRight.configAllSettings(config);

    motorRight.setInverted(InvertType.FollowMaster);
    motorRight.follow(motorLeft);

    limitSwitch = new DigitalInput(CoralHolderConstants.LIMIT_SW_ID);
  }

  public void forward() {
    motorLeft.set(leftThrottleMultiplier * CoralHolderConstants.THROTTLE);
  }

  public Command forwardCommand() {
    return this.startEnd(this::forward, this::stop);
  }

  public void reverse() {
    motorLeft.set(rightThrottleMultiplier * CoralHolderConstants.THROTTLE);
  }

  public Command reverseCommand() {
    return this.startEnd(this::reverse, this::stop);
  }

  public void stop() {
    motorLeft.set(0);
  }

  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command intakeCommand() {
    return reverseCommand().until(limitSwitch::get);
  }
}
