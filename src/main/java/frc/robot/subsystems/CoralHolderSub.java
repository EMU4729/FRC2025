package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.CoralHolderConstants;
import frc.robot.constants.ElevatorConstants;

public class CoralHolderSub extends SubsystemBase {
  private final WPI_VictorSPX motorLeft;
  private final WPI_VictorSPX motorRight;
  private final DigitalInput limitSwitch;

  public CoralHolderSub() {
    motorLeft = new WPI_VictorSPX(CoralHolderConstants.LEFT_CAN_ID);
    motorLeft.setInverted(true);
    motorRight = new WPI_VictorSPX(CoralHolderConstants.RIGHT_CAN_ID);

    var config = new VictorSPXConfiguration();

    motorLeft.configAllSettings(config);
    motorRight.configAllSettings(config);

    motorRight.setInverted(false);
    motorLeft.setInverted(false);
    // motorRight.follow(motorLeft);

    limitSwitch = new DigitalInput(CoralHolderConstants.LIMIT_SW_ID);
  }

  public void forward() {
    if(Subsystems.elevator.getTargetPosition() == ElevatorConstants.ElevatorStops.L1){
      motorLeft.set(0.8); //flick left
    } else {
      motorLeft.set(1); //run straight
    }
    motorRight.set(0.8);
  }
  
  public void reverse() {
    motorLeft.set(CoralHolderConstants.THROTTLE);
    motorRight.set(CoralHolderConstants.THROTTLE_ALT);
  }
  
  
  public void stop() {
    motorLeft.set(0);
    motorRight.set(0);
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
    return manualOutCommand().until(() -> !hasCoral());
  }
  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command autoOutCommand() {
    return this.runOnce(()->forward())                                                      //run
        .andThen(new InstantCommand().until(this::hasCoral).withTimeout(3))         //until no coral
        .andThen(new WaitCommand(1))                                                //ensure it's out
        .finallyDo(()->stop());                                                             //stop
  }

  public boolean hasCoral(){
    if(Robot.isSimulation()){return new Random().nextBoolean();}
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Holder Limit Switch", hasCoral());
    SmartDashboard.putNumber("Coral Power", motorLeft.get());
  }
}
