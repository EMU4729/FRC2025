package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.LEDs.FlashSolidLEDCommand;
import frc.robot.LEDs.LEDCommand;
import frc.robot.LEDs.RepeatedFlashLEDCommand;
import frc.robot.LEDs.SolidLEDCommand;
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

    setupSmartDash();
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
    return new ParallelCommandGroup(
            manualOutCommand(), 
            new SolidLEDCommand(Color.kOrange).withZone()
        ).until(() -> hasCoral())
        .andThen(new SolidLEDCommand(Color.kGreen).withZone());
  }

  /**
   * @return a {@link Command} that runs the coral holder in reverse until the
   *         limit switch is triggered, indicating that a coral was intaken.
   */
  public Command autoOutCommand() {
    return 
        new ParallelCommandGroup(
            this.runOnce(()->forward())                                                      //run
            .andThen(new InstantCommand().until(()-> !hasCoral()).withTimeout(3))    //until no coral
            .andThen(new WaitCommand(1))                                             //ensure it's out
            .finallyDo(()->stop()),

            new FlashSolidLEDCommand(Color.kBlue, 500).withZone()
        );
  }

  private boolean simLimitSwitch = true;
  private int simHoldlimitState = 0;
  public boolean hasCoral(){
    if(Robot.isSimulation()){
      if(simHoldlimitState < 0){
        simLimitSwitch = !simLimitSwitch;
        simHoldlimitState = 75;
      } else {
        simHoldlimitState--;
      }

      return simLimitSwitch;
    }

    return !limitSwitch.get();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Coral Loaded", hasCoral());
    //SmartDashboard.putNumber("Coral Power", motorLeft.get());
  }

  private void setupSmartDash(){
    
    SmartDashboard.putData("Coral Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Coral Loaded", ()->hasCoral(), null);
        builder.addDoubleProperty("Power Left", ()->motorLeft.get(), null);
        builder.addDoubleProperty("Power Right", ()->motorRight.get(), null);
      }
    });
  }
}
