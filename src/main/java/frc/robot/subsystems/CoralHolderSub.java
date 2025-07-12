package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralHolderConstants;
import frc.robot.constants.DriveConstants;

public class CoralHolderSub extends SubsystemBase {
  private final TalonFX motor = CoralHolderConstants.MOTOR_ID.get();
  private final double loadcurrentThreshold = CoralHolderConstants.loadcurrentThreshold;

  private double baselineTorqueCurrent = 0; //TODO CHANGE THIS
  private boolean isCalibrated = false;

  public CoralHolderSub() {
    setupSmartDash();
    Calibrate();
  }

  public void Calibrate(){
    this.baselineTorqueCurrent = motor.getTorqueCurrent().getValueAsDouble();
    this.isCalibrated = true;
  }

  public boolean isLoadDetected(){
    if (!isCalibrated){
      return false;
    }
    double currentTorque = motor.getTorqueCurrent().getValueAsDouble();
    double difference = currentTorque - baselineTorqueCurrent;

    return difference > loadcurrentThreshold;
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

  private void setupSmartDash() {
    SmartDashboard.putData("Coral Sub", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Duty Cycle", () -> motor.get(), null);
        builder.addDoubleProperty("Torque Current", () -> motor.getTorqueCurrent().getValueAsDouble(), null);
      }
    });
  }
  @Override
  public void periodic(){
    Double TorqueDifference = getTorqueCurrent() - getTorqueCurrent();
  } 
}
