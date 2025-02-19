package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.utils.motorsupplier.FalconMotorSupplier;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX motor;
  private final Encoder encoder = ElevatorConstants.ENCODER_ID.get();
  private final ProfiledPIDController controller = new ProfiledPIDController(
      ElevatorConstants.CONTROLLER_P,
      ElevatorConstants.CONTROLLER_I,
      ElevatorConstants.CONTROLLER_D,
      ElevatorConstants.MOTION_CONSTRAINTS);

  private EStopState eStopped = EStopState.NONE;

  public ElevatorSub() {
    super();
    controller.setIntegratorRange(-0.2, 0.2);
    controller.setIZone(0.05);

    motor = new FalconMotorSupplier(ElevatorConstants.MOTOR_ID)
              .withEncoder(91.37833019)
              .get();

    controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE);

    SmartDashboard.putData("Elevator Motor", motor);
    SmartDashboard.putData("Elevator Encoder", encoder);
    SmartDashboard.putData("Elevator Controller", controller);
    SmartDashboard.putNumber("targetHeight", 0);
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public void setTargetPosition(double position) {
    controller.setGoal(position);
  }

  public double getTargetPosition() {
    return controller.getGoal().position;
  }

  public boolean atTargetPosition() {
    return controller.atGoal();
  }

  private EStopState shouldEStop() {
    final var encoderPosition = getPosition();
    final var motorPosition = motor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("motor pos", motorPosition);

    if (encoderPosition < 0 || motorPosition < 0) {
      return EStopState.BOTTOM;
    }
    
    if (motorPosition > ElevatorConstants.MAX_ALLOWABLE_POSITION ||
        encoderPosition > ElevatorConstants.MAX_ALLOWABLE_POSITION) {
      return EStopState.TOP;
    }

    return EStopState.NONE;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);
    
    
    final var position = getPosition();
    var out = controller.calculate(position);
    out = MathUtil.clamp(out, -0.1, 0.1);

    eStopped = shouldEStop();
    boolean preventMove = false;
    switch (eStopped) {
      case NONE:
        break;
      case TOP:
        if (out > 0) preventMove = true;
        break;
      case BOTTOM:
        if (out < 0) preventMove = true;
      break;
    }

    if (preventMove) {
      motor.set(0);
      return;
    }

    motor.set(out);
    SmartDashboard.putNumber("power", out);
    SmartDashboard.putNumber("targetHeight", controller.getGoal().position);
  }

  private enum EStopState {
    NONE,
    TOP,
    BOTTOM
  }

}


