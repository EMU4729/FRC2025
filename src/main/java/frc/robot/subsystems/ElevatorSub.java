package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Distance;
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
  private boolean disableEStop = false;

  public ElevatorSub() {
    super();
    controller.setIntegratorRange(-0.2, 0.2);
    controller.setIZone(0.05);

    motor = new FalconMotorSupplier(ElevatorConstants.MOTOR_ID)
        .withEncoder(91.37833019)
        .get();
    motor.setPosition(encoder.getDistance());

    controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE);

    SmartDashboard.putData("Elevator Motor", motor);
    SmartDashboard.putData("Elevator Encoder", encoder);
    SmartDashboard.putData("Elevator Controller", controller);
    SmartDashboard.putNumber("targetHeight", 0);
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);
  }

  public Distance getPosition() {
    return Meters.of(encoder.getDistance());
  }

  public void setTargetPosition(Distance position) {
    controller.setGoal(position.in(Meters));
  }

  public Distance getTargetPosition() {
    return Meters.of(controller.getGoal().position);
  }

  public boolean atTargetPosition() {
    return controller.atGoal();
  }

  private EStopState shouldEStop() {
    final var encoderPosition = getPosition();
    final var motorPosition = Meters.of(motor.getPosition().getValueAsDouble());

    if (motorPosition.minus(encoderPosition).abs(Meters) > 0.05) {
      return EStopState.ALL;
    }

    SmartDashboard.putNumber("motor pos", motorPosition.in(Meters));

    if (encoderPosition.in(Meters) < 0 || motorPosition.in(Meters) < 0) {
      return EStopState.BOTTOM;
    }

    if (motorPosition.gt(ElevatorConstants.MAX_ALLOWABLE_POSITION) ||
        encoderPosition.gt(ElevatorConstants.MAX_ALLOWABLE_POSITION)) {
      return EStopState.TOP;
    }

    return EStopState.NONE;
  }

  @Override
  public void periodic() {
    if (true) {
      return;
    }

    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);

    var out = 0.0d;
    if (!atTargetPosition()) {
      final var position = getPosition();
      out = controller.calculate(position.in(Meters));
      out = MathUtil.clamp(out, -0.1, 0.1);
    }

    eStopped = shouldEStop();

    if (!disableEStop) {
      boolean preventMove = false;
      switch (eStopped) {
        case NONE:
          break;
        case TOP:
          if (out > 0)
            preventMove = true;
          break;
        case BOTTOM:
          if (out < 0)
            preventMove = true;
        case ALL:
          preventMove = true;
          break;
      }

      if (preventMove) {
        motor.set(0);
        return;
      }
    }

    motor.set(out);
    SmartDashboard.putNumber("power", out);
    SmartDashboard.putNumber("targetHeight", controller.getGoal().position);
  }

  public void toggleDisableEStop() {
    disableEStop = !disableEStop;
  }

  private enum EStopState {
    NONE,
    TOP,
    BOTTOM,
    ALL
  }

}
