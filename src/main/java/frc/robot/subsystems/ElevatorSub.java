package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Control.OI;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSub extends SubsystemBase {
  private final TalonFX motorLeft = ElevatorConstants.LEFT_MOTOR_ID.get();
  private final TalonFX motorRight = ElevatorConstants.RIGHT_MOTOR_ID.get();

  private final PIDController controller = new PIDController(
    ElevatorConstants.CONTROLLER_P, ElevatorConstants.CONTROLLER_I, ElevatorConstants.CONTROLLER_D);

  private EStopState eStopped = EStopState.NONE;
  private boolean disableEStop = false;

  public ElevatorSub() {
    motorLeft.setPosition(0);
    motorRight.setPosition(0);

    

    motorRight.setControl(new Follower(motorLeft.getDeviceID(), true));

    controller.setIntegratorRange(-0.2, 0.2);
    controller.setIZone(0.05);
    controller.setTolerance(ElevatorConstants.POSITION_TOLERANCE);

    SmartDashboard.putData("Elevator Motor", motorLeft);
    SmartDashboard.putData("Elevator Controller", controller);
    SmartDashboard.putNumber("targetHeight", 0);
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);
  }

  public Distance getPosition() {
    return Meters.of((motorLeft.getPosition().getValueAsDouble() + motorRight.getPosition().getValueAsDouble()) / 2);
  }

  public void setTargetPosition(Distance position) {
    controller.setSetpoint(position.in(Meters));
  }

  public Distance getTargetPosition() {
    return Meters.of(controller.getSetpoint());
  }

  public boolean atTargetPosition() {
    return controller.atSetpoint();
  }

  private EStopState shouldEStop() {
    final var motorLeftPosition = Meters.of(motorLeft.getPosition().getValueAsDouble());
    final var motorRightPosition = Meters.of(motorRight.getPosition().getValueAsDouble());
    final var position = motorLeftPosition.plus(motorRightPosition).div(2);

    if (motorLeftPosition.minus(motorRightPosition).abs(Meters) > 0.1) {
      return EStopState.ALL;
    }

    SmartDashboard.putNumber("motor pos", position.in(Meters));

    if (motorLeftPosition.lt(Meters.of(0.02)) || motorRightPosition.lt(Meters.of(0.02))) {
      return EStopState.BOTTOM;
    }

    if (motorLeftPosition.gt(ElevatorConstants.MAX_ALLOWABLE_POSITION) ||
        motorRightPosition.gt(ElevatorConstants.MAX_ALLOWABLE_POSITION)) {
      return EStopState.TOP;
    }

    return EStopState.NONE;
  }

  public void modifyTargetPosition(Distance amount) {
    Distance newTarget = getTargetPosition().plus(amount);
    if (newTarget.gt(ElevatorConstants.MAX_ALLOWABLE_POSITION)) {
      newTarget = ElevatorConstants.MAX_ALLOWABLE_POSITION;
    } else if (newTarget.lt(Meters.of(0))) {
      newTarget = Meters.of(0);
    }
    setTargetPosition(newTarget);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Elevator E-Stopped", eStopped != EStopState.NONE);
    SmartDashboard.putString("Elevator E-Stop State", eStopped.toString());
    SmartDashboard.putNumber("Elevator Distance", getPosition().in(Meters));
    SmartDashboard.putNumber("Elevator Left Dist", motorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Right Dist", motorRight.getPosition().getValueAsDouble());

    var out = 0.0d;
    if (!atTargetPosition()) {
      final var position = getPosition();
      out = controller.calculate(position.in(Meters)) + 0.1;
      out = MathUtil.clamp(out, -1, 1);
    } else {

      // motor.setPosition(getPosition().in(Meters));
    }

    eStopped = shouldEStop();

    if (!disableEStop) {
      boolean preventMove = false;
      boolean slowMove = false;
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
          break;
        case ALL:
          preventMove = false;
          slowMove = true;
          break;
      }

      if (preventMove) {
        motorLeft.set(0);
        return;
      } else if (slowMove) {
        out = MathUtil.clamp(out, -0.1, 0.1);
      }
    }

    motorLeft.set(out);

    SmartDashboard.putNumber("power", out);
    SmartDashboard.putNumber("targetHeight", controller.getSetpoint());
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
