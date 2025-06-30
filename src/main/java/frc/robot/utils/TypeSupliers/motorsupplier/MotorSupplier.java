package frc.robot.utils.TypeSupliers.motorsupplier;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class MotorSupplier<T extends MotorController> implements Supplier<T> {
  public final int port;

  public boolean invert = false;
  public boolean brake = false;
  public boolean safety = false;
  public boolean voltageComp = false;

  public MotorSupplier(int motorPort) {
    this.port = motorPort;
  }

  public MotorSupplier<T> withBrake() {
    brake = true;
    return this;
  }

  public MotorSupplier<T> withVoltageComp() {
    voltageComp = true;
    return this;
  }

  public MotorSupplier<T> withInvert() {
    invert = true;
    return this;
  }

  public MotorSupplier<T> withSafety() {
    safety = true;
    return this;
  }

  public abstract T get();
}
