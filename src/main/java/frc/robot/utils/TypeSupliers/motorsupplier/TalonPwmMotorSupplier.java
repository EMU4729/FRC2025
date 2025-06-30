package frc.robot.utils.TypeSupliers.motorsupplier;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class TalonPwmMotorSupplier extends MotorSupplier<Talon> {
  public TalonPwmMotorSupplier(int port) {
    super(port);
  }

  @Override
  public MotorSupplier<Talon> withBrake() {
    throw new UnsupportedOperationException();
  }

  @Override
  public MotorSupplier<Talon> withVoltageComp() {
    throw new UnsupportedOperationException();
  }
  
  public Talon get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new Talon(port);
    }
    Talon talon = new Talon(port);
    if (!talon.isAlive()) {
      System.out.println(
        "MotorInfo : new WPI_TalonSRX on port " + port + "not found, may not exist or be of wrong type");
    }
    talon.setInverted(invert);
    talon.setSafetyEnabled(safety);
    return talon;
  }
}
