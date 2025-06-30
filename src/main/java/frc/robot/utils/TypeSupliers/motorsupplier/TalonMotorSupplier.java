package frc.robot.utils.TypeSupliers.motorsupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonMotorSupplier extends MotorSupplier<WPI_TalonSRX> {
  public TalonMotorSupplier(int port) {
    super(port);
  }

  public WPI_TalonSRX get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new WPI_TalonSRX(99);
    }
    WPI_TalonSRX talon = new WPI_TalonSRX(port);
    if (!talon.isAlive()) {
      System.out.println(
          "MotorInfo : new WPI_TalonSRX on port " + port + "not found, may not exist or be of wrong type");
    }
    talon.setInverted(invert);
    if (brake) {
      talon.setNeutralMode(NeutralMode.Brake);
    } else {
      talon.setNeutralMode(NeutralMode.Coast);
    }
    talon.setSafetyEnabled(safety);
    talon.enableVoltageCompensation(voltageComp);
    talon.configVoltageCompSaturation(12);
    return talon;
  }
}
