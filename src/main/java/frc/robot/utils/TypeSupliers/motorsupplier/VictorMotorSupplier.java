package frc.robot.utils.TypeSupliers.motorsupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class VictorMotorSupplier extends MotorSupplier<WPI_VictorSPX> {
  public VictorMotorSupplier(int port) {
    super(port);
  }

  public WPI_VictorSPX get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new WPI_VictorSPX(99);
    }
    WPI_VictorSPX victor = new WPI_VictorSPX(port);
    if (!victor.isAlive()) {
      System.out.println(
          "MotorInfo : new WPI_VictorSPX on port " + port + "not found, may not exist or be of wrong type");
    }
    victor.setInverted(invert);
    if (brake) {
      victor.setNeutralMode(NeutralMode.Brake);
    } else {
      victor.setNeutralMode(NeutralMode.Coast);
    }
    victor.setSafetyEnabled(safety);
    victor.enableVoltageCompensation(voltageComp);
    victor.configVoltageCompSaturation(12);
    return victor;
  }
}
