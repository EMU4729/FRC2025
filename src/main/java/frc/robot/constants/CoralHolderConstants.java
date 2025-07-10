package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import frc.robot.utils.TypeSupliers.motorsupplier.FalconMotorSupplier;

public class CoralHolderConstants {
  public static final Time INTAKE_DURATION = Seconds.of(1);
  public static final double THROTTLE = 0.4;
  public static final FalconMotorSupplier MOTOR_ID = new FalconMotorSupplier(5).withInvert();
}
