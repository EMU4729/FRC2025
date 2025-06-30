package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.TypeSupliers.motorsupplier.FalconMotorSupplier;

public class ClimberConstants {
  public static final FalconMotorSupplier MOTOR_ID = new FalconMotorSupplier(9).withBrake().withEncoder(1.0 / 75.0);
  public static final Distance WHEEL_DIAMETER = Millimeters.of(62);
  public static final Distance LOWER_LIMIT = Meters.of(0);
  public static final Distance UPPER_LIMIT = Meters.of(0.18);
}
