package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.TypeSupliers.EncoderSupplier;
import frc.robot.utils.TypeSupliers.motorsupplier.FalconMotorSupplier;

public class ElevatorConstants {
  protected ElevatorConstants() {
  }

  public static final int MOTOR_ID = 7;
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 8, 9 }, 0.00003896311201);
  public static final Distance GROUND_TO_ELEVATOR = Meters.of(0.55);

  public static final double CONTROLLER_P = 4;
  public static final double CONTROLLER_I = 0;
  public static final double CONTROLLER_D = 0;
  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  public static final double POSITION_TOLERANCE = 0.02;

  public static final Distance MAX_ALLOWABLE_POSITION = Meters.of(0.55);

  public static class ElevatorStops {
    // TODO: tune these at field calibration
    public static final Distance INTAKE = Meters.of(0);
    public static final Distance L1 = Meters.of(0.15);
    public static final Distance L2 = Meters.of(0.3);
    public static final Distance L3 = Meters.of(0.5);
  }

  // ((1m) / (circumference of pulley)) * (gear ratio)
  public static final double ENCODER_RATIO = (1000 / (27.666 * Math.PI)) * 5;

  public static final FalconMotorSupplier LEFT_MOTOR_ID = new FalconMotorSupplier(7)
      .withEncoder(ENCODER_RATIO)
      .withInvert()
      .withBrake();

  public static final FalconMotorSupplier RIGHT_MOTOR_ID = new FalconMotorSupplier(8)
      .withEncoder(ENCODER_RATIO)
      .withBrake();
}
