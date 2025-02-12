package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.EncoderSupplier;

public class ElevatorConstants {
  protected ElevatorConstants() {
  }

  public static final int MOTOR_ID = 7;
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 8, 9 }, 0.00003896311201);
  public static final double GROUND_TO_ELEVATOR = 0;

  public static final double CONTROLLER_P = 0.1;
  public static final double CONTROLLER_I = 0;
  public static final double CONTROLLER_D = 0;
  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  public static final double POSITION_TOLERANCE = 0.1;

  public static final double MAX_ALLOWABLE_POSITION = 0.75;

  public static class ElevatorStops {
    public static final double INTAKE = Units.inchesToMeters((3 * 12) + 1 + (1 / 2)) - GROUND_TO_ELEVATOR;
    public static final double L1 = Units.inchesToMeters((1 * 12) + 6) - GROUND_TO_ELEVATOR;
    public static final double L2 = Units.inchesToMeters((2 * 12) + 7 + (7 / 8)) - GROUND_TO_ELEVATOR;
    public static final double L3 = Units.inchesToMeters((3 * 12) + 11 + (5 / 8)) - GROUND_TO_ELEVATOR;
    public static final double L4 = Units.inchesToMeters(6 * 12) - GROUND_TO_ELEVATOR;
  }
}
