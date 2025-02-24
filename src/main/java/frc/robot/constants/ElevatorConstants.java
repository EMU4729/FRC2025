package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.EncoderSupplier;

public class ElevatorConstants {
  protected ElevatorConstants() {
  }

  public static final int MOTOR_ID = 7;
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 8, 9 }, 0.00003896311201);
  public static final Distance GROUND_TO_ELEVATOR = Meters.of(0); // TODO set to non-zero value

  public static final double CONTROLLER_P = 0.1;
  public static final double CONTROLLER_I = 0;
  public static final double CONTROLLER_D = 0;
  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  public static final double POSITION_TOLERANCE = 0.1;

  public static final Distance MAX_ALLOWABLE_POSITION = Meters.of(0.75);

  public static class ElevatorStops {
    public static final Distance INTAKE = Inches.of((3 * 12) + 1 + (1 / 2)).minus(GROUND_TO_ELEVATOR).plus(Feet.of(3));
    public static final Distance L1 = Inches.of((1 * 12) + 6).minus(GROUND_TO_ELEVATOR);
    public static final Distance L2 = Inches.of((2 * 12) + 7 + (7 / 8)).minus(GROUND_TO_ELEVATOR);
    public static final Distance L3 = Inches.of((3 * 12) + 11 + (5 / 8)).minus(GROUND_TO_ELEVATOR);
    public static final Distance L4 = Inches.of(6 * 12).minus(GROUND_TO_ELEVATOR);
  }
}
