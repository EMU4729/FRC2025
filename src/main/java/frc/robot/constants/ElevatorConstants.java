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
  public static final Distance GROUND_TO_ELEVATOR = Meters.of(0.55);

  public static final double CONTROLLER_P = 0.3;
  public static final double CONTROLLER_I = 0.1;
  public static final double CONTROLLER_D = 0;
  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  public static final double POSITION_TOLERANCE = 0.02;

  public static final Distance MAX_ALLOWABLE_POSITION = Meters.of(0.78
  );

  public static class ElevatorStops {
    public static final Distance INTAKE = Meters.of(0);
    public static final Distance L1 = Meters.of(0.28);
    public static final Distance L2 = Meters.of(0.4);
    public static final Distance L3 = Meters.of(0.8);
  }
}
