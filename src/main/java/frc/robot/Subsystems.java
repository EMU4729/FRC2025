package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.subsystems.CoralHolderSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.NavigationSub;
import frc.robot.utils.LEDs.LEDZone;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final DriveSub drive = new DriveSub();
  public static final NavigationSub nav = new NavigationSub();
  public static final CoralHolderSub coralHolder = new CoralHolderSub();
  public static final ElevatorSub elevator = new ElevatorSub();
  // public static final ClimberSub climber = new ClimberSub();

  public static final List<LEDZone> ledZones = new ArrayList<LEDZone>(Arrays.asList(
      new LEDZone(new short[] { 0, 108 }, new short[] { 42, 149 }, 0),
      new LEDZone(new short[] { 43, 87 }, new short[] { 63, 107 }, 1),
      new LEDZone(64, 86, 2)));
}
