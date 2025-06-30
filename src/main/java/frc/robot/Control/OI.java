package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.OIConstants;

/**
 * OI - Use this class to access and initialize all controller-related stuff.
 */
public class OI {
  /**
   * Checks if the pilot is moving the robot.
   * 
   * @return true if the pilot is moving the robot, false otherwise.
   */
  public static boolean pilotIsActive() {
    return Math.abs(pilot.getLeftY()) > OIConstants.CONTROLLER_AXIS_DEADZONE ||
        Math.abs(pilot.getRightY()) > OIConstants.CONTROLLER_AXIS_DEADZONE ||
        Math.abs(pilot.getRightX()) > OIConstants.CONTROLLER_AXIS_DEADZONE;
  }

  /** The pilot's controller */
  public static final CommandXboxController pilot = new CommandXboxController(
      OIConstants.PILOT_XBOX_CONTROLLER_PORT);

  /** The copilot's controller */
  public static final CommandXboxController copilot = new CommandXboxController(
      OIConstants.COPILOT_XBOX_CONTROLLER_PORT);
}
