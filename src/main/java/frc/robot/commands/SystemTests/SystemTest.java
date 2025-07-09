package frc.robot.commands.SystemTests;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;

public class SystemTest extends SequentialCommandGroup {
  public SystemTest() {
    addCommands(
        Subsystems.drive.testFunction());
  }
}
