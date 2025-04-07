// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LEDs.FlashSolidLEDCommand;
import frc.robot.LEDs.RainbowLEDCommand;
import frc.robot.LEDs.RepeatedFlashLEDCommand;
import frc.robot.LEDs.SolidLEDCommand;
import frc.robot.auto.AutoProvider;
import frc.robot.auto.autoPath;
import frc.robot.constants.ElevatorConstants;
import frc.robot.teleop.TeleopProvider;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider;
  private final TeleopProvider teleopProvider;

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoProvider = AutoProvider.getInstance();
    teleopProvider = TeleopProvider.getInstance();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Robot Automations
    // flash leds yellow during endgame
    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
        .onTrue(new RepeatedFlashLEDCommand(
            (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kYellow, 300).withZone()), 5));

    NamedCommands.registerCommand("LED Flash", new FlashSolidLEDCommand(Color.kGreen, 500).withZone());
    
    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---

    // OI.pilot.y().onTrue(new
    // InstantCommand(()->BatteryPercentLEDCommand.runFor(50)));
    // OI.pilot.a().onTrue(new FlashSolidLEDCommand(Color.kCrimson,
    // 1000).withZone());
    // OI.pilot.b().onTrue(new RepeatedFlashLEDCommand(
    // (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kYellow,
    // 200).withZone(new int[] { 1, 2 })),
    // 5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // OI.pilot.x().onTrue(new RepeatedFlashLEDCommand(
    // (FlashSolidLEDCommand) (new FlashSolidLEDCommand(Color.kBlue,
    // 200).withZone(new int[] { 0 })),
    // 5));

    OI.pilot.start()
        .onTrue(new InstantCommand(Subsystems.nav::zeroHeading, Subsystems.drive));
    //OI.pilot.back().toggleOnTrue(new RainbowLEDCommand().withZone());
    OI.pilot.povUp().whileTrue(Subsystems.climber.upCommand());
    OI.pilot.povDown().whileTrue(Subsystems.climber.downCommand());
    // Drive bindings handled in teleop command

    OI.pilot.povLeft().whileTrue(new autoPath(-0.15));
    OI.pilot.povRight().whileTrue(new autoPath(0.15));

    // elevator elevations
    Command elevateIntake = new InstantCommand(
        () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.INTAKE),
        Subsystems.elevator);
    OI.copilot.a().onTrue(elevateIntake);
    OI.copilot.b().onTrue(elevateIntake);
    OI.copilot.x().onTrue(elevateIntake);
    OI.copilot.y().onTrue(elevateIntake);
    NamedCommands.registerCommand("elevate Intake", elevateIntake);

    Command elevateL1 = new InstantCommand(
        () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L1),
        Subsystems.elevator);
    OI.copilot.povDown().onTrue(elevateL1);
    NamedCommands.registerCommand("elevate L1", elevateL1);

    Command elevateL2 = new InstantCommand(
        () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L2),
        Subsystems.elevator);
    OI.copilot.povRight().onTrue(elevateL2);
    OI.copilot.povLeft().onTrue(elevateL2);
    NamedCommands.registerCommand("elevate L2", elevateL2);
    
    Command elevateL3 = new InstantCommand(
      () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L3),
      Subsystems.elevator);
      OI.copilot.povUp().onTrue(elevateL3);
      NamedCommands.registerCommand("elevate L3", elevateL3);

    // disable elevator E stop
    //OI.copilot.back().onTrue(new InstantCommand(() -> Subsystems.elevator.toggleDisableEStop()));
    // coral holder
    OI.copilot.rightTrigger().whileTrue(Subsystems.coralHolder.autoInCommand());
    NamedCommands.registerCommand("coral Intake", Subsystems.coralHolder.autoInCommand());
    OI.copilot.leftTrigger().whileTrue(Subsystems.coralHolder.manualOutCommand());
    NamedCommands.registerCommand("coral outTake", Subsystems.coralHolder.autoOutCommand());
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getSelected();
  }
}
