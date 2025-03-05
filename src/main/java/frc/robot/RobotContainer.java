// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.auto.AutoProvider;
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
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
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
    OI.pilot.back().toggleOnTrue(new RainbowLEDCommand().withZone());
    OI.pilot.povUp().whileTrue(Subsystems.climber.upCommand());
    OI.pilot.povDown().whileTrue(Subsystems.climber.downCommand());
    // Drive bindings handled in teleop command

    // elevator elevations
    OI.copilot.a().onTrue(
        new InstantCommand(
            () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.INTAKE),
            Subsystems.elevator));
    OI.copilot.povDown()
        .onTrue(new InstantCommand(
            () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L1),
            Subsystems.elevator));
    OI.copilot.povLeft()
        .onTrue(new InstantCommand(
            () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L2),
            Subsystems.elevator));
    OI.copilot.povRight()
        .onTrue(new InstantCommand(
            () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L3),
            Subsystems.elevator));
    OI.copilot.povUp()
        .onTrue(new InstantCommand(
            () -> Subsystems.elevator.setTargetPosition(ElevatorConstants.ElevatorStops.L4),
            Subsystems.elevator));
    // disable elevator E stop
    OI.copilot.back().onTrue(new InstantCommand(() -> Subsystems.elevator.toggleDisableEStop()));
    // coral holder
    OI.copilot.leftTrigger().whileTrue(Subsystems.coralHolder.intakeCommand());
    OI.copilot.rightTrigger().whileTrue(Subsystems.coralHolder.forwardCommand());

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
