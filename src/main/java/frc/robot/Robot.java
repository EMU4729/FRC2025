// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Control.RobotContainer;
import frc.robot.utils.LEDs.BatteryPercentLEDCommand;
import frc.robot.utils.LEDs.ClearLEDCommand;
import frc.robot.utils.LEDs.RainbowLEDCommand;
import frc.robot.utils.LEDs.TeamColorLEDCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // start host the deploy dir for Elastic's Remote Layout Download functionality.
    // https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  BatteryPercentLEDCommand batteryLEDDisplay;

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    new RainbowLEDCommand().withZone().schedule();
    System.out
        .println("Disabled ----------------------------------------------------------------------------------------");
    new TeamColorLEDCommand().withZone(new int[] { 1 }).schedule();
    batteryLEDDisplay = new BatteryPercentLEDCommand();
    batteryLEDDisplay.schedule();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    batteryLEDDisplay.finish();
    new ClearLEDCommand().withZone().schedule();
    autoCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
    System.out
        .println("Auto Start --------------------------------------------------------------------------------------");
    // Subsystems.drive.resetIntegral();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    batteryLEDDisplay.finish();
    new ClearLEDCommand().withZone().schedule();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    System.out
        .println("Teleop Start ------------------------------------------------------------------------------------");
    // Subsystems.swerveDrive.resetIntegral();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    batteryLEDDisplay.finish();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    System.out
        .println("Test Start --------------------------------------------------------------------------------------");

    Subsystems.drive.testFunction().schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
