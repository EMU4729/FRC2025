package frc.robot.auto.SystemTests;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.LEDs.SolidLEDCommand;

public class SystemTest extends SequentialCommandGroup {
  public SystemTest(){
    addCommands(
      new ParallelRaceGroup(
        new RunCommand(()->setAll(MetersPerSecond.of(0), Degrees.of(0))),
        new WaitCommand(1)),

        //test turning motors

      new ParallelRaceGroup(
        new RunCommand(()->setFL(MetersPerSecond.of(0), Degrees.of(90))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[0].angle.minus(Rotation2d.kCCW_90deg)).getDegrees(), 0, 5)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setFR(MetersPerSecond.of(0), Degrees.of(90))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[1].angle.minus(Rotation2d.kCCW_90deg)).getDegrees(), 0, 5)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setBL(MetersPerSecond.of(0), Degrees.of(90))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[2].angle.minus(Rotation2d.kCCW_90deg)).getDegrees(), 0, 5)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setBR(MetersPerSecond.of(0), Degrees.of(90))),
        new WaitCommand(1)),
        new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[3].angle.minus(Rotation2d.kCCW_90deg)).getDegrees(), 0, 5)).withTimeout(1),

        // test drive motors
        
      new ParallelRaceGroup(
        new RunCommand(()->setFL(MetersPerSecond.of(1), Degrees.of(0))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[0].speedMetersPerSecond - 1), 0, 0.2)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setFR(MetersPerSecond.of(1), Degrees.of(0))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[1].speedMetersPerSecond - 1), 0, 5)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setBL(MetersPerSecond.of(1), Degrees.of(0))),
        new WaitCommand(1)),
      new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[2].speedMetersPerSecond - 1), 0, 5)).withTimeout(1),
      new ParallelRaceGroup(
        new RunCommand(()->setBR(MetersPerSecond.of(1), Degrees.of(0))),
        new WaitCommand(1)),
        new ConditionalCommand(new SolidLEDCommand(Color.kGreen).withZone(), new SolidLEDCommand(Color.kRed).withZone(),
        ()->MathUtil.isNear((Subsystems.drive.getModuleStates()[3].speedMetersPerSecond - 1), 0, 5)).withTimeout(1),

      new ParallelRaceGroup(
        new RunCommand(()->setAll(MetersPerSecond.of(0), Degrees.of(0))),
        new WaitCommand(1))

    );
  }

  private void setAll(LinearVelocity v, Angle a) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
    });
  }

  private void setFL(LinearVelocity v, Angle a) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
    });
  }
  private void setFR(LinearVelocity v, Angle a) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
    });
  }
  private void setBL(LinearVelocity v, Angle a) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
        new SwerveModuleState(0, Rotation2d.fromRadians(0)),
    });
  }
  private void setBR(LinearVelocity v, Angle a) {
    Subsystems.drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(0, Rotation2d.fromRadians(0)),
      new SwerveModuleState(v.in(MetersPerSecond), Rotation2d.fromRadians(a.in(Radians))),
    });
  }
}
