package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {
    chooser = new SendableChooser<>(); // pub for shuffle board

    // This is here to ensure PathPlanner is configured before we attempt to call
    // anything AutoBuilder-related, since static classes are lazily constructed.
    // I now see why WPILib's docs recommend dependency-injecting subsystems rather
    // than global static access. - Neel
    @SuppressWarnings("unused")
    final var _nav = Subsystems.nav;

    try {
      //final var path = PathPlannerPath.fromPathFile("TestPath");
      chooser.addOption("Test Path", new PathPlannerAuto("New Auto"));
      chooser.addOption("Left Side Auto", new PathPlannerAuto("Left Side Auto"));
      chooser.addOption("Right Side Auto", new PathPlannerAuto("Right Side Auto"));
    } catch (Exception e) {
      System.err.println("Big oopsies when loading PathPlanner Path");
      e.printStackTrace();
    }

    chooser.addOption("Pathfind to Pose Test",
        AutoBuilder.pathfindToPose(
            new Pose2d(1, 1, Rotation2d.k180deg),
            DriveConstants.PATH_CONSTRAINTS));
    chooser.addOption("Lateral Speed Analysis", new LateralSpeedAnalysis());
    chooser.addOption("Angular Speed Analysis", new AngularSpeedAnalysis());

    SmartDashboard.putData("Auto Chooser", chooser);


    
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected().handleInterrupt(()->System.out.println("inter-----------")).finallyDo(()->System.out.println("fin"));
  }
}