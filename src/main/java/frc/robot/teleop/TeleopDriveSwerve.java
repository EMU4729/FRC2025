package frc.robot.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import javax.print.attribute.standard.Fidelity;

import com.fasterxml.jackson.databind.ser.std.ClassSerializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.CoralOutake.CoralOutakeInputPosition;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.ClosedSlewRateLimiter;
import frc.robot.utils.RangeMath.DriveBaseFit;

public class TeleopDriveSwerve extends Command {
  private final DriveBaseFit settings;
  private final ClosedSlewRateLimiter xLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter yLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond),
      DriveConstants.MAX_DECELERATION.in(MetersPerSecondPerSecond));
  private final ClosedSlewRateLimiter rLimiter = new ClosedSlewRateLimiter(
      DriveConstants.MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond),
      DriveConstants.MAX_ANGULAR_DECELERATION.in(RadiansPerSecondPerSecond));
      private final CoralOutakeInputPosition coralPositioner;

  private Rotation2d targetYaw = new Rotation2d(0);

  private final HolonomicDriveController controller;

  public TeleopDriveSwerve(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
    this.coralPositioner = new CoralOutakeInputPosition(new Pose2d()); 

    

        // Initialize the software controller.
        // TUNE THESE GAINS for robot!
        this.controller = new HolonomicDriveController(
                new PIDController(1.5, 0, 0),      // X controller
                new PIDController(1.5, 0, 0),      // Y controller
                new ProfiledPIDController(      // Theta controller
                        3.0, 0, 0,
                        new TrapezoidProfile.Constraints(
                                6.28, // Max angular velocity (rad/s)
                                3.14  // Max angular acceleration (rad/s^2)
                        )
                )
        );
        controller.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
  }

  

  @Override
  public void execute() {
    if(!DriverStation.isTeleop()){return;}

    AutoAlignRobotToCoral();

    //if (OI.pilot.leftTrigger().getAsBoolean()) {
    //  Subsystems.drive.setX();
    //  return;


    //}

    double limiter = OI.pilot.getRightTriggerAxis();
    double booster = OI.pilot.getHID().getRightBumperButton() ? 1 : 0;
    boolean fieldRelative = !OI.pilot.getHID().getLeftBumperButton();
    
    final var control = settings.fitSwerve(
        -OI.pilot.getLeftY(),
        -OI.pilot.getLeftX(),
        -OI.pilot.getRightX(),
        booster,
        limiter);

    var x = control[0] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var y = control[1] * DriveConstants.MAX_SPEED.in(MetersPerSecond);
    var r = control[2] * DriveConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond);

    final var currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(Subsystems.nav.getChassisSpeeds(), Subsystems.nav.getHeadingR2D());

    // if we are on the red alliance and driving field-relative, we should invert
    // driver inputs since the field's origin is taken from the blue alliance
    if (fieldRelative) {
      final var onRedAlliance = DriverStation.getAlliance()
          .map(alliance -> alliance == Alliance.Red)
          .orElse(false);
      if (onRedAlliance) {
        x *= -1;
        y *= -1;
      }
    }

    x = xLimiter.calculate(x, currentSpeeds.vxMetersPerSecond);
    y = yLimiter.calculate(y, currentSpeeds.vyMetersPerSecond);
    r = rLimiter.calculate(r, currentSpeeds.omegaRadiansPerSecond);

    final var speeds = new ChassisSpeeds(x, y, r);
    if (OI.pilot.getHID().getAButton()) {
      //TODO TUNE THE PID BEFORE DDU
      double headingangle = Math.atan2(-OI.pilot.getRightX(),-OI.pilot.getRightY());
      System.out.println(headingangle);
      r =Math.hypot(OI.pilot.getRightX(), OI.pilot.getRightY());
      if (r > 0.5){
        Subsystems.drive.driveAtAngle(speeds, fieldRelative, new Rotation2d(headingangle));
      }
      else{
        speeds.omegaRadiansPerSecond = 0;
        Subsystems.drive.drive(speeds, fieldRelative);
      }
    
    } else {
      Subsystems.drive.drive(speeds, fieldRelative);
    }

  }
  

  public void AutoAlignRobotToCoral(){

    
    if (OI.pilot.getHID().getBButton()){
      //System.out.println("AutoAlignRObot Coral working");
      coralPositioner.SetCoordinateValues();
      Pose2d CurrentPose_fromcam = Subsystems.nav.getPose();
      System.out.println(CurrentPose_fromcam);
      
      Pose2d targetcoordinates = coralPositioner.calculateTargetPoseFromClosestAprilTag(CurrentPose_fromcam);
      
      //TODO this targetcoordinates is null
      ChassisSpeeds speeds = controller.calculate(CurrentPose_fromcam, targetcoordinates, 0, targetcoordinates.getRotation());
  
      Subsystems.drive.drive(speeds);

      if (CurrentPose_fromcam == targetcoordinates){
        System.out.println("target = current");
        Subsystems.drive.drive(new ChassisSpeeds(0,0,0));
      }
    }

    

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}