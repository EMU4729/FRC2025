package frc.robot.classes.photon;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

public class PhotonCameraPoseEstimator {
  private final PhotonCamera cam;
  public final PhotonCameraSim camSim;
  private final PhotonPoseEstimator poseEstimator;
  private int errorCounter = 0;

  public PhotonCameraPoseEstimator(
      String cameraName,
      Transform3d robotToCam,
      AprilTagFieldLayout fieldLayout,
      SimCameraProperties camProps) {
    cam = new PhotonCamera(cameraName);
    camSim = new PhotonCameraSim(cam, camProps);
    camSim.enableProcessedStream(RobotBase.isSimulation());

    poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Transform3d getRobotToCameraTransform() {
    return poseEstimator.getRobotToCameraTransform();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return getLatestResult().flatMap(poseEstimator::update);
  }

  public Optional<PhotonPipelineResult> getLatestResult() {
    if (!cam.isConnected()) {
      printErr("PhotonBridge: Error: Camera not connected");
      return Optional.empty();
    }

    final var results = cam.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }
    // is the latest result at the end or the beginning??
    final var latestResult = results.get(results.size() - 1);
    return Optional.of(latestResult);
  }

  public void reset() {
    reset(new Pose2d());
  }

  public void reset(Pose2d pose) {
    poseEstimator.setLastPose(pose);
    poseEstimator.setReferencePose(pose);
  }

  public void printErr(String message){
    if(errorCounter <= 0){
      System.err.println(message);
      errorCounter = 100;
    }
    errorCounter--;
  }
}
