package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;


public class Vision extends SubsystemBase {
  private final Transform3d kLeftCameraPosition = new Transform3d(Units.inchesToMeters(-10.3), Units.inchesToMeters(11), Units.inchesToMeters(4.5), new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(20)));
  private final Transform3d kRightCameraPosition = new Transform3d(Units.inchesToMeters(10.3), Units.inchesToMeters(11), Units.inchesToMeters(4.5), new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(-20)));
  
  private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  
  private final PhotonPoseEstimator m_LeftPhotonPoseEstimator = new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kLeftCameraPosition);
  private final PhotonPoseEstimator m_RightPhotonPoseEstimator = new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kLeftCameraPosition);
  
  private final Camera[] m_cameras = {
      new Camera(new PhotonCamera("LeftCamera"), m_LeftPhotonPoseEstimator, kLeftCameraPosition),
      new Camera(new PhotonCamera("RightCamera"), m_RightPhotonPoseEstimator, kRightCameraPosition)
  };
  
  private final EstimateConsumer m_estimateConsumer;
  
  public Vision(EstimateConsumer estimateConsumer) {
    m_estimateConsumer = estimateConsumer;
  }
  
  
  @Override
  public void periodic() {
    for (Camera camera : m_cameras) {
      var photonCamera = camera.camera;
      var photonEstimator = camera.poseEstimator;
      for (var change : photonCamera.getAllUnreadResults()) {
        var visionEst = photonEstimator.update(change);
        
        visionEst.ifPresent(
            est -> {
              // Change our trust in the measurement based on the tags we can see
              var estStdDevs = getEstimationStdDevs(visionEst, change.getTargets(), photonEstimator);
              
              m_estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
      }
    }
  }
  
  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private Matrix<N3, N1> getEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEstimator) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      return kSingleTagStdDevs;
      
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;
      
      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }
      
      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        return kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        return estStdDevs;
      }
    }
  }
  
  public record Camera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, Transform3d cameraToRobotTransform) {
  }
  
  @FunctionalInterface
  public interface EstimateConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
