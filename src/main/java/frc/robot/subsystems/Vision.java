package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final Transform3d kLeftCameraPosition = new Transform3d(Units.inchesToMeters(-10.3),Units.inchesToMeters(11),Units.inchesToMeters(4.5),new Rotation3d(0.0,Units.degreesToRadians(-30),Units.degreesToRadians(20)));
    private final Transform3d kRightCameraPosition = new Transform3d(Units.inchesToMeters(10.3),Units.inchesToMeters(11),Units.inchesToMeters(4.5),new Rotation3d(0.0,Units.degreesToRadians(-30),Units.degreesToRadians(-20)));

    private final double kAmbiguity = 0.0;

    private final PhotonCamera m_LeftCamera = new PhotonCamera("LeftCamera");
    private final PhotonCamera m_RightCamera = new PhotonCamera("RightCamera");

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private PhotonTrackedTarget leftLatestTarget;
    private PhotonTrackedTarget rightLatestTarget;

    private PhotonPipelineResult leftLatestResult;
    private PhotonPipelineResult rightLatestResult;

    @Override
    public void periodic() {
        List<PhotonPipelineResult> LeftPipelineResults = m_LeftCamera.getAllUnreadResults();
        if(LeftPipelineResults.size() > 0){
            leftLatestResult = LeftPipelineResults.get(LeftPipelineResults.size() - 1);
            leftLatestTarget = leftLatestResult.getBestTarget();
        }
    }

    public Pose3d getRobotPose(){
        if(leftLatestTarget == null) return new Pose3d(); 
        if (fieldLayout.getTagPose(leftLatestTarget.getFiducialId()).isPresent()) {
            return PhotonUtils.estimateFieldToRobotAprilTag(leftLatestTarget.getBestCameraToTarget(), fieldLayout.getTagPose(leftLatestTarget.getFiducialId()).get(), kLeftCameraPosition);
        }
        return new Pose3d();
    }

    public double getLatestTimestamp(){
        if(leftLatestTarget == null) return 0.0;
        return leftLatestResult.getTimestampSeconds();
    }
}
