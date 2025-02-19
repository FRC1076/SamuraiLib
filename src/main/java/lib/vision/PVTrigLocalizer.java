package lib.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.apache.commons.lang3.function.Suppliers;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import lib.vision.CameraLocalizer.CommonPoseEstimate;

public class PVTrigLocalizer implements CameraLocalizer {
    private static final Matrix<N3,N1> maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Supplier<Rotation2d> headingSupplier;
    private final Matrix<N3,N1> defaultSingleStdDevs;
    private final Matrix<N3,N1> defaultMultiStdDevs;

    public PVTrigLocalizer(
        PhotonCamera camera, 
        Transform3d offset,
        Supplier<Rotation2d> headingSupplier,
        AprilTagFieldLayout field,
        Matrix<N3,N1> defaultSingleStdDevs,
        Matrix<N3,N1> defaultMultiStdDevs
    ) {
        this.camera = camera;
        this.poseEstimator = new PhotonPoseEstimator(field,PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,offset);
        this.headingSupplier = headingSupplier;
        this.defaultSingleStdDevs = defaultSingleStdDevs;
        this.defaultMultiStdDevs = defaultMultiStdDevs;
    }

    public PVTrigLocalizer withCameraOffset(Transform3d offset){
        setCameraOffset(offset);
        return this;
    }

    public void setCameraOffset(Transform3d offset){
        this.poseEstimator.setRobotToCameraTransform(offset);
    }

    //visionSystem = new visionSystem.withCamera(cameraConfig).withStrategy().withRotationSupplier()
    //visionSystem.update()

    private Matrix<N3,N1> calculateStdDevs(EstimatedRobotPose est) {
        var stdDevs = defaultSingleStdDevs;
        int numTargets = 0;
        double avgDist = 0;
        var targets = est.targetsUsed;
        for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {continue;}
            numTargets++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }
        if (numTargets == 0) {
            return maxStdDevs; //No targets detected, resort to maximum std devs
        }

        // One or more tags visible, run the full heuristic.

        // Decrease std devs if multiple targets are visible
        avgDist /= numTargets;
        if (numTargets > 1) {
            stdDevs = defaultMultiStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTargets == 1 && avgDist > 4){
            //Distance greater than 4 meters, and only one tag detected, resort to maximum std devs
            stdDevs = maxStdDevs;
        } else {
            stdDevs = stdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return stdDevs;
    }

    public Optional<CommonPoseEstimate> getPoseEstimate() {
        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), headingSupplier.get());
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        for (var res : results) {
            visionEst = poseEstimator.update(res);
        }

        return visionEst.map(
            (EstimatedRobotPose estimate) -> {
                return new CommonPoseEstimate(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds,
                    calculateStdDevs(estimate)
                );
            }
        );
    }

    public String getName() {
        return camera.getName();
    }
}
