// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.fieldLayout;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultMultiTagStdDevs;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultSingleTagStdDevs;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import frc.robot.utils.VirtualSubsystem;
import frc.robot.utils.Visionhelpers;
import lib.functional.TriConsumer;
import lib.vision.PhotonVisionSource;
import lib.vision.VisionLocalizationSystem;

public class VisionSubsystem extends VirtualSubsystem {
    private final VisionLocalizationSystem m_localizationSystem;

    private final PhotonVisionSource LeftElevatorCamera;
    private final PhotonVisionSource RightElevatorCamera;
    private final PhotonVisionSource RearLeftCamera;
    private final PhotonVisionSource RearRightCamera;

    //private final VisionSystemSim sim;

    private final Supplier<Pose2d> poseSupplier;

    public VisionSubsystem(Supplier<Pose2d> poseSupplier) {

        m_localizationSystem = new VisionLocalizationSystem();
        this.poseSupplier = poseSupplier;

        LeftElevatorCamera = Visionhelpers.buildPVSourceFromConfig(PhotonConfig.ELEVATOR_LEFT_CAM, () -> poseSupplier.get().getRotation());
        RightElevatorCamera = Visionhelpers.buildPVSourceFromConfig(PhotonConfig.ELEVATOR_RIGHT_CAM, () -> poseSupplier.get().getRotation());
        
        RearLeftCamera = Visionhelpers.buildPVSourceFromConfig(PhotonConfig.REAR_LEFT_CAM, () -> poseSupplier.get().getRotation());
        RearRightCamera = Visionhelpers.buildPVSourceFromConfig(PhotonConfig.REAR_RIGHT_CAM, () -> poseSupplier.get().getRotation());

        m_localizationSystem.addSource(LeftElevatorCamera);
        m_localizationSystem.addSource(RightElevatorCamera);
        m_localizationSystem.addSource(RearLeftCamera);
        m_localizationSystem.addSource(RearRightCamera);

    }

    public VisionSubsystem withLocalizationConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        m_localizationSystem.addMeasurementConsumer(consumer);
        return this;
    }

    public void enableRearCameras(boolean enabled) {
        m_localizationSystem.enableSources(enabled,"REAR_LEFT_CAMERA","REAR_RIGHT_CAMERA");
    }

    @Override
    public void periodic() {
        if (SystemConstants.currentMode == 1) {
            sim.update(poseSupplier.get());
        }
        m_localizationSystem.update();
    }
}
