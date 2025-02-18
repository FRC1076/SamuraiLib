// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.functional.TriConsumer;



public class VisionLocalizationSystem {

    private class CamStruct {
        public final CameraLocalizer camera;
        public boolean cameraActive; //Signals whether or not the camera reading should be added to vision measurements

        // sets whether or not the camera is active
        public void setActive(boolean active) {
            cameraActive = active;
        }

        public CamStruct(CameraLocalizer camera) {
            this.camera = camera;
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private TriConsumer<Pose2d,Double,Matrix<N3,N1>> measurementConsumer;

    private final Map<String,CamStruct> cameras = new HashMap<>();
    
    public void registerMeasurementConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        if (measurementConsumer == null) {
            //Initialize measurementConsumer if none exists
            measurementConsumer = consumer;
        } else {
            //If measurementConsumer already exists, then compose it with the new measurementConsumer
            measurementConsumer = measurementConsumer.andThen(consumer);
        }
    }

    /**
     * Adds a camera to the ApriltagLocalizer
     * 
     * @return
     * A BooleanConsumer that can set whether or not the ApriltagLocalizer will consider the measurements from this camera in its pose estimates
     */
    public void addCamera(CameraLocalizer camera) {
        CamStruct camStruct = new CamStruct(camera);
        cameras.put(camera.getName(),camStruct);
    }

    public void enableCameras(boolean enabled, String... cams) {
        for (String camID : cams) {
            cameras.get(camID).setActive(enabled);
        }
    }

    /**
     * Fetches pose estimate from cameras and sends them to all consumers. This function should be called exactly once every main function loop
     */
    public void update() {
        for (var camStruct : cameras.values()) {
            if (camStruct.cameraActive) {
                camStruct.camera.getPoseEstimate().ifPresent(
                    (estimate) -> {
                        measurementConsumer.accept(
                            estimate.pose(),
                            estimate.timestampSeconds(),
                            estimate.stdDevs()
                        );
                    }
                );
            }
        }
    }
}