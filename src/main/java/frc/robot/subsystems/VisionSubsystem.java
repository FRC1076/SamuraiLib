// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.VirtualSubsystem;
import lib.functional.TriConsumer;
import lib.vision.VisionLocalizationSystem;

public class VisionSubsystem extends VirtualSubsystem {
    private final VisionLocalizationSystem m_localizationSystem;

    public VisionSubsystem() {
        m_localizationSystem = new VisionLocalizationSystem();
    }

    @Override
    public void periodic() {
        m_localizationSystem.update();
    }
}
