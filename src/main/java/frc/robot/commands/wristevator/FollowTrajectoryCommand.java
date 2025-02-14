// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands.wristevator;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wristevator.Wristevator;
import frc.robot.subsystems.wristevator.control.WristevatorController;
import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

public class FollowTrajectoryCommand extends Command {
    private final Wristevator m_Wristevator;
    private final List<WristevatorState> trajectory;
    private final WristevatorController controller;
    private int targetStateIndex;
    private boolean isFinished;
    

    public FollowTrajectoryCommand(Wristevator wristevator, WristevatorController controller, List<WristevatorState> trajectory) {
        this.trajectory = trajectory;
        m_Wristevator = wristevator;
        this.controller = controller;
        addRequirements(m_Wristevator.m_elevator,m_Wristevator.m_wrist);
    }

    @Override
    public void initialize() {
        isFinished = false;
        targetStateIndex = 0;
        controller.setSetpoint(trajectory.get(targetStateIndex));
    }

    @Override
    public void execute() {
        WristevatorState currentState = m_Wristevator.getCurrentState();
        System.out.println(targetStateIndex);
        if (controller.atSetpoint(currentState)) {
            if (targetStateIndex ==  trajectory.size()-1) {
                isFinished = true;
                return;
            }
            targetStateIndex++;
            controller.setSetpoint(trajectory.get(targetStateIndex));
        }
        m_Wristevator.applySpeeds(controller.calculate(currentState));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
