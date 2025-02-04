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
    

    public FollowTrajectoryCommand(Wristevator wristevator, WristevatorController controller, List<WristevatorState> trajectory) {
        this.trajectory = trajectory;
        m_Wristevator = wristevator;
        this.controller = controller;
        addRequirements(m_Wristevator.m_elevator,m_Wristevator.m_wrist);
    }

    @Override
    public void initialize() {
        targetStateIndex = 0;
        controller.setSetpoint(trajectory.get(targetStateIndex));
    }

    @Override
    public void execute() {
        WristevatorState currentState = m_Wristevator.getCurrentState();
        if (controller.atSetpoint(currentState)) {
            targetStateIndex++;
            controller.setSetpoint(trajectory.get(targetStateIndex));
        }
        m_Wristevator.applySpeeds(controller.calculate(currentState));
        System.out.println(controller.calculate(currentState));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint(m_Wristevator.getCurrentState()) && targetStateIndex == trajectory.size() - 1;
    }
}
