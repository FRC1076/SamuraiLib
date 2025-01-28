package frc.robot.subsystems;

import frc.robot.Constants.ElevatorSimConstants;
import frc.robot.Constants.SuperstructureConstants.SuperState;
import frc.robot.Constants.SuperstructureConstants.GamePieceState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.wrist.SetWristAngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.util.Units;

public class Superstructure {
    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private Boolean m_indexBreamBreak = false;
    private Boolean m_transferBeamBreak = false;
    private Boolean m_grabberEndBreamBreak = false;
    private SuperState superState;
    private GamePieceState gamePieceState;

    public Superstructure(
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
    }

    //Use this function to construct a generic command that transitions from one state to another (just elevator and wrist angle ATM)
    private Command transitionToState(SuperState desiredState){
        Command elevatorPremoveCommand;
        //This won't work if we don't have start possesion
        /*
        switch (desiredState.startPossession){
            case CORAL_ALGAE:
            case ALGAE:
                elevatorPremoveCommand = new SetWristAngleCommand(Units.degreesToRadians(65), m_wrist);
                break;
            default:
                elevatorPremoveCommand = new SetWristAngleCommand(Units.degreesToRadians(90), m_wrist);
                break;
        }*/
        return Commands.sequence(
            //First, fold the grabber up 90 degrees before moving the elevator
            elevatorPremoveCommand,
            //Next, set the elevator to the desired position
            new SetElevatorPositionCommand(desiredState.elevatorHeightMeters, m_elevator),
            //Finally, lower the grabber to the desired angle
            new SetWristAngleCommand(desiredState.wristAngle.getRadians(), m_wrist)
            
        );
    }

    /*
     * This method returns a command that has the grabber eject and stop
     */
    private Command shootAndStop(){
        //Add a check for if algae or coral
        return Commands.runEnd(
            () -> {m_grabber.runVolts(0);}, //placeholder
            () -> {m_grabber.runVolts(0);},
            m_grabber
        );
    }

}
