package frc.robot.subsystems;

import frc.robot.Constants.ElevatorSimConstants;
import frc.robot.Constants.SuperstructureConstants.CoralScore;
import frc.robot.Constants.SuperstructureConstants.EffectorPosition;
import frc.robot.Constants.SuperstructureConstants.EffectorPossession;
import frc.robot.Constants.SuperstructureConstants.EffectorState;
import frc.robot.Constants.SuperstructureConstants.GamePieceState;
import frc.robot.Constants.SuperstructureConstants.IndexerState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.wrist.SetWristAngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.IndexConstants.kIndexVoltage;

public class Superstructure {

    // A mutable class representing the Superstructure's state
    public static class SuperState {

        private EffectorState effectorState;
        private EffectorPosition effectorPosition;
        private IndexerState indexerState;
        private GamePieceState possession;

        public SuperState(EffectorState effectorState, EffectorPosition effectorPosition, IndexerState indexerState){
            this.effectorState = effectorState;
            this.effectorPosition = effectorPosition;
            this.indexerState = indexerState;
            calculatePossession();
        }

        public SuperState() {}

        //Calculates game piece possession from substates
        private void calculatePossession(){
            switch (effectorState.possession) {
                case EMPTY -> {
                    this.possession = indexerState.possession 
                    ? GamePieceState.CORAL_INDEXER 
                    : GamePieceState.EMPTY;
                }
                case CORAL -> {
                    this.possession = GamePieceState.CORAL_GRABBER;
                }
                case ALGAE -> {
                    this.possession = indexerState.possession 
                    ? GamePieceState.CORAL_ALGAE 
                    : GamePieceState.ALGAE;
                }
            }
        }

        public void setEffectorPosition(EffectorPosition position) {
            this.effectorPosition = position;
        }

        public void setEffectorState(EffectorState state) {
            this.effectorState = state;
            calculatePossession();
        }

        public void setIndexerState(IndexerState state) {
            this.indexerState = state;
            calculatePossession();
        }

        public EffectorState getEffectorState() {
            return effectorState;
        }

        public EffectorPosition getEffectorPosition() {
            return effectorPosition;
        }

        public IndexerState getIndexerState() {
            return indexerState;
        }

        public GamePieceState getPossession(){
            return this.possession;
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final SuperState superState = new SuperState();

    public Superstructure(
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        BooleanSupplier indexBeamBreak, //returns true when beam broken
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier effectorBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;

        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, effectorBeamBreak);
    }

    public SuperState getSuperState() {
        return this.superState;
    }

    public Command applyEffectorPosition(EffectorPosition position) {

        Command wristPreMoveCommand;

        switch (superState.getEffectorState().possession) {
            case EMPTY:
            case CORAL:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.kCW_90deg, m_wrist);
                break;
            case ALGAE:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.fromDegrees(65), m_wrist);
                break;
            default:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.kCW_90deg, m_wrist);
                break;
        }

        return Commands.sequence (
            Commands.runOnce(() -> superState.setEffectorPosition(position)),
            wristPreMoveCommand,
            new SetElevatorPositionCommand(position.elevatorHeightMeters, m_elevator),
            new SetWristAngleCommand(position.wristAngle, m_wrist)
        );
    }

    public Command applyEffectorState(EffectorState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setEffectorState(state)),
            m_grabber.run(() -> m_grabber.runVoltsDifferential(state.leftVoltage, state.rightVoltage))
        );
    }

    public Command applyIndexerState(IndexerState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.run(() -> m_index.runVolts(kIndexVoltage))
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

    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_effectorBeamBreak;
        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier indexBeamBreak,
            BooleanSupplier transferBeamBreak,
            BooleanSupplier effectorBeamBreak
        ) {
            this.superstructure = superstructure;
            m_indexBeamBreak = indexBeamBreak;
            m_transferBeamBreak = transferBeamBreak;
            m_effectorBeamBreak = effectorBeamBreak;
        }
        public Command scoreCoral(CoralScore scoreLevel) {
            switch (scoreLevel) {
                case L1:
                    return Commands.sequence(
                        superstructure.applyEffectorPosition(EffectorPosition.L1),
                        superstructure.applyEffectorState(EffectorState.CORAL_OUTTAKE).onlyWhile(m_effectorBeamBreak),
                        superstructure.applyEffectorState(EffectorState.EMPTY_IDLE),
                        superstructure.applyEffectorPosition(EffectorPosition.TRAVEL)
                    );
                case L2:
                    return Commands.sequence(
                        superstructure.applyEffectorPosition(EffectorPosition.L2),
                        superstructure.applyEffectorState(EffectorState.CORAL_OUTTAKE).onlyWhile(m_effectorBeamBreak),
                        superstructure.applyEffectorState(EffectorState.EMPTY_IDLE),
                        superstructure.applyEffectorPosition(EffectorPosition.TRAVEL)
                    );
                case L3:
                    return Commands.sequence(
                        superstructure.applyEffectorPosition(EffectorPosition.L3),
                        superstructure.applyEffectorState(EffectorState.CORAL_OUTTAKE).onlyWhile(m_effectorBeamBreak),
                        superstructure.applyEffectorState(EffectorState.EMPTY_IDLE),
                        superstructure.applyEffectorPosition(EffectorPosition.TRAVEL)
                    );
                case L4:
                    return Commands.sequence(
                        superstructure.applyEffectorPosition(EffectorPosition.L4),
                        superstructure.applyEffectorState(EffectorState.CORAL_OUTTAKE).onlyWhile(m_effectorBeamBreak),
                        superstructure.applyEffectorState(EffectorState.EMPTY_IDLE),
                        superstructure.applyEffectorPosition(EffectorPosition.TRAVEL)
                    );
            }
        }
    }

}
