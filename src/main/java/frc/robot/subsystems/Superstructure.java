// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import frc.robot.Constants.SuperstructureConstants.WristevatorPreset;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wristevator.Wristevator;
import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;
import frc.robot.utils.Elastic;

import static frc.robot.Constants.IndexConstants.kIndexVoltage;

import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.SelectWithFallbackCommand;

import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure class that contains all subsystems and commands for the robot's superstructure <p>
 * Allows all of the subsystems to talk to each other <p>
 * Allows sensors to interact with subystems <p>
 * Contains command factories for actions requiring multiple systems <p>
 */
public class Superstructure {

    /** A mutable (you can change the state after instantiation) class representing the Superstructure's desired state */
    @AutoLog
    public static class MutableSuperState {

        protected GrabberState grabberState;
        protected WristevatorPreset WristevatorPreset;
        protected IndexState indexState;
        protected GrabberPossession grabberPossession;
        protected IndexPossession indexPossession;

        public MutableSuperState(GrabberState grabberState, WristevatorPreset WristevatorPreset, IndexState indexState){
            this.grabberState = grabberState;
            this.WristevatorPreset = WristevatorPreset;
            this.indexState = indexState;
            grabberPossession = GrabberPossession.EMPTY;
            indexPossession = IndexPossession.EMPTY;
        }

        public MutableSuperState() {}
        
        /** Set state of the wristevator <p>
         * States include elevator heights and wrist angles corresponding to specific actions
         */
        public void setWristevatorPreset(WristevatorPreset position) {
            this.WristevatorPreset = position;
        }

        /** Set state of the grabber. <p>
         * States include voltage to run grabber wheels at for different actions (intake, outtake, idle)
         */
        public void setGrabberState(GrabberState state) {
            this.grabberState = state;
        }

        public void setIndexerState(IndexState state) {
            this.indexState = state;
        }

        public void setGrabberPossession(GrabberPossession possession) {
            this.grabberPossession = possession;
        }

        public void setIndexPossession(IndexPossession possession) {
            this.indexPossession = possession;
        }

        public GrabberState getGrabberState() {
            return grabberState;
        }

        public WristevatorPreset getWristevatorPreset() {
            return WristevatorPreset;
        }

        public IndexState getIndexerState() {
            return indexState;
        }

        public GrabberPossession getGrabberPossession(){
            return grabberPossession;
        }

        public IndexPossession getIndexerPossession(){
            return indexPossession;
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private final Wristevator wristevator;

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final MutableSuperStateAutoLogged superState = new MutableSuperStateAutoLogged();

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        BooleanSupplier indexBeamBreak, //returns true when beam broken
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier grabberBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        wristevator = new Wristevator(elevator, wrist);
        
        CommandUtils.makePeriodic(() -> {
            Logger.processInputs("Superstructure", superState);
        });
        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, grabberBeamBreak);
    }

    public MutableSuperState getSuperState() {
        return this.superState;
    }

    public ElevatorSubsystem getElevator() {
        return m_elevator;
    }

    public GrabberSubsystem getGrabber() {
        return m_grabber;
    }

    public WristSubsystem getWrist() {
        return m_wrist;
    }
    
    public IndexSubsystem getIndex() {
        return m_index;
    }

    public SuperstructureCommandFactory getCommandBuilder(){
        return CommandBuilder;
    }

    // Command factories that apply states are private because they are only accessed by the main SuperStructureCommandFactory

    /**
     * Folds back wrist, moves elevator, then deploys wrist
     * @param position the WristevatorPreset, which consists of elevator height and wrist angle, to transition to
     * @return generic transition command from one state to another 
     */
    private Command applyWristevatorPreset(WristevatorPreset position) {
        
        return Commands.sequence(
            Commands.runOnce(() -> superState.setWristevatorPreset(position)),
            wristevator.CommandBuilder.applyPresetSequential(position,() -> superState.getGrabberPossession() == GrabberPossession.ALGAE ? Rotation2d.fromDegrees(65) : Rotation2d.kCCW_90deg)
        );
    }

    /**
     * Sets grabber wheels to run at desired state
     * @param state the GrabberState
     * @return command to run grabber as certain state
     */
    private Command applyGrabberState(GrabberState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    /**
     * Sets indexer to run at desired state
     * @param state the IndexState
     * @return command to run index as certain state
     */
    private Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.applyVolts(kIndexVoltage).onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    /**
     * Updates game piece possession based on beambreaks and updates kG accordingly
     * (gamepieces have weight that affects elevator and wrist)
     * @param indexBB whether the beambreak sensor in the indexer detects something
     * @param transferBB whether the beambreak sensor between the indexer and grabber detects something
     * @param greabberBB whether the beambreak sensor in the grabber detects something
     */
    public void updatePossessionAndKg(
        boolean indexBB,
        boolean transferBB,
        boolean grabberBB
    ) {
        IndexPossession indexPossession = indexBB 
            ? IndexPossession.CORAL 
            : IndexPossession.EMPTY;
        GrabberPossession grabberPossession = grabberBB
            ? (transferBB
                ? GrabberPossession.CORAL 
                : GrabberPossession.ALGAE) 
            : GrabberPossession.EMPTY;
        m_elevator.setKg(grabberPossession.elevator_kG);
        m_wrist.setKg(grabberPossession.wrist_kG);
        superState.setIndexPossession(indexPossession);
        superState.setGrabberPossession(grabberPossession);
        Elastic.putIndexPossession(indexPossession);
        Elastic.putGrabberPossession(grabberPossession);
    }

    /** Contains all the command factories for the superstructure */
    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_grabberBeamBreak;
        private final Map<WristevatorPreset, Command> grabberActionCommands = new HashMap<>(); // We use a map of grabber action commands so that we can use the SelectWithFallBackCommand factory
        private final Command grabberActionSelectCommand;
        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier indexBeamBreak,
            BooleanSupplier transferBeamBreak,
            BooleanSupplier grabberBeamBreak
        ) {
            this.superstructure = superstructure;
            m_indexBeamBreak = indexBeamBreak;
            m_transferBeamBreak = transferBeamBreak;
            m_grabberBeamBreak = grabberBeamBreak;
            grabberActionCommands.put(WristevatorPreset.L1, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorPreset.L2, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorPreset.L3, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); 
            grabberActionCommands.put(WristevatorPreset.L4, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); //TODO: Are there any risks if we have a coral, try to score it, and then hit the button again? The grabber will think it has an algae
            grabberActionCommands.put(WristevatorPreset.GROUND_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE));
            grabberActionCommands.put(WristevatorPreset.PROCESSOR, superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE));
            grabberActionCommands.put(WristevatorPreset.LOW_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE));
            grabberActionCommands.put(WristevatorPreset.HIGH_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE));
            grabberActionCommands.put(WristevatorPreset.NET, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            grabberActionSelectCommand = new SelectWithFallbackCommand<WristevatorPreset>(
                grabberActionCommands,
                superstructure.applyGrabberState(GrabberState.DEFAULT_OUTTAKE), // Default command to do if command can't be chosen from grabberActionCommands
                this.superstructure.getSuperState()::getWristevatorPreset
            );
        }

        public Command followWristevatorTrajectory(List<WristevatorState> trajectory) {
            return superstructure.wristevator.CommandBuilder.followTrajectory(trajectory);
        }

        /**
         * Returns a command that does a grabber action differently depending on robot state (scoring coral, scoring algae, intaking algae)
         */
        public Command doGrabberAction() {
            return grabberActionSelectCommand;
        }

        /**
         * Returns a command that stops grabber wheels from spinning
         */
        public Command stopGrabber(){
            return superstructure.applyGrabberState(GrabberState.IDLE);
        }

        /**
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.TRAVEL);
        }

        /**
         * Call this on button releases after scoring game pieces, stops grabber movement and retracts mechanisms to travel state
         */
        public Command stopAndRetract(){
            return Commands.parallel(
                stopGrabber(),
                retractMechanisms()
            );
        }

        /**
         * Set elevator and wrist to L1 preset
         */
        public Command preL1(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.L1);
        }

        /**
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.L2);
        }

        /**
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.L3);
        }

        /**
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.L4);
        }

        /**
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.NET);
        }

        /**
         * Set elevator and wrist to processor preset
         */
        public Command preProcessor(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.PROCESSOR);
        }

        /**
         * Set elevator and wrist to ground algae preset
         */
        public Command groundAlgaeIntake(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.GROUND_INTAKE);
        }

        /**
         * Set elevator and wrist to low algae preset
         */
        public Command lowAlgaeIntake(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.LOW_INTAKE);  
        }

        /**
         * Set elevator and wrist to high algae preset
         */
        public Command highAlgaeIntake(){
            return superstructure.applyWristevatorPreset(WristevatorPreset.HIGH_INTAKE);
            
        }

        /** 
         * Transfers a coral from the funnel to the indexer 
         */
        private Command indexCoral() {
            return Commands.sequence(
                superstructure.applyIndexState(IndexState.CORAL_INTAKE),
                Commands.waitUntil(m_indexBeamBreak),
                superstructure.applyIndexState(IndexState.CORAL_IDLE)
            );
        }

        /** 
         * Transfers a coral from the indexer to the grabber, without checking for position 
         */
        private Command transferCoral() {
            return Commands.parallel(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    Commands.waitUntil(m_grabberBeamBreak),
                    superstructure.applyGrabberState(GrabberState.IDLE)
                ),
                Commands.sequence(
                    superstructure.applyIndexState(IndexState.CORAL_TRANSFER),
                    Commands.waitUntil(() -> !m_indexBeamBreak.getAsBoolean()),
                    superstructure.applyIndexState(IndexState.EMPTY_IDLE)
                )
            ).onlyIf(() -> superstructure.getSuperState().getGrabberPossession() == GrabberPossession.EMPTY);
        }

        /**
         * Bring the wristevator down to its travel state (wrist up, elevator all the way down),
         * brings the coral from the funnel into the indexer,
         * waits for a signal that it is safe to transfer the coral to the grabber,
         * rotates the wrist down,
         * and the rotates the motors to bring the coral into the grabber.
         * @param safeSignal a supplier indicating whether or not is safe to transfer the coral from the indexer to the grabber
         * @return a command sequence
         */
        public Command intakeCoral(BooleanSupplier safeSignal){
            return Commands.sequence(
                superstructure.applyWristevatorPreset(WristevatorPreset.TRAVEL),
                indexCoral(),
                Commands.waitUntil(safeSignal),
                superstructure.applyWristevatorPreset(WristevatorPreset.CORAL_TRANSFER),
                transferCoral()
            );
        }

        //TODO: Write separate intake coral command for auton

        /**
         * Used to calculate what the robot is possessing based on breambreaks
         */
        public Command updatePossessionAndKg(){
            return Commands.runOnce(
                () -> superstructure.updatePossessionAndKg(
                    m_indexBeamBreak.getAsBoolean(),
                    m_transferBeamBreak.getAsBoolean(),
                    m_grabberBeamBreak.getAsBoolean()
                )
            );
        }
    }
}
