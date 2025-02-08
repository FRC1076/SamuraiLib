// DO NOT DELETE COMMENTS
// THEY ARE FOR EDUCATIONAL PURPOSES
package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDStates;

/** This is kind of like a subsystem.
 * <p>
 * The implementation to be used will be selected in the constructor upon instantiation.
 * <p>
 * All of the methods in this file will call the corresponding method in the chosen IO layer.
 */
public class LED {
    private final LEDIO io;

    /** Create the LEDs with one of the IO layers.
     * 
     * @param io The chosen IO layer.
     */
    public LED(LEDIO io) {
        this.io = io;
    }

    /** Set the state of the LEDs through the chosen IO layer.
     * 
     * @param state The chosen state in the enum LEDStates.
     */
    public void setState(LEDStates state) {
        this.io.setState(state);
    }
}
