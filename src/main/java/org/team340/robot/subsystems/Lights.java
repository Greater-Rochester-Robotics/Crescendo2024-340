package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.idle;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.LightsConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the lights.
 */
public class Lights extends GRRSubsystem {

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    /**
     * Create the lights subsystem.
     */
    public Lights() {
        super("Lights");
        lights = new AddressableLED(RobotMap.LIGHTS);
        buffer = new AddressableLEDBuffer(LightsConstants.LENGTH);

        lights.setLength(buffer.getLength());
        lights.setData(buffer);
        lights.start();
    }

    /**
     * Sets the entire LED strip to a solid color.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void setRGB(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        lights.setData(buffer);
    }

    /**
     * Displays the "Has Note" signal.
     */
    public Command hasNote() {
        return sequence(
            runOnce(() -> setRGB(0, 255, 0)),
            waitSeconds(0.4),
            runOnce(() -> setRGB(0, 0, 0)),
            waitSeconds(0.4),
            runOnce(() -> setRGB(0, 255, 0)),
            waitSeconds(0.4),
            runOnce(() -> setRGB(0, 0, 0)),
            waitSeconds(0.4),
            runOnce(() -> setRGB(0, 255, 0)),
            idle(this)
        )
            .finallyDo(() -> setRGB(0, 0, 0))
            .withName("lights.hasNote()");
    }

    public Command onDisable() {
        return commandBuilder()
            .onExecute(() -> setRGB(255, 0, 0))
            .onEnd(() -> setRGB(0, 0, 0))
            .ignoringDisable(true)
            .withName("lights.onDisable()");
    }
}
