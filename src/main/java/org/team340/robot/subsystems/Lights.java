package org.team340.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.LightsConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the lights.
 */
public class Lights extends GRRSubsystem {

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;
    private final Timer timer;

    /**
     * Create the lights subsystem.
     */
    public Lights() {
        super("Lights");
        lights = new AddressableLED(RobotMap.LIGHTS);
        buffer = new AddressableLEDBuffer(LightsConstants.LENGTH);
        timer = new Timer();

        lights.setLength(buffer.getLength());
        lights.setData(buffer);
        lights.start();
        timer.start();
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
     * The default command for the lights.
     */
    public Command defaultCommand(Supplier<Boolean> hasNote) {
        return commandBuilder()
            .onExecute(() -> {
                if (DriverStation.isTeleopEnabled()) {
                    if (hasNote.get()) {
                        double time = timer.get();
                        for (int i = 0; i < buffer.getLength() / 2; i++) {
                            int v = (int) ((Math.cos((time * 20.0) - ((i / (buffer.getLength() / 2.0)) * Math2.TWO_PI)) + 1.0) * 100.0);
                            buffer.setRGB(i, v, v, v);
                            buffer.setRGB(buffer.getLength() - i - 1, v, v, v);
                        }
                        lights.setData(buffer);
                    } else {
                        int v = (int) (((Math.cos(timer.get() * 8.5) + 1.0) * 87.5) + 25.0);
                        if (Alliance.isBlue()) {
                            setRGB(0, 0, v);
                        } else {
                            setRGB(v, 0, 0);
                        }
                    }
                } else if (DriverStation.isAutonomousEnabled()) {
                    double time = timer.get();
                    for (int i = 0; i < buffer.getLength() / 2; i++) {
                        int v = (int) ((Math.cos((time * 30.0) - ((i / (buffer.getLength() / 2.0)) * Math2.TWO_PI)) + 1.0) * 100.0);
                        if (Alliance.isBlue()) {
                            buffer.setRGB(i, 0, 0, v);
                            buffer.setRGB(buffer.getLength() - i - 1, 0, 0, v);
                        } else {
                            buffer.setRGB(i, v, 0, 0);
                            buffer.setRGB(buffer.getLength() - i - 1, v, 0, 0);
                        }
                    }
                    lights.setData(buffer);
                } else if (DriverStation.isDisabled()) {
                    if (Alliance.isBlue()) setRGB(0, 0, 150); else setRGB(150, 0, 0);
                } else {
                    setRGB(0, 0, 0);
                }
            })
            .onEnd(() -> setRGB(0, 0, 0))
            .ignoringDisable(true)
            .withName("lights.defaultCommand()");
    }
}
