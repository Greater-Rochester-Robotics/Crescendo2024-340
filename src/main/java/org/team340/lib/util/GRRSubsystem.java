package org.team340.lib.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class GRRSubsystem implements Subsystem {

    public GRRSubsystem() {
        register();
    }

    /**
     * Creates a command builder that requires this subsystem.
     */
    protected CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     * @param name The name of the command.
     */
    protected CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }

    /**
     * Returns {@code true} if there are no commands
     * running that require the subsystem.
     * @return
     */
    public boolean isIdle() {
        return getCurrentCommand() == null;
    }
}
