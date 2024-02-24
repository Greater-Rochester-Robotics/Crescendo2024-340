package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.FeederConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * This subsystem controls the feeder wheels, which accepts a note from the
 * intake and pushes it to be shot or back into the intake for amp scoring.
 */
public class Feeder extends GRRSubsystem {

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final DigitalInput noteDetector;
    private final SparkPIDController feedPID;

    /**
     * Create the feeder subsystem.
     */
    public Feeder() {
        super("Feeder");
        feedMotor = createSparkMax("Motor", RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        feedEncoder = feedMotor.getEncoder();
        noteDetector = createDigitalInput("Note Detector", RobotMap.SHOOTER_NOTE_DETECTOR);
        feedPID = feedMotor.getPIDController();

        FeederConstants.Configs.MOTOR.apply(feedMotor);
        FeederConstants.Configs.PID.apply(feedMotor, feedPID);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("hasNote", this::hasNote, null);
    }

    /**
     * Returns {@code true} when the beam break detects a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Receives a note from the intake.
     */
    public Command receiveNote() {
        return commandBuilder()
            .onInitialize(() -> feedMotor.set(FeederConstants.INTAKE_SPEED))
            .isFinished(() -> hasNote())
            .onEnd(() -> feedMotor.stopMotor())
            .onlyIf(() -> !hasNote())
            .withName("feeder.receiveNote()");
    }

    /**
     * Sets the feeder to receive a note through the shooter from the human player.
     */
    public Command intakeHuman() {
        return commandBuilder("feeder.intakeHuman()")
            .onInitialize(() -> feedMotor.set(FeederConstants.INTAKE_HUMAN_SPEED))
            .onEnd(() -> feedMotor.stopMotor());
    }

    /**
     * Seats the note in the shooter to a set position.
     */
    public Command seatNote() {
        return sequence(
            commandBuilder()
                .onInitialize(() -> feedMotor.set(FeederConstants.IN_SLOW_SPEED))
                .isFinished(() -> hasNote())
                .onEnd(() -> feedEncoder.setPosition(0.0)),
            commandBuilder()
                .onInitialize(() -> feedPID.setReference(FeederConstants.POSITION_OFFSET, ControlType.kPosition))
                .isFinished(() ->
                    Math2.epsilonEquals(feedEncoder.getPosition(), FeederConstants.POSITION_OFFSET, FeederConstants.CLOSED_LOOP_ERR)
                )
                .onEnd(() -> feedMotor.stopMotor())
        )
            .onlyIf(() -> !hasNote())
            .withTimeout(2.0)
            .withName("feeder.seatNote()");
    }

    /**
     * Feeds the note into the shooter wheels. Ends after the note has left the shooter.
     */
    public Command shootNote() {
        return commandBuilder()
            .onInitialize(() -> feedMotor.set(FeederConstants.SHOOT_SPEED))
            .isFinished(() -> !hasNote())
            .andThen(waitSeconds(FeederConstants.SHOOT_DELAY))
            .finallyDo(() -> feedMotor.stopMotor())
            .withName("feeder.shootNote()");
    }

    /**
     * Spits the note out of the feeder towards the intake.
     */
    public Command barfForward() {
        return commandBuilder("feeder.barfForward()")
            .onInitialize(() -> feedMotor.set(FeederConstants.BARF_FORWARD_SPEED))
            .onEnd(() -> feedMotor.stopMotor());
    }

    /**
     * Spits the note out of the feeder towards the shooter.
     */
    public Command barfBackward() {
        return commandBuilder("feeder.barfBackward()")
            .onInitialize(() -> feedMotor.set(FeederConstants.BARF_BACKWARD_SPEED))
            .onEnd(() -> feedMotor.stopMotor());
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                feedMotor.setIdleMode(IdleMode.kCoast);
                feedMotor.stopMotor();
            })
            .onEnd(() -> feedMotor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("feeder.onDisable()");
    }
}
