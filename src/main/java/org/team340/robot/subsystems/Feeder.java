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

// TODO Handoff back to intake

/**
 * This subsystem controls the feeder wheels, which accept the notes from the intake, and then push them into the shooter wheels to be shot.
 */
public class Feeder extends GRRSubsystem {

    private final CANSparkMax feedMotor;
    private final RelativeEncoder feedEncoder;
    private final DigitalInput noteDetector;
    private final SparkPIDController feedPID;

    /**
     * Creates a new feeder subsystem.
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

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("hasNote", this::hasNote, null);
    }

    /**
     * Returns {@code true} when the note detector is detecting a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Receives a note from the intake.
     */
    public Command receiveNote() {
        return commandBuilder("feeder.receiveNote()")
            .onInitialize(() -> feedMotor.set(FeederConstants.INTAKE_SPEED))
            .isFinished(() -> hasNote())
            .onEnd(() -> feedMotor.stopMotor())
            .onlyIf(() -> !hasNote())
            .withName("feeder.receiveNote()");
    }

    /**
     * This command moves it forward until it's detected, then backwards till it's not, then forward again,
     * to try to get it as consistent as possible.
     */
    public Command seatNote() {
        return sequence(
            commandBuilder("feeder.seatNote().slowIn")
                .onInitialize(() -> feedMotor.set(FeederConstants.IN_SLOW_SPEED))
                .isFinished(() -> hasNote())
                .onEnd(() -> feedEncoder.setPosition(0.0)),
            commandBuilder("feeder.seatNote().inToPos")
                .onInitialize(() -> feedPID.setReference(FeederConstants.POSITION_OFFSET, ControlType.kPosition))
                .isFinished(() ->
                    Math2.epsilonEquals(feedEncoder.getPosition(), FeederConstants.POSITION_OFFSET, FeederConstants.CLOSED_LOOP_ERR)
                )
                .onEnd(() -> feedMotor.stopMotor())
        )
            .withTimeout(2.0)
            .onlyIf(() -> !hasNote())
            .withName("feeder.seatNote()");
    }

    /**
     * Feeds the note into the shooters. Ends after the note is no longer detected.
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
     * Spits the note out of the feeder in case it is stuck.
     */
    public Command spitFront() {
        return commandBuilder("feeder.spitFront()")
            .onInitialize(() -> feedMotor.set(FeederConstants.SPIT_SPEED_FRONT))
            .onEnd(() -> feedMotor.stopMotor());
    }

    public Command spitBack() {
        return commandBuilder("feeder.spitBack()")
            .onInitialize(() -> feedMotor.set(FeederConstants.SPIT_SPEED_BACK))
            .onEnd(() -> feedMotor.stopMotor());
    }

    public Command intakeFromHuman() {
        return commandBuilder("feeder.intakeFromHuman()")
            .onInitialize(() -> feedMotor.set(FeederConstants.INTAKE_HUMAN_SPEED))
            .onEnd(() -> feedMotor.stopMotor());
    }

    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> feedMotor.setIdleMode(IdleMode.kCoast))
            .onEnd(() -> feedMotor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("feeder.onDisable()");
    }
}
