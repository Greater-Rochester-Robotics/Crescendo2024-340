package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

    /**
     * Returns {@code true} when the note detector is detecting a note.
     */
    public boolean getNoteDetector() {
        return !noteDetector.get();
    }

    /**
     * Receives a note from the intake.
     */
    public Command receiveNote() {
        return commandBuilder("feeder.receiveNote()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.INTAKE_SPEED, ControlType.kDutyCycle))
            .isFinished(() -> getNoteDetector())
            .onEnd(() -> feedMotor.stopMotor());
    }

    /**
     * This command moves it forward until it's detected, then backwards till it's not, then forward again,
     * to try to get it as consistent as possible.
     */
    public Command reseatNote() {
        return either(
            sequence(
                commandBuilder("feeder.reseatNote().backUp")
                    .onInitialize(() -> feedPID.setReference(FeederConstants.BACK_SLOW_SPEED, ControlType.kDutyCycle))
                    .isFinished(() -> !getNoteDetector())
                    .onEnd(() -> feedEncoder.setPosition(0.0)),
                commandBuilder("feeder.reseatNote().inToPos")
                    .onInitialize(() -> feedPID.setReference(FeederConstants.POSITION_OFFSET, ControlType.kPosition))
                    .isFinished(() ->
                        Math2.epsilonEquals(feedEncoder.getPosition(), FeederConstants.POSITION_OFFSET, FeederConstants.CLOSED_LOOP_ERR)
                    )
                    .onEnd(() -> feedMotor.stopMotor())
            ),
            none(),
            this::getNoteDetector
        );
    }

    /**
     * Feeds the note into the shooters. Ends after the note is no longer detected.
     */
    public Command shootNote() {
        return commandBuilder("feeder.shootNote()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.SHOOT_SPEED, ControlType.kDutyCycle))
            .isFinished(() -> !getNoteDetector())
            .andThen(waitSeconds(FeederConstants.SHOOT_DELAY))
            .andThen(runOnce(() -> feedMotor.stopMotor()));
    }

    /**
     * Spits the note out of the feeder in case it is stuck.
     */
    public Command spit() {
        return commandBuilder("feeder.spit()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.SPIT_SPEED, ControlType.kDutyCycle))
            .onEnd(() -> feedMotor.stopMotor());
    }
}
