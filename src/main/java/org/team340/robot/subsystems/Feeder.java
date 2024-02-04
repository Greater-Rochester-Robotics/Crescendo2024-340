package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.FeederConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Handoff back to intake

// TODO Docs
public class Feeder extends GRRSubsystem {

    private final CANSparkMax feedMotor;
    private final DigitalInput noteDetector;
    private final SparkPIDController feedPID;

    public Feeder() {
        super("Feeder");
        feedMotor = createSparkMax("Motor", RobotMap.SHOOTER_FEEDER_MOTOR, MotorType.kBrushless);
        noteDetector = createDigitalInput("Note Detector", RobotMap.SHOOTER_NOTE_DETECTOR);
        feedPID = feedMotor.getPIDController();

        FeederConstants.FEED_MOTOR_CONFIG.apply(feedMotor);
        FeederConstants.FEED_PID_CONFIG.apply(feedMotor, feedPID);
    }

    // TODO Shorten name?
    /**
     * Returns {@code true} when the note detector is detecting a note.
     */
    public boolean getNoteDetector() {
        return !noteDetector.get();
    }

    /**
     * Receives a note from the intake. Ends when the note is detected.
     */
    public Command receiveNote() {
        // TODO
        // The note may not end in a consistent position, as there is likely to be a short
        // delay between the sensor detecting the note and the motor stopping. After the note
        // is detected, the motors should back the note up until it is not detected anymore,
        // then slowly fed it until it is detected to ensure the note is consistently staged.
        // END TODO
        return commandBuilder("feeder.receiveNote()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.FEED_INTAKE_SPEED, ControlType.kDutyCycle))
            .isFinished(this::getNoteDetector)
            .onEnd(() -> feedMotor.stopMotor());
    }

    /**
     * Feeds the note into the shooters. Ends after the note is no longer detected.
     */
    public Command shootNote() {
        return commandBuilder("feeder.shootNote()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.FEED_SHOOT_SPEED, ControlType.kDutyCycle))
            .isFinished(() -> !getNoteDetector())
            .andThen(waitSeconds(FeederConstants.SHOOT_DELAY))
            .andThen(runOnce(() -> feedMotor.stopMotor()));
    }

    /**
     * Spits the note out of the feeder in case it is stuck.
     */
    public Command spit() {
        return commandBuilder("feeder.spit()")
            .onInitialize(() -> feedPID.setReference(FeederConstants.FEEDER_SPIT_SPEED, ControlType.kDutyCycle))
            .onEnd(() -> feedMotor.stopMotor());
    }
}
