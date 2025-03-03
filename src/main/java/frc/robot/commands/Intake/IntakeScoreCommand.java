package frc.robot.commands.Intake;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final double heightThreshold;
    private final double normalSpeed;
    private final double slowSpeed;
    private final double reverseSpeed;
    private final double reverseRotations; // Rotations to reverse
    private final double scoringSpeed;
    private final double detectionThresholdMm;

    private enum State {
        INIT, DETECTED_OBJECT, REVERSING
    }

    private State currentState;
    private boolean isScoringMode;
    private double initialEncoderPosition; // Track encoder position for rotation counting

    public IntakeScoreCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.heightThreshold = 0.4;
        this.normalSpeed = 0.07;  // Increased for intake
        this.slowSpeed = 0.05;
        this.reverseSpeed = 0.05; 
        this.reverseRotations = 2.5; // Reverse for 0.5 rotations
        this.scoringSpeed = -0.5; // Negative for reverse scoring
        this.detectionThresholdMm = 30;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        double elevatorHeight = elevatorSubsystem.getHeight();
        isScoringMode = (elevatorHeight >= heightThreshold);
        
        if (isScoringMode) {
            intakeSubsystem.moveWheel(scoringSpeed); // Reverse for scoring
        } else {
            currentState = State.INIT;
            intakeSubsystem.moveWheel(normalSpeed); // Forward for intake
        }
        initialEncoderPosition = intakeSubsystem.getEncoderPosition(); // Initialize encoder position
    }

    @Override
    public void execute() {
        // Continuously check elevator height
        double elevatorHeight = elevatorSubsystem.getHeight();
        boolean nowScoringMode = (elevatorHeight >= heightThreshold);

        if (nowScoringMode != isScoringMode) {
            // Mode changed! Reset state
            isScoringMode = nowScoringMode;
            if (isScoringMode) {
                intakeSubsystem.moveWheel(scoringSpeed);
            } else {
                currentState = State.INIT;
                intakeSubsystem.moveWheel(normalSpeed);
            }
        }

        if (!isScoringMode) {
            switch (currentState) {
                case INIT:
                    LaserCan.Measurement meas = intakeSubsystem.laserCan.getMeasurement();
                    if (isObjectDetected(meas)) {
                        intakeSubsystem.moveWheel(slowSpeed); // Slow down
                        currentState = State.DETECTED_OBJECT;
                    }
                    break;

                case DETECTED_OBJECT:
                    // Keep running slow speed while object is detected
                    intakeSubsystem.moveWheel(slowSpeed); 

                    LaserCan.Measurement newMeas = intakeSubsystem.laserCan.getMeasurement();
                    if (!isObjectDetected(newMeas)) {
                        intakeSubsystem.moveWheel(-reverseSpeed); // Reverse direction
                        initialEncoderPosition = intakeSubsystem.getEncoderPosition(); // Reset encoder position
                        currentState = State.REVERSING;
                    }
                    break;

                case REVERSING:
                    // Check if the required number of rotations has been achieved
                    double currentEncoderPosition = intakeSubsystem.getEncoderPosition();
                    double rotations = Math.abs(currentEncoderPosition - initialEncoderPosition);
                    if (rotations >= reverseRotations) {
                        intakeSubsystem.stopWheel(); // Stop after reversing the required rotations
                    }
                    break;
            }
        }
    }

    private boolean isObjectDetected(LaserCan.Measurement meas) {
        return meas != null 
            && meas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
            && meas.distance_mm < detectionThresholdMm;
    }

    @Override
    public boolean isFinished() {
        if (isScoringMode) {
            return false; // Run until interrupted
        } else {
            return currentState == State.REVERSING && intakeSubsystem.getEncoderPosition() - initialEncoderPosition >= reverseRotations;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopWheel();
    }
}