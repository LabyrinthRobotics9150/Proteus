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
    private final double reverseRotations;
    public static double scoringSpeed;
    private final double detectionThresholdMm;

    private enum State {
        INIT, DETECTED_OBJECT, REVERSING
    }

    private State currentState;
    private boolean isScoringMode;
    private double initialEncoderPosition;
    private double elevatorHeight;

    public IntakeScoreCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.heightThreshold = 0.10;
        this.normalSpeed = 0.1;
        this.slowSpeed = 0.03;
        this.reverseSpeed = 0.03; 
        this.reverseRotations = 1.0;
        this.scoringSpeed = 0.3;
        this.detectionThresholdMm = 20;
        elevatorHeight = elevatorSubsystem.getHeight();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        isScoringMode = elevatorHeight > heightThreshold;
        initialEncoderPosition = intakeSubsystem.getEncoderPosition();
        if (!isScoringMode) {
            currentState = State.INIT;
        }
    }

    @Override
    public void execute() {
        // Continuously check elevator height
        elevatorHeight = elevatorSubsystem.getHeight();
        isScoringMode = (elevatorHeight > heightThreshold);

        if (!isScoringMode) {
            switch (currentState) {
                case INIT:
                    LaserCan.Measurement meas = intakeSubsystem.laserCan.getMeasurement();
                    if (isObjectDetected(meas)) {
                        intakeSubsystem.moveWheel(slowSpeed); 
                        currentState = State.DETECTED_OBJECT;
                    } else {
                        intakeSubsystem.moveWheel(normalSpeed);
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
        } else {
            intakeSubsystem.moveWheel(scoringSpeed);
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
            return false;
        } else {
            return currentState == State.REVERSING && intakeSubsystem.getEncoderPosition() - initialEncoderPosition >= reverseRotations;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopWheel();
    }
}