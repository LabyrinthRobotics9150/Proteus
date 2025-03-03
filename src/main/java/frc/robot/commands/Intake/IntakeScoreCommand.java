package frc.robot.commands.Intake;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Timer;
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
    private final double reverseTime;
    private final double scoringSpeed;
    private final double detectionThresholdMm;

    private enum State {
        INIT, DETECTED_OBJECT, REVERSING
    }

    private State currentState;
    private final Timer timer = new Timer();
    private boolean isScoringMode;

    public IntakeScoreCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.heightThreshold = -0.3;
        this.normalSpeed = 0.5;  // Increased for intake
        this.slowSpeed = 0.2;
        this.reverseSpeed = 0.3; 
        this.reverseTime = 0.3;
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
        timer.reset();
        timer.stop();
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

        System.out.println(isScoringMode);

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
                        timer.reset();
                        timer.start();
                        currentState = State.REVERSING;
                    }
                    break;

                case REVERSING:
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
            return currentState == State.REVERSING && timer.hasElapsed(reverseTime);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopWheel();
    }
}