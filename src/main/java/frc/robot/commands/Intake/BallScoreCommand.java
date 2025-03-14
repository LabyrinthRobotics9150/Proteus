package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BallScoreCommand extends Command {
    IntakeSubsystem intake;
    ElevatorSubsystem elevator;
    Boolean processor;
    public BallScoreCommand(IntakeSubsystem intake, ElevatorSubsystem elevator, boolean processor) {
        this.intake = intake;
        this.elevator = elevator;
        this.processor = processor;
        addRequirements(intake);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if(!processor) {
            elevator.setHeight(3.75);
        }
    }

    @Override
    public void execute() {
        // raise to l4, and as soon as it gets there, release the ball
        new SequentialCommandGroup(
            new WaitCommand(1.2)
        );
        intake.intakeScoreBall(true, processor);
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel();
        elevator.setHeight(0);
    }

}
