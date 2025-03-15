package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.ElevatorRaise;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BallScoreCommand extends Command {
    IntakeSubsystem intake;
    ElevatorSubsystem elevator;
    Boolean processor;
    int counter;
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
            new ElevatorRaise(elevator, 3.9);
        }
    }

    @Override
    public void execute() {
        if (counter > 60) {
            intake.intakeScoreBall(true, processor);
        }
        counter++;

        
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel();
        elevator.setHeight(0);
    }

}
