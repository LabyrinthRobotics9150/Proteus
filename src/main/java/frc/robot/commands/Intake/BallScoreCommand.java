package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BallScoreCommand extends Command {
    IntakeSubsystem intake;
    ElevatorSubsystem elevator;
    public BallScoreCommand(IntakeSubsystem intake, ElevatorSubsystem elevator) {
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(intake);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // raise to l4, and as soon as it gets there, release the ball
        elevator.setHeight(3.9);
        new WaitCommand(.5);
        intake.intakeScoreBall(true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopWheel();
        elevator.setHeight(0);
    }

}
