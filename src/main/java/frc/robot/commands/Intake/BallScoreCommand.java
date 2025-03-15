package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.autos.ElevatorRaise;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class BallScoreCommand extends Command {
    IntakeSubsystem intake;
    ElevatorSubsystem elevator;
    Boolean processor;
    ElevatorRaise elevatorRaise;
    int counter;
    double elevatorHeight;
    public BallScoreCommand(IntakeSubsystem intake, ElevatorSubsystem elevator, boolean processor) {
        this.intake = intake;
        this.elevator = elevator;
        this.processor = processor;
        addRequirements(intake);
        addRequirements(elevator);
        elevatorHeight = 3.9;
        if (processor) {
            elevatorHeight = 0;
        }
    }

    @Override
    public void initialize() {
        elevator.setHeight(2);
        RobotContainer.m_slowMode = true;
        counter = 0;
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
        intake.setHeight(intake.HOME_POSITION);
        elevator.setHeight(0);
    }

}
