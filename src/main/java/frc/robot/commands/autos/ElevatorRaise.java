package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorRaise extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetHeight;

    public ElevatorRaise(ElevatorSubsystem elevator, double height) {
        this.elevator = elevator;
        this.targetHeight = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (targetHeight > 0.5) {
            RobotContainer.m_slowMode = true;
        }
        elevator.setHeight(targetHeight);
        System.out.println("Moving elevator to: " + targetHeight);
    }

    @Override
    public boolean isFinished() {
        // Continue running to maintain position
        return false; 
    }

    @Override
    public void execute() {
        // Continuous position maintenance
        elevator.setHeight(targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.setHeight(0);
            if (targetHeight > 0.5) {
                RobotContainer.m_slowMode = false;
            }
        }
    }
}