package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetHeight;

    public MoveElevatorCommand(ElevatorSubsystem elevator, double height) {
        this.elevator = elevator;
        this.targetHeight = height;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("Moving elevator to height: " + targetHeight);
        elevator.setHeight(targetHeight);
    }

    @Override
    public void execute() {
        // Optional: Log current position and velocity for debugging
        System.out.println("Current Height: " + elevator.getHeight() + ", Velocity: " + elevator.getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator movement interrupted!");
            elevator.stopElevator(); // Safer than reverting position
        } else {
            System.out.println("Elevator reached target height: " + targetHeight);
        }
    }

    @Override
    public boolean isFinished() {
        // Looser tolerance + velocity check
        return Math.abs(elevator.getHeight() - targetHeight) < 0.5 
            && Math.abs(elevator.getVelocity()) < 0.1;
    }
}