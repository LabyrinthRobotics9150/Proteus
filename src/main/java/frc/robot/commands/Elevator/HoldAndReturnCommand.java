package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldAndReturnCommand extends Command {
    private final ElevatorSubsystem elevator;
    private double targetHeight;

    public HoldAndReturnCommand(ElevatorSubsystem elevator, double height) {
        this.elevator = elevator;
        this.targetHeight = height;
    }

    @Override
    public void initialize() {
        if (targetHeight > .5) {
            RobotContainer.m_slowMode = true;
        }
        elevator.setHeight(targetHeight);
        System.out.println("Moving elevator to: " + targetHeight);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            if (targetHeight > .5) {
                RobotContainer.m_slowMode = false;
            }
            elevator.setHeight(0); // Only return to zero if button released
        }
    }
}