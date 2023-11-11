package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class GroundIntakeElevatorCommand extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;
    private BooleanSupplier isCube;

    public GroundIntakeElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier isCube) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.isCube = isCube;

        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void execute() {
        if(isCube.getAsBoolean()) {
            elevatorSubsystem.setTargetState(Constants.Elevator.Setpoints.GROUND_INTAKE_CUBE);
        } else {
            elevatorSubsystem.setTargetState(Constants.Elevator.Setpoints.GROUND_INTAKE_CONE);
        }
    }
}
