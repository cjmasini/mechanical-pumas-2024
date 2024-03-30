package frc.robot.commands.swervedrive.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class IntakeAndMoveCommand extends Command {

    private final LauncherSubsystem launcherSubsystem;
    private final DriveSubsystem drivetrain;

    public IntakeAndMoveCommand(LauncherSubsystem launcherSubsystem, DriveSubsystem driveSubsystem) {
        this.drivetrain = driveSubsystem;
        this.launcherSubsystem = launcherSubsystem;
    }

    @Override
    public void initialize() {
        this.launcherSubsystem.setIntakeSpeed(.5);
        this.launcherSubsystem.setLoaderSpeed(.5);
    } 

    @Override
    public void execute() {
        this.drivetrain.drive(.5, 0, 0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.drive(0, 0, 0, true, true);
        this.launcherSubsystem.setIntakeSpeed(0);
        this.launcherSubsystem.setLoaderSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
