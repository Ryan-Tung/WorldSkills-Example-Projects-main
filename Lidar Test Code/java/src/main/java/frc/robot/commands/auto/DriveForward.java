package frc.robot.commands.auto;

import frc.robot.commands.driveCommands.SimpleDrive;

public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        // super(new SimpleDrive(0.0, 0.0, 1).withTimeout(2));
        super(new SimpleDrive(0.5, 0.0, -0.05).withTimeout(5));

    }
}