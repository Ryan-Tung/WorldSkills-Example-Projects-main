package frc.robot.commands.auto;

import frc.robot.commands.driveCommands.DriveWithPID;

public class DriveForwardWithPID extends AutoCommand
{
    public DriveForwardWithPID ()
    {
        //Drive 1000mm, 10mm error, maintain an angle of 0, 1 degree error
        super(new DriveWithPID(1000, 1, 0, 0).withTimeout(1));
        //Timeout used just incase the command does not finish
    }
}