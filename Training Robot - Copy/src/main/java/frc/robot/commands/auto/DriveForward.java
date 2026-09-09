package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveForwardPrimitive;
import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.TurnToAnglePrimitive;

public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        super(new SequentialCommandGroup(
            
        
            new DriveForwardPrimitive(300,0),

            new WaitCommand(0.25),

            new TurnToAnglePrimitive(-50),

            new WaitCommand(0.25),

            new DriveForwardPrimitive(1000,-50),

            
            new WaitCommand(0.25),

            new TurnToAnglePrimitive(-140),

            
            new WaitCommand(0.25),

            new DriveForwardPrimitive(500,-140),  

            new WaitCommand(0.25),

            new TurnToAnglePrimitive(-50),

            new WaitCommand(0.25),

            new DriveForwardPrimitive(500,-50),
            
            new WaitCommand(0.25),
            
            new TurnToAnglePrimitive(40),
            
            new WaitCommand(0.25),
            
            new DriveForwardPrimitive(500,40),
            
            new WaitCommand(0.25),
            
            new DriveForwardPrimitive(500, 170)));
            




}}