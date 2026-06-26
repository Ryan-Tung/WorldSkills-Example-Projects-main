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
        
            new DriveForwardPrimitive(500,-30),

            new WaitCommand(0.25),

            new TurnToAnglePrimitive(60),

            new WaitCommand(0.25),

            new DriveForwardPrimitive(500,60),

            
            new WaitCommand(0.25),

            new TurnToAnglePrimitive(150),

            
            new WaitCommand(0.25),

            new DriveForwardPrimitive(500,150),  

            new WaitCommand(0.25),

            new TurnToAnglePrimitive(-120),

            new WaitCommand(0.25),

            new DriveForwardPrimitive(500,-130),
            
            new WaitCommand(0.25),
            
            new TurnToAnglePrimitive(-30),
            
            new WaitCommand(0.25),
            
            new DriveForwardPrimitive(500,-30)));

            




}}