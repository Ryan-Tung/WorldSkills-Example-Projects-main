package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.NavigateOneMeter;

public class Robot extends TimedRobot
{
    private RobotContainer robotContainer;


    @Override
    public void robotInit()
    {
        robotContainer =
                new RobotContainer();
    }


    @Override
    public void robotPeriodic()
    {
        CommandScheduler
                .getInstance()
                .run();
    }


    @Override
    public void disabledInit()
    {
        CommandScheduler
                .getInstance()
                .cancelAll();


        RobotContainer.driveTrain
                .holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );
    }


    @Override
    public void disabledPeriodic()
    {
    }


    @Override
    public void autonomousInit()
    {
    }


    @Override
    public void autonomousPeriodic()
    {
    }


    @Override
    public void teleopInit()
    {
        CommandScheduler
                .getInstance()
                .cancelAll();


        new NavigateOneMeter()
                .schedule();
    }


    @Override
    public void teleopPeriodic()
    {
    }


    @Override
    public void testInit()
    {
        CommandScheduler
                .getInstance()
                .cancelAll();


        RobotContainer.driveTrain
                .holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );
    }


    @Override
    public void testPeriodic()
    {
    }
}