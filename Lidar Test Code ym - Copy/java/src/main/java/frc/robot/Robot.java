package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.NavigateOneMeter;
import frc.robot.commands.DriveUntilBlack;

public class Robot extends TimedRobot
{
    private RobotContainer robotContainer;


    // =====================================================
    // ROBOT INIT
    // =====================================================

    @Override
    public void robotInit()
    {
        robotContainer =
                new RobotContainer();


        // =================================================
        // DEFAULT MODE
        // =================================================

        RobotContainer.autoChooser.setDefaultOption(
                "LIDAR_MOVING",
                "LIDAR_MOVING"
        );

        RobotContainer.autoMode.put(
                "LIDAR_MOVING",
                new NavigateOneMeter()
        );


        // =================================================
        // ADD OTHER MODES
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "COBRA_STOP_AT_BLACK",
                new DriveUntilBlack()
        );


        // =================================================
        // SHOW CHOOSER ON SHUFFLEBOARD
        // =================================================

        SmartDashboard.putData(
                "Robot Mode",
                RobotContainer.autoChooser
        );
    }


    // =====================================================
    // ADD AUTO MODE
    // =====================================================

    public void addAutoMode(
            SendableChooser<String> chooser,
            String auto,
            CommandBase cmd)
    {
        chooser.addOption(
                auto,
                auto
        );

        RobotContainer.autoMode.put(
                auto,
                cmd
        );
    }


    // =====================================================
    // ROBOT PERIODIC
    // =====================================================

    @Override
    public void robotPeriodic()
    {
        CommandScheduler
                .getInstance()
                .run();
    }


    // =====================================================
    // DISABLED
    // =====================================================

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


    // =====================================================
    // AUTONOMOUS
    // =====================================================

    @Override
    public void autonomousInit()
    {
    }


    @Override
    public void autonomousPeriodic()
    {
    }


    // =====================================================
    // TELEOP
    // =====================================================

    @Override
    public void teleopInit()
    {
        CommandScheduler
                .getInstance()
                .cancelAll();


        String selectedMode =
                RobotContainer.autoChooser
                        .getSelected();


        CommandBase selectedCommand =
                RobotContainer.autoMode
                        .get(
                                selectedMode
                        );


        if (selectedCommand != null)
        {
            selectedCommand.schedule();
        }
    }


    @Override
    public void teleopPeriodic()
    {
    }


    // =====================================================
    // TEST
    // =====================================================

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