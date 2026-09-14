package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.commands.NavigateOneMeter;
import frc.robot.commands.DriveUntilBlack;
import frc.robot.commands.LocalizationMonitor;


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
        // DEFAULT MODE - LIDAR MOVING
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
        // COBRA MODE
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "COBRA_STOP_AT_BLACK",
                new DriveUntilBlack()
        );


        // =================================================
        // LIDAR LOCALIZATION MODE
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "LIDAR_LOCALIZATION",
                new LocalizationMonitor()
        );


        // =================================================
        // SHUFFLEBOARD MODE CHOOSER
        // =================================================

        Shuffleboard
                .getTab("Training Robot")
                .add(
                        "Robot Mode",
                        RobotContainer.autoChooser
                )
                .withWidget(
                        BuiltInWidgets.kComboBoxChooser
                );


        // =================================================
        // DEBUG SELECTED MODE
        // =================================================

        SmartDashboard.putString(
                "Selected Robot Mode",
                "LIDAR_MOVING"
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


        // Show currently selected mode
        String selectedMode =
                RobotContainer.autoChooser
                        .getSelected();


        if (selectedMode != null)
        {
            SmartDashboard.putString(
                    "Selected Robot Mode",
                    selectedMode
            );
        }
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
        // Cancel previously running command
        CommandScheduler
                .getInstance()
                .cancelAll();


        // =================================================
        // GET SELECTED MODE
        // =================================================

        String selectedMode =
                RobotContainer.autoChooser
                        .getSelected();


        // Safety fallback
        if (selectedMode == null)
        {
            selectedMode =
                    "LIDAR_MOVING";
        }


        // =================================================
        // SHOW SELECTED MODE
        // =================================================

        SmartDashboard.putString(
                "Selected Robot Mode",
                selectedMode
        );


        // =================================================
        // GET COMMAND
        // =================================================

        CommandBase selectedCommand =
                RobotContainer.autoMode
                        .get(
                                selectedMode
                        );


        // =================================================
        // RUN COMMAND
        // =================================================

        if (selectedCommand != null)
        {
            selectedCommand.schedule();
        }
        else
        {
            // Safety stop
            RobotContainer.driveTrain
                    .holonomicDrive(
                            0.0,
                            0.0,
                            0.0
                    );
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