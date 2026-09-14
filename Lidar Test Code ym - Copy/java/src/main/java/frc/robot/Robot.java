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
import frc.robot.commands.KeyboardDrive;


public class Robot extends TimedRobot
{
    // =====================================================
    // CURRENT SELECTED COMMAND
    // =====================================================

    private CommandBase selectedCommand;


    // =====================================================
    // ROBOT INIT
    // =====================================================

    @Override
    public void robotInit()
    {
        // =================================================
        // CREATE ROBOT CONTAINER
        // =================================================

        new RobotContainer();


        // =================================================
        // DEFAULT MODE
        // =================================================

        RobotContainer.autoChooser
                .setDefaultOption(
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
        // KEYBOARD DRIVE MODE
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "KEYBOARD_DRIVE",
                new KeyboardDrive()
        );


        // =================================================
        // SHUFFLEBOARD MODE SELECTOR
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
        // DASHBOARD
        // =================================================

        SmartDashboard.putString(
                "Selected Robot Mode",
                "LIDAR_MOVING"
        );


        SmartDashboard.putString(
                "Keyboard Drive State",
                "STOPPED"
        );
    }


    // =====================================================
    // ADD MODE HELPER
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
        // Run commands
        CommandScheduler
                .getInstance()
                .run();


        // =================================================
        // SHOW SELECTED MODE
        // =================================================

        String selected =
                RobotContainer.autoChooser
                        .getSelected();


        if (selected == null)
        {
            selected =
                    "LIDAR_MOVING";
        }


        SmartDashboard.putString(
                "Selected Robot Mode",
                selected
        );
    }


    // =====================================================
    // DISABLED INIT
    // =====================================================

    @Override
    public void disabledInit()
    {
        // Cancel running command
        CommandScheduler
                .getInstance()
                .cancelAll();


        selectedCommand =
                null;


        // Stop drivetrain
        RobotContainer.driveTrain
                .holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );


        SmartDashboard.putString(
                "Keyboard Drive State",
                "STOPPED"
        );
    }


    // =====================================================
    // DISABLED PERIODIC
    // =====================================================

    @Override
    public void disabledPeriodic()
    {
    }


    // =====================================================
    // AUTONOMOUS INIT
    // =====================================================

    @Override
    public void autonomousInit()
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


    // =====================================================
    // AUTONOMOUS PERIODIC
    // =====================================================

    @Override
    public void autonomousPeriodic()
    {
    }


    // =====================================================
    // TELEOP INIT
    // =====================================================

    @Override
    public void teleopInit()
    {
        // =================================================
        // CANCEL PREVIOUS COMMAND
        // =================================================

        CommandScheduler
                .getInstance()
                .cancelAll();


        // =================================================
        // GET SELECTED SHUFFLEBOARD MODE
        // =================================================

        String selectedMode =
                RobotContainer.autoChooser
                        .getSelected();


        if (selectedMode == null)
        {
            selectedMode =
                    "LIDAR_MOVING";
        }


        // =================================================
        // GET COMMAND
        // =================================================

        selectedCommand =
                RobotContainer.autoMode
                        .get(
                                selectedMode
                        );


        // =================================================
        // SHOW MODE
        // =================================================

        SmartDashboard.putString(
                "Selected Robot Mode",
                selectedMode
        );


        // =================================================
        // RUN COMMAND
        // =================================================

        if (selectedCommand != null)
        {
            selectedCommand.schedule();
        }
    }


    // =====================================================
    // TELEOP PERIODIC
    // =====================================================

    @Override
    public void teleopPeriodic()
    {
        /*
         * CommandScheduler runs in robotPeriodic().
         *
         * KEYBOARD_DRIVE will therefore continuously run
         * KeyboardDrive.execute().
         */
    }


    // =====================================================
    // TEST INIT
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


    // =====================================================
    // TEST PERIODIC
    // =====================================================

    @Override
    public void testPeriodic()
    {
    }
}