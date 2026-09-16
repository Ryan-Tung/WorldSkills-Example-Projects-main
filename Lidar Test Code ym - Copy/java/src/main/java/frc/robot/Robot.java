package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.NavigateOneMeter;
import frc.robot.commands.DriveUntilBlack;
import frc.robot.commands.LocalizationMonitor;
import frc.robot.commands.KeyboardDrive;
import frc.robot.commands.EncoderLocalizationTest;
import frc.robot.commands.FusionLocalizationCommand;



public class Robot extends TimedRobot
{
    // =====================================================
    // CURRENT SELECTED COMMAND
    // =====================================================

    private CommandBase selectedCommand;


    // =====================================================
    // ACTIVE TELEOP MODE
    //
    // The old code only read the chooser once in teleopInit().
    // If the chooser was changed AFTER Teleop was enabled,
    // FUSION_LOCALIZATION was visible in Shuffleboard but was
    // never actually scheduled.
    //
    // We keep track of the currently scheduled mode so chooser
    // changes can safely take effect during Teleop.
    // =====================================================

    private String activeTeleopMode = "";


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
                new ParallelCommandGroup(
                        new KeyboardDrive(),
                        new LocalizationMonitor()
                )
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
        // ENCODER LOCALIZATION MODE
        //
        // Existing mode remains untouched.
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "ENCODER_LOCALIZATION",
                new KeyboardDrive()
        );


        // =================================================
        // FUSION LOCALIZATION MODE
        //
        // KeyboardDrive:
        //     controls robot motors
        //
        // FusionLocalizationCommand:
        //     encoder odometry = prediction
        //     LiDAR ICP       = correction
        //     navX            = heading
        //
        // Both run together.
        // =================================================

        addAutoMode(
                RobotContainer.autoChooser,
                "FUSION_LOCALIZATION",
                new ParallelCommandGroup(
                        new KeyboardDrive(),

                        new FusionLocalizationCommand(
                                RobotContainer.fusionLocalization
                        )
                )
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


        SmartDashboard.putString(
                "Active Teleop Mode",
                "NONE"
        );


        SmartDashboard.putString(
                "Mode Schedule Status",
                "NOT SCHEDULED"
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
    // GET CURRENT CHOOSER SELECTION
    // =====================================================

    private String getSelectedModeName()
    {
        String selectedMode =
                RobotContainer.autoChooser
                        .getSelected();


        if (selectedMode == null)
        {
            selectedMode =
                    "LIDAR_MOVING";
        }


        return selectedMode;
    }


    // =====================================================
    // SCHEDULE SELECTED TELEOP MODE
    //
    // forceRestart = true:
    //     used when Teleop first starts
    //
    // forceRestart = false:
    //     only changes command when chooser selection changes
    // =====================================================

    private void scheduleSelectedTeleopMode(
            boolean forceRestart)
    {
        String selectedMode =
                getSelectedModeName();


        boolean modeChanged =
                !selectedMode.equals(
                        activeTeleopMode
                );


        if (
            !forceRestart
            &&
            !modeChanged
        )
        {
            return;
        }


        // -------------------------------------------------
        // STOP PREVIOUS MODE
        // -------------------------------------------------

        if (selectedCommand != null)
        {
            selectedCommand.cancel();

            selectedCommand =
                    null;
        }


        RobotContainer.driveTrain
                .holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );


        // -------------------------------------------------
        // GET NEW MODE COMMAND
        // -------------------------------------------------

        selectedCommand =
                RobotContainer.autoMode
                        .get(
                                selectedMode
                        );


        activeTeleopMode =
                selectedMode;


        // -------------------------------------------------
        // SHOW EXACT MODE THAT WAS ACTUALLY SCHEDULED
        // -------------------------------------------------

        SmartDashboard.putString(
                "Selected Robot Mode",
                selectedMode
        );


        SmartDashboard.putString(
                "Active Teleop Mode",
                activeTeleopMode
        );


        // -------------------------------------------------
        // START NEW MODE
        // -------------------------------------------------

        if (selectedCommand != null)
        {
            selectedCommand.schedule();


            SmartDashboard.putString(
                    "Mode Schedule Status",
                    "SCHEDULED: "
                    +
                    selectedMode
            );
        }

        else
        {
            SmartDashboard.putString(
                    "Mode Schedule Status",
                    "COMMAND NOT FOUND: "
                    +
                    selectedMode
            );
        }
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


        activeTeleopMode =
                "";


        SmartDashboard.putString(
                "Active Teleop Mode",
                "NONE"
        );


        SmartDashboard.putString(
                "Mode Schedule Status",
                "DISABLED"
        );


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
        // Cancel anything left from a previous robot mode.
        CommandScheduler
                .getInstance()
                .cancelAll();


        selectedCommand =
                null;


        activeTeleopMode =
                "";


        RobotContainer.driveTrain
                .holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );


        /*
         * Schedule whatever is currently selected.
         *
         * If Shuffleboard delivers a different chooser value a
         * little later, teleopPeriodic() below will detect the
         * change and safely switch modes.
         */
        scheduleSelectedTeleopMode(
                true
        );
    }


    // =====================================================
    // TELEOP PERIODIC
    // =====================================================

    @Override
    public void teleopPeriodic()
    {
        /*
         * CommandScheduler itself runs in robotPeriodic().
         *
         * Here we only watch for a chooser change.
         *
         * This means:
         *
         * 1. Robot can already be in Teleop.
         * 2. User selects FUSION_LOCALIZATION in Shuffleboard.
         * 3. Previous command is cancelled.
         * 4. ParallelCommandGroup is scheduled immediately.
         * 5. FusionLocalizationCommand.initialize() calls
         *    startFusion().
         * 6. FusionLocalization/Active becomes TRUE.
         */
        scheduleSelectedTeleopMode(
                false
        );
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