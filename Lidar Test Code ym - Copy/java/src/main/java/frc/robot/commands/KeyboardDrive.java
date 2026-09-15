package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;


public class KeyboardDrive extends CommandBase
{
    // =====================================================
    // DRIVETRAIN
    // =====================================================

    private final DriveTrain drive;


    // =====================================================
    // NETWORKTABLES
    //
    // Python sends:
    //
    // X
    // Y
    // Z
    // Enabled
    // Heartbeat
    // =====================================================

    private final NetworkTable keyboardTable =
            NetworkTableInstance
                    .getDefault()
                    .getTable("KeyboardDrive");


    // =====================================================
    // SAFETY
    //
    // If Python stops communicating for more than
    // 0.30 seconds, immediately stop the robot.
    // =====================================================

    private static final double HEARTBEAT_TIMEOUT =
            0.30;


    private double lastHeartbeat =
            -1.0;


    private double lastHeartbeatTime =
            0.0;


    // =====================================================
    // PURE CRAB HEADING HOLD
    //
    // A / D should move sideways without slowly turning.
    //
    // When a pure crab command begins, remember the current
    // navX heading. A small Z correction is then added to
    // keep that heading while the robot moves sideways.
    // =====================================================

    private static final double CRAB_COMMAND_THRESHOLD =
            0.05;


    private static final double CRAB_HEADING_KP =
            0.015;


    private static final double CRAB_HEADING_DEADBAND_DEG =
            0.50;


    private static final double CRAB_MAX_TURN_CORRECTION =
            0.10;


    private boolean crabHeadingHoldActive =
            false;


    private double crabHeadingTarget =
            0.0;


    private double crabHeadingError =
            0.0;


    private double crabTurnCorrection =
            0.0;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public KeyboardDrive()
    {
        drive =
                RobotContainer.driveTrain;


        addRequirements(
                drive
        );
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        stopDrive();


        lastHeartbeat =
                keyboardTable
                        .getEntry("Heartbeat")
                        .getDouble(-1.0);


        lastHeartbeatTime =
                Timer.getFPGATimestamp();


        crabHeadingHoldActive =
                false;


        crabHeadingTarget =
                drive.getYaw();


        crabHeadingError =
                0.0;


        crabTurnCorrection =
                0.0;


        SmartDashboard.putString(
                "Keyboard Drive State",
                "WAITING"
        );


        SmartDashboard.putNumber(
                "Keyboard X",
                0.0
        );


        SmartDashboard.putNumber(
                "Keyboard Y",
                0.0
        );


        SmartDashboard.putNumber(
                "Keyboard Z",
                0.0
        );


        SmartDashboard.putBoolean(
                "Crab Heading Hold",
                false
        );


        SmartDashboard.putNumber(
                "Crab Heading Target",
                crabHeadingTarget
        );


        SmartDashboard.putNumber(
                "Crab Heading Error",
                0.0
        );


        SmartDashboard.putNumber(
                "Crab Turn Correction",
                0.0
        );
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // READ HEARTBEAT
        // =================================================

        double heartbeat =
                keyboardTable
                        .getEntry("Heartbeat")
                        .getDouble(-1.0);


        // =================================================
        // CHECK FOR NEW PYTHON UPDATE
        // =================================================

        if (heartbeat != lastHeartbeat)
        {
            lastHeartbeat =
                    heartbeat;


            lastHeartbeatTime =
                    Timer.getFPGATimestamp();
        }


        // =================================================
        // TIME SINCE LAST PYTHON MESSAGE
        // =================================================

        double timeSinceHeartbeat =
                Timer.getFPGATimestamp()
                -
                lastHeartbeatTime;


        // =================================================
        // IS KEYBOARD WINDOW ENABLED?
        // =================================================

        boolean keyboardEnabled =
                keyboardTable
                        .getEntry("Enabled")
                        .getBoolean(false);


        // =================================================
        // SAFETY STOP
        //
        // Stop if:
        //
        // Python closed
        // Python crashed
        // NetworkTables stopped
        // keyboard window loses focus
        // window minimized
        // user Alt+Tabs away
        // =================================================

        if (
            !keyboardEnabled
            ||
            timeSinceHeartbeat
            >
            HEARTBEAT_TIMEOUT
        )
        {
            stopDrive();


            SmartDashboard.putString(
                    "Keyboard Drive State",
                    "STOPPED"
            );


            return;
        }


        // =================================================
        // READ X / Y / Z
        // =================================================

        double x =
                keyboardTable
                        .getEntry("X")
                        .getDouble(0.0);


        double y =
                keyboardTable
                        .getEntry("Y")
                        .getDouble(0.0);


        double z =
                keyboardTable
                        .getEntry("Z")
                        .getDouble(0.0);


        // =================================================
        // EXTRA SAFETY CLAMP
        // =================================================

        x = clamp(
                x,
                -0.60,
                0.60
        );


        y = clamp(
                y,
                -0.60,
                0.60
        );


        z = clamp(
                z,
                -0.60,
                0.60
        );


        // =================================================
        // PURE A / D CRAB HEADING HOLD
        //
        // Pure crab means:
        //
        // X != 0
        // Y  = 0
        // Z  = 0
        //
        // W+A, W+D, A+Q, D+E, etc. are NOT treated as
        // pure crab, so the user's combined commands are
        // preserved exactly.
        // =================================================

        boolean pureCrab =
                Math.abs(x)
                >=
                CRAB_COMMAND_THRESHOLD
                &&
                Math.abs(y)
                <
                CRAB_COMMAND_THRESHOLD
                &&
                Math.abs(z)
                <
                CRAB_COMMAND_THRESHOLD;


        if (pureCrab)
        {
            if (!crabHeadingHoldActive)
            {
                crabHeadingTarget =
                        drive.getYaw();


                crabHeadingHoldActive =
                        true;
            }


            crabHeadingError =
                    normalizeAngle(
                            crabHeadingTarget
                            -
                            drive.getYaw()
                    );


            if (
                Math.abs(
                        crabHeadingError
                )
                <=
                CRAB_HEADING_DEADBAND_DEG
            )
            {
                crabTurnCorrection =
                        0.0;
            }
            else
            {
                crabTurnCorrection =
                        clamp(
                                CRAB_HEADING_KP
                                *
                                crabHeadingError,
                                -CRAB_MAX_TURN_CORRECTION,
                                CRAB_MAX_TURN_CORRECTION
                        );
            }


            z =
                    crabTurnCorrection;
        }
        else
        {
            crabHeadingHoldActive =
                    false;


            crabHeadingError =
                    0.0;


            crabTurnCorrection =
                    0.0;
        }


        // =================================================
        // DRIVE
        //
        // +X = crab right
        // -X = crab left
        //
        // +Y = forward
        // -Y = backward
        //
        // +Z = rotate right
        // -Z = rotate left
        // =================================================

        drive.holonomicDrive(
                x,
                y,
                z
        );


        // =================================================
        // SHUFFLEBOARD
        // =================================================

        SmartDashboard.putNumber(
                "Keyboard X",
                x
        );


        SmartDashboard.putNumber(
                "Keyboard Y",
                y
        );


        SmartDashboard.putNumber(
                "Keyboard Z",
                z
        );


        SmartDashboard.putBoolean(
                "Crab Heading Hold",
                crabHeadingHoldActive
        );


        SmartDashboard.putNumber(
                "Crab Heading Target",
                crabHeadingTarget
        );


        SmartDashboard.putNumber(
                "Crab Heading Error",
                crabHeadingError
        );


        SmartDashboard.putNumber(
                "Crab Turn Correction",
                crabTurnCorrection
        );


        SmartDashboard.putString(
                "Keyboard Drive State",
                "RUNNING"
        );
    }


    // =====================================================
    // NORMALIZE ANGLE
    // =====================================================

    private double normalizeAngle(
            double angle)
    {
        while (angle > 180.0)
        {
            angle -=
                    360.0;
        }


        while (angle < -180.0)
        {
            angle +=
                    360.0;
        }


        return angle;
    }


    // =====================================================
    // CLAMP
    // =====================================================

    private double clamp(
            double value,
            double minimum,
            double maximum)
    {
        if (value > maximum)
        {
            return maximum;
        }


        if (value < minimum)
        {
            return minimum;
        }


        return value;
    }


    // =====================================================
    // STOP DRIVE
    // =====================================================

    private void stopDrive()
    {
        crabHeadingHoldActive =
                false;


        crabHeadingError =
                0.0;


        crabTurnCorrection =
                0.0;


        drive.holonomicDrive(
                0.0,
                0.0,
                0.0
        );


        SmartDashboard.putNumber(
                "Keyboard X",
                0.0
        );


        SmartDashboard.putNumber(
                "Keyboard Y",
                0.0
        );


        SmartDashboard.putNumber(
                "Keyboard Z",
                0.0
        );


        SmartDashboard.putBoolean(
                "Crab Heading Hold",
                false
        );


        SmartDashboard.putNumber(
                "Crab Heading Error",
                0.0
        );


        SmartDashboard.putNumber(
                "Crab Turn Correction",
                0.0
        );
    }


    // =====================================================
    // KEEP RUNNING
    // =====================================================

    @Override
    public boolean isFinished()
    {
        return false;
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        stopDrive();


        SmartDashboard.putString(
                "Keyboard Drive State",
                "STOPPED"
        );
    }
}