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


        SmartDashboard.putString(
                "Keyboard Drive State",
                "RUNNING"
        );
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