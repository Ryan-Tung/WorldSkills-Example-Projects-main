package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.CobraTest;


public class DriveUntilBlack extends CommandBase
{
    private static final DriveTrain drive =
            RobotContainer.driveTrain;

    private static final CobraTest cobra =
            RobotContainer.cobra;


    // =====================================================
    // TARGET DISTANCE
    // =====================================================

    // 2000 mm = 2 metres
    private static final double TARGET_DISTANCE_MM =
            2000.0;


    // =====================================================
    // FORWARD SPEED
    // =====================================================

    private static final double FORWARD_SPEED =
            0.25;


    // =====================================================
    // HEADING PID
    // =====================================================

    private final PIDController headingPID;

    private static final double HEADING_KP =
            0.01;

    private static final double MAX_HEADING_CORRECTION =
            0.08;


    // =====================================================
    // VARIABLES
    // =====================================================

    private boolean finished =
            false;

    private double startDistance =
            0.0;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public DriveUntilBlack()
    {
        addRequirements(
                drive
        );


        headingPID =
                new PIDController(
                        HEADING_KP,
                        0.0,
                        0.0
                );


        headingPID.enableContinuousInput(
                -180.0,
                180.0
        );
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        // Reset robot pose / encoders
        drive.resetPose();


        // Reset heading PID
        headingPID.reset();


        // Record starting forward position
        startDistance =
                drive.getAverageForwardEncoderDistance();


        finished =
                false;


        SmartDashboard.putString(
                "Cobra Drive State",
                "MOVING"
        );


        SmartDashboard.putNumber(
                "Cobra Target Distance",
                TARGET_DISTANCE_MM
        );
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // CALCULATE DISTANCE TRAVELLED
        // =================================================

        double currentDistance =
                drive.getAverageForwardEncoderDistance();


        double distanceTravelled =
                Math.abs(
                        currentDistance
                        -
                        startDistance
                );


        SmartDashboard.putNumber(
                "Cobra Distance Travelled",
                distanceTravelled
        );


        // =================================================
        // STOP IF BLACK TAPE IS DETECTED
        // =================================================

        if (cobra.blackTapeDetected())
        {
            finished =
                    true;


            SmartDashboard.putString(
                    "Cobra Drive State",
                    "BLACK TAPE DETECTED"
            );


            stopRobot();


            return;
        }


        // =================================================
        // STOP AT 2000 MM
        // =================================================

        if (
            distanceTravelled
            >=
            TARGET_DISTANCE_MM
        )
        {
            finished =
                    true;


            SmartDashboard.putString(
                    "Cobra Drive State",
                    "2000 MM REACHED"
            );


            stopRobot();


            return;
        }


        // =================================================
        // HEADING CORRECTION
        // =================================================

        double yaw =
                drive.getYaw();


        double correction =
                headingPID.calculate(
                        yaw,
                        0.0
                );


        correction =
                MathUtil.clamp(
                        correction,
                        -MAX_HEADING_CORRECTION,
                        MAX_HEADING_CORRECTION
                );


        // =================================================
        // DEBUG VALUES
        // =================================================

        SmartDashboard.putNumber(
                "Cobra Drive Yaw",
                yaw
        );


        SmartDashboard.putNumber(
                "Cobra Heading Correction",
                correction
        );


        SmartDashboard.putNumber(
                "Cobra Forward Speed",
                FORWARD_SPEED
        );


        SmartDashboard.putBoolean(
                "Cobra Tape Detected",
                false
        );


        // =================================================
        // MOVE FORWARD
        // =================================================

        drive.holonomicDrive(
                0.0,
                FORWARD_SPEED,
                correction
        );
    }


    // =====================================================
    // FINISHED?
    // =====================================================

    @Override
    public boolean isFinished()
    {
        return finished;
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        stopRobot();


        if (interrupted)
        {
            SmartDashboard.putString(
                    "Cobra Drive State",
                    "INTERRUPTED"
            );
        }
    }


    // =====================================================
    // STOP ROBOT
    // =====================================================

    private void stopRobot()
    {
        drive.holonomicDrive(
                0.0,
                0.0,
                0.0
        );
    }
}