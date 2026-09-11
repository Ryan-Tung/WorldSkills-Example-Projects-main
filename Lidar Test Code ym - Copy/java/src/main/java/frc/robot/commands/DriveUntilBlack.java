package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
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
    // TAPE DETECTED
    // =====================================================

    private boolean tapeDetected =
            false;


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
        drive.resetPose();

        headingPID.reset();

        tapeDetected =
                false;
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // BLACK TAPE DETECTED
        // =================================================

        if (
            cobra.blackTapeDetected()
        )
        {
            tapeDetected =
                    true;


            stopRobot();


            return;
        }


        // =================================================
        // KEEP ROBOT STRAIGHT
        // =================================================

        double correction =
                headingPID.calculate(
                        drive.getYaw(),
                        0.0
                );


        correction =
                MathUtil.clamp(
                        correction,
                        -MAX_HEADING_CORRECTION,
                        MAX_HEADING_CORRECTION
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
    // FINISHED
    // =====================================================

    @Override
    public boolean isFinished()
    {
        return tapeDetected;
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        stopRobot();
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