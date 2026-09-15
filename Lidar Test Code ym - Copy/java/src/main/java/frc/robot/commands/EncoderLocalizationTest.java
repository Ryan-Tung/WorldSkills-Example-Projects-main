package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;


public class EncoderLocalizationTest extends CommandBase
{
    // =====================================================
    // SUBSYSTEM
    // =====================================================

    private final DriveTrain driveTrain;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public EncoderLocalizationTest(
            DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        /*
         * Do NOT call addRequirements(driveTrain).
         *
         * This command only monitors odometry, so it is
         * allowed to run in parallel with KeyboardDrive,
         * which owns the drivetrain for motor control.
         */
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        // Start encoder localization from a clean pose.
        driveTrain.resetPose();

        SmartDashboard.putString(
                "EncoderLocalization/Status",
                "RUNNING"
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/X",
                0.0
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/Y",
                0.0
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/Heading",
                0.0
        );
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        double robotX =
                driveTrain.getRobotX();

        double robotY =
                driveTrain.getRobotY();

        double robotHeading =
                driveTrain.getRobotHeading();

        double leftEncoder =
                driveTrain.getLeftEncoderDistance();

        double rightEncoder =
                driveTrain.getRightEncoderDistance();

        double backEncoder =
                driveTrain.getBackEncoderDistance();

        SmartDashboard.putNumber(
                "EncoderLocalization/X",
                robotX
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/Y",
                robotY
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/Heading",
                robotHeading
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/LeftEncoder",
                leftEncoder
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/RightEncoder",
                rightEncoder
        );

        SmartDashboard.putNumber(
                "EncoderLocalization/BackEncoder",
                backEncoder
        );

        SmartDashboard.putString(
                "EncoderLocalization/Pose",
                String.format(
                        "X: %.3f m   Y: %.3f m   Heading: %.1f deg",
                        robotX,
                        robotY,
                        robotHeading
                )
        );
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        SmartDashboard.putString(
                "EncoderLocalization/Status",
                interrupted
                        ? "INTERRUPTED"
                        : "FINISHED"
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
}
