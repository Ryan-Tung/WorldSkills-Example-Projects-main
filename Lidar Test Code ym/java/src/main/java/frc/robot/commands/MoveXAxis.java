package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class MoveXAxis extends CommandBase
{
    // =====================================================
    // DRIVETRAIN
    // =====================================================

    private static final DriveTrain drive =
            RobotContainer.driveTrain;


    // =====================================================
    // TARGET
    // =====================================================

    private final double targetDistance;

    private final int direction;


    // =====================================================
    // X DISTANCE PID
    // =====================================================

    private final PIDController pidX;


    // =====================================================
    // TIMER
    // =====================================================

    private final Timer timer =
            new Timer();


    // =====================================================
    // TUNING
    // =====================================================

    /*
     * Same basic X PID gain as your old
     * StrafeWithPID.
     */
    private static final double X_KP =
            0.01;

    private static final double X_KI =
            0.0;

    private static final double X_KD =
            0.0003;


    // Maximum sideways speed
    private static final double MAX_X_SPEED =
            0.50;


    /*
     * Important:
     *
     * Your working robot code already used:
     *
     * +X -> Z = -0.05
     * -X -> Z = +0.05
     *
     * This compensates for the robot naturally
     * curving while strafing.
     */
    private static final double STRAFE_CORRECTION =
            0.05;


    // Distance tolerance
    private static final double TOLERANCE =
            8.0;


    // Safety timeout
    private static final double MAX_TIME =
            5.0;


    // =====================================================
    // STATE
    // =====================================================

    private boolean finished =
            false;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public MoveXAxis(
            double distance_mm,
            double heading_deg)
    {
        /*
         * heading_deg is intentionally not used yet.
         *
         * For now we use the known working
         * physical correction from your old code.
         */

        targetDistance =
                Math.abs(distance_mm);


        if (distance_mm >= 0.0)
        {
            direction =
                    1;
        }
        else
        {
            direction =
                    -1;
        }


        addRequirements(
                drive
        );


        pidX =
                new PIDController(
                        X_KP,
                        X_KI,
                        X_KD
                );


        pidX.setTolerance(
                TOLERANCE
        );
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        drive.resetEncoders();


        pidX.reset();


        timer.reset();

        timer.start();


        finished =
                false;


        System.out.println(
                "============================"
        );


        System.out.println(
                "MoveXAxis START"
        );


        System.out.println(
                "Target = "
                + targetDistance
                + " mm"
        );


        System.out.println(
                "Direction = "
                + direction
        );


        System.out.println(
                "============================"
        );
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // SIDEWAYS DISTANCE
        // =================================================

        double rawDistance =
                drive
                        .getAverageStrafeEncoderDistance();


        double travelled =
                Math.abs(
                        rawDistance
                );


        double remaining =
                targetDistance
                -
                travelled;


        // =================================================
        // TARGET REACHED
        // =================================================

        if (
            remaining
            <=
            TOLERANCE
        )
        {
            /*
             * Stop ALL 3 wheels on the same loop.
             */

            drive.holonomicDrive(
                    0.0,
                    0.0,
                    0.0
            );


            finished =
                    true;


            return;
        }


        // =================================================
        // X PID
        // =================================================

        double xSpeed =
                Math.abs(
                        pidX.calculate(
                                travelled,
                                targetDistance
                        )
                );


        xSpeed =
                MathUtil.clamp(
                        xSpeed,
                        0.0,
                        MAX_X_SPEED
                );


        // Apply local X direction
        xSpeed *=
                direction;


        // =================================================
        // STRAIGHT-STRAFE CORRECTION
        // =================================================
        //
        // From your existing working code:
        //
        // +X:
        // x = +0.5
        // z = -0.05
        //
        // -X:
        // x = -0.5
        // z = +0.05
        //
        // Therefore:

        double zCorrection =
                -STRAFE_CORRECTION
                *
                direction;


        // =================================================
        // IMPORTANT:
        // REDUCE CORRECTION WHEN X SPEED GETS SMALL
        // =================================================

        /*
         * We don't want Z to become stronger than
         * the X movement near the end.
         */

        double maxCorrection =
                Math.abs(xSpeed)
                *
                0.20;


        if (
            Math.abs(zCorrection)
            >
            maxCorrection
        )
        {
            zCorrection =
                    Math.copySign(
                            maxCorrection,
                            zCorrection
                    );
        }


        // =================================================
        // MOVE
        // =================================================

        drive.holonomicDrive(
                xSpeed,
                0.0,
                zCorrection
        );


        // =================================================
        // DEBUG
        // =================================================

        SmartDashboard.putNumber(
                "Move X Target",
                targetDistance
        );


        SmartDashboard.putNumber(
                "Move X Travelled",
                travelled
        );


        SmartDashboard.putNumber(
                "Move X Remaining",
                remaining
        );


        SmartDashboard.putNumber(
                "Move X Speed",
                xSpeed
        );


        SmartDashboard.putNumber(
                "Move X Z Correction",
                zCorrection
        );


        SmartDashboard.putNumber(
                "Move X Yaw",
                drive.getYaw()
        );
    }


    // =====================================================
    // FINISHED
    // =====================================================

    @Override
    public boolean isFinished()
    {
        if (finished)
        {
            return true;
        }


        if (
            timer.get()
            >=
            MAX_TIME
        )
        {
            System.out.println(
                    "MoveXAxis TIMEOUT"
            );


            return true;
        }


        return false;
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        timer.stop();


        /*
         * Force all three motors to zero together.
         */

        drive.holonomicDrive(
                0.0,
                0.0,
                0.0
        );


        System.out.println(
                "============================"
        );


        System.out.println(
                "MoveXAxis END"
        );


        System.out.println(
                "Final Distance = "
                + Math.abs(
                        drive
                                .getAverageStrafeEncoderDistance()
                )
        );


        System.out.println(
                "Final Yaw = "
                + drive.getYaw()
        );


        System.out.println(
                "============================"
        );
    }
}