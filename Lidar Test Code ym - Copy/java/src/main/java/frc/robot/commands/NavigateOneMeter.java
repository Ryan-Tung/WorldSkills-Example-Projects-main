package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LidarTest;

public class NavigateOneMeter extends CommandBase
{
    private static final DriveTrain drive =
            RobotContainer.driveTrain;

    private static final LidarTest lidar =
            RobotContainer.lidar;


    // =====================================================
    // FINAL TARGET
    // =====================================================

    private static final double TARGET_FORWARD_MM =
            1000.0;


    // =====================================================
    // FORWARD SPEED
    // =====================================================

    private static final double FORWARD_SPEED =
            0.30;

    private static final double FINAL_FORWARD_SPEED =
            0.18;

    private static final double FINAL_SLOW_DISTANCE =
            150.0;


    // =====================================================
    // CRAB
    // =====================================================

    private static final double CRAB_SPEED =
            0.45;

    /*
     * Your working crab correction.
     *
     * +X -> -0.05
     * -X -> +0.05
     */
    private static final double CRAB_CORRECTION =
            0.05;


    /*
     * +X physically moves right.
     *
     * Change to -1 only if your robot is opposite.
     */
    private static final int RIGHT_X_SIGN =
            1;


    // =====================================================
    // LIDAR
    // =====================================================

    private static final double MIN_VALID_LIDAR =
            120.0;

    private static final double OBSTACLE_DISTANCE =
            300.0;

    private static final double FRONT_CLEAR_DISTANCE =
            400.0;

    private static final double SIDE_CLEAR_DISTANCE =
            400.0;


    // =====================================================
    // FILTERING
    // =====================================================

    private static final int OBSTACLE_CONFIRM_LOOPS =
            15;

    private static final int CLEAR_CONFIRM_LOOPS =
            8;


    // =====================================================
    // IMPORTANT CLEARANCE SETTINGS
    // =====================================================

    /*
     * OLD = 150 mm
     *
     * NEW = 300 mm.
     *
     * Front LiDAR becoming clear does NOT mean
     * the side wheel has cleared the object.
     */
    private static final double MIN_CRAB_DISTANCE =
            300.0;


    private static final double MAX_CRAB_DISTANCE =
            700.0;


    /*
     * Robot must already move this far forward
     * beside the obstacle before side-clear is
     * even considered.
     */
    private static final double MIN_PASS_DISTANCE =
            400.0;


    /*
     * NEW:
     *
     * Once the side LiDAR says the obstacle has
     * disappeared, KEEP DRIVING FORWARD another
     * 250 mm.
     *
     * This gives the rear wheel/body enough room
     * before the robot comes sideways again.
     */
    private static final double EXTRA_CLEARANCE_DISTANCE =
            250.0;


    // =====================================================
    // HEADING
    // =====================================================

    private final PIDController headingPID;

    private static final double HEADING_KP =
            0.01;

    private static final double MAX_HEADING_CORRECTION =
            0.08;


    // =====================================================
    // STATES
    // =====================================================

    private enum State
    {
        FORWARD,

        CRAB_OUT,

        PASS_OBSTACLE,

        CLEARANCE_FORWARD,

        CRAB_BACK,

        FINISHED,

        FAILED
    }


    private State state =
            State.FORWARD;


    // =====================================================
    // COUNTERS
    // =====================================================

    private int obstacleCounter =
            0;

    private int frontClearCounter =
            0;

    private int sideClearCounter =
            0;


    // =====================================================
    // AVOIDANCE
    // =====================================================

    private int crabDirection =
            0;


    private double crabStartEncoder =
            0.0;


    private double crabOutDistance =
            0.0;


    private double crabReturnStartEncoder =
            0.0;


    private double passStartForward =
            0.0;


    /*
     * NEW:
     *
     * Used for the additional forward clearance
     * after the obstacle disappears from the side.
     */
    private double clearanceStartForward =
            0.0;


    // =====================================================
    // COOLDOWN
    // =====================================================

    private double ignoreObstacleUntil =
            0.0;


    // =====================================================
    // TIMER
    // =====================================================

    private final Timer timer =
            new Timer();

    private static final double TOTAL_TIMEOUT =
            25.0;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public NavigateOneMeter()
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


        headingPID.setTolerance(
                1.5
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


        state =
                State.FORWARD;


        obstacleCounter =
                0;

        frontClearCounter =
                0;

        sideClearCounter =
                0;


        crabDirection =
                0;


        crabStartEncoder =
                0.0;

        crabOutDistance =
                0.0;

        crabReturnStartEncoder =
                0.0;

        passStartForward =
                0.0;

        clearanceStartForward =
                0.0;


        timer.reset();

        timer.start();


        ignoreObstacleUntil =
                0.50;


        System.out.println(
                "================================"
        );

        System.out.println(
                "NAVIGATION START"
        );

        System.out.println(
                "================================"
        );
    }


    // =====================================================
    // MAIN LOOP
    // =====================================================

    @Override
    public void execute()
    {
        switch (state)
        {
            case FORWARD:

                runForward();

                break;


            case CRAB_OUT:

                runCrabOut();

                break;


            case PASS_OBSTACLE:

                runPassObstacle();

                break;


            case CLEARANCE_FORWARD:

                runClearanceForward();

                break;


            case CRAB_BACK:

                runCrabBack();

                break;


            case FINISHED:

            case FAILED:

                drive.holonomicDrive(
                        0.0,
                        0.0,
                        0.0
                );

                break;
        }
    }


    // =====================================================
    // NORMAL FORWARD
    // =====================================================

    private void runForward()
    {
        double forwardDistance =
                Math.abs(
                        drive
                                .getAverageForwardEncoderDistance()
                );


        double remaining =
                TARGET_FORWARD_MM
                -
                forwardDistance;


        // =================================================
        // 1 METRE REACHED
        // =================================================

        if (remaining <= 15.0)
        {
            stopRobot();


            state =
                    State.FINISHED;


            System.out.println(
                    "1 METRE REACHED"
            );


            return;
        }


        // =================================================
        // FRONT OBSTACLE
        // =================================================

        double front =
                lidar.getFrontDistance();


        boolean obstacle =
                front >= MIN_VALID_LIDAR
                &&
                front <= OBSTACLE_DISTANCE;


        if (
            timer.get()
            <
            ignoreObstacleUntil
        )
        {
            obstacleCounter =
                    0;
        }
        else
        {
            if (obstacle)
            {
                obstacleCounter++;
            }
            else
            {
                obstacleCounter =
                        0;
            }
        }


        // =================================================
        // REAL OBSTACLE CONFIRMED
        // =================================================

        if (
            obstacleCounter
            >=
            OBSTACLE_CONFIRM_LOOPS
        )
        {
            stopRobot();


            chooseSide();


            crabStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            frontClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.CRAB_OUT;


            System.out.println(
                    "OBSTACLE CONFIRMED"
            );


            return;
        }


        // =================================================
        // STRAIGHT FORWARD
        // =================================================

        double ySpeed;


        if (
            remaining
            <=
            FINAL_SLOW_DISTANCE
        )
        {
            ySpeed =
                    FINAL_FORWARD_SPEED;
        }
        else
        {
            ySpeed =
                    FORWARD_SPEED;
        }


        drive.holonomicDrive(
                0.0,
                ySpeed,
                getHeadingCorrection()
        );
    }


    // =====================================================
    // CHOOSE SIDE
    // =====================================================

    private void chooseSide()
    {
        double left =
                lidar.getLeftDistance();


        double right =
                lidar.getRightDistance();


        System.out.println(
                "LEFT = "
                + left
        );


        System.out.println(
                "RIGHT = "
                + right
        );


        if (right > left)
        {
            crabDirection =
                    RIGHT_X_SIGN;


            System.out.println(
                    "AVOID RIGHT"
            );
        }
        else
        {
            crabDirection =
                    -RIGHT_X_SIGN;


            System.out.println(
                    "AVOID LEFT"
            );
        }
    }


    // =====================================================
    // CRAB OUT
    // =====================================================

    private void runCrabOut()
    {
        double currentStrafe =
                drive
                        .getAverageStrafeEncoderDistance();


        double crabDistance =
                Math.abs(
                        currentStrafe
                        -
                        crabStartEncoder
                );


        // =================================================
        // MAXIMUM CRAB SAFETY
        // =================================================

        if (
            crabDistance
            >=
            MAX_CRAB_DISTANCE
        )
        {
            stopRobot();


            state =
                    State.FAILED;


            System.out.println(
                    "CRAB LIMIT REACHED"
            );


            return;
        }


        // =================================================
        // FRONT CLEAR
        // =================================================

        double front =
                lidar.getFrontDistance();


        boolean frontClear =
                front > FRONT_CLEAR_DISTANCE
                ||
                front == 9999.0;


        /*
         * IMPORTANT:
         *
         * Must move AT LEAST 300 mm sideways.
         */
        if (
            crabDistance
            >=
            MIN_CRAB_DISTANCE
            &&
            frontClear
        )
        {
            frontClearCounter++;
        }
        else
        {
            frontClearCounter =
                    0;
        }


        // =================================================
        // READY TO GO FORWARD
        // =================================================

        if (
            frontClearCounter
            >=
            CLEAR_CONFIRM_LOOPS
        )
        {
            stopRobot();


            crabOutDistance =
                    crabDistance;


            passStartForward =
                    getForwardDistance();


            sideClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.PASS_OBSTACLE;


            System.out.println(
                    "CRAB OUT COMPLETE"
            );


            System.out.println(
                    "CRAB DISTANCE = "
                    + crabOutDistance
            );


            return;
        }


        // =================================================
        // YOUR WORKING CRAB
        // =================================================

        drive.holonomicDrive(
                CRAB_SPEED
                        *
                        crabDirection,

                0.0,

                -CRAB_CORRECTION
                        *
                        crabDirection
        );
    }


    // =====================================================
    // DRIVE ALONGSIDE OBSTACLE
    // =====================================================

    private void runPassObstacle()
    {
        double currentForward =
                getForwardDistance();


        double passed =
                currentForward
                -
                passStartForward;


        // =================================================
        // FRONT SAFETY
        // =================================================

        double front =
                lidar.getFrontDistance();


        boolean frontBlocked =
                front >= MIN_VALID_LIDAR
                &&
                front <= OBSTACLE_DISTANCE;


        if (frontBlocked)
        {
            obstacleCounter++;
        }
        else
        {
            obstacleCounter =
                    0;
        }


        /*
         * If still blocked ahead,
         * move farther sideways.
         */
        if (
            obstacleCounter
            >=
            OBSTACLE_CONFIRM_LOOPS
        )
        {
            stopRobot();


            crabStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            frontClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.CRAB_OUT;


            System.out.println(
                    "FRONT BLOCKED - CRAB FARTHER"
            );


            return;
        }


        // =================================================
        // FIND OBSTACLE ON SIDE
        // =================================================

        double sideDistance;


        if (
            crabDirection
            ==
            RIGHT_X_SIGN
        )
        {
            /*
             * Robot went right.
             *
             * Obstacle is on left.
             */
            sideDistance =
                    lidar.getLeftDistance();
        }
        else
        {
            /*
             * Robot went left.
             *
             * Obstacle is on right.
             */
            sideDistance =
                    lidar.getRightDistance();
        }


        boolean sideClear =
                sideDistance > SIDE_CLEAR_DISTANCE
                ||
                sideDistance == 9999.0;


        // =================================================
        // SIDE CLEAR CONFIRMATION
        // =================================================

        if (
            passed
            >=
            MIN_PASS_DISTANCE
            &&
            sideClear
        )
        {
            sideClearCounter++;
        }
        else
        {
            sideClearCounter =
                    0;
        }


        // =================================================
        // IMPORTANT:
        // DO NOT CRAB BACK YET
        // =================================================

        if (
            sideClearCounter
            >=
            CLEAR_CONFIRM_LOOPS
        )
        {
            /*
             * Obstacle has disappeared from side,
             * BUT the rear wheel might still be
             * close to it.
             *
             * So start another straight-forward
             * clearance section.
             */

            clearanceStartForward =
                    currentForward;


            sideClearCounter =
                    0;


            state =
                    State.CLEARANCE_FORWARD;


            System.out.println(
                    "SIDE CLEAR"
            );


            System.out.println(
                    "NOW ADDING EXTRA BODY CLEARANCE"
            );


            return;
        }


        // =================================================
        // KEEP FORWARD
        // =================================================

        drive.holonomicDrive(
                0.0,
                FORWARD_SPEED,
                getHeadingCorrection()
        );
    }


    // =====================================================
    // EXTRA BODY / WHEEL CLEARANCE
    // =====================================================

    private void runClearanceForward()
    {
        double currentForward =
                getForwardDistance();


        double extraDistance =
                currentForward
                -
                clearanceStartForward;


        // =================================================
        // STILL CHECK FRONT
        // =================================================

        double front =
                lidar.getFrontDistance();


        boolean frontBlocked =
                front >= MIN_VALID_LIDAR
                &&
                front <= OBSTACLE_DISTANCE;


        /*
         * Don't drive into another obstacle
         * while doing the clearance movement.
         */
        if (frontBlocked)
        {
            stopRobot();


            crabStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            frontClearCounter =
                    0;


            state =
                    State.CRAB_OUT;


            System.out.println(
                    "NEW FRONT OBSTACLE"
            );


            return;
        }


        // =================================================
        // EXTRA 250 mm COMPLETED
        // =================================================

        if (
            extraDistance
            >=
            EXTRA_CLEARANCE_DISTANCE
        )
        {
            stopRobot();


            crabReturnStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            state =
                    State.CRAB_BACK;


            System.out.println(
                    "BODY CLEARANCE COMPLETE"
            );


            System.out.println(
                    "NOW CRAB BACK"
            );


            return;
        }


        // =================================================
        // KEEP STRAIGHT
        // =================================================

        drive.holonomicDrive(
                0.0,
                FORWARD_SPEED,
                getHeadingCorrection()
        );
    }


    // =====================================================
    // CRAB BACK TO ORIGINAL TRACK
    // =====================================================

    private void runCrabBack()
    {
        double currentStrafe =
                drive
                        .getAverageStrafeEncoderDistance();


        double returned =
                Math.abs(
                        currentStrafe
                        -
                        crabReturnStartEncoder
                );


        // =================================================
        // BACK TO TRACK
        // =================================================

        if (
            returned
            >=
            crabOutDistance
            -
            10.0
        )
        {
            stopRobot();


            state =
                    State.FORWARD;


            obstacleCounter =
                    0;

            frontClearCounter =
                    0;

            sideClearCounter =
                    0;


            headingPID.reset();


            ignoreObstacleUntil =
                    timer.get()
                    +
                    0.75;


            System.out.println(
                    "BACK ON ORIGINAL TRACK"
            );


            return;
        }


        // =================================================
        // REVERSE CRAB
        // =================================================

        int returnDirection =
                -crabDirection;


        drive.holonomicDrive(
                CRAB_SPEED
                        *
                        returnDirection,

                0.0,

                -CRAB_CORRECTION
                        *
                        returnDirection
        );
    }


    // =====================================================
    // FORWARD DISTANCE
    // =====================================================

    private double getForwardDistance()
    {
        return Math.abs(
                drive
                        .getAverageForwardEncoderDistance()
        );
    }


    // =====================================================
    // HEADING CORRECTION
    // =====================================================

    private double getHeadingCorrection()
    {
        double correction =
                headingPID.calculate(
                        drive.getYaw(),
                        0.0
                );


        return MathUtil.clamp(
                correction,
                -MAX_HEADING_CORRECTION,
                MAX_HEADING_CORRECTION
        );
    }


    // =====================================================
    // STOP
    // =====================================================

    private void stopRobot()
    {
        drive.holonomicDrive(
                0.0,
                0.0,
                0.0
        );
    }


    // =====================================================
    // FINISHED
    // =====================================================

    @Override
    public boolean isFinished()
    {
        if (
            state == State.FINISHED
            ||
            state == State.FAILED
        )
        {
            return true;
        }


        if (
            timer.get()
            >=
            TOTAL_TIMEOUT
        )
        {
            System.out.println(
                    "NAVIGATION TIMEOUT"
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


        stopRobot();


        System.out.println(
                "================================"
        );


        System.out.println(
                "NAVIGATION END"
        );


        System.out.println(
                "FORWARD DISTANCE = "
                + getForwardDistance()
                + " mm"
        );


        System.out.println(
                "YAW = "
                + drive.getYaw()
        );


        System.out.println(
                "================================"
        );
    }
}