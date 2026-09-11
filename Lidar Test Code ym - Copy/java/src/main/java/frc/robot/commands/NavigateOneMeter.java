package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            2000.0;


    // =====================================================
    // FORWARD SPEED
    // =====================================================

    private static final double FORWARD_SPEED =
            0.35;

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
     * Working crab correction:
     *
     * +X -> -0.05
     * -X -> +0.05
     */
    private static final double CRAB_CORRECTION =
            0.05;


    /*
     * +X physically moves right.
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
    // CLEARANCE
    // =====================================================

    private static final double MIN_CRAB_DISTANCE =
            300.0;


    private static final double MAX_CRAB_DISTANCE =
            700.0;


    private static final double MIN_PASS_DISTANCE =
            600.0;


    private static final double EXTRA_CLEARANCE_DISTANCE =
            250.0;


    // =====================================================
    // RETURN TO ORIGINAL TRACK
    // =====================================================

    /*
     * How close to original strafe encoder position
     * before we consider the robot back on track.
     */
    private static final double RETURN_TOLERANCE =
            10.0;


    /*
     * Slow down for final part of return.
     */
    private static final double RETURN_SLOW_ZONE =
            100.0;


    private static final double RETURN_SLOW_SPEED =
            0.20;


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


    /*
     * Encoder position at beginning of current
     * outward crab section.
     */
    private double crabStartEncoder =
            0.0;


    /*
     * IMPORTANT:
     *
     * Saved BEFORE the robot first moves sideways.
     *
     * This is the actual sideways track that
     * we want to return to.
     */
    private double originalTrackEncoder =
            0.0;


    /*
     * Learns how the strafe encoder responds
     * to crab motor direction.
     *
     * Example:
     *
     * command +X
     * encoder becomes negative
     *
     * then this becomes -1.
     */
    private double strafeEncoderDirection =
            1.0;


    /*
     * Used for monitoring only.
     */
    private double crabOutDistance =
            0.0;


    private double passStartForward =
            0.0;


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
            35.0;


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


        originalTrackEncoder =
                0.0;


        strafeEncoderDirection =
                1.0;


        crabOutDistance =
                0.0;


        passStartForward =
                0.0;


        clearanceStartForward =
                0.0;


        timer.reset();

        timer.start();


        ignoreObstacleUntil =
                0.50;
    }


    // =====================================================
    // MAIN LOOP
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // SMARTDASHBOARD
        // =================================================

        SmartDashboard.putString(
                "Navigation State",
                state.toString()
        );


        SmartDashboard.putNumber(
                "Front Distance",
                lidar.getFrontDistance()
        );


        SmartDashboard.putNumber(
                "Left Distance",
                lidar.getLeftDistance()
        );


        SmartDashboard.putNumber(
                "Right Distance",
                lidar.getRightDistance()
        );


        SmartDashboard.putNumber(
                "Forward Distance",
                getForwardDistance()
        );


        SmartDashboard.putNumber(
                "Current Strafe",
                drive.getAverageStrafeEncoderDistance()
        );


        SmartDashboard.putNumber(
                "Original Track",
                originalTrackEncoder
        );


        SmartDashboard.putNumber(
                "Strafe Encoder Direction",
                strafeEncoderDirection
        );


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

                stopRobot();

                break;
        }
    }


    // =====================================================
    // NORMAL FORWARD
    // =====================================================

    private void runForward()
    {
        double forwardDistance =
                getForwardDistance();


        double remaining =
                TARGET_FORWARD_MM
                -
                forwardDistance;


        // =================================================
        // FINAL TARGET
        // =================================================

        if (
            remaining
            <=
            15.0
        )
        {
            stopRobot();


            state =
                    State.FINISHED;


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
        // OBSTACLE CONFIRMED
        // =================================================

        if (
            obstacleCounter
            >=
            OBSTACLE_CONFIRM_LOOPS
        )
        {
            stopRobot();


            chooseSide();


            /*
             * SAVE THE ORIGINAL TRACK HERE.
             *
             * This value must NOT be changed
             * while avoiding the same obstacle.
             */
            originalTrackEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            crabStartEncoder =
                    originalTrackEncoder;


            crabOutDistance =
                    0.0;


            frontClearCounter =
                    0;


            obstacleCounter =
                    0;


            sideClearCounter =
                    0;


            state =
                    State.CRAB_OUT;


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


        if (
            right
            >
            left
        )
        {
            crabDirection =
                    RIGHT_X_SIGN;
        }
        else
        {
            crabDirection =
                    -RIGHT_X_SIGN;
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


        /*
         * Distance moved during this current
         * crab section.
         */
        double crabDistance =
                Math.abs(
                        currentStrafe
                        -
                        crabStartEncoder
                );


        /*
         * Total actual sideways distance
         * from original track.
         */
        double totalDistanceFromTrack =
                Math.abs(
                        currentStrafe
                        -
                        originalTrackEncoder
                );


        SmartDashboard.putNumber(
                "Distance From Original Track",
                totalDistanceFromTrack
        );


        // =================================================
        // MAXIMUM CRAB SAFETY
        // =================================================

        if (
            totalDistanceFromTrack
            >=
            MAX_CRAB_DISTANCE
        )
        {
            stopRobot();


            state =
                    State.FAILED;


            return;
        }


        // =================================================
        // LEARN STRAFE ENCODER DIRECTION
        // =================================================

        /*
         * Once robot has moved enough sideways,
         * determine which way encoder changes
         * for the current crab command.
         */
        double encoderChange =
                currentStrafe
                -
                crabStartEncoder;


        if (
            Math.abs(encoderChange)
            >
            20.0
        )
        {
            /*
             * Example:
             *
             * crabDirection = +1
             * encoderChange = -100
             *
             * sign(-100) * +1 = -1
             */
            strafeEncoderDirection =
                    Math.signum(
                            encoderChange
                    )
                    *
                    crabDirection;
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
        // READY TO PASS OBSTACLE
        // =================================================

        if (
            frontClearCounter
            >=
            CLEAR_CONFIRM_LOOPS
        )
        {
            stopRobot();


            /*
             * Store total displacement from
             * original track.
             */
            crabOutDistance =
                    totalDistanceFromTrack;


            passStartForward =
                    getForwardDistance();


            sideClearCounter =
                    0;


            obstacleCounter =
                    0;


            frontClearCounter =
                    0;


            state =
                    State.PASS_OBSTACLE;


            return;
        }


        // =================================================
        // WORKING CRAB
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
        // CHECK FRONT
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


        // =================================================
        // NEED TO CRAB FARTHER
        // =================================================

        if (
            obstacleCounter
            >=
            OBSTACLE_CONFIRM_LOOPS
        )
        {
            stopRobot();


            /*
             * IMPORTANT:
             *
             * Only update crabStartEncoder.
             *
             * DO NOT update originalTrackEncoder.
             *
             * We are still avoiding the same obstacle.
             */
            crabStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            frontClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.CRAB_OUT;


            return;
        }


        // =================================================
        // CHECK OBSTACLE SIDE
        // =================================================

        double sideDistance;


        if (
            crabDirection
            ==
            RIGHT_X_SIGN
        )
        {
            /*
             * Robot moved right,
             * so obstacle is on the left.
             */
            sideDistance =
                    lidar.getLeftDistance();
        }
        else
        {
            /*
             * Robot moved left,
             * so obstacle is on the right.
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
        // START EXTRA BODY CLEARANCE
        // =================================================

        if (
            sideClearCounter
            >=
            CLEAR_CONFIRM_LOOPS
        )
        {
            clearanceStartForward =
                    currentForward;


            sideClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.CLEARANCE_FORWARD;


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
        // CHECK FRONT
        // =================================================

        double front =
                lidar.getFrontDistance();


        boolean frontBlocked =
                front >= MIN_VALID_LIDAR
                &&
                front <= OBSTACLE_DISTANCE;


        /*
         * Another obstacle appeared.
         */
        if (frontBlocked)
        {
            stopRobot();


            /*
             * Again:
             * keep originalTrackEncoder unchanged.
             */
            crabStartEncoder =
                    drive
                            .getAverageStrafeEncoderDistance();


            frontClearCounter =
                    0;


            obstacleCounter =
                    0;


            state =
                    State.CRAB_OUT;


            return;
        }


        // =================================================
        // ENOUGH CLEARANCE
        // =================================================

        if (
            extraDistance
            >=
            EXTRA_CLEARANCE_DISTANCE
        )
        {
            stopRobot();


            /*
             * Recalculate actual distance from
             * original track immediately before return.
             */
            crabOutDistance =
                    Math.abs(
                            drive
                                    .getAverageStrafeEncoderDistance()
                            -
                            originalTrackEncoder
                    );


            state =
                    State.CRAB_BACK;


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


        // =================================================
        // CALCULATE ERROR TO ORIGINAL TRACK
        // =================================================

        /*
         * Example:
         *
         * Original = 0
         * Current  = -350
         *
         * Error = +350
         *
         * Therefore encoder needs to increase.
         */
        double encoderError =
                originalTrackEncoder
                -
                currentStrafe;


        double distanceFromTrack =
                Math.abs(
                        encoderError
                );


        SmartDashboard.putNumber(
                "Return Error",
                encoderError
        );


        SmartDashboard.putNumber(
                "Return Distance From Track",
                distanceFromTrack
        );


        // =================================================
        // ORIGINAL TRACK REACHED
        // =================================================

        if (
            distanceFromTrack
            <=
            RETURN_TOLERANCE
        )
        {
            finishReturn();


            return;
        }


        // =================================================
        // DETERMINE CORRECT MOTOR DIRECTION
        // =================================================

        /*
         * THIS is the important fix.
         *
         * We don't simply use:
         *
         * returnDirection = -crabDirection
         *
         * We calculate which crab command will make
         * the strafe encoder move toward home.
         */

        int returnDirection;


        if (
            encoderError
            *
            strafeEncoderDirection
            >
            0.0
        )
        {
            returnDirection =
                    1;
        }
        else
        {
            returnDirection =
                    -1;
        }


        SmartDashboard.putNumber(
                "Return Direction",
                returnDirection
        );


        // =================================================
        // SLOW NEAR ORIGINAL TRACK
        // =================================================

        double returnSpeed;


        if (
            distanceFromTrack
            <=
            RETURN_SLOW_ZONE
        )
        {
            returnSpeed =
                    RETURN_SLOW_SPEED;
        }
        else
        {
            returnSpeed =
                    CRAB_SPEED;
        }


        // =================================================
        // CRAB CORRECTION
        // =================================================

        double correctionScale =
                returnSpeed
                /
                CRAB_SPEED;


        double returnCorrection =
                -CRAB_CORRECTION
                *
                returnDirection
                *
                correctionScale;


        // =================================================
        // MOVE TOWARD HOME
        // =================================================

        drive.holonomicDrive(
                returnSpeed
                        *
                        returnDirection,

                0.0,

                returnCorrection
        );
    }


    // =====================================================
    // RETURN COMPLETE
    // =====================================================

    private void finishReturn()
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


        /*
         * Short cooldown so the same obstacle
         * does not immediately trigger again.
         */
        ignoreObstacleUntil =
                timer.get()
                +
                0.75;
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
    }
}