package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.TurnWithPID;
import frc.robot.subsystems.NetPrinter_v2;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.driveCommands.SimpleDrive; 
import frc.robot.commands.driveCommands.DriveWithPID; 

public class DriveAroundObstacleWithPID extends AutoCommand {
    
    // =========================================================================
    // TUNABLE CONSTANTS (Adjust these to match your real-world arena)
    // =========================================================================
    // Lidar Thresholds (Measured in CM)
    private static final double MIN_DETECTION_CM = 10.0;
    private static final double MAX_DETECTION_CM = 80.0;
    private static final double APPROACH_STOP_CM = 30.0;  // Distance to stop in front of the box
    
    // Motor Speeds & Directions
    private static final double FORWARD_SPEED = 0.5;
    private static final double CRAB_Y_SPEED = 0.0;
    
    // Outward Crab Walk (Stepping away from centerline)
    private static final double CRAB_X_SPEED = 0.5;
    private static final double CRAB_ROT_CORRECTION = -0.05;
    
    // Return Crab Walk (Stepping back to centerline)
    private static final double RETURN_CRAB_X_SPEED = -0.5; 
    private static final double RETURN_ROT_CORRECTION = 0.05; 
    
    // Timeouts (Measured in Seconds)
    private static final double WAIT_TIME = 0.25;
    private static final double FIRST_CLEAR_TIMEOUT = 2.5;   // Time to clear robot width
    private static final double FORWARD_DRIVE_TIMEOUT = 4; // Time to drive past box length
    private static final double SECOND_CLEAR_TIMEOUT = 1.5;  // Time to clear front corner of box
    private static final double RETURN_LINE_TIMEOUT = 3;   // Time to crab walk back to centerline
    private static final double FINAL_DRIVE_TIMEOUT = 5.0;   // Final straight away duration
    // =========================================================================

    private final DriveTrain driveTrain;

    /**
     * Fully automated obstacle avoidance using 0° LiDAR distance.
     * @param driveTrain The DriveTrain subsystem instance
     */
    public DriveAroundObstacleWithPID(DriveTrain driveTrain) {
        super(new SequentialCommandGroup());
        this.driveTrain = driveTrain;
        
        NetPrinter_v2.printf("LidarLog", "LIDAR INIT ERROR: HI!!!!");

        addCommands(
            // Step 1: Run straight toward the box until the 0° LiDAR detects it within range
            new DriveWithPID(1000, 1, 0, 1) {
                @Override
                public boolean isFinished() {
                    double distance = getDistance();
                    return (distance <= APPROACH_STOP_CM && distance >= MIN_DETECTION_CM);
                }
            },
            new WaitCommand(WAIT_TIME),
            
            // Step 2: Crab walk sideways out of the way until the sensor no longer sees the object
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),
            
            // Step 3: Continue crab walking so the entire chassis clears the box width
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(FIRST_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 4: Drive forward to clear the side profile length of the box
            new DriveWithPID(1000, 1, 0, 1).withTimeout(FORWARD_DRIVE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 5: Turn 90 degrees Left to point 0° LiDAR back toward the obstacle side
            new TurnWithPID(0, 10, -90, 1),
            new WaitCommand(WAIT_TIME),

            // Step 6: Crab walk along the box profile until the 0° LiDAR clears it again
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION) {
                @Override
                public boolean isFinished() {
                    return !isObjectDetected();
                }
            },
            new WaitCommand(WAIT_TIME),

            // Step 7: Continue crab walking to safely clear the front corner of the box
            new SimpleDrive(CRAB_X_SPEED, CRAB_Y_SPEED, CRAB_ROT_CORRECTION).withTimeout(SECOND_CLEAR_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 8: Turn 90 degrees Right to face back towards the original direction (front)
            new TurnWithPID(0, 10, 90, 1),
            new WaitCommand(WAIT_TIME),

            // Step 9: Crab walk sideways using return variables to get back onto the centerline
            new SimpleDrive(RETURN_CRAB_X_SPEED, CRAB_Y_SPEED, RETURN_ROT_CORRECTION).withTimeout(RETURN_LINE_TIMEOUT),
            new WaitCommand(WAIT_TIME),

            // Step 10: Continue driving straight down the original track layout
            new DriveWithPID(1000, 1, 0, 1).withTimeout(FINAL_DRIVE_TIMEOUT)
        );
    }

    /**
     * Calculates sensor distance using the 0 degree LiDAR reading.
     * @return distance in centimeters (CM)
     */
    public double getDistance() {
        return driveTrain.getLidarAtZeroDegrees();
    }
    
    /**
     * Evaluates if an object is currently within the reliable tracking bounds of the sensor.
     * @return true if an object is actively detected between 10cm and 80cm
     */
    public boolean isObjectDetected() {
        double distance = getDistance();
        return (distance >= MIN_DETECTION_CM && distance <= MAX_DETECTION_CM);
    }
}