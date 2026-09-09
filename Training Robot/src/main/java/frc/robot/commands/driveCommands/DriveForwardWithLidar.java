package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetPrinter_v2;
import frc.robot.RobotContainer;

public class DriveForwardWithLidar extends CommandBase {
    private final double targetDistance;
    private final double targetHeading;
    private final double safetyStopDistance;
    
    private final PIDController pidYAxis;
    private final PIDController pidZAxis;

    /**
     * @param distance_mm Distance to drive forward
     * @param heading_deg Angle to maintain while driving
     * @param stopThreshold_mm Distance at which the LIDAR triggers an emergency stop
     */
    public DriveForwardWithLidar(double distance_mm, double heading_deg, double stopThreshold_mm) {
        this.targetDistance = distance_mm;
        this.targetHeading = heading_deg;
        this.safetyStopDistance = stopThreshold_mm;
        
        addRequirements(RobotContainer.driveTrain);

        pidYAxis = new PIDController(0.03, 0.0, 0.0);
        pidZAxis = new PIDController(0.01, 0.0, 0.0);
        
        pidYAxis.setTolerance(15.0, 10.0); 
    }

    @Override
    public void initialize() {
        RobotContainer.driveTrain.resetEncoders();
    }

    @Override
    public void execute() {
        double speedY = MathUtil.clamp(
            pidYAxis.calculate(RobotContainer.driveTrain.getAverageForwardEncoderDistance(), targetDistance), 
            -0.5, 0.5
        );
        
        double speedZ = MathUtil.clamp(
            pidZAxis.calculate(RobotContainer.driveTrain.getYaw(), targetHeading), 
            -0.5, 0.5
        );
        
        RobotContainer.driveTrain.holonomicDrive(0.0, speedY, speedZ);
    }

    @Override
    public boolean isFinished() {
        boolean atDistance = pidYAxis.atSetpoint();

        // Read distance straight ahead from your Lidar class
        double frontObstacleDist = RobotContainer.lidar.getDistanceAtAngle(0);
        
        // Trigger stop if object is detected closer than the threshold (and ignore 0.0 errors)
        boolean obstacleFound = (frontObstacleDist > 0.0) && (frontObstacleDist < safetyStopDistance);

        if (obstacleFound) {
            NetPrinter_v2.printf("LidarLog", "STOP: Obstacle detected at %.1f mm", frontObstacleDist);
        }

        return atDistance || obstacleFound;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.holonomicDrive(0.0, 0.0, 0.0);
    }
}