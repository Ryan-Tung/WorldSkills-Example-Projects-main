package frc.robot.commands.driveCommands;

// Use the 2020/2021 WPILib import paths
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardPrimitive extends CommandBase {
    private final double targetDistance;
    private final double targetHeading;
    
    private final PIDController pidYAxis;
    private final PIDController pidZAxis;

    public DriveForwardPrimitive(double distance_mm, double heading_deg) {
        this.targetDistance = distance_mm;
        this.targetHeading = heading_deg;
        
        addRequirements(RobotContainer.driveTrain);

        pidYAxis = new PIDController(0.03, 0.0, 0.0);
        pidZAxis = new PIDController(0.01, 0.0, 0.0);
        
        pidYAxis.setTolerance(15.0,10.0); 
    }

    @Override
    public void initialize() {
        
        
        RobotContainer.driveTrain.resetYaw();
        RobotContainer.driveTrain.resetEncoders();
        
    }

    @Override
    public void execute() {
        double speedY = MathUtil.clamp(pidYAxis.calculate(RobotContainer.driveTrain.getAverageForwardEncoderDistance(), targetDistance), -0.5, 0.5);
        double speedZ = MathUtil.clamp(pidZAxis.calculate(RobotContainer.driveTrain.getYaw(), targetHeading), -0.5, 34);
        
        RobotContainer.driveTrain.holonomicDrive(0.0, speedY, speedZ);
    }

    @Override
    public boolean isFinished() {
        return pidYAxis.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.holonomicDrive(0.0, 0.0, 0.0);
    }
}