package frc.robot.commands.driveCommands;

// Use the 2020/2021 WPILib import paths
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnToAnglePrimitive extends CommandBase {
    private final double targetHeading;
    private final PIDController pidZAxis;

    public TurnToAnglePrimitive(double heading_deg) {
        this.targetHeading = heading_deg;
        addRequirements(RobotContainer.driveTrain);

        pidZAxis = new PIDController(0.03, 0.0, 0.0);
        pidZAxis.enableContinuousInput(-180.0, 180.0); 
        pidZAxis.setTolerance(2.0); 
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedZ = MathUtil.clamp(pidZAxis.calculate(RobotContainer.driveTrain.getYaw(), targetHeading), -0.5, 0.5);
        RobotContainer.driveTrain.holonomicDrive(0.0, 0.0, speedZ);
    }

    @Override
    public boolean isFinished() {
        return pidZAxis.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveTrain.holonomicDrive(0.0, 0.0, 0.0);
    }
}