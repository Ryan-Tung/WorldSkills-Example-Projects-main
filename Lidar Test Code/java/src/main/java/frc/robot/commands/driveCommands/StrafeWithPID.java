// package frc.robot.commands.driveCommands;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpiutil.math.MathUtil;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.DriveTrain;

// public class StrafeWithPID extends CommandBase {
//     private static final DriveTrain drive = RobotContainer.driveTrain;

//     private double setpointDistance;
//     private double setpointYaw; 

//     private PIDController pidXAxis;
//     private PIDController pidZAxis;

//     public StrafeWithPID(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw) {
//         this.setpointDistance = setpointDistance;
//         this.setpointYaw = setpointYaw;
//         addRequirements(drive);

//         pidXAxis = new PIDController(0.01, 0.0, 0); // Tune P gain as needed for strafing
//         pidXAxis.setTolerance(epsilonDistance);

//         pidZAxis = new PIDController(0.01, 0.0, 0); // Non-zero kP allows active yaw correction
//         pidZAxis.setTolerance(epsilonYaw);
//     }

//     @Override
//     public void initialize() {
//         drive.resetEncoders();
//         drive.resetYaw();
//         pidXAxis.reset();
//         pidZAxis.reset();
//     }

//     @Override
//     public void execute() {
//         // Calculate strafe power (X axis) and rotation power (Z axis)
//         double strafeSpeed = MathUtil.clamp(
//             pidXAxis.calculate(drive.getAverageStrafeEncoderDistance(), setpointDistance), -0.5, 0.5
//         );
//         double turnSpeed = MathUtil.clamp(
//             pidZAxis.calculate(drive.getYaw(), setpointYaw), -1.0, 1.0
//         );

//         // 1st arg = Strafe (X), 2nd arg = Forward (Y = 0.0), 3rd arg = Rotation (Z)
//         drive.holonomicDrive(strafeSpeed, 0.0, turnSpeed);

//         SmartDashboard.putNumber("Yaw", drive.getYaw());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         drive.holonomicDrive(0.0, 0.0, 0.0);
//     }

//     @Override
//     public boolean isFinished() {
//         return pidXAxis.atSetpoint();
//     }
// }