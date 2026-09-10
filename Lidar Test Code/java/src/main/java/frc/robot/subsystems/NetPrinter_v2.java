package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetPrinter_v2 {
    
    /**
     * Emulates printf by sending a formatted string to NetworkTables.
     * 
     * @param key    The unique NetworkTables key (e.g., "DriveLog" or "OMSLog")
     * @param format A format string (e.g., "Motor speed: %.2f")
     * @param args   Arguments referenced by the format specifiers
     */
    public static void printf(String key, String format, Object... args) {
        String message = String.format(format, args);
        
        // Prepend the FPGA timestamp
        String networkMessage = String.format("[%7.3f] %s", Timer.getFPGATimestamp(), message);
        
        // Push to the SmartDashboard table using the custom key
        SmartDashboard.putString(key, networkMessage);
    }
}