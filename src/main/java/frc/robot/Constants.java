package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 3;
        public static final int kLeftMotor2Port = 4;
        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 2;
        public static boolean kLeftEncoderReversed = false;
        public static boolean kRightEncoderReversed = true;
        public static final int[] kLeftEncoderPorts = new int[] {3, 4};
        public static final int[] kRightEncoderPorts = new int[] {1, 2};
        public static final  double kEncoderDistancePerPulse = 0.69;
		
        public static final double ksVolts = 0.642;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00926;
        public static final double kPDriveVel = 0.406;

        //UNITS IN METERS
        public static final double kTrackwidthMeters = 0.6858;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        
        	
        //RAMSETE constants
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

}