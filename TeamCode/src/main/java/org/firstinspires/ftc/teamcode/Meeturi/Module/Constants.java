package org.firstinspires.ftc.teamcode.Meeturi.Module;

public class Constants {
    static class Intake {
        public static double powerMotorTake, powerMotorEject, powerMotorOff;
    }

    static class Outtake {
        public static double powerMotorOff, powerMotorShoot;
        public static double positionServoAngleClose, positionServoAngleFar, positionServoAngleCustom; 
    }

    static class Turret {
        public static double targetHeading = 0, currentHeading, error, kp = 0.5, deadband = 0.01, power; //deadband = 0.02
    }
}
