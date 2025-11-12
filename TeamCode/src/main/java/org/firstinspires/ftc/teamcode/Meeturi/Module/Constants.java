package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double poz;
        public static double jos = 0.56, sus = 0;
    }
    @Configurable
    public static class outtake {
        //public static double kp = 0, ki = 0, kd = 0;
        public static double ks = 0, kv = 1, ka = 0;
        public static double target_velocity;
        public static double pos_servo;
        public static double blocat = 0.5, deblocat = 1;
        public static double aproape = 0.05, departe = 0.5;

    }
    @Configurable
    public static class turret {
        public static double targetHeading = 230, currentHeading = 0, kp = 0.04, ki, kd = 0.0007, deadband = 0, power, error; //deadband = 0.02
    }
}
