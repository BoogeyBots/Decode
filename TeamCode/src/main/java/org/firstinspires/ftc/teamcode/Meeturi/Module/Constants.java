package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double poz;
        public static double jos = 0.8, sus = 0.1;
    }
    @Configurable
    public static class outtake {
        public static double kp = 0, ki = 0, kd = 0;
        public static double ks = 0, kv = 1, ka = 0;
        public static double target_velocity;
        public static double pos_servo;
        public static double blocat = 0.4, deblocat = 0.12;
        public static double aproape = 0, departe = 0.6;

    }
    @Configurable
    public static class turret {
        public static double targetHeading = 237, currentHeading = 0, kp = 0.05, ki, kd = 0.002, deadband = 0, power, error; //deadband = 0.02
    }
}
