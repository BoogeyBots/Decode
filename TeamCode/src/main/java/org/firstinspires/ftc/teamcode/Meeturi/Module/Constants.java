package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double poz;
        public static double jos = 0.95, sus = 0.55;
    }
    @Configurable
    public static class outtake {
        public static double kp = 0, ki = 0, kd = 0;
        public static double ks = 0, kv = 1, ka = 0;
        public static double target_velocity;
        public static double pos_servo;
        public static double blocat = 0.87, deblocat = 0.35;

        public double velocity;

    }
    @Configurable
    public static class turret {
        public static double targetHeading = 0, currentHeading, error, kp = 0.5, deadband = 0.01, power; //deadband = 0.02
    }
}
