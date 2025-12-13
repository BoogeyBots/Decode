package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double poz;
        public static double jos = 0.96, sus = 0.9;
    }
    @Configurable
    public static class outtake {
        public static double kp = 0.06, ki = 0, kd = 0, kf = 0.0004;
        public static double ks = 0, kv = 1, ka = 0;
        public static double target_velocity, velocity;
        public static int zone;
        public static double blocat = 0.45, deblocat = 1;
        public static double aproape = 0.05, departe = 0.5;
        public static boolean activated, auto, ramp;

    }
    @Configurable
    public static class turret {
        public static double kp = 0.04, ki, kd = 0.0007, power, error;
        public static double TICKS_PER_DEGREE = 121.3629, relative_angle, decalation = 0;

    }

    public static class pinpoint {
        public static double currentHeading, currentX, currentY, distanta, deltaX, deltaY;
    }
}
