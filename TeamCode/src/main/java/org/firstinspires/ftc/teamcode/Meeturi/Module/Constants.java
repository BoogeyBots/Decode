package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double poz;
        public static double jos = 1, sus = 0.83;
    }
    @Configurable
    public static class outtake {
        public static double kp = 0.001, ki = 0, kd = 0, kf = 0.0005;
        public static double ks = 0, kv = 1, ka = 0;
        public static double target_velocity, velocity;
        public static double pos_servo;
        public static double blocat = 0.55, deblocat = 1;
        public static double aproape = 0.05, departe = 0.5;
        public static boolean activated;

    }
    @Configurable
    public static class turret {
        public static double kp = 0.04, ki, kd = 0.0007, power, error;
        public static double TICKS_PER_DEGREE = 121.3629, relative_angle, targetRelativeAngle;

    }

    public static class pinpoint {
        public static double currentHeading, currentX, currentY, distanta, deltaX, deltaY;
    }
}
