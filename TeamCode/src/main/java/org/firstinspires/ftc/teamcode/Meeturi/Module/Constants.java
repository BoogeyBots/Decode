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
        public static double kp = 0.0087, ki = 0, kd = 0, kf = 0;
        public static double ks = 0.12, kv = 0.000553144, ka = 0.005; // ka = 0.005
        public static double target_velocity, velocity;
        public static int zone;
        public static double blocat = 0.21, deblocat = 0.9;
        public static double aproape = 0.05, departe = 0.5;
        public static boolean act_outtake, auto, ramp;
        public static double voltage, nominalvoltage = 10.7;
        public static double factor_corectie = 5; //de tunat
        public static double final_target;

    }
    @Configurable
    public static class turret {
        public static double kp = 0.013, ki, kd = 0.0008, power, error, gr;
        public static double TICKS_PER_DEGREE = 121.3629, relative_angle, decalation = 0;
        public static boolean act_turret;
        public static double constanta_inertie = 2; //de tunat
        public static double virtual_distance;
        public static double frecari = 0.37; //de tunat
        public static double timp_aer;


    }

    public static class pinpoint {
        public static double currentHeading, currentX, currentY, distanta, deltaX, deltaY, velocityX, velocityY;
    }
}
