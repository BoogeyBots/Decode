package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    @Configurable
    public static class intake {
        public static double power;
        public static double jos = 0.87, sus = 0.75;
    }
    @Configurable
    public static class outtake {
        public static double kp = 0.0087; //kp = 0.0087
        public static double ks = 0.07, kv = 0.000446563, ka = 0.005; // ks = 0.12, kv = 0.000553144, ka = 0.005;
        public static double target_velocity, velocity;
        public static int zone;
        public static double blocat = 0.1, deblocat = 0.3;
        public static double aproape = 0.05, departe = 0.5;
        public static boolean act_outtake, auto, ramp;
        public static double voltage, nominalvoltage = 9.3;
        public static double factor_corectie = 3.5; //de tunat
        public static double final_target;

    }
    @Configurable
    public static class turret {
        public static double kp = 0.015, ki, kd = 0.0003, kf = 0.1, ks = 0.03, power, error, gr; //kp = 0.01   kd = 0.00027
        public static double kp2 = 0.02, kd2 = 0.0003, kf2 = 0.2;
        public static double TICKS_PER_DEGREE = 60.681481, relative_angle, decalation = 0, turretCurrentPos;
        public static boolean act_turret;
        public static double constanta_inertie = 2; //de tunat
        public static double virtual_distance;
        public static double frecari = 0.37; //de tunat
        public static double timp_aer;
        public static boolean trage_gresit = false;


    }
    @Configurable
    public static class pinpoint {
        public static double currentHeading, currentX, currentY, distanta, deltaX, deltaY, velocityX, velocityY;
        public static double odoY = 4.579, odoX = 4.469;   //4.87
    }
}
