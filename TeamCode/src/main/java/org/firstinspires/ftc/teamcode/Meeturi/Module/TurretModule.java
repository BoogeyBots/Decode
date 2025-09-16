package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TurretModule extends Constants.Turret {
    public TurretModule (HardwareMap hardwareMap) {
        init(hardwareMap);
    }
    GoBildaPinpointDriver pinpoint;
    CRServo s1, s2;

    public enum TurretCases {
        FOLLOWING,
        CONTROLLED
    }
    public TurretCases state = null;

    public void init(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");

        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        state = TurretCases.CONTROLLED;
    }

    public Actions actions;
    public class Actions {
        public void update() {
            pinpoint.update();
            currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
            error = targetHeading - currentHeading;
    
            /*
            Daca oscileaza, implementam "banda moarta"
            if(Math.abs(error) < deadband) {
                s1.setPower(0);
                s2.setPower(0);
                return;
            }
    
             */
    
            power = kp * error;
            s1.setPower(power);
            s2.setPower(power);
        }
    
    
        //pentru calibrare
    
        public void calibrare(double p, double d) {
            kp = p;
            deadband = d;
        }
    
        public double getHeading() {
            return currentHeading;
        }
    
        public double getError() {
            return error;
        }
    
        public double getPower() {
            return power;
        }
    
        public double getkP() {
            return kp;
        }
    }


}
