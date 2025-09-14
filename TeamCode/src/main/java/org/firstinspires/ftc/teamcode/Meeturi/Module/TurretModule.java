package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TurretModule {
    HardwareMap hardwareMap;
    public TurretModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    GoBildaPinpointDriver pinpoint;
    CRServo s1, s2;
    double targetHeading = 0, currentHeading, error, kp = 0.5;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");

        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        error = targetHeading - currentHeading;
        double power = kp * error;
        s1.setPower(power);
        s2.setPower(power);
    }
}
