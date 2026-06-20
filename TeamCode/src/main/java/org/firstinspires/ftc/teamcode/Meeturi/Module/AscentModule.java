package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AscentModule {
    HardwareMap hardwareMap;
    public AscentModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    Servo ridicare_right, ridicare_left;

    public void init() {
        ridicare_right = hardwareMap.get(Servo.class, "ridicare_right");
        ridicare_left = hardwareMap.get(Servo.class, "ridicare_left");

        ridicare_right.setPosition(1-0.3);
        ridicare_left.setPosition(0.3);
    }

    public void tilt() {
        ridicare_right.setPosition(1-0.6);
        ridicare_left.setPosition(0.6);
    }

}
