package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PinpointModule {
    HardwareMap hardwareMap;
    public PinpointModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    GoBildaPinpointDriver pinpoint;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }


}
