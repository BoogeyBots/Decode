package org.firstinspires.ftc.teamcode.Meeturi.Module;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointModule extends Constants.pinpoint {
    HardwareMap hardwareMap;
    public PinpointModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    GoBildaPinpointDriver pinpoint;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        pinpoint.setOffsets(-5.087, -0.417, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    }

    public void update_red() {
        pinpoint.update();

        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);

        deltaX = 144 - currentX;
        deltaY = 144 - currentY;

        distanta = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    }

    public void update_blue() {
        pinpoint.update();

        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);

        velocityX = pinpoint.getVelX(DistanceUnit.INCH);
        velocityY = pinpoint.getVelY(DistanceUnit.INCH);

        deltaY = 144 - currentY;

        distanta = Math.sqrt((0 - currentX) * (0 - currentX) + deltaY * deltaY);

    }

    public void reset_blue() {
//        pinpoint.setPosX(18.1, DistanceUnit.INCH);
//        pinpoint.setPosY(78.2, DistanceUnit.INCH);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 18.1, 78.2, AngleUnit.DEGREES, 0));
    }

//    public void recalibration() {
//        LLResult result = camera.getLatestResult();
//        if(result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, botpose.getPosition().y * 39.3701 + 72, 72 - botpose.getPosition().x * 39.3701, AngleUnit.DEGREES, currentHeading));
//        }
//    }
}
