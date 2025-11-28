package org.firstinspires.ftc.teamcode.Meeturi.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class PinpointModule extends Constants.pinpoint {
    HardwareMap hardwareMap;
    public PinpointModule (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    GoBildaPinpointDriver pinpoint;
    Limelight3A camera;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        camera = hardwareMap.get(Limelight3A.class, "camera");
        camera.pipelineSwitch(0);

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        camera.start();

    }

    public void update() {
        pinpoint.update();

        camera.updateRobotOrientation(currentHeading);

        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);

        deltaX = 144 - currentX;
        deltaY = 144 - currentY;

        distanta = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    }

    public void recalibration() {
        LLResult result = camera.getLatestResult();
        if(result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, botpose.getPosition().y * 39.3701 + 72, 72 - botpose.getPosition().x * 39.3701, AngleUnit.DEGREES, currentHeading));
        }
    }
}
