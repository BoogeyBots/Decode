package org.firstinspires.ftc.teamcode.Meeturi.Auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class Auto1 extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    static TelemetryManager panelsTelemetry;
    boolean turretActivated = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double x_startPose = 89.94, y_startPose = 133.77, heading_startPose = 270;
    public static double x_preload = 89.94, y_preload = 86, heading_preload = 270;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose));
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload));

    private Path scorePreload;
    private PathChain move1;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                turretActivated = true;
                follower.followPath(scorePreload, true);
                setPathState(1);

                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init_auto();
        outtake.init_auto_aproape();
        turret.init_auto();

        buildPaths();
        follower.setStartingPose(startPose);


        //follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        turret.update_auto(Math.toDegrees(follower.getPose().getHeading()));
        outtake.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", 180 + Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("error", turret.getErrore());
        telemetry.addData("power", turret.getPower());
        telemetry.addData("kp", turret.getkP());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void init_loop() {}

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
