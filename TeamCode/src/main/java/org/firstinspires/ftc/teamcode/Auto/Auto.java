package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
@Autonomous (group = "Blue")
public class Auto extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    private Follower follower;
    double m;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.4;
    public static double x_startPose = 118.651, y_startPose = 127.826, heading_startPose = 225;
    public static double x_preload = 86, y_preload = 83, heading_preload = 225;
    public static double x_collect1 = 127.5, y_collect1 = 84, heading_collect = 180;
    public static double x_trapa = 126, y_trapa = 70, heading_trapa = 110;
    public static double x_collect2 = 125, y_collect2 = 57;
    public static double x_collect3 = 126, y_collect3 = 35;
    public static double x_cp2 = 70.4, y_cp2 = 59;
    public static double x_cp3 = 82, y_cp3 = 29;
    public static double x_cptrapa = 109, y_cptrapa = 66;
    public static double x_afara = 115, y_afara = 83;
    public static double x_colectare_gate = 10, y_colectare_gate = 61.2, heading_colectare_gate = -32; //-32
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose)).mirror();
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload)).mirror();
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect)).mirror();
    private final Pose collect2 = new Pose(x_collect2, y_collect2, heading_collect).mirror();
    private final Pose collect3 = new Pose(x_collect3, y_collect3, heading_collect).mirror();
    private final Pose trapa = new Pose(x_trapa, y_trapa, heading_trapa).mirror();
    private final Pose cp_rand2 = new Pose(x_cp2, y_cp2).mirror();
    private final Pose cp_rand3 = new Pose(x_cp3, y_cp3).mirror();
    private final Pose cp_trapa = new Pose(x_cptrapa, y_cptrapa).mirror();
    private final Pose afara = new Pose(x_afara, y_afara).mirror();
    private final Pose colectare_gate = new Pose(x_colectare_gate, y_colectare_gate, Math.toRadians(heading_colectare_gate));
    private final Pose cp_gate = new Pose(40, 60);
    private final Pose parcare = new Pose(84.2, 104).mirror();

    private Path scorePreload;
    private PathChain rand1, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4, sprecolt2, trage5, leave, spretrapa2, gate, trage_gate;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(scorePose.getHeading());

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();

        spretrapa = follower.pathBuilder()
                .addPath(new BezierCurve(collect2, cp_trapa, trapa))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();


        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1, parcare))
                .setTangentHeadingInterpolation()
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand2, collect2))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .setNoDeceleration()
                .build();


        trage2 = follower.pathBuilder()
                .addPath(new BezierCurve(trapa, cp_rand2, scorePose))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                        new HeadingInterpolator.PiecewiseNode(
                                                0,
                                                .8,
                                                HeadingInterpolator.tangent
                                        ),
                                        new HeadingInterpolator.PiecewiseNode(
                                                .8,
                                                1,
                                                HeadingInterpolator.linear(follower.getHeading(), Math.toRadians(110))
                                        )
                        )
                )
                .build();

        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand3, collect3))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .setNoDeceleration()
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, afara))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setNoDeceleration()
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_gate, colectare_gate))
                .setLinearHeadingInterpolation(follower.getHeading(), colectare_gate.getHeading())
                .build();

        trage_gate = follower.pathBuilder()
                .addPath(new BezierLine(colectare_gate, scorePose))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                act_outtake = true;
                intake.trage_intake(0.3);
                setPathState(1);

                break;

            case 1:
                if(follower.atPose(scorePose, 5, 5)) {
                    deblocare();
                    setPathState(2);
                }

                break;

            case 2:
                if(!follower.isBusy()) {
                    trage();
                    setPathState(3);
                }

                break;

            case 3:
                if(pathTimer.getElapsedTime() > cat_trage) {
                    numaitrag();
                }

                break;

            case 4:
                follower.followPath(rand1, true);
                setPathState(5);

                break;

            case 5:
                if(!follower.isBusy()){
                    follower.followPath(trage1, true);
                    setPathState(6);
                }

                break;

            case 6:
                if(follower.atPose(scorePose, 5, 5)) {
                    deblocare();
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    trage();
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTime() > cat_trage) {
                    numaitrag();
                    setPathState(9);
                }

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

        intake.init();
        outtake.init();
        turret.init();

        buildPaths();
        follower.setStartingPose(startPose);

        auto = true;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        distanta = Math.sqrt((0 - x) * (0 - x) + (144 - y) * (144 - y));

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
        telemetry.update();

        outtake.update_auto();
        turret.update_blue_auto(x, y, h);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        target_velocity = 0;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void trage() {
        intake.trage_intake(1);
        intake.trage_transfer(1);
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 1100;
        intake.trage_transfer(0.47);
    }

    public void deblocare() {
        act_outtake = true;
        intake.stop_intake();
        intake.stop_transfer();
        outtake.deblocat();
    }
}