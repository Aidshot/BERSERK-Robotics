package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.Competition.PIDTEST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.HardwareBERSERK;
import org.firstinspires.ftc.teamcode.drive.opmode.BERSERK.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.velo.TuningController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.Arrays;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config;

@Autonomous(group = "BERSERK")
public class BLUE_FULL_AUTO_TEST extends LinearOpMode {

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 50; // horizon value to tune
    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "webcam1"; // insert webcam name from configuration if using webcam

    // DUAL MOTOR PID \\
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.003, 0, 0);
    public static double kV = 0.00039;
    public static double kA = 0.00015;
    public static double kStatic = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final ElapsedTime veloTimer = new ElapsedTime();

    TelemetryPacket packet = new TelemetryPacket();

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    enum State {
        FOUR,
        ONE,
        ZERO
    }



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot    = new HardwareBERSERK();
        robot.init(hardwareMap);

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        robot.blinkinLedDriver.setPattern(pattern);

        double foldout = 0; //SET TO -1 TO FOLDOUT INTAKE, 0 TO DISABLE

        double shooter_target_velo = 1650;
        double launch_angle = 0.652; //0.173
        double kicker_out = 0.7;
        double kicker_in = 0.25; //02
        double wobble_close = 0.18;
        double wobble_open = 0.6;
        double wobble_up = 0.3;
        double wobble_down = 0.8;
        long shootWait = 330;
        double webcam_right = 0.1;
        double webcam_left = 0.3;

        double targetVelo = 0;

        robot.webcam_servo.setPosition(webcam_right);

        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
        Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        Config.setHORIZON(HORIZON);
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        State state = State.ZERO;

        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);

        //   A AUTO TRAJECTORIES   //
        //SHOOT POSITION
        Trajectory A1 = drive.trajectoryBuilder(startPose)
                .back(3)
                .build();

        //WOBBLE A POSITION
        Trajectory A2 = drive.trajectoryBuilder(A1.end())
                .splineToLinearHeading(new Pose2d(5.0, 53.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //MOVE TO WOBBLE 2
        Trajectory A3 = drive.trajectoryBuilder(A2.end())
                .strafeRight(10)
                //   .splineTo(new Vector2d(-32.0, 24.0),Math.toRadians(80))
                //   .splineTo(new Vector2d(-32.0, 24.0),Math.toRadians(-10))
                .splineTo(new Vector2d(-20.0, 18.0),Math.toRadians(350))
                .addTemporalMarker(2.0, () -> {
                    robot.wobble_lift.setPosition(wobble_down);
                })
                //  .splineToConstantHeading(new Vector2d(-40, 25.0),Math.toRadians(80),
                .splineToConstantHeading(new Vector2d(-41.5, 20.5),Math.toRadians(80),
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                //  new MecanumVelocityConstraint(7, DriveConstants.TRACK_WIDTH)
                                new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //WOBBLE A POSITION
        Trajectory A4 = drive.trajectoryBuilder(A3.end())
                .splineToLinearHeading(new Pose2d(10.0, 44.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //PARK
        Trajectory A5 = drive.trajectoryBuilder(A4.end())
                .strafeRight(5)
                .build();

        while (!isStarted()) {
            switch (pipeline.getHeight()) {
                case ZERO:
                    state = State.ZERO;
                    break;
                case ONE:
                    state = State.ONE;
                    break;
                case FOUR:
                    state = State.FOUR;
                    break;
            }
        }

        // DUAL MOTOR PID \\
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        PIDFController veloController = new PIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        TuningController tuningController = new TuningController();

        double lastTargetVelo = 0.0;
        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        // AUTO CASE STATEMENT
        switch (state) {
            case ZERO:
                telemetry.addData("Stack:", "ZERO");
                telemetry.update();

                drive.followTrajectory(A1);

                targetVelo = 1650;

                // RAMP UP PID
                double timer = System.currentTimeMillis(); // get the current time in milliseconds
                while(System.currentTimeMillis() - timer <= 2000) { // That condition will be true after exactly 3 seconds
                    //double targetVelo = tuningController.update();

                    veloController.setTargetPosition(targetVelo);
                    veloController.setTargetVelocity(targetVelo);
                    veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
                    veloTimer.reset();

                    lastTargetVelo = targetVelo;

                    telemetry.addData("targetVelocity", targetVelo);

                    double motorVelo = myMotor1.getVelocity();

                    double power = veloController.update(motorVelo);
                    myMotor1.setPower(power);
                    myMotor2.setPower(power);

                    if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                        lastKv = kV;
                        lastKa = kA;
                        lastKstatic = kStatic;

                        veloController = new PIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
                        packet.put("FlyWheel Velocity", motorVelo);
                        telemetry.update();
                    }
                }

                //SHOOT x 3
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);

                robot.kicker.setPosition(kicker_out);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_in);
                sleep(shootWait);
                robot.kicker.setPosition(kicker_out);


                //TURN OFF SHOOTER
                ((DcMotorEx) robot.shooter1).setVelocity(0);
                ((DcMotorEx) robot.shooter2).setVelocity(0);

                PoseStorage.currentPose = drive.getPoseEstimate();

                break;
            case ONE:

                break;

            case FOUR:

                break;
        }
    }
}