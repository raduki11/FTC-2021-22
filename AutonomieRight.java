package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "AUTONOMUS_RIGHT_SIDE", group = "LinearOpMode")
@Config
public class AutonomieRight extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo claw=null;
    int sleeveCase = 2;

    SampleMecanumDrive drive;
    private double CLOSED_POS = 0, OPEN_POS = 0.8; //claw stuff
    private int urca=1;
    private double p = 0, d = 0, f = 0, current_time = 0, current_error = 0, previous_time = 0, previous_error = 0, desired_position = 0, current_position = 0,
            Kp = 0.0165, Kd = 0,
            Kp2 = 0.0001, Kd2 = 0.0001, Kf2 = 0.115; //pid stuff
    // pipeline stuff down
    public static double pipeP = 0.005, pipeD = 0;
    public static double tolerance = 5, now_time = 0, now_error = 0, last_time = 0, last_error = 0, targetCenter = 150, now_position = 0;
    // pipeline stuff up
    private int i=5,normalizing=0;
    private double powArm = 0, HIGHJJ = 200, GEAR_RATIO = 25.0/10, offSet = 18, currAngle = 0, EC_PER_REV = 288;

    private enum State
    {
        TRAJ1,
        GOTOTRAJ2,
        GOTOTRAJ3,
        GOTOTRAJ4,
        GOTOTRAJ5,
        GOTOTRAJ6,
        ALIGNTOPOLE,
        DROP,
        INTAKE,
        PARKFS,
        PARKSC,
        TURN,
        IDLE
    };
    double waitTimeToDrop = 0.4;
    ElapsedTime waitTimerToDrop = new ElapsedTime();

    double waitTimeToIntake = 0.6;
    ElapsedTime waitTimerToIntake = new ElapsedTime();

    double Turn=Math.toRadians(-90);

    State driveState = State.TRAJ1;
    Pose2d startPos = new Pose2d(35,-65,Math.toRadians(90));
    Pose2d ConeStack = new Pose2d(55,-12,Math.toRadians(0));
    Trajectory traj1, traj2, traj3, traj4, traj5, traj6, park, park1, park2, park3;

    OpenCvWebcam webcam1,webcam2;
    KevinGodPipelineAprilTag pipeline;
    KevinGodPipelineAprilTag.ParkPos parkPos;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "C");
        leftArm  = hardwareMap.get(DcMotor.class, "AL");
        rightArm = hardwareMap.get(DcMotor.class, "AR");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive=new SampleMecanumDrive(hardwareMap);
        traj1 = drive.trajectoryBuilder(startPos)
                        .forward(51)
                        .build();
        traj2 = drive.trajectoryBuilder(traj1.end())
                //.lineToConstantHeading(new Vector2d(23,-13))
                .strafeLeft(15)
                .build();
        //traj3 = //drive.trajectoryBuilder(traj2.end()) //x
                //.build();
        traj4 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0,0,-90)))
                .lineToLinearHeading(ConeStack)
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(21.5,-14,Math.toRadians(90)))
                .build();
        //traj6 = drive.trajectoryBuilder(traj4.end()) //x
                //.forward(2)
               // .build();
        park = drive.trajectoryBuilder(traj5.end())
                .back(2)
                .build();
        park1 = drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(12,-12))
                .build();
        park2 = drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(36,-12))
                .build();
        park3 = drive.trajectoryBuilder(park.end())
                .lineToConstantHeading(new Vector2d(58,-12))
                .build();
        timer.reset();

       // WebcamName webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        WebcamName webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName1, cameraMonitorViewId);
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName1, cameraMonitorViewId);
        pipeline = new KevinGodPipelineAprilTag(telemetry, KevinGodPipelineAprilTag.AutoSide.RED_RIGHT,false);
        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.SLEEVE);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.setPipeline(pipeline);
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        /*
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.setPipeline(pipeline);
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened

            }
        });
        */
        telemetry.setMsTransmissionInterval(50);
        FtcDashboard.getInstance().startCameraStream(webcam2,100);
        telemetry.addData("Initialize done","da");
    }

    @Override
    public void init_loop() {
        GetCase();
        telemetry.addData("Caz nebunatic", sleeveCase);
        telemetry.update();
    }

    @Override
    public void start() {
        GetCase();
        FtcDashboard.getInstance().startCameraStream(webcam2,100);
        claw.setPosition(CLOSED_POS);
        drive.setPoseEstimate(startPos);
        drive.followTrajectoryAsync(traj1);
    }

    @Override
    public void loop() {
        drive.update();
        switch (driveState){
            case TRAJ1:
                if (!drive.isBusy())
                {
                    drive.followTrajectoryAsync(traj2);
                    driveState = State.GOTOTRAJ2;
                    desired_position = HIGHJJ;
                }
                break;
            case GOTOTRAJ2:
                if (!drive.isBusy())
                {
                    driveState = State.ALIGNTOPOLE;
                    pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);
                    webcam1.setPipeline(pipeline);
                    normalizing=1;
                }
                break;
            case ALIGNTOPOLE:
            if(now_position >= targetCenter-tolerance && now_position <= targetCenter+tolerance)
            {
                driveState=State.DROP;
                pipeline.changeMode(KevinGodPipelineAprilTag.Mode.BLUECONE);
                webcam1.setPipeline(pipeline);
                normalizing=0;
                claw.setPosition(OPEN_POS);
            }
            break;
            case DROP:
                if(i == 2){
                    driveState = State.PARKFS;
                    desired_position = 5;
                    drive.followTrajectoryAsync(park);
                }
                else
                if (!drive.isBusy()){
                    driveState = State.TURN;
                    drive.turnAsync(Turn);
                }
                break;

            case GOTOTRAJ3:
                if (!drive.isBusy())
                {
                    driveState = State.GOTOTRAJ4;
                    traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(ConeStack)
                            .build();
                    drive.followTrajectoryAsync(traj4);
                }
                break;
            case GOTOTRAJ4:
                if (!drive.isBusy())
                {
                    driveState = State.INTAKE;
                    waitTimerToIntake.reset();
                    claw.setPosition(CLOSED_POS);
                }
                break;
            case INTAKE:
                if(waitTimerToIntake.seconds() >= waitTimeToIntake){
                    driveState = State.GOTOTRAJ2;
                    i--;
                    drive.followTrajectoryAsync(traj2);
                    desired_position = HIGHJJ;
                }
                break;
            case TURN:
            if(!drive.isBusy())
            {
                driveState=State.GOTOTRAJ4;
                drive.followTrajectoryAsync(traj4);
                desired_position=6*i;
            }
            case PARKFS:
                if (!drive.isBusy())
                {
                    driveState = State.PARKSC;
                }
                break;
            case PARKSC:
                if(!drive.isBusy()) {
                    switch (sleeveCase) {
                        case 1:
                            drive.followTrajectoryAsync(park1);
                            break;
                        case 2:
                            drive.followTrajectoryAsync(park2);
                            break;
                        case 3:
                            drive.followTrajectoryAsync(park3);
                            break;
                    }
                    driveState = State.IDLE;
                    desired_position = 5;
                }
                break;
            case IDLE:
                break;
        }
        current_position = leftArm.getCurrentPosition();
        powArm = getPDFArm();
        leftArm.setPower(powArm);
        rightArm.setPower(powArm);
        drive.update();
        FtcDashboard.getInstance().startCameraStream(webcam1,120);
        if(normalizing==1) {
            double x = getPDCamera();
            drive.setMotorPowers(-x, -x, x, x);
        }
        telemetry.addData("Pozitie", pipeline.getXContour());
        telemetry.addData("Target Position", desired_position);
        telemetry.addData("Current Position", current_position);
        telemetry.update();
    }

    private double getPDCamera()
    {
        now_position = pipeline.getXContour();
        double output = 0;
        if (now_position >= targetCenter-tolerance && now_position <= targetCenter+tolerance)
            return 0;
        double pP = 0, pD = 0;
        now_time = timer.seconds();
        now_error = targetCenter - now_position;
        pP = pipeP * now_error;
        pD = pipeD * (now_error-last_error)/(now_time-last_time);
        last_time = now_time;
        last_error = now_error;
        output = pP + pD;
        return output;
    }

    private double getPDFArm()
    {
        double localp = Kp2, locald = Kd2, localf = Kf2;
        double output;
        current_time = timer.seconds();
        current_error = desired_position - current_position;
        if (desired_position < current_position)
            urca = 0;
        else
            urca = 1;
        if(urca==1)
        {
            localp=Kp;
            locald=Kd;
        }
        currAngle = Math.toDegrees(((1.0*current_position)/EC_PER_REV)*GEAR_RATIO) - offSet;
        p = localp * current_error;
        d = locald * (current_error-previous_error)/(current_time-previous_time);
        f = localf * Math.cos(Math.toRadians(currAngle));
        previous_time = current_time;
        previous_error = current_error;
        output = p + d + f;
        return output;
    }

    private void GetCase() {
        parkPos = pipeline.getPosition();
        if (parkPos == KevinGodPipelineAprilTag.ParkPos.LEFT)
            sleeveCase = 1;
        else if (parkPos == KevinGodPipelineAprilTag.ParkPos.CENTER)
            sleeveCase = 2;
        else if (parkPos == KevinGodPipelineAprilTag.ParkPos.RIGHT)
            sleeveCase = 3;
    }

}