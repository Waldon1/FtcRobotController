package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "AutonLeftJunctionMid2022")
public class AutonLeftJunctionMid2022 extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotor rightfront;
    private DcMotor leftfront;
    private DcMotor leftback;
    private DcMotor rightback;
    private DigitalChannel liftencoder;
    private Servo GripperServo;
    private Servo armSwivl;
    private CRServo lift;

    int recognition;
    double CountsPerInch;
    List<Recognition> recognitions;
    int index;

    /**
     * Describe this function...
     */
    private void Cone_Delivery() {
        timeLift(0.2, 0.8);
        // strafe away from wall
        drive(0, 46.25, 0, 0.75);
        // forward to a3
        timeLift(0.5, 1);
        drive(4, 0, 0, 0.75);
        timeLift(0.33, -1);
        control_gripper(0.4);
        drive(-4, 0, 0, 0.7);
        // Move to align with high post
        drive(0, -19.25, 0, 0.5);
    }

    /**
     * Describe this function...
     */
    private void Initialize_Vuforia() {
        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useDefaultModel();
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.65, true, true);
        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(1.25, 4 / 3);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (!recognitions) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfod.getRecognitions();
                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                    callStop();
                } else {
                    index = 0;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        // Display info.
                        displayinfo(index);
                        // Increment index.
                        index = index + 1;
                        callStop();
                    }
                }
                telemetry.update();
                callStop();
            }
            callStop();
        }
        // Deactivate TFOD.
        tfod.deactivate();
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int liftTime;
        int forward;
        int strafe;
        int turn;
        int speed;
        double WheelCircomference;
        double CountsPerMotorRev;
        double linearSlideCircomference;
        double CountsPerLinearRev;
        double CountsPerLinearInch;
        int Lift;
        int liftEncoder;

        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        rightfront = hardwareMap.get(DcMotor.class, "right front");
        leftfront = hardwareMap.get(DcMotor.class, "left front");
        leftback = hardwareMap.get(DcMotor.class, "left back");
        rightback = hardwareMap.get(DcMotor.class, "right back");
        liftencoder = hardwareMap.get(DigitalChannel.class, "lift encoder");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");
        armSwivl = hardwareMap.get(Servo.class, "armSwivl");
        lift = hardwareMap.get(CRServo.class, "lift");

        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);
        rightback.setDirection(DcMotorSimple.Direction.REVERSE);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftencoder.setMode(DigitalChannel.Mode.INPUT);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelCircomference = 3.77953 * Math.PI;
        CountsPerMotorRev = 537.7;
        linearSlideCircomference = 4.4;
        CountsPerLinearRev = 145.1;
        CountsPerInch = CountsPerMotorRev / WheelCircomference;
        CountsPerLinearInch = CountsPerLinearRev / linearSlideCircomference;
        forward = 0;
        forward = 0;
        liftTime = 0;
        recognition = 0;
        strafe = 0;
        turn = 0;
        speed = 0;
        Lift = 0;
        liftEncoder = 0;
        GripperServo.setPosition(0.8);
        // 0 is home and 1 is open
        Initialize_Vuforia();
        CenterSwivel();
        displayinfo(index);
        waitForStart();
        telemetry.update();
        Cone_Delivery();
        parking();

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Describe this function...
     */
    private void CenterSwivel() {
        armSwivl.setPosition(0.47);
        sleep(450);
    }

    /**
     * Describe this function...
     */
    private void parking() {
    }

    /**
     * Describe this function...
     */
    private void callStop() {
        if (isStopRequested()) {
            terminateOpModeNow();
        }
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayinfo(int i) {
        int ParkPos;

        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        // Need to work on Park 3
        // Need to work on Park 3
        if (recognition.getLabel().equals("1 Bolt")) {
            ParkPos = 1;
        } else if (recognition.getLabel().equals("2 Bulb")) {
            ParkPos = 2;
        } else if (recognition.getLabel().equals("3 Panel")) {
            ParkPos = 3;
        } else if (JavaUtil.listLength(recognitions) == 0) {
            ParkPos = 3;
        }
        callStop();
    }

    /**
     * Describe this function...
     */
    private void RightSwivel() {
        armSwivl.setPosition(0.92);
        sleep(625);
    }

    /**
     * Describe this function...
     */
    private void timeLift(double liftTime, double liftSpeed) {
        lift.setPower(liftSpeed);
        sleep(liftTime * 1000);
        lift.setPower(0.1);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    private void LeftSwivel() {
        armSwivl.setPosition(0);
        sleep(625);
    }

    /**
     * Describe this function...
     */
    private void control_gripper(double GripperServo2) {
        GripperServo.setPosition(GripperServo2);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    private void drive(int forward, double strafe, double turn, double speed) {
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setTargetPosition((int) ((forward * 1 + strafe * -1 + turn * 1) * CountsPerInch));
        rightfront.setTargetPosition((int) ((forward * 1 - (strafe * -1 + turn * 1)) * CountsPerInch));
        leftback.setTargetPosition((int) ((forward * 1 - (strafe * -1 - turn * 1)) * CountsPerInch));
        rightback.setTargetPosition((int) ((forward * -1 + (strafe * -1 - turn * 1)) * CountsPerInch));
        leftback.setPower(speed * 0.5);
        leftfront.setPower(speed * 0.5);
        rightback.setPower(speed * 0.5);
        rightfront.setPower(speed * 0.5);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && rightback.isBusy() && rightfront.isBusy() && leftback.isBusy() && leftfront.isBusy()) {
            callStop();
        }
        sleep(450);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
