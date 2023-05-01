package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "MecanumFieldCentricDrive2022")
public class MecanumFieldCentricDrive2022 extends LinearOpMode {

    private Servo GripperServo;
    private CRServo lift;
    private BNO055IMU imu;

    float Turn;
    BNO055IMU.Parameters imuParameters;
    double Forward;
    double Strafe;
    boolean Last_Value_of_Gripper_Position = false;
    double Gripper_Position2 = 0.75;

    /**
     * Describe this function...
     */
    private void callStop() {
        // this means that if said to stop, it stops
        if (isStopRequested()) {
            terminateOpModeNow();
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double denominator;

        DcMotor leftback = hardwareMap.get(DcMotor.class, "left back");
        DcMotor leftfront = hardwareMap.get(DcMotor.class, "left front");
        DcMotor rightback = hardwareMap.get(DcMotor.class, "right back");
        DcMotor rightfront = hardwareMap.get(DcMotor.class, "right front");
        Servo armSwivl = hardwareMap.get(Servo.class, "armSwivl");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");
        lift = hardwareMap.get(CRServo.class, "lift");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        init_IMU();
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSwivl.setPosition(0.47);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Drive();
                callStop();
                driveSpeed();
                Gripper_Position();
                manaulSetLIftPos();
                reset_IMU();
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(Forward) + Math.abs(Strafe + Math.abs(Turn)), 1));
                leftfront.setPower((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe) + Turn) / denominator);
                rightfront.setPower(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) - Turn) / denominator);
                rightback.setPower(((Forward * Math.abs(Forward) + Strafe * Math.abs(Strafe)) - Turn) / denominator);
                leftback.setPower(((Forward * Math.abs(Forward) - Strafe * Math.abs(Strafe)) + Turn) / denominator);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void driveSpeed() {
        // Setting the drive speed.
        if (gamepad1.left_bumper) {
            Forward = Forward * 0.9;
            Strafe = Strafe * 0.9;
            Turn = (float) (Turn * 0.9);
        } else if (gamepad1.right_bumper) {
            Forward = Forward * 0.5;
            Strafe = Strafe * 0.5;
            Turn = (float) (Turn * 0.5);
        } else {
            Forward = Forward * 0.8;
            Strafe = Strafe * 0.8;
            Turn = (float) (Turn * 0.8);
        }
    }

    /**
     * Describe this function...
     */
    private void Gripper_Position() {


        if (!Last_Value_of_Gripper_Position && gamepad2.a) {
            if (Gripper_Position2 == 0.45) {
                Gripper_Position2 = 0.75;
            } else {
                Gripper_Position2 = 0.45;
            }
        }
        Last_Value_of_Gripper_Position = gamepad2.a;
        GripperServo.setPosition(Gripper_Position2);
    }

    /**
     * Describe this function...
     */
    private void manaulSetLIftPos() {
        // Sets drive speed.
        if (gamepad2.dpad_down) {
            if (gamepad2.right_bumper) {
                lift.setPower(-0.25);
            } else {
                lift.setPower(-0.6);
            }
        } else if (gamepad2.dpad_up) {
            if (gamepad2.right_bumper) {
                lift.setPower(0.5);
            } else {
                lift.setPower(0.75);
            }
        } else {
            lift.setPower(0.1);
        }
    }

    /**
     * Describe this function...
     */
    private void Drive() {
        double Drive2;
        double GamePadDegree;
        double Movement;

        Turn = gamepad1.right_stick_x;
        Drive2 = Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), 0, 1);
        GamePadDegree = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180;
        Movement = GamePadDegree - getZAxis();
        Strafe = Math.cos(Movement / 180 * Math.PI) * Drive2;
        Forward = Math.sin(Movement / 180 * Math.PI) * Drive2;
    }

    /**
     * Describe this function...
     */
    private void init_IMU() {
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParameters);
    }

    /**
     * Describe this function...
     */
    private void reset_IMU() {
        if (gamepad1.a) {
            imuParameters = new BNO055IMU.Parameters();
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.mode = BNO055IMU.SensorMode.IMU;
            imu.initialize(imuParameters);
        }
    }

    /**
     * Describe this function...
     */
    private float getZAxis() {
        Orientation Angle;

        Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Angle.firstAngle;
    }
}
