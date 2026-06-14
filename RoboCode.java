import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "GoGurt")
public class RoboCode extends LinearOpMode {

    private IMU imu;

    double TicksPerRev = 537.7;
    double SpindexerTargetDegree = 0;

    double TurretDegree = 0;
    boolean IntakeMode = false;
    boolean spindexerMoved = false;
    boolean forcespindexerMove = false;

    boolean ShootBalls = false;
    int intakeMovement = 4;

    boolean delayedspindexerMove = false;

    private DcMotor motorLF;
    private DcMotor motorRF;
    private DcMotor motorLB;
    private DcMotor motorRB;
    private DcMotor intake;

    private DcMotor turretwheel;
    private DcMotor spindexer;
    DcMotorEx flywheel;

    GoBildaPinpointDriver odo;

    Servo kicker;
    RevHubOrientationOnRobot revHubOrientationOnRobot;

    float MAX_TURRET = 520;
    float MIN_TURRET = -520;

    double PosX;
    double PosY;




    double GoalPosX = -120;
    double GoalPosY = -100;









    @Override
    public void runOpMode(){
        ElapsedTime timer = new ElapsedTime();
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setPosition(0.0);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spindexer.setPower(0.8f);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretwheel = hardwareMap.get(DcMotor.class, "turretwheel");
        turretwheel.setTargetPosition(0);
        turretwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretwheel.setDirection(DcMotor.Direction.REVERSE);
        turretwheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretwheel.setPower(0.5);


        imu = hardwareMap.get(IMU.class, "imu");

        revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        imu.resetYaw();






        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-190.5, -190.5, DistanceUnit.MM);
        odo.setPosition(new Pose2D(DistanceUnit.CM, -100, -100, AngleUnit.DEGREES, 0));








        waitForStart();
        while (opModeIsActive()) {
            odo.update();
            telemetry.addData("KickerPos = ",kicker.getPosition());
            telemetry.addData("SpindexerDegree = ",SpindexerTargetDegree);
            telemetry.addData("Time: ", timer.seconds());
            telemetry.addData("forcespindexerMove: ", forcespindexerMove);
            telemetry.addData("Flywheel Velocity:", flywheel.getVelocity());
            telemetry.addData("IntakeCount: ", intakeMovement);
            telemetry.addData("TurretDegree: ", TurretDegree);
            telemetry.addData("RobotTurn: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            PosX = (int)odo.getPosition().getX(DistanceUnit.CM);
            PosY = (int)odo.getPosition().getY(DistanceUnit.CM);
            telemetry.addData("RobotPositionX: ", PosX);
            telemetry.addData("RobotPositionY: ", PosY);

            telemetry.update();

            DriveRobot();

            if(gamepad1.aWasPressed())
            {
                kicker.setPosition(0.3);
            }

            UsingSpindexer(timer);

            MoveTurret();


            if(gamepad1.right_trigger > 0.5) {
                Intake(timer);
            }
            else
            {
                resetIntake();
            }

            if(gamepad1.left_trigger >0.5f)
            {
                flywheel.setVelocity(500);
            }
            else
            {
                flywheel.setVelocity(0);
            }

            if(gamepad1.b)
            {
                timer.reset();
                intakeMovement = 0;
                ShootBalls = true;

            }

            if(ShootBalls)
            {
                if(intakeMovement < 3) {

                    if(timer.seconds() > 0.45)
                    {
                        kicker.setPosition(0);
                    }
                    else if(timer.seconds() > 0.2 && timer.seconds() < 0.35)
                    {
                        kicker.setPosition(0.3);
                    }

                    if (timer.seconds() > 0.55) {
                        MoveSpindexer();
                        intakeMovement++;
                        timer.reset();

                    }

                }
                else {
                    ShootBalls = false;
                }
            }

        }

    }

    public void DriveRobot()
    {
        double leftPower;
        double rightPower;


        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Send calculated power to wheels
        motorLF.setPower(leftPower);
        motorLB.setPower(leftPower);
        motorRF.setPower(rightPower);
        motorRB.setPower(rightPower);


    }


    public void Intake(ElapsedTime timer)
    {
        kicker.setPosition(0);

        if (!IntakeMode) {
            timer.reset();
            IntakeMode = true;
            spindexerMoved = false;

        }

        if(timer.seconds() > 0.3 && !spindexerMoved)
        {
            spindexerMoved = true;
            SpindexerTargetDegree += TicksPerRev / 6.0;
            spindexer.setTargetPosition((int) (SpindexerTargetDegree));
            intakeMovement = 0;
            timer.reset();

        }

        if(intakeMovement < 3)
        {
            if(timer.seconds() > 0.3)
            {
                MoveSpindexer();
                intakeMovement++;
                timer.reset();
            }
        }


        intake.setPower(1);

    }

    public void resetIntake()
    {
        if(IntakeMode) {
            IntakeMode = false;
            SpindexerTargetDegree += TicksPerRev /6.0;
            spindexer.setTargetPosition((int) (SpindexerTargetDegree));
        }
        intake.setPower(0);
    }


    void UsingSpindexer(ElapsedTime timer)
    {
        if(gamepad1.rightBumperWasPressed())
        {
            if(kicker.getPosition() == 0)
            {
                forcespindexerMove = true;
            }
            else
            {
                kicker.setPosition(0);
                delayedspindexerMove = true;
                timer.reset();

            }
        }

        if(delayedspindexerMove && timer.seconds() > 0.2f)
        {
            delayedspindexerMove = false;
            forcespindexerMove = true;
        }

        if(forcespindexerMove) {
            MoveSpindexer();
            forcespindexerMove = false;
        }
    }
    public void MoveSpindexer()
    {

        SpindexerTargetDegree += TicksPerRev/3.0;
        spindexer.setTargetPosition((int)(SpindexerTargetDegree));

    }

    public void MoveTurret()
    {
        double TempX = GoalPosX - odo.getPosition().getX(DistanceUnit.CM);
        double TempY = GoalPosY - odo.getPosition().getY(DistanceUnit.CM);

        double TempDegree = Math.toDegrees(Math.atan2(TempY, TempX));

        double Shootdegree = TempDegree - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while(Shootdegree > 180)
        {
            Shootdegree -= 360;
        }

        while(Shootdegree < -180)
        {
            Shootdegree += 360;
        }

        TurretDegree = Range.clip(Shootdegree/180 * TicksPerRev, MIN_TURRET, MAX_TURRET);

        turretwheel.setTargetPosition((int)TurretDegree);
    }


}
