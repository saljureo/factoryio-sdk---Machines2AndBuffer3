using EngineIO;
using System;
using System.Diagnostics;

namespace Controllers.Scenes.MachinesAndBuffer
{
    public class Machines2AndBuffer3 : Controller
    {
        //Mc1 inputs
        readonly MemoryBit mc1StartButton;
        readonly MemoryBit mc1FailButton;
        readonly MemoryBit mc1RepairButton;
        readonly MemoryBit mc1Busy;
        readonly MemoryInt mc1Progress;
        readonly MemoryBit mc1PositionerClamped;//Mc1 positioner clamped sensor
        readonly MemoryBit mc1GripperItemDetected;//Mc1 positioner item detected sensor

        //Mc1 outputs
        readonly MemoryBit mc1Start;//Machining Center start
        readonly MemoryBit mc1Reset;//Machining Center reset (so it leaves piece incomplete)
        readonly MemoryBit mc1RedLight;
        readonly MemoryBit mc1YellowLight;
        readonly MemoryBit mc1GreenLight;
        readonly MemoryBit mc1StartButtonLight;
        readonly MemoryBit mc1RepairButtonLight;
        readonly MemoryBit mc1FailButtonLight;
        readonly MemoryBit mc1AlarmSiren;
        readonly MemoryBit mc1PositionerRise;
        readonly MemoryBit mc1PositionerClamp;

        //Mc2 inputs
        readonly MemoryBit mc2StartButton;
        readonly MemoryBit mc2FailButton;
        readonly MemoryBit mc2RepairButton;
        readonly MemoryBit mc2Busy;
        readonly MemoryBit mc2PositionerClamped;
        readonly MemoryBit mc2GripperItemDetected;

        //Mc2 outputs
        readonly MemoryBit mc2Start;
        readonly MemoryBit mc2RedLight;
        readonly MemoryBit mc2YellowLight;
        readonly MemoryBit mc2GreenLight;
        readonly MemoryBit mc2StartButtonLight;
        readonly MemoryBit mc2RepairButtonLight;
        readonly MemoryBit mc2FailButtonLight;
        readonly MemoryBit mc2AlarmSiren;
        readonly MemoryBit mc2PositionerClamp;
        readonly MemoryBit mc2PositionerRise;

        //Sensors
        readonly MemoryBit sensorMc1ConveyorEntrance;//Diffuse Sensor 0 - Emitter
        readonly MemoryBit sensorEntranceMc1;//Diffuse Sensor 1 - MC0 entrance
        readonly MemoryBit sensorExitMc1;//Diffuse Sensor 2 - MC0 exit/Buffer conveyor entry/MC0 bad piece filter
        readonly MemoryBit sensorBufferEnd;//Diffuse Sensor 3 - Buffer end
        readonly MemoryBit sensorMc2loadingConveyorStart;//Diffuse Sensor 3_2 - Buffer start
        readonly MemoryBit sensorEntranceMc2;//Diffuse Sensor 5 - MC2 entrance
        readonly MemoryBit sensorBadPieceFilterConveyorStartMc1;//Diffuse Sensor 6 - MC0 bad piece filter conveyor entrance
        readonly MemoryBit sensorBadPieceFilterConveyorEndMc1;//Diffuse Sensor 7 - MC0 bad piece filter conveyor exit
        readonly MemoryBit sensorExitMc2;//Diffuse Sensor 8 - MC2 exit/MC2 bad piece filter
        readonly MemoryBit sensorFinishedPartExit;//Diffuse Sensor 9 - Finished piece exit
        readonly MemoryBit sensorBadPieceFilterConveyorStartMc2;//Diffuse Sensor 10 - MC2 bad piece conveyor entrance
        readonly MemoryBit sensorBadPieceFilterConveyorEndMc2;//Diffuse Sensor 11 - MC2 bad piece conveyor exit
        readonly MemoryBit sensorBufferSpot2;//Diffuse Sensor 13 
        readonly MemoryBit sensorBufferSpot3;//Diffuse Sensor 14
        readonly MemoryBit sensorEmitter;

        //Emitter
        readonly MemoryBit emitter;//Emitter

        //Buffer
        readonly MemoryBit bufferStopblade;//Buffer stopblade

        //GripperArms
        readonly GripperArm gripperMc1;
        readonly GripperArm gripperMc2;

        //Light controls
        readonly McLightsControl mc1Lights;
        readonly McLightsControl mc2Lights;

        //SUPERVISORY CONTROL
        readonly Machines2AndBuffer3Supervisor supervisoryControl;
        bool supervisoryApproval;

        //conveyor belts
        readonly MemoryFloat conveyorMc2Entrance;
        readonly MemoryFloat conveyorMc1Entrance;
        readonly MemoryBit conveyorMc1BadPiece;
        readonly MemoryBit conveyorFinishedPiece;
        readonly MemoryBit conveyorMc2BadPiece;
        readonly MemoryBit conveyorEmitter;
        readonly MemoryBit conveyorBuffer;

        //Failing time (potenciometer and display)
        readonly MemoryFloat potentiometerMc1;
        readonly MemoryFloat potentiometerMc2;
        readonly MemoryFloat displayMc1;
        readonly MemoryFloat displayMc2;
        readonly Stopwatch failTimeMc1;
        float failTimeFloatMc1;
        readonly Stopwatch failTimeMc2;
        float failTimeFloatMc2;
        bool mc1Failed;
        bool mc2Failed;
        bool timeStartBool;

        //Others
        bool initialStateMessagePrinted;
        bool changeStateMessagePrinted;
        string newState;
        string newStateName;
        

        //Enums
        McStatus mc1Status;
        McStatus mc2Status;
        McPositionerStatus mc1PositionerStatus;
        McPositionerStatus mc2PositionerStatus;
        BufferStatus bufferStatus;
        Mc2andMc3LoadingSteps loadingMc2Step;
        Mc1PieceReady mc1PieceReady;
        Mc1PieceReadySteps mc1PieceReadySteps;
        Events eventsMc;
        Mc1WorkingStage mc1WorkingStage;

        //Controllable events
        private int s1Counter;
        private int r1Counter;
        private int s2Counter;
        private int r2Counter;

        //FTRIG
        readonly FTRIG ftAtEntranceMc1;
        readonly FTRIG ftAtEntranceMc2;
        readonly FTRIG ftAtExitMc1;
        readonly FTRIG ftAtExitMc2;
        readonly FTRIG ftAtBufferEnd;
        readonly FTRIG ftAtBadPieceExitMc1;
        readonly FTRIG ftAtBadPieceExitMc2;
        readonly FTRIG ftAtFinishedPieceExit;
        readonly FTRIG ftAtBufferStart;
        readonly FTRIG ftAtMc2LoadingConveyorStart;
        readonly FTRIG ftAtEmitter;

        //RTRIG
        readonly RTRIG rtAtExitMc1;
        readonly RTRIG rtAtExitMc2;
        readonly RTRIG rtAtMc2LoadingConveyorStart;

        private enum BufferStatus
        {
            EMPTY,
            ONE,
            TWO,
            THREE
        }
        private enum BufferSteps
        {
            PIECE_ARRIVED_TO_BUFFER,
            FILTERING_BAD_PIECE,
            PIECE_REACHING_MC2,
            PIECE_ARRIVED_TO_MC2
        }
        private enum McPositionerStatus
        {
            UP,
            DOWN,
            CLAMP,
            GOING_UP
        }
        private enum Mc1PieceReady
        {
            NOT_READY,
            READY
        }
        private enum Mc1PieceReadySteps
        {
            IDLE,
            SWITCHING_CONVEYORS,
            REACHING_MC1ENTRANCE,
            ENTERINGMC1
        }
        private enum Mc2andMc3LoadingSteps
        {
            IDLE,
            PIECE_TO_LOADING_CONVEYOR,
            SEPARATE_OTHER_PIECES,
            RESTORING_BUFFER_ORDER,
            REACHED_MC
        }
        private enum Events
        {
            s1,
            f1,
            b1,
            r1,
            i1,
            s2,
            f2,
            b2,
            r2,
            i2,
            s3,
            f3,
            b3,
            r3,
            i3
        }
        private enum BreakdownMc2OrMc3
        {
            OK,
            KO
        }
        private enum Mc1WorkingStage
        {
            CONVEYOR,
            MACHINING_CENTER1,
            MACHINING_CENTER2
        }
        private enum McStatus
        {
            IDLE,
            WORKING,
            DOWN
        }

        public Machines2AndBuffer3()// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRUCTOR STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        {
            //Mc1 inputs
            mc1StartButton = MemoryMap.Instance.GetBit("Start Mc1", MemoryType.Input);
            mc1FailButton = MemoryMap.Instance.GetBit("Force Failure Mc1", MemoryType.Input);
            mc1RepairButton = MemoryMap.Instance.GetBit("Repair Mc1", MemoryType.Input);
            mc1Busy = MemoryMap.Instance.GetBit("Machining Center 0 (Is Busy)", MemoryType.Input);//Machining Center 0 busy
            mc1Progress = MemoryMap.Instance.GetInt("Machining Center 0 (Progress)", MemoryType.Input);//Machining Center 0 opened
            mc1PositionerClamped = MemoryMap.Instance.GetBit("Right Positioner 0 (Clamped)", MemoryType.Input);//Mc1 positioner clamped sensor
            mc1GripperItemDetected = MemoryMap.Instance.GetBit("Two-Axis Pick & Place 0 (Item Detected)", MemoryType.Input);//Mc1 positioner item detected sensor

            //Mc1 outputs
            mc1Start = MemoryMap.Instance.GetBit("Machining Center 0 (Start)", MemoryType.Output);//Machining Center start
            mc1Reset = MemoryMap.Instance.GetBit("Machining Center 0 (Reset)", MemoryType.Output);//Machining Center reset (so it leaves piece incomplete)
            mc1RedLight = MemoryMap.Instance.GetBit("Stack Light 0 (Red)", MemoryType.Output);
            mc1YellowLight = MemoryMap.Instance.GetBit("Stack Light 0 (Yellow)", MemoryType.Output);
            mc1GreenLight = MemoryMap.Instance.GetBit("Stack Light 0 (Green)", MemoryType.Output);
            mc1StartButtonLight = MemoryMap.Instance.GetBit("Start mc1 (Light)", MemoryType.Output);
            mc1RepairButtonLight = MemoryMap.Instance.GetBit("Repair mc1 (Light)", MemoryType.Output);
            mc1FailButtonLight = MemoryMap.Instance.GetBit("Failure mc1 (Light)", MemoryType.Output);
            mc1AlarmSiren = MemoryMap.Instance.GetBit("Alarm Siren 0", MemoryType.Output);
            mc1PositionerRise = MemoryMap.Instance.GetBit("Right Positioner 0 (Raise)", MemoryType.Output);
            mc1PositionerClamp = MemoryMap.Instance.GetBit("Right Positioner 0 (Clamp)", MemoryType.Output);

            //mc1 machine values
            mc1Start.Value = false;
            mc1Busy.Value = false;

            //mc1 buttons
            mc1StartButton.Value = false;
            mc1FailButton.Value = true;//unpressed is true
            mc1RepairButton.Value = false;

            //mc1 others
            mc1PositionerClamp.Value = false;
            mc1PositionerRise.Value = true;

            //Mc2 inputs
            mc2StartButton = MemoryMap.Instance.GetBit("Start Mc2", MemoryType.Input);
            mc2StartButton.Value = false;
            mc2FailButton = MemoryMap.Instance.GetBit("Force Failure Mc2", MemoryType.Input);
            mc2FailButton.Value = true;//True is unpressed
            mc2RepairButton = MemoryMap.Instance.GetBit("Repair Mc2", MemoryType.Input);
            mc2RepairButton.Value = false;
            mc2Busy = MemoryMap.Instance.GetBit("Machining Center 1 (Is Busy)", MemoryType.Input);
            mc2PositionerClamped = MemoryMap.Instance.GetBit("Right Positioner 3 (Clamped)", MemoryType.Input);//Mc2 positioner clamped sensor
            mc2GripperItemDetected = MemoryMap.Instance.GetBit("Two-Axis Pick & Place 1 (Item Detected)", MemoryType.Input);//Mc2 positioner item detected sensor

            //Mc2 outputs
            mc2Start = MemoryMap.Instance.GetBit("Machining Center 1 (Start)", MemoryType.Output);
            mc2RedLight = MemoryMap.Instance.GetBit("Stack Light 1 (Red)", MemoryType.Output);
            mc2YellowLight = MemoryMap.Instance.GetBit("Stack Light 1 (Yellow)", MemoryType.Output);
            mc2GreenLight = MemoryMap.Instance.GetBit("Stack Light 1 (Green)", MemoryType.Output);
            mc2StartButtonLight = MemoryMap.Instance.GetBit("Start mc2 (Light)", MemoryType.Output);
            mc2RepairButtonLight = MemoryMap.Instance.GetBit("Repair mc2 (Light)", MemoryType.Output);
            mc2FailButtonLight = MemoryMap.Instance.GetBit("Failure mc2 (Light)", MemoryType.Output);
            mc2AlarmSiren = MemoryMap.Instance.GetBit("Alarm Siren 1", MemoryType.Output);
            mc2PositionerClamp = MemoryMap.Instance.GetBit("Right Positioner 3 (Clamp)", MemoryType.Output);//mc2 positioner clamp
            mc2PositionerRise = MemoryMap.Instance.GetBit("Right Positioner 3 (Raise)", MemoryType.Output);//mc2 positioner rise

            //mc2 others
            mc2Failed = false;

            //Sensors
            sensorMc1ConveyorEntrance = MemoryMap.Instance.GetBit("Diffuse Sensor 0", MemoryType.Input);//Diffuse Sensor 0 - Emitter
            sensorEntranceMc1 = MemoryMap.Instance.GetBit("Diffuse Sensor 1", MemoryType.Input);//Diffuse Sensor 1 - MC1 entrance
            sensorExitMc1 = MemoryMap.Instance.GetBit("Diffuse Sensor 2", MemoryType.Input);//Diffuse Sensor 2 - MC1 exit/Buffer conveyor entry/MC1 bad piece filter
            sensorBufferEnd = MemoryMap.Instance.GetBit("Diffuse Sensor 3", MemoryType.Input);//Diffuse Sensor 3 - Buffer end
            sensorMc2loadingConveyorStart = MemoryMap.Instance.GetBit("Diffuse Sensor 3_2", MemoryType.Input);//Diffuse Sensor 3_2 - Buffer start
            sensorEntranceMc2 = MemoryMap.Instance.GetBit("Diffuse Sensor 5", MemoryType.Input);//Diffuse Sensor 5 - MC1 entrance
            sensorBadPieceFilterConveyorStartMc1 = MemoryMap.Instance.GetBit("Diffuse Sensor 6", MemoryType.Input);//Diffuse Sensor 6 - MC1 bad piece filter conveyor entrance
            sensorBadPieceFilterConveyorEndMc1 = MemoryMap.Instance.GetBit("Diffuse Sensor 7", MemoryType.Input);//Diffuse Sensor 7 - MC1 bad piece filter conveyor exit
            sensorExitMc2 = MemoryMap.Instance.GetBit("Diffuse Sensor 8", MemoryType.Input);//Diffuse Sensor 8 - MC1 exit/MC1 bad piece filter
            sensorFinishedPartExit = MemoryMap.Instance.GetBit("Diffuse Sensor 9", MemoryType.Input);//Diffuse Sensor 9 - Finished piece exit
            sensorBadPieceFilterConveyorStartMc2 = MemoryMap.Instance.GetBit("Diffuse Sensor 10", MemoryType.Input);//Diffuse Sensor 10 - MC2 bad piece conveyor entrance
            sensorBadPieceFilterConveyorEndMc2 = MemoryMap.Instance.GetBit("Diffuse Sensor 11", MemoryType.Input);//Diffuse Sensor 11 - MC2 bad piece conveyor exit
            sensorBufferSpot2 = MemoryMap.Instance.GetBit("Diffuse Sensor 13", MemoryType.Input);//Diffuse Sensor 13
            sensorBufferSpot3 = MemoryMap.Instance.GetBit("Diffuse Sensor 14", MemoryType.Input);//Diffuse Sensor 14
            sensorEmitter = MemoryMap.Instance.GetBit("Diffuse Sensor 4", MemoryType.Input);//Diffuse Sensor 4 - Sensor emitter

            //Emitter
            emitter = MemoryMap.Instance.GetBit("Emitter 0 (Emit)", MemoryType.Output);//Emitter
            emitter.Value = true;

            //Buffer
            bufferStopblade = MemoryMap.Instance.GetBit("Stop Blade 0", MemoryType.Output);
            bufferStopblade.Value = true;//True is rised

            //GripperArms
            gripperMc1 = new GripperArm(
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 0 X Position (V)", MemoryType.Input),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 0 Z Position (V)", MemoryType.Input),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 0 X Set Point (V)", MemoryType.Output),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 0 Z Set Point (V)", MemoryType.Output),
                MemoryMap.Instance.GetBit("Two-Axis Pick & Place 0 (Grab)", MemoryType.Output)
            );

            gripperMc2 = new GripperArm(
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 1 X Position (V)", MemoryType.Input),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 1 Z Position (V)", MemoryType.Input),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 1 X Set Point (V)", MemoryType.Output),
                MemoryMap.Instance.GetFloat("Two-Axis Pick & Place 1 Z Set Point (V)", MemoryType.Output),
                MemoryMap.Instance.GetBit("Two-Axis Pick & Place 1 (Grab)", MemoryType.Output)
            );

            //Light controls
            mc1Lights = new McLightsControl(mc1RedLight, mc1YellowLight, mc1GreenLight);
            mc2Lights = new McLightsControl(mc2RedLight, mc2YellowLight, mc2GreenLight);

            //conveyor belts
            conveyorMc2Entrance = MemoryMap.Instance.GetFloat("Belt Conveyor (2m) 0 (V)", MemoryType.Output);
            conveyorMc1Entrance = MemoryMap.Instance.GetFloat("Belt Conveyor (2m) 4 (V)", MemoryType.Output);
            conveyorMc1BadPiece = MemoryMap.Instance.GetBit("Belt Conveyor (2m) 1", MemoryType.Output);
            conveyorFinishedPiece = MemoryMap.Instance.GetBit("Belt Conveyor (2m) 2", MemoryType.Output);
            conveyorMc2BadPiece = MemoryMap.Instance.GetBit("Belt Conveyor (2m) 3", MemoryType.Output);
            conveyorEmitter = MemoryMap.Instance.GetBit("Belt Conveyor (2m) 5", MemoryType.Output);
            conveyorBuffer = MemoryMap.Instance.GetBit("Belt Conveyor (4m) 1", MemoryType.Output);

            //Failing time (potenciometer and display)
            potentiometerMc1 = MemoryMap.Instance.GetFloat("Mc1 Failure Time Control", MemoryType.Input);
            potentiometerMc2 = MemoryMap.Instance.GetFloat("Mc2 Failure Time Control", MemoryType.Input);
            displayMc1 = MemoryMap.Instance.GetFloat("Mc1 Failure time (in seconds x 100)", MemoryType.Output);
            displayMc2 = MemoryMap.Instance.GetFloat("Mc2 Failure time (in seconds x 100)", MemoryType.Output);
            timeStartBool = false;
            failTimeMc1 = new Stopwatch();
            failTimeFloatMc1 = 0.0f;
            failTimeMc2 = new Stopwatch();
            failTimeFloatMc2 = 0.0f;

            failTimeMc1.Start();
            failTimeMc2.Start();

            //Enums
            mc1Status = McStatus.IDLE;
            mc2Status = McStatus.IDLE;
            mc1PositionerStatus = McPositionerStatus.UP;
            mc2PositionerStatus = McPositionerStatus.UP;
            bufferStatus = BufferStatus.EMPTY;
            loadingMc2Step = Mc2andMc3LoadingSteps.IDLE;
            mc1PieceReady = Mc1PieceReady.NOT_READY;
            mc1PieceReadySteps = Mc1PieceReadySteps.IDLE;
            eventsMc = Events.i1;
            mc1WorkingStage = Mc1WorkingStage.CONVEYOR;

            //FTRIG
            ftAtEntranceMc1 = new FTRIG();
            ftAtEntranceMc2 = new FTRIG();
            ftAtExitMc1 = new FTRIG();
            ftAtExitMc2 = new FTRIG();
            ftAtBufferEnd = new FTRIG();
            ftAtBadPieceExitMc1 = new FTRIG();
            ftAtBadPieceExitMc2 = new FTRIG();
            ftAtFinishedPieceExit = new FTRIG();
            ftAtBufferStart = new FTRIG();
            ftAtMc2LoadingConveyorStart = new FTRIG();
            ftAtEmitter = new FTRIG();

            //RTRIG
            rtAtExitMc1 = new RTRIG();
            rtAtExitMc2 = new RTRIG();
            rtAtMc2LoadingConveyorStart = new RTRIG();

            // Controllable events
            s1Counter = 0;
            r1Counter = 0;
            s2Counter = 0;
            r2Counter = 0;

            //SUPERVISORY CONTROL
            supervisoryControl = new Machines2AndBuffer3Supervisor();
            supervisoryApproval = true;

            //Trick for printing initial state in console after start up messages
            initialStateMessagePrinted = false;

            //Others
            changeStateMessagePrinted = false;
            newState = "";
            newStateName = "";
            
        }
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRUCTOR ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        public override void Execute(int elapsedMilliseconds) // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EXECUTE STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        {


            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRICK FOR PRINTING INITIAL STATE AFTER START UP MESSAGES START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (initialStateMessagePrinted == false)
            {
                Console.WriteLine("\nTip: Set the failure time using the potentiometers in the control panels before starting. Recomended number ranges from 0.90 to 1.50. Also recommended to place same failure time to both machines.\n");
                supervisoryControl.CreateController();
                initialStateMessagePrinted = true;
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TRICK FOR PRINTING INITIAL STATE AFTER START UP MESSAGES END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FAILING TIME AND DISPLAY START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            displayMc1.Value = float.Parse(String.Format("{0:0.0}", potentiometerMc1.Value));
            displayMc2.Value = float.Parse(String.Format("{0:0.0}", potentiometerMc2.Value));
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FAILING TIME AND DISPLAY END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BUTTON LIGHTS START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            //mc1
            if (supervisoryControl.IsInActiveEventsLights("e1"))
                mc1StartButtonLight.Value = true;
            else
                mc1StartButtonLight.Value = false;

            if (supervisoryControl.IsInActiveEventsLights("r1"))
                mc1RepairButtonLight.Value = true;
            else
                mc1RepairButtonLight.Value = false;

            if (!mc1Failed)
                mc1FailButtonLight.Value = true;
            else
                mc1FailButtonLight.Value = false;

            //mc2
            if (supervisoryControl.IsInActiveEventsLights("e2"))
                mc2StartButtonLight.Value = true;
            else
                mc2StartButtonLight.Value = false;

            if (supervisoryControl.IsInActiveEventsLights("r2"))
                mc2RepairButtonLight.Value = true;
            else
                mc2RepairButtonLight.Value = false;

            if (!mc2Failed)
                mc2FailButtonLight.Value = true;
            else
                mc2FailButtonLight.Value = false;

            }
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BUTTON LIGHTS END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLLABLE EVENTS START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            // Keyboard input:
            try
            {
                if (!changeStateMessagePrinted)
                {
                    changeStateMessagePrinted = true;
                }
                newStateName = "";
                newState = Reader.ReadLine(5);
                try
                {
                    newStateName = supervisoryControl.StateName(int.Parse(newState));
                    if (newStateName != "Event number pressed does not exist")
                    {
                        if (!supervisoryControl.IsInActiveEvents(int.Parse(newState)))
                        {
                            Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
                            Console.WriteLine("\nEvent " + newState + " is not in active events. Try again.\n");
                            Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
                            supervisoryControl.ListOfActiveEvents();
                        }
                    }
                }
                catch
                {
                    Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
                    Console.WriteLine("\nSorry, please insert a number.\n");
                    Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
                    supervisoryControl.ListOfActiveEvents();
                }
                changeStateMessagePrinted = false;
            }
            catch (TimeoutException)
            {
            }

            //s1
            if (mc1StartButton.Value == true || (newStateName == "e1" && supervisoryControl.IsInActiveEvents(int.Parse(newState))))
            {
                if (!timeStartBool)
                {
                    timeStartBool = true;
                }

                if (s1Counter == 0)
                {
                    supervisoryApproval = supervisoryControl.On("e1");
                    if (supervisoryApproval == true)
                    {
                        eventsMc = Events.s1;
                        s1Counter++;
                    }
                }
            }
            else
            {
                s1Counter = 0;
            }

            //r1
            if (mc1RepairButton.Value == true || (newStateName == "r1" && supervisoryControl.IsInActiveEvents(int.Parse(newState))))
            {
                if (r1Counter == 0)
                {
                    supervisoryApproval = supervisoryControl.On("r1");
                    if (supervisoryApproval == true)
                    {
                        eventsMc = Events.r1;
                        r1Counter++;
                    }
                }
            }
            else
            {
                r1Counter = 0;
            }

            //s2
            if (mc2StartButton.Value == true || (newStateName == "e2" && supervisoryControl.IsInActiveEvents(int.Parse(newState))))
            {
                if (s2Counter == 0)
                {
                    supervisoryApproval = supervisoryControl.On("e2");
                    if (supervisoryApproval == true)
                    {
                        eventsMc = Events.s2;
                        s2Counter++;
                    }
                }
            }
            else
            {
                s2Counter = 0;
            }

            //r2
            if (mc2RepairButton.Value == true || (newStateName == "r2" && supervisoryControl.IsInActiveEvents(int.Parse(newState))))
            {
                if (r2Counter == 0)
                {
                    supervisoryApproval = supervisoryControl.On("r2");
                    if (supervisoryApproval == true)
                    {
                        eventsMc = Events.r2;
                        r2Counter++;
                    }
                }
            }
            else
            {
                r2Counter = 0;
            }
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLLABLE EVENTS END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UNCONTROLLABLE EVENTS START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            if (timeStartBool)
            {
                failTimeFloatMc1 = Convert.ToSingle(failTimeMc1.ElapsedMilliseconds / 100);
                failTimeFloatMc1 = failTimeFloatMc1 / 10;

                failTimeFloatMc2 = Convert.ToSingle(failTimeMc2.ElapsedMilliseconds / 100);
                failTimeFloatMc2 = failTimeFloatMc2 / 10;
            }


            if (mc1FailButton.Value == false || float.Parse(String.Format("{0:0.0}", (failTimeFloatMc1) % (displayMc1.Value * 100))) == 1.0f)//false is button pressed
            {
                mc1Failed = true;
            }

            if (mc2FailButton.Value == false || float.Parse(String.Format("{0:0.0}", (failTimeFloatMc2) % (displayMc2.Value * 100))) == 1.0f)//false is button pressed)
            {
                mc2Failed = true;
            }
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UNCONTROLLABLE EVENTS END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FALLING AND RISING TRIGGERS START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            ftAtEntranceMc1.CLK(sensorEntranceMc1.Value);
            ftAtEntranceMc2.CLK(sensorEntranceMc2.Value);
            ftAtExitMc1.CLK(sensorExitMc1.Value);
            ftAtExitMc2.CLK(sensorExitMc2.Value);
            ftAtBufferEnd.CLK(sensorBufferEnd.Value);
            ftAtBufferStart.CLK(sensorMc2loadingConveyorStart.Value);
            ftAtBadPieceExitMc1.CLK(sensorBadPieceFilterConveyorEndMc1.Value);
            ftAtBadPieceExitMc2.CLK(sensorBadPieceFilterConveyorEndMc2.Value);
            ftAtFinishedPieceExit.CLK(sensorFinishedPartExit.Value);
            ftAtMc2LoadingConveyorStart.CLK(sensorMc2loadingConveyorStart.Value);
            ftAtEmitter.CLK(sensorMc1ConveyorEntrance.Value);

            rtAtExitMc1.CLK(sensorExitMc1.Value);
            rtAtExitMc2.CLK(sensorExitMc2.Value);
            rtAtMc2LoadingConveyorStart.CLK(sensorMc2loadingConveyorStart.Value);
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FALLING AND RISING TRIGGERS END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONVEYORS START %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            {
            //Mc1 bad piece conveyor
            if (sensorBadPieceFilterConveyorStartMc1.Value == true)
            {
                conveyorMc1BadPiece.Value = true;
            }
            else if (ftAtBadPieceExitMc1.Q == true)
            {
                conveyorMc1BadPiece.Value = false;
            }

            //Mc2 bad piece conveyor
            if (sensorBadPieceFilterConveyorStartMc2.Value == true)
            {
                conveyorMc2BadPiece.Value = true;
            }
            else if (ftAtBadPieceExitMc2.Q == true)
            {
                conveyorMc2BadPiece.Value = false;
            }
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONVEYORS ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BUFFER STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            {
            if (bufferStatus == BufferStatus.EMPTY)
            {
                //type here
                if (rtAtExitMc1.Q == true && mc1Failed == false)
                {
                    conveyorBuffer.Value = true;
                    bufferStatus = BufferStatus.ONE;
                }
            }
            else if (bufferStatus == BufferStatus.ONE)
            {
                if (sensorBufferEnd.Value == true && loadingMc2Step == Mc2andMc3LoadingSteps.IDLE && mc1Failed == false)
                {
                    conveyorBuffer.Value = false;
                }
                if (rtAtMc2LoadingConveyorStart.Q == true)
                {
                    bufferStatus = BufferStatus.EMPTY;
                }
                if (rtAtExitMc1.Q == true && mc1Failed == false)
                {
                    conveyorBuffer.Value = true;
                    bufferStatus = BufferStatus.TWO;
                }
            }
            else if (bufferStatus == BufferStatus.TWO)
            {
                if (sensorBufferSpot2.Value == true && loadingMc2Step == Mc2andMc3LoadingSteps.IDLE && mc1Failed == false)
                {
                    conveyorBuffer.Value = false;
                }
                if (rtAtMc2LoadingConveyorStart.Q == true)
                {
                    bufferStatus = BufferStatus.ONE;
                }
                if (rtAtExitMc1.Q == true && mc1Failed == false)
                {
                    conveyorBuffer.Value = true;
                    bufferStatus = BufferStatus.THREE;
                }
            }
            else if (bufferStatus == BufferStatus.THREE)
            {
                if (sensorBufferSpot3.Value == true && loadingMc2Step == Mc2andMc3LoadingSteps.IDLE && mc1Failed == false)
                {
                    conveyorBuffer.Value = false;
                }
                if (rtAtMc2LoadingConveyorStart.Q == true)
                {
                    bufferStatus = BufferStatus.TWO;
                }
            }
            }
            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BUFFER ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



            // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MACHINE CENTER STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            { 
            // %%%%%%%%%%%%%%%%%%%% MC1 STARTS %%%%%%%%%%%%%%%%%%%%
            {
            // %%%% BAD PIECES FILTER STARTS %%%%%
            gripperMc1.StateTransition();

            if (mc1PositionerStatus == McPositionerStatus.UP)
            {
                mc1PositionerClamp.Value = false;
                mc1PositionerRise.Value = true;
                if (rtAtExitMc1.Q == true && mc1Failed == true)
                {
                    conveyorBuffer.Value = true;
                    mc1PositionerStatus = McPositionerStatus.DOWN;
                }
            }
            else if (mc1PositionerStatus == McPositionerStatus.DOWN)
            {
                mc1PositionerRise.Value = false;
                if (ftAtExitMc1.Q == true)
                {
                    conveyorBuffer.Value = false;
                    mc1PositionerStatus = McPositionerStatus.CLAMP;
                }
            }
            else if (mc1PositionerStatus == McPositionerStatus.CLAMP)
            {
                mc1PositionerClamp.Value = true;
                if (mc1PositionerClamped.Value == true)
                {
                    gripperMc1.Start();
                    if (mc1GripperItemDetected.Value == true)
                    {
                        mc1PositionerStatus = McPositionerStatus.GOING_UP;
                    }
                }
            }
            else if (mc1PositionerStatus == McPositionerStatus.GOING_UP)
            {
                mc1PositionerClamp.Value = false;
                if (sensorBadPieceFilterConveyorStartMc1.Value == true)
                {
                    mc1PositionerStatus = McPositionerStatus.UP;
                }
            }
            // %%%% BAD PIECES FILTER ENDS %%%%

            // %%%% PREPARING MC1 PIECE STARTS %%%%

            if (mc1PieceReady == Mc1PieceReady.NOT_READY)
            {
                emitter.Value = true;
                if (sensorEmitter.Value == true)
                {
                    mc1PieceReady = Mc1PieceReady.READY;
                }
            }
            else if (mc1PieceReady == Mc1PieceReady.READY)
            {
                emitter.Value = false;
                if (ftAtEntranceMc1.Q)
                {
                    mc1PieceReady = Mc1PieceReady.NOT_READY;
                }
            }

            // %%%% PREPARING MC1 PIECE ENDS %%%%%


            // %%%% MC1 IDLE STARTS %%%%%
            if (mc1Status == McStatus.IDLE)
            {
                mc1Lights.IdleLights();
                if (eventsMc == Events.s1 && mc1PieceReady == Mc1PieceReady.READY)
                {
                    mc1Status = McStatus.WORKING;
                    mc1WorkingStage = Mc1WorkingStage.CONVEYOR;
                    mc1PieceReadySteps = Mc1PieceReadySteps.SWITCHING_CONVEYORS;
                    eventsMc = Events.i1;
                }
            }
            // %%%% MC1 IDLE ENDS %%%%%

            // %%%% MC1 WORKING STARTS %%%%%
            else if (mc1Status == McStatus.WORKING)
            {
                mc1Lights.WorkingLights();
                if (mc1WorkingStage == Mc1WorkingStage.CONVEYOR)
                {
                    if (mc1PieceReadySteps == Mc1PieceReadySteps.SWITCHING_CONVEYORS)
                    {
                        conveyorEmitter.Value = true;//Turns on both conveyors
                        conveyorMc1Entrance.Value = 5;//Turns on both conveyors
                        if (ftAtEmitter.Q == true)//If it exits emitter sensor
                        {
                            mc1PieceReadySteps = Mc1PieceReadySteps.REACHING_MC1ENTRANCE;
                        }
                    }
                    else if (mc1PieceReadySteps == Mc1PieceReadySteps.REACHING_MC1ENTRANCE)
                    {
                        conveyorEmitter.Value = false;//Turns off emitter conveyor

                        if (ftAtEntranceMc1.Q == true)
                        {
                            conveyorMc1Entrance.Value = 0;//Turns off mc1 entrance conveyor
                            mc1Start.Value = true;//Starts mc1
                            mc1PieceReadySteps = Mc1PieceReadySteps.IDLE;
                            mc1WorkingStage = Mc1WorkingStage.MACHINING_CENTER1;
                        }
                    }

                }
                else if (mc1WorkingStage == Mc1WorkingStage.MACHINING_CENTER1)
                {
                    if (mc1Busy.Value == true)
                    {
                        mc1Start.Value = false;
                    }

                    if (mc1Progress.Value > 80)
                    {
                        mc1Reset.Value = true;
                        mc1WorkingStage = Mc1WorkingStage.MACHINING_CENTER2;
                    }
                }
                else if (mc1WorkingStage == Mc1WorkingStage.MACHINING_CENTER2)
                {
                    if (rtAtExitMc1.Q == true && mc1Failed == false)
                    {
                        mc1Reset.Value = false;
                        mc1Status = McStatus.IDLE;
                        supervisoryApproval = supervisoryControl.On("t1");
                        //mc1Failed = true; //will fail next time
                    }
                    else if (rtAtExitMc1.Q == true && mc1Failed == true)
                    {
                        mc1Reset.Value = false;
                        mc1Status = McStatus.DOWN;
                        supervisoryApproval = supervisoryControl.On("f1");
                    }
                }


            }
            // %%%% MC1 WORKING ENDS %%%%%

            // %%%% MC1 DOWN STARTS %%%%%
            else if (mc1Status == McStatus.DOWN)
            {
                mc1AlarmSiren.Value = true;
                mc1Lights.FailingLights();

                if (eventsMc == Events.r1)
                {
                    mc1Failed = false;//Next piece will not fail
                    mc1Status = McStatus.IDLE;
                    mc1AlarmSiren.Value = false;
                }
            }
            // %%%% MC1 DOWN ENDS %%%%%
            }
            // %%%%%%%%%%%%%%%%%%%% MC1 ENDS %%%%%%%%%%%%%%%%%%%%



            // %%%%%%%%%%%%%%%%%%%% MC2 STARTS %%%%%%%%%%%%%%%%%%%%
            {
            //%%%% BAD PIECES FILTER STARTS %%%%%
            gripperMc2.StateTransition();

            if (mc2PositionerStatus == McPositionerStatus.UP)
            {
                mc2PositionerClamp.Value = false;
                mc2PositionerRise.Value = true;
                if (rtAtExitMc2.Q == true && mc2Failed == true)
                {
                    conveyorFinishedPiece.Value = true;
                    mc2PositionerStatus = McPositionerStatus.DOWN;
                }
            }
            else if (mc2PositionerStatus == McPositionerStatus.DOWN)
            {
                mc2PositionerRise.Value = false;
                if (ftAtExitMc2.Q == true)
                {
                    conveyorFinishedPiece.Value = false;
                    mc2PositionerStatus = McPositionerStatus.CLAMP;
                }
            }
            else if (mc2PositionerStatus == McPositionerStatus.CLAMP)
            {
                mc2PositionerClamp.Value = true;
                if (mc2PositionerClamped.Value == true)
                {
                    gripperMc2.Start();
                    if (mc2GripperItemDetected.Value == true)
                    {
                        mc2PositionerStatus = McPositionerStatus.GOING_UP;
                    }
                }
            }
            else if (mc2PositionerStatus == McPositionerStatus.GOING_UP)
            {
                mc2PositionerClamp.Value = false;
                if (sensorBadPieceFilterConveyorStartMc2.Value == true)
                {
                    mc2PositionerStatus = McPositionerStatus.UP;
                }
            }
            //%%% BAD PIECES FILTER ENDS %%%%

            //%%% MC2 IDLE STARTS %%%%
            if (mc2Status == McStatus.IDLE)
            {
                mc2Lights.IdleLights();
                if (eventsMc == Events.s2 && bufferStatus != BufferStatus.EMPTY && loadingMc2Step == Mc2andMc3LoadingSteps.IDLE)
                {
                    mc2Status = McStatus.WORKING;
                    loadingMc2Step = Mc2andMc3LoadingSteps.PIECE_TO_LOADING_CONVEYOR;
                }
            }
            //%%% MC2 IDLE ENDS %%%%

            //%%% MC2 WORKING STARTS %%%%
            else if (mc2Status == McStatus.WORKING)
            {
                mc2Lights.WorkingLights();
                if (loadingMc2Step == Mc2andMc3LoadingSteps.PIECE_TO_LOADING_CONVEYOR)
                {
                    bufferStopblade.Value = false;//Drop Stopblade
                    mc2Start.Value = true;
                    conveyorBuffer.Value = true;//turn on both conveyors
                    conveyorMc2Entrance.Value = 1.0f;//turn on both conveyors

                    if (sensorMc2loadingConveyorStart.Value == true)
                    {
                        loadingMc2Step = Mc2andMc3LoadingSteps.SEPARATE_OTHER_PIECES;
                    }
                }
                else if (loadingMc2Step == Mc2andMc3LoadingSteps.SEPARATE_OTHER_PIECES)
                {
                    conveyorBuffer.Value = false;//turn of buffer conveyor

                    if (ftAtMc2LoadingConveyorStart.Q == true)
                    {
                        loadingMc2Step = Mc2andMc3LoadingSteps.RESTORING_BUFFER_ORDER;
                    }
                }
                else if (loadingMc2Step == Mc2andMc3LoadingSteps.RESTORING_BUFFER_ORDER)
                {
                    bufferStopblade.Value = true;
                    conveyorBuffer.Value = true;//turn on buffer conveyor

                    if (sensorBufferEnd.Value == true || bufferStatus == BufferStatus.EMPTY)
                    {
                        conveyorBuffer.Value = false;//turn off buffer conveyor
                    }
                    if (ftAtEntranceMc2.Q == true)
                    {
                        loadingMc2Step = Mc2andMc3LoadingSteps.REACHED_MC;
                    }
                }
                else if (loadingMc2Step == Mc2andMc3LoadingSteps.REACHED_MC)
                {
                    conveyorMc2Entrance.Value = 0;//turn off entrance conveyor
                    loadingMc2Step = Mc2andMc3LoadingSteps.IDLE;
                }

                if (mc2Busy.Value == true)
                {
                    mc2Start.Value = false;
                }

                if (rtAtExitMc2.Q == true && mc2Failed == false)
                {
                    conveyorFinishedPiece.Value = true;
                    eventsMc = Events.f2;
                    mc2Status = McStatus.IDLE;
                    supervisoryApproval = supervisoryControl.On("t2");
                    //mc2Failed = true; //will fail next time
                }
                else if (rtAtExitMc2.Q == true && mc2Failed == true)
                {
                    eventsMc = Events.b2;
                    mc2Status = McStatus.DOWN;
                    supervisoryApproval = supervisoryControl.On("f2");
                }
            }
            //%%% MC2 WORKING ENDS %%%%

            //%%% MC2 DOWN STARTS %%%%
            else if (mc2Status == McStatus.DOWN)
            {
                mc2AlarmSiren.Value = true;
                mc2Lights.FailingLights();

                if (eventsMc == Events.r2)
                {
                    mc2Failed = false;
                    mc2Status = McStatus.IDLE;
                    mc2AlarmSiren.Value = false;
                }
            }
            //%%% MC2 DOWN ENDS %%%%

            }
            // %%%%%%%%%%%%%%%%%%%% MC2 ENDS %%%%%%%%%%%%%%%%%%%%
            }
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MACHINE CENTER ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FINISHED PIECE STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (ftAtFinishedPieceExit.Q == true)
            {
                conveyorFinishedPiece.Value = false;
            }
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FINISHED PIECE ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        }
    }
}