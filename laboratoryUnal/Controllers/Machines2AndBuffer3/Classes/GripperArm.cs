using EngineIO;
using System;

namespace Controllers.Scenes.MachinesAndBuffer
{
    class GripperArm
    {
        private readonly MemoryFloat posX;
        private readonly MemoryFloat posZ;
        private readonly MemoryFloat setX;
        private readonly MemoryFloat setZ;
        public readonly MemoryBit grab;

        private GripperStatus gripperStatus = GripperStatus.IDLE;
        private GripperStep gripperStep = GripperStep.INITIAL;
        private int timeVibrationGripper;

        private enum GripperStatus
        {
            IDLE,
            WORKING,
            DOWN
        }
        private enum GripperStep
        {
            INITIAL,
            DOWN_LOOK_FOR_PART,
            GRAB,
            UP_WITH_PART,
            X_EXTEND,
            DOWN_WITH_PART,
            RELEASE,
            UP_NO_PART,
            X_RETRACT,
            DOWN_VIBRATING
        }

        public GripperArm(MemoryFloat posX, MemoryFloat posZ, MemoryFloat setX, MemoryFloat setZ, MemoryBit grab)
        {
            this.posX = posX;
            this.posZ = posZ;
            this.setX = setX;
            this.setZ = setZ;
            this.grab = grab;
        }

        public void Start()
        {
            if (gripperStep == GripperStep.INITIAL && gripperStatus != GripperStatus.DOWN)
            {
                gripperStatus = GripperStatus.WORKING;
            }
        }

        public void Fail()
        {
            gripperStatus = GripperStatus.DOWN;
            gripperStep = GripperStep.DOWN_VIBRATING;
        }

        public void Repair()
        {
            if (gripperStatus == GripperStatus.DOWN)
            {
                setZ.Value = 0.0f;
                setX.Value = 0.0f;
                gripperStatus = GripperStatus.IDLE;
            } 
        }

        public void StateTransition()
        {
            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRIPPER STARTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            // %%%%%%%%%% gripper status: idle, working or down %%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if (gripperStatus == GripperStatus.IDLE)// %%%%%%%%%% IF GRIPPER IS IDLE %%%%%%%
            {
                gripperStep = GripperStep.INITIAL;
                grab.Value = false;
            }
            else if (gripperStatus == GripperStatus.WORKING)// %% IF GRIPPER IS WORKING %%%%
            {
                if (gripperStep == GripperStep.INITIAL) //Gripper going to initial position.
                {
                    setZ.Value = 0.0f;
                    if (posZ.Value < 0.5f)
                    {
                        setX.Value = 0.0f;
                    }
                    if (posX.Value < 0.01f && posZ.Value < 0.01f)
                    {
                        gripperStep = GripperStep.DOWN_LOOK_FOR_PART;
                    }
                }
                else if (gripperStep == GripperStep.DOWN_LOOK_FOR_PART)//Gripper in initial position. Z descending.
                {
                    setZ.Value = 9.4f;//Distance descended in Z until piece is reached                
                    if (posZ.Value > 9.2f)
                    {
                        gripperStep = GripperStep.GRAB;
                    }
                }
                else if (gripperStep == GripperStep.GRAB)
                {
                    grab.Value = true;// Z descended. Grabbing piece.
                    gripperStep = GripperStep.UP_WITH_PART;
                }
                else if (gripperStep == GripperStep.UP_WITH_PART)// Piece grabbed. Z ascending with part.
                {
                    setZ.Value = 0.0f;
                    if (posZ.Value < 0.1f)
                    {
                        gripperStep = GripperStep.X_EXTEND;// Z ascended with part.
                    }
                }
                else if (gripperStep == GripperStep.X_EXTEND)// Z ascended with part. X extending with part
                {
                    setX.Value = 7.7f;
                    if (posX.Value > 7.6f)//X extended with part
                    {
                        gripperStep = GripperStep.DOWN_WITH_PART;
                    }
                }
                else if (gripperStep == GripperStep.DOWN_WITH_PART)//X extended with part. Z descending with part.
                {
                    setZ.Value = 9.3f;
                    if (posZ.Value > 9.2f)//Z descended with part
                    {
                        gripperStep = GripperStep.RELEASE;
                    }
                }
                else if (gripperStep == GripperStep.RELEASE)//Z descended with part. Gripper releases part.
                {
                    grab.Value = false;
                    gripperStep = GripperStep.UP_NO_PART;//Gripper released part.                    
                }
                else if (gripperStep == GripperStep.UP_NO_PART)//Gripper released part. Z ascend w/out part.
                {
                    setZ.Value = 0.0f;
                    if (posZ.Value < 0.1f)
                    {
                        gripperStep = GripperStep.X_RETRACT;//Z ascended w/out part.
                    }
                }
                else if (gripperStep == GripperStep.X_RETRACT)//Z ascended w/out part. X retracting.
                {
                    setX.Value = 0.0f;
                    if (posX.Value < 0.1f)
                    {
                        gripperStep = GripperStep.INITIAL;//X retracting. Going for initial state.
                        gripperStatus = GripperStatus.IDLE;
                    }
                }
            }
            else if (gripperStatus == GripperStatus.DOWN) // %%%%%%%%%%%%%%%%% GRIPPER IS DOWN %%%%%%%%%%%%%%%%
            {
                if (gripperStep == GripperStep.DOWN_VIBRATING) // %%%%%%%%%%%%%%%%% DOWN VIBRATION STARTS %%%%%%%%%%%%%%%%
                {
                    if (timeVibrationGripper == 1)
                    {
                        setX.Value = 10.0f;
                        setZ.Value = 10.0f;
                    }
                    else if (timeVibrationGripper == 6)
                    {
                        setX.Value = 0.0f;
                        setZ.Value = 0.0f;
                    }
                    else if (timeVibrationGripper == 10)
                    {
                        timeVibrationGripper = 0;
                    }
                    timeVibrationGripper++;
                }// %%%%%%%%%%%%%%%%% DOWN VIBRATION ENDS %%%%%%%%%%%%%%%%
            }

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRIPPER ENDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        }
    }
}
