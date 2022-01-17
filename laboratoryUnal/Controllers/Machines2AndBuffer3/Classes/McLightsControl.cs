using EngineIO;

namespace Controllers.Scenes.MachinesAndBuffer
{
    class McLightsControl
    {
        private readonly MemoryBit mcRedLight;
        private readonly MemoryBit mcYellowLight;
        private readonly MemoryBit mcGreenLight;
        private int timeBlinkingLights;

        public McLightsControl(MemoryBit mcRedLight, MemoryBit mcYellowLight, MemoryBit mcGreenLight)
        {
            this.mcRedLight = mcRedLight;
            this.mcYellowLight = mcYellowLight;
            this.mcGreenLight = mcGreenLight;
            timeBlinkingLights = 0;
        }

        public void FailingLights()
        {
            if (timeBlinkingLights < 30)
            {
                mcGreenLight.Value = false;
                mcYellowLight.Value = false;
                mcRedLight.Value = false;
            }
            else if (timeBlinkingLights < 60)
            {
                mcGreenLight.Value = true;
                mcYellowLight.Value = true;
                mcRedLight.Value = true;
            }
            else if (timeBlinkingLights >= 60)
            {
                timeBlinkingLights = 0;
            }
            timeBlinkingLights++;
        }

        public void WorkingLights()
        {
            if (timeBlinkingLights < 60)
            {
                mcGreenLight.Value = true;
                mcYellowLight.Value = false;
                mcRedLight.Value = false;
            }
            else if (timeBlinkingLights < 120)
            {
                mcGreenLight.Value = true;
                mcYellowLight.Value = true;
                mcRedLight.Value = false;
            }
            else if (timeBlinkingLights >= 120)
            {
                timeBlinkingLights = 0;
            }
            timeBlinkingLights++;
        }
        public void IdleLights()
        {
            mcGreenLight.Value = true;
            mcYellowLight.Value = false;
            mcRedLight.Value = false;
        }
    }
}
