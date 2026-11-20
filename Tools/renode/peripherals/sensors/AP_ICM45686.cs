//
// ICM-45686 IMU as AP_InertialSensor_Invensensev3 drives it. The ICM-456xy
// family uses a different user-bank map from the ICM-426xx parts:
// WHO_AM_I at 0x72, FIFO count at 0x12/0x13 and FIFO data at 0x14, a soft
// reset through REG_MISC2, and an indirect register window (IREG at
// 0x7C-0x7E) for the IPREG banks. The model supplies that map, a reset-done
// flag, the IREG window, and 16-byte or 20-byte (FIFO_HIRES_EN) little-endian
// FIFO packets at the configured accel ODR. Full scale follows the
// ACCEL_UI_FS_SEL/GYRO_UI_FS_SEL fields (32g/4000dps at reset). Samples follow
// physics truth after applying the board's sensor rotation; temperature
// follows the board thermal model.
//
// Only the packet layout the driver uses is modelled: accel and gyro both
// enabled, with the timestamp field selected through SMC_CONTROL_0.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // IGPIOReceiver provides chip-select transaction framing in addition to
    // the SPI model's transfer completion, as for AP_ICM42688.
    public class AP_ICM45686 : ISPIPeripheral, IGPIOReceiver
    {
        public AP_ICM45686(IMachine machine, byte whoAmI = DefaultWhoAmI, byte rotation = 8)
        {
            this.whoAmI = whoAmI;
            this.rotation = rotation;
            physics = AP_PhysicsState.ForMachine(machine);
            thermal = AP_BoardThermalState.ForMachine(machine);
            fifo = new Queue<byte>();
            registers = new byte[RegisterCount];
            indirect = new Dictionary<ushort, byte>();
            sampleTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "icm45686 odr",
                                         limit: OdrLimit(DefaultOdr), direction: Direction.Ascending,
                                         enabled: true, workMode: WorkMode.Periodic, eventEnabled: true);
            sampleTimer.LimitReached += OnSampleTick;
            Reset();
        }

        // Fault injection: while set, the IMU stops producing FIFO samples, as
        // if it had failed, but still answers register accesses.
        public bool SuppressOutput { get; set; }

        public void Reset()
        {
            SoftReset();
            transferByte = 0;
            currentRegister = 0;
            reading = false;
        }

        public byte Transmit(byte value)
        {
            byte response = 0;
            if(transferByte == 0)
            {
                reading = (value & ReadFlag) != 0;
                currentRegister = (byte)(value & RegisterMask);
            }
            else if(reading)
            {
                response = ReadRegister(currentRegister);
                Advance();
            }
            else
            {
                WriteRegister(currentRegister, value);
                Advance();
            }
            transferByte++;
            return response;
        }

        public void FinishTransmission()
        {
            transferByte = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                transferByte = 0;
            }
        }

        // FIFO_DATA and IREG_DATA are data ports: burst accesses stay on them
        private void Advance()
        {
            if(currentRegister != FifoData && currentRegister != IregData)
            {
                currentRegister = (byte)((currentRegister + 1) & RegisterMask);
            }
        }

        private void SoftReset()
        {
            Array.Clear(registers, 0, registers.Length);
            indirect.Clear();
            fifo.Clear();
            timestamp = 0;
            latchedFifoCount = 0;
            registers[AccelConfig0] = DefaultOdr;
            registers[GyroConfig0] = DefaultOdr;
            registers[WhoAmI] = whoAmI;
            // reset_done_int: power-up or soft reset has completed
            resetDone = true;
            sampleTimer.Limit = OdrLimit(DefaultOdr);
        }

        private byte ReadRegister(byte register)
        {
            switch(register)
            {
            case FifoCount0:
                // the driver reads the 16-bit record count low byte first
                latchedFifoCount = FifoRecords;
                return (byte)latchedFifoCount;
            case FifoCount1:
                return (byte)(latchedFifoCount >> 8);
            case FifoData:
                return fifo.Count > 0 ? fifo.Dequeue() : (byte)0xFF;
            case Int1Status0:
            {
                // status bits clear on read
                var status = resetDone ? ResetDoneInterrupt : (byte)0;
                resetDone = false;
                return status;
            }
            case IregData:
            {
                indirect.TryGetValue(IregAddress, out var data);
                IncrementIregAddress();
                return data;
            }
            case RegMisc2:
                // IREG_DONE: indirect accesses complete immediately
                return IregDone;
            }
            return registers[register];
        }

        private void WriteRegister(byte register, byte value)
        {
            switch(register)
            {
            case FifoData:
            case FifoCount0:
            case FifoCount1:
            case Int1Status0:
            case WhoAmI:
                return;
            case RegMisc2:
                if((value & SoftResetBit) != 0)
                {
                    SoftReset();
                }
                return;
            case IregData:
                indirect[IregAddress] = value;
                IncrementIregAddress();
                return;
            case FifoConfig2:
                if((value & FifoFlush) != 0)
                {
                    fifo.Clear();
                }
                registers[register] = (byte)(value & ~FifoFlush);
                return;
            case AccelConfig0:
                registers[register] = value;
                sampleTimer.Limit = OdrLimit((byte)(value & OdrMask));
                return;
            }
            registers[register] = value;
        }

        private ushort IregAddress =>
            (ushort)(registers[IregAddrH] << 8 | registers[IregAddrL]);

        private void IncrementIregAddress()
        {
            var next = (ushort)(IregAddress + 1);
            registers[IregAddrH] = (byte)(next >> 8);
            registers[IregAddrL] = (byte)next;
        }

        private void OnSampleTick()
        {
            if(SuppressOutput)
            {
                return;
            }
            var config3 = registers[FifoConfig3];
            if((config3 & FifoSensorsEnabled) != FifoSensorsEnabled ||
               (registers[PwrMgmt0] & SensorsLowNoise) != SensorsLowNoise)
            {
                return;
            }
            var mode = registers[FifoConfig0] >> 6;
            if(mode == FifoModeBypass)
            {
                fifo.Clear();
                return;
            }
            var sampleSize = CurrentSampleSize;
            var capacity = FifoCapacity;
            if(fifo.Count + sampleSize > capacity)
            {
                if(mode != FifoModeStream)
                {
                    return; // stop-on-full
                }
                while(fifo.Count + sampleSize > capacity)
                {
                    fifo.Dequeue();
                }
            }

            var truth = physics.Current;
            var acceleration = AP_SensorOrientation.BodyToSensor(truth.SpecificForceMS2, rotation);
            var gyro = AP_SensorOrientation.BodyToSensor(truth.GyroRadS, rotation);
            if(HighResolutionEnabled)
            {
                PushHighResolutionSample(acceleration, gyro);
            }
            else
            {
                PushSample(acceleration, gyro);
            }
        }

        private void PushSample(float[] acceleration, float[] gyro)
        {
            fifo.Enqueue(Header);
            for(var axis = 0; axis < 3; axis++)
            {
                PushWord(ScaleWord(acceleration[axis], AccelScale));
            }
            for(var axis = 0; axis < 3; axis++)
            {
                PushWord(ScaleWord(gyro[axis], GyroScale));
            }
            // 8-bit temperature, 2 LSB/C around 25C
            fifo.Enqueue((byte)(sbyte)Math.Max(sbyte.MinValue, Math.Min(sbyte.MaxValue,
                Math.Round((thermal.TemperatureC - 25.0) * 2.0))));
            PushWord((short)timestamp++);
        }

        private void PushHighResolutionSample(float[] acceleration, float[] gyro)
        {
            fifo.Enqueue(Header);
            var values = new int[6];
            for(var axis = 0; axis < 3; axis++)
            {
                values[axis] = ScaleHighResolution(acceleration[axis], AccelScale / 16.0);
                values[axis + 3] = ScaleHighResolution(gyro[axis], GyroScale / 16.0);
            }
            foreach(var value in values)
            {
                fifo.Enqueue((byte)(value >> 4));
                fifo.Enqueue((byte)(value >> 12));
            }
            // 16-bit temperature, 128 LSB/C around 25C
            PushWord(ScaleWord((float)(thermal.TemperatureC - 25.0), 1.0 / 128.0));
            PushWord((short)timestamp++);
            fifo.Enqueue((byte)((values[3] & 0xF) | (values[0] & 0xF) << 4));
            fifo.Enqueue((byte)((values[4] & 0xF) | (values[1] & 0xF) << 4));
            fifo.Enqueue((byte)((values[5] & 0xF) | (values[2] & 0xF) << 4));
        }

        // -32768 (0x8000) is the ICM-456xy invalid-sample marker, so valid
        // samples saturate one count short of it
        private static short ScaleWord(float value, double scale)
        {
            return (short)Math.Max(Int16.MinValue + 1,
                Math.Min(Int16.MaxValue, Math.Round(value / scale)));
        }

        // likewise an upper word of 0x8000 marks an invalid 20-bit sample
        private static int ScaleHighResolution(float value, double scale)
        {
            return (int)Math.Max(-524272,
                Math.Min(524287, Math.Round(value / scale)));
        }

        private void PushWord(short value)
        {
            fifo.Enqueue((byte)(value & 0xFF));
            fifo.Enqueue((byte)((value >> 8) & 0xFF));
        }

        // ODR codes 0x3 (6.4kHz) to 0xF (1.5625Hz) halve the rate per step;
        // the timer runs at 64MHz so every period is a whole number of ticks
        private static ulong OdrLimit(byte code)
        {
            if(code < 0x3)
            {
                code = 0x3;
            }
            return (ulong)(TimerFrequency / 6400) << (code - 0x3);
        }

        private byte Header
        {
            get
            {
                var header = HeaderAccel | HeaderGyro;
                if(HighResolutionEnabled)
                {
                    header |= HeaderHighResolution;
                }
                indirect.TryGetValue(SmcControl0, out var smc);
                if((smc & TimestampEnable) != 0)
                {
                    header |= HeaderTimestamp;
                }
                return (byte)header;
            }
        }

        // ACCEL_UI_FS_SEL 0..4 is 32g down to 2g; GYRO_UI_FS_SEL 0..8 is
        // 4000dps down to 15.625dps
        private double AccelScale =>
            Gravity * (32.0 / (1 << Math.Min(4, (registers[AccelConfig0] >> 4) & 0x7))) / 32768.0;
        private double GyroScale =>
            Math.PI / 180.0 * (4000.0 / (1 << Math.Min(8, registers[GyroConfig0] >> 4))) / 32768.0;

        private bool HighResolutionEnabled => (registers[FifoConfig3] & FifoHighResolutionEnable) != 0;
        private int CurrentSampleSize => HighResolutionEnabled ? HighResolutionSampleSize : SampleSize;
        private int FifoRecords => fifo.Count / CurrentSampleSize;
        // FIFO_DEPTH counts 256-byte blocks above the first (0x07 is 2KB)
        private int FifoCapacity => ((registers[FifoConfig0] & FifoDepthMask) + 1) * 256;

        private readonly Queue<byte> fifo;
        private readonly byte[] registers;
        private readonly Dictionary<ushort, byte> indirect;
        private readonly LimitTimer sampleTimer;
        private readonly byte whoAmI;
        private readonly byte rotation;
        private readonly AP_PhysicsState physics;
        private readonly AP_BoardThermalState thermal;
        private int transferByte;
        private byte currentRegister;
        private bool reading;
        private bool resetDone;
        private int latchedFifoCount;
        private ushort timestamp;

        private const long TimerFrequency = 64000000;
        private const int RegisterCount = 128;
        private const int SampleSize = 16;
        private const int HighResolutionSampleSize = 20;

        // user bank 0
        private const byte PwrMgmt0 = 0x10;
        private const byte FifoCount0 = 0x12;
        private const byte FifoCount1 = 0x13;
        private const byte FifoData = 0x14;
        private const byte Int1Status0 = 0x19;
        private const byte AccelConfig0 = 0x1B;
        private const byte GyroConfig0 = 0x1C;
        private const byte FifoConfig0 = 0x1D;
        private const byte FifoConfig2 = 0x20;
        private const byte FifoConfig3 = 0x21;
        private const byte WhoAmI = 0x72;
        private const byte IregAddrH = 0x7C;
        private const byte IregAddrL = 0x7D;
        private const byte IregData = 0x7E;
        private const byte RegMisc2 = 0x7F;

        // IPREG_TOP1 SMC_CONTROL_0, reached through IREG
        private const ushort SmcControl0 = 0xA258;

        private const byte ReadFlag = 0x80;
        private const byte RegisterMask = 0x7F;
        private const byte DefaultWhoAmI = 0xE9;
        private const byte DefaultOdr = 0x06; // 800Hz
        private const byte OdrMask = 0x0F;
        private const byte ResetDoneInterrupt = 0x80;
        private const byte IregDone = 0x01;
        private const byte SoftResetBit = 0x02;
        private const byte FifoFlush = 0x80;
        private const byte FifoDepthMask = 0x3F;
        private const int FifoModeBypass = 0;
        private const int FifoModeStream = 1;
        private const byte FifoSensorsEnabled = 0x07; // FIFO_IF_EN | GYRO_EN | ACCEL_EN
        private const byte FifoHighResolutionEnable = 0x08;
        private const byte SensorsLowNoise = 0x0F;
        private const byte TimestampEnable = 0x01;
        private const byte HeaderAccel = 0x40;
        private const byte HeaderGyro = 0x20;
        private const byte HeaderHighResolution = 0x10;
        private const byte HeaderTimestamp = 0x08;
        private const double Gravity = 9.80665;
    }
}
