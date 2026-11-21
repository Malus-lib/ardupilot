// Injected sensor faults for fault-handling tests under Renode.
//
// A model that supports faults exposes one string property, Fault, settable
// from the Renode monitor while the firmware runs, for example:
//
//   sysbus.spi1.spi1Mux.imu0 Fault "accel_bias=0,0,0.5"
//   sysbus.spi5.spi5Mux.baro0 Fault "drift=6"
//   sysbus.spi5.spi5Mux.baro0 Fault ""          (clears the fault)
//
// Keys are separated by ';'. Vectors are x,y,z in the sensor frame.
//   IMU:        accel_bias (m/s^2), gyro_bias (deg/s), accel_drift (m/s^2 per s),
//               gyro_drift (deg/s per s), accel_noise (m/s^2, 1 sigma),
//               gyro_noise (deg/s, 1 sigma), freeze (1 repeats the last sample)
//   barometer:  offset (Pa), drift (Pa/s), noise (Pa, 1 sigma), freeze
// Drift grows from the moment the fault is set, in physics time. Reading the
// property returns the last accepted text, so test scripts can confirm it.
//
using System;
using System.Collections.Generic;
using System.Globalization;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public sealed class AP_SensorFault
    {
        public AP_SensorFault(uint seed)
        {
            random = new Random((int)seed);
        }

        public string Spec { get; private set; } = "";

        public bool Active => values.Count > 0;

        public void Set(string spec, ulong nowUs, ICollection<string> allowedKeys)
        {
            var parsed = new Dictionary<string, double[]>();
            foreach(var item in (spec ?? "").Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries))
            {
                var pair = item.Split(new[] { '=' }, 2);
                var key = pair[0].Trim().ToLowerInvariant();
                if(!allowedKeys.Contains(key))
                {
                    throw new ArgumentException(string.Format(
                        "unknown fault key '{0}'; allowed: {1}", key, string.Join(", ", allowedKeys)));
                }
                var text = pair.Length > 1 ? pair[1] : "1";
                var parts = text.Split(',');
                var numbers = new double[parts.Length];
                for(var i = 0; i < parts.Length; i++)
                {
                    numbers[i] = double.Parse(parts[i].Trim(), CultureInfo.InvariantCulture);
                }
                parsed[key] = numbers;
            }
            values = parsed;
            startUs = nowUs;
            Spec = spec ?? "";
            frozen = false;
        }

        // accel in m/s^2 and gyro in rad/s, both in the sensor frame; changed in place
        public void ApplyImu(float[] accel, float[] gyro, ulong nowUs)
        {
            if(!Active)
            {
                return;
            }
            if(Has("freeze") && values["freeze"][0] != 0)
            {
                if(frozen)
                {
                    Array.Copy(frozenAccel, accel, 3);
                    Array.Copy(frozenGyro, gyro, 3);
                    return;
                }
            }
            var elapsed = Elapsed(nowUs);
            for(var axis = 0; axis < 3; axis++)
            {
                accel[axis] += (float)(Component("accel_bias", axis) +
                    Component("accel_drift", axis) * elapsed +
                    Gaussian(Scalar("accel_noise")));
                gyro[axis] += (float)((Component("gyro_bias", axis) +
                    Component("gyro_drift", axis) * elapsed +
                    Gaussian(Scalar("gyro_noise"))) * DegToRad);
            }
            if(Has("freeze") && values["freeze"][0] != 0)
            {
                Array.Copy(accel, frozenAccel, 3);
                Array.Copy(gyro, frozenGyro, 3);
                frozen = true;
            }
        }

        public double ApplyPressure(double pressurePa, ulong nowUs)
        {
            if(!Active)
            {
                return pressurePa;
            }
            if(Has("freeze") && values["freeze"][0] != 0)
            {
                if(!frozen)
                {
                    frozenPressure = pressurePa;
                    frozen = true;
                }
                return frozenPressure;
            }
            return pressurePa + Scalar("offset") + Scalar("drift") * Elapsed(nowUs) +
                Gaussian(Scalar("noise"));
        }

        public static readonly string[] ImuKeys = {
            "accel_bias", "gyro_bias", "accel_drift", "gyro_drift",
            "accel_noise", "gyro_noise", "freeze" };
        public static readonly string[] BarometerKeys = { "offset", "drift", "noise", "freeze" };

        private bool Has(string key) => values.ContainsKey(key);

        private double Scalar(string key) => Has(key) ? values[key][0] : 0.0;

        private double Component(string key, int axis)
        {
            if(!Has(key))
            {
                return 0.0;
            }
            var v = values[key];
            return v.Length == 1 ? (axis == 0 ? v[0] : 0.0) : (axis < v.Length ? v[axis] : 0.0);
        }

        private double Elapsed(ulong nowUs) => nowUs > startUs ? (nowUs - startUs) * 1e-6 : 0.0;

        private double Gaussian(double sigma)
        {
            if(sigma <= 0)
            {
                return 0.0;
            }
            // Box-Muller, deterministic for a given seed
            var u1 = 1.0 - random.NextDouble();
            var u2 = random.NextDouble();
            return sigma * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        private readonly Random random;
        private Dictionary<string, double[]> values = new Dictionary<string, double[]>();
        private ulong startUs;
        private bool frozen;
        private readonly float[] frozenAccel = new float[3];
        private readonly float[] frozenGyro = new float[3];
        private double frozenPressure;

        private const double DegToRad = Math.PI / 180.0;
    }

    public interface IAP_BarometerFault
    {
        string Fault { get; set; }
    }
}
