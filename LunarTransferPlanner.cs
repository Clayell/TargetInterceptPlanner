/*---------------------------------------------------------------------------
License: The code is subject to the MIT license (see below).
------------------------------------------------
Copyright (c) 2022 RCrockford

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
---------------------------------------------------------------------------*/

using Contracts.Agents.Mentalities;
using KerbalConstructionTime;
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics; // remove this
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;
using static KSP.UI.Screens.Settings.SettingsSetup;
using static ProceduralSpaceObject;
using static SpaceObjectCollider;
using static UnityEngine.GraphicsBuffer;
using Debug = UnityEngine.Debug; // remove this

namespace LunarTransferPlanner
{
    public static class Util
    {
        public static Vector3 x = Vector3.right;
        public static Vector3 y = Vector3.up;
        public static Vector3 z = Vector3.forward;
        public static void TryReadValue<T>(ref T target, ConfigNode node, string name)
        {
            if (node.HasValue(name))
            {
                try
                {
                    target = (T)TypeDescriptor.GetConverter(typeof(T)).ConvertFromString(node.GetValue(name));
                }
                catch
                {
                    // just skip over it
                }
            }
            // skip again
        }

        // Math.Acosh does not seem to work in .NET 4
        public static double Acosh(double x)
        {
            if (x >= 1)
            {
                return Math.Log(x + Math.Sqrt(x * x - 1));
            }
            else
            {
                return Double.NaN;
            }

        }
    }



    [KSPAddon(KSPAddon.Startup.FlightAndKSC, false)]
    public class LunarTransferPlanner : DaMichelToolbarSuperWrapper.PluginWithToolbarSupport
    {
        Rect windowRect = new Rect(100, 100, -1, -1);
        bool isWindowOpen = true;

        int currentBody = 0;
        CelestialBody target = null;
        Vector3d launchPos;
        double inclination;
        bool PrincipiaInstalled;
        bool KACInstalled;
        double currentUT;
        bool useAltSkin = false; // Use Unity GUI skin instead of default
        double flightTime = 4; // Desired flight time after maneuver, in days
        double tliAltitudeKM = 200; // Parking orbit altitude (circular orbit assumed)
        bool useAltBehavior = false; // Find global minimum for low latitudes instead of local minimum
        bool useVesselPosition = false; // Use vessel position for latitude instead of launch site position
        const double EarthSiderealDay = 86164.0905;
        double latitude = 0d;
        double longitude = 0d;
        bool expandLatLong = false; // Expand/collapse custom Latitude/Longitude specifier
        bool expandAltitude = false; // Expand/collapse parking orbit altitude changer
        bool expandParking0 = false; // Expand/collapse time in parking orbit for launch now
        bool expandParking1 = false; // Expand/collapse time in parking orbit for first window
        bool expandParking2 = false; // Expand/collapse time in parking orbit for second window
        bool showPhasing = false; // Show the phasing angle instead of the time in parking orbit, applies to all boxes

        bool specialWarp1 = false;
        bool specialWarpBuffer = false;
        bool specialWarp2 = false;
        bool specialWarp3 = false;
        bool specialWarpWait = false;
        double waitingTime;

        bool targetSet = false;
        double warpMargin = 60; // Time difference from the launch window that the warp will stop at
        string windowTitle = "";
        GUISkin skin;

        readonly List<(CelestialBody target, Vector3d launchPos, double inclination, double absoluteLaunchTime)> cache = new List<(CelestialBody, Vector3d, double, double)>();
        readonly static Dictionary<string, double> nextTickMap = new Dictionary<string, double>();
        readonly static Dictionary<string, string> textBuffer = new Dictionary<string, string>();
        readonly string SettingsPath = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/LunarTransferPlanner/Plugins/PluginData/settings.cfg");

        #region boring stuff
        protected override DaMichelToolbarSuperWrapper.ToolbarInfo GetToolbarInfo()
        {
            return new DaMichelToolbarSuperWrapper.ToolbarInfo
            {
                name = "LunarTransferPlanner",
                tooltip = "LunarTransferPlanner Show/Hide Gui",
                toolbarTexture = "LunarTransferPlanner/toolbarbutton",
                launcherTexture = "LunarTransferPlanner/launcherbutton",
                visibleInScenes = new GameScenes[] { GameScenes.FLIGHT, GameScenes.SPACECENTER } // need to add tracking station
            };
        }

        void Awake()
        {
            skin = (GUISkin)GUISkin.Instantiate(HighLogic.Skin);
            skin.button.padding = new RectOffset(2, 2, 2, 2);
            skin.button.margin = new RectOffset(1, 1, 1, 1);
            skin.box.padding = new RectOffset(2, 2, 2, 2);
            skin.box.margin = new RectOffset(1, 1, 1, 1);
            skin.textField.margin = new RectOffset(3, 1, 1, 1);
            skin.textField.padding = new RectOffset(4, 2, 1, 0);

            LoadSettings();
            InitializeToolbars();
            OnGuiVisibilityChange();
        }

        public void Start()
        {
            KACWrapper.InitKACWrapper();
            KACInstalled = KACWrapper.APIReady;
            PrincipiaWrapper.InitPrincipiaWrapper();
            PrincipiaInstalled = PrincipiaWrapper.APIReady;
            Tooltip.RecreateInstance(); // Need to make sure that a new Tooltip instance is created after every scene change
        }

        public void OnDestroy()
        {
            SaveSettings();
            TearDownToolbars();
        }


        protected override void OnGuiVisibilityChange()
        {
            isWindowOpen = isGuiVisible;
        }

        private void SaveSettings()
        {
            ConfigNode settings = new ConfigNode("SETTINGS");
            SaveMutableToolbarSettings(settings);
            SaveImmutableToolbarSettings(settings);
            settings.AddValue("windowRect.xMin", windowRect.xMin);
            settings.AddValue("windowRect.yMin", windowRect.yMin);
            settings.AddValue("useAltSkin", useAltSkin);
            settings.AddValue("flightTime", flightTime);
            settings.AddValue("tliAltitudeKM", tliAltitudeKM);
            settings.AddValue("useAltBehavior", useAltBehavior);
            settings.AddValue("useVesselPosition", useVesselPosition);
            settings.AddValue("latitude", latitude);
            settings.AddValue("longitude", longitude);
            settings.AddValue("expandLatLong", expandLatLong);
            settings.AddValue("expandAltitude", expandAltitude);
            settings.AddValue("expandParking0", expandParking0);
            settings.AddValue("expandParking1", expandParking1);
            settings.AddValue("expandParking2", expandParking2);
            settings.AddValue("showPhasing", showPhasing);
            settings.AddValue("warpMargin", warpMargin);

            ConfigNode root = new ConfigNode();
            root.AddNode(settings);
            root.Save(SettingsPath);
        }


        private void LoadSettings()
        {
            ConfigNode root = ConfigNode.Load(SettingsPath);
            if (root != null)
            {
                ConfigNode settings = root.GetNode("SETTINGS");
                if (settings != null)
                {
                    float x = windowRect.xMin, y = windowRect.yMin;
                    Util.TryReadValue(ref x, settings, "windowRect.xMin");
                    Util.TryReadValue(ref y, settings, "windowRect.yMin");
                    Util.TryReadValue(ref useAltSkin, settings, "useAltSkin");
                    Util.TryReadValue(ref flightTime, settings, "flightTime");
                    Util.TryReadValue(ref tliAltitudeKM, settings, "tliAltitudeKM");
                    Util.TryReadValue(ref useAltBehavior, settings, "useAltBehavior");
                    Util.TryReadValue(ref useVesselPosition, settings, "useVesselPosition");
                    Util.TryReadValue(ref latitude, settings, "latitude");
                    Util.TryReadValue(ref longitude, settings, "longitude");
                    Util.TryReadValue(ref expandLatLong, settings, "expandLatLong");
                    Util.TryReadValue(ref expandAltitude, settings, "expandAltitude");
                    Util.TryReadValue(ref expandParking0, settings, "expandParking0");
                    Util.TryReadValue(ref expandParking1, settings, "expandParking1");
                    Util.TryReadValue(ref expandParking2, settings, "expandParking2");
                    Util.TryReadValue(ref showPhasing, settings, "showPhasing");
                    Util.TryReadValue(ref warpMargin, settings, "warpMargin");
                    windowRect = new Rect(x, y, windowRect.width, windowRect.height);
                    LoadMutableToolbarSettings(settings);
                    LoadImmutableToolbarSettings(settings);
                }
            }
        }
        #endregion

        void OnGUI()
        {
            if (isWindowOpen && buttonVisible)
            {
                GUI.skin = !useAltSkin ? this.skin : null;
                //GUI.skin = this.skin;
                windowRect = GUILayout.Window(this.GetHashCode(), windowRect, MakeMainWindow, windowTitle);
                float left = Mathf.Clamp(windowRect.x, 0, Screen.width - windowRect.width);
                float top = Mathf.Clamp(windowRect.y, 0, Screen.height - windowRect.height);
                windowRect = new Rect(left, top, windowRect.width, windowRect.height);
                Tooltip.Instance.ShowTooltip(this.GetHashCode());
            }
        }

        private void MakeNumberEditField(string controlId, ref double value, double step, double minValue, double maxValue)
        {
            // retrieve tick time
            if (!nextTickMap.TryGetValue(controlId, out double nextTick))
                nextTick = 0;

            // retrieve text buffer
            if (!textBuffer.TryGetValue(controlId, out string textValue))
                textValue = value.ToString("G17");

            if (double.TryParse(textValue, out double parsedBufferValue))
            {
                if (Math.Abs(parsedBufferValue - value) > 1e-9)
                {
                    // external change detected, update buffer
                    textValue = value.ToString("G17");
                    textBuffer[controlId] = textValue;
                }
            }
            else
            {
                // invalid buffer, resync
                textValue = value.ToString("G17");
                textBuffer[controlId] = textValue;
            }

            GUILayout.BeginHorizontal();

            string newlabel = GUILayout.TextField(textValue, GUILayout.MinWidth(40));

            // if text changed, update buffer and try to parse value
            if (newlabel != textValue)
            {
                textBuffer[controlId] = newlabel;

                if (double.TryParse(newlabel, out double newvalue))
                {
                    value = Math.Max(minValue, Math.Min(maxValue, newvalue));
                }
            }
            
            bool hitMinusButton = GUILayout.RepeatButton("\u2013", GUILayout.MinWidth(40));
            bool hitPlusButton = GUILayout.RepeatButton("+", GUILayout.MinWidth(40));

            if (hitPlusButton || hitMinusButton)
            {
                double tick = Time.realtimeSinceStartup;
                if (tick > nextTick)
                {
                    if (hitMinusButton)
                    {

                        double snappedValue = Math.Floor(value / step) * step;
                        if (Math.Abs(value - snappedValue) > 1e-9)
                            value = Math.Max(minValue, snappedValue); // snap to next number
                        else
                            value = Math.Max(minValue, value - step); // then decrement
                    }
                    else
                    {
                        double snappedValue = Math.Ceiling(value / step) * step;
                        if (Math.Abs(value - snappedValue) > 1e-9)
                            value = Math.Min(maxValue, snappedValue); // snap to next number
                        else
                            value = Math.Min(maxValue, value + step); // then increment
                    }

                    int decimals = Math.Max(0, (int)Math.Ceiling(-Math.Log10(step))); // avoid annoying floating point issues when rounding
                    value = Math.Round(value, decimals);
                    nextTickMap[controlId] = tick + 0.1d; // 0.1s wait before incrementing again
                    textBuffer[controlId] = value.ToString($"F{decimals}");
                }
            }

            GUILayout.EndHorizontal();
        }


        public static PQSCity FindKSC(CelestialBody home)
        {
            if (home != null)
            {
                if (home.pqsController != null && home.pqsController.transform != null)
                {
                    Transform t = home.pqsController.transform.Find("KSC");
                    if (t != null)
                    {
                        PQSCity KSC = (PQSCity)t.GetComponent(typeof(PQSCity));
                        if (KSC != null) { return KSC; }
                    }
                }
            }

            PQSCity[] cities = Resources.FindObjectsOfTypeAll<PQSCity>();
            foreach (PQSCity c in cities)
            {
                if (c.name == "KSC")
                {
                    return c;
                }
            }

            return null;
        }

        private Vector3d GetLaunchPos(CelestialBody mainBody, ref double Latitude, ref double Longitude, bool useVesselPosition)
        {
            //if (useVesselPosition && HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null)
            if (useVesselPosition) // all checks are safe, can go ahead
            {
                Vessel vessel = FlightGlobals.ActiveVessel;
                Latitude = vessel.latitude;
                Longitude = vessel.longitude;
                return mainBody.GetWorldSurfacePosition(vessel.latitude, vessel.longitude, 0);
            }

            if (SpaceCenter.Instance != null)
            {
                PQSCity ksc = FindKSC(FlightGlobals.GetHomeBody());
                if (ksc)
                {
                    Latitude = ksc.lat;
                    Longitude = ksc.lon;
                    return mainBody.GetWorldSurfacePosition(ksc.lat, ksc.lon, 0);
                }
                else
                {
                    Latitude = SpaceCenter.Instance.Latitude;
                    Longitude = SpaceCenter.Instance.Longitude;
                    return mainBody.GetWorldSurfacePosition(SpaceCenter.Instance.Latitude, SpaceCenter.Instance.Longitude, 0);
                }
            }

            return Vector3d.zero;
        }

        private double GetTargetInclination(object orbit)
        {
            if (orbit != null && PrincipiaInstalled)
            {
                return PrincipiaWrapper.Reflection.GetMemberValue<double>(orbit, "inclination");
            }
            else if (orbit != null && orbit is Orbit stockOrbit)
            {
                return stockOrbit.inclination;
            }
            else
            {
                //Debug.Log("WARNING: Invalid or unsupported orbit object type: " + orbit?.GetType().FullName);
                throw new ArgumentException("Invalid orbit passed to GetTargetInclination.");
            }
        }


        //private Vector3d GetPositionAtTime(CelestialBody body, double time)
        //{
        //    if (body != null && PrincipiaInstalled)
        //    {
        //        Debug.Log("GetPositionAtTime: Principia Active");
        //        var xyz = PrincipiaWrapper.CelestialGetPosition(body.flightGlobalsIndex, time);

        //        double x = PrincipiaWrapper.Reflection.GetMemberValue<double>(xyz, "x");
        //        double y = PrincipiaWrapper.Reflection.GetMemberValue<double>(xyz, "y");
        //        double z = PrincipiaWrapper.Reflection.GetMemberValue<double>(xyz, "z");

        //        return new Vector3d(x, y, z);
        //    }
        //    else if (body != null && body is CelestialBody stockBody)
        //    {
        //        Debug.Log("GetPositionAtTime: Principia Inactive");
        //        return stockBody.getPositionAtUT(time);
        //    }
        //    else
        //    {
        //        throw new ArgumentNullException("Invalid body passed to GetPositionAtTime");
        //    }
        //}



        private readonly struct OrbitData
        {
            public OrbitData(Vector3d n, double i, double a)
            {
                normal = n;
                inclination = i;
                azimuth = a;
            }

            public readonly Vector3d normal;
            public readonly double inclination;
            public readonly double azimuth;
        }

        private OrbitData CalcOrbitForTime(CelestialBody target, Vector3d launchPos, double delayTime)
        {
            // Form a plane with the launch site, moon and earth centre in, use this as the orbital plane for launch
            CelestialBody mainBody = target.referenceBody;
            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = Planetarium.GetUniversalTime() + flightTime * mainBody.solarDayLength + delayTime;
            Vector3d targetPos = target.getPositionAtUT(targetTime); // this doesn't take into account changing target inclination due to principia
            //Vector3d targetPos = GetPositionAtTime(target, targetTime);

            Vector3d upVector = QuaternionD.AngleAxis(delayTime * 360d / mainBody.rotationPeriod, MainAxis) * (launchPos - MainPos).normalized; // use rotationPeriod for sidereal time

            Vector3d orbitNorm = Vector3d.Cross(targetPos - MainPos, upVector).normalized;
            double inclination = Math.Acos(Vector3d.Dot(orbitNorm, MainAxis)); // inclination of the launch orbit, not the target orbit
            if (inclination > Math.PI / 2)
            {
                inclination = Math.PI - inclination;
                orbitNorm *= -1; // make sure orbitNorm always points roughly northwards
            }

            // When checking this: remember that Unity (and KSP) use a left-handed coordinate system; therefore, the
            // cross product follows the left-hand rule.
            Vector3d eastVec = Vector3d.Cross(upVector, MainAxis).normalized;
            Vector3d northVec = Vector3d.Cross(eastVec, upVector).normalized;
            Vector3d launchVec = Vector3d.Cross(upVector, orbitNorm).normalized;

            double azimuth = Math.Acos(Vector3d.Dot(launchVec, northVec));

            return new OrbitData(orbitNorm, inclination * 180d / Math.PI, azimuth * 180d / Math.PI);
        }

        private double EstimateLaunchTime(CelestialBody target, Vector3d launchPos, double startTime, bool useAltBehavior)
        {
            const double tolerance = 0.01;
            double coarseStep = 1200 * target.referenceBody.rotationPeriod / EarthSiderealDay; // scale based on EarthSiderealDay
            double maxTimeLimit = useAltBehavior ? target.referenceBody.solarDayLength * 30 : target.referenceBody.solarDayLength; // expand to 30 days if global minimum
            const double targetAz = 90d;
            const double epsilon = 1e-9;
            const double buffer = 1.0;

            double AzimuthError(double t)
            {
                double az = CalcOrbitForTime(target, launchPos, t).azimuth;
                return Math.Abs(((az - targetAz + 540) % 360) - 180);
            }

            double GoldenSectionSearch(double lowerBound, double upperBound) // adopted from https://en.wikipedia.org/wiki/Golden-section_search
            {
                // I have no idea why this works, but it works so well
                double invphi = (Math.Sqrt(5) - 1) / 2;  // positive conjugate of golden ratio
                double left = upperBound - invphi * (upperBound - lowerBound);
                double right = lowerBound + invphi * (upperBound - lowerBound);

                double eLeft = AzimuthError(left);
                double eRight = AzimuthError(right);

                while (Math.Abs(left - right) > epsilon)
                {
                    if (eLeft < eRight)
                    {
                        upperBound = right;
                        right = left;
                        eRight = eLeft;
                        left = upperBound - invphi * (upperBound - lowerBound);
                        eLeft = AzimuthError(left);
                    }
                    else
                    {
                        lowerBound = left;
                        left = right;
                        eLeft = eRight;
                        right = lowerBound + invphi * (upperBound - lowerBound);
                        eRight = AzimuthError(right);
                    }
                }

                return (upperBound + lowerBound) / 2;
            }

            if (useAltBehavior)
            {

                //double currentUT = Planetarium.GetUniversalTime();
                double candidateTime = startTime;

                while (candidateTime <= startTime + maxTimeLimit)
                {
                    double refinedTime = EstimateLaunchTime(target, launchPos, candidateTime, false); // recursive call with useAltBehavior = false
                    if (double.IsNaN(refinedTime)) // no min found within normal time limit to analyze
                    {
                        //Debug.Log("[Failure] No minimum found within time limit.");
                        return double.NaN;
                    }

                    double refinedError = AzimuthError(refinedTime);
                    //Debug.Log($"[Refined] refinedTime formatted={FormatTime(refinedTime)}, refinedTime current = {currentUT + refinedTime}, refinedError={refinedError}");

                    if (refinedError < epsilon) // global minimum found
                    //if (refinedError < tolerance)
                    {

                        //Debug.Log("[Global Minimum Found]");
                        return refinedTime;
                    }
                    else // global min not found yet
                    {
                        //Debug.Log("[Local Minimum Found] Not global, continuing search.");

                        candidateTime = refinedTime + 3600d;

                        if (candidateTime > startTime + maxTimeLimit) // no global min found within extended time limit
                        {
                            //Debug.Log("[Failure] No global minimum found within time limit.");
                            return double.NaN;
                        }
                    }
                }
            }

            double t0 = startTime;
            double t1 = startTime + coarseStep;
            double e0 = AzimuthError(t0);
            double e1 = AzimuthError(t1);

            if (e0 < e1) // either increasing slope, or t0 and t1 are on opposite sides of a min
            {
                double refinedTime = GoldenSectionSearch(t0, t1);

                if (refinedTime > startTime + tolerance) // t0 and t1 are on opposite sides of a min
                {
                    return refinedTime;
                }

                t0 = startTime;
                t1 = t0 + coarseStep;
                e0 = AzimuthError(t0);
                e1 = AzimuthError(t1);

                while (e0 <= e1) // increasing slope (we just passed a min)
                {
                    t0 = t1;
                    t1 += coarseStep;
                    e0 = e1;
                    e1 = AzimuthError(t1);
                    if (t0 >= startTime + maxTimeLimit) // no min found within time limit
                    {
                        return double.NaN;
                    }
                }
            }
            else if (Math.Abs(e0 - e1) < epsilon) // perfectly balanced between min or max
            {
                t0 += buffer;
                t1 += buffer;
                e0 = AzimuthError(t0);
                e1 = AzimuthError(t1);
            }

            double tBest = t0;
            double eBest = e0;

            while (e0 > e1) // decreasing slope
            {
                if (e1 < eBest)
                {
                    eBest = e1;
                    tBest = t1;
                }

                t0 = t1;
                e0 = e1;
                t1 += coarseStep;
                e1 = AzimuthError(t1);

                if (t0 >= startTime + maxTimeLimit) // no min found within time limit
                {
                    return double.NaN;
                }

                if (Math.Abs(e0 - e1) < epsilon) // perfectly balanced between min
                {
                    t0 += buffer;
                    t1 += buffer;
                    e0 = AzimuthError(t0);
                    e1 = AzimuthError(t1);
                }
            }

            double fineT0 = Math.Max(startTime, tBest - coarseStep);
            double fineT1 = Math.Min(startTime + maxTimeLimit, tBest + coarseStep);

            double finalResult = GoldenSectionSearch(fineT0, fineT1);

            return finalResult;
        }

        private double GetCachedLaunchTime(CelestialBody target, Vector3d launchPos, double inclination, bool useAltBehavior, int windowNumber)
        {         
            const double tolerance = 0.01;

            double offset = 3600d * target.referenceBody.rotationPeriod / EarthSiderealDay; // 1 hour offset between windows, scale based on EarthSiderealDay

            // remove expired or mismatched entries
            for (int i = cache.Count - 1; i >= 0; i--)
            {
                var entry = cache[i];
                //Debug.Log($"entry: {entry}");
                bool expired = currentUT > entry.absoluteLaunchTime;
                bool targetMismatch = entry.target != target;
                //Debug.Log($"CLAYELADDEDLOGS Current launchPos: {launchPos}, Cached launchPos: {entry.launchPos}");
                bool posMismatch = Vector3d.Distance(entry.launchPos, launchPos) >= tolerance;
                double distance = Vector3d.Distance(entry.launchPos, launchPos);
                //Debug.Log($"Launch Window Cache launchPos distance: {distance}");
                bool inclinationMismatch = Math.Abs(entry.inclination - inclination) >= tolerance * 2; 

                if (expired || targetMismatch || posMismatch || inclinationMismatch)
                {
                    //Debug.Log($"INVALID: Launch Window Cache {i} with target={entry.target.name}, launchPos={entry.launchPos}, inclination={entry.inclination:F3}, time={entry.absoluteLaunchTime:F3}");
                    //if (inclinationMismatch) Debug.Log($"CLAYELADDEDLOGS INCLINATION MISMATCH{i}, difference: {Math.Abs(entry.inclination - inclination)}");
                    //if (posMismatch) Debug.Log("CLAYELADDEDLOGS POSITION MISMATCH");
                    //cache.RemoveAt(i);
                    cache.Clear(); // doing cache.RemoveAt(i) leads to compounding errors with the other remaining launch times, don't do it
                    break;
                }
            }

            // sort cache after removing bad stuff
            cache.Sort((a, b) => a.absoluteLaunchTime.CompareTo(b.absoluteLaunchTime));

            // if window exists, return it
            if (windowNumber <= cache.Count && windowNumber > 0)
            {
                return cache[windowNumber - 1].absoluteLaunchTime;
            }

            double startTime;

            if (cache.Count == 0)
            {
                startTime = 0;
            }
            else
            {
                // start after the last cached window + offset
                startTime = cache.Last().absoluteLaunchTime - currentUT + offset;
            }

            // compute windows from cache.Count+1 up to windowNumber
            double newLaunchTime = 0;
            double absoluteLaunchTime = 0;

            for (int w = cache.Count + 1; w <= windowNumber; w++)
            {
                newLaunchTime = EstimateLaunchTime(target, launchPos, startTime, useAltBehavior);

                absoluteLaunchTime = currentUT + newLaunchTime;

                cache.Add((target, launchPos, inclination, absoluteLaunchTime));

                startTime = newLaunchTime + offset;
            }

            //Debug.Log($"cache count: {cache.Count}");

            return absoluteLaunchTime;
        }

        private (double flightTime, double rotationAngle) EstimateFlightTimeBeforeTLI(CelestialBody target, Vector3d launchPos, double delayTime)
        {
            CelestialBody mainBody = target.referenceBody;
            double gravParameter = mainBody.gravParameter;
            double orbitRadius = mainBody.Radius + tliAltitudeKM * 1000;

            //if (FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.mainBody == mainBody && FlightGlobals.ActiveVessel.orbit != null && FlightGlobals.ActiveVessel.orbit.semiMajorAxis > 0)
            // dont check if in orbit, this would be unintuitive and inconsistent with the rest of the ui

            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = Planetarium.GetUniversalTime() + flightTime * mainBody.solarDayLength + delayTime;
            Vector3d targetPos = target.getPositionAtUT(targetTime);
            //Vector3d targetPos = GetPositionAtTime(target, targetTime);

            Vector3d upVector = QuaternionD.AngleAxis(delayTime * 360d / mainBody.rotationPeriod, MainAxis) * ((launchPos - MainPos).normalized * orbitRadius).normalized; // use rotationPeriod for sidereal time

            // TLI takes place at the point of the orbit that is opposite to the future position of the Moon
            Vector3d tliUpVector = (MainPos - targetPos).normalized;

            //KSP apparently lies about having QuaternionD.FromToRotation because it just gives an error
            Vector3d rotationAxis = Vector3d.Cross(upVector, tliUpVector).normalized;
            double dot = Vector3d.Dot(upVector.normalized, tliUpVector.normalized);
            double rotationAngle = Math.Acos(Math.Max(-1.0, Math.Min(1.0, dot))) * (180.0 / Math.PI); // degrees

            if (Vector3d.Dot(rotationAxis, MainAxis) < 0)
            {
                rotationAngle = 360.0 - rotationAngle;
            }

            // Convert angle to time in orbit
            double orbitPeriod = 2 * Math.PI * Math.Sqrt(Math.Pow(orbitRadius, 3) / gravParameter);
            //double offset = 180; // for some reason it just constantly underestimates by ~3 minutes, no idea why
            //rotationAngle = (rotationAngle + (offset / orbitPeriod) * 360.0) % 360.0;
            double flightTimeBeforeTLI = rotationAngle / 360.0 * orbitPeriod;

            return (flightTimeBeforeTLI, rotationAngle);
        }

        private double EstimateFlightTimeAfterTLI(CelestialBody target, double dV, bool movingTarget = true)
        {
            CelestialBody mainBody = target.referenceBody;
            double gravParameter = mainBody.gravParameter;

            // The formulas are from http://www.braeunig.us/space/orbmech.htm

            // Assuming that the TLI is performed from a circular orbit with altitude = tliAltitudeKM
            // Radius of the orbit, including the radius of the Earth
            double r0 = mainBody.Radius + tliAltitudeKM * 1000;

            // Orbital velocity after TLI
            double v0 = Math.Sqrt(gravParameter / r0) + dV;

            // Eccentricity after TLI (not the full formula, this is correct only at the periapsis)
            double e = r0 * v0 * v0 / gravParameter - 1;

            // e == 1 would mean that the orbit is parabolic. No idea which formulas are applicable in this case.
            // But it's so unlikely that I will just cheat and make such orbits slightly hyperbolic.
            if (e == 1)
            {
                // Increase velocity after TLI  by 0.1 m/s
                v0 += 0.1;
                // Recalculate eccentricity
                e = r0 * v0 * v0 / gravParameter - 1;
            }

            // Semi-major axis after TLI
            double a = 1 / (2 / r0 - v0 * v0 / gravParameter);

            // Altitude of the Moon at the time of the TLI 
            double r1;

            // The Moon is moving, so we need to know its altitude when the probe arrives
            // For that we need to know the flight time, which is being calculated here in the first place
            // It can be done in two steps: 
            // 1) make a recursive call of EstimateFlightTimeAfterTLI to find out the approximate flight time,
            //    based on the Moon's current altitude
            // 2) use the Moon's altitude at this approximate time for further calculations
            if (movingTarget)
            {
                // This is the "normal" call of EstimateFlightTimeAfterTLI from outside

                // Making the recursive call of EstimateFlightTimeAfterTLI with movingTarget = false
                double approxFlightTime = EstimateFlightTimeAfterTLI(target, dV, false);

                // Altitude of the Moon at the approximate time of the TLI 
                r1 = target.orbit.GetRadiusAtUT(Planetarium.GetUniversalTime() + approxFlightTime);
            }
            else
            {
                // This is the recursive call of EstimateFlightTimeAfterTLI

                // Altitude of the Moon now
                r1 = target.orbit.GetRadiusAtUT(Planetarium.GetUniversalTime());
            }

            // True anomaly when the vessel reaches the altitude of the Moon (r1)
            double trueAnomaly1 = Math.Acos((a * (1 - e * e) - r1) / (e * r1));

            // Time until the vessel reaches the altitude of the Moon (r1)
            double t1 = 0;

            // Elliptic orbit after TLI
            if (e < 1)
            {
                // Eccentric Anomaly when the vessel reaches the altitude of the Moon
                double eccAnomaly1 = Math.Acos((e + Math.Cos(trueAnomaly1)) / (1 + e * Math.Cos(trueAnomaly1)));
                double meanAnomaly1 = eccAnomaly1 - e * Math.Sin(eccAnomaly1);
                t1 = meanAnomaly1 / Math.Sqrt(gravParameter / Math.Pow(a, 3));
            }

            // Parabolic orbit (e == 1) has been prevented earlier

            // Hyperbolic orbit
            if (e > 1)
            {
                // Hyperbolic Eccentric Anomaly when the vessel reaches the altitude of the Moon
                // Can't use Math.Acosh, it does not seem to work in .NET 4
                double hEccAnomaly1 = Util.Acosh((e + Math.Cos(trueAnomaly1)) / (1 + e * Math.Cos(trueAnomaly1)));

                t1 = Math.Sqrt(Math.Pow(-a, 3) / gravParameter) * (e * Math.Sinh(hEccAnomaly1) - hEccAnomaly1);
            }

            return t1;
        }

        private double EstimateDV(CelestialBody target)
        {
            double dV = double.NaN;
            CelestialBody mainBody = target.referenceBody;

            const double EarthRadius = 6371000.0d;
            const double EarthMass = 3.9860043543609598e+14 / 6.673e-11;

            // Search in this range
            double minPossibleDV = 2500 * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius), gives 766 for Kerbin
            double maxPossibleDV = 6000 * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius), gives 1840 for Kerbin
            const double tolerance = 0.001;

            // Current search range, will be gradually narrowed
            double lowerBound = minPossibleDV;
            double upperBound = maxPossibleDV;

            // Max. 16 attempts, then return whatever value was found
            for (int i = 0; i < 16; i++)
            {
                // guess dV
                dV = (lowerBound + upperBound) / 2;

                // calculate flight time for this dV
                double flightTimeAfterTLI = EstimateFlightTimeAfterTLI(target, dV);
                //(double flightTimeBeforeTLI, _) = EstimateFlightTimeBeforeTLI(target, launchPos, 0d);
                //double estimatedFlightTime = flightTimeBeforeTLI + flightTimeAfterTLI; // the inputted flight time is for after the maneuver
                double expectedFlightTime = flightTime * mainBody.solarDayLength;

                // Debug.Log(i + " " + dV + " " + flightTime + " " + flightTimeBeforeTLI + " " + flightTimeAfterTLI + " " + estimatedFlightTime);

                if (Double.IsNaN(flightTimeAfterTLI))
                {
                    // dV is so low that target is unreachable, set lower bound to current guess and try again
                    lowerBound = dV;
                    continue;
                }
                else if (flightTimeAfterTLI > (expectedFlightTime + 5))
                {
                    // dV is too low, set lower bound to current guess and try again
                    lowerBound = dV;
                    continue;
                }
                else if (flightTimeAfterTLI < (expectedFlightTime - 5))
                {
                    // dV is too high, set upper bound to current guess and try again
                    upperBound = dV;
                    continue;
                }
                else
                {
                    // correct flight time with this dV
                    break;
                }
            }

            if (Math.Abs(dV - minPossibleDV) <= tolerance * Math.Abs(minPossibleDV) || Math.Abs(dV - maxPossibleDV) <= tolerance * Math.Abs(maxPossibleDV))
            {
                // dV is incorrect, the correct value is outside the initial search range
                dV = double.NaN;
            }

            return dV;
        }

        private string FormatTime(double t)
        {
            t = Math.Round(t);
            int days = (int)Math.Floor(t / Math.Round(target.referenceBody.solarDayLength)); // avoid stuff like 3d 24h 0m 0s
            t -= days * Math.Round(target.referenceBody.solarDayLength);
            int hours = (int)Math.Floor(t / (60 * 60));
            t -= hours * 60d * 60d;
            int minutes = (int)Math.Floor(t / 60);
            t -= minutes * 60d;
            if (days > 0)
                return $"{days}d {hours}h {minutes}m {t:F0}s";
            else if (hours > 0)
                return $"{hours}h {minutes}m {t:F0}s";
            else if (minutes > 0)
                return $"{minutes}m {t:F0}s";
            return $"{t:F0}s";
        }

        private void ButtonPressed(bool pressed, ref bool show)
        {
            if (pressed)
            {
                show = !show;
                ResetWindow();
            }
        }

        private void TogglePressed(bool pressed, ref bool show)
        {
            if (pressed != show)
            {
                show = !show;
                ResetWindow();
            }
        }

        private void ResetWindow()
        { // Doing this forces the window to be resized. Without it, the window will become bigger when controls expand, but never become smaller again
            windowRect = new Rect(windowRect.xMin, windowRect.yMin, -1, -1); 
        }

        private void ShowOrbitInfo(ref bool showPhasing, double timeInOrbit, double phaseAngle)
        {
            if (showPhasing)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Phasing angle", "Phasing angle between launch location and maneuver in orbit"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button("T", GUILayout.Width(20));
                ButtonPressed(showPhasing_pressed, ref showPhasing);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{phaseAngle:F2}\u00B0", $"{phaseAngle * Math.PI/180d:F5} rads"), GUILayout.MinWidth(60)); // for some reason the tooltip doesn't work for the immediate launch window, idk why
            }
            else
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Time in orbit", "Time spent waiting in parking orbit before maneuver"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button(" P", GUILayout.Width(20));
                ButtonPressed(showPhasing_pressed, ref showPhasing);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent(FormatTime(timeInOrbit), $"{timeInOrbit:0}s"), GUILayout.MinWidth(100));
            }
        }

        private void MakeMainWindow(int id)
        {
            GUILayout.BeginVertical();

            windowTitle = "Lunar Transfer";

            //CelestialBody target = FlightGlobals.fetch.bodies.FirstOrDefault(body => body.name.Equals("Moon", StringComparison.OrdinalIgnoreCase));
            List<CelestialBody> satellites = FlightGlobals.currentMainBody?.orbitingBodies?.OrderBy(body => body.bodyName).ToList(); // use currentMainBody here, target.referenceBody elsewhere

            GUILayout.Space(5);

            if (satellites == null || satellites.Count == 0)
            {
                GUILayout.Box("ERROR: There are no moons of this planet!", GUILayout.MinWidth(80));
            }
            else
            {
                currentUT = Planetarium.GetUniversalTime();

                bool inSurfaceVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null && (FlightGlobals.ActiveVessel.Landed || FlightGlobals.ActiveVessel.Splashed);
                if (!inSurfaceVessel) useVesselPosition = false;

                if (target == null || !satellites.Contains(target)) target = satellites[0];
                inclination = GetTargetInclination(target.orbit);
                bool isLowLatitude = Math.Abs(latitude) <= inclination;

                GUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();
                bool useAltSkin_toggled = GUILayout.Toggle(useAltSkin, new GUIContent("", "Use Unity Skin"));

                GUI.enabled = isLowLatitude;
                bool useAltBehavior_toggled = GUILayout.Toggle(useAltBehavior, new GUIContent("", $"Find Global Minimum inclination instead of Local Minimum, disabled if latitude is higher than {target.bodyName} inclination"));
                GUI.enabled = true;

                GUI.enabled = inSurfaceVessel;
                useVesselPosition = GUILayout.Toggle(useVesselPosition, new GUIContent("", "Use surface vessel position for latitude instead of launch site position, disabled if not on surface"));
                GUI.enabled = true;

                if (!useAltSkin) GUILayout.Space(10); // spacing is different with unity skin
                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();

                TogglePressed(useAltSkin_toggled, ref useAltSkin);

                if (!isLowLatitude) useAltBehavior_toggled = false; // if we change useAltBehavior to false, the next code will just set it to true again

                if (useAltBehavior != useAltBehavior_toggled)
                {
                    useAltBehavior = useAltBehavior_toggled;
                    //Debug.Log("cache Cleared due to useAltBehavior change");
                    if (cache.Count > 0) cache.Clear(); // remove local minimums
                    // technically this only needs to be done when switching from false to true, but switching from true to false without clearing
                    // would result in some extra data in the cache, which might lead to a memory leak if abused
                }

                if (satellites.Count > 1) // only display target selector screen if needed
                {
                    GUILayout.Space(5);
                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button("<", GUILayout.MinWidth(20)))
                    {
                        currentBody--;
                        if (currentBody < 0) currentBody = satellites.Count - 1;
                        //target = satellites[currentBody];
                    }

                    GUILayout.Box(target.bodyName, GUILayout.MinWidth(80));

                    if (GUILayout.Button(">", GUILayout.MinWidth(20)))
                    {
                        currentBody++;
                        if (currentBody > satellites.Count - 1) currentBody = 0;
                        //target = satellites[currentBody];
                    }
                    GUILayout.EndHorizontal();
                    target = satellites[currentBody];
                }
                
                CelestialBody mainBody = target.referenceBody;

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent($"Latitude: <b>{latitude:F2}\u00B0</b>", "Latitude of launch location"), GUILayout.ExpandWidth(true));
                bool expandLatLong_pressed = GUILayout.Button(!expandLatLong ? "+" : "\u2013", GUILayout.Width(30));
                GUILayout.EndHorizontal();
                ButtonPressed(expandLatLong_pressed, ref expandLatLong);

                if (expandLatLong)
                {
                    GUILayout.Space(6); // weird spacing
                    MakeNumberEditField("latitude", ref latitude, 1d, -90d, 90d);
                    GUILayout.Space(5);
                    GUILayout.Label(new GUIContent($"Longitude: <b>{longitude:F2}\u00B0</b>", "Longitude of launch location"));
                    MakeNumberEditField("longitude", ref longitude, 1d, -180d, 180d);
                    //launchPos = mainBody.GetWorldSurfacePosition(latitude, longitude, 0);
                }
                //else
                //{
                //    launchPos = GetLaunchPos(mainBody, ref latitude, ref longitude, useVesselPosition);
                //}

                // keep this outside to prevent argument exception caused by updating values between Layout and Repaint // not working
                launchPos = expandLatLong ? mainBody.GetWorldSurfacePosition(latitude, longitude, 0) : GetLaunchPos(mainBody, ref latitude, ref longitude, useVesselPosition);

                GUILayout.Space(5);
                GUILayout.Label(new GUIContent("Flight Time (days)", $"Coast duration to {target.bodyName} after the maneuver (in {mainBody.bodyName} solar days)"), GUILayout.ExpandWidth(true));
                MakeNumberEditField("flightTime", ref flightTime, 0.1d, 0.1d, double.MaxValue);
                double l = flightTime * mainBody.solarDayLength;
                string s = FormatTime(flightTime * mainBody.solarDayLength);
                GUILayout.Box(new GUIContent(FormatTime(l), $"{l:0}s"), GUILayout.MinWidth(100)); // tooltips in Box have a problem with width, use {0:0}

                OrbitData launchOrbit = CalcOrbitForTime(target, launchPos, 0d);
                //Debug.Log("CLAYELADDEDDLOGS FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW");
                //var stopwatch = Stopwatch.StartNew();
                double firstLaunchETA = GetCachedLaunchTime(target, launchPos, inclination, useAltBehavior, 1) - currentUT;
                //double firstLaunchETA = EstimateLaunchTime(target, launchPos, 0d);
                //stopwatch.Stop();
                //Debug.Log($"Window 1 Launch Time: {firstLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");
                //Debug.Log("CLAYELADDEDDLOGS SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW");
                //stopwatch = Stopwatch.StartNew();
                double secondLaunchETA = GetCachedLaunchTime(target, launchPos, inclination, useAltBehavior, 2) - currentUT;
                //double secondLaunchETA = EstimateLaunchTime(target, launchPos, firstLaunchETA + 3600d); // next window is at least 1 hour past the previous window
                //stopwatch.Stop();
                //Debug.Log($"Window 2 Launch Time: {secondLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");

                double dV = EstimateDV(target);
                specialWarp1 = firstLaunchETA >= mainBody.rotationPeriod && PrincipiaInstalled;

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Required \u0394V", "Required change in velocity for the maneuver if launched now"), GUILayout.ExpandWidth(true));
                bool expandAltitude_pressed = GUILayout.Button(!expandAltitude ? "+" : "\u2013", GUILayout.Width(30));
                GUILayout.EndHorizontal();
                ButtonPressed(expandAltitude_pressed, ref expandAltitude);

                GUILayout.Box(new GUIContent($"{dV:F2} m/s", $"{dV/1000d:F5} km/s"), GUILayout.MinWidth(100));

                if (expandAltitude)
                {
                    GUILayout.Label(new GUIContent("Parking Orbit (km)", "Planned altitude of the circular parking orbit before the maneuver"), GUILayout.ExpandWidth(true));
                    MakeNumberEditField("tliAltitudeKM", ref tliAltitudeKM, 5d, mainBody.atmosphere ? mainBody.atmosphereDepth / 1000d : 0d, target.orbit.PeA / 1000d - 5d); // PeA updates every frame so we don't need to ask Principia
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Launch Now Incl.", "Launch to this inclination now to get into the right parking orbit (positive = Northeast, negative = Southeast)"), GUILayout.ExpandWidth(true));
                bool expandParking0_pressed = GUILayout.Button(!expandParking0 ? "+" : "\u2013", GUILayout.Width(30));
                GUILayout.EndHorizontal();
                ButtonPressed(expandParking0_pressed, ref expandParking0);

                GUILayout.Space(5);
                double launchInclination = launchOrbit.azimuth > 90d ? -launchOrbit.inclination : launchOrbit.inclination;
                GUILayout.Box(new GUIContent($"{launchInclination:F2}\u00B0", $"{launchInclination * Math.PI/180d:F5} rads"), GUILayout.MinWidth(100));

                string tooltip = Math.Abs(latitude) >= inclination ?
                    "Launch Easterly at this time to get into the right parking orbit" :
                    "Launch at this time at this inclination to get into the right parking orbit";

                if (expandParking0)
                {
                    (double timeInOrbit0, double phaseAngle0) = EstimateFlightTimeBeforeTLI(target, launchPos, 0d);
                    ShowOrbitInfo(ref showPhasing, timeInOrbit0, phaseAngle0);
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("First Window", tooltip), GUILayout.ExpandWidth(true));
                bool expandParking1_pressed = GUILayout.Button(!expandParking1 ? "+" : "\u2013", GUILayout.Width(30));
                GUILayout.EndHorizontal();
                ButtonPressed(expandParking1_pressed, ref expandParking1);

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent(FormatTime(firstLaunchETA), $"UT: {currentUT + firstLaunchETA:0}s"), GUILayout.MinWidth(100)); // itll flash every second if we just do {firstLaunchETA}, we need absolute time

                if (expandParking1)
                {
                    (double timeInOrbit1, double phaseAngle1) = EstimateFlightTimeBeforeTLI(target, launchPos, firstLaunchETA);
                    ShowOrbitInfo(ref showPhasing, timeInOrbit1, phaseAngle1);
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Second Window", tooltip), GUILayout.ExpandWidth(true));
                bool expandParking2_pressed = GUILayout.Button(!expandParking2 ? "+" : "\u2013", GUILayout.Width(30));
                GUILayout.EndHorizontal();
                ButtonPressed(expandParking2_pressed, ref expandParking2);

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent(FormatTime(secondLaunchETA), $"UT: {currentUT + secondLaunchETA:0}s"), GUILayout.MinWidth(100));

                if (expandParking2)
                {
                    (double timeInOrbit2, double phaseAngle2) = EstimateFlightTimeBeforeTLI(target, launchPos, secondLaunchETA);
                    ShowOrbitInfo(ref showPhasing, timeInOrbit2, phaseAngle2);
                }

                GUILayout.Space(5);
                GUILayout.Label(new GUIContent("Warp Margin (sec)", "The time difference from the launch window that the warp will stop at"), GUILayout.ExpandWidth(true));
                MakeNumberEditField("warpMargin", ref warpMargin, 5d, 0d, double.MaxValue);

                GUILayout.Space(10);
                GUILayout.BeginHorizontal();
                GUI.enabled = KACInstalled;
                bool addAlarm = GUILayout.Button(new GUIContent(" Add Alarm", "Add Alarm to Kerbal Alarm Clock"), GUILayout.MinWidth(80));
                GUI.enabled = true;
                bool toggleWarp = GUILayout.Button(new GUIContent(TimeWarp.CurrentRate > 1d || specialWarp2 || specialWarp3 ? "Stop Warp" : "Warp", "Warp to the First Window, taking into account the Warp Margin"), GUILayout.MinWidth(80));
                GUILayout.EndHorizontal();

                if (specialWarp1 || specialWarp2 || specialWarp3) // need to reset window
                //if (specialWarp1)
                {
                    GUILayout.Label(new GUIContent("Special Warp", "The special warp uses multiple warps to avoid overshooting/undershooting due to perturbations of the target's orbit (only activates when Principia is installed and the First Window is more than 1 sidereal day away)"));
                }

                TogglePressed(specialWarp1, ref specialWarpBuffer);
                TogglePressed(specialWarp2, ref specialWarpBuffer);
                TogglePressed(specialWarp3, ref specialWarpBuffer);

                bool targetPressed = GUILayout.Button(targetSet ? "Unset Target" : $"Target {target.bodyName}", GUILayout.MinWidth(160));

                if (addAlarm)
                {
                    if (KACInstalled && !double.IsNaN(firstLaunchETA))
                    {
                        string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, "Lunar Transfer Window", currentUT + firstLaunchETA - warpMargin);
                        if (!string.IsNullOrEmpty(alarmId))
                        {
                            //if the alarm was made get the object so we can update it
                            KACWrapper.KACAPI.KACAlarm alarm = KACWrapper.KAC.Alarms.First(z => z.ID == alarmId);

                            //Now update some of the other properties
                            alarm.AlarmAction = KACWrapper.KACAPI.AlarmActionEnum.KillWarp;
                        }
                    }
                }

                if (toggleWarp)
                {
                    if (TimeWarp.CurrentRate > 1d)
                    {
                        TimeWarp.fetch.CancelAutoWarp();
                        TimeWarp.SetRate(0, false);
                    }
                    else
                    {
                        if (specialWarp1)
                        {
                            Debug.Log("Begining special warp");
                            TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                            TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - (warpMargin + mainBody.rotationPeriod)); // warp to within a day first to prevent perturbations
                            Debug.Log("Special warp in progress");
                            specialWarp2 = true;
                            specialWarpWait = true;
                            //TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - warpMargin); // now warp to actual time
                        }
                        else
                        {
                            TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - warpMargin);
                        }
                    }
                }

                if (specialWarpWait && !(TimeWarp.CurrentRate > 0.5d))
                {
                    waitingTime = Planetarium.GetUniversalTime();
                    Debug.Log($"waitingTime: {waitingTime}");
                    cache.Clear(); // make sure we have an up to date time
                    specialWarpWait = false;
                }


                if (specialWarp2 && !(TimeWarp.CurrentRate > 1d) && currentUT > waitingTime + 0.5d)
                {
                    Debug.Log($"currentUT: {currentUT}");
                    
                    Debug.Log("Beginning aux warp 1");
                    TimeWarp.fetch.CancelAutoWarp();
                    //TimeWarp.fetch.CancelAutoWarp(-1, true);
                    //TimeWarp.SetRate(5, false);
                    //TimeWarp.SetRate(0, true);
                    TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                    TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - (warpMargin + 3600d * mainBody.rotationPeriod/EarthSiderealDay));
                    specialWarp2 = false;
                    specialWarp3 = true;
                    specialWarpWait = true;
                    
                    Debug.Log("Aux warp 1 in progress");
                }

                if (specialWarp3 && !(TimeWarp.CurrentRate > 1d) && currentUT > waitingTime + 1d)
                {
                    Debug.Log($"currentUT: {currentUT}");

                    Debug.Log("Beginning aux warp 2");
                    TimeWarp.fetch.CancelAutoWarp();
                    //TimeWarp.fetch.CancelAutoWarp(-1, true);
                    //TimeWarp.SetRate(5, false);
                    //TimeWarp.SetRate(0, true);
                    TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - (warpMargin));
                    specialWarp3 = false;
                    specialWarpWait = false;

                    Debug.Log("Aux warp 2 in progress");
                }





                //if (toggleWarp)
                //{
                //    if (TimeWarp.CurrentRate > 1d)
                //    {
                //        TimeWarp.fetch.CancelAutoWarp();
                //        TimeWarp.SetRate(0, false);
                //    }
                //    else
                //    {
                //        TimeWarp.fetch.WarpTo(currentUT + firstLaunchETA - warpMargin);
                //    }
                //}





                if (targetPressed)
                {
                    if (!targetSet)
                    {
                        targetSet = true;
                        FlightGlobals.fetch.SetVesselTarget(target);
                    }
                    else
                    {
                        targetSet = false;
                        FlightGlobals.fetch.SetVesselTarget(null);
                    }
                }
            }

            Tooltip.Instance.RecordTooltip(id);

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
