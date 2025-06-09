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

using KerbalAlarmClock;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics; // remove this
using System.IO;
using System.Linq;
using UnityEngine;
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

        // Unity only has Clamp for floats
        public static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }
    }



    [KSPAddon(KSPAddon.Startup.FlightAndKSC, false)]
    public class LunarTransferPlanner : DaMichelToolbarSuperWrapper.PluginWithToolbarSupport
    {
        const double EarthSiderealDay = 86164.098903691; // this is what RSS uses at RealSolarSystem/RSSKopernicus/Earth/Earth.cfg
        const double tau = 2 * Math.PI;
        const double radToDeg = 180d / Math.PI; // unity only has floats
        const double degToRad = Math.PI / 180d; // unity only has floats

        Rect windowRect = new Rect(100, 100, -1, -1);
        Rect settingsRect = new Rect(100, 100, -1, -1);
        readonly string windowTitle = "Lunar Transfer";
        readonly string settingsTitle = "Additional Settings";
        GUISkin skin;
        Texture2D gearBlack;
        Texture2D gearGreen;
        bool isWindowOpen = true;

        int currentBody = 0;
        CelestialBody target = null;
        Vector3d launchPos;
        double inclination;
        bool KACInstalled;
        bool PrincipiaInstalled;
        double currentUT;
        bool showSettings = false; // Show settings UI
        bool useAltSkin = false; // Use Unity GUI skin instead of default
        double flightTime = 4; // Desired flight time after maneuver, in days
        double parkingAltitude = 200; // Parking orbit altitude (circular orbit assumed)
        bool useAltBehavior = false; // Find global minimum for low latitudes instead of local minimum
        bool useVesselPosition = false; // Use vessel position for latitude instead of launch site position
        bool inSurfaceVessel;
        bool useAltAlarm = false; // Set an alarm based on the extra window instead of the next launch window
        double latitude = 0d;
        double longitude = 0d;
        int referenceTimeButton = 0;
        string referenceTimeLabel;
        string referenceTimeTooltip;
        string label;
        double referenceTime = 0d;
        double targetAzimuth = 90d;
        double targetInclination = double.NaN;
        bool showAzimuth = false;
        bool expandLatLong = false; // Expand/collapse custom Latitude/Longitude picker
        bool expandAltitude = false; // Expand/collapse parking orbit altitude changer
        bool expandParking0 = false; // Expand/collapse time in parking orbit for launch now
        bool expandParking1 = false; // Expand/collapse time in parking orbit for next window
        bool expandParking2 = false; // Expand/collapse time in parking orbit for extra window
        int extraWindowNumber = 2;
        int maxWindows = 100; // Maximum amount of extra windows that can be calculated
        bool showPhasing = false; // Show the phasing angle instead of the time in parking orbit, applies to all boxes
        bool isLowLatitude;
        bool targetSet = false;
        double warpMargin = 60; // Time difference from the launch window that the warp will stop at

        bool specialWarp = true;
        enum SpecialWarp { None, Warp1, Warp2, Warp3 }
        SpecialWarp warpState = SpecialWarp.None;
        //bool specialWarp1 = false;
        //bool specialWarp2 = false;
        //bool specialWarp3 = false;
        bool specialWarpBuffer = false;
        bool specialWarpWait = false;
        double waitingTime;

        List<CelestialBody> satellites;
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
            gearBlack = GameDatabase.Instance.GetTexture("LunarTransferPlanner/gearBlack", false);
            gearGreen = GameDatabase.Instance.GetTexture("LunarTransferPlanner/gearGreen", false);

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

            Dictionary<string, object> settingValues = new Dictionary<string, object>
            {
                { "windowRect.xMin", windowRect.xMin },
                { "windowRect.yMin", windowRect.yMin },
                { "settingsRect.xMin", settingsRect.xMin },
                { "settingsRect.yMin", settingsRect.yMin },
                { "showSettings", showSettings },
                { "useAltSkin", useAltSkin },
                { "flightTime", flightTime },
                { "parkingAltitude", parkingAltitude },
                { "useAltBehavior", useAltBehavior },
                { "useVesselPosition", useVesselPosition },
                { "latitude", latitude },
                { "longitude", longitude },
                { "showAzimuth", showAzimuth },
                { "expandLatLong", expandLatLong },
                { "expandAltitude", expandAltitude },
                { "expandParking0", expandParking0 },
                { "expandParking1", expandParking1 },
                { "expandParking2", expandParking2 },
                { "showPhasing", showPhasing },
                { "maxWindows", maxWindows },
                { "warpMargin", warpMargin },
                { "specialWarp", specialWarp },
                { "targetAzimuth", targetAzimuth }
            };

            foreach (KeyValuePair<string, object> kvp in settingValues) settings.AddValue(kvp.Key, kvp.Value);

            ConfigNode root = new ConfigNode();
            root.AddNode(settings);
            root.Save(SettingsPath);

            Dictionary<string, string> comments = new Dictionary<string, string>
            {
                { "maxWindows", "Changes the maximum amount of windows that can be calculated with the extra window chooser, default of 100. Each launch window is temporarily cached, so caching a ridiculous amount may lead to performance degradation" },
                { "targetAzimuth", "Target Inclination is converted to and from Target Azimuth automatically" }
            };

            List<string> lines = File.ReadAllLines(SettingsPath).ToList();
            foreach (KeyValuePair<string, string> kvp in comments)
            {
                int index = lines.FindIndex(line => line.Contains(kvp.Key));
                if (index != -1)
                    lines[index] += $" // {kvp.Value}";
            }
            File.WriteAllLines(SettingsPath, lines);
        }


        private void LoadSettings()
        {
            ConfigNode root = ConfigNode.Load(SettingsPath);
            if (root != null)
            {
                ConfigNode settings = root.GetNode("SETTINGS");
                if (settings != null)
                {
                    float x1 = windowRect.xMin, y1 = windowRect.yMin;
                    float x2 = settingsRect.xMin, y2 = settingsRect.yMin;
                    Util.TryReadValue(ref x1, settings, "windowRect.xMin");
                    Util.TryReadValue(ref y1, settings, "windowRect.yMin");
                    Util.TryReadValue(ref x2, settings, "settingsRect.xMin");
                    Util.TryReadValue(ref y2, settings, "settingsRect.yMin");
                    Util.TryReadValue(ref showSettings, settings, "showSettings");
                    Util.TryReadValue(ref useAltSkin, settings, "useAltSkin");
                    Util.TryReadValue(ref flightTime, settings, "flightTime");
                    Util.TryReadValue(ref parkingAltitude, settings, "parkingAltitude");
                    Util.TryReadValue(ref useAltBehavior, settings, "useAltBehavior");
                    Util.TryReadValue(ref useVesselPosition, settings, "useVesselPosition");
                    Util.TryReadValue(ref latitude, settings, "latitude");
                    Util.TryReadValue(ref longitude, settings, "longitude");
                    Util.TryReadValue(ref showAzimuth, settings, "showAzimuth");
                    Util.TryReadValue(ref expandLatLong, settings, "expandLatLong");
                    Util.TryReadValue(ref expandAltitude, settings, "expandAltitude");
                    Util.TryReadValue(ref expandParking0, settings, "expandParking0");
                    Util.TryReadValue(ref expandParking1, settings, "expandParking1");
                    Util.TryReadValue(ref expandParking2, settings, "expandParking2");
                    Util.TryReadValue(ref showPhasing, settings, "showPhasing");
                    Util.TryReadValue(ref maxWindows, settings, "maxWindows");
                    Util.TryReadValue(ref warpMargin, settings, "warpMargin");
                    Util.TryReadValue(ref specialWarp, settings, "specialWarp");
                    Util.TryReadValue(ref targetAzimuth, settings, "targetAzimuth");
                    windowRect = new Rect(x1, y1, windowRect.width, windowRect.height);
                    settingsRect = new Rect(x2, y2, settingsRect.width, settingsRect.height);

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
                windowRect = GUILayout.Window(this.GetHashCode(), windowRect, MakeMainWindow, windowTitle);
                ClampToScreen(ref windowRect);
                Tooltip.Instance.ShowTooltip(this.GetHashCode());

                if (showSettings)
                {
                    settingsRect = GUILayout.Window(this.GetHashCode() + 1, settingsRect, MakeSettingsWindow, settingsTitle);
                    ClampToScreen(ref settingsRect);
                    Tooltip.Instance.ShowTooltip(this.GetHashCode() + 1);
                }
            }
        }

        private void ClampToScreen(ref Rect rect)
        {
            float left = Mathf.Clamp(rect.x, 0, Screen.width - rect.width);
            float top = Mathf.Clamp(rect.y, 0, Screen.height - rect.height);
            rect = new Rect(left, top, rect.width, rect.height);
        }

        private void MakeNumberEditField<T>(string controlId, ref T value, IConvertible step, IConvertible minValue, IConvertible maxValue, bool wrapAround = false, string minusTooltip = "", string plusTooltip = "") where T : struct, IConvertible
        {
            // allow ints, doubles, floats, etc.
            double valueDouble = Convert.ToDouble(value);
            double stepDouble = Convert.ToDouble(step);
            double minValueDouble = Convert.ToDouble(minValue);
            double maxValueDouble = Convert.ToDouble(maxValue);

            // retrieve tick time buffer
            if (!nextTickMap.TryGetValue(controlId, out double nextTick))
                nextTick = 0;

            // retrieve text buffer
            if (!textBuffer.TryGetValue(controlId, out string textValue))
                textValue = valueDouble.ToString("G17");

            if (double.TryParse(textValue, out double parsedBufferValue))
            {
                if (Math.Abs(parsedBufferValue - valueDouble) > 1e-9)
                {
                    // external change detected, update buffer
                    textValue = valueDouble.ToString("G17");
                    textBuffer[controlId] = textValue;
                }
            }
            else
            {
                // invalid buffer, resync
                textValue = valueDouble.ToString("G17");
                textBuffer[controlId] = textValue;
            }

            GUILayout.BeginHorizontal();

            string newLabel = GUILayout.TextField(textValue, GUILayout.MinWidth(40), GUILayout.MaxWidth(100)); // cannot add tooltip to TextField

            // if text changed, update buffer and try to parse value
            if (newLabel != textValue)
            {
                textBuffer[controlId] = newLabel;

                if (double.TryParse(newLabel, out double newValue))
                {
                    valueDouble = Util.Clamp(newValue, minValueDouble, maxValueDouble);
                }
            }

            bool hitMinusButton = GUILayout.RepeatButton(new GUIContent("\u2013", minusTooltip), GUILayout.MinWidth(40), GUILayout.MaxWidth(60)); // en dash shows up as the same width as + ingame, while the minus symbol is way thinner
            bool hitPlusButton = GUILayout.RepeatButton(new GUIContent("+", plusTooltip), GUILayout.MinWidth(40), GUILayout.MaxWidth(60));

            if (hitPlusButton || hitMinusButton)
            {
                double tick = Time.realtimeSinceStartup;
                if (tick > nextTick)
                {
                    if (hitMinusButton)
                    {

                        double snappedValue = Math.Floor(valueDouble / stepDouble) * stepDouble;
                        if (Math.Abs(valueDouble - snappedValue) > 1e-9)
                            valueDouble = Math.Max(minValueDouble, snappedValue); // snap to next number
                        else
                            if (valueDouble - stepDouble < minValueDouble && wrapAround)
                            valueDouble = maxValueDouble;
                        else valueDouble = Math.Max(minValueDouble, valueDouble - stepDouble); // then decrement
                    }
                    else
                    {
                        double snappedValue = Math.Ceiling(valueDouble / stepDouble) * stepDouble;
                        if (Math.Abs(valueDouble - snappedValue) > 1e-9)
                            valueDouble = Math.Min(maxValueDouble, snappedValue); // snap to next number
                        else
                            if (valueDouble + stepDouble > maxValueDouble && wrapAround)
                            valueDouble = minValueDouble;
                        else valueDouble = Math.Min(maxValueDouble, valueDouble + stepDouble); // then increment
                    }

                    int decimals = Math.Max(0, (int)Math.Ceiling(-Math.Log10(stepDouble))); // avoid annoying floating point issues when rounding
                    valueDouble = Math.Round(valueDouble, decimals);
                    nextTickMap[controlId] = tick + 0.2d; // wait 0.2s before changing again
                    textBuffer[controlId] = valueDouble.ToString($"F{decimals}");
                }
            }

            GUILayout.EndHorizontal();

            value = (T)Convert.ChangeType(valueDouble, typeof(T)); // convert back to original type
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
            if (useVesselPosition && inSurfaceVessel) // double check if it is safe to avoid null-ref
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
            double inclination = Math.Acos(Vector3d.Dot(orbitNorm, MainAxis)); // inclination of the launch orbit, not the target orbit (this should be able to handle retrograde target orbits? test this)
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

            //double azimuth = Math.Acos(Vector3d.Dot(launchVec, northVec)); // this only allows azimuths between 0 and 180
            double azimuth = Math.Atan2(Vector3d.Dot(launchVec, eastVec), Vector3d.Dot(launchVec, northVec)); // this allows azimuths between 0 and 360
            azimuth = ((azimuth % tau) + tau) % tau;

            return new OrbitData(orbitNorm, inclination * radToDeg, azimuth * radToDeg);
        }

        private double EstimateLaunchTime(CelestialBody target, Vector3d launchPos, double startTime, bool useAltBehavior)
        {
            if (!isLowLatitude) useAltBehavior = false; // this only changes the parameter

            const double tolerance = 0.01;
            double coarseStep = 1200 * target.referenceBody.rotationPeriod / EarthSiderealDay; // scale based on EarthSiderealDay
            double maxTimeLimit = useAltBehavior ? target.referenceBody.rotationPeriod * 30 : target.referenceBody.rotationPeriod; // expand to 30 days if global minimum
            const double epsilon = 1e-9;
            const double buffer = 1.0;

            double AzimuthError(double t)
            {
                double az = CalcOrbitForTime(target, launchPos, t).azimuth;
                return Math.Abs(((az - targetAzimuth + 540) % 360) - 180);
            }
            // dont turn this into InclinationError, CalcOrbitForTime is limited in which inclinations it can return, but it can return 0 to 360 of azimuth

            double GoldenSectionSearch(double lowerBound, double upperBound) // adopted from https://en.wikipedia.org/wiki/Golden-section_search
            {
                // I have no idea why this works, but it works so well
                double invphi = (Math.Sqrt(5) - 1) / 2; // positive conjugate of golden ratio
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
                        return double.NaN;
                    }

                    double refinedError = AzimuthError(refinedTime);

                    if (refinedError < epsilon) // global minimum found
                    {

                        return refinedTime;
                    }
                    else // global min not found yet
                    {

                        candidateTime = refinedTime + 3600d;

                        if (candidateTime > startTime + maxTimeLimit) // no global min found within extended time limit
                        {
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
                    //Debug.Log($"launchTime found at {refinedTime}");
                    return refinedTime;
                }

                //Debug.Log("Increasing Slope");

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

            //Debug.Log("Decreasing Slope");

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

            //Debug.Log($"launchTime found at {finalResult}");

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
                bool expired = currentUT > entry.absoluteLaunchTime;
                bool targetMismatch = entry.target != target;
                bool posMismatch = Vector3d.Distance(entry.launchPos, launchPos) >= tolerance; // we restart when changing launch sites, so this only triggers when changing position by vessel or manually
                bool inclinationMismatch = Math.Abs(entry.inclination - inclination) >= tolerance * 2;

                if (expired || targetMismatch || posMismatch || inclinationMismatch)
                {
                    //Debug.Log($"INVALID: Launch Window Cache {i} with target={entry.target.name}, launchPos={entry.launchPos}, inclination={entry.inclination:F3}, time={entry.absoluteLaunchTime:F3}");
                    //cache.RemoveAt(i);
                    cache.Clear(); // doing cache.RemoveAt(i) leads to compounding errors with the other remaining launch times, don't do it
                    break;
                }
            }

            // sort cache in ascending order of launch time after removing bad entries
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

                if (cache.Count > maxWindows)
                {
                    Debug.Log("cache.Count has grown greater than maxWindows!");
                    return double.NaN;
                }
            }

            // prevent excessive growth, just in case we somehow still have extra windows
            while (cache.Count > maxWindows)
            {
                cache.RemoveAt(cache.Count - 1); // remove the last one
            }

            //Debug.Log($"cache count: {cache.Count}");

            return absoluteLaunchTime;
        }

        private (double flightTime, double rotationAngle) EstimateFlightTimeBeforeTLI(CelestialBody target, Vector3d launchPos, double delayTime)
        {
            CelestialBody mainBody = target.referenceBody;
            double gravParameter = mainBody.gravParameter;
            double orbitRadius = mainBody.Radius + parkingAltitude * 1000;

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
            double rotationAngle = Math.Acos(Util.Clamp(dot, -1d, 1d) * radToDeg);

            if (Vector3d.Dot(rotationAxis, MainAxis) < 0)
            {
                rotationAngle = 360.0 - rotationAngle;
            }

            // Convert angle to time in orbit
            double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / gravParameter);
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

            // Assuming that the TLI is performed from a circular orbit with altitude = parkingAltitude
            // Radius of the orbit, including the radius of the Earth
            double r0 = mainBody.Radius + parkingAltitude * 1000;

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
            //double minPossibleDV = 2500 * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius), gives 766 for Kerbin
            const double minPossibleDV = 1;
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

        private void ButtonPressed(bool pressed, ref bool button, bool reset = true)
        { // this doesn't need to be done for all buttons, just the ones that change the size of the window
            if (pressed)
            {
                button = !button;
                if (reset) ResetWindow(ref windowRect); // if needed in settings window, change this to be parameter
            }
        }

        private void ResetWindow(ref Rect rect)
        { // Doing this forces the window to be resized. Without it, the window will become bigger when controls expand, but never become smaller again
            rect = new Rect(rect.xMin, rect.yMin, -1, -1);
        }

        private void DrawLine()
        {
            GUILayout.Space(10);
            GUIStyle lineStyle = new GUIStyle();
            lineStyle.normal.background = Texture2D.whiteTexture;
            lineStyle.padding = new RectOffset(0, 0, 0, 0);
            lineStyle.margin = new RectOffset(0, 0, 0, 0);
            lineStyle.border = new RectOffset(0, 0, 0, 0);
            GUILayout.Box(GUIContent.none, lineStyle, GUILayout.Height(2), GUILayout.ExpandWidth(true));
            GUILayout.Space(10);
        }

        private void ShowOrbitInfo(ref bool showPhasing, double timeInOrbit, double phaseAngle)
        {
            if (showPhasing)
            {
                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Phasing angle", "Phasing angle between launch location and maneuver in orbit"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button("T", GUILayout.Width(20));
                ButtonPressed(showPhasing_pressed, ref showPhasing);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{phaseAngle:F2}\u00B0", $"{phaseAngle * degToRad:F5} rads"), GUILayout.MinWidth(60)); // for some reason the tooltip doesn't work for the immediate launch window, idk why
            }
            else
            {
                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Time in orbit", "Time spent waiting in parking orbit before maneuver"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button(" P", GUILayout.Width(20));
                ButtonPressed(showPhasing_pressed, ref showPhasing);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent(FormatTime(timeInOrbit), $"{timeInOrbit:0}s"), GUILayout.MinWidth(100));
            }
        }

        private void BeginCenter(bool isVertical = true)
        { // must be followed by EndCenter()
            if (isVertical) GUILayout.BeginVertical();
            else GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
        }

        private void EndCenter(bool isVertical = true)
        { // must follow BeginCenter()
            GUILayout.FlexibleSpace();
            if (isVertical) GUILayout.EndVertical();
            else GUILayout.EndHorizontal();
        }

        private void ExpandCollapse(ref bool button, string tooltip = "")
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5); // push down 5

            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace(); // push to right side
            bool button_pressed = GUILayout.Button(new GUIContent(!button ? "+" : "\u2013", tooltip), GUILayout.Width(30)); // en dash shows up as the same width as + ingame, while the minus symbol is way thinner
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            ButtonPressed(button_pressed, ref button);
        }

        private void MakeMainWindow(int id)
        {
            GUILayout.BeginVertical();

            //CelestialBody target = FlightGlobals.fetch.bodies.FirstOrDefault(body => body.name.Equals("Moon", StringComparison.OrdinalIgnoreCase));
            satellites = FlightGlobals.currentMainBody?.orbitingBodies?.OrderBy(body => body.bodyName).ToList(); // use currentMainBody here, target.referenceBody elsewhere

            GUILayout.Space(5);

            if (satellites == null || satellites.Count == 0)
            {
                GUILayout.Box("ERROR: There are no moons of this planet!", GUILayout.MinWidth(80));
            }
            else
            {
                currentUT = Planetarium.GetUniversalTime();
                if (target == null || !satellites.Contains(target)) target = satellites[0];
                inclination = GetTargetInclination(target.orbit);
                isLowLatitude = Math.Abs(latitude) <= inclination;
                CelestialBody mainBody = target.referenceBody;
                double dV = EstimateDV(target);

                inSurfaceVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null && (FlightGlobals.ActiveVessel.Landed || FlightGlobals.ActiveVessel.Splashed); // this needs to be here, as settings window isnt always open

                if (satellites.Count > 1) // only display target selector screen if needed
                {
                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button("<", GUILayout.MinWidth(20)))
                    {
                        currentBody--;
                        if (currentBody < 0) currentBody = satellites.Count - 1;
                    }

                    GUILayout.Box(target.bodyName, GUILayout.MinWidth(80));

                    if (GUILayout.Button(">", GUILayout.MinWidth(20)))
                    {
                        currentBody++;
                        if (currentBody > satellites.Count - 1) currentBody = 0;
                    }
                    GUILayout.EndHorizontal();
                    target = satellites[currentBody];
                    GUILayout.Space(5);
                }

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent($"Latitude: <b>{latitude:F2}\u00B0</b>", "Latitude of launch location"), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandLatLong, "Set manual latitude and longitude");
                GUILayout.EndHorizontal();

                if (expandLatLong)
                {
                    GUILayout.Space(6); // weird spacing
                    MakeNumberEditField("latitude", ref latitude, 1d, -90d, 90d, true);
                    GUILayout.Space(5);
                    GUILayout.Label(new GUIContent($"Longitude: <b>{longitude:F2}\u00B0</b>", "Longitude of launch location"));
                    MakeNumberEditField("longitude", ref longitude, 1d, -180d, 180d, true);
                    launchPos = mainBody.GetWorldSurfacePosition(latitude, longitude, 0);
                }
                else launchPos = GetLaunchPos(mainBody, ref latitude, ref longitude, useVesselPosition);

                GUILayout.Space(5);
                GUILayout.Label(new GUIContent("Flight Time (days)", $"Coast duration to {target.bodyName} after the maneuver (in {mainBody.bodyName} solar days)"), GUILayout.ExpandWidth(true));
                MakeNumberEditField("flightTime", ref flightTime, 0.1d, 0.1d, double.MaxValue);
                double l = Math.Round(flightTime * mainBody.solarDayLength);
                GUILayout.Box(new GUIContent(FormatTime(l), $"{l:0}s"), GUILayout.MinWidth(100)); // tooltips in Box have a problem with width, use {0:0}

                OrbitData launchOrbit = CalcOrbitForTime(target, launchPos, referenceTime);
                //Debug.Log("CLAYELADDEDDLOGS FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW");
                //var stopwatch = Stopwatch.StartNew();
                double nextLaunchETA = GetCachedLaunchTime(target, launchPos, inclination, useAltBehavior, 1) - currentUT;
                //stopwatch.Stop();
                //Debug.Log($"Window 1 Launch Time: {firstLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");
                //Debug.Log("CLAYELADDEDDLOGS SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW");
                //stopwatch = Stopwatch.StartNew();
                double extraLaunchETA = GetCachedLaunchTime(target, launchPos, inclination, useAltBehavior, extraWindowNumber) - currentUT;
                //stopwatch.Stop();
                //Debug.Log($"Window 2 Launch Time: {secondLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");

                //Debug.Log($"Initial warpState: {warpState}");
                bool inSpecialWarp = warpState == SpecialWarp.Warp2 || warpState == SpecialWarp.Warp3;
                bool specialWarpActive = warpState == SpecialWarp.Warp1 || inSpecialWarp;
                if (nextLaunchETA >= mainBody.rotationPeriod && PrincipiaInstalled && specialWarp && !inSpecialWarp) warpState = SpecialWarp.Warp1;
                else if (!inSpecialWarp && !specialWarpWait) warpState = SpecialWarp.None;
                //else if (nextLaunchETA < mainBody.rotationPeriod || PrincipiaInstalled) warpState = SpecialWarp.None;
                //Debug.Log($"Final warpState: {warpState}, inSpecialWarp: {inSpecialWarp}");

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Required \u0394V", "Required change in velocity for the maneuver if launched now"), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandAltitude, "Set parking orbit altitude");
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent($"{dV:F2} m/s", $"{dV / 1000d:F5} km/s"), GUILayout.MinWidth(100));

                if (expandAltitude)
                {
                    GUILayout.Label(new GUIContent("Parking Orbit (km)", "Planned altitude of the circular parking orbit before the maneuver"), GUILayout.ExpandWidth(true));
                    MakeNumberEditField("parkingAltitude", ref parkingAltitude, 5d, mainBody.atmosphere ? mainBody.atmosphereDepth / 1000d : 0d, target.orbit.PeA / 1000d - 5d); // PeA updates every frame so we don't need to ask Principia
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                if (showAzimuth)
                {
                    GUILayout.Label(new GUIContent(label + (showAzimuth ? "Az." : "Incl."), "Launch to this azimuth to get into the right parking orbit"), GUILayout.ExpandWidth(true));
                }
                else
                {
                    GUILayout.Label(new GUIContent(label + (showAzimuth ? "Az." : "Incl."), "Launch to this inclination to get into the right parking orbit (positive = North, negative = South, regardless of latitude sign)"), GUILayout.ExpandWidth(true));
                }
                ExpandCollapse(ref expandParking0);
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                if (showAzimuth)
                {
                    GUILayout.Box(new GUIContent($"{launchOrbit.azimuth:F2}\u00B0", $"{launchOrbit.azimuth * degToRad:F5} rads, this is {(launchOrbit.azimuth < 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                }
                else
                {
                    double launchInclination = launchOrbit.azimuth > 90d && launchOrbit.azimuth < 270d ? -launchOrbit.inclination : launchOrbit.inclination;
                    GUILayout.Box(new GUIContent($"{launchInclination:F2}\u00B0", $"{launchInclination * degToRad:F5} rads, this is {(launchOrbit.azimuth < 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                }

                    switch (referenceTimeButton)
                    {
                        case 0:
                            referenceTimeLabel = "Launch Now";
                            referenceTimeTooltip = "Change to Next Launch Window";
                            label = "Launch Now ";
                            referenceTime = 0d;
                            break;
                        case 1:
                            referenceTimeLabel = "Next Window";
                            referenceTimeTooltip = $"Change to Launch Window {extraWindowNumber}";
                            label = "Next Window ";
                            referenceTime = nextLaunchETA;
                            break;
                        case 2:
                            referenceTimeLabel = $"Window {extraWindowNumber}";
                            referenceTimeTooltip = "Change to Current Launch Window";
                            label = $"Window {extraWindowNumber} ";
                            referenceTime = extraLaunchETA;
                            break;
                    }

                if (expandParking0)
                {
                    (double timeInOrbit0, double phaseAngle0) = EstimateFlightTimeBeforeTLI(target, launchPos, referenceTime);
                    ShowOrbitInfo(ref showPhasing, timeInOrbit0, phaseAngle0);
                }

                string tooltip = isLowLatitude ?
                    "Launch Easterly at this time to get into the right parking orbit" :
                    "Launch at this time at this inclination to get into the right parking orbit";

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Next Window", tooltip), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandParking1);
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent(FormatTime(nextLaunchETA), $"UT: {currentUT + nextLaunchETA:0}s"), GUILayout.MinWidth(100)); // itll flash every second if we just do {nextLaunchETA}, we need absolute time

                if (expandParking1)
                {
                    (double timeInOrbit1, double phaseAngle1) = EstimateFlightTimeBeforeTLI(target, launchPos, nextLaunchETA);
                    ShowOrbitInfo(ref showPhasing, timeInOrbit1, phaseAngle1);
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent($"Window {extraWindowNumber}", "Extra Window: " + tooltip), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandParking2);
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent(FormatTime(extraLaunchETA), $"UT: {currentUT + extraLaunchETA:0}s"), GUILayout.MinWidth(100));

                if (expandParking2)
                {
                    (double timeInOrbit2, double phaseAngle2) = EstimateFlightTimeBeforeTLI(target, launchPos, extraLaunchETA); // itll flash every second if we just do {extraLaunchETA}, we need absolute time
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

                bool toggleWarp = GUILayout.Button(new GUIContent(TimeWarp.CurrentRate > 1d || inSpecialWarp ? "Stop Warp" : "Warp", "Warp to the Next Window, taking into account the Warp Margin"), GUILayout.MinWidth(80));
                GUILayout.EndHorizontal();

                if (specialWarpActive)
                {
                    BeginCenter(false);
                    GUILayout.Label(new GUIContent("Special Warp", "See the settings for an in-depth explanation, this CANNOT be halted once started"));
                    EndCenter(false);
                }

                // the error "Getting control 2's position in a group with only 2 controls when doing repaint..." is thrown when changing the visibility of this label for some reason (always with EndCenter)
                // its harmless as far as I can tell, but i couldnt figure out a way to actually catch it

                if (specialWarpBuffer != specialWarpActive) // only reset if changed
                {
                    specialWarpBuffer = !specialWarpBuffer;
                    ResetWindow(ref windowRect);
                }

                GUILayout.BeginHorizontal();
                bool targetPressed = GUILayout.Button(targetSet ? "Unset Target" : $"Target {target.bodyName}", GUILayout.MinWidth(140));

                bool showSettings_pressed;

                if (gearBlack != null && gearGreen != null)
                {
                    showSettings_pressed = GUILayout.Button(new GUIContent(useAltSkin ? gearBlack : gearGreen, "Show Settings"), new GUIStyle(GUI.skin.button) { padding = new RectOffset(0, 0, 0, 0) }, GUILayout.Width(20), GUILayout.Height(20)); // remove padding in style to prevent image getting scaled down with unity skin

                    //ButtonPressed(showSettings_pressed, ref showSettings, false);
                }
                else
                {
                    showSettings_pressed = GUILayout.Button(new GUIContent(" ", "Show Settings (Error: A settings icon is missing!)"), GUILayout.Width(20), GUILayout.Height(20));
                }
                ButtonPressed(showSettings_pressed, ref showSettings, false);
                GUILayout.EndHorizontal();

                if (addAlarm && KACInstalled)
                {
                    if (!useAltAlarm && !double.IsNaN(nextLaunchETA))
                    {
                        string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, $"{target.bodyName} Launch Window", currentUT + nextLaunchETA - warpMargin);
                        if (!string.IsNullOrEmpty(alarmId))
                        {
                            //if the alarm was made get the object so we can update it
                            KACWrapper.KACAPI.KACAlarm alarm = KACWrapper.KAC.Alarms.First(z => z.ID == alarmId);

                            //Now update some of the other properties
                            alarm.AlarmAction = KACWrapper.KACAPI.AlarmActionEnum.KillWarp;
                        }
                    }
                    else if (useAltAlarm && !double.IsNaN(extraLaunchETA))
                    {
                        string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, $"{target.bodyName} Launch Window {extraWindowNumber}", currentUT + extraLaunchETA - warpMargin);
                        if (!string.IsNullOrEmpty(alarmId))
                        {
                            //if the alarm was made get the object so we can update it
                            KACWrapper.KACAPI.KACAlarm alarm = KACWrapper.KAC.Alarms.First(z => z.ID == alarmId);

                            //Now update some of the other properties
                            alarm.AlarmAction = KACWrapper.KACAPI.AlarmActionEnum.KillWarp;
                        }
                    }
                }

                bool inWarp() => TimeWarp.CurrentRate > 1d;

                if (toggleWarp)
                {
                    if (inWarp())
                    {
                        TimeWarp.fetch.CancelAutoWarp();
                        TimeWarp.SetRate(0, false);
                    }
                    else
                    {
                        if (warpState == SpecialWarp.Warp1)
                        {
                            //Debug.Log("Begining special warp 1");
                            TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                            TimeWarp.fetch.WarpTo(currentUT + nextLaunchETA - (warpMargin + mainBody.rotationPeriod)); // warp to within a day
                            Debug.Log("Special warp 1 in progress");
                            //specialWarp2 = true;
                            warpState = SpecialWarp.Warp2;
                            specialWarpWait = true;
                        }
                        else
                        {
                            TimeWarp.fetch.WarpTo(currentUT + nextLaunchETA - warpMargin);
                        }
                    }
                }

                if (specialWarpWait && !inWarp())
                {
                    waitingTime = Planetarium.GetUniversalTime();
                    //Debug.Log($"waitingTime: {waitingTime}");
                    cache.Clear(); // make sure we have an up to date time
                    specialWarpWait = false;
                }

                if (warpState == SpecialWarp.Warp2 && !inWarp() && currentUT > waitingTime + 0.5d)
                {
                    //Debug.Log($"currentUT: {currentUT}");

                    //Debug.Log("Beginning special warp 2");
                    TimeWarp.fetch.CancelAutoWarp();
                    //TimeWarp.fetch.CancelAutoWarp(-1, true);
                    //TimeWarp.SetRate(5, false);
                    //TimeWarp.SetRate(0, true);
                    TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                    TimeWarp.fetch.WarpTo(currentUT + nextLaunchETA - (warpMargin + 3600d * mainBody.rotationPeriod / EarthSiderealDay)); // warp to within an hour
                    //specialWarp2 = false;
                    //specialWarp3 = true;
                    warpState = SpecialWarp.Warp3;
                    specialWarpWait = true;

                    Debug.Log("Special warp 2 in progress");
                }

                if (warpState == SpecialWarp.Warp3 && !inWarp() && currentUT > waitingTime + 0.5d)
                {
                    //Debug.Log($"currentUT: {currentUT}");

                    //Debug.Log("Beginning special warp 3");
                    TimeWarp.fetch.CancelAutoWarp();
                    //TimeWarp.fetch.CancelAutoWarp(-1, true);
                    //TimeWarp.SetRate(5, false);
                    //TimeWarp.SetRate(0, true);
                    TimeWarp.fetch.WarpTo(currentUT + nextLaunchETA - warpMargin); // now warp to final
                    warpState = SpecialWarp.None;
                    specialWarpWait = false;

                    Debug.Log("Special warp 3 in progress");
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
                //        TimeWarp.fetch.WarpTo(currentUT + nextLaunchETA - warpMargin);
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

        private void MakeSettingsWindow(int id)
        {
            GUILayout.BeginVertical();

            GUILayout.Label("Hover over text to see additional information", GUILayout.Width(350)); // use this to set the width of the window

            GUILayout.BeginHorizontal();
            GUILayout.Label("Use Unity Skin");
            BeginCenter();
            bool useAltSkin_toggled = GUILayout.Toggle(useAltSkin, "");
            EndCenter();
            GUILayout.EndHorizontal();

            if (useAltSkin_toggled != useAltSkin)
            {
                useAltSkin = !useAltSkin;
                ResetWindow(ref windowRect);
                ResetWindow(ref settingsRect);
            }

            DrawLine();
            ;
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Find Global Minimum inclination instead of Local Minimum", $"Ignored if latitude is higher than {target.bodyName} inclination"));
            GUI.enabled = isLowLatitude;
            BeginCenter();
            bool useAltBehavior_toggled = GUILayout.Toggle(useAltBehavior, "");
            EndCenter();
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            //if (!isLowLatitude) useAltBehavior_toggled = false; // if we change useAltBehavior to false instead, the next code will just set it to true again // not needed, we check for isLowLatitude anyway

            if (useAltBehavior != useAltBehavior_toggled)
            {
                useAltBehavior = useAltBehavior_toggled;
                //Debug.Log("cache Cleared due to useAltBehavior change");
                cache.Clear(); // remove local mins so that we can replace them with global mins
                // technically this only needs to be done when switching from false to true, but switching from true to false without clearing would result in some extra data in the cache, which might lead to problems if abused
            }

            DrawLine();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Use surface vessel position for latitude/longitude instead of launch site position", "Ignored if not in a vessel on the surface")); // can enable when not on surface?
            GUI.enabled = inSurfaceVessel;
            BeginCenter();
            useVesselPosition = GUILayout.Toggle(useVesselPosition, "");
            EndCenter();
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            if (KACInstalled)
            {
                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Change the \"Add Alarm\" button to set an alarm based on the extra window instead of the next launch window");
                BeginCenter();
                useAltAlarm = GUILayout.Toggle(useAltAlarm, "");
                EndCenter();
                GUILayout.EndHorizontal();
            }

            if (PrincipiaInstalled)
            {
                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("<b>Special Warp</b>: Change the \"Warp\" button to use 3 warps to avoid overshooting/undershooting the launch window due to perturbations of the target's orbit. It CANNOT be halted once started.", "Only visible when Principia is installed, and only activates when the next window is more than 1 sidereal day away"));
                BeginCenter();
                specialWarp = GUILayout.Toggle(specialWarp, "");
                EndCenter();
                GUILayout.EndHorizontal();
            }

            DrawLine();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Show Azimuth instead of Inclination");
            BeginCenter();
            showAzimuth = GUILayout.Toggle(showAzimuth, "");
            EndCenter();
            GUILayout.EndHorizontal();

            DrawLine();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Switch between reference times for the launch {(showAzimuth ? "azimuth" : "inclination")}");
            BeginCenter();
            if (GUILayout.Button(new GUIContent(referenceTimeLabel, referenceTimeTooltip), GUILayout.MinWidth(120))) referenceTimeButton = (referenceTimeButton + 1) % 3;
            EndCenter();
            GUILayout.EndHorizontal();

            DrawLine();

            GUILayout.Label("Change Extra Window Number");
            MakeNumberEditField("extraWindowNumber", ref extraWindowNumber, 1, 2, maxWindows);

            DrawLine();

            // azimuth to inclination formulas derived from https://www.astronomicalreturns.com/p/section-46-interesting-orbital.html
            // sin(azimuth) = cos(inclination) / cos(latitude)
            // cos(inclination) = cos(latitude) * sin(azimuth)

            double originalAzimuth = targetAzimuth;
            double azRad = targetAzimuth * degToRad;
            double latRad = latitude * degToRad;
            if (double.IsNaN(targetInclination)) targetInclination = targetAzimuth <= 90d || targetAzimuth >= 270d
                ? Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg
                : -Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg; // initialize value, this isnt rounded but rounding led to floating point issues so whatever

            if (showAzimuth)
            {
                targetInclination = targetAzimuth <= 90d || targetAzimuth >= 270d
                ? Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg
                : -Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg; // continuously update value

                GUILayout.Label(new GUIContent("Change Target Launch Azimuth", "90\u00B0 is the default, which is directly east. Range is 0\u00B0 to 360\u00B0, where 0\u00B0 and 180\u00B0 are North and South respectively. Changing the Target Launch Azimuth may not change the launch window time, this is normal and expected."));
                MakeNumberEditField("targetAzimuth", ref targetAzimuth, 1d, 0d, 360d, true);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Target Launch Inclination", $"Your latitude of {latitude:F2}\u00B0 is the default, which is directly east. Range is -180\u00B0 to 180\u00B0, where +90\u00B0 and -90\u00B0 are North and South respectively."));
                GUILayout.Box($"{targetInclination:F2}\u00B0", GUILayout.MaxWidth(100)); // F2 is overkill but just in case
                GUILayout.EndHorizontal();
            }
            else
            {
                GUILayout.Label(new GUIContent("Change Target Launch Inclination", $"Your latitude of {latitude:F2}\u00B0 is the default, which is directly east. Range is -180\u00B0 to 180\u00B0, where +90\u00B0 and -90\u00B0 are North and South respectively. Changing the Target Launch Inclination may not change the launch window time, this is normal and expected."));

                GUILayout.BeginHorizontal();
                MakeNumberEditField("targetInclination", ref targetInclination, 1d, -180d, 180d, true);

                //double absInc = Math.Abs(targetInclination);
                double cosInc = Math.Cos(targetInclination * degToRad);
                double cosLat = Math.Cos(latRad);
                double sinAz = cosInc / cosLat;

                if (Math.Abs(sinAz) > 1d)
                {
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(new GUIContent("Unreachable", $"The Target Inclination of {targetInclination:F2}° is unreachable from your latitude of {latitude:F2}°, so it has been clamped to the nearest reachable inclination."));
                    GUILayout.FlexibleSpace();
                }
                GUILayout.EndHorizontal();

                sinAz = Util.Clamp(sinAz, -1d, 1d);
                double azInter = Math.Abs(Math.Asin(sinAz) * radToDeg); // intermediate value for azimuth

                targetAzimuth = targetInclination >= 0
                    ? (targetInclination <= 90d ? azInter : 360d - azInter) // NE (prograde) or NW (retrograde)
                    : (Math.Abs(targetInclination) <= 90d ? 180d - azInter : 180d + azInter); // SE (prograde) or SW (retrograde)

                targetAzimuth = (targetAzimuth + 360d) % 360d; // normalize just in case

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Target Launch Azimuth", "90\u00B0 is the default, which is directly east. Range is 0\u00B0 to 360\u00B0, where 0\u00B0 and 180\u00B0 are North and South respectively."));
                GUILayout.Box($"{targetAzimuth:F2}\u00B0", GUILayout.MaxWidth(100));
                GUILayout.EndHorizontal();
            }
            if (targetAzimuth != originalAzimuth)
            {
                cache.Clear(); // only clear if changed, also this doesn't always result in new minimums
            }

            Tooltip.Instance.RecordTooltip(id);

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        // html tags rendered by ksp: <b> and </b>, <i> and </i>, (add more)
    }
}
