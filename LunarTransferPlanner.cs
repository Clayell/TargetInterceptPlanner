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

using ClickThroughFix;
using ToolbarControl_NS;
using KerbalAlarmClock;
using KSP.UI.Screens;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics; // use this for stopwatch
using System.IO;
using System.Linq;
using UnityEngine;
using Debug = UnityEngine.Debug; // needed to avoid conflict with System.Diagnostics.Debug

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
                return Math.Log(x + Math.Sqrt(x * x - 1)); // ln(x + sqrt(x^2 - 1))
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

    [KSPAddon(KSPAddon.Startup.MainMenu, true)] // startup on menu according to https://github.com/linuxgurugamer/ToolbarControl/wiki/Registration
    public class RegisterToolbar : MonoBehaviour
    {
        void Start()
        {
            ToolbarControl.RegisterMod("LTP", "Lunar Transfer Planner");
        }
    }

    [KSPAddon(KSPAddon.Startup.AllGameScenes, false)] // TODO, stop it from appearing in R&D, Admininistration, Astronaut Complex
    public class LunarTransferPlanner : MonoBehaviour
    {
        const double EarthSiderealDay = 86164.098903691;
        const double EarthRadius = 6371000; // meters
        const double EarthMass = 3.9860043543609598e+14 / 6.673e-11;
        // Earth sources from RealSolarSystem/RSSKopernicus/Earth/Earth.cfg
        const double tau = 2 * Math.PI;
        const double radToDeg = 180d / Math.PI; // unity only has floats
        const double degToRad = Math.PI / 180d; // unity only has floats
        readonly double invphi = (Math.Sqrt(5) - 1) / 2; // positive conjugate of golden ratio

        Rect mainRect = new Rect(100, 100, -1, -1);
        Rect settingsRect = new Rect(100, 100, -1, -1);
        readonly string mainTitle = "Lunar Transfer";
        readonly string settingsTitle = "Additional Settings";
        int windowWidth;
        GUISkin skin;
        Texture2D gearBlack;
        Texture2D gearGreen;
        bool isWindowOpen = false;
        bool isKSPGUIActive = true; // for some reason, this initially only turns to true when you turn off and on the KSP GUI

        int currentBody = 0;
        object target = null; // will later become CelestialBody or Vessel
        CelestialBody mainBody = null;
        Orbit targetOrbit = null;
        string targetName = string.Empty;
        Vector3d launchPos;
        bool targetVessel = false;
        bool moonsInvalid = true;
        bool vesselsInvalid = true;
        bool KACInstalled;
        bool PrincipiaInstalled;

        double inclination; // inclination of target's orbit
        double currentUT;
        double dayScale;
        double solarDayLength;
        double targetAltitude;
        bool useHomeSolarDay = true;
        int errorState = 0;
        bool showSettings = false; // Show settings UI
        bool useAltSkin = false; // Use Unity GUI skin instead of default
        double flightTime = 4.0d; // Desired flight time after maneuver, in solar days of homeBody or mainBody (depending on useHomeSolarDay)
        double parkingAltitude = 200d; // Parking orbit altitude (circular orbit)
        bool useAltBehavior = false; // Find global minimum for low latitudes instead of local minimum
        double altBehaviorTimeLimit = 30d; // Max time limit for the global minimum search, in sidereal days of mainBody
        bool altBehaviorNaN = false; // Return NaN when a global min can't be found, instead of returning the closest time
        bool useVesselPosition = true; // Use vessel position for latitude instead of launch site position, default is true as the KSC location isn't always the same as the actual launch site directly from the VAB
        bool requireSurfaceVessel = true; // Do not consider useVesselPosition if the vessel is not on the surface
        bool inVessel;
        bool useAltAlarm = false; // Set an alarm based on the extra window instead of the next launch window
        double latitude = 0d;
        double longitude = 0d;
        //double altitude = 0d; // height above sea level in meters // currently, this has no effect on the calculations, because the vectors are always normalized

        int referenceTimeButton = 0;
        string referenceTimeLabel;
        string referenceTimeTooltip;
        string launchLabel;
        double referenceTime = 0d;
        int? referenceWindowNumber = null;

        double targetLaunchAzimuth = 90d; // due east
        double targetLaunchInclination = double.NaN;
        double targetPhasingAngle = 180d;
        double targetPhasingTime = double.NaN;
        bool showAzimuth = false;
        bool expandExtraWindow = true; // Show the extra window chooser
        int extraWindowNumber = 2; // counting from 1
        bool useWindowOptimizer = false; // Optimize for a certain phasing angle/time
        bool expandLatLong = false; // Expand/collapse custom Latitude/Longitude picker
        bool expandAltitude = false; // Expand/collapse parking orbit altitude changer
        bool expandParking0 = false; // Expand/collapse time in parking orbit for launch now
        bool expandParking1 = false; // Expand/collapse time in parking orbit for next window
        bool expandParking2 = false; // Expand/collapse time in parking orbit for extra window
        double deltaVTolerance = 0.01d; // Tolerance (in seconds) of how close the estimated flight time given by the delta-V should be to the expected flight time
        double maxDeltaVScaled = 10000d; // Max amount of delta-V that can be calculated, scaled based on the length of a sidereal day for the main body
        bool ranSearch = false; // If we've ran the targetPhasingAngle/Time search
        int decimals = 2; // Amount of decimal precision to display
        bool showPhasing = false; // Show the phasing angle instead of the time in parking orbit, applies to all boxes
        int maxWindows = 100; // Maximum amount of extra windows that can be calculated
        bool isLowLatitude;
        bool targetSet = false;
        double warpMargin = 60d; // Time difference from the launch window that the warp will stop at

        bool specialWarpSelected = true;
        int warpState = 0;
        bool specialWarpWait = false;
        double waitingTime;

        List<CelestialBody> moons;
        List<Vessel> vessels;
        readonly List<(object target, double targetAltitude, double latitude, double longitude, double inclination, double absoluteLaunchTime)> windowCache = new List<(object, double, double, double, double, double)>();
        readonly List<(OrbitData launchOrbit, int windowNumber)> launchOrbitCache = new List<(OrbitData, int)>();
        readonly List<(double phasingTime, double phasingAngle, int windowNumber)> phasingCache = new List<(double, double, int)>();
        (object target, double targetAltitude, double deltaV, double eccentricity)? deltaVCache = null;
        // TODO, when increasing flight time, eventually we'll hit a minimum deltaV that wont decrease any further (for a given target and parking altitude). this should be told to the user
        readonly Dictionary<string, double> nextTickMap = new Dictionary<string, double>();
        readonly Dictionary<string, string> textBuffer = new Dictionary<string, string>();
        readonly Dictionary<string, object> stateBuffer = new Dictionary<string, object>();
        readonly string SettingsPath = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/LunarTransferPlanner/PluginData/settings.cfg");

        ToolbarControl toolbarControl = null;

        #region GUI Setup

        private void InitToolbar()
        {
            if (toolbarControl == null)
            {
                toolbarControl = gameObject.AddComponent<ToolbarControl>();
                toolbarControl.AddToAllToolbars(ToggleWindow, ToggleWindow,
                    ApplicationLauncher.AppScenes.ALWAYS & ~ApplicationLauncher.AppScenes.MAINMENU, // all but main menu
                    "LTP",
                    "LTP_Button",
                    "LunarTransferPlanner/PluginData/ToolbarIcons/button-64",
                    "LunarTransferPlanner/PluginData/ToolbarIcons/button-24",
                    "Lunar Transfer Planner"
                );
            }
        }

        private void ToggleWindow() => isWindowOpen = !isWindowOpen;

        void Awake()
        {
            skin = (GUISkin)GUISkin.Instantiate(HighLogic.Skin);
            skin.button.padding = new RectOffset(2, 2, 2, 2);
            skin.button.margin = new RectOffset(1, 1, 1, 1);
            skin.box.padding = new RectOffset(2, 2, 2, 2);
            skin.box.margin = new RectOffset(1, 1, 1, 1);
            skin.textField.margin = new RectOffset(3, 1, 1, 1);
            skin.textField.padding = new RectOffset(4, 2, 1, 0);

            gearBlack = GameDatabase.Instance.GetTexture("LunarTransferPlanner/Icons/gearBlack", false);
            gearGreen = GameDatabase.Instance.GetTexture("LunarTransferPlanner/Icons/gearGreen", false);

            LoadSettings();

            GameEvents.onShowUI.Add(KSPShowGUI);
            GameEvents.onHideUI.Add(KSPHideGUI);
        }

        // for some reason the button icons only load if they're in PluginData, but the setting icons only load if they're NOT in PluginData /shrug

        void KSPShowGUI() => isKSPGUIActive = true;

        void KSPHideGUI() => isKSPGUIActive = false;

        void Start()
        {
            InitToolbar();

            KACWrapper.InitKACWrapper();
            KACInstalled = KACWrapper.APIReady;

            PrincipiaWrapper.InitPrincipiaWrapper();
            PrincipiaInstalled = PrincipiaWrapper.APIReady;

            Tooltip.RecreateInstance(); // Need to make sure that a new Tooltip instance is created after every scene change
        }

        void OnDestroy()
        {
            SaveSettings();
            if (toolbarControl != null)
            {
                toolbarControl.OnDestroy();
                Destroy(toolbarControl);
            }

            GameEvents.onShowUI.Remove(KSPShowGUI);
            GameEvents.onHideUI.Remove(KSPHideGUI);
        }

        void OnGUI()
        {
            if (isWindowOpen && isKSPGUIActive)
            {
                GUI.skin = !useAltSkin ? this.skin : null;
                mainRect = ClickThruBlocker.GUILayoutWindow(this.GetHashCode(), mainRect, MakeMainWindow, mainTitle);
                ClampToScreen(ref mainRect);
                Tooltip.Instance?.ShowTooltip(this.GetHashCode());

                if (showSettings)
                {
                    settingsRect = ClickThruBlocker.GUILayoutWindow(this.GetHashCode() + 1, settingsRect, MakeSettingsWindow, settingsTitle);
                    ClampToScreen(ref settingsRect);
                    Tooltip.Instance?.ShowTooltip(this.GetHashCode() + 1);
                }
            }
        }

        private void ClampToScreen(ref Rect rect)
        {
            float left = Mathf.Clamp(rect.x, 0, Screen.width - rect.width);
            float top = Mathf.Clamp(rect.y, 0, Screen.height - rect.height);
            rect = new Rect(left, top, rect.width, rect.height);
        }

        #endregion
        #region Settings

        private void SaveSettings()
        {
            ConfigNode settings = new ConfigNode("SETTINGS");

            Dictionary<string, object> settingValues = new Dictionary<string, object>
            {
                { "mainRect.xMin", mainRect.xMin },
                { "mainRect.yMin", mainRect.yMin },
                { "settingsRect.xMin", settingsRect.xMin },
                { "settingsRect.yMin", settingsRect.yMin },
                { "isWindowOpen", isWindowOpen },
                { "targetVessel", targetVessel },
                { "showSettings", showSettings },
                { "useAltSkin", useAltSkin },
                { "useHomeSolarDay", useHomeSolarDay },
                { "flightTime", flightTime },
                { "parkingAltitude", parkingAltitude },
                { "useAltBehavior", useAltBehavior },
                { "altBehaviorTimeLimit", altBehaviorTimeLimit },
                { "altBehaviorNaN", altBehaviorNaN },
                { "useVesselPosition", useVesselPosition },
                { "requireSurfaceVessel", requireSurfaceVessel },
                { "latitude", latitude },
                { "longitude", longitude },
                { "showAzimuth", showAzimuth },
                { "expandExtraWindow", expandExtraWindow },
                { "extraWindowNumber", extraWindowNumber },
                { "useWindowOptimizer", useWindowOptimizer },
                { "expandLatLong", expandLatLong },
                { "expandAltitude", expandAltitude },
                { "expandParking0", expandParking0 },
                { "expandParking1", expandParking1 },
                { "expandParking2", expandParking2 },
                { "deltaVTolerance", deltaVTolerance },
                { "maxDeltaVScaled", maxDeltaVScaled },
                { "showPhasing", showPhasing },
                { "maxWindows", maxWindows },
                { "warpMargin", warpMargin },
                { "specialWarpSelected", specialWarpSelected },
                { "targetLaunchAzimuth", targetLaunchAzimuth },
                { "targetPhasingAngle", targetPhasingAngle },
            };

            foreach (KeyValuePair<string, object> kvp in settingValues) settings.AddValue(kvp.Key, kvp.Value);

            ConfigNode root = new ConfigNode();
            root.AddNode(settings);
            root.Save(SettingsPath);

            Dictionary<string, string> comments = new Dictionary<string, string>
            {
                { "maxWindows", "Changes the maximum amount of windows that can be calculated with the extra window chooser (or considered in the phasing angle/time optimizer), default of 100. Each launch window is temporarily cached, so caching a ridiculous amount may lead to performance degradation" },
                { "altBehaviorTimeLimit", "Max time limit for the global minimum search in sidereal days of the main body, default of 30. Increase this if you're getting close local minimums instead of absolute global minimums" },
                { "altBehaviorNaN", "Return a NaN when a global minimum cannot be found within the time limit, instead of returning the best local minimum" },
                { "deltaVTolerance", "Tolerance (in seconds) of how close the estimated flight time given by the delta-V should be to the expected flight time, default of 0.01 seconds. Decrease if you want more accuracy" },
                { "maxDeltaVScaled", "Max amount of delta-V that can be calculated, scaled based on the length of a sidereal day for the main body, default of 10000 (the Moon is about 3100). Increase if you're getting NaNs with very far-out moons/vessels" },
                { "targetLaunchAzimuth", "Target Inclination is converted to and from Target Azimuth automatically" },
                { "targetPhasingAngle", "Target Phasing Time is converted to and from Target Phasing Angle automatically" },
                { "requireSurfaceVessel", "For useVesselPosition, require that the vessel be on the surface (landed or splashed) for the position to actually be considered" },
                { "useHomeSolarDay", "Use the solar day length of the home body, instead of the currently focused main body, for the purpose of formatting times" },
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
                    float x1 = mainRect.xMin, y1 = mainRect.yMin;
                    float x2 = settingsRect.xMin, y2 = settingsRect.yMin;

                    void Read<T>(ref T field, string name) => Util.TryReadValue(ref field, settings, name);

                    Read(ref x1, "mainRect.xMin");
                    Read(ref y1, "mainRect.yMin");
                    Read(ref x2, "settingsRect.xMin");
                    Read(ref y2, "settingsRect.yMin");

                    Read(ref isWindowOpen, "isWindowOpen");
                    Read(ref targetVessel, "targetVessel");
                    Read(ref showSettings, "showSettings");
                    Read(ref useAltSkin, "useAltSkin");
                    Read(ref flightTime, "flightTime");
                    Read(ref useHomeSolarDay, "useHomeSolarDay");
                    Read(ref parkingAltitude, "parkingAltitude");
                    Read(ref useAltBehavior, "useAltBehavior");
                    Read(ref altBehaviorTimeLimit, "altBehaviorTimeLimit");
                    Read(ref altBehaviorNaN, "altBehaviorNaN");
                    Read(ref useVesselPosition, "useVesselPosition");
                    Read(ref requireSurfaceVessel, "requireSurfaceVessel");
                    Read(ref latitude, "latitude");
                    Read(ref longitude, "longitude");
                    Read(ref showAzimuth, "showAzimuth");
                    Read(ref expandExtraWindow, "expandExtraWindow");
                    Read(ref extraWindowNumber, "extraWindowNumber");
                    Read(ref useWindowOptimizer, "useWindowOptimizer");
                    Read(ref expandLatLong, "expandLatLong");
                    Read(ref expandAltitude, "expandAltitude");
                    Read(ref expandParking0, "expandParking0");
                    Read(ref expandParking1, "expandParking1");
                    Read(ref expandParking2, "expandParking2");
                    Read(ref deltaVTolerance, "deltaVTolerance");
                    Read(ref maxDeltaVScaled, "maxDeltaVScaled");
                    Read(ref showPhasing, "showPhasing");
                    Read(ref maxWindows, "maxWindows");
                    Read(ref warpMargin, "warpMargin");
                    Read(ref specialWarpSelected, "specialWarpSelected");
                    Read(ref targetLaunchAzimuth, "targetLaunchAzimuth");
                    Read(ref targetPhasingAngle, "targetPhasingAngle");

                    mainRect = new Rect(x1, y1, mainRect.width, mainRect.height);
                    settingsRect = new Rect(x2, y2, settingsRect.width, settingsRect.height);
                }
            }
        }

        #endregion
        #region Helper Methods

        private void Log(string message)
        {
            Debug.Log($"[LunarTransferPlanner]: {message}");
        }

        private void LogError(string message)
        {
            Debug.LogError($"[LunarTransferPlanner]: {message}");
        }


        public PQSCity FindKSC(CelestialBody home)
        {
            if (home != null)
            {
                if (home.pqsController != null && home.pqsController.transform != null)
                {
                    Transform t = home.pqsController.transform.Find("KSC");
                    if (t != null)
                    {
                        PQSCity KSC = (PQSCity)t.GetComponent(typeof(PQSCity));
                        if (KSC != null)
                        {
                            return KSC;
                        }
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
            if (!(useVesselPosition && inVessel) && SpaceCenter.Instance == null)
                return Vector3d.zero;

            // currently, altitude has no effect on the calculations with launchPos, because the vectors are always normalized. uncomment altitude if we do need it
            if (useVesselPosition && inVessel) // double check if it is safe to avoid null-ref
            {
                Vessel vessel = FlightGlobals.ActiveVessel;
                Latitude = vessel.latitude;
                Longitude = vessel.longitude;
                //Altitude = vessel.altitude;
            }
            else
            {
                if (SpaceCenter.Instance != null)
                {
                    PQSCity ksc = FindKSC(FlightGlobals.GetHomeBody());
                    if (ksc)
                    {
                        Latitude = ksc.lat;
                        Longitude = ksc.lon;
                        //Altitude = ksc.alt;
                    }
                    else
                    {
                        Latitude = SpaceCenter.Instance.Latitude;
                        Longitude = SpaceCenter.Instance.Longitude;
                        //Altitude = 0d; // TODO, find a way to get altitude of SpaceCenter? tbh idek when we use SpaceCenter
                    }
                }
            }

            return mainBody.GetWorldSurfacePosition(Latitude, Longitude, 0d);
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
                LogError("Invalid orbit passed to GetTargetInclination.");
                throw new ArgumentException("[LunarTransferPlanner] Invalid orbit passed to GetTargetInclination.");
            }
        }

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
            // add LAN to OrbitData? TODO
        }

        #endregion
        #region Main Methods

        OrbitData CalcOrbitForTime(Vector3d launchPos, double startTime)
        {
            // Form a plane with the launch site, target and mainBody centre, use this as the orbital plane for launch
            //CelestialBody mainBody = target.referenceBody;
            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = currentUT + flightTime * solarDayLength + startTime;
            Vector3d targetPos = targetOrbit.getPositionAtUT(targetTime); // this doesn't take into account changing target inclination due to principia
            //CelestialGetPosition is the corresponding method for Principia, but it doesn't work for a future time. TODO

            Vector3d upVector = QuaternionD.AngleAxis(startTime * 360d / mainBody.rotationPeriod, MainAxis) * (launchPos - MainPos).normalized; // use rotationPeriod for sidereal time

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

        double EstimateLaunchTime(Vector3d launchPos, double startTime, bool useAltBehavior)
        {
            if (!isLowLatitude) useAltBehavior = false; // this only changes the parameter

            const double epsilon = 1e-9;
            const double tolerance = 0.01;
            const double buffer = 1d;

            if (Math.Abs(targetOrbit.period - mainBody.rotationPeriod) < epsilon)
            {
                Log("Target orbital period exactly equals the sidereal day length, so a window cannot be found. Returning NaN.");
                return double.NaN;
            }

            double coarseStep = 1200d * dayScale;
            double alignmentMultiplier = targetOrbit.period / Math.Abs(targetOrbit.period - mainBody.rotationPeriod); // this is the number of rotations per alignment cycle, it approaches infinity as the orbital period and rotation period converge
            double maxTimeLimit = mainBody.rotationPeriod * (useAltBehavior ? altBehaviorTimeLimit : 1d) * alignmentMultiplier; // expand to 30 days (altBehaviorTimeLimit) to search for global min

            //Log($"beginning, coarseStep: {coarseStep}, maxTimeLimit: {maxTimeLimit}, startTime: {startTime}");

            double AzimuthError(double t)
            {
                double az = GetCachedLaunchOrbit(launchPos, t).azimuth;
                //Log($"az: {az}");
                return Math.Abs(((az - targetLaunchAzimuth + 540d) % 360d) - 180d);
            }
            // dont turn this into InclinationError, CalcOrbitForTime is limited in which inclinations it can return, but it can return 0 to 360 of azimuth

            double GoldenSectionSearch(double lowerBound, double upperBound) // adopted from https://en.wikipedia.org/wiki/Golden-section_search
            {
                // I have no idea why this works, but it works so well
                double left = upperBound - invphi * (upperBound - lowerBound);
                double right = lowerBound + invphi * (upperBound - lowerBound);

                double eLeft = AzimuthError(left);
                double eRight = AzimuthError(right);

                //Log($"lowerBound: {lowerBound}, upperBound: {upperBound}, left: {left}, right: {right}, eLeft: {eLeft}, eRight: {eRight}");

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

                //Log($"(upperBound + lowerBound) / 2d: {(upperBound + lowerBound) / 2d}");

                return (upperBound + lowerBound) / 2d;
            }

            if (useAltBehavior)
            {
                double candidateTime = startTime;

                double bestTime = double.NaN;
                double smallestError = double.MaxValue;

                while (candidateTime <= startTime + maxTimeLimit)
                {
                    double refinedTime = EstimateLaunchTime(launchPos, candidateTime, false); // recursive call with useAltBehavior = false
                    if (double.IsNaN(refinedTime)) // no min found within normal time limit to analyze
                    {
                        Log("No minimum found within normal time limit to analyze!");
                        return double.NaN;
                    }

                    double refinedError = AzimuthError(refinedTime);

                    if (refinedError < epsilon) // global minimum found
                    {
                        return refinedTime;
                    }
                    else // global min not found yet
                    {
                        candidateTime = refinedTime + 3600d * dayScale; // offset

                        if (refinedError < smallestError)
                        {
                            smallestError = refinedError;
                            bestTime = refinedTime;
                        }

                        if (candidateTime > startTime + maxTimeLimit) // no global min found within extended time limit
                        {
                            if (altBehaviorNaN)
                            {
                                Log($"No time found with error of 0 within time limit, returning NaN.");
                                return double.NaN;
                            }
                            else
                            {
                                Log($"No time found with error of 0 within time limit, returning time {bestTime} with error closest to 0.");
                                return bestTime;
                            }
                        } 
                    }
                }
            }

            double t0 = startTime;
            double t1 = startTime + coarseStep;
            double e0 = AzimuthError(t0);
            double e1 = AzimuthError(t1);

            if (e0 < e1) // either increasing slope, or t0 and t1 are on opposite sides of a min (and t0 has a lower error)
            {
                double refinedTime = GoldenSectionSearch(t0, t1);

                if (refinedTime > t0 + tolerance) // t0 and t1 are on opposite sides of a min (and t0 has a lower error)
                {
                    Log($"launchTime found at {refinedTime}");
                    return refinedTime;
                } // else, t0 (startTime) is the 'local' min, so this is an increasing slope

                //Log("Increasing Slope");

                while (e0 <= e1) // increasing slope (we just passed a min)
                {
                    t0 = t1;
                    t1 += coarseStep;
                    e0 = e1;
                    e1 = AzimuthError(t1);
                    if (t0 >= startTime + maxTimeLimit) // no min found within time limit
                    {
                        Log("No minimum found within time limit (increasing slope)");
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

            //Log("Decreasing Slope");

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

                //Log($"t0: {t0}, e0: {e0}, t1: {t1}, e1: {e1}");

                if (t0 >= startTime + maxTimeLimit) // no min found within time limit
                {
                    Log("No minimum found within time limit! (decreasing slope)");
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

            double fineT0 = Math.Max(startTime, tBest - coarseStep); // we need to back a step, in case we skipped over the min
            double fineT1 = Math.Min(startTime + maxTimeLimit, tBest + coarseStep);

            double finalResult = GoldenSectionSearch(fineT0, fineT1);

            Log($"launchTime found at {finalResult}");

            return finalResult;
        }

        (double phasingTime, double phasingAngle) EstimateTimeBeforeManeuver(Vector3d launchPos, double startTime)
        {
            //CelestialBody mainBody = target.referenceBody;
            double gravParameter = mainBody.gravParameter;
            double orbitRadius = mainBody.Radius + parkingAltitude * 1000d;

            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = currentUT + flightTime * solarDayLength + startTime;
            Vector3d targetPos = targetOrbit.getPositionAtUT(targetTime);

            Vector3d upVector = QuaternionD.AngleAxis(startTime * 360d / mainBody.rotationPeriod, MainAxis) * ((launchPos - MainPos).normalized * orbitRadius).normalized; // use rotationPeriod for sidereal time

            // Maneuver takes place at the point of the orbit that is opposite to the future position of the target
            Vector3d maneuverUpVector = (MainPos - targetPos).normalized;

            // KSP lies about having QuaternionD.FromToRotation, EulerAngles, and Euler (https://github.com/KSPModdingLibs/KSPCommunityFixes/issues/316)
            Vector3d rotationAxis = Vector3d.Cross(upVector, maneuverUpVector).normalized;
            double dot = Vector3d.Dot(upVector.normalized, maneuverUpVector.normalized);
            double phasingAngle = Math.Acos(Util.Clamp(dot, -1d, 1d)) * radToDeg;

            if (Vector3d.Dot(rotationAxis, MainAxis) < 0)
            { // rotationAxis is pointing roughly opposite to mainBody rotation axis, therefore rotation is retrograde
                phasingAngle = 360d - phasingAngle;
            }

            // Convert angle to time in orbit
            double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / gravParameter);
            //double offset = 180; // for some reason it just constantly underestimates by ~3 minutes, no idea why
            //phasingAngle = (phasingAngle + (offset / orbitPeriod) * 360d) % 360d;
            double phasingTime = phasingAngle / 360d * orbitPeriod;

            return (phasingTime, phasingAngle);
        }

        private (double time, double eccentricity) EstimateTimeAfterManeuver(double dV)
        {
            // The formulas are from http://www.braeunig.us/space/orbmech.htm

            double gravParameter = mainBody.gravParameter;

            // Radius of the circular orbit, including the radius of the mainBody
            double r0 = mainBody.Radius + parkingAltitude * 1000d;

            // Initial guess for the altitude of the target
            double r1 = targetOrbit.GetRadiusAtUT(currentUT);

            //Log($"initial r1: {r1}");

            double previousT1 = 0d;
            double t1 = double.MaxValue;
            double e = 0d;

            // The target is moving, so we need to know its altitude when the vessel arrives
            // For that we need to know the flight time, so we can iterate the flight time until the error is small enough

            while (Math.Abs(t1 - previousT1) >= deltaVTolerance)
            {
                previousT1 = t1;
                double v0 = Math.Sqrt(gravParameter / r0) + dV;
                e = r0 * v0 * v0 / gravParameter - 1;

                // e == 1 would mean that the orbit is parabolic. No idea which formulas are applicable in this case.
                // But it's so unlikely that I will just cheat and make such orbits slightly hyperbolic.
                if (e == 1d)
                {
                    v0 += 0.1;
                    e = r0 * v0 * v0 / gravParameter - 1;
                }

                // Semi-major axis after the maneuver
                double a = 1d / (2d / r0 - v0 * v0 / gravParameter);

                // True anomaly when the vessel reaches the altitude of the target (r1)
                double trueAnomaly1 = Math.Acos((a * (1d - e * e) - r1) / (e * r1));

                // Elliptic orbit after the maneuver
                if (e < 1d)
                {
                    // Eccentric Anomaly when the vessel reaches the altitude of the target
                    double eccAnomaly1 = Math.Acos((e + Math.Cos(trueAnomaly1)) / (1d + e * Math.Cos(trueAnomaly1)));
                    double meanAnomaly1 = eccAnomaly1 - e * Math.Sin(eccAnomaly1);
                    t1 = meanAnomaly1 / Math.Sqrt(gravParameter / Math.Pow(a, 3));
                }
                else // Hyperbolic orbit, Parabolic orbit (e == 1) should have been prevented earlier
                {
                    // Hyperbolic Eccentric Anomaly when the vessel reaches the altitude of the target
                    // Can't use Math.Acosh, it does not seem to work in .NET 4
                    double hEccAnomaly1 = Util.Acosh((e + Math.Cos(trueAnomaly1)) / (1d + e * Math.Cos(trueAnomaly1)));

                    t1 = Math.Sqrt(Math.Pow(-a, 3) / gravParameter) * (e * Math.Sinh(hEccAnomaly1) - hEccAnomaly1);
                }

                // Update r1 using new estimate of flight time
                r1 = targetOrbit.GetRadiusAtUT(currentUT + t1);
                if (double.IsNaN(r1)) return (double.NaN, double.NaN); // Target is unreachable from this deltaV

                //Log($"looped r1: {r1}, t1: {t1}");
            }

            //Log($"final r1: {r1}, t1: {t1}");

            return (t1, e);
        }

        (double dV, double eccentricity) EstimateDV()
        {
            //CelestialBody mainBody = target.referenceBody;

            // Search in this range
            //double minPossibleDV = 2500 * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius), gives 766 for Kerbin
            const double minPossibleDV = 1d; // no need to have an actual min for dV, especially for vessels
            double maxPossibleDV = maxDeltaVScaled * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius), gives 1840 for Kerbin
            double expectedFlightTime = flightTime * solarDayLength;
            const double epsilon = 1e-9;
            //Log($"Starting up, expectedFlightTime: {expectedFlightTime}");

            // Current search range, will be gradually narrowed
            double lowerBound = minPossibleDV;
            double upperBound = maxPossibleDV;

            double FlightTimeError(double candidateDV)
            {
                (double estimatedFlightTime, _) = EstimateTimeAfterManeuver(candidateDV);

                //Log($"estimatedFlightTime: {estimatedFlightTime}, candidateDV: {candidateDV}");

                if (double.IsNaN(estimatedFlightTime))
                    return double.MaxValue; // invalidate bad guess

                return Math.Abs(estimatedFlightTime - expectedFlightTime);
            }

            //Log($"starting run");

            // golden section search, adapted from https://en.wikipedia.org/wiki/Golden-section_search

            double left = upperBound - invphi * (upperBound - lowerBound);
            double right = lowerBound + invphi * (upperBound - lowerBound);

            double eLeft = FlightTimeError(left);
            double eRight = FlightTimeError(right);

            while (Math.Abs(upperBound - lowerBound) > deltaVTolerance)
            {
                if (eLeft < eRight)
                {
                    upperBound = right;
                    right = left;
                    eRight = eLeft;
                    left = upperBound - invphi * (upperBound - lowerBound);
                    eLeft = FlightTimeError(left);
                }
                else
                {
                    lowerBound = left;
                    left = right;
                    eLeft = eRight;
                    right = lowerBound + invphi * (upperBound - lowerBound);
                    eRight = FlightTimeError(right);
                }
            }

            //Log($"Done with calculations");

            double dV = (lowerBound + upperBound) / 2;
            (_, double eccentricity) = EstimateTimeAfterManeuver(dV);

            if (Math.Abs(dV - minPossibleDV) <= epsilon * Math.Abs(minPossibleDV) || Math.Abs(dV - maxPossibleDV) <= epsilon * Math.Abs(maxPossibleDV))
            {
                Log($"dV is incorrect, the correct value is outside the initial search range");
                dV = double.NaN;
                eccentricity = double.NaN;
            }

            //Log($"Final dV: {dV}, eccentricity: {eccentricity}");

            return (dV, eccentricity);
        }

        #endregion
            #region Caching Methods

            // if the factor that should trigger a cache reset can only be changed by the user in one location, then its fine to use StateChanged and then clear the cache
            // StateChanged checks if something is exactly equal to the previous value, which is fine for a value that only the user can change (especially as the user will want the value to update regardless of how small their changes are, which would make a tolerance not make sense)
            // if the factor can be changed by the game itself, or can be changed by the user in multiple ways (in the case of the target), then put the value in the cache and check in the method if its crossed the tolerance
            // (the reset in special warp is a unique case and should be kept as such)

        private void CheckWindowCache(double latitude, double longitude, double inclination, double targetAltitude)
        {
            const double tolerance = 0.01;

            // remove expired or mismatched entries
            for (int i = 0; i <= windowCache.Count - 1; i++)
            {
                var entry = windowCache[i];
                bool expired = currentUT > entry.absoluteLaunchTime;
                bool targetMismatch = entry.target != target; // this will also trigger when changing mainBody, assuming we dont get restarted due to a scene switch
                //Log($"oldTarget: {entry.target}, newTarget: {target}");
                bool posMismatch = Math.Abs(entry.latitude - latitude) >= tolerance || Math.Abs(entry.longitude - longitude) >= tolerance; // add altitude if necessary, also, we restart when changing launch sites, so posMismatch only triggers when changing position by vessel or manually
                bool altitudeMismatch = Math.Abs(entry.targetAltitude - targetAltitude) >= tolerance * 100d * 1000d; // kilometer
                bool inclinationMismatch = Math.Abs(entry.inclination - inclination) >= tolerance * 2;

                if (expired || targetMismatch || posMismatch || inclinationMismatch || altitudeMismatch)
                {
                    //Log($"Resetting Window Cache due to change of Cached Launch Window {i + 1}, old values: target:{entry.target}, latitude: {entry.latitude}, longitude: {entry.longitude}, inclination: {entry.inclination:F3}, time: {entry.absoluteLaunchTime:F3} due to {(expired ? "time expiration " : "")}{(targetMismatch ? "target mismatch " : "")}{(posMismatch ? "position mismatch " : "")}{(inclinationMismatch ? "inclination mismatch" : "")}");
                    if (targetMismatch) Log($"Now targeting {target}"); // this will only trigger if the mainBody actually has targets(s)
                    windowCache.Clear(); // dont use windowCache.RemoveAt(i), it leads to compounding errors with the other remaining launch times
                    launchOrbitCache.Clear();
                    phasingCache.Clear();
                    break;
                }
            }

        }

        private double GetCachedLaunchTime(Vector3d launchPos, double latitude, double longitude, double inclination, bool useAltBehavior, int windowNumber)
        {
            // bad caches already removed by CheckWindowCache

            double offset = 3600d * dayScale; // 1 hour offset between windows, scale based on EarthSiderealDay

            // sort windowCache in ascending order of launch time
            windowCache.Sort((a, b) => a.absoluteLaunchTime.CompareTo(b.absoluteLaunchTime));

            // if window exists, return it
            if (windowNumber <= windowCache.Count - 1 && windowNumber >= 0)
            {
                return windowCache[windowNumber].absoluteLaunchTime;
            }

            double startTime;

            if (windowCache.Count == 0)
            {
                startTime = 0;
            }
            else
            {
                // start after the last cached window + offset
                startTime = windowCache.Last().absoluteLaunchTime - currentUT + offset;
            }

            // compute windows from windowCache.Count up to windowNumber
            double newLaunchTime = 0d;
            double absoluteLaunchTime = 0d;

            for (int w = windowCache.Count; w <= windowNumber; w++)
            {
                newLaunchTime = EstimateLaunchTime(launchPos, startTime, useAltBehavior);

                if (double.IsNaN(newLaunchTime))
                {
                    //Log("LaunchTime is NaN, exiting"); // keep this log inside EstimateLaunchTime
                    return double.NaN;
                }

                if (newLaunchTime < 60d * dayScale && PrincipiaInstalled)
                { // perturbations make a new window that is way too close, so just skip to the next one
                    Log("New window is too close, skipping to the next one.");
                    windowNumber++;
                }
                else
                {
                    absoluteLaunchTime = currentUT + newLaunchTime;
                    windowCache.Add((target, targetAltitude, latitude, longitude, inclination, absoluteLaunchTime));
                }

                startTime = newLaunchTime + offset;

                if (windowCache.Count > maxWindows)
                {
                    Log("windowCache.Count has grown greater than maxWindows!");
                    return double.NaN;
                }
            }

            //Log($"windowCache count: {windowCache.Count}");

            return absoluteLaunchTime;
        }

        private OrbitData GetCachedLaunchOrbit(Vector3d launchPos, double startTime, int? windowNumber = null)
        {
            if (windowNumber.HasValue)
            {
                int index = launchOrbitCache.FindIndex(item => item.windowNumber == windowNumber.Value);
                if (index != -1) return launchOrbitCache[index].launchOrbit; // return if exists
                else
                {
                    OrbitData launchOrbit = CalcOrbitForTime(launchPos, startTime);
                    launchOrbitCache.Add((launchOrbit, windowNumber.Value));
                    return launchOrbit;
                }
            }
            else
            {
                return CalcOrbitForTime(launchPos, startTime);
            }
        }

        private (double phasingTime, double phasingAngle) GetCachedPhasingTime(Vector3d launchPos, double startTime, int? windowNumber = null)
        {
            if (windowNumber.HasValue)
            {
                int index = phasingCache.FindIndex(item => item.windowNumber == windowNumber.Value);
                if (index != -1) return (phasingCache[index].phasingTime, phasingCache[index].phasingAngle); // return if exists
                else
                {
                    (double phasingTime, double phasingAngle) = EstimateTimeBeforeManeuver(launchPos, startTime);
                    phasingCache.Add((phasingTime, phasingAngle, windowNumber.Value));
                    return (phasingTime, phasingAngle);
                }
            }
            else
            {
                return EstimateTimeBeforeManeuver(launchPos, startTime);
            }
        }

        private (double dV, double eccentricity) GetCachedDeltaV()
        {
            const double tolerance = 0.01;

            if (!deltaVCache.HasValue)
            {
                (double dV, double eccentricity) = EstimateDV();
                deltaVCache = (target, targetAltitude, dV, eccentricity);
            }
            else
            {
                bool targetMismatch = deltaVCache.Value.target != target;
                bool altitudeMismatch = Math.Abs(deltaVCache.Value.targetAltitude - targetAltitude) >= tolerance * 100d * 1000d; // kilometer

                if (targetMismatch || altitudeMismatch)
                {
                    //Log($"Resetting DeltaV Cache due to change of target. Old values: target: {deltaVCache.Value.target}, deltaV: {deltaVCache.Value.deltaV}");
                    (double dV, double eccentricity) = EstimateDV();
                    deltaVCache = (target, targetAltitude, dV, eccentricity);
                }
            }

            return (deltaVCache.Value.deltaV, deltaVCache.Value.eccentricity);
        }

        #endregion
        #region GUI Methods

        private void MakeNumberEditField<T>(string controlId, ref T value, IConvertible step, IConvertible minValue, IConvertible maxValue, bool wrapAround = false, string minusTooltip = "", string plusTooltip = "") where T : struct, IConvertible
        {
            const double epsilon = 1e-9;

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
                if (Math.Abs(parsedBufferValue - valueDouble) > epsilon)
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

            string newLabel = GUILayout.TextField(textValue, GUILayout.Width(Mathf.Clamp(GUI.skin.textField.CalcSize(new GUIContent(textValue)).x + 10, 60, windowWidth - (40 * 2)))); // change width based on text length

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
                        if (Math.Abs(valueDouble - snappedValue) > epsilon)
                            valueDouble = Math.Max(minValueDouble, snappedValue); // snap to next number
                        else
                            if (valueDouble - stepDouble < minValueDouble && wrapAround)
                            valueDouble = maxValueDouble;
                        else valueDouble = Math.Max(minValueDouble, valueDouble - stepDouble); // then decrement
                    }
                    else
                    {
                        double snappedValue = Math.Ceiling(valueDouble / stepDouble) * stepDouble;
                        if (Math.Abs(valueDouble - snappedValue) > epsilon)
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

        private string FormatTime(double t)
        {
            // TODO, some bodies have really dumb solarDayLengths, so switch to using the solarDayLength of the home body?
            int days = (int)Math.Floor(t / Math.Round(solarDayLength)); // round to avoid stuff like 3d 24h 0m 0s
            t -= days * Math.Round(solarDayLength);
            int hours = (int)Math.Floor(t / (60d * 60d));
            t -= hours * 60d * 60d;
            int minutes = (int)Math.Floor(t / 60d);
            t -= minutes * 60d;
            if (days > 0d)
                return $"{days}d {hours}h {minutes}m {FormatDecimals(t)}s";
            else if (hours > 0d)
                return $"{hours}h {minutes}m {FormatDecimals(t)}s";
            else if (minutes > 0d)
                return $"{minutes}m {FormatDecimals(t)}s";
            return $"{FormatDecimals(t)}s";
        }

        private string FormatDecimals(double value, int extra = 0)
        {
            if (StateChanged("decimals", decimals))
            {
                ResetWindow(ref mainRect);
                ResetWindow(ref settingsRect);
            }
            return $"{value.ToString($"F{Math.Max(0, decimals + extra)}")}";
        }

        private bool StateChanged<T>(string key, T state)
        {
            if (stateBuffer.TryGetValue(key, out var existing))
            {
                if (!EqualityComparer<T>.Default.Equals((T)existing, state))
                {
                    stateBuffer[key] = state;
                    return true;
                }
                return false;
            }
            else
            {
                stateBuffer[key] = state;
                return true; // first access
            }
        }

        private bool StateChanged<T>(string key, ref T state, T newState)
        {
            state = newState;
            if (StateChanged(key, state)) return true;
            else return false;
        }

        private void ButtonPressed(ref bool state, bool pressed, bool reset = true)
        { // this doesn't need to be done for all buttons, just the ones that change the size of the window
            if (pressed)
            {
                state = !state;
                if (reset) ResetWindow(ref mainRect); // if needed in settings window, change this to be parameter
            }
        }

        private void ResetWindow(ref Rect rect)
        { // Doing this forces the window to be resized. Without it, the window will become bigger when controls expand, but never become smaller again
            rect = new Rect(rect.xMin, rect.yMin, -1f, -1f);
        }

        private void ResetDefault(ref double value, double defaultValue)
        {
            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical();
            GUILayout.Space(5);
            if (GUILayout.Button(new GUIContent(" R", "Reset to Default"), GUILayout.Width(20))) value = defaultValue;
            GUILayout.EndVertical();
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
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

        private void ShowOrbitInfo(ref bool showPhasing, double phaseTime, double phaseAngle)
        {
            if (showPhasing)
            {
                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Phasing angle", "Phasing angle between launch location and maneuver in orbit"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button(new GUIContent("T", "Switch to phasing time"), GUILayout.Width(20));
                ButtonPressed(ref showPhasing, showPhasing_pressed);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{FormatDecimals(phaseAngle)}\u00B0", $"{FormatDecimals(phaseAngle * degToRad)} rads"), GUILayout.MinWidth(60)); // TODO, for some reason the tooltip doesn't work for the next launch window
            }
            else
            {
                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Time in orbit", "Phasing time spent waiting in parking orbit before maneuver"), GUILayout.ExpandWidth(true));
                bool showPhasing_pressed = GUILayout.Button(new GUIContent(" P", "Switch to phasing angle"), GUILayout.Width(20));
                ButtonPressed(ref showPhasing, showPhasing_pressed);
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent(FormatTime(phaseTime), $"{phaseTime:0}s"), GUILayout.MinWidth(100));
            }
        }

        private void ShowSettings()
        {
            bool showSettings_pressed;
            if (gearBlack != null && gearGreen != null)
            {
                showSettings_pressed = GUILayout.Button(new GUIContent(useAltSkin ? gearBlack : gearGreen, "Show Settings"), new GUIStyle(GUI.skin.button) { padding = new RectOffset(0, 0, 0, 0) }, GUILayout.Width(20), GUILayout.Height(20));
                // remove padding in style to prevent image getting scaled down with unity skin
            }
            else
            {
                showSettings_pressed = GUILayout.Button(new GUIContent("S", "Show Settings (Error: A settings icon is missing!)"), GUILayout.Width(20), GUILayout.Height(20));
            }
            ButtonPressed(ref showSettings, showSettings_pressed, false);
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

        private string TextOverspill(string text, int width, GUIStyle style)
        {
            Vector2 size = style.CalcSize(new GUIContent(text));
            if (size.x > width)
            {
                int maxChars = text.Length - 1;

                while (maxChars > 0)
                {
                    string truncatedText = text.Substring(0, maxChars) + "...";
                    size = style.CalcSize(new GUIContent(truncatedText));

                    if (size.x <= width)
                    {
                        return truncatedText;
                    }

                    maxChars--;
                }
                return "...";
            }
            else return text;
        }

        private void ExpandCollapse(ref bool button, string tooltip = "", bool overrideButton = false)
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5); // push down 5

            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace(); // push button to the right
            GUI.enabled = !overrideButton;
            bool button_pressed = GUILayout.Button(new GUIContent(!button ? "+" : "\u2013", tooltip + (overrideButton ? "\nThis button is currently disabled" : "")), GUILayout.Width(30)); // en dash shows up as the same width as + ingame, while the minus symbol is way thinner
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            ButtonPressed(ref button, button_pressed);
        }

        #endregion
        #region MakeMainWindow

        private void MakeMainWindow(int id)
        {
            windowWidth = 160;

            if (FlightGlobals.currentMainBody != null)
                mainBody = FlightGlobals.currentMainBody; // spacecenter/flight/mapview
            else if (HighLogic.LoadedSceneHasPlanetarium && MapView.MapCamera?.target?.celestialBody != null) // if we dont check that its in map view, then the vab/sph body will get overwritten
                mainBody = MapView.MapCamera.target.celestialBody; // tracking station, could work for flight map view too
            else if (FlightGlobals.GetHomeBody() != null)
                mainBody = FlightGlobals.GetHomeBody(); // vab/sph, this always gives the home body (could also do Planetarium.fetch.Home)
            else LogError("CRITICAL ERROR: No main body found!");

            //CelestialBody target = FlightGlobals.fetch.bodies.FirstOrDefault(body => body.name.Equals("Moon", StringComparison.OrdinalIgnoreCase));
            moons = mainBody?.orbitingBodies?.OrderBy(body => body.bodyName).ToList();
            vessels = FlightGlobals.Vessels?.Where(vessel => vessel != null && mainBody != null && vessel.mainBody == mainBody && vessel.situation == Vessel.Situations.ORBITING).OrderBy(vessel => vessel.vesselName).ToList();
            // TODO, instead of ordering alphabetically, order by closest periapsis? make this a setting

            GUILayout.Space(5);

            moonsInvalid = moons == null || moons.Count == 0;
            vesselsInvalid = vessels == null || vessels.Count == 0;

            if (mainBody == null)
            {
                GUILayout.Label("CRITICAL ERROR: No main body found!", GUILayout.Width(windowWidth)); // this is really bad
                if (StateChanged("errorState", ref errorState, 4))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }
                ShowSettings();
            }
            else if (moonsInvalid && vesselsInvalid)
            {
                GUILayout.Label("ERROR: There are no moons or vessels orbiting this planet!", GUILayout.Width(windowWidth));
                if (StateChanged("errorState", ref errorState, 1))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }
                ShowSettings();
            }
            else if (targetVessel && vesselsInvalid)
            {
                GUILayout.Label("ERROR: There are no vessels orbiting this planet!", GUILayout.Width(windowWidth));
                if (StateChanged("errorState", ref errorState, 2))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }
                ShowSettings();
                GUILayout.Label("If you want to get out of this error, open settings and toggle the \"<i>Target an orbiting Vessel instead of an orbiting Moon</i>\" button.");
                // do not switch automatically, this would change the user's settings silently, and they may not want to switch
            }
            else if (!targetVessel && moonsInvalid)
            {
                GUILayout.Box("ERROR: There are no moons orbiting this planet!", GUILayout.Width(windowWidth));
                if (StateChanged("errorState", ref errorState, 3))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }
                ShowSettings();
                GUILayout.Label("If you want to get out of this error, open settings and toggle the \"<i>Target an orbiting Vessel instead of an orbiting Moon</i>\" button.");
                // do not switch automatically, this would change the user's settings silently, and they may not want to switch
            }
            else
            {
                if (StateChanged("errorState", ref errorState, 0))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }
                int count;
                if (targetVessel)
                {
                    if (target == null || !vessels.Contains(target)) target = vessels[0] as Vessel;
                    count = vessels.Count;
                    //Log($"vessels: {vessels}, count: {count}, target: {target}");
                }
                else
                {
                    if (target == null || !moons.Contains(target)) target = moons[0] as CelestialBody;
                    count = moons.Count;
                    //Log($"vessels: {moons}, count: {count}, target: {target}");
                }

                if (target is Vessel vessel)
                {
                    targetOrbit = vessel?.orbit;
                    targetName = vessel?.vesselName;
                }
                else if (target is CelestialBody body)
                {
                    targetOrbit = body?.orbit;
                    targetName = body?.bodyName;;
                }
                else LogError("Unknown target type: " + target.GetType().Name);

                currentUT = Planetarium.GetUniversalTime();
                //if (target == null || !moons.Contains(target)) target = moons[0];
                inclination = GetTargetInclination(targetOrbit);
                isLowLatitude = Math.Abs(latitude) <= inclination;
                (double dV, double eccentricity) = GetCachedDeltaV();
                dayScale = mainBody.rotationPeriod / EarthSiderealDay;
                CelestialBody homeBody = FlightGlobals.GetHomeBody();
                solarDayLength = useHomeSolarDay ? homeBody.solarDayLength : mainBody.solarDayLength;
                targetAltitude = targetOrbit.GetRadiusAtUT(currentUT);

                if (requireSurfaceVessel) inVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null && (FlightGlobals.ActiveVessel.Landed || FlightGlobals.ActiveVessel.Splashed);
                else inVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null; // this needs to be set here, as settings window isnt always open

                if (count > 1) // only display target selector screen if theres multiple targets
                {
                    if (currentBody > count - 1) currentBody = count - 1; // this can happen when switching mainBody or when a vessel is destroyed

                    GUILayout.BeginHorizontal();
                    GUILayout.Box(new GUIContent(TextOverspill(targetName, 80, GUI.skin.box), targetName), GUILayout.MinWidth(80));

                    if (GUILayout.Button("<", GUILayout.MinWidth(20)))
                    {
                        currentBody--;
                        if (currentBody < 0) currentBody = count - 1;
                    }

                    if (GUILayout.Button(">", GUILayout.MinWidth(20)))
                    {
                        currentBody++;
                        if (currentBody > count - 1) currentBody = 0;
                    }
                    GUILayout.EndHorizontal();
                    if (targetVessel) target = vessels[currentBody];
                    else target = moons[currentBody]; // cant do ternary for this
                    GUILayout.Space(5);
                }

                if (StateChanged("displayTargetSelector", count > 1))
                {
                    ResetWindow(ref mainRect);
                }

                GUILayout.BeginHorizontal();
                if (mainBody != homeBody && (!useVesselPosition || !inVessel) && !expandLatLong) GUILayout.Label(new GUIContent("<b>(!)</b>", $"Using latitude/longitude of the Space Center on a body that is not {homeBody.bodyName}!"));

                GUILayout.Label(new GUIContent($"Latitude: <b>{FormatDecimals(latitude)}\u00B0</b>", $"Latitude of launch location\nUsing Vessel Position: {(useVesselPosition && inVessel ? "True" : "False")}"), GUILayout.ExpandWidth(true));
                
                ExpandCollapse(ref expandLatLong, "Set manual latitude and longitude");
                GUILayout.EndHorizontal();

                if (expandLatLong)
                {
                    GUILayout.Space(6); // weird spacing
                    MakeNumberEditField("latitude", ref latitude, 1d, -90d, 90d, true);
                    GUILayout.Space(5);
                    GUILayout.Label(new GUIContent($"Longitude: <b>{FormatDecimals(longitude)}\u00B0</b>", $"Longitude of launch location,\nUsing Vessel Position: {(useVesselPosition && inVessel ? "True" : "False")}"));
                    MakeNumberEditField("longitude", ref longitude, 1d, -180d, 180d, true);
                    //GUILayout.Space(5);
                    //GUILayout.Label(new GUIContent($"Altitude: <b>{Decimals(altitude)}</b>", "Altitude of launch location (in meters)"));
                    //MakeNumberEditField("altitude", ref altitude, 100d, -mainBody.Radius + 5d, targetOrbit.PeA - 5d); // this is a laughably large range, but its the only way to make sure it can cover altitudes below and above sea level
                    launchPos = mainBody.GetWorldSurfacePosition(latitude, longitude, 0d);
                }
                else launchPos = GetLaunchPos(mainBody, ref latitude, ref longitude, useVesselPosition);

                //TODO, add button to add waypoint at launchPos? kept getting a NRE but perhaps im doing it wrong

                GUILayout.Space(5);
                GUILayout.Label(new GUIContent("Flight Time (days)", $"Coast duration to {targetName} after the maneuver (in {(useHomeSolarDay ? homeBody.bodyName : mainBody.bodyName)} solar days)"), GUILayout.ExpandWidth(true), GUILayout.Width(windowWidth)); // this sets the width of the window
                MakeNumberEditField("flightTime", ref flightTime, 0.1d, 0.1d, double.MaxValue);
                if (StateChanged("flightTime", flightTime))
                {
                    windowCache.Clear();
                    launchOrbitCache.Clear();
                    phasingCache.Clear();
                    deltaVCache = null;
                }
                double l = Math.Round(flightTime * solarDayLength);
                GUILayout.Box(new GUIContent(FormatTime(l), $"{l:0}s"), GUILayout.MinWidth(100)); // tooltips in Box have a problem with width, use {0:0}
                
                CheckWindowCache(latitude, longitude, inclination, targetAltitude);

                OrbitData launchOrbit = GetCachedLaunchOrbit(launchPos, referenceTime, referenceWindowNumber);
                //Log("CLAYELADDEDDLOGS FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW");
                //var stopwatch = Stopwatch.StartNew();
                double nextLaunchUT = GetCachedLaunchTime(launchPos, latitude, longitude, inclination, useAltBehavior, 0);
                double nextLaunchETA = nextLaunchUT - currentUT;
                //stopwatch.Stop();
                //Log($"Window 1 Launch Time: {firstLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");
                //Log("CLAYELADDEDDLOGS SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW");
                //stopwatch = Stopwatch.StartNew();
                double extraLaunchUT = GetCachedLaunchTime(launchPos, latitude, longitude, inclination, useAltBehavior, extraWindowNumber - 1);
                double extraLaunchETA = extraLaunchUT - currentUT;
                //stopwatch.Stop();
                //Log($"Window 2 Launch Time: {secondLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");

                bool inSpecialWarp = warpState == 2 || warpState == 3;
                bool specialWarpActive = warpState == 1 || inSpecialWarp;
                if (nextLaunchETA >= mainBody.rotationPeriod && PrincipiaInstalled && specialWarpSelected && !inSpecialWarp) warpState = 1;
                else if (!inSpecialWarp && !specialWarpWait) warpState = 0;

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Required \u0394V", "Required change in velocity for the maneuver in parking orbit"), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandAltitude, "Set parking orbit altitude");
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent($"{FormatDecimals(dV)} m/s", $"Eccentricity of {(eccentricity > 1 ? "hyperbolic" : "elliptical")} trajectory: {FormatDecimals(eccentricity)}"), GUILayout.MinWidth(100));

                if (expandAltitude)
                {
                    GUILayout.Label(new GUIContent("Parking Orbit (km)", "Planned altitude of the circular parking orbit before the maneuver"), GUILayout.ExpandWidth(true));
                    MakeNumberEditField("parkingAltitude", ref parkingAltitude, 5d, mainBody.atmosphere ? mainBody.atmosphereDepth / 1000d : 0d, targetOrbit.PeA / 1000d - 5d); // PeA updates every frame so we don't need to ask Principia
                    if (StateChanged("parkingAltitude", parkingAltitude))
                    {
                        phasingCache.Clear();
                        deltaVCache = null;
                    }
                }

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                if (showAzimuth)
                {
                    GUILayout.Label(new GUIContent(launchLabel + (showAzimuth ? "Az." : "Incl."), "Launch to this azimuth to get into the target parking orbit"), GUILayout.ExpandWidth(true));
                }
                else
                {
                    GUILayout.Label(new GUIContent(launchLabel + (showAzimuth ? "Az." : "Incl."), "Launch to this inclination to get into the target parking orbit (positive = North, negative = South, regardless of latitude sign)"), GUILayout.ExpandWidth(true));
                }
                ExpandCollapse(ref expandParking0);
                GUILayout.EndHorizontal();

                switch (referenceTimeButton)
                {
                    case 0:
                        referenceTimeLabel = "Launch Now";
                        referenceTimeTooltip = "Change to Next Launch Window";
                        launchLabel = "Launch Now ";
                        referenceTime = 0d;
                        referenceWindowNumber = null;
                        break;
                    case 1:
                        referenceTimeLabel = "Next Window";
                        referenceTimeTooltip = $"Change to Launch Window {extraWindowNumber}";
                        launchLabel = "Next Window ";
                        referenceTime = nextLaunchETA;
                        referenceWindowNumber = 0;
                        break;
                    case 2:
                        referenceTimeLabel = $"Window {extraWindowNumber}";
                        referenceTimeTooltip = "Change to Current Launch Window";
                        launchLabel = $"Window {extraWindowNumber} ";
                        referenceTime = extraLaunchETA;
                        referenceWindowNumber = extraWindowNumber - 1;
                        break;
                }

                GUILayout.Space(5);
                if (showAzimuth)
                {
                    GUILayout.Box(new GUIContent($"{FormatDecimals(launchOrbit.azimuth)}\u00B0", $"{FormatDecimals(launchOrbit.azimuth * degToRad)} rads, this is {(launchOrbit.azimuth < 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                }
                else
                {
                    double launchInclination = launchOrbit.azimuth > 90d && launchOrbit.azimuth < 270d ? -launchOrbit.inclination : launchOrbit.inclination;
                    GUILayout.Box(new GUIContent($"{FormatDecimals(launchInclination)}\u00B0", $"{FormatDecimals(launchInclination * degToRad)} rads, this is {(launchOrbit.azimuth < 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                }

                if (expandParking0)
                {
                    (double phaseTime0, double phaseAngle0) = GetCachedPhasingTime(launchPos, referenceTime, referenceWindowNumber);
                    ShowOrbitInfo(ref showPhasing, phaseTime0, phaseAngle0);
                }

                string windowTooltip = !isLowLatitude && Math.Abs(targetLaunchAzimuth - 90d) < 0.01 ?
                    "Launch Easterly at this time to get into the target parking orbit" :
                    "Launch at this time at this inclination to get into the target parking orbit";

                GUILayout.Space(5);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Next Window", windowTooltip), GUILayout.ExpandWidth(true));
                ExpandCollapse(ref expandParking1);
                GUILayout.EndHorizontal();

                GUILayout.Space(5);
                GUILayout.Box(new GUIContent(FormatTime(nextLaunchETA), $"UT: {nextLaunchUT:0}s"), GUILayout.MinWidth(100)); // the tooltip will flash every second if we just do {nextLaunchETA}, we need absolute time

                if (expandParking1)
                {
                    (double phaseTime1, double phaseAngle1) = GetCachedPhasingTime(launchPos, nextLaunchETA, 0);
                    ShowOrbitInfo(ref showPhasing, phaseTime1, phaseAngle1);
                }

                if (expandExtraWindow)
                {
                    GUILayout.Space(5);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent($"Window {extraWindowNumber}", "Extra Window: " + windowTooltip), GUILayout.ExpandWidth(true));
                    ExpandCollapse(ref expandParking2);
                    GUILayout.EndHorizontal();

                    GUILayout.Space(5);
                    GUILayout.Box(new GUIContent(FormatTime(extraLaunchETA), $"UT: {extraLaunchUT:0}s"), GUILayout.MinWidth(100)); // the tooltip will flash every second if we just do {extraLaunchETA}, we need absolute time

                    if (expandParking2)
                    {
                        (double phaseTime2, double phaseAngle2) = GetCachedPhasingTime(launchPos, extraLaunchETA, extraWindowNumber - 1);
                        ShowOrbitInfo(ref showPhasing, phaseTime2, phaseAngle2);
                    }
                }

                GUILayout.Space(5);
                GUILayout.Label(new GUIContent("Warp Margin (sec)", "The time difference from the launch window that the warp will stop at"), GUILayout.ExpandWidth(true));
                MakeNumberEditField("warpMargin", ref warpMargin, 5d, 0d, double.MaxValue);

                bool addAlarm;
                bool toggleWarp;
                GUILayout.Space(10);
                if (KACInstalled)
                {
                    GUILayout.BeginHorizontal();
                    addAlarm = GUILayout.Button(new GUIContent(" Add Alarm", "Add Alarm to Kerbal Alarm Clock"), GUILayout.MinWidth(80));

                    toggleWarp = GUILayout.Button(new GUIContent(TimeWarp.CurrentRate > 1d || inSpecialWarp ? "Stop Warp" : "Warp", "Warp to the Next Window, taking into account the Warp Margin"), GUILayout.MinWidth(80));
                    GUILayout.EndHorizontal();
                }
                else
                {
                    addAlarm = false;
                    toggleWarp = GUILayout.Button(new GUIContent(TimeWarp.CurrentRate > 1d || inSpecialWarp ? "Stop Warp" : "Warp", "Warp to the Next Window, taking into account the Warp Margin"), GUILayout.MinWidth(160));
                }

                //GUILayout.Space(10);
                //GUILayout.BeginHorizontal();
                //GUI.enabled = KACInstalled;
                //bool addAlarm = GUILayout.Button(new GUIContent(" Add Alarm", "Add Alarm to Kerbal Alarm Clock"), GUILayout.MinWidth(80));
                //GUI.enabled = true;

                //bool toggleWarp = GUILayout.Button(new GUIContent(TimeWarp.CurrentRate > 1d || inSpecialWarp ? "Stop Warp" : "Warp", "Warp to the Next Window, taking into account the Warp Margin"), GUILayout.MinWidth(80));
                //GUILayout.EndHorizontal();

                if (specialWarpActive)
                {
                    BeginCenter(false);
                    GUILayout.Label(new GUIContent("Special Warp", "See the settings for an in-depth explanation, this CANNOT be halted once started"));
                    EndCenter(false);
                }

                // the error "Getting control 2's position in a group with only 2 controls when doing repaint..." is thrown when changing the visibility of the above label for some reason (always with BeginCenter or EndCenter)
                // its harmless as far as I can tell, but i couldnt figure out a way to actually catch it

                if (StateChanged("specialWarpActive", specialWarpActive))
                {
                    ResetWindow(ref mainRect);
                }

                GUILayout.BeginHorizontal();
                GUI.enabled = InputLockManager.IsUnlocked(ControlTypes.TARGETING); // targeting will become "disabled" when the game isnt in focus or paused, but its not actually disabled. not sure how i can fix this, its just visual tho
                bool targetPressed = GUILayout.Button(new GUIContent(targetSet ? "Unset Target" : TextOverspill($"Target {targetName}", 140, GUI.skin.button), $"Target {targetName}{(InputLockManager.IsUnlocked(ControlTypes.TARGETING) ? "" : "\nTarget Switching is Locked")}"), GUILayout.MinWidth(140));
                GUI.enabled = true;
                ShowSettings();
                GUILayout.EndHorizontal();

                // TODO, add compatibility with normal alarm clock if KAC not installed

                if (addAlarm)
                {
                    if (!useAltAlarm && !double.IsNaN(nextLaunchETA))
                    {
                        string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, $"{targetName} Launch Window", nextLaunchUT - warpMargin); // remove warpMargin? might need to check for overspill?
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
                        string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, $"{targetName} Launch Window {extraWindowNumber}", extraLaunchUT - warpMargin); // remove warpMargin? might need to check for overspill?
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
                        if (warpState == 1)
                        {
                            Log("Special warp 1 in progress");
                            TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                            TimeWarp.fetch.WarpTo(nextLaunchUT - (warpMargin + mainBody.rotationPeriod)); // warp to within a day
                            warpState = 2;
                            specialWarpWait = true;
                        }
                        else
                        {
                            TimeWarp.fetch.WarpTo(nextLaunchUT - warpMargin);
                        }
                    }
                }

                if (specialWarpWait && !inWarp())
                {
                    waitingTime = currentUT;
                    //Log($"waitingTime: {waitingTime}");
                    windowCache.Clear(); // make sure we have an up to date time
                    specialWarpWait = false;
                }

                if (warpState == 2 && !inWarp() && currentUT > waitingTime + 0.5d)
                {
                    Log("Special warp 2 in progress");
                    TimeWarp.fetch.CancelAutoWarp();
                    TimeWarp.SetRate(5, true); // set to >1x to delay next-stage check
                    TimeWarp.fetch.WarpTo(nextLaunchUT - (warpMargin + 3600d * dayScale)); // warp to within an hour
                    warpState = 3;
                    specialWarpWait = true;
                }

                if (warpState == 3 && !inWarp() && currentUT > waitingTime + 0.5d)
                {
                    Log("Special warp 3 in progress");
                    TimeWarp.fetch.CancelAutoWarp();
                    TimeWarp.fetch.WarpTo(nextLaunchUT - warpMargin); // now warp to final
                    warpState = 0;
                    specialWarpWait = false;
                }

                if (targetPressed)
                {
                    if (!targetSet)
                    {
                        targetSet = true;
                        if (target is Vessel vesselTarget)
                            FlightGlobals.fetch.SetVesselTarget(vesselTarget);
                        else if (target is CelestialBody bodyTarget)
                            FlightGlobals.fetch.SetVesselTarget(bodyTarget);
                        else LogError("Unknown target type passed to targetSet: " + target.GetType().Name);
                    }
                    else
                    {
                        targetSet = false;
                        FlightGlobals.fetch.SetVesselTarget(null);
                    }
                }
            }

            Tooltip.Instance?.RecordTooltip(id);
            GUI.DragWindow();
        }

        #endregion
        #region MakeSettingsWindow

        private void MakeSettingsWindow(int id)
        {
            windowWidth = 500;

            BeginCenter(false);
            GUILayout.Label("Hover over text to see additional information", GUILayout.Width(windowWidth)); // this sets the width of the window
            EndCenter(false);

            GUILayout.BeginHorizontal();
            GUILayout.Label("Use Unity Skin");
            GUILayout.FlexibleSpace();
            BeginCenter();
            useAltSkin = GUILayout.Toggle(useAltSkin, "");
            EndCenter();
            GUILayout.EndHorizontal();

            if (StateChanged("useAltSkin", useAltSkin))
            {
                ResetWindow(ref mainRect);
                ResetWindow(ref settingsRect);
            }

            if (errorState == 0 || errorState == 2 || errorState == 3)
            {
                DrawLine();

                if (errorState == 2 || errorState == 3) GUILayout.Label("<b><i>TOGGLE THIS TO GET OUT OF ERROR</i></b>");
                GUILayout.BeginHorizontal();
                GUILayout.Label("Target an orbiting Vessel instead of an orbiting Moon");
                GUILayout.FlexibleSpace();
                BeginCenter();
                targetVessel = GUILayout.Toggle(targetVessel, "");
                EndCenter();
                GUILayout.EndHorizontal();

                if (StateChanged("targetVessel", targetVessel))
                {
                    ResetWindow(ref mainRect);
                }
            }

            if (errorState == 0)
            {
                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent($"Find the Global Minimum of the {(showAzimuth ? "azimuth" : "inclination")} error instead of the Local Minimum of the {(showAzimuth ? "azimuth" : "inclination")} error", $"Ignored if latitude is higher than {targetName} inclination"));
                GUILayout.FlexibleSpace();
                BeginCenter();
                GUI.enabled = isLowLatitude;
                useAltBehavior = GUILayout.Toggle(useAltBehavior, "");
                GUI.enabled = true;
                EndCenter();
                GUILayout.EndHorizontal();

                //if (!isLowLatitude) useAltBehavior_toggled = false; // if we change useAltBehavior to false instead, the next code will just set it to true again // not needed, we check for isLowLatitude anyway

                if (StateChanged("useAltBehavior", useAltBehavior))
                {
                    //Log("windowCache Cleared due to useAltBehavior change");
                    windowCache.Clear(); // remove local mins so that we can replace them with global mins
                    // technically this only needs to be done when switching from false to true, but switching from true to false without clearing would result in some extra data in the cache, which might lead to problems if abused
                }

                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Use surface vessel position for latitude/longitude instead of launch site position", $"Ignored if not in a {(requireSurfaceVessel ? "surface " : "")}vessel"));
                GUILayout.FlexibleSpace();
                BeginCenter();
                GUI.enabled = inVessel;
                useVesselPosition = GUILayout.Toggle(useVesselPosition, "");
                GUI.enabled = true;
                EndCenter();
                GUILayout.EndHorizontal();

                if (KACInstalled)
                {
                    DrawLine();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Change the \"Add Alarm\" button to set an alarm based on the extra window instead of the next launch window");
                    GUILayout.FlexibleSpace();
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
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                    specialWarpSelected = GUILayout.Toggle(specialWarpSelected, "");
                    EndCenter();
                    GUILayout.EndHorizontal();
                }

                // TODO, activate special warp if altitude of target changes significantly?

                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Show Azimuth instead of Inclination");
                GUILayout.FlexibleSpace();
                BeginCenter();
                showAzimuth = GUILayout.Toggle(showAzimuth, "");
                EndCenter();
                GUILayout.EndHorizontal();

                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Show Extra Window");
                GUILayout.FlexibleSpace();
                BeginCenter();
                expandExtraWindow = GUILayout.Toggle(expandExtraWindow, "");
                EndCenter();
                GUILayout.EndHorizontal();

                if (StateChanged("expandExtraWindow", expandExtraWindow))
                {
                    ResetWindow(ref mainRect);
                    ResetWindow(ref settingsRect);
                }

                if (expandExtraWindow)
                {
                    DrawLine();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"Optimize for a certain {(showPhasing ? "phasing angle" : "phasing time")} in orbit instead of choosing a window number manually");
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                    useWindowOptimizer = GUILayout.Toggle(useWindowOptimizer, "");
                    EndCenter();
                    GUILayout.EndHorizontal();

                    if (StateChanged("useWindowOptimizer", useWindowOptimizer))
                    {
                        ResetWindow(ref settingsRect);
                    }
                }

                DrawLine();

                GUILayout.BeginHorizontal();
                GUILayout.Label($"Switch between reference times for the launch {(showAzimuth ? "azimuth" : "inclination")} infobox");
                GUILayout.FlexibleSpace();
                BeginCenter();
                if (GUILayout.Button(new GUIContent(referenceTimeLabel, referenceTimeTooltip), GUILayout.Width(120))) referenceTimeButton = (referenceTimeButton + 1) % (expandExtraWindow ? 3 : 2);
                EndCenter();
                GUILayout.EndHorizontal();

                if (expandExtraWindow && !useWindowOptimizer)
                {
                    DrawLine();

                    GUILayout.Label("Change Extra Window Number");
                    MakeNumberEditField("extraWindowNumber", ref extraWindowNumber, 1, 1, maxWindows);
                }

                DrawLine();

                GUILayout.Label(new GUIContent("Change Decimal Places of Precision", "Editable text fields will not be effected"));
                MakeNumberEditField("decimals", ref decimals, 1, 0, int.MaxValue);

                DrawLine();

                // azimuth to inclination formulas derived from https://www.astronomicalreturns.com/p/section-46-interesting-orbital.html
                // sin(azimuth) = cos(inclination) / cos(latitude)
                // cos(inclination) = cos(latitude) * sin(azimuth)

                string azimuthTooltip = "90° is the default, which is directly east. Range is 0° to 360°, where 0° and 180° are North and South respectively.";
                string inclinationTooltip = $"Your latitude of {FormatDecimals(latitude)}\u00B0 is the default, which is directly east. Range is -180\u00B0 to 180\u00B0, where +90\u00B0 and -90\u00B0 are North and South respectively.";
                double azRad = targetLaunchAzimuth * degToRad;
                double latRad = latitude * degToRad;
                if (double.IsNaN(targetLaunchInclination)) targetLaunchInclination = targetLaunchAzimuth <= 90d || targetLaunchAzimuth >= 270d
                    ? Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg
                    : -Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg; // initialize value, this isnt rounded but rounding led to floating point issues so whatever

                if (showAzimuth)
                {
                    targetLaunchInclination = targetLaunchAzimuth <= 90d || targetLaunchAzimuth >= 270d
                    ? Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg
                    : -Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg; // continuously update value

                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Change Target Launch Azimuth", azimuthTooltip + " Changing the Target Launch Azimuth may not change the launch window time, this is normal and expected."));
                    ResetDefault(ref targetLaunchAzimuth, 90d);
                    GUILayout.EndHorizontal();
                    MakeNumberEditField("targetLaunchAzimuth", ref targetLaunchAzimuth, 1d, 0d, 360d, true);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Target Launch Inclination", inclinationTooltip));
                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                    GUILayout.Box($"{FormatDecimals(targetLaunchInclination)}\u00B0", GUILayout.MaxWidth(100)); // F2 is overkill but just in case
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();
                }
                else
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Change Target Launch Inclination", inclinationTooltip + " Changing the Target Launch Inclination may not change the launch window time, this is normal and expected."));
                    ResetDefault(ref targetLaunchInclination, latitude);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    MakeNumberEditField("targetLaunchInclination", ref targetLaunchInclination, 1d, -180d, 180d, true);

                    double cosInc = Math.Cos(targetLaunchInclination * degToRad);
                    double cosLat = Math.Cos(latRad);
                    double sinAz = cosInc / cosLat;

                    if (Math.Abs(sinAz) >= 1d)
                    {
                        GUILayout.FlexibleSpace();
                        GUILayout.Label(new GUIContent("Unreachable", $"The Target Inclination of {FormatDecimals(targetLaunchInclination)}° is unreachable from your latitude of {FormatDecimals(latitude)}°, so it has been automatically converted to the nearest reachable inclination."));
                        GUILayout.FlexibleSpace();
                    }
                    GUILayout.EndHorizontal();

                    sinAz = Util.Clamp(sinAz, -1d, 1d);
                    double azInter = Math.Abs(Math.Asin(sinAz) * radToDeg); // intermediate value for azimuth

                    targetLaunchAzimuth = targetLaunchInclination >= 0
                        ? (targetLaunchInclination <= 90d ? azInter : 360d - azInter) // NE (prograde) or NW (retrograde)
                        : (Math.Abs(targetLaunchInclination) <= 90d ? 180d - azInter : 180d + azInter); // SE (prograde) or SW (retrograde)

                    targetLaunchAzimuth = ((targetLaunchAzimuth % 360d) + 360d) % 360d; // normalize just in case

                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Target Launch Azimuth", azimuthTooltip));
                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                    GUILayout.Box($"{FormatDecimals(targetLaunchAzimuth)}\u00B0", GUILayout.MaxWidth(100));
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();
                }
                // its not possible to have both textfields on screen at once, bugs out

                if (StateChanged("targetLaunchAzimuth", targetLaunchAzimuth))
                {
                    windowCache.Clear(); // only clear if changed, also this doesn't always result in new minimums
                }

                if (expandExtraWindow && useWindowOptimizer)
                {
                    DrawLine();
                    double orbitRadius = mainBody.Radius + parkingAltitude * 1000d;
                    double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / mainBody.gravParameter);

                    if (double.IsNaN(targetPhasingTime)) targetPhasingTime = targetPhasingAngle / 360d * orbitPeriod; // initialize value

                    if (showPhasing)
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label("Target Phasing Angle (degrees)");
                        GUILayout.BeginVertical();
                        GUILayout.Space(5);
                        bool showPhasing_pressed = GUILayout.Button(new GUIContent("T", "Switch to phasing time"), GUILayout.Width(20));
                        ButtonPressed(ref showPhasing, showPhasing_pressed);
                        GUILayout.EndVertical();
                        GUILayout.FlexibleSpace();
                        GUILayout.EndHorizontal();
                        MakeNumberEditField("targetPhasingAngle", ref targetPhasingAngle, 1d, 0.01, 360d, true); // min of 0.01 degrees to avoid division by zero

                        targetPhasingTime = targetPhasingAngle / 360d * orbitPeriod;
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Target Phasing Time (seconds)", $"Max of {FormatDecimals(orbitPeriod)} seconds, the orbit period"));
                        GUILayout.BeginVertical();
                        GUILayout.Space(5);
                        GUILayout.Box(FormatTime(targetPhasingTime), GUILayout.MaxWidth(100));
                        GUILayout.EndVertical();
                        GUILayout.FlexibleSpace();
                        GUILayout.EndHorizontal();
                    }
                    else
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Target Phasing Time (seconds)", $"Max of {FormatDecimals(orbitPeriod)} seconds, the orbit period"));
                        GUILayout.BeginVertical();
                        GUILayout.Space(5);
                        bool showPhasing_pressed = GUILayout.Button(new GUIContent(" P", "Switch to phasing angle"), GUILayout.Width(20));
                        ButtonPressed(ref showPhasing, showPhasing_pressed);
                        GUILayout.EndVertical();
                        GUILayout.FlexibleSpace();
                        GUILayout.EndHorizontal();
                        GUILayout.BeginHorizontal();
                        MakeNumberEditField("targetPhasingTime", ref targetPhasingTime, 60d, 1d, orbitPeriod); // min of 1 second to avoid division by zero
                        GUILayout.Box(FormatTime(targetPhasingTime), GUILayout.Width(150));
                        GUILayout.FlexibleSpace();
                        GUILayout.EndHorizontal();

                        GUILayout.BeginHorizontal();
                        GUILayout.Label("Target Phasing Angle (degrees)");
                        GUILayout.BeginVertical();
                        GUILayout.Space(5);
                        GUILayout.Box($"{FormatDecimals(targetPhasingAngle)}\u00B0", GUILayout.MaxWidth(100));
                        GUILayout.EndVertical();
                        GUILayout.FlexibleSpace();
                        GUILayout.EndHorizontal();

                        targetPhasingAngle = targetPhasingTime / orbitPeriod * 360d;
                    }

                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button("Search for Closest Window", GUILayout.Width(200)))
                    {
                        ranSearch = true;
                        int bestWindow = -1;
                        double bestError = double.MaxValue;
                        for (int candidateWindow = 0; candidateWindow <= maxWindows - 1; candidateWindow++)
                        {
                            double candidateLaunchTime = GetCachedLaunchTime(launchPos, latitude, longitude, inclination, useAltBehavior, candidateWindow) - currentUT;

                            if (double.IsNaN(candidateLaunchTime))
                            {
                                Log("A Launchtime was NaN, skipping this window");
                                continue;
                            }

                            if (showPhasing)
                            {
                                (_, double candidatePhaseAngle) = GetCachedPhasingTime(launchPos, candidateLaunchTime, candidateWindow);
                                double errorRatio = Math.Abs(candidatePhaseAngle - targetPhasingAngle) / targetPhasingAngle;
                                if (errorRatio < bestError)
                                {
                                    bestError = errorRatio;
                                    bestWindow = candidateWindow;
                                }
                            }
                            else
                            {
                                (double candidatePhaseTime, _) = GetCachedPhasingTime(launchPos, candidateLaunchTime, candidateWindow);
                                double errorRatio = Math.Abs(candidatePhaseTime - targetPhasingTime) / targetPhasingTime;
                                if (errorRatio < bestError)
                                {
                                    bestError = errorRatio;
                                    bestWindow = candidateWindow;
                                }
                            }
                        }
                        if (bestWindow >= 0) extraWindowNumber = bestWindow + 1;
                    }
                    if (ranSearch)
                    {
                        GUILayout.Label(new GUIContent($"Found window {extraWindowNumber}!", $"Window {extraWindowNumber} is the closest window to your target phasing {(showPhasing ? "angle" : "time")} within the max of {maxWindows} windows"));
                    }
                    GUILayout.EndHorizontal();
                }
                else ranSearch = false; // this is to remove the label

            }

            Tooltip.Instance?.RecordTooltip(id);

            GUI.DragWindow();
        }

        #endregion

        // html tags rendered by ksp: <b> and </b>, <i> and </i>, (add more)
    }
}
