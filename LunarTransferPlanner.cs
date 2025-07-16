using ClickThroughFix;
using ToolbarControl_NS;
using KSP.UI.Screens;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics; // use this for stopwatch
using System.Globalization; // stick to CultureInfo.InvariantCulture
using System.IO;
using System.Linq;
using UnityEngine;

namespace LunarTransferPlanner
{
    internal static class Util
    {
        internal static void Log(string message, string prefix = "[LunarTransferPlanner]")
        {
            UnityEngine.Debug.Log($"{prefix}: {message}"); // could also do KSPLog.print
        }

        internal static void LogWarning(string message, string prefix = "[LunarTransferPlanner]")
        {
            UnityEngine.Debug.LogWarning($"{prefix}: {message}");
        }

        internal static void LogError(string message, string prefix = "[LunarTransferPlanner]")
        { // could also do LogWarning
            UnityEngine.Debug.LogError($"{prefix}: {message}");
        }

        internal static void TryReadValue<T>(ref T target, ConfigNode node, string name)
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
        internal static double Acosh(double x)
        {
            if (x >= 1)
            {
                return Math.Log(x + Math.Sqrt(x * x - 1d)); // ln(x + sqrt(x^2 - 1))
            }
            else
            {
                return Double.NaN;
            }

        }

        // Unity only has Clamp for floats
        internal static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        internal static (double value, bool changed) ClampChanged(double value, double min, double max)
        {
            bool changed;
            double initialValue = value;
            value = Clamp(value, min, max);
            if (value != initialValue) changed = true;
            else changed = false;

            return (value, changed);
        }

        internal static double ClampAngle(double value, bool useRads)
        {
            const double tau = 2 * Math.PI;
            if (useRads) return ((value % tau) + tau) % tau;
            else return ((value % 360d) + 360d) % 360d;
        }

        // max with unlimited values
        internal static double Max(params double[] values)
        {
            if (values == null || values.Length == 0)
                throw new ArgumentException("At least one value is required for Max.");

            double max = values[0];
            for (int i = 1; i < values.Length; i++)
            {
                max = Math.Max(max, values[i]);
            }
            return max;
        }

        // min with unlimited values
        internal static double Min(params double[] values)
        {
            if (values == null || values.Length == 0)
                throw new ArgumentException("At least one value is required for Min.");

            double min = values[0];
            for (int i = 1; i < values.Length; i++)
            {
                min = Math.Min(min, values[i]);
            }
            return min;
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, true)] // startup on main menu according to https://github.com/linuxgurugamer/ToolbarControl/wiki/Registration
    public class RegisterToolbar : MonoBehaviour
    {
        void Start()
        {
            ToolbarControl.RegisterMod("LTP", "Lunar Transfer Planner");
        }
    }

    [KSPAddon(KSPAddon.Startup.AllGameScenes, false)] // TODO, stop it from appearing in R&D, Admininistration, Astronaut Complex, during Loading
    public class LunarTransferPlanner : MonoBehaviour
    {
        #region Fields

        const double EarthSiderealDay = 86164.098903691;
        const double EarthRadius = 6371000; // meters
        const double EarthMass = 3.9860043543609598e+14 / 6.67408e-11; // API docs say 6.673e-11 for the grav constant, which is wrong
        // Earth sources from RealSolarSystem/RSSKopernicus/Earth/Earth.cfg
        const double tau = 2 * Math.PI; // Math.Tau is in .NET 5
        internal const double radToDeg = 180d / Math.PI; // unity only has floats
        internal const double degToRad = Math.PI / 180d; // unity only has floats
        readonly double invphi = (Math.Sqrt(5) - 1) / 2; // positive conjugate of golden ratio

        Rect mainRect = new Rect(100, 100, -1, -1);
        Rect settingsRect = new Rect(100, 100, -1, -1);
        Rect manualOrbitRect = new Rect(100, 100, -1, -1);
        readonly string mainTitle = "Lunar Transfer";
        readonly string settingsTitle = "Additional Settings";
        readonly string manualOrbitTitle = "Specify Manual Orbit";
        bool needMainReset = true;
        bool needSettingsReset = true;
        bool needManualOrbitReset = true;
        enum WindowState
        {
            Main,
            Settings,
            ManualOrbit
        }
        WindowState windowState;
        float windowWidth;
        //float UIScale; // TODO, can't think of a great way to make this look good right now
        GUISkin skin;
        Texture2D gearWhite;
        Texture2D gearGreen;
        Texture2D resetWhite;
        Texture2D resetGreen;
        bool isWindowOpen = false; // hide on first start-up
        bool isKSPGUIActive = true; // for some reason, this initially only turns to true when you turn off and on the KSP GUI
        Vector2 settingsScroll = Vector2.zero; // TODO, save this in settings?
        bool needCacheClear = true;

        int currentBody = -1;
        object target = null; // will later become CelestialBody or Vessel
        CelestialBody mainBody = null;
        Orbit targetOrbit = null;
        string targetName = "";
        Vector3d launchPos;
        bool targetVessel = false;
        bool targetManual = false;
        bool moonsInvalid = true;
        bool vesselsInvalid = true;
        bool KACInstalled;
        bool PrincipiaInstalled;

        double manualEccentricity = double.NaN; // eccentricity of manual target
        double eccentricity_Adj;
        double manualSMA = double.NaN; // semi major axis of manual target, in meters
        double SMA_Adj;
        double manualInclination = double.NaN; // inclination of manual target, in degrees
        double inclination_Adj;
        double manualLAN = double.NaN; // longitude of the ascending node of manual target, in degrees
        double LAN_Adj;
        double manualAoP = double.NaN; // argument of periapsis of manual target, in degrees
        double AoP_Adj;
        double manualMNA = double.NaN; // mean anonaly at epoch of manual target, in radians
        double MNA_Adj;
        double manualApR = double.NaN; // apoapsis from center in meters
        double ApA_Adj = double.NaN;
        double manualPeR = double.NaN; // periapsis from center in meters
        double PeA_Adj = double.NaN;
        double manualPeriod = double.NaN; // period in seconds
        double period_Adj = double.NaN;
        bool useRadians = false;
        bool useCenterDistance = false; // for apoapsis and periapsis, define from sea level instead of from center
        bool? isSavedOrbitCorrect = null;
        int manualTargetMode = 0;
        string manualModeLabel;
        string manualModeTooltip;

        double targetInclination;
        double currentUT;
        double dayScale;
        double solarDayLength;
        bool useHomeSolarDay = true;
        int errorStateTargets = 0;
        bool showSettings = false; // Show settings UI
        bool showManualOrbit = false; // Show manual orbit setting UI
        bool useAltSkin = false; // Use Unity GUI skin instead of default
        double tickSpeed = 0.2;

        double flightTime = double.NaN; // Desired flight time after maneuver, in seconds (this gets initialized to 4d * solarDayLength later)
        double flightTime_Adj = double.NaN; // Desired flight time after maneuver, in whatever unit the user has selected
        int flightTimeMode = 0; // 0, 1, 2, 3 (default to days)
        string flightTimeLabel;
        string flightTimeTooltip;

        double parkingAltitude = 200d; // Parking orbit altitude (circular orbit)
        bool useAltBehavior = false; // Find global minimum for low latitudes instead of local minimum
        double altBehaviorTimeLimit = 30d; // Max time limit for the global minimum search, in sidereal days of mainBody
        bool altBehaviorNaN = false; // Return NaN when a global min can't be found, instead of returning the closest time
        int maxIterations = 10000; // Max iterations for GSS and EstimateTimeAfterManeuver (mostly relevant for EstimateTimeAfterManeuver)
        bool displaySeconds = false;
        bool useVesselPosition = true; // Use vessel position for latitude instead of launch site position, default is true as the KSC location isn't always the same as the actual launch site directly from the VAB
        bool requireSurfaceVessel = true; // Do not consider useVesselPosition if the vessel is not on the surface
        bool inVessel;
        bool useAltAlarm = false; // Set an alarm based on the extra window instead of the next launch window
        bool useKAC = true; // Use KAC if installed
        double latitude = 0d;
        double longitude = 0d;
        //double altitude = 0d; // height above sea level in meters // currently, this has no effect on the calculations, because the vectors are always normalized

        int referenceTimeButton = 1; // 0, 1, 2 (default to Next Window)
        string referenceTimeLabel;
        string referenceTimeTooltip;
        double referenceTime = 0d;
        int? referenceWindowNumber = null;

        double targetLaunchAzimuth = 90d; // due east
        double targetLaunchInclination = double.NaN;
        double targetPhasingAngle = 180d;
        double targetPhasingTime = double.NaN;
        double bestAngle = double.NaN;
        double bestTime = double.NaN;
        bool showAzimuth = false;
        bool expandExtraWindow = true; // Show the extra window chooser
        int extraWindowNumber = 2; // counting from 1
        bool useWindowOptimizer = false; // Optimize for a certain phasing angle/time
        bool expandLatLong = false; // Expand/collapse custom Latitude/Longitude picker
        bool expandAltitude = false; // Expand/collapse parking orbit altitude changer
        bool expandParking0 = false; // Expand/collapse time in parking orbit for launch now
        bool expandParking1 = false; // Expand/collapse time in parking orbit for next window
        bool expandParking2 = false; // Expand/collapse time in parking orbit for extra window
        double maxDeltaVScaled = 100000d; // Max amount of delta-V that can be calculated, scaled based on the length of a sidereal day for the main body
        bool ranSearch = false; // If we've ran the targetPhasingAngle/Time search
        internal static int decimals = 2; // Amount of decimal precision to display
        bool useAngle = false; // Show the phasing angle instead of the time in parking orbit, applies to all boxes
        bool useLAN = true; // Show the LAN of the parking orbit instead of the AoP of the parking orbit, applies to all boxes
        int maxWindows = 100; // Maximum amount of extra windows that can be calculated
        bool isLowLatitude;
        bool targetSet = false;
        double warpMargin = 60d; // Time difference from the launch window that the warp will stop at

        bool specialWarpSelected = true;
        int warpState = 0;
        bool specialWarpWait = false;
        double waitingTime;

        bool displayParking = false; // Show parking orbit in map view
        bool displayTransfer = false; // Show transfer orbit in map view
        bool displayManual = false; // Show manual orbit in map view, if applicable
        bool displayPhasing = false; // Show phasing angle in map view
        OrbitRendererHack _parkingOrbitRenderer = null;
        OrbitRendererHack _transferOrbitRenderer = null;
        OrbitRendererHack _manualOrbitRenderer = null;
        MapAngleRenderer _phasingAngleRenderer = null;

        // colors use r,g,b,a floats from 0 to 1
        // XKCDColors has an impressively large color list, but displaying it in-game would be a nightmare. better for it to be .cfg only
        Color parkingColor = Color.red; // 1,0,0,1
        Color transferColor = Color.green; // 0,1,0,1
        Color manualColor = Color.blue; // 0,0,1,1
        internal static Color startLineColor = Color.red; // 1,0,0,1
        internal static Color endLineColor = Color.green; // 0,1,0,1
        internal static Color arcLineColor = Color.yellow; // 1,0.9215686,0.01568628,1

        List<CelestialBody> moons;
        List<Vessel> vessels;
        double lastLaunchTime = double.NaN;
        readonly List<(object target, double latitude, double longitude, double targetInclination, double absoluteLaunchTime)> windowCache = new List<(object, double, double, double, double)>();
        readonly List<(OrbitData launchOrbit, int windowNumber)> launchOrbitCache = new List<(OrbitData, int)>();
        readonly List<(double phasingTime, double phasingAngle, int windowNumber)> phasingCache = new List<(double, double, int)>();
        readonly List<(double LAN, double AoP, int windowNumber)> LANCache = new List<(double, double, int)>();
        readonly List<(double deltaV, double eccentricity, int errorStateDV, int windowNumber)> deltaVCache = new List<(double, double, int, int)>();
        readonly Dictionary<string, double> nextTickMap = new Dictionary<string, double>();
        readonly Dictionary<string, string> textBuffer = new Dictionary<string, string>();
        readonly Dictionary<string, object> stateBuffer = new Dictionary<string, object>();
        readonly string SettingsPath = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/LunarTransferPlanner/PluginData/settings.cfg");

        ToolbarControl toolbarControl = null;

        #endregion
        #region GUI Setup

        void Awake()
        {
            skin = (GUISkin)GUISkin.Instantiate(HighLogic.Skin);
            skin.button.padding = new RectOffset(2, 2, 2, 2);
            skin.button.margin = new RectOffset(1, 1, 1, 1);
            skin.box.padding = new RectOffset(2, 2, 2, 2);
            skin.box.margin = new RectOffset(1, 1, 1, 1);
            skin.textField.margin = new RectOffset(3, 1, 1, 1);
            skin.textField.padding = new RectOffset(4, 2, 1, 0);

            // white: (255, 255, 255) | green: (183, 255, 0) | 16x16

            Texture2D LoadImage(string url)
            {
                Log($"Loaded {url} image");
                return GameDatabase.Instance.GetTexture("LunarTransferPlanner/Icons/" + url, false);
            }

            gearWhite = LoadImage("gearWhite");
            gearGreen = LoadImage("gearGreen");
            resetWhite = LoadImage("resetWhite");
            resetGreen = LoadImage("resetGreen");

            LoadSettings();

            GameEvents.onShowUI.Add(KSPShowGUI);
            GameEvents.onHideUI.Add(KSPHideGUI);
            GameEvents.onGameSceneLoadRequested.Add(OnSceneChange);
        }

        // for some reason the button icons only load if they're in PluginData, but the other icons only load if they're NOT in PluginData /shrug

        void KSPShowGUI() => isKSPGUIActive = true;

        void KSPHideGUI() => isKSPGUIActive = false;

        void Start()
        {
            //UIScale = GameSettings.UI_SCALE;

            InitToolbar();

            KACWrapper.InitKACWrapper();
            KACInstalled = KACWrapper.APIReady;

            PrincipiaWrapper.InitPrincipiaWrapper();
            PrincipiaInstalled = PrincipiaWrapper.APIReady;

            Tooltip.RecreateInstance(); // Need to make sure that a new Tooltip instance is created after every scene change
        }

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

        void OnDestroy()
        {
            SaveSettings();

            toolbarControl?.OnDestroy();
            Destroy(toolbarControl);

            GameEvents.onShowUI.Remove(KSPShowGUI);
            GameEvents.onHideUI.Remove(KSPHideGUI);
            GameEvents.onGameSceneLoadRequested.Remove(OnSceneChange);

            ClearAllCaches(); // mostly needed for clearing orbits and angle displays
        }

        private void OnSceneChange(GameScenes s) // thanks Nazfib
        { // we could call ToggleWindow here to make it closed during loading, but that would leave it closed during the next scene until the user opens it again
            SaveSettings();

            ClearAllCaches(); // mostly needed for clearing orbits and angle displays
        }

        void OnGUI()
        {
            if (isWindowOpen && isKSPGUIActive) // all windows hide if not true
            { // HighLogic.LoadedScene != GameScenes.LOADING && HighLogic.LoadedScene != GameScenes.LOADINGBUFFER // these dont seem to work? at least not in the way im using them
                GUI.skin = !useAltSkin ? this.skin : null;
                int id0 = this.GetHashCode();
                int id1 = this.GetHashCode() + 1;
                int id2 = this.GetHashCode() + 2;

                mainRect = ClickThruBlocker.GUILayoutWindow(id0, mainRect, MakeMainWindow, mainTitle);
                ClampToScreen(ref mainRect);
                Tooltip.Instance?.ShowTooltip(id0);

                if (showSettings)
                {
                    settingsRect = ClickThruBlocker.GUILayoutWindow(id1, settingsRect, MakeSettingsWindow, settingsTitle);
                    ClampToScreen(ref settingsRect);
                    Tooltip.Instance?.ShowTooltip(id1);
                }

                if (showManualOrbit && targetManual)
                {
                    manualOrbitRect = ClickThruBlocker.GUILayoutWindow(id2, manualOrbitRect, MakeManualOrbitWindow, manualOrbitTitle);
                    ClampToScreen(ref manualOrbitRect);
                    Tooltip.Instance?.ShowTooltip(id2);
                }

                needCacheClear = false; // reset this at the end of every frame
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
                { "manualOrbitRect.xMin", manualOrbitRect.xMin },
                { "manualOrbitRect.yMin", manualOrbitRect.yMin },
                { "isWindowOpen", isWindowOpen },
                { "targetName", targetName }, // we convert it to an actual target later
                { "targetManual", targetManual },

                // targetManual options
                { "manualEccentricity", manualEccentricity },
                { "manualSMA", manualSMA },
                { "manualInclination", manualInclination },
                { "manualLAN", manualLAN },
                { "manualAoP", manualAoP },
                { "manualMNA", manualMNA },
                { "useRadians", useRadians },
                { "useCenterDistance", useCenterDistance },
                { "manualTargetMode", manualTargetMode },
                { "showManualOrbit", showManualOrbit },

                { "targetVessel", targetVessel },
                { "showSettings", showSettings },
                { "useAltSkin", useAltSkin },
                { "tickSpeed", tickSpeed },
                { "useHomeSolarDay", useHomeSolarDay },
                { "flightTime", flightTime },
                { "flightTimeMode", flightTimeMode },
                { "parkingAltitude", parkingAltitude },
                { "useAltBehavior", useAltBehavior },
                { "altBehaviorTimeLimit", altBehaviorTimeLimit },
                { "altBehaviorNaN", altBehaviorNaN },
                { "maxIterations", maxIterations },
                { "displaySeconds", displaySeconds },
                { "useVesselPosition", useVesselPosition },
                { "requireSurfaceVessel", requireSurfaceVessel },
                { "useAltAlarm", useAltAlarm },
                { "useKAC", useKAC },
                { "latitude", latitude },
                { "longitude", longitude },
                { "showAzimuth", showAzimuth },
                { "expandExtraWindow", expandExtraWindow },
                { "referenceTimeButton", referenceTimeButton },
                { "extraWindowNumber", extraWindowNumber },
                { "useWindowOptimizer", useWindowOptimizer },
                { "expandLatLong", expandLatLong },
                { "expandAltitude", expandAltitude },
                { "expandParking0", expandParking0 },
                { "expandParking1", expandParking1 },
                { "expandParking2", expandParking2 },
                { "maxDeltaVScaled", maxDeltaVScaled },
                { "useAngle", useAngle },
                { "useLAN", useLAN },
                { "maxWindows", maxWindows },
                { "warpMargin", warpMargin },
                { "specialWarpSelected", specialWarpSelected },
                { "displayParking", displayParking },
                { "displayTransfer", displayTransfer },
                { "displayManual", displayManual },
                { "displayPhasing", displayPhasing },
                { "targetLaunchAzimuth", targetLaunchAzimuth },
                { "targetPhasingAngle", targetPhasingAngle },
            };

            foreach (KeyValuePair<string, object> kvp in settingValues) settings.AddValue(kvp.Key, kvp.Value);

            ConfigNode root = new ConfigNode();
            root.AddNode(settings);

            ConfigNode colorNode = new ConfigNode("COLORS");

            Dictionary<string, Color> colors = new Dictionary<string, Color>
            {
                { "parkingColor", parkingColor },
                { "transferColor", transferColor },
                { "manualColor", manualColor },
                { "startLineColor", startLineColor },
                { "endLineColor", endLineColor },
                { "arcLineColor", arcLineColor },
            };
            foreach (var kvp in colors)
            {
                Color c = kvp.Value;
                colorNode.AddValue(kvp.Key, $"{c.r},{c.g},{c.b},{c.a}");
            }

            settings.AddNode(colorNode);

            root.Save(SettingsPath); // this makes a new file if settings.cfg didnt exist already

            Dictionary<string, string> comments = new Dictionary<string, string>
            {
                { "maxWindows", "Changes the maximum amount of windows that can be calculated with the extra window chooser (or considered in the phasing angle/time optimizer), default of 100. Each launch window is temporarily cached, so caching a ridiculous amount may lead to performance degradation" },
                { "altBehaviorTimeLimit", "Max time limit for the global minimum search in sidereal days of the main body, default of 30. Increase this if you're getting close local minimums instead of absolute global minimums" },
                { "altBehaviorNaN", "Return a NaN when a global minimum cannot be found within the time limit, instead of returning the best local minimum" },
                { "maxDeltaVScaled", "Max amount of delta-V that can be calculated, scaled based on the length of a sidereal day for the main body, default of 100000 (the Moon is about 3100). Increase if you're getting NaN for delta-V and the error messages say you need to increase the delta-V" },
                { "targetLaunchAzimuth", "Target Inclination is converted to and from Target Azimuth automatically" },
                { "targetPhasingAngle", "Target Phasing Time is converted to and from Target Phasing Angle automatically" },
                { "requireSurfaceVessel", "For useVesselPosition, require that the vessel be on the surface (landed or splashed) for the position to actually be considered" },
                { "useHomeSolarDay", "Use the solar day length of the home body, instead of the currently focused main body, for the purpose of formatting times" },
                { "manualEccentricity", "Only used when targetManual is true. Eccentricity is converted to and from Apoapsis, Periapsis, and/or Period automatically" },
                { "manualSMA", "Only used when targetManual is true. Semi-major axis is converted to and from Apoapsis, Periapsis, and/or Period automatically" },
                { "manualInclination", "Only used when targetManual is true (degrees)" },
                { "manualLAN", "Only used when targetManual is true (degrees)" },
                { "manualAoP", "Only used when targetManual is true (degrees)" },
                { "manualMNA", "Only used when targetManual is true (radians)" },
                { "useRadians", "Only used when targetManual is true" },
                { "useCenterDistance", "Only used when targetManual is true" },
                { "manualTargetMode", "Only used when targetManual is true, ranges from 0 to 8" },
                { "showManualOrbit", "Only used when targetManual is true" },
                { "tickSpeed", "The rate at which holding down the \"-\" or \"+\" button changes the value in seconds, default of 0.2" },
                { "maxIterations", "The max amount of iterations for various calculations, most relevant for EstimateTimeAfterManeuver (which is used for Delta-V calculations). Increase for more accuracy in exchange for a larger lag spike." },
                { "COLORS", "All colors are stored as red, green, blue, and alpha, from 0 to 1" },
            };

            List<string> lines = File.ReadAllLines(SettingsPath).ToList();
            foreach (KeyValuePair<string, string> kvp in comments)
            {
                int index = lines.FindIndex(line => line.Contains(kvp.Key));
                if (index != -1) lines[index] += $" // {kvp.Value}";
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
                    void Read<T>(ref T field, string name) => Util.TryReadValue(ref field, settings, name);

                    float x1 = mainRect.xMin, y1 = mainRect.yMin;
                    float x2 = settingsRect.xMin, y2 = settingsRect.yMin;
                    float x3 = manualOrbitRect.xMin, y3 = manualOrbitRect.yMin;

                    Read(ref x1, "mainRect.xMin");
                    Read(ref y1, "mainRect.yMin");
                    Read(ref x2, "settingsRect.xMin");
                    Read(ref y2, "settingsRect.yMin");
                    Read(ref x3, "manualOrbitRect.xMin");
                    Read(ref y3, "manualOrbitRect.yMin");

                    Read(ref isWindowOpen, "isWindowOpen");
                    Read(ref targetName, "targetName"); // we convert it to an actual target later
                    Read(ref targetManual, "targetManual");

                    // targetManual options
                    Read(ref manualEccentricity, "manualEccentricity");
                    Read(ref manualSMA, "manualSMA");
                    Read(ref manualInclination, "manualInclination");
                    Read(ref manualLAN, "manualLAN");
                    Read(ref manualAoP, "manualAoP");
                    Read(ref manualMNA, "manualMNA");
                    Read(ref useRadians, "useRadians");
                    Read(ref useCenterDistance, "useCenterDistance");
                    Read(ref manualTargetMode, "manualTargetMode");
                    Read(ref showManualOrbit, "showManualOrbit");

                    Read(ref targetVessel, "targetVessel");
                    Read(ref showSettings, "showSettings");
                    Read(ref useAltSkin, "useAltSkin");
                    Read(ref tickSpeed, "tickSpeed");
                    Read(ref flightTime, "flightTime");
                    Read(ref flightTimeMode, "flightTimeMode");
                    Read(ref useHomeSolarDay, "useHomeSolarDay");
                    Read(ref parkingAltitude, "parkingAltitude");
                    Read(ref useAltBehavior, "useAltBehavior");
                    Read(ref altBehaviorTimeLimit, "altBehaviorTimeLimit");
                    Read(ref altBehaviorNaN, "altBehaviorNaN");
                    Read(ref maxIterations, "maxIterations");
                    Read(ref displaySeconds, "displaySeconds");
                    Read(ref useVesselPosition, "useVesselPosition");
                    Read(ref requireSurfaceVessel, "requireSurfaceVessel");
                    Read(ref useAltAlarm, "useAltAlarm");
                    Read(ref useKAC, "useKAC");
                    Read(ref latitude, "latitude");
                    Read(ref longitude, "longitude");
                    Read(ref showAzimuth, "showAzimuth");
                    Read(ref expandExtraWindow, "expandExtraWindow");
                    Read(ref referenceTimeButton, "referenceTimeButton");
                    Read(ref extraWindowNumber, "extraWindowNumber");
                    Read(ref useWindowOptimizer, "useWindowOptimizer");
                    Read(ref expandLatLong, "expandLatLong");
                    Read(ref expandAltitude, "expandAltitude");
                    Read(ref expandParking0, "expandParking0");
                    Read(ref expandParking1, "expandParking1");
                    Read(ref expandParking2, "expandParking2");
                    Read(ref maxDeltaVScaled, "maxDeltaVScaled");
                    Read(ref useAngle, "useAngle");
                    Read(ref useLAN, "useLAN");
                    Read(ref maxWindows, "maxWindows");
                    Read(ref warpMargin, "warpMargin");
                    Read(ref specialWarpSelected, "specialWarpSelected");
                    Read(ref displayParking, "displayParking");
                    Read(ref displayTransfer, "displayTransfer");
                    Read(ref displayManual, "displayManual");
                    Read(ref displayPhasing, "displayPhasing");
                    Read(ref targetLaunchAzimuth, "targetLaunchAzimuth");
                    Read(ref targetPhasingAngle, "targetPhasingAngle");

                    mainRect = new Rect(x1, y1, mainRect.width, mainRect.height);
                    settingsRect = new Rect(x2, y2, settingsRect.width, settingsRect.height);
                    manualOrbitRect = new Rect(x3, y3, manualOrbitRect.width, manualOrbitRect.height);

                    ConfigNode colorNode = settings.GetNode("COLORS");
                    if (colorNode != null)
                    {
                        bool TryParseColor(string s, out Color c)
                        {
                            c = Color.white; // fallback
                            if (string.IsNullOrEmpty(s)) return false;

                            string[] parts = s.Split(',');
                            if (parts.Length != 4) return false;

                            return float.TryParse(parts[0], out float r) &&
                                   float.TryParse(parts[1], out float g) &&
                                   float.TryParse(parts[2], out float b) &&
                                   float.TryParse(parts[3], out float a) &&
                                   ((c = new Color(r, g, b, a)) != null);
                        }

                        Dictionary<string, Action<Color>> colorSetters = new Dictionary<string, Action<Color>>
                        {
                            { "parkingColor", c => parkingColor = c },
                            { "transferColor", c => transferColor = c },
                            { "manualColor", c => manualColor = c },
                            { "startLineColor", c => startLineColor = c },
                            { "endLineColor", c => endLineColor = c },
                            { "arcLineColor", c => arcLineColor = c },
                        };

                        foreach (var kvp in colorSetters)
                        {
                            string value = colorNode.GetValue(kvp.Key);
                            if (TryParseColor(value, out Color c))
                            {
                                kvp.Value(c);
                            }
                        }
                    }
                }
            }
        }

        #endregion
        #region Helper Methods

        private void Log(string message) => Util.Log(message);

        private void LogError(string message) => Util.LogError(message);

        private bool ValueChanged(string key, double value, double tolerance, bool detectFirstAccess = true)
        {
            if (stateBuffer.TryGetValue(key, out var existing) && double.TryParse(existing.ToString(), out double existingDouble))
            {
                if (Math.Abs(existingDouble - value) > tolerance)
                {
                    stateBuffer[key] = value;
                    return true;
                }
                return false;
            }
            else
            {
                stateBuffer[key] = value;
                if (detectFirstAccess) return true; // first access
                else return false;
            }
        }

        private bool StateChanged<T>(string key, T state, bool detectFirstAccess = true)
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
                if (detectFirstAccess) return true; // first access
                else return false;
            }
        }

        private bool StateChanged(bool updateValues, string key, params object[] stateElements) // bool needs to be first to prevent conflicts with other overloads
        {
            if (stateBuffer.TryGetValue(key, out var existing))
            {
                if (!(existing is object[] existingElements) || existingElements.Length != stateElements.Length)
                {
                    if (updateValues) stateBuffer[key] = stateElements;
                    return true;
                }

                for (int i = 0; i < stateElements.Length; i++)
                {
                    if (!Equals(existingElements[i], stateElements[i]))
                    {
                        if (updateValues) stateBuffer[key] = stateElements;
                        return true;
                    }
                }
                return false;
            }
            else
            {
                if (updateValues) stateBuffer[key] = stateElements;
                return true; // first access
            }
        }

        private bool StateChanged<T>(string key, ref T state, T newState)
        {
            state = newState;
            return StateChanged(key, state);
        }

        private T GetState<T>(string key)
        {
            return stateBuffer.TryGetValue(key, out var value) && value is T typedValue ? typedValue : default;
        }

        private object[] GetStateElements(string key)
        {
            return GetState<object[]>(key);
        }

        private double GoldenSectionSearch(double lowerBound, double upperBound, double epsilon, Func<double, double> errorFunc, bool enableLogging = false)
        {
            // adopted from https://en.wikipedia.org/wiki/Golden-section_search
            // I have no idea why this works, but it works so well
            // TODO, switch to Brent?

            double left = upperBound - invphi * (upperBound - lowerBound);
            double right = lowerBound + invphi * (upperBound - lowerBound);
            int i = 0;
            if (enableLogging) Log($"GSS Logging on for {errorFunc.Method.Name}");

            double eLeft = errorFunc(left);
            double eRight = errorFunc(right);

            while (Math.Abs(left - right) > epsilon)
            {
                // do NOT return early if double.NaN, some of the errorFuncs are fine with it
                i++;
                if (eLeft < eRight)
                {
                    upperBound = right;
                    right = left;
                    eRight = eLeft;
                    left = upperBound - invphi * (upperBound - lowerBound);
                    eLeft = errorFunc(left);
                }
                else
                {
                    lowerBound = left;
                    left = right;
                    eLeft = eRight;
                    right = lowerBound + invphi * (upperBound - lowerBound);
                    eRight = errorFunc(right);
                }
                if (i > maxIterations - 1)
                {
                    Log($"GSS for {errorFunc.Method.Name} ran over maxIterations ({maxIterations})");
                    break;
                }
            }

            if (enableLogging) Log($"i for {errorFunc.Method.Name}: {i}");

            return (upperBound + lowerBound) / 2d;
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
        }

        #endregion
        #region Main Methods

        OrbitData CalcOrbitForTime(Vector3d launchPos, double startTime)
        {
            // Remember that Unity (and KSP) use a left-handed coordinate system; therefore, the cross product follows the left-hand rule.

            // Form a plane with the launch site, target and mainBody centre, use this as the orbital plane for launch
            //CelestialBody mainBody = target.referenceBody;
            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = currentUT + flightTime + startTime;
            Vector3d targetPos = targetOrbit.getPositionAtUT(targetTime); // this doesn't take into account changing target inclination due to principia
            //CelestialGetPosition is the corresponding method for Principia, but it doesn't work for a future time. TODO

            Vector3d upVector = QuaternionD.AngleAxis(startTime * 360d / mainBody.rotationPeriod, MainAxis) * (launchPos - MainPos).normalized; // use rotationPeriod for sidereal time, this needs to use startTime

            Vector3d orbitNorm = Vector3d.Cross(targetPos - MainPos, upVector).normalized;

            double inclination = Math.Acos(Vector3d.Dot(orbitNorm, MainAxis)); // inclination of the launch orbit, not the target orbit (this should be able to handle retrograde target orbits? TODO test this)
            if (inclination > Math.PI / 2)
            {
                inclination = Math.PI - inclination;
                orbitNorm *= -1; // make sure orbitNorm always points roughly northwards
            }

            Vector3d eastVec = Vector3d.Cross(upVector, MainAxis).normalized;
            Vector3d northVec = Vector3d.Cross(eastVec, upVector).normalized;
            Vector3d launchVec = Vector3d.Cross(upVector, orbitNorm).normalized;

            //double azimuth = Math.Acos(Vector3d.Dot(launchVec, northVec)); // this only allows azimuths between 0 and 180
            double azimuth = Math.Atan2(Vector3d.Dot(launchVec, eastVec), Vector3d.Dot(launchVec, northVec)); // this allows azimuths between 0 and 360
            azimuth = Util.ClampAngle(azimuth, true);

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
                double refinedTime = GoldenSectionSearch(t0, t1, epsilon, AzimuthError);

                if (refinedTime > t0 + tolerance) // t0 and t1 are on opposite sides of a min (and t0 has a lower error)
                {
                    //Log($"launchTime found at {refinedTime}");
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

            double finalResult = GoldenSectionSearch(fineT0, fineT1, epsilon, AzimuthError);

            // TODO, im unaware of a way to reliably run GoldenSectionSearch only once in this method without severely messing up the logic, its a small optimization but its something

            //Log($"launchTime found at {finalResult}");

            return finalResult;
        }

        private (double LAN, double AoP) CalculateLAN(double latitude, double longitude, double azimuth, double startTime)
        {
            // Remember that Unity (and KSP) use a left-handed coordinate system; therefore, the cross product follows the left-hand rule.

            // rotation angle of the body at target time (in UT) to get longitude of launch location
            double bodyRotationAngle = Util.ClampAngle(mainBody.initialRotation + ((startTime + currentUT) * 360d / mainBody.rotationPeriod) - 180d, false); // im not entirely sure why -180 is needed
            double lonRad = Util.ClampAngle(longitude + bodyRotationAngle, false) * degToRad;

            double latRad = (Math.Abs(latitude) > 1e-9 ? latitude : 1e-9) * degToRad; // prevent NaN from equatorial orbit
            double aziRad = Util.ClampAngle((180d - azimuth) * degToRad, true); // im not entirely sure why 180 is needed
            double r = mainBody.Radius;

            Vector3d pos = new Vector3d(r * Math.Cos(latRad) * Math.Cos(lonRad), r * Math.Sin(latRad), r * Math.Cos(latRad) * Math.Sin(lonRad)); // convert spherical coordinates to cartesian

            Vector3d equatorNormal = mainBody.angularVelocity.normalized; // mainBody.GetTransform().up is also correct I think, but this gets flipped anyway so its fine
            Vector3d east = Vector3d.Cross(equatorNormal, pos).normalized; // this points east when at the launch location
            Vector3d north = Vector3d.Cross(pos, east).normalized; // this points north when at the launch location

            Vector3d launchDir = Math.Sin(aziRad) * east + Math.Cos(aziRad) * north;
            Vector3d orbitNormal = Vector3d.Cross(launchDir, pos).normalized;

            //Log("----------------------------------");

            //Log($"Vector3d.Dot(orbitNormal, Vector3d.up): {Vector3d.Dot(orbitNormal, Vector3d.up)}");

            if (Vector3d.Dot(orbitNormal, Vector3d.up) < 0d)
            {
                orbitNormal *= -1; // make sure orbitNorm always points roughly northwards
            }

            Vector3d nodeVector = Vector3d.Cross(orbitNormal, equatorNormal).normalized; // line of nodes pointing towards ascending node

            //Log($"Math.Acos(Vector3d.Dot(nodeVector, Vector3d.right): {Math.Acos(Vector3d.Dot(nodeVector, Vector3d.right))}");

            double LAN = Util.ClampAngle(Math.Acos(Vector3d.Dot(nodeVector, Vector3d.right)), true);

            //Log($"nodeVector.z: {nodeVector.z}");

            if (nodeVector.z < 0d) LAN = Util.ClampAngle(tau - LAN, true); // make sure LAN is in the correct quadrant

            if ((azimuth > 90d && azimuth <= 180d) || (azimuth >= 270d && azimuth < 360d)) LAN = Util.ClampAngle(LAN + Math.PI, true); // azimuth is pointing south-east or north-west


            Vector3d periapsisVector = (pos - (Vector3d.Dot(pos, orbitNormal) * orbitNormal)).normalized; // bring pos into orbit, consider it the periapsis of the parking orbit

            double AoP = Util.ClampAngle(Math.PI - Math.Acos(Vector3d.Dot(nodeVector, periapsisVector)), true); // im not entirely sure why Math.PI is needed

            //Log($"Vector3d.Dot(Vector3d.Cross(nodeVector, periapsisVector), orbitNormal): {Vector3d.Dot(Vector3d.Cross(nodeVector, periapsisVector), orbitNormal)}");

            if (Vector3d.Dot(Vector3d.Cross(nodeVector, periapsisVector), orbitNormal) < 0d) AoP = Util.ClampAngle(tau - AoP, true);

            // intentionally excluding 180 for these, 180 just doesnt need any flips for some reason
            if (azimuth > 90d && azimuth < 180d) AoP = Util.ClampAngle(AoP + Math.PI, true); // azimuth is pointing south-east
            else if (azimuth > 180d && azimuth < 360d) AoP = Util.ClampAngle(Math.PI - AoP, true); // azimuth is pointing retrograde

            return (LAN * radToDeg, AoP * radToDeg);
        }

        private (double phasingTime, double phasingAngle) EstimateTimeBeforeManeuver(Vector3d launchPos, double startTime)
        {
            // Remember that Unity (and KSP) use a left-handed coordinate system; therefore, the cross product follows the left-hand rule.

            //CelestialBody mainBody = target.referenceBody;
            double gravParameter = mainBody.gravParameter;
            double orbitRadius = mainBody.Radius + parkingAltitude * 1000d;

            Vector3d MainPos = mainBody.position;
            Vector3d MainAxis = mainBody.angularVelocity.normalized;

            double targetTime = currentUT + flightTime + startTime;
            Vector3d targetPos = targetOrbit.getPositionAtUT(targetTime);

            Vector3d upVector = QuaternionD.AngleAxis(startTime * 360d / mainBody.rotationPeriod, MainAxis) * ((launchPos - MainPos).normalized * orbitRadius).normalized; // use rotationPeriod for sidereal time

            // Maneuver takes place at the point of the orbit that is opposite to the future position of the target
            Vector3d maneuverUpVector = (MainPos - targetPos).normalized;

            // KSP lies about having QuaternionD.FromToRotation, EulerAngles, and Euler (https://github.com/KSPModdingLibs/KSPCommunityFixes/issues/316)
            Vector3d rotationAxis = Vector3d.Cross(maneuverUpVector, upVector).normalized;
            double dot = Vector3d.Dot(upVector.normalized, maneuverUpVector.normalized);
            double phasingAngle = Math.Acos(Util.Clamp(dot, -1d, 1d)) * radToDeg;

            if (Vector3d.Dot(rotationAxis, MainAxis) > 1e-9)
            {
                phasingAngle = 360d - phasingAngle;
            }

            // Convert angle to time in orbit
            double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / gravParameter);
            //double offset = 180; // for some reason it just constantly underestimates by ~3 minutes, no idea why
            //phasingAngle = (phasingAngle + (offset / orbitPeriod) * 360d) % 360d;
            double phasingTime = phasingAngle / 360d * orbitPeriod;

            return (phasingTime, phasingAngle);
        }

        private (double time, double eccentricity) EstimateTimeAfterManeuver(double dV, double startUT)
        { // The formulas are from http://www.braeunig.us/space/orbmech.htm
            if (targetOrbit == null) return (double.NaN, double.NaN);

            const double tolerance = 0.01;
            double gravParameter = mainBody.gravParameter;
            double r0 = mainBody.Radius + parkingAltitude * 1000d; // Radius of the circular orbit, including the radius of the mainBody
            double r1 = targetOrbit.GetRadiusAtUT(startUT); // Initial guess for the altitude of the target
            double t1 = double.MaxValue;
            double previousT1 = 0d;
            double v0 = Math.Sqrt(gravParameter / r0) + dV;
            double e = r0 * v0 * v0 / gravParameter - 1;
            if (e == 1d) // e == 1 would mean that the orbit is parabolic. No idea which formulas are applicable in this case.
            {
                v0 += 0.1;
                e = r0 * v0 * v0 / gravParameter - 1;
            }
            double a = 1d / (2d / r0 - v0 * v0 / gravParameter);
            int i = 0;

            // The target is moving, so we need to know its altitude when the vessel arrives
            // For that we need to know the flight time, so we can iterate the flight time until the error is small enough or goes over maxIterations
            // TODO, optimize this much better (https://github.com/Clayell/LunarTransferPlanner/commit/d73af739c4ffa7163fc43c567cb28710b0bc0113#commitcomment-160841101)
            // TLDR of TODO: this converges really slowly, but I can't find a better way to find a good time (maybe something with Lambert?)

            while (Math.Abs(t1 - previousT1) >= tolerance)
            {
                i++;

                previousT1 = t1; // MaxValue

                // True anomaly when the vessel reaches the altitude of the target (r1)
                double trueAnomaly1 = Math.Acos((a * (1d - e * e) - r1) / (e * r1));

                if (e < 1d) // Elliptic orbit
                {
                    double eccAnomaly1 = Math.Acos((e + Math.Cos(trueAnomaly1)) / (1d + e * Math.Cos(trueAnomaly1)));
                    double meanAnomaly1 = eccAnomaly1 - e * Math.Sin(eccAnomaly1);

                    t1 = meanAnomaly1 / Math.Sqrt(gravParameter / Math.Pow(a, 3));
                }
                else // Hyperbolic orbit, Parabolic orbit (e == 1) should have been prevented earlier
                {
                    // Can't use Math.Acosh, it does not seem to work in .NET 4
                    double hEccAnomaly1 = Util.Acosh((e + Math.Cos(trueAnomaly1)) / (1d + e * Math.Cos(trueAnomaly1)));

                    t1 = Math.Sqrt(Math.Pow(-a, 3) / gravParameter) * (e * Math.Sinh(hEccAnomaly1) - hEccAnomaly1);
                }

                // Update r1 using new estimate of flight time
                r1 = targetOrbit.GetRadiusAtUT(startUT + t1);

                //Log($"new r1: {r1}");

                if (double.IsNaN(r1)) // Target is unreachable from this deltaV
                {
                    //Log("Target is unreachable from this deltaV");
                    return (double.NaN, double.NaN);
                }

                //Log($"looped r1: {r1}, t1: {t1}, i: {i}");

                if (i >= maxIterations - 1)
                {
                    Log($"Max iteration limit ({maxIterations}) reached in EstimateTimeAfterManeuver, returning last value");
                    break;
                }
            }

            //Log($"final r1: {r1}, t1: {t1}, i: {i}");

            return (t1, e);
        }

        (double dV, double eccentricity, int errorStateDV) EstimateDV(double startUT)
        {
            //CelestialBody mainBody = target.referenceBody;

            // Search in this range
            double maxPossibleDV = maxDeltaVScaled * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius); // scale by Earth sqrt(mass/radius)
            double expectedFlightTime = flightTime;
            const double epsilon = 1e-9;
            const double tolerance = 0.01;
            int errorStateDV = 0; // no errors

            // current search range, both of these values will change during the search
            double lowerBound = epsilon; // no need to have an actual min for dV
            double upperBound = maxPossibleDV;

            //Log($"Starting up, expectedFlightTime: {expectedFlightTime}, lowerBound: {lowerBound}, maxPossibleDV: {maxPossibleDV}");

            double FlightTimeError(double candidateDV)
            {
                (double estimatedFlightTime, _) = EstimateTimeAfterManeuver(candidateDV, startUT);

                //Log($"estimatedFlightTime: {estimatedFlightTime}, candidateDV: {candidateDV}");

                if (double.IsNaN(estimatedFlightTime))
                    return double.NaN; // invalidate bad guess

                return Math.Abs(estimatedFlightTime - expectedFlightTime);
            }

            double dV = GoldenSectionSearch(lowerBound, upperBound, epsilon, FlightTimeError);

            (double finalTime, double eccentricity) = EstimateTimeAfterManeuver(dV, startUT);

            //Log($"Final Time: {finalTime}, expectedFlightTime: {expectedFlightTime}, maxPossibleDV: {maxPossibleDV}, eccentricity: {eccentricity}");

            if (Math.Abs(dV - maxPossibleDV) <= 1d)
            { // dV seems to get only sorta close to the max, like within .03
                //Log($"dV is above the maximum of {maxPossibleDV} for this body, returning NaN.");
                dV = double.NaN;
                eccentricity = double.NaN;
                errorStateDV = 2;
            }
            else if (double.IsNaN(finalTime) || Math.Abs(finalTime - expectedFlightTime) >= tolerance)
            {
                //Log($"dV is below the minimum possible to reach the target at this parking altitude, returning NaN.");
                dV = double.NaN;
                eccentricity = double.NaN;
                errorStateDV = 1;
            }

            //Log($"Final dV: {dV}, eccentricity: {eccentricity}, errorStateDV: {errorStateDV}");

            return (dV, eccentricity, errorStateDV);
        }

        #endregion
        #region Caching Methods

        // if the factor that should trigger a cache reset can only be changed by the user in one location, then its fine to use StateChanged and then clear the cache
        // StateChanged checks if something is exactly equal to the previous value, which is fine for a value that only the user can change (especially as the user will want the value to update regardless of how small their changes are, which would make a tolerance not make sense)
        // if the factor can be changed by the game itself, or can be changed by the user in multiple ways (in the case of the target), then put the value in the cache and check in the method if its crossed the tolerance
        // (the reset in special warp is a unique case and should be kept as such)

        private void ClearAllCaches() // TODO, just replace all cache clears with this?
        {
            windowCache.Clear();
            launchOrbitCache.Clear();
            phasingCache.Clear();
            LANCache.Clear();
            deltaVCache.Clear();
            ClearAllOrbitDisplays();
            ClearAngleRenderer();
            ResetLaunchInclination();

            needCacheClear = true;
        }

        private void CheckWindowCache(double latitude, double longitude, double targetInclination)
        {
            const double tolerance = 0.01;

            for (int i = 0; i <= windowCache.Count - 1; i++)
            {
                var entry = windowCache[i];
                bool expired = currentUT > entry.absoluteLaunchTime;
                bool targetMismatch = StateChanged("targetManualWindowCache", targetManual) || (!targetManual && entry.target != target); // this will also trigger when changing mainBody, assuming we dont get restarted due to a scene switch
                bool posMismatch = Math.Abs(entry.latitude - latitude) >= tolerance || Math.Abs(entry.longitude - longitude) >= tolerance; // add altitude if necessary, also, we restart when changing launch sites, so posMismatch only triggers when changing position by vessel or manually
                //bool altitudeMismatch = StateChanged("targetAltitudeNaN", targetAltitude == double.NaN) || Math.Abs(entry.targetAltitude - targetAltitude) / targetAltitude >= tolerance; // 1%
                bool inclinationMismatch = Math.Abs(entry.targetInclination - targetInclination) >= tolerance * 2;

                if (expired || targetMismatch || posMismatch || inclinationMismatch)
                {
                    if (expired) lastLaunchTime = windowCache[0].absoluteLaunchTime;
                    //Log($"Resetting Window Cache due to change of Cached Launch Window {i + 1}, old values: target:{entry.target}, latitude: {entry.latitude}, longitude: {entry.longitude}, inclination: {entry.inclination:F3}, time: {entry.absoluteLaunchTime:F3} due to {(expired ? "time expiration " : "")}{(targetMismatch ? "target mismatch " : "")}{(posMismatch ? "position mismatch " : "")}{(inclinationMismatch ? "inclination mismatch " : "")}{(altitudeMismatch ? "altitude mismatch" : "")}");
                    if (targetMismatch) // this will only trigger if the mainBody actually has targets(s)
                    {
                        Log($"Now targeting {(targetManual ? "[Manual Target]" : target)}");
                    }
                    windowCache.Clear(); // dont use windowCache.RemoveAt(i), it leads to compounding errors with the other remaining launch times
                    launchOrbitCache.Clear();
                    phasingCache.Clear();
                    LANCache.Clear();
                    deltaVCache.Clear();
                    ClearAllOrbitDisplays();
                    ClearAngleRenderer();
                    ResetLaunchInclination();
                    break;
                }
            }
        }

        private double GetCachedLaunchTime(Vector3d launchPos, double latitude, double longitude, double targetInclination, bool useAltBehavior, int windowNumber)
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

                if (Math.Abs(newLaunchTime - (lastLaunchTime - currentUT)) < 60d * dayScale && PrincipiaInstalled)
                { // perturbations make a new window that is way too close (when due to time expiration), so just skip to the next one
                    Log("New window is too close, skipping to the next one.");
                    windowNumber++;
                }
                else
                {
                    absoluteLaunchTime = currentUT + newLaunchTime;
                    windowCache.Add((target, latitude, longitude, targetInclination, absoluteLaunchTime));
                }

                if (double.IsNaN(newLaunchTime)) // this needs to be done after we set the cache, otherwise itll go into an endless loop of returning NaN
                {
                    //Log("LaunchTime is NaN, exiting"); // keep this log inside EstimateLaunchTime
                    break;
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

        private (double LAN, double AoP) GetCachedLAN(double latitude, double longitude, double azimuth, double startTime, int? windowNumber = null)
        {
            if (windowNumber.HasValue)
            {
                int index = LANCache.FindIndex(item => item.windowNumber == windowNumber.Value);
                if (index != -1) return (LANCache[index].LAN, LANCache[index].AoP); // return if exists
                else
                {
                    (double LAN, double AoP) = CalculateLAN(latitude, longitude, azimuth, startTime);
                    LANCache.Add((LAN, AoP, windowNumber.Value));
                    return (LAN, AoP);
                }
            }
            else
            {
                return CalculateLAN(latitude, longitude, azimuth, startTime);
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

        private (double dV, double eccentricity, int errorStateDV) GetCachedDeltaV(double startUT, int? windowNumber = null)
        {
            if (windowNumber.HasValue)
            {
                int index = deltaVCache.FindIndex(item => item.windowNumber == windowNumber.Value);
                if (index != -1) return (deltaVCache[index].deltaV, deltaVCache[index].eccentricity, deltaVCache[index].errorStateDV); // return if exists
                else
                {
                    (double dV, double eccentricity, int errorStateDV) = EstimateDV(startUT);
                    deltaVCache.Add((dV, eccentricity, errorStateDV, windowNumber.Value));
                    return (dV, eccentricity, errorStateDV);
                }
            }
            else
            {
                return EstimateDV(startUT);
            }
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

            if (minValueDouble > maxValueDouble) Log("Min value is greater than max value!");

            //valueDouble = Math.Max(epsilon, valueDouble);
            if (double.IsNaN(valueDouble)) valueDouble = minValueDouble;
            valueDouble = Util.Clamp(valueDouble, minValueDouble, maxValueDouble);
            double testValue = Math.Round(valueDouble);
            if (Math.Abs(testValue - valueDouble) < epsilon) valueDouble = testValue; // this works for stuff like 3.99999999, but not for stuff like 4.0999999999. oh well

            stepDouble = Math.Max(epsilon, stepDouble);
            if (!wrapAround) minValueDouble = Math.Abs(minValueDouble) < epsilon ? epsilon : minValueDouble;
            maxValueDouble = Math.Abs(maxValueDouble) < epsilon ? epsilon : maxValueDouble;

            // retrieve tick time buffer
            if (!nextTickMap.TryGetValue(controlId, out double nextTick))
                nextTick = 0d;

            // retrieve text buffer
            if (!textBuffer.TryGetValue(controlId, out string textValue))
                textValue = valueDouble.ToString("G17", CultureInfo.InvariantCulture);

            if (double.TryParse(textValue, NumberStyles.Float, CultureInfo.InvariantCulture, out double parsedBufferValue))
            {
                if (Math.Abs(parsedBufferValue - valueDouble) > epsilon)
                {
                    // external change detected, update buffer
                    textValue = valueDouble.ToString("G17", CultureInfo.InvariantCulture);
                    textBuffer[controlId] = textValue;
                }
            }
            else
            {
                // invalid buffer, resync
                textValue = valueDouble.ToString("G17", CultureInfo.InvariantCulture);
                textBuffer[controlId] = textValue;
            }

            GUILayout.BeginHorizontal();

            string newLabel = GUILayout.TextField(textValue, GUILayout.Width(Mathf.Clamp(GUI.skin.textField.CalcSize(new GUIContent(textValue)).x + 10, 60, windowWidth - (40 * 2)))); // change width based on text length
            // if are other things on the same line (like labels), the width detection wont really work and will increase the width of the window past the windowWidth, TODO

            // if text changed, update buffer and try to parse value
            if (newLabel != textValue)
            {
                textBuffer[controlId] = newLabel;

                if (double.TryParse(newLabel, NumberStyles.Float, CultureInfo.InvariantCulture, out double newValue))
                {
                    valueDouble = Util.Clamp(newValue, minValueDouble, maxValueDouble);
                }
            }

            bool canDecrease = wrapAround || valueDouble > minValueDouble + epsilon;
            bool canIncrease = wrapAround || valueDouble < maxValueDouble - epsilon;

            GUI.enabled = canDecrease;
            if (!canDecrease) minusTooltip += (minusTooltip == "" ? "" : "\n") + "This button is currently disabled as the value is at the minimum";
            bool hitMinusButton = GUILayout.RepeatButton(new GUIContent("\u2013", minusTooltip), GUILayout.MinWidth(40), GUILayout.MaxWidth(60)); // en dash shows up as the same width as + ingame, while the minus symbol is way thinner
            GUI.enabled = true;

            GUI.enabled = canIncrease;
            if (!canIncrease) plusTooltip += (plusTooltip == "" ? "" : "\n") + "This button is currently disabled as the value is at the maximum";
            bool hitPlusButton = GUILayout.RepeatButton(new GUIContent("+", plusTooltip), GUILayout.MinWidth(40), GUILayout.MaxWidth(60));
            GUI.enabled = true;

            if (hitPlusButton || hitMinusButton)
            {
                double tick = Time.realtimeSinceStartup; // TODO, account for game lag which can cause the button to repeat itself
                if (tick > nextTick)
                {
                    int decimals = -1;

                    void SetDecimals(double number)
                    {
                        string str = number.ToString("0.###############", CultureInfo.InvariantCulture); // this is such a hack but it works for rounding (15 #, 16 leads to issues with Math.Round)
                        // probably the best solution would be to switch to decimal, but thats a lot of work

                        int index = str.IndexOf('.');
                        if (index == -1) decimals = 0;
                        else decimals = str.Length - index - 1;
                    }

                    if (hitMinusButton && canDecrease)
                    {
                        if (wrapAround && valueDouble <= minValueDouble + epsilon)
                        {
                            valueDouble = maxValueDouble;
                            SetDecimals(maxValueDouble);
                        }
                        else
                        {
                            double floored = Math.Floor(valueDouble / stepDouble) * stepDouble;
                            if (floored <= valueDouble - epsilon)
                            {
                                if (floored > minValueDouble + epsilon)
                                {
                                    valueDouble = floored;
                                    SetDecimals(stepDouble);
                                }
                                else
                                {
                                    valueDouble = minValueDouble;
                                    SetDecimals(minValueDouble);
                                }
                            }
                            else // already on a step
                            {
                                double next = valueDouble - stepDouble;
                                if (next > minValueDouble + epsilon)
                                {
                                    valueDouble = next;
                                    SetDecimals(stepDouble);
                                }
                                else
                                {
                                    valueDouble = minValueDouble;
                                    SetDecimals(minValueDouble);
                                }
                            }
                        }
                    }
                    else if (hitPlusButton && canIncrease)
                    {
                        if (wrapAround && valueDouble >= maxValueDouble - epsilon)
                        {
                            valueDouble = minValueDouble;
                            SetDecimals(minValueDouble);
                        }
                        else
                        {
                            double ceilinged = Math.Ceiling(valueDouble / stepDouble) * stepDouble; // yes ceilinged is a word
                            if (ceilinged >= valueDouble + epsilon)
                            {
                                if (ceilinged < maxValueDouble - epsilon)
                                {
                                    valueDouble = ceilinged;
                                    SetDecimals(stepDouble);
                                }
                                else
                                {
                                    valueDouble = maxValueDouble;
                                    SetDecimals(maxValueDouble);
                                }
                            }
                            else // already on a step
                            {
                                double next = valueDouble + stepDouble;
                                if (next < maxValueDouble - epsilon)
                                {
                                    valueDouble = next;
                                    SetDecimals(stepDouble);
                                }
                                else
                                {
                                    valueDouble = maxValueDouble;
                                    SetDecimals(maxValueDouble);
                                }
                            }
                        }
                    }

                    //Log($"initial valueDouble: {valueDouble}, decimals: {decimals}");

                    nextTickMap[controlId] = tick + tickSpeed;

                    if (decimals != -1)
                    {
                        valueDouble = Math.Round(valueDouble, decimals);
                        textBuffer[controlId] = valueDouble.ToString($"F{decimals}", CultureInfo.InvariantCulture);
                        //Log($"set valueDouble and textBuffer, valueDouble: {valueDouble}");
                    }
                }
            }

            value = (T)Convert.ChangeType(valueDouble, typeof(T)); // convert back to original type

            GUILayout.EndHorizontal();
        }

        private string FormatTime(double t)
        {
            if (displaySeconds) return $"{FormatDecimals(t)}s";
            else
            {
                // TODO, add years? would have to be similar to useHomeSolarDay
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
        }

        internal static string FormatDecimals(double value, int extra = 0)
        {
            return $"{value.ToString($"F{Math.Max(0, decimals + extra)}", CultureInfo.InvariantCulture)}";
        }

        private void ResetWindow(ref bool needsReset, ref Rect rect) // This should only be used at the end of the current window
        { // Doing this forces the window to be resized. Without it, the window will become bigger when controls expand, but never become smaller again
            if (needsReset)
            {
                rect = new Rect(rect.xMin, rect.yMin, -1f, -1f);
                needsReset = false;
            }
        }

        private void ResetWindow(WindowState windowState) // This should only be used when resetting windows other than the current one
        {
            switch (windowState)
            {
                case WindowState.Main:
                    needMainReset = true;
                    break;
                case WindowState.Settings:
                    needSettingsReset = true;
                    break;
                case WindowState.ManualOrbit:
                    needManualOrbitReset = true;
                    break;
            }
        }

        private void ResetWindow() => ResetWindow(windowState); // This resets the current window

        private void ResetDefault(ref double value, double defaultValue, string tooltip = "Reset to Default", bool pushDown = true) => value = ResetDefault(tooltip, pushDown) ? defaultValue : value;

        private bool ResetDefault(string tooltip = "Reset to Default", bool pushDown = true)
        { // this is for really expensive calculations so that we dont execute them every frame
            bool pressed;
            GUILayout.BeginHorizontal();
            if (pushDown) GUILayout.BeginVertical(); // for some reason having a blank vertical causes issues with spacing, so it needs to be conditional
            if (pushDown) GUILayout.Space(5);
            if (resetWhite != null && resetGreen != null)
            {
                pressed = GUILayout.Button(new GUIContent(useAltSkin ? resetWhite : resetGreen, tooltip), new GUIStyle(GUI.skin.button) { padding = new RectOffset(0, 0, 0, 0) }, GUILayout.Width(20), GUILayout.Height(20));
                // remove padding in style to prevent image getting scaled down with unity skin
            }
            else
            {
                pressed = GUILayout.Button(new GUIContent(" R", tooltip + "\nError: A reset icon is missing!"), GUILayout.Width(20), GUILayout.Height(20));
            }
            if (pushDown) GUILayout.EndVertical();
            GUILayout.FlexibleSpace(); // push to left
            GUILayout.EndHorizontal();

            return pressed;
        }

        private void ClearAllOrbitDisplays()
        {
            ClearOrbitDisplay(ref _parkingOrbitRenderer);
            ClearOrbitDisplay(ref _transferOrbitRenderer);
            ClearOrbitDisplay(ref _manualOrbitRenderer);
        }

        private void ClearOrbitDisplay(ref OrbitRendererHack renderer)
        {
            renderer?.Cleanup();
            renderer = null;
        }

        private void ClearAngleRenderer() 
        {
            _phasingAngleRenderer?.Hide();
            Destroy(_phasingAngleRenderer);
            _phasingAngleRenderer = null;
            
        }

        private void ResetLaunchInclination()
        {
            double azRad = targetLaunchAzimuth * degToRad;
            double latRad = latitude * degToRad;

            targetLaunchInclination = targetLaunchAzimuth <= 90d || targetLaunchAzimuth >= 270d
            ? Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg
            : -Math.Acos(Math.Cos(latRad) * Math.Sin(azRad)) * radToDeg;
        }

        private void DrawLine() // use this in the main window? TODO
        {
            GUILayout.Space(10);
            GUIStyle lineStyle = new GUIStyle();
            lineStyle.normal.background = Texture2D.whiteTexture;
            lineStyle.padding = new RectOffset(0, 0, 0, 0);
            lineStyle.margin = new RectOffset(0, 0, 0, 0);
            lineStyle.border = new RectOffset(0, 0, 0, 0);
            GUILayout.Box("", lineStyle, GUILayout.Height(2), GUILayout.ExpandWidth(true));
            GUILayout.Space(10);
        }

        private void ShowOrbitInfo(ref bool useAngle, ref bool useLAN, double phaseTime, double phaseAngle, double launchLAN, double launchAoP)
        {
            GUILayout.Space(5);

            if (useAngle)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Phasing angle", "Phasing angle between launch location and maneuver in orbit, max of 360\u00B0"), GUILayout.ExpandWidth(true));
                if (GUILayout.Button(new GUIContent("Time", "Switch to phasing time"), GUILayout.Width(60))) useAngle = !useAngle;
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{FormatDecimals(phaseAngle)}\u00B0", $"{FormatDecimals(phaseAngle * degToRad)} rads"));
            }
            else
            {
                double orbitRadius = mainBody.Radius + parkingAltitude * 1000d;
                double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / mainBody.gravParameter);

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Phasing time", $"Phasing time spent waiting in parking orbit before maneuver, max of {FormatDecimals(orbitPeriod)} seconds (the orbit period)"), GUILayout.ExpandWidth(true));
                if (GUILayout.Button(new GUIContent("Angle", "Switch to phasing angle"), GUILayout.Width(60))) useAngle = !useAngle;
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent(FormatTime(phaseTime), $"{FormatDecimals(phaseTime)}s"));
            }

            GUILayout.Space(5);

            if (useLAN)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("LAN", "Longitude of the Ascending Node of the Parking Orbit, max of 360\u00B0"), GUILayout.ExpandWidth(true));
                if (GUILayout.Button(new GUIContent("AoP", "Switch to Argument of Periapsis"), GUILayout.Width(40))) useLAN = !useLAN;
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{FormatDecimals(launchLAN)}\u00B0", $"{FormatDecimals(launchLAN * degToRad)} rads"));
            }
            else
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("AoP", "Argument of Periapsis of the Parking Orbit (if the position in orbit directly above the launch location was the periapsis), max of 360\u00B0\nAdd this to the phasing angle to get the AoP of the maneuver"), GUILayout.ExpandWidth(true));
                if (GUILayout.Button(new GUIContent("LAN", "Switch to Longitude of the Ascending Node"), GUILayout.Width(40))) useLAN = !useLAN;
                GUILayout.EndHorizontal();
                GUILayout.Box(new GUIContent($"{FormatDecimals(launchAoP)}\u00B0", $"{FormatDecimals(launchAoP * degToRad)} rads"));
            }
        }

        private void ShowSettings() => ShowSettings(ref showSettings); // this is for the normal settings menu

        private void ShowSettings(ref bool button, string tooltip = "Show Settings", bool pushDown = false)
        {
            if (pushDown) GUILayout.BeginVertical(); // for some reason having a blank vertical causes issues with spacing, so it needs to be conditional
            if (pushDown) GUILayout.Space(5);
            if (gearWhite != null && gearGreen != null)
            {
                if (GUILayout.Button(new GUIContent(useAltSkin ? gearWhite : gearGreen, tooltip), new GUIStyle(GUI.skin.button) { padding = new RectOffset(0, 0, 0, 0) }, GUILayout.Width(20), GUILayout.Height(20))) button = !button;
                // remove padding in style to prevent image getting scaled down with unity skin
            }
            else
            {
                if (GUILayout.Button(new GUIContent("S", tooltip + "\nError: A gear icon is missing!"), GUILayout.Width(20), GUILayout.Height(20))) button = !button;
            }
            if (pushDown) GUILayout.EndVertical();
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
        } // this is mainly for the visible text, putting the full text in the tooltip is a good idea anyway

        private void ExpandCollapse(ref bool button, string tooltip = "", bool overrideButton = false)
        {
            GUILayout.BeginVertical();
            GUILayout.Space(5); // push down 5

            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace(); // push button to the right
            GUI.enabled = !overrideButton;
            if (GUILayout.Button(new GUIContent(!button ? "+" : "\u2013", tooltip + (overrideButton ? "\nThis button is currently disabled" : "")), GUILayout.Width(30))) // en dash shows up as the same width as + ingame, while the minus symbol is way thinner
            { button = !button; ResetWindow(); }
            GUI.enabled = true;
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }

        #endregion
        #region MakeMainWindow

        private void MakeMainWindow(int id)
        {
            windowWidth = 160;
            windowState = WindowState.Main;

            try
            {
                currentUT = Planetarium.GetUniversalTime();

                if (FlightGlobals.currentMainBody != null) // this makes it impossible to switch your mainBody in flight map view without physically changing your SOI. TODO?
                    mainBody = FlightGlobals.currentMainBody; // spacecenter/flight/mapview
                else if (MapView.MapIsEnabled && MapView.MapCamera?.target?.celestialBody != null) // if we dont check that its in map view, then the vab/sph body will get overwritten
                    mainBody = MapView.MapCamera.target.celestialBody; // tracking station, technically works for flight map view too but we already checked that
                else if (FlightGlobals.GetHomeBody() != null)
                    mainBody = FlightGlobals.GetHomeBody(); // vab/sph, this always gives the home body (could also do Planetarium.fetch.Home)
                else LogError("CRITICAL ERROR: No main body found!");

                // TODO, implement setting that allows people to switch mainBody in the flight map view instead of being locked to their currentMainBody

                if (StateChanged("mainBody", mainBody))
                {
                    target = null;
                    targetName = "";
                    ClearAllCaches();
                    moons = mainBody?.orbitingBodies?.OrderBy(body => body.bodyName).ToList();
                    vessels = FlightGlobals.Vessels?.Where(vessel => vessel != null && mainBody != null && vessel.mainBody == mainBody && vessel.situation == Vessel.Situations.ORBITING).OrderBy(vessel => vessel.vesselName).ToList();
                }
                // TODO, instead of ordering alphabetically, order by closest periapsis? make this a setting

                moonsInvalid = moons == null || moons.Count == 0;
                vesselsInvalid = vessels == null || vessels.Count == 0;

                if (mainBody == null)
                {
                    GUILayout.Label("CRITICAL ERROR: No main body found!", GUILayout.Width(windowWidth)); // this is really bad
                    if (StateChanged("errorStateTargets", ref errorStateTargets, 4))
                    {
                        ResetWindow();
                        //ResetWindow(ref settingsRect);

                        ClearAllCaches();
                    }
                    ShowSettings();
                }
                else if (!targetManual && moonsInvalid && vesselsInvalid)
                {
                    GUILayout.Label("ERROR: There are no moons or vessels orbiting this planet!", GUILayout.Width(windowWidth));
                    if (StateChanged("errorStateTargets", ref errorStateTargets, 1))
                    {
                        ResetWindow();
                        //ResetWindow(ref settingsRect);

                        ClearAllCaches();
                    }
                    ShowSettings();
                }
                else if (!targetManual && targetVessel && vesselsInvalid)
                {
                    GUILayout.Label("ERROR: There are no vessels orbiting this planet!", GUILayout.Width(windowWidth));
                    if (StateChanged("errorStateTargets", ref errorStateTargets, 2))
                    {
                        ResetWindow();
                        //ResetWindow(ref settingsRect);
                    }
                    ShowSettings();
                    GUILayout.Label("If you want to get out of this error, open settings and toggle the \"<i>Target an orbiting Vessel instead of an orbiting Moon</i>\" button.");
                    // do not switch automatically, this would change the user's settings silently, and they may not want to switch
                }
                else if (!targetManual && !targetVessel && moonsInvalid)
                {
                    GUILayout.Box("ERROR: There are no moons orbiting this planet!", GUILayout.Width(windowWidth));
                    if (StateChanged("errorStateTargets", ref errorStateTargets, 3))
                    {
                        ResetWindow();
                        //ResetWindow(ref settingsRect);

                        ClearAllCaches();
                    }
                    ShowSettings();
                    GUILayout.Label("If you want to get out of this error, open settings and toggle the \"<i>Target an orbiting Vessel instead of an orbiting Moon</i>\" button.");
                    // do not switch automatically, this would change the user's settings silently, and they may not want to switch
                }
                else
                {
                    if (StateChanged("errorStateTargets", ref errorStateTargets, 0))
                    {
                        ResetWindow();
                        //ResetWindow(ref settingsRect);

                        ClearAllCaches();
                    }

                    int count = -1;

                    if (!targetManual)
                    {
                        _ = StateChanged("targetManual", false); // this should have been done already, but just in case
                        Vessel matchingVessel = null;
                        CelestialBody matchingMoon = null;
                        if (targetName != "") // this is for loading the target from settings
                        {
                            matchingVessel = vessels?.FirstOrDefault(v => v.vesselName == targetName);
                            matchingMoon = moons?.FirstOrDefault(b => b.bodyName == targetName);
                        }

                        if (targetVessel)
                        {
                            if (target == null || StateChanged("targetVessel", targetVessel))
                            {
                                if (matchingVessel != null) target = matchingVessel as Vessel;
                                else target = vessels[0] as Vessel;
                            }
                            count = vessels.Count;
                        }
                        else
                        {
                            if (target == null || StateChanged("targetVessel", targetVessel))
                            {
                                if (matchingMoon != null) target = matchingMoon as CelestialBody;
                                else target = moons[0] as CelestialBody;
                            }
                            count = moons.Count;
                        }

                        if (target is Vessel vessel)
                        {
                            targetOrbit = vessel?.orbit;
                            targetName = vessel?.vesselName;
                            if (currentBody == -1) currentBody = vessels.FindIndex(v => v.vesselName == targetName);
                        }
                        else if (target is CelestialBody body)
                        {
                            targetOrbit = body?.orbit;
                            targetName = body?.bodyName;
                            if (currentBody == -1) currentBody = moons.FindIndex(b => b.bodyName == targetName);
                        }
                        else LogError("Unknown target type: " + target.GetType().Name);
                    }
                    else
                    {
                        if (double.IsNaN(manualEccentricity)) manualEccentricity = 0d;
                        if (double.IsNaN(manualSMA)) manualSMA = mainBody.Radius + mainBody.atmosphereDepth;
                        if (double.IsNaN(manualInclination)) manualInclination = 0d;
                        if (double.IsNaN(manualLAN)) manualLAN = 0d;
                        if (double.IsNaN(manualAoP)) manualAoP = 0d;
                        if (double.IsNaN(manualMNA)) manualMNA = 0d;

                        double radiusAdjusted = useCenterDistance ? 0d : mainBody.Radius;
                        if (double.IsNaN(manualApR)) manualApR = manualSMA * (1d + manualEccentricity);
                        if (double.IsNaN(ApA_Adj)) ApA_Adj = (manualApR - radiusAdjusted) / 1000d;
                        if (double.IsNaN(manualPeR)) manualPeR = manualSMA * (1d - manualEccentricity);
                        if (double.IsNaN(PeA_Adj)) PeA_Adj = (manualPeR - radiusAdjusted) / 1000d;
                        if (double.IsNaN(manualPeriod)) manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / mainBody.gravParameter);
                        if (double.IsNaN(period_Adj)) period_Adj = manualPeriod;

                        if (StateChanged("targetManual", true)) // || StateChanged("mainBodyManualTarget", mainBody) // dont think this is needed
                        { // Init and SetOrbit murder FPS, so they should only be called when absolutely necessary (like 200 FPS drop if done every frame)
                            targetOrbit = new Orbit
                            {
                                eccentricity = manualEccentricity,
                                semiMajorAxis = manualSMA,
                                inclination = manualInclination,
                                LAN = manualLAN,
                                argumentOfPeriapsis = manualAoP,
                                meanAnomalyAtEpoch = manualMNA,
                                epoch = currentUT,
                                referenceBody = mainBody,
                            };
                            targetOrbit.Init();

                            Log("Manual Orbit Initialized");
                        } // we handle changes of manual targetOrbit in the orbit selector screen

                        targetName = "[Manual Target]";
                    }

                    const double epsilon = 1e-9;
                    targetInclination = GetTargetInclination(targetOrbit); // this works regardless of manual or non-manual target
                    isLowLatitude = Math.Abs(latitude) <= targetInclination;
                    dayScale = mainBody.rotationPeriod / EarthSiderealDay;
                    CelestialBody homeBody = FlightGlobals.GetHomeBody();
                    solarDayLength = useHomeSolarDay ? homeBody.solarDayLength : mainBody.solarDayLength;
                    //targetAltitude = targetOrbit != null ? targetOrbit.GetRadiusAtUT(currentUT) : double.NaN;

                    CheckWindowCache(latitude, longitude, targetInclination);

                    if (requireSurfaceVessel) inVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null && (FlightGlobals.ActiveVessel.Landed || FlightGlobals.ActiveVessel.Splashed);
                    else inVessel = HighLogic.LoadedSceneIsFlight && FlightGlobals.ActiveVessel != null; // this needs to be set here, as settings window isnt always open

                    if (!targetManual)
                    {
                        GUILayout.Space(5);

                        if (count > 1) // only display target selector screen if theres multiple targets
                        {
                            if (currentBody == -1) currentBody = 0; // currentBody should already be set, but just in case
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
                            ResetWindow();
                        }
                    }
                    else
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("<b>Manual Target</b>", $"Manual Target Mode is on, open the orbit specifier to specify the orbit{(PrincipiaInstalled ? "\nThe manual orbit specifier does not take into account perturbations from Principia" : "")}"));
                        GUILayout.FlexibleSpace();
                        ShowSettings(ref showManualOrbit, manualOrbitTitle, true);
                        GUILayout.EndHorizontal();

                        GUILayout.Space(5);
                    }

                    GUILayout.BeginHorizontal();
                    if (mainBody != homeBody && (!useVesselPosition || !inVessel) && !expandLatLong) GUILayout.Label(new GUIContent("<b>!!!</b>", $"Using latitude/longitude of the Space Center on a body that is not {homeBody.bodyName}!"));
                    GUILayout.Label(new GUIContent($"Latitude: <b>{FormatDecimals(latitude)}\u00B0</b>", $"{latitude}\u00B0\nCurrently using {(expandLatLong ? "manual" : (useVesselPosition && inVessel ? "Active Vessel as" : "Space Center as"))} launch location"), GUILayout.ExpandWidth(true));
                    ExpandCollapse(ref expandLatLong, "Set manual latitude and longitude");
                    GUILayout.EndHorizontal();

                    if (expandLatLong)
                    {
                        GUILayout.Space(6); // weird spacing
                        MakeNumberEditField("latitude", ref latitude, 1d, -90d, 90d, true);
                        GUILayout.Space(5);
                        GUILayout.Label(new GUIContent($"Longitude: <b>{FormatDecimals(longitude)}\u00B0</b>", $"{longitude}\u00B0"));
                        MakeNumberEditField("longitude", ref longitude, 1d, -180d, 180d, true);
                        //GUILayout.Space(5);
                        //GUILayout.Label(new GUIContent($"Altitude: <b>{FormatDecimals(altitude)}</b>", "Altitude of launch location (in meters)"));
                        //MakeNumberEditField("altitude", ref altitude, 100d, -mainBody.Radius + 5d, targetOrbit.PeA - 5d); // this is a laughably large range, but its the only way to make sure it can cover altitudes below and above sea level
                        launchPos = mainBody.GetWorldSurfacePosition(latitude, longitude, 0d);
                    }
                    else launchPos = GetLaunchPos(mainBody, ref latitude, ref longitude, useVesselPosition);

                    //TODO, add button to add waypoint at launchPos? kept getting a NRE but perhaps im doing it wrong

                    if (double.IsNaN(flightTime)) flightTime = 4d * solarDayLength;

                    GUILayout.Space(5);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Flight Time", $"Coast duration to {targetName} after the maneuver\nClick the button to the right to change the unit"));
                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                    if (GUILayout.Button(new GUIContent($"({flightTimeLabel})", flightTimeTooltip), GUILayout.Width(25))) flightTimeMode = (flightTimeMode + 1) % 4;
                    GUILayout.EndVertical();

                    switch (flightTimeMode) // TODO, round solarDayLength to prevent excessive decimals?
                    {
                        case 0:
                            flightTimeLabel = "d";
                            flightTimeTooltip = $"Currently using {(useHomeSolarDay ? homeBody.bodyName : mainBody.bodyName)} solar days\nClick to change flight time unit to hours";
                            if (double.IsNaN(flightTime_Adj))
                            {
                                flightTime_Adj = flightTime / solarDayLength;
                            }
                            else if (StateChanged("flightTimeMode", flightTimeMode, false))
                            {
                                flightTime_Adj /= solarDayLength;
                            }
                            flightTime = flightTime_Adj * solarDayLength;
                            break;
                        case 1:
                            flightTimeLabel = "h";
                            flightTimeTooltip = "Currently using hours\nClick to change flight time unit to minutes";
                            if (double.IsNaN(flightTime_Adj))
                            {
                                flightTime_Adj = flightTime / (60d * 60d);
                            }
                            else if (StateChanged("flightTimeMode", flightTimeMode, false))
                            {
                                flightTime_Adj *= solarDayLength / (60d * 60d);
                            }
                            flightTime = flightTime_Adj * 60d * 60d;
                            break;
                        case 2:
                            flightTimeLabel = "m";
                            flightTimeTooltip = "Currently using minutes\nClick to change flight time unit to seconds";
                            if (double.IsNaN(flightTime_Adj))
                            {
                                flightTime_Adj = flightTime / 60d;
                            }
                            else if (StateChanged("flightTimeMode", flightTimeMode, false))
                            {
                                flightTime_Adj *= 60d;
                            }
                            flightTime = flightTime_Adj * 60d;
                            break;
                        case 3:
                            flightTimeLabel = "s";
                            flightTimeTooltip = $"Currently using seconds\nClick to change flight time unit to {(useHomeSolarDay ? homeBody.bodyName : mainBody.bodyName)} solar days";
                            if (double.IsNaN(flightTime_Adj))
                            {
                                flightTime_Adj = flightTime;
                            }
                            else if (StateChanged("flightTimeMode", flightTimeMode, false))
                            {
                                flightTime_Adj *= 60d;
                            }
                            flightTime = flightTime_Adj;
                            break;
                    }

                    if (ResetDefault())
                    {
                        flightTime = EstimateTimeAfterManeuver(Math.Sqrt(mainBody.gravParameter / (parkingAltitude * 1000d + mainBody.Radius)) * (Math.Sqrt(2d * targetOrbit.ApR / (parkingAltitude * 1000d + mainBody.Radius + targetOrbit.ApR)) - 1d) + .01d, currentUT).time;
                        // min delta-V from first half of hohmann transfer, + .01 to make it not NaN (source: https://en.wikipedia.org/wiki/Hohmann_transfer_orbit#Calculation)
                        // it wont always find the actual maximum flight time because we're using ApR instead of targetAltitude, but using targetAltitude can lead to situations where it gives you a flight time that is too high and results in a NaN delta-V
                        // this gives a time that is likely to be viable, but not the maximum. the best way would be to use nextLaunchUT + phaseTime1, but changing the flightTime changes the nextLaunchUT, so its an annoying recursive behavior

                        switch (flightTimeMode)
                        {
                            case 0:
                                flightTime_Adj = flightTime / solarDayLength;
                                break;
                            case 1:
                                flightTime_Adj = flightTime / (60d * 60d);
                                break;
                            case 2:
                                flightTime_Adj = flightTime / 60d;
                                break;
                            case 3:
                                flightTime_Adj = flightTime;
                                break;
                        }
                    }

                    GUILayout.EndHorizontal();

                    MakeNumberEditField("flightTime", ref flightTime_Adj, 0.1d, epsilon, double.MaxValue);
                    if (StateChanged("flightTime", flightTime)) ClearAllCaches();
                    GUILayout.Box(new GUIContent(FormatTime(flightTime), $"{FormatDecimals(flightTime)}s"), GUILayout.MinWidth(100));

                    //Log("CLAYELADDEDDLOGS FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW FIRST WINDOW");
                    //var stopwatch = Stopwatch.StartNew();
                    double nextLaunchUT = GetCachedLaunchTime(launchPos, latitude, longitude, targetInclination, useAltBehavior, 0);
                    double nextLaunchETA = nextLaunchUT - currentUT;
                    //stopwatch.Stop();
                    //Log($"Window 1 Launch Time: {firstLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");
                    //Log("CLAYELADDEDDLOGS SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW SECOND WINDOW");
                    //stopwatch = Stopwatch.StartNew();
                    double extraLaunchUT = GetCachedLaunchTime(launchPos, latitude, longitude, targetInclination, useAltBehavior, extraWindowNumber - 1);
                    double extraLaunchETA = extraLaunchUT - currentUT;
                    //stopwatch.Stop();
                    //Log($"Window 2 Launch Time: {secondLaunchETA}. Completed in {stopwatch.Elapsed.TotalSeconds}s");

                    (double phaseTime0, double phaseAngle0) = GetCachedPhasingTime(launchPos, referenceTime, referenceWindowNumber);

                    (double dV, double trajectoryEccentricity, int errorStateDV) = GetCachedDeltaV(referenceTime + currentUT + phaseTime0, referenceWindowNumber);

                    if (ValueChanged("trajectoryEccentricity", trajectoryEccentricity, 1e-5) || StateChanged("referenceTimeButton", referenceTimeButton)) ClearOrbitDisplay(ref _transferOrbitRenderer);
                    // 1e-5 resets it about every minute or so when using Launch Now

                    bool inSpecialWarp = warpState == 2 || warpState == 3;
                    bool specialWarpActive = warpState == 1 || inSpecialWarp;
                    if (nextLaunchETA >= mainBody.rotationPeriod && PrincipiaInstalled && specialWarpSelected && !inSpecialWarp) warpState = 1;
                    else if (!inSpecialWarp && !specialWarpWait) warpState = 0;

                    GUILayout.Space(5);

                    GUILayout.BeginHorizontal();
                    if (errorStateDV != 0) GUILayout.Label(new GUIContent("<b>!!!</b>", errorStateDV == 1
                        ? "The delta-V is below the minimum possible to reach the target. Try reducing your flight time or increasing your parking altitude."
                        : $"The delta-V is above the maximum allowed for this body ({FormatDecimals(maxDeltaVScaled * Math.Sqrt(mainBody.Mass / mainBody.Radius) / Math.Sqrt(EarthMass / EarthRadius))}). Try increasing maxDeltaVScaled in settings, or increasing your flight time."));
                    GUILayout.Label(new GUIContent("Required \u0394V", "Required change in velocity for the maneuver in parking orbit"), GUILayout.ExpandWidth(true));
                    ExpandCollapse(ref expandAltitude, "Set parking orbit altitude");
                    GUILayout.EndHorizontal();

                    GUILayout.Space(5);
                    GUILayout.Box(new GUIContent($"{FormatDecimals(dV)} m/s", $"Eccentricity of {(!double.IsNaN(trajectoryEccentricity) ? (trajectoryEccentricity > 1 ? "hyperbolic" : "elliptical") : "NaN")} trajectory: {FormatDecimals(trajectoryEccentricity)}"), GUILayout.MinWidth(100));

                    if (expandAltitude)
                    {
                        GUILayout.Label(new GUIContent("Parking Orbit (km)", "Planned altitude of the circular parking orbit before the maneuver"), GUILayout.ExpandWidth(true));
                        MakeNumberEditField("parkingAltitude", ref parkingAltitude, 5d, mainBody.atmosphere ? mainBody.atmosphereDepth / 1000d : epsilon, Math.Max(targetOrbit.PeA / 1000d, mainBody.atmosphereDepth / 1000d)); // PeA updates every frame so we don't need to ask Principia
                        if (StateChanged("parkingAltitude", parkingAltitude))
                        {
                            phasingCache.Clear();
                            deltaVCache.Clear();
                            ClearAllOrbitDisplays();
                            ClearAngleRenderer();
                        }
                    }

                    GUILayout.Space(5);

                    GUILayout.BeginHorizontal();

                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                    if (GUILayout.Button(new GUIContent(referenceTimeLabel, referenceTimeTooltip), GUILayout.Width(100))) referenceTimeButton = (referenceTimeButton + 1) % (expandExtraWindow ? 3 : 2);
                    GUILayout.EndVertical();

                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                    if (GUILayout.Button(new GUIContent(showAzimuth ? "Az." : "In.", showAzimuth ? "Launch to this azimuth to get into the target parking orbit" : "Launch to this inclination to get into the target parking orbit (positive = North, negative = South, regardless of latitude sign)"), GUILayout.Width(25))) showAzimuth = !showAzimuth;
                    GUILayout.EndVertical();

                    ExpandCollapse(ref expandParking0, "Show Orbit Details");
                    GUILayout.EndHorizontal();

                    switch (referenceTimeButton)
                    {
                        case 0:
                            referenceTimeLabel = "Launch Now";
                            referenceTimeTooltip = "Change reference time to Next Launch Window";
                            referenceTime = 0d;
                            referenceWindowNumber = null;
                            break;
                        case 1:
                            referenceTimeLabel = "Next Window";
                            referenceTimeTooltip = $"Change reference time to Launch Window {extraWindowNumber}";
                            referenceTime = nextLaunchETA;
                            referenceWindowNumber = 0;
                            break;
                        case 2:
                            referenceTimeLabel = $"Window {extraWindowNumber}";
                            referenceTimeTooltip = "Change reference time to the Launch Now Window";
                            referenceTime = extraLaunchETA;
                            referenceWindowNumber = extraWindowNumber - 1;
                            break;
                    }

                    OrbitData launchOrbit0 = GetCachedLaunchOrbit(launchPos, referenceTime, referenceWindowNumber);
                    OrbitData launchOrbit1 = GetCachedLaunchOrbit(launchPos, nextLaunchETA, 0);

                    double displayAz = referenceTimeButton == 0 || (isLowLatitude && !useAltBehavior) ? launchOrbit0.azimuth : targetLaunchAzimuth;
                    double displayInc = referenceTimeButton == 0 || (isLowLatitude && !useAltBehavior) ? launchOrbit0.azimuth > 90d && launchOrbit0.azimuth < 270d ? -launchOrbit0.inclination : launchOrbit0.inclination : targetLaunchInclination;
                    // launchOrbit doesnt really display retrograde azimuths/inclinations, so itd be unhelpful to display them if they're misleading

                    GUILayout.Space(5);
                    if (showAzimuth)
                    {
                        GUILayout.Box(new GUIContent($"{FormatDecimals(displayAz)}\u00B0", $"Azimuth\n{FormatDecimals(displayAz * degToRad)} rads, this is {(displayAz <= 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                    }
                    else
                    {
                        GUILayout.Box(new GUIContent($"{FormatDecimals(displayInc)}\u00B0", $"Inclination\n{FormatDecimals(displayInc * degToRad)} rads, this is {(displayAz <= 180d ? "prograde" : "retrograde")}"), GUILayout.MinWidth(100));
                    }

                    if (expandParking0)
                    {
                        (double launchLAN0, double launchAoP0) = GetCachedLAN(latitude, longitude, targetLaunchAzimuth, referenceTime, referenceWindowNumber);
                        ShowOrbitInfo(ref useAngle, ref useLAN, phaseTime0, phaseAngle0, launchLAN0, launchAoP0);
                    }

                    string windowTooltip = (!isLowLatitude || useAltBehavior) && Math.Abs(targetLaunchAzimuth - 90d) < epsilon ?
                        "Launch Easterly at this time to get into the required parking orbit" :
                        $"Launch at this time at this {(showAzimuth ? "azimuth" : "inclination")} to get into the required parking orbit";

                    GUILayout.Space(5);
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Next Window", windowTooltip), GUILayout.ExpandWidth(true));
                    ExpandCollapse(ref expandParking1, "Show Orbit Details");
                    GUILayout.EndHorizontal();

                    GUILayout.Space(5);
                    GUILayout.Box(new GUIContent(FormatTime(nextLaunchETA), $"UT: {nextLaunchUT:0}s"), GUILayout.MinWidth(100)); // the tooltip will flash every second if we just do {nextLaunchETA}, we need absolute time

                    // we need this outside for alarm description and transfer orbit
                    (double phaseTime1, double phaseAngle1) = GetCachedPhasingTime(launchPos, nextLaunchETA, 0);
                    (double launchLAN1, double launchAoP1) = GetCachedLAN(latitude, longitude, targetLaunchAzimuth, nextLaunchETA, 0);
                    if (expandParking1)
                    {
                        ShowOrbitInfo(ref useAngle, ref useLAN, phaseTime1, phaseAngle1, launchLAN1, launchAoP1);
                    }

                    if (expandExtraWindow)
                    {
                        GUILayout.Space(5);
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent($"Window {extraWindowNumber}", "Extra Window: " + windowTooltip), GUILayout.ExpandWidth(true));
                        ExpandCollapse(ref expandParking2, "Show Orbit Details");
                        GUILayout.EndHorizontal();

                        GUILayout.Space(5);
                        GUILayout.Box(new GUIContent(FormatTime(extraLaunchETA), $"UT: {extraLaunchUT:0}s"), GUILayout.MinWidth(100)); // the tooltip will flash every second if we just do {extraLaunchETA}, we need absolute time

                        if (expandParking2)
                        {
                            (double phaseTime2, double phaseAngle2) = GetCachedPhasingTime(launchPos, extraLaunchETA, extraWindowNumber - 1);
                            (double launchLAN2, double launchAoP2) = GetCachedLAN(latitude, longitude, targetLaunchAzimuth, extraLaunchETA, extraWindowNumber - 1);
                            ShowOrbitInfo(ref useAngle, ref useLAN, phaseTime2, phaseAngle2, launchLAN2, launchAoP2);
                        }
                    }

                    GUILayout.Space(5);
                    GUILayout.Label(new GUIContent("Warp Margin (sec)", "The time difference from the launch window that the warp will stop at"), GUILayout.ExpandWidth(true), GUILayout.Width(windowWidth)); // this sets the width of the window
                    MakeNumberEditField("warpMargin", ref warpMargin, 5d, 0d, double.MaxValue);

                    GUILayout.Space(10);
                    GUILayout.BeginHorizontal();
                    bool addAlarm = GUILayout.Button(new GUIContent(" Add Alarm", $"Add Alarm for {(useAltAlarm && expandExtraWindow ? $"Window {extraWindowNumber}" : "Next Window")} to {(KACInstalled && useKAC ? "Kerbal Alarm Clock" : "stock Alarm Clock")}"), GUILayout.MinWidth(80));

                    if (addAlarm)
                    {
                        string title = ""; // might need to check for overspill in title?
                        string description = "";
                        double time = double.NaN;

                        void DescribeD(string identifier, string unit, double value) => description += !double.IsNaN(value) ? $"{identifier}: {FormatDecimals(value)}{unit}\n" : "";
                        void DescribeI(string identifier, string unit, int value) => description += $"{identifier}: {value}{unit}\n";
                        void DescribeB(string identifier, bool value) => description += $"{identifier}: {value}\n";

                        if (!double.IsNaN(nextLaunchUT) && (!useAltAlarm || (useAltAlarm && expandExtraWindow && extraWindowNumber == 1)))
                        {
                            title = $"Next {targetName} Launch Window";
                            time = nextLaunchUT - warpMargin;
                            DescribeI("Window Number", "", 1);
                        }
                        else if (useAltAlarm && expandExtraWindow && !double.IsNaN(extraLaunchUT))
                        {
                            title = $"{targetName} Launch Window {extraWindowNumber}";
                            time = extraLaunchUT - warpMargin;
                            DescribeI("Window Number", "", extraWindowNumber);
                        }

                        DescribeD("UT", "s", time);
                        DescribeD("Warp Margin", "s", warpMargin);
                        DescribeD("Latitude", "\u00B0", latitude);
                        DescribeD("Longitude", "\u00B0", longitude);
                        DescribeD("Flight Time", "s", flightTime);
                        DescribeD("Required delta-V", "m/s", dV);
                        DescribeD("Parking Orbit Altitude", "km", parkingAltitude);
                        DescribeD("Launch Inclination", "\u00B0", (isLowLatitude && !useAltBehavior) ? launchOrbit1.azimuth > 90d && launchOrbit1.azimuth < 270d ? -launchOrbit1.inclination : launchOrbit1.inclination : targetLaunchInclination);
                        DescribeD("Launch Azimuth", "\u00B0", (isLowLatitude && !useAltBehavior) ? launchOrbit1.azimuth : targetLaunchAzimuth);
                        DescribeD("Phasing Time", "s", phaseTime1);
                        DescribeD("Phasing Angle", "\u00B0", phaseAngle1);
                        DescribeD("Parking Orbit LAN", "\u00B0", launchLAN1);
                        DescribeD("Parking Orbit AoP", "\u00B0", launchAoP1);
                        DescribeB("Special Warp Active", specialWarpActive);

                        description = description.TrimEnd('\n');

                        if (title != "" && time != double.NaN)
                        {
                            // alarm type should be raw because this isn't really a maneuver or transfer alarm, its a launch time alarm
                            if (KACInstalled && useKAC)
                            {
                                string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, title, time);
                                if (!string.IsNullOrEmpty(alarmId))
                                {
                                    // if the alarm was made, get the object
                                    KACWrapper.KACAPI.KACAlarm alarm = KACWrapper.KAC.Alarms.First(z => z.ID == alarmId);

                                    alarm.AlarmAction = KACWrapper.KACAPI.AlarmActionEnum.KillWarp;
                                    //alarm.AlarmMargin = warpMargin; // this doesnt seem to work, so we need to put warpMargin into the time
                                    alarm.Notes = description;
                                }
                            }
                            else
                            {
                                AlarmTypeRaw alarm = new AlarmTypeRaw
                                {
                                    title = title,
                                    description = description,
                                    ut = time,
                                    //eventOffset = warpMargin, // this doesnt seem to work, so we need to put warpMargin into the time
                                };
                                AlarmClockScenario.AddAlarm(alarm);
                                alarm.actions.warp = AlarmActions.WarpEnum.KillWarp;
                            }
                        }
                    }

                    bool inWarp() => TimeWarp.CurrentRate > 1d;

                    bool toggleWarp = GUILayout.Button(new GUIContent(inWarp() || inSpecialWarp ? "Stop Warp" : "Warp", "Warp to the Next Window, taking into account the Warp Margin"), GUILayout.MinWidth(80));
                    GUILayout.EndHorizontal();

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
                        ResetWindow();
                    }

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

                    bool MapViewEnabled() => MapView.MapIsEnabled && !HighLogic.LoadedSceneIsEditor && (HighLogic.LoadedSceneIsFlight || HighLogic.LoadedScene == GameScenes.TRACKSTATION);

                    GUILayout.BeginHorizontal();
                    bool targetPressed;
                    bool focusPressed;
                    if (MapViewEnabled() && !targetManual) // if in map view, map view carries over to editor sometimes so just double-check
                    {
                        bool enabled = InputLockManager.IsUnlocked(ControlTypes.TARGETING) && HighLogic.LoadedSceneIsFlight; // ControlTypes.TARGETING will be 'locked' when the game isnt in focus or paused, but its not actually locked. not sure how i can fix this, its just visual tho
                        GUI.enabled = enabled;
                        targetPressed = GUILayout.Button(new GUIContent(targetSet ? "Unset" : "Target", $"{$"Target {targetName}{(enabled ? "" : "\nTarget Switching is Locked, must be in flight")}"}"), GUILayout.MinWidth(70));
                        GUI.enabled = true;
                        focusPressed = GUILayout.Button(new GUIContent("Focus", $"Focus {targetName}"), GUILayout.MinWidth(70));
                    }
                    else
                    {
                        bool enabled = InputLockManager.IsUnlocked(ControlTypes.TARGETING) && HighLogic.LoadedSceneIsFlight; // ControlTypes.TARGETING will be 'locked' when the game isnt in focus or paused, but its not actually locked. not sure how i can fix this, its just visual tho
                        GUI.enabled = enabled && !targetManual;
                        targetPressed = GUILayout.Button(new GUIContent(targetManual ? "[Manual Target]" : (targetSet ? "Unset Target" : TextOverspill($"Target {targetName}", 140, GUI.skin.button)), $"{(targetManual ? "Manual Target Mode:\nTargeting Disabled" : $"Target {targetName}{(enabled ? "" : "\nTarget Switching is Locked, must be in flight")}")}"), GUILayout.MinWidth(140));
                        GUI.enabled = true;
                        focusPressed = false; // not in map view, so cant focus
                    }
                    ShowSettings();
                    GUILayout.EndHorizontal();

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

                    if (focusPressed)
                    {
                        if (target is Vessel vesselTarget)
                            PlanetariumCamera.fetch.SetTarget(vesselTarget.mapObject);
                        else if (target is CelestialBody bodyTarget)
                            PlanetariumCamera.fetch.SetTarget(bodyTarget.MapObject);
                        else LogError("Unknown target type passed to focusSet: " + target.GetType().Name);

                        // lol, who put Vessel as mapObject and CelestialBody as MapObject?
                    }

                    if (MapViewEnabled())
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Show Parking Orbit", $"Show Parking Orbit for the Next Launch Window in Map View"));
                        GUILayout.FlexibleSpace();
                        BeginCenter();
                        displayParking = GUILayout.Toggle(displayParking, "");
                        EndCenter();
                        GUILayout.EndHorizontal();

                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Show Transfer Orbit", $"Show Transfer Orbit for the Next Launch Window in Map View{(double.IsNaN(trajectoryEccentricity) || double.IsNaN(dV) ? "\nError: Trajectory is invalid" : "")}"));
                        GUILayout.FlexibleSpace();
                        BeginCenter();
                        displayTransfer = GUILayout.Toggle(displayTransfer, "");
                        EndCenter();
                        GUILayout.EndHorizontal();

                        if (targetManual)
                        {
                            GUILayout.BeginHorizontal();
                            GUILayout.Label(new GUIContent("Show Manual Orbit", "Show Manual Target Orbit in Map View"));
                            GUILayout.FlexibleSpace();
                            BeginCenter();
                            displayManual = GUILayout.Toggle(displayManual, "");
                            EndCenter();
                            GUILayout.EndHorizontal();
                        }

                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Show Phasing Angle", "Show Phasing Angle for the Next Launch Window in Map View"));
                        GUILayout.FlexibleSpace();
                        BeginCenter();
                        GUI.enabled = displayParking;
                        displayPhasing = GUILayout.Toggle(displayPhasing, new GUIContent("", displayParking ? "" : "The Parking Orbit needs to be enabled for this to be shown"));
                        GUI.enabled = true;
                        EndCenter();
                        GUILayout.EndHorizontal();
                    }

                    if (StateChanged("RendererButtons", MapViewEnabled()))
                    {
                        ResetWindow();
                        ClearAllOrbitDisplays();
                        ClearAngleRenderer();
                    }

                    if (double.IsNaN(targetLaunchInclination)) // need to update if NaN and showSettings isnt open to update it (in case of a latitude change)
                    {
                        ResetLaunchInclination();

                        if (StateChanged("targetLaunchInclination", targetLaunchInclination)) ClearAllCaches();
                    }

                    if (displayParking && MapViewEnabled() && !needCacheClear)
                    {
                        Orbit parkingOrbit = new Orbit
                        {
                            inclination = (isLowLatitude && !useAltBehavior) ? launchOrbit1.inclination : targetLaunchInclination,
                            eccentricity = epsilon, // just to make periapsis visible
                            semiMajorAxis = mainBody.Radius + (parkingAltitude * 1000d),
                            LAN = launchLAN1,
                            argumentOfPeriapsis = launchAoP1,
                            meanAnomalyAtEpoch = 0d,
                            epoch = currentUT,
                            referenceBody = mainBody,
                        };

                        //Log($"launchLAN1: {launchLAN1}, launchAoP1: {launchAoP1}");

                        if (_parkingOrbitRenderer == null)
                        {
                            _parkingOrbitRenderer = OrbitRendererHack.Setup(parkingOrbit, parkingColor);
                        }

                        if (displayPhasing && (_phasingAngleRenderer == null || !_phasingAngleRenderer.IsDrawing))
                        {
                            // we need to do both of these to access the orbit.pos and orbit.vel in Draw (we're not running them every frame, so its fine)
                            parkingOrbit.Init();
                            parkingOrbit.UpdateFromUT(currentUT);

                            ClearAngleRenderer();

                            if (_phasingAngleRenderer == null)
                            {
                                _phasingAngleRenderer = MapView.MapCamera.gameObject.AddComponent<MapAngleRenderer>();
                            }

                            double AoPmodified = (targetLaunchAzimuth > 90d && targetLaunchAzimuth < 180d) ? Util.ClampAngle(360d - launchAoP1, false) : 180d - launchAoP1;
                            _phasingAngleRenderer.Draw(parkingOrbit, AoPmodified, phaseAngle1);

                            //Log($"AoPmodified: {AoPmodified}, phaseAngle1: {phaseAngle1}, parkingOrbit: {parkingOrbit}");
                        }
                    }
                    else if (!displayParking && _parkingOrbitRenderer != null)
                    {
                        ClearOrbitDisplay(ref _parkingOrbitRenderer);
                    }

                    if ((!displayParking || !displayPhasing) && _phasingAngleRenderer != null && !_phasingAngleRenderer.IsHiding)
                    {
                        ClearAngleRenderer();
                    }

                    //Log($"trajectoryEccentricity: {trajectoryEccentricity}, errorStateDV: {errorStateDV}");

                    if (displayTransfer && _transferOrbitRenderer == null && MapViewEnabled() && !needCacheClear)
                    {
                        double phaseAoPmodified = targetLaunchAzimuth >= 180d && targetLaunchAzimuth < 270d ? Util.ClampAngle(launchAoP1 + phaseAngle1 + 180d, false) : Util.ClampAngle(launchAoP1 + phaseAngle1, false);
                        Orbit transferOrbit = new Orbit
                        {
                            inclination = (isLowLatitude && !useAltBehavior) ? launchOrbit1.inclination : targetLaunchInclination,
                            eccentricity = double.IsNaN(trajectoryEccentricity) || double.IsNaN(dV) ? double.NaN : trajectoryEccentricity, // dont display transfer orbit if NaN
                            semiMajorAxis = (mainBody.Radius + parkingAltitude * 1000d) / (1 - trajectoryEccentricity),
                            LAN = launchLAN1,
                            argumentOfPeriapsis = phaseAoPmodified,
                            meanAnomalyAtEpoch = 0d,
                            epoch = currentUT,
                            referenceBody = mainBody,
                        };

                        //Log($"transferOrbit.eccentricity: {transferOrbit.eccentricity}");

                        _transferOrbitRenderer = OrbitRendererHack.Setup(transferOrbit, transferColor);
                    }
                    else if (!displayTransfer && _transferOrbitRenderer != null)
                    {
                        ClearOrbitDisplay(ref _transferOrbitRenderer);
                    }

                    if (displayManual && _manualOrbitRenderer == null && MapViewEnabled() && targetManual && !needCacheClear)
                    {
                        _manualOrbitRenderer = OrbitRendererHack.Setup(targetOrbit, manualColor);
                    }
                    else if ((!displayManual || !targetManual) && _manualOrbitRenderer != null)
                    {
                        ClearOrbitDisplay(ref _manualOrbitRenderer);
                    }
                }
            }
            finally
            {
                Tooltip.Instance?.RecordTooltip(id);
                GUI.DragWindow();
                ResetWindow(ref needMainReset, ref mainRect);
            }
        }

        #endregion
        #region MakeSettingsWindow

        private void MakeSettingsWindow(int id)
        {
            windowWidth = 500;
            windowState = WindowState.Settings;

            void BeginCombined() => GUILayout.BeginHorizontal();

            void MiddleCombined(bool useAlternate = false)
            {
                if (!useAlternate)
                {
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                }
                else
                {
                    GUILayout.BeginVertical();
                    GUILayout.Space(5);
                }
            }

            void EndCombined(bool useAlternate = false)
            {
                if (!useAlternate)
                {
                    EndCenter();
                    GUILayout.EndHorizontal();
                }
                else
                {
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();
                }
            }

            GUILayout.Space(5);
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent($"Hover over select text for tooltips. Current UT: <b>{FormatDecimals(currentUT)}</b>s", FormatTime(currentUT)), GUILayout.Width(windowWidth - 50)); // this sets the width of the window
            // this tooltip is really only useful when paused, it flashes too quickly to be seen otherwise
            GUILayout.BeginVertical();
            GUILayout.Space(5);
            if (GUILayout.Button(new GUIContent("Reset","Reset all windows and caches"), GUILayout.Width(50)))
            {
                ResetWindow(WindowState.Main);
                ResetWindow();
                if (showManualOrbit && targetManual) ResetWindow(WindowState.ManualOrbit);

                ClearAllCaches();
            }
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            settingsScroll = GUILayout.BeginScrollView(settingsScroll, GUILayout.Width(windowWidth), GUILayout.Height(500));

            try
            {
                GUILayout.Space(10);

                BeginCombined();
                GUILayout.Label("Use Unity Skin");
                MiddleCombined();
                useAltSkin = GUILayout.Toggle(useAltSkin, "");
                EndCombined();

                if (StateChanged("useAltSkin", useAltSkin))
                {
                    ResetWindow(WindowState.Main);
                    ResetWindow();
                    if (showManualOrbit && targetManual) ResetWindow(WindowState.ManualOrbit);
                }

                if (errorStateTargets != 4)
                {
                    DrawLine();

                    BeginCombined();
                    GUILayout.Label("Select an orbit to target manually");
                    MiddleCombined();
                    targetManual = GUILayout.Toggle(targetManual, "");
                    EndCombined();

                    if (StateChanged("targetManual", targetManual))
                    {
                        ResetWindow(WindowState.Main);
                        //ResetWindow(ref settingsRect); // TODO, for some reason this isnt working with errorStateTargets != 0 // this isnt needed with the scroll bar
                        target = null;
                        targetName = "";

                        double radiusAdjusted = useCenterDistance ? 0d : mainBody.Radius;
                        manualApR = manualSMA * (1d + manualEccentricity);
                        ApA_Adj = (manualApR - radiusAdjusted) / 1000d;
                        manualPeR = manualSMA * (1d - manualEccentricity);
                        PeA_Adj = (manualPeR - radiusAdjusted) / 1000d;
                        manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / mainBody.gravParameter);
                        period_Adj = manualPeriod;

                        // set the orbit to current parameters initially
                        targetOrbit = new Orbit
                        {
                            eccentricity = manualEccentricity,
                            semiMajorAxis = manualSMA,
                            inclination = manualInclination,
                            LAN = manualLAN,
                            argumentOfPeriapsis = manualAoP,
                            meanAnomalyAtEpoch = manualMNA,
                            epoch = currentUT,
                            referenceBody = mainBody,
                        };
                        targetOrbit.Init(); // do NOT use SetOrbit, it causes the previous target's orbit to be changed
                        targetOrbit.UpdateFromUT(currentUT);
                        _ = StateChanged(true, "manualOrbitStates", manualEccentricity, manualSMA, manualInclination, manualLAN, manualAoP, manualMNA, mainBody); // update cached values to current
                    }
                }

                if ((errorStateTargets == 0 || errorStateTargets == 2 || errorStateTargets == 3) && !targetManual)
                {
                    DrawLine();

                    if (errorStateTargets == 2 || errorStateTargets == 3) GUILayout.Label("<b><i>TOGGLE THIS TO GET OUT OF ERROR</i></b>");
                    BeginCombined();
                    GUILayout.Label("Target an orbiting Vessel instead of an orbiting Moon");
                    MiddleCombined();
                    targetVessel = GUILayout.Toggle(targetVessel, "");
                    EndCombined();

                    if (StateChanged("targetVessel", targetVessel))
                    {
                        ResetWindow(WindowState.Main);
                        //ResetWindow(ref settingsRect); // this isnt needed with the scroll bar
                        target = null;
                        targetName = "";
                    }
                }

                if (errorStateTargets == 0)
                {
                    DrawLine();

                    BeginCombined();
                    GUILayout.Label("Display raw seconds instead of time formatted into days, hours, minutes, and seconds");
                    MiddleCombined();
                    displaySeconds = GUILayout.Toggle(displaySeconds, "");
                    EndCombined();

                    DrawLine();

                    BeginCombined();
                    GUILayout.Label(new GUIContent($"Find the Global Minimum of the {(showAzimuth ? "azimuth" : "inclination")} error instead of the Local Minimum of the {(showAzimuth ? "azimuth" : "inclination")} error", $"Ignored if latitude is higher than {targetName} inclination ({FormatDecimals(targetInclination)}\u00B0)"));
                    MiddleCombined();
                    GUI.enabled = isLowLatitude;
                    useAltBehavior = GUILayout.Toggle(useAltBehavior, "");
                    GUI.enabled = true;
                    EndCombined();

                    if (StateChanged("useAltBehavior", useAltBehavior))
                    {
                        ClearAllCaches(); // remove local mins so that we can replace them with global mins
                        // technically this only needs to be done when switching from false to true, but switching from true to false without clearing would result in some extra data in the cache, which might lead to problems if abused
                    }

                    DrawLine();

                    BeginCombined();
                    GUILayout.Label(new GUIContent("Use surface vessel position for latitude/longitude instead of launch site position", $"Ignored if not in a {(requireSurfaceVessel ? "surface " : "")}vessel"));
                    MiddleCombined();
                    GUI.enabled = inVessel;
                    useVesselPosition = GUILayout.Toggle(useVesselPosition, "");
                    GUI.enabled = true;
                    EndCombined();

                    if (KACInstalled)
                    {
                        DrawLine();

                        BeginCombined();
                        GUILayout.Label("Use Kerbal Alarm Clock instead of the stock Alarm Clock");
                        MiddleCombined();
                        useKAC = GUILayout.Toggle(useKAC, "");
                        EndCombined();
                    }

                    if (PrincipiaInstalled)
                    {
                        DrawLine();

                        BeginCombined();
                        GUILayout.Label(new GUIContent("<b>Special Warp</b>: Change the \"Warp\" button to use 3 warps to avoid overshooting/undershooting the launch window due to perturbations of the target's orbit. It CANNOT be halted once started.", "Only visible when Principia is installed, and only activates when the next window is more than 1 sidereal day away"));
                        MiddleCombined();
                        specialWarpSelected = GUILayout.Toggle(specialWarpSelected, "");
                        EndCombined();
                    }

                    // TODO, activate special warp if altitude of target changes significantly?

                    DrawLine();

                    BeginCombined();
                    GUILayout.Label("Show Extra Window");
                    MiddleCombined();
                    expandExtraWindow = GUILayout.Toggle(expandExtraWindow, "");
                    EndCombined();

                    if (StateChanged("expandExtraWindow", expandExtraWindow))
                    {
                        ResetWindow(WindowState.Main);
                        //ResetWindow(ref settingsRect); // this isnt needed with the scroll bar
                    }

                    if (expandExtraWindow)
                    {
                        DrawLine();

                        BeginCombined();
                        GUILayout.Label("Change the \"Add Alarm\" button to set an alarm based on the extra launch window instead of the next launch window");
                        MiddleCombined();
                        useAltAlarm = GUILayout.Toggle(useAltAlarm, "");
                        EndCombined();

                        DrawLine();

                        BeginCombined();
                        GUILayout.Label($"Optimize for a certain {(useAngle ? "phasing angle" : "phasing time")} in orbit instead of choosing a window number manually");
                        MiddleCombined();
                        useWindowOptimizer = GUILayout.Toggle(useWindowOptimizer, "");
                        EndCombined();

                        //if (StateChanged("useWindowOptimizer", useWindowOptimizer))
                        //{
                        //    ResetWindow(ref settingsRect); // this isnt needed with the scroll bar
                        //}
                    }

                    if (expandExtraWindow)
                    {
                        DrawLine();

                        if (!useWindowOptimizer)
                        {
                            GUILayout.Label("Extra Window Number");
                            MakeNumberEditField("extraWindowNumber", ref extraWindowNumber, 1, 1, maxWindows);
                            ranSearch = false; // this is to remove the label
                        }
                        else
                        {
                            // we could set extra window number to 2, but theres really no reason to not just keep the initial value until they hit search

                            double orbitRadius = mainBody.Radius + parkingAltitude * 1000d;
                            double orbitPeriod = tau * Math.Sqrt(Math.Pow(orbitRadius, 3) / mainBody.gravParameter);

                            if (double.IsNaN(targetPhasingTime)) targetPhasingTime = targetPhasingAngle / 360d * orbitPeriod; // initialize value

                            if (useAngle)
                            {
                                BeginCombined();
                                GUILayout.Label("Target Phasing Angle (degrees)");
                                MiddleCombined(true);
                                if (GUILayout.Button(new GUIContent("Time", "Switch to phasing time"), GUILayout.Width(60))) useAngle = !useAngle;
                                EndCombined(true);

                                MakeNumberEditField("targetPhasingAngle", ref targetPhasingAngle, 1d, 0.01, 360d, true); // min of 0.01 degrees to avoid division by zero

                                targetPhasingTime = targetPhasingAngle / 360d * orbitPeriod;

                                BeginCombined();
                                GUILayout.Label(new GUIContent("Target Phasing Time (seconds)", $"Max of {FormatDecimals(orbitPeriod)} seconds (the orbit period)"));
                                MiddleCombined(true);
                                GUILayout.Box(new GUIContent(FormatTime(targetPhasingTime), $"{targetPhasingTime}s"), GUILayout.MaxWidth(100));
                                EndCombined(true);
                            }
                            else
                            {
                                BeginCombined();
                                GUILayout.Label(new GUIContent("Target Phasing Time (seconds)", $"Max of {FormatDecimals(orbitPeriod)} seconds (the orbit period)"));
                                MiddleCombined(true);
                                if (GUILayout.Button(new GUIContent("Angle", "Switch to phasing angle"), GUILayout.Width(60))) useAngle = !useAngle;
                                EndCombined(true);

                                GUILayout.BeginHorizontal();
                                MakeNumberEditField("targetPhasingTime", ref targetPhasingTime, 60d, 1d, orbitPeriod); // min of 1 second to avoid division by zero
                                GUILayout.Box(new GUIContent(FormatTime(targetPhasingTime), $"{targetPhasingTime}s"), GUILayout.Width(150));
                                GUILayout.FlexibleSpace();
                                GUILayout.EndHorizontal();

                                targetPhasingAngle = targetPhasingTime / orbitPeriod * 360d;

                                BeginCombined();
                                GUILayout.Label("Target Phasing Angle (degrees)");
                                MiddleCombined(true);
                                GUILayout.Box(new GUIContent($"{FormatDecimals(targetPhasingAngle)}\u00B0", $"{targetPhasingAngle}\u00B0"), GUILayout.MaxWidth(100));
                                EndCombined(true);
                            }

                            GUILayout.BeginHorizontal();
                            if (GUILayout.Button("Search for Closest Window", GUILayout.Width(200)))
                            {
                                ranSearch = true;
                                int bestWindow = -1;
                                double bestError = double.MaxValue;
                                for (int candidateWindow = 0; candidateWindow <= maxWindows - 1; candidateWindow++)
                                {
                                    double candidateLaunchTime = GetCachedLaunchTime(launchPos, latitude, longitude, targetInclination, useAltBehavior, candidateWindow) - currentUT;

                                    if (double.IsNaN(candidateLaunchTime))
                                    {
                                        Log("A Launchtime was NaN, skipping this window"); // GetCachedLaunchTime should remove NaN launchtimes, so this shouldn't happen
                                        continue;
                                    }

                                    if (useAngle)
                                    {
                                        (_, double candidatePhaseAngle) = GetCachedPhasingTime(launchPos, candidateLaunchTime, candidateWindow);
                                        double errorRatio = Math.Abs(candidatePhaseAngle - targetPhasingAngle) / targetPhasingAngle;
                                        if (errorRatio < bestError)
                                        {
                                            bestError = errorRatio;
                                            bestWindow = candidateWindow;
                                            bestAngle = candidatePhaseAngle;
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
                                            bestTime = candidatePhaseTime;
                                        }
                                    }
                                }
                                if (bestWindow >= 0) extraWindowNumber = bestWindow + 1;
                            }
                            if (ranSearch)
                            {
                                GUILayout.Label(new GUIContent($"Found window {extraWindowNumber} with {(useAngle ? $"angle {FormatDecimals(bestAngle)}\u00B0" : $"time {FormatTime(bestTime)}")}!", $"Window {extraWindowNumber} is the closest window to your target phasing {(useAngle ? $"angle of {targetPhasingAngle}\u00B0" : $"time of {targetPhasingTime}s")} within the max of {maxWindows} windows"));
                            }
                            GUILayout.EndHorizontal();
                        }
                    }
                    else ranSearch = false; // this is to remove the label

                    DrawLine();

                    GUILayout.Label(new GUIContent("Shown Decimal Places of Precision", "This is only visual, and editable text fields will not be effected"));
                    MakeNumberEditField("decimals", ref decimals, 1, 0, int.MaxValue);

                    if (StateChanged("decimals", decimals))
                    {
                        ResetWindow(WindowState.Main);
                        //ResetWindow(ref settingsRect); // this isnt needed with the scroll bar
                    }

                    DrawLine();

                    // azimuth to inclination formulas derived from https://www.astronomicalreturns.com/p/section-46-interesting-orbital.html
                    // sin(azimuth) = cos(inclination) / cos(latitude)
                    // cos(inclination) = cos(latitude) * sin(azimuth)

                    string azimuthTooltip = "90° is the default, which is directly east. Range is 0° to 360°, where 0° and 180° are North and South respectively.";
                    string inclinationTooltip = $"Your latitude of {FormatDecimals(latitude)}\u00B0 is the default, which is directly east. Range is -180\u00B0 to 180\u00B0, where +90\u00B0 and -90\u00B0 are North and South respectively.";

                    if (showAzimuth)
                    {
                        ResetLaunchInclination(); // continuously update value

                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Target Launch Azimuth", azimuthTooltip + " Changing the Target Launch Azimuth may not change the launch window time, this is normal and expected."));
                        ResetDefault(ref targetLaunchAzimuth, 90d);
                        GUILayout.EndHorizontal();

                        MakeNumberEditField("targetLaunchAzimuth", ref targetLaunchAzimuth, 1d, 0d, 360d, true);

                        BeginCombined();
                        GUILayout.Label(new GUIContent("Target Launch Inclination", inclinationTooltip));
                        MiddleCombined(true);
                        GUILayout.Box(new GUIContent($"{FormatDecimals(targetLaunchInclination)}\u00B0", $"{targetLaunchInclination}\u00B0"), GUILayout.MaxWidth(100));
                        EndCombined(true);
                    }
                    else
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Target Launch Inclination", inclinationTooltip + " Changing the Target Launch Inclination may not change the launch window time, this is normal and expected."));
                        ResetDefault(ref targetLaunchInclination, latitude);
                        GUILayout.EndHorizontal();

                        GUILayout.BeginHorizontal();
                        MakeNumberEditField("targetLaunchInclination", ref targetLaunchInclination, 1d, -180d, 180d, true);

                        double cosInc = Math.Cos(targetLaunchInclination * degToRad);
                        double cosLat = Math.Cos(latitude * degToRad);
                        double sinAz = cosInc / cosLat;

                        if (Math.Abs(sinAz) >= 1d - 1e-9)
                        {
                            GUILayout.FlexibleSpace();
                            GUILayout.Label(new GUIContent("Unreachable", $"The Target Inclination of {FormatDecimals(targetLaunchInclination)}\u00B0 is unreachable from your latitude of {FormatDecimals(latitude)}°, so it has been automatically converted to the nearest reachable inclination."));
                            GUILayout.FlexibleSpace();
                        }
                        GUILayout.EndHorizontal();

                        sinAz = Util.Clamp(sinAz, -1d, 1d);
                        double azInter = Math.Abs(Math.Asin(sinAz) * radToDeg); // intermediate value for azimuth

                        targetLaunchAzimuth = targetLaunchInclination >= 0
                            ? (targetLaunchInclination <= 90d ? azInter : 360d - azInter) // NE (prograde) or NW (retrograde)
                            : (Math.Abs(targetLaunchInclination) <= 90d ? 180d - azInter : 180d + azInter); // SE (prograde) or SW (retrograde)

                        targetLaunchAzimuth = Util.ClampAngle(targetLaunchAzimuth, false);

                        BeginCombined();
                        GUILayout.Label(new GUIContent("Target Launch Azimuth", azimuthTooltip));
                        MiddleCombined(true);
                        GUILayout.Box(new GUIContent($"{FormatDecimals(targetLaunchAzimuth)}\u00B0", $"{targetLaunchAzimuth}\u00B0"), GUILayout.MaxWidth(100));
                        EndCombined(true);
                    }
                    // its not possible to have both textfields on screen at once, bugs out

                    if (StateChanged("targetLaunchAzimuth", targetLaunchAzimuth))
                    {
                        ClearAllCaches(); // this doesn't always result in new minimums, intentional
                    }
                }
            }
            finally
            {
                GUILayout.EndScrollView(); // prevent GUIClips error by using finally
                Tooltip.Instance?.RecordTooltip(id);
                GUI.DragWindow();
                ResetWindow(ref needSettingsReset, ref settingsRect);
            }
        }

        #endregion
        #region MakeManualOrbitWindow

        private void MakeManualOrbitWindow(int id)
        {
            windowWidth = 500;
            windowState = WindowState.ManualOrbit;
            const double epsilon = 1e-9;

            try
            {
                double radius = mainBody.Radius;
                double radiusScaled = radius / 1000d;
                double atmosphereDepth = mainBody.atmosphere ? mainBody.atmosphereDepth : epsilon;
                double atmosphereDepthScaled = atmosphereDepth / 1000d;
                double SoI = mainBody.sphereOfInfluence;
                double gravParameter = mainBody.gravParameter;
                CelestialBody homeBody = FlightGlobals.GetHomeBody();

                GUILayout.Space(5);
                GUILayout.Label("Hover over select text for tooltips", GUILayout.Width(windowWidth)); // this sets the width of the window

                if (errorStateTargets != 4)
                {
                    DrawLine();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Use radians instead of degrees");
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                    useRadians = GUILayout.Toggle(useRadians, "");
                    EndCenter();
                    GUILayout.EndHorizontal();

                    if (StateChanged("useRadians", useRadians))
                    {
                        if (useRadians)
                        {
                            inclination_Adj *= degToRad;
                            LAN_Adj *= degToRad;
                            AoP_Adj *= degToRad;
                            MNA_Adj *= degToRad;
                        }
                        else
                        {
                            inclination_Adj *= radToDeg;
                            LAN_Adj *= radToDeg;
                            AoP_Adj *= radToDeg;
                            MNA_Adj *= radToDeg;
                        }
                    }

                    DrawLine();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent($"Define heights from the center of {mainBody.bodyName} instead of sea level", "Only used for specifying Apoapsis and Periapsis"));
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                    useCenterDistance = GUILayout.Toggle(useCenterDistance, "");
                    EndCenter();
                    GUILayout.EndHorizontal();

                    // no one defines SMA from sea level, so we don't need to include it in here

                    if (StateChanged("useCenterDistance", useCenterDistance))
                    {
                        if (useCenterDistance)
                        {
                            ApA_Adj += radius / 1000d;
                            PeA_Adj += radius / 1000d;
                        }
                        else
                        {
                            ApA_Adj -= radius / 1000d;
                            PeA_Adj -= radius / 1000d;
                        }
                    }

                    DrawLine();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Switch between manual target modes");
                    GUILayout.FlexibleSpace();
                    BeginCenter();
                    if (GUILayout.Button(new GUIContent(manualModeLabel, manualModeTooltip), GUILayout.Width(120))) manualTargetMode = (manualTargetMode + 1) % 9;
                    EndCenter();
                    GUILayout.EndHorizontal();

                    DrawLine();

                    GUILayout.FlexibleSpace();

                    double max = useRadians ? tau : 360d;
                    double step = useRadians ? Math.PI / 12d : 1d; // jump by 15 degree increments if using radians
                    double radiusAdjusted = useCenterDistance ? 0d : radius;
                    string textAdjusted = useCenterDistance ? $"the center of {mainBody.bodyName}" : "sea level";

                    // TODO, fix the various min/max bugs edit fields in the modes below

                    void EditApoapsis()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Apoapsis (km)", $"In kilometers from {textAdjusted}"));
                        GUILayout.FlexibleSpace();
                        if (manualTargetMode == 0) MakeNumberEditField("apoapsis", ref ApA_Adj, 1d, Util.Max(epsilon, (manualPeR - radiusAdjusted) / 1000d, parkingAltitude), SoI);
                        else MakeNumberEditField("apoapsis", ref ApA_Adj, 1d, Util.Max(epsilon, (manualPeR - radiusAdjusted) / 1000d, (radiusScaled + atmosphereDepthScaled) * (1d + manualEccentricity) / (1d - manualEccentricity) - radiusScaled, parkingAltitude), SoI);
                        GUILayout.EndHorizontal();

                        manualApR = (ApA_Adj * 1000d) + radiusAdjusted;
                    }

                    void DisplayApoapsis()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Apoapsis (km)", $"In kilometers from {textAdjusted}"));
                        GUILayout.FlexibleSpace();
                        GUILayout.Box(new GUIContent(FormatDecimals((manualApR - radiusAdjusted) / 1000d) + "km", $"{manualApR - radiusAdjusted}m"), GUILayout.MinWidth(150));
                        GUILayout.EndHorizontal();
                    }

                    void ResetApoapsis(bool fullReset = false)
                    {
                        if (fullReset) manualApR = manualSMA * (1d + manualEccentricity);
                        ApA_Adj = (manualApR - radiusAdjusted) / 1000d;
                    }

                    void EditPeriapsis()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Periapsis (km)", $"In kilometers from {textAdjusted}"));
                        GUILayout.FlexibleSpace();
                        MakeNumberEditField("periapsis", ref PeA_Adj, 1d, Math.Max(atmosphereDepthScaled + (useCenterDistance ? radius / 1000d : 0d), parkingAltitude), (manualApR - radiusAdjusted) / 1000d);
                        GUILayout.EndHorizontal();

                        manualPeR = (PeA_Adj * 1000d) + radiusAdjusted;
                    }

                    void DisplayPeriapsis()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Periapsis (km)", $"In kilometers from {textAdjusted}"));
                        GUILayout.FlexibleSpace();
                        GUILayout.Box(new GUIContent(FormatDecimals((manualPeR - radiusAdjusted) / 1000d) + "km", $"{manualPeR - radiusAdjusted}m"), GUILayout.MinWidth(150));
                        GUILayout.EndHorizontal();
                    }

                    void ResetPeriapsis(bool fullReset = false)
                    {
                        if (fullReset) manualPeR = manualSMA * (1d - manualEccentricity);
                        PeA_Adj = (manualPeR - radiusAdjusted) / 1000d;
                    }

                    void EditPeriod()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Period (s)", $"Formatted using {(useHomeSolarDay ? homeBody.bodyName : mainBody.bodyName)} solar days"));
                        GUILayout.FlexibleSpace();
                        if (manualTargetMode == 6) MakeNumberEditField("period", ref period_Adj, 1d, epsilon, tau * Math.Sqrt(Math.Pow(SoI, 3) / gravParameter));
                        else MakeNumberEditField("period", ref period_Adj, 1d, Math.Max(epsilon, tau * Math.Sqrt(Math.Pow((radius + atmosphereDepth) / (1d - manualEccentricity), 3) / gravParameter)), tau * Math.Sqrt(Math.Pow(SoI, 3) / gravParameter));
                        GUILayout.EndHorizontal();

                        manualPeriod = period_Adj;
                    }

                    void DisplayPeriod()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Period (s)", $"Formatted using {(useHomeSolarDay ? homeBody.bodyName : mainBody.bodyName)} solar days"));
                        GUILayout.FlexibleSpace();
                        GUILayout.Box(new GUIContent(FormatTime(manualPeriod), $"{manualPeriod}s"), GUILayout.MinWidth(150)); ;
                        GUILayout.EndHorizontal();
                    }

                    void ResetPeriod(bool fullReset = false)
                    {
                        if (fullReset) manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);
                        period_Adj = manualPeriod;
                    }

                    void EditEccentricity()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Eccentricity", "Ranges from 0 to 1"));
                        GUILayout.FlexibleSpace();
                        if (manualTargetMode == 4) MakeNumberEditField("eccentricity", ref eccentricity_Adj, 0.1d, epsilon, 1d - epsilon); // eccentricity needs to be below 1
                        else MakeNumberEditField("eccentricity", ref eccentricity_Adj, 0.1d, epsilon, Math.Min(1d - epsilon, 1d - (radius + atmosphereDepth) / manualSMA)); // eccentricity needs to be below 1, make sure periapsis doesnt go below body
                        GUILayout.EndHorizontal();

                        manualEccentricity = eccentricity_Adj;
                    }

                    void DisplayEccentricity()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Eccentricity", "Ranges from 0 to 1"));
                        GUILayout.FlexibleSpace();
                        GUILayout.Box(new GUIContent(FormatDecimals(manualEccentricity), $"{manualEccentricity}"), GUILayout.MinWidth(150));
                        GUILayout.EndHorizontal();
                    }

                    void ResetEccentricity() => eccentricity_Adj = manualEccentricity;

                    void EditSMA()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Semi-major Axis (km)", $"In kilometers from the center of {mainBody.bodyName}"));
                        GUILayout.FlexibleSpace();
                        if (manualTargetMode == 8) MakeNumberEditField("SMA", ref SMA_Adj, 1d, radiusScaled + atmosphereDepthScaled, SoI);
                        else MakeNumberEditField("SMA", ref SMA_Adj, 1d, Util.Max(epsilon, radiusScaled + atmosphereDepthScaled, (radiusScaled + atmosphereDepthScaled) / (1d - manualEccentricity)), SoI); // km, make sure periapsis doesnt go below body
                        GUILayout.EndHorizontal();

                        manualSMA = SMA_Adj * 1000d;
                    }

                    void DisplaySMA()
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Semi-major Axis (km)", $"In kilometers from the center of {mainBody.bodyName}"));
                        GUILayout.FlexibleSpace();
                        GUILayout.Box(new GUIContent(FormatDecimals(manualSMA / 1000d) + "km", $"{manualSMA}m"), GUILayout.MinWidth(150));
                        GUILayout.EndHorizontal();
                    }

                    void ResetSMA() => SMA_Adj = manualSMA / 1000d;

                    switch (manualTargetMode)
                    {
                        case 0:
                            manualModeLabel = "Ap + Pe";
                            manualModeTooltip = "Eccentricity, period, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 0))
                            {
                                ResetApoapsis();
                                ResetPeriapsis();
                            }

                            EditApoapsis();

                            GUILayout.Space(5);

                            EditPeriapsis();

                            manualEccentricity = (manualApR - manualPeR) / (manualApR + manualPeR);
                            manualSMA = (manualApR + manualPeR) / 2d;
                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);

                            DisplayEccentricity();
                            DisplaySMA();
                            DisplayPeriod();

                            break;
                        case 1:
                            manualModeLabel = "Ecc + SMA";
                            manualModeTooltip = "Apoapsis, periapsis, and period are calculated automatically";

                            if (StateChanged("manualTargetMode", 1))
                            {
                                ResetEccentricity();
                                ResetSMA();
                            }

                            EditEccentricity();

                            GUILayout.Space(5);

                            EditSMA();

                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);
                            manualApR = manualSMA * (1 + manualEccentricity);
                            manualPeR = manualSMA * (1 - manualEccentricity);

                            DisplayPeriod();
                            DisplayApoapsis();
                            DisplayPeriapsis();

                            break;
                        case 2:
                            manualModeLabel = "Ecc + Period";
                            manualModeTooltip = "Apoapsis, periapsis, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 2))
                            {
                                ResetEccentricity();
                                ResetPeriod();
                            }

                            EditPeriod();

                            GUILayout.Space(5);

                            EditEccentricity();

                            manualSMA = Math.Pow(gravParameter * manualPeriod * manualPeriod / (tau * tau), 1 / 3d);
                            manualApR = manualSMA * (1 + manualEccentricity);
                            manualPeR = manualSMA * (1 - manualEccentricity);

                            DisplaySMA();
                            DisplayApoapsis();
                            DisplayPeriapsis();

                            break;
                        case 3:
                            manualModeLabel = "Ap + Ecc";
                            manualModeTooltip = "Periapsis, period, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 3))
                            {
                                ResetApoapsis();
                                ResetEccentricity();
                            }

                            EditApoapsis();

                            GUILayout.Space(5);

                            EditEccentricity();

                            manualSMA = manualApR / (1 + manualEccentricity);
                            manualPeR = manualSMA * (1 - manualEccentricity);
                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);

                            DisplaySMA();
                            DisplayPeriapsis();
                            DisplayPeriod();

                            break;
                        case 4:
                            manualModeLabel = "Pe + Ecc";
                            manualModeTooltip = "Apoapsis, period, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 4))
                            {
                                ResetPeriapsis();
                                ResetEccentricity();
                            }

                            EditPeriapsis();

                            GUILayout.Space(5);

                            EditEccentricity();

                            manualSMA = manualPeR / (1 - manualEccentricity);
                            manualApR = manualSMA * (1 + manualEccentricity);
                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);

                            DisplaySMA();
                            DisplayApoapsis();
                            DisplayPeriod();

                            break;
                        case 5:
                            manualModeLabel = "Ap + Period";
                            manualModeTooltip = "Periapsis, eccentricity, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 5))
                            {
                                ResetApoapsis();
                                ResetPeriod();
                            }

                            EditApoapsis();

                            GUILayout.Space(5);

                            EditPeriod();

                            manualSMA = Math.Pow(gravParameter * manualPeriod * manualPeriod / (tau * tau), 1 / 3d);
                            manualPeR = manualSMA * (1 - manualEccentricity);
                            manualEccentricity = (manualApR - manualPeR) / (manualApR + manualPeR);

                            DisplaySMA();
                            DisplayPeriapsis();
                            DisplayEccentricity();

                            break;
                        case 6:
                            manualModeLabel = "Pe + Period";
                            manualModeTooltip = "Apoapsis, eccentricity, and SMA are calculated automatically";

                            if (StateChanged("manualTargetMode", 6))
                            {
                                ResetPeriapsis();
                                ResetPeriod();
                            }

                            EditPeriapsis();

                            GUILayout.Space(5);

                            EditPeriod();

                            manualSMA = Math.Pow(gravParameter * manualPeriod * manualPeriod / (tau * tau), 1 / 3d);
                            manualApR = manualSMA * (1 + manualEccentricity);
                            manualEccentricity = (manualApR - manualPeR) / (manualApR + manualPeR);

                            DisplaySMA();
                            DisplayApoapsis();
                            DisplayEccentricity();

                            break;
                        case 7:
                            manualModeLabel = "Ap + SMA";
                            manualModeTooltip = "Periapsis, eccentricity, and period are calculated automatically";

                            if (StateChanged("manualTargetMode", 7))
                            {
                                ResetApoapsis();
                                ResetSMA();
                            }

                            EditApoapsis();

                            GUILayout.Space(5);

                            EditSMA();

                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);
                            manualEccentricity = (manualApR / manualSMA) - 1;
                            manualPeR = manualSMA * (1 - manualEccentricity);

                            DisplayPeriod();
                            DisplayPeriapsis();
                            DisplayEccentricity();

                            break;
                        case 8:
                            manualModeLabel = "Pe + SMA";
                            manualModeTooltip = "Apoapsis, eccentricity, and period are calculated automatically";

                            if (StateChanged("manualTargetMode", 8))
                            {
                                ResetPeriapsis();
                                ResetSMA();
                            }

                            EditPeriapsis();

                            GUILayout.Space(5);

                            EditSMA();

                            manualPeriod = tau * Math.Sqrt(Math.Pow(manualSMA, 3) / gravParameter);
                            manualEccentricity = 1 - (manualPeR / manualSMA);
                            manualApR = manualSMA * (1 + manualEccentricity);

                            DisplayPeriod();
                            DisplayApoapsis();
                            DisplayEccentricity();

                            break;
                    }

                    GUILayout.Space(5);

                    void ResetInclination() => inclination_Adj = useRadians ? manualInclination * degToRad : manualInclination;
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"Inclination ({(useRadians ? "radians" : "degrees")})");
                    GUILayout.FlexibleSpace();
                    MakeNumberEditField("inclination", ref inclination_Adj, step, 0d, max, true);
                    GUILayout.EndHorizontal();
                    manualInclination = useRadians ? inclination_Adj * radToDeg : inclination_Adj; // inclination is in degrees

                    GUILayout.Space(5);

                    void ResetLAN() => LAN_Adj = useRadians ? manualLAN : manualLAN * radToDeg;
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent($"LAN ({(useRadians ? "radians" : "degrees")})", "Longitude of the ascending node"));
                    GUILayout.FlexibleSpace();
                    MakeNumberEditField("LAN", ref LAN_Adj, step, 0d, max, true);
                    GUILayout.EndHorizontal();
                    manualLAN = useRadians ? LAN_Adj * radToDeg : LAN_Adj; // LAN is in degrees

                    GUILayout.Space(5);

                    void ResetAoP() => AoP_Adj = useRadians ? manualAoP : manualAoP * radToDeg;
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent($"AoP ({(useRadians ? "radians" : "degrees")})", "Argument of Perigee"));
                    GUILayout.FlexibleSpace();
                    MakeNumberEditField("AoP", ref AoP_Adj, step, 0d, max, true);
                    GUILayout.EndHorizontal();
                    manualAoP = useRadians ? AoP_Adj * radToDeg : AoP_Adj; // AoP is in degrees

                    GUILayout.Space(5);

                    void ResetMNA() => MNA_Adj = useRadians ? manualMNA : manualMNA * radToDeg;
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent($"MNA ({(useRadians ? "radians" : "degrees")})", "Mean anomaly at epoch"));
                    GUILayout.FlexibleSpace();
                    MakeNumberEditField("MNA", ref MNA_Adj, step, 0d, max, true);
                    GUILayout.EndHorizontal();
                    manualMNA = useRadians ? MNA_Adj : MNA_Adj * degToRad; // MNA is in radians

                    //GUILayout.Space(5);
                    //if (GUILayout.Button("Log")) Log($"useRadians: {useRadians}, manualTargetMode: {manualTargetMode}," +
                    //    $" manualEccentricity: {manualEccentricity}, manualSMA: {manualSMA}, manualPeriod: {manualPeriod}, manualInclination: {manualInclination}, " +
                    //    $"manualApR: {manualApR}, manualPeR: {manualPeR}, manualLAN: {manualLAN}, manualAoP: {manualAoP}, manualMNA: {manualMNA}, maxEccentricity: {Math.Min(1d - epsilon, 1d - (radius + atmosphereDepth) / manualSMA)}, minSMA: {Math.Max(radiusScaled + atmosphereDepthScaled, (radiusScaled + atmosphereDepthScaled) / (1d - manualEccentricity))}, mainBody: {mainBody}");

                    GUILayout.Space(5);

                    GUILayout.BeginHorizontal();
                    bool needsUpdate = StateChanged(false, "manualOrbitStates", manualEccentricity, manualSMA, manualInclination, manualLAN, manualAoP, manualMNA, mainBody); // does not update cached values
                    GUI.enabled = needsUpdate;
                    if (GUILayout.Button(new GUIContent($"{(needsUpdate ? "Save*" : "Saved")}", $"{(needsUpdate ? "There have been changes to the inputs that have not been saved" : "The saved orbit is the same as the current inputs")}"), GUILayout.Width(250)))
                    {
                        targetOrbit = new Orbit
                        {
                            eccentricity = manualEccentricity,
                            semiMajorAxis = manualSMA,
                            inclination = manualInclination,
                            LAN = manualLAN,
                            argumentOfPeriapsis = manualAoP,
                            meanAnomalyAtEpoch = manualMNA,
                            epoch = currentUT,
                            referenceBody = mainBody,
                        };
                        targetOrbit.Init(); // do NOT use SetOrbit, it causes the previous target's orbit to be changed
                        targetOrbit.UpdateFromUT(currentUT);
                        _ = StateChanged(true, "manualOrbitStates", manualEccentricity, manualSMA, manualInclination, manualLAN, manualAoP, manualMNA, mainBody); // update cached values to current
                        ClearAllCaches();
                        //Log("manual orbit changed");
                    }
                    if (ResetDefault("Reset to Last Saved Orbit", false))
                    {
                        var stateElements = GetStateElements("manualOrbitStates");
                        if (stateElements != null && stateElements.Length == 7 && stateElements[6] is CelestialBody body && body.Equals(mainBody)) // mod gets restarted during scene changes, so this wont work between scenes
                        {
                            isSavedOrbitCorrect = true;
                            manualEccentricity = (double)stateElements[0];
                            manualSMA = (double)stateElements[1];
                            manualInclination = (double)stateElements[2];
                            manualLAN = (double)stateElements[3];
                            manualAoP = (double)stateElements[4];
                            manualMNA = (double)stateElements[5];

                            ResetSMA();
                            ResetEccentricity();
                            ResetInclination();
                            ResetLAN();
                            ResetAoP();
                            ResetMNA();

                            ResetApoapsis(true);
                            ResetPeriapsis(true);
                            ResetPeriod(true);

                            // if in a different mode than the original, needsUpdate wont switch to false due to floating point stuff, TODO (need optional tolerance check with StateChanged)
                        }
                        else isSavedOrbitCorrect = false;
                    }
                    GUI.enabled = true;
                    GUILayout.FlexibleSpace();
                    if (isSavedOrbitCorrect == false) GUILayout.Label(new GUIContent("Error!", "Saved orbit is not valid for this celestial body! A new saved orbit must be set."));
                    GUILayout.EndHorizontal();
                }
            }
            finally
            {
                Tooltip.Instance?.RecordTooltip(id);
                GUI.DragWindow();
                ResetWindow(ref needManualOrbitReset, ref manualOrbitRect);
            }
        }

        #endregion

        // html tags rendered by ksp: <b> and </b>, <i> and </i>, (add more)
    }
}
