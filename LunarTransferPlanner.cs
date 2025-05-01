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

using System;
using System.ComponentModel;
using System.Text;
using UnityEngine;
using System.Linq;
using Contracts.Agents.Mentalities;
using KerbalConstructionTime;

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

        public static float Floor(float x, int digits)
        {
            int lol = 1;
            while (digits > 0)
            {
                lol *= 10;
                --digits;
            }
            return (float)((int)(x * lol)) / lol;
        }
    }

    [KSPAddon(KSPAddon.Startup.FlightAndKSC, false)]
    public class LunarTransferPlanner : DaMichelToolbarSuperWrapper.PluginWithToolbarSupport
    {
        // main window
        Rect        windowRect = new Rect(100,100,-1,-1);
        bool        isWindowOpen = true;
        // gui stuff
        float       flightTime = 4f;
        float       nextTickFT = 0f;
        float       warpMargin = 60f;
        float       nextTickWM = 0f;
        string      windowTitle = "";
        GUISkin     skin;

#region boring stuff
        protected override DaMichelToolbarSuperWrapper.ToolbarInfo GetToolbarInfo()
        {
            return new DaMichelToolbarSuperWrapper.ToolbarInfo {
                name = "LunarTransferPlanner",
                tooltip = "LunarTransferPlanner Show/Hide Gui",
                toolbarTexture = "LunarTransferPlanner/toolbarbutton",
                launcherTexture = "LunarTransferPlanner/launcherbutton",
                visibleInScenes = new GameScenes[] { GameScenes.FLIGHT, GameScenes.SPACECENTER }
            };
        }

        void Awake()
        {
            skin = (GUISkin)GUISkin.Instantiate(HighLogic.Skin);
            skin.button.padding = new RectOffset(2, 2, 2, 2);
            skin.button.margin = new RectOffset(1, 1, 1, 1);
            skin.box.padding = new RectOffset(2, 2, 2, 2);
            skin.box.margin = new RectOffset(1, 1, 1, 1);
            skin.textField.margin = new RectOffset(3,1,1,1);
            skin.textField.padding = new RectOffset(4,2,1,0);

            LoadSettings();
            InitializeToolbars();
            OnGuiVisibilityChange();
        }

        public void Start()
        {
            KACWrapper.InitKACWrapper();
        }

        public void OnDestroy()
        {
            SaveSettings();
            TearDownToolbars();
        }


        protected override  void OnGuiVisibilityChange()
        {
            isWindowOpen = isGuiVisible;
        }

        void SaveSettings()
        {
            ConfigNode settings = new ConfigNode();
            settings.name = "LUNAR_TRANSFER_SETTINGS";
            SaveMutableToolbarSettings(settings);
            SaveImmutableToolbarSettings(settings);
            settings.AddValue("windowRect.xMin", windowRect.xMin);
            settings.AddValue("windowRect.yMin", windowRect.yMin);
            settings.AddValue("flightTime", flightTime);
            settings.AddValue("warpMargin", warpMargin);
            settings.Save(AssemblyLoader.loadedAssemblies.GetPathByType(typeof(LunarTransferPlanner)) + "/settings.cfg");
        }


        void LoadSettings()
        {
            ConfigNode settings = ConfigNode.Load(AssemblyLoader.loadedAssemblies.GetPathByType(typeof(LunarTransferPlanner)) + "/settings.cfg");
            if (settings != null)
            {
                float x = windowRect.xMin, y = windowRect.yMin;
                Util.TryReadValue(ref x, settings, "windowRect.xMin");
                Util.TryReadValue(ref y, settings, "windowRect.yMin");
                Util.TryReadValue(ref flightTime, settings, "flightTime");
                Util.TryReadValue(ref warpMargin, settings, "warpMargin");
                windowRect = new Rect(x, y, windowRect.width, windowRect.height);
                LoadMutableToolbarSettings(settings);
                LoadImmutableToolbarSettings(settings);
            }
        }
#endregion

        void OnGUI()
        {
            if (isWindowOpen && buttonVisible)
            {
                GUI.skin = this.skin;
                windowRect = GUILayout.Window(this.GetHashCode(), windowRect, MakeMainWindow, windowTitle);
                float left = Mathf.Clamp(windowRect.x, 0, Screen.width-windowRect.width);
                float top = Mathf.Clamp(windowRect.y, 0, Screen.height-windowRect.height);
                windowRect = new Rect(left, top, windowRect.width, windowRect.height);
            }
        }

        private void MakeNumberEditField(ref float value, ref float nextTick, int digits, float step, float minValue)
        {
            GUILayout.BeginHorizontal();
            string textValue = value.ToString("F" + digits.ToString());
            string newlabel = GUILayout.TextField(textValue, GUILayout.MinWidth(40));
            if (textValue != newlabel)
            {
                float newvalue;
                if (float.TryParse(newlabel, out newvalue))
                {
                    value = newvalue;
                }
            }
            bool hitMinusButton = GUILayout.RepeatButton("-", GUILayout.MinWidth(16));
            bool hitPlusButton  = GUILayout.RepeatButton("+", GUILayout.MinWidth(16));
            if (hitPlusButton || hitMinusButton)
            {
                float tick = Time.realtimeSinceStartup;
                if (tick > nextTick)
                {
                    if (hitMinusButton)
                        value = Mathf.Max(minValue, value - step);
                    else
                        value += step;
                    value = Mathf.Round(value / step) * step;

                    nextTick = tick + 0.2f;
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

        private Vector3d GetLaunchPos(CelestialBody mainBody, ref double Latitude)
        {
            if (SpaceCenter.Instance != null)
            {
                PQSCity ksc = FindKSC(FlightGlobals.GetHomeBody());
                if (ksc)
                {
                    Latitude = ksc.lat;
                    return mainBody.GetWorldSurfacePosition(ksc.lat, ksc.lon, 0);
                }
                else
                {
                    Latitude = SpaceCenter.Instance.Latitude;
                    return mainBody.GetWorldSurfacePosition(SpaceCenter.Instance.Latitude, SpaceCenter.Instance.Longitude, 0);
                }
            }

            return Vector3d.zero;
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
            public readonly double   inclination;
            public readonly double   azimuth;
        }

        private OrbitData CalcOrbitForTime(CelestialBody target, Vector3d launchPos, double delayTime)
        {
            // Form a plane with the launch site, moon and earth centre in, use this as the orbital plane for launch
            CelestialBody mainBody = target.referenceBody;
            Vector3d EarthPos = mainBody.position;
            Vector3d EarthAxis = mainBody.angularVelocity.normalized;

            double targetTime = Planetarium.GetUniversalTime() + flightTime * 24d * 3600d + delayTime;
            Vector3d targetPos = target.getPositionAtUT(targetTime);

            Vector3d upVector = Quaternion.AngleAxis((float)(delayTime * 360d / mainBody.rotationPeriod), EarthAxis) * (launchPos - EarthPos).normalized;

            Vector3d orbitNorm = Vector3d.Cross(targetPos - EarthPos, upVector).normalized;
            double inclination = Math.Acos(Vector3d.Dot(orbitNorm, mainBody.angularVelocity.normalized));
            if (inclination > Math.PI / 2)
                inclination = Math.PI - inclination;

            Vector3d eastVec = Vector3d.Cross(EarthAxis, upVector).normalized;
            Vector3d northVec = Vector3d.Cross(eastVec, upVector).normalized;
            Vector3d launchVec = Vector3d.Cross(upVector, orbitNorm).normalized;

            double azimuth = Math.Acos(Vector3d.Dot(launchVec, northVec));
            if (Vector3d.Dot(launchVec, eastVec) < 0d)
                azimuth = Math.PI - azimuth;

            return new OrbitData(orbitNorm, inclination * 180d / Math.PI, azimuth * 180d / Math.PI);
        }

        private double EstimateLaunchTime(CelestialBody target, Vector3d launchPos, double latitude, double startTime)
        {
            double targetAz = 90d;
            double t = startTime;
            OrbitData launchOrbit = CalcOrbitForTime(target, launchPos, t);

            if (latitude >= target.orbit.inclination)
            {
                // High latitude path - find the next easterly launch to the target
                while (Math.Abs(launchOrbit.azimuth - targetAz) > 0.01d)
                {
                    double tStep = 60d;
                    double margin = 2.0;
                    double wrapMargin = 1.25;

                    OrbitData nextOrbit = CalcOrbitForTime(target, launchPos, t + tStep);

                    double gradient = (nextOrbit.azimuth - launchOrbit.azimuth);
        
                    if (Math.Abs(gradient) > 100d)
                    {
                        // Wrapping from north to south, short step and recalc.
                        t += tStep;
                    }
                    else if (gradient > 0)
                    {
                        if (launchOrbit.azimuth < targetAz)
                        {
                            t += tStep * (targetAz - launchOrbit.azimuth) / (margin * gradient);
                        }
                        else
                        {
                            t += tStep * (180 - launchOrbit.azimuth) / (wrapMargin * gradient);
                        }
                    }
                    else
                    {
                        if (launchOrbit.azimuth > targetAz)
                        {
                            t += tStep * (targetAz - launchOrbit.azimuth) / (margin * gradient);
                        }
                        else
                        {
                            t += tStep * (0 - launchOrbit.azimuth) / (wrapMargin * gradient);
                        }
                    }
                    
                    launchOrbit = CalcOrbitForTime(target, launchPos, t);
                }
            }
            else
            {
                double tStep = 1200d;
                double startGradient;

                for (;;)
                {
                    OrbitData nextOrbit = CalcOrbitForTime(target, launchPos, t + tStep);
                    startGradient = (nextOrbit.azimuth - launchOrbit.azimuth);

                    if (Math.Abs(startGradient) < 100)
                    {
                        if (startGradient > 0 && launchOrbit.azimuth < targetAz)
                            break;

                        if (startGradient < 0 && launchOrbit.azimuth > targetAz)
                            break;
                    }

                    t += tStep;
                    launchOrbit = nextOrbit;
                }

                while (tStep >= 1d)
                {
                    OrbitData nextOrbit = CalcOrbitForTime(target, launchPos, t + tStep);

                    double gradient = (nextOrbit.azimuth - launchOrbit.azimuth);

                    if ((gradient < 0) == (startGradient < 0))
                    {
                        t += tStep;
                        launchOrbit = nextOrbit;
                    }
                    else
                    {
                        t = Math.Max(t - tStep, 0d);
                        tStep /= 2;
                        launchOrbit = CalcOrbitForTime(target, launchPos, t);
                    }
                }
            }

            return t;
        }

        private string FormatTime(double t)
        {
            t = Math.Round(t);
            int hours = (int)Math.Floor(t / 3600);
            t -= hours * 3600d;
            int minutes = (int)Math.Floor(t / 60);
            t -= minutes * 60d;
            if (hours > 0)
                return $"{hours}h {minutes}m {t:F0}s";
            else if (minutes > 0)
                return $"{minutes}m {t:F0}s";
            return $"{t:F0}s";
        }

        void MakeMainWindow(int id)
        {
            GUILayout.BeginVertical();

                windowTitle = "Lunar Transfer";

                GUILayout.Space(4);
                GUILayout.Label("Flight Time (days)", GUILayout.ExpandWidth(true));
                MakeNumberEditField(ref flightTime, ref nextTickFT, 1, 0.1f, 0.1f);

                CelestialBody target = FlightGlobals.fetch.bodies.FirstOrDefault(body => body.name.Equals("Moon", StringComparison.OrdinalIgnoreCase));
                if (target == null)
                {
                    GUILayout.Space(4);
                    GUILayout.Box("Cannot find the Moon", GUILayout.MinWidth(80));
                }
                else
                {
                    double latitude = 0d;
                    Vector3d launchPos = GetLaunchPos(target.referenceBody, ref latitude);

                    OrbitData launchOrbit = CalcOrbitForTime(target, launchPos, 0d);
                    double firstLaunchETA = EstimateLaunchTime(target, launchPos, latitude, 0d);
                    double secondLaunchETA = EstimateLaunchTime(target, launchPos, latitude, firstLaunchETA + 3600d);

                    GUILayout.Space(4);
                    GUILayout.Label("Latitude", GUILayout.ExpandWidth(true));
                    GUILayout.Box(new GUIContent(String.Format("{0:0.00}\u00B0", latitude), "Latitude of current launch site"), GUILayout.MinWidth(100));

                    GUILayout.Space(4);
                    GUILayout.Label("Launch Now Incl", GUILayout.ExpandWidth(true));
                    GUILayout.Box(new GUIContent($"{(launchOrbit.azimuth > 90d ? -launchOrbit.inclination : launchOrbit.inclination):F2}°",
                        "Launch to this inclination now to reach a Lunar parking orbit"), GUILayout.MinWidth(100));

                    string tooltip = latitude >= target.orbit.inclination ?
                        "Launch at this time for an Easterly launch to Lunar parking orbit" :
                        "Launch at this time for a low inclination launch to Lunar parking orbit";

                    GUILayout.Space(4);
                    GUILayout.Label("First Window", GUILayout.ExpandWidth(true));
                    GUILayout.Box(new GUIContent(FormatTime(firstLaunchETA), tooltip), GUILayout.MinWidth(100));

                    GUILayout.Space(4);
                    GUILayout.Label("Second Window", GUILayout.ExpandWidth(true));
                    GUILayout.Box(new GUIContent(FormatTime(secondLaunchETA), tooltip), GUILayout.MinWidth(100));

                    GUILayout.Label("Warp Margin (sec)", GUILayout.ExpandWidth(true));
                    MakeNumberEditField(ref warpMargin, ref nextTickWM, 0, 5f, 0f);

                    GUILayout.Space(2);
                    GUILayout.BeginHorizontal();
                    GUI.enabled = KACWrapper.APIReady;
                    bool addAlarm = GUILayout.Button("Add Alarm", GUILayout.MinWidth(80));
                    GUI.enabled = true;
                    bool toggleWarp = GUILayout.Button(TimeWarp.CurrentRate > 1f ? "Stop Warp" : "Warp", GUILayout.MinWidth(80));
                    GUILayout.EndHorizontal();

                    if (addAlarm)
                    {
                        if (KACWrapper.APIReady)
                        {
                            string alarmId = KACWrapper.KAC.CreateAlarm(KACWrapper.KACAPI.AlarmTypeEnum.Raw, "Lunar transfer window", Planetarium.GetUniversalTime() + firstLaunchETA - warpMargin);
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
                        if (TimeWarp.CurrentRate > 1f)
                        {
                            TimeWarp.fetch.CancelAutoWarp();
                            TimeWarp.SetRate(0, false);
                        }
                        else
                        {
                            TimeWarp.fetch.WarpTo(Planetarium.GetUniversalTime() + firstLaunchETA - warpMargin);
                        }
                }
                }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
