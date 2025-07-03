// Adapted from TWP2 (https://github.com/Nazfib/TransferWindowPlanner2), thanks Nazfib!


using System;
using UnityEngine;

namespace LunarTransferPlanner
{
    public static class RenderUtils
    {
        public static LineRenderer InitLine(
            GameObject objToAttach, Color lineColor, int vertexCount, int initialWidth, Material linesMaterial)
        {
            objToAttach.layer = 9;
            var lineReturn = objToAttach.AddComponent<LineRenderer>();

            lineReturn.material = linesMaterial;
            lineReturn.startColor = lineColor;
            lineReturn.endColor = lineColor;
            lineReturn.transform.parent = null;
            lineReturn.useWorldSpace = true;
            lineReturn.startWidth = initialWidth;
            lineReturn.endWidth = initialWidth;
            lineReturn.positionCount = vertexCount;
            lineReturn.enabled = false;

            return lineReturn;
        }

        public static Vector3d VectorToUnityFrame(Vector3d v) => Planetarium.fetch.rotation * v.xzy;

        public static void DrawArc(
            LineRenderer line, Vector3d center, Vector3d start, Vector3d end, double scale, int arcPoints)
        {
            for (var i = 0; i < arcPoints; i++)
            {
                var t = (float)i / (arcPoints - 1);
                // I'd like to use Vector3d.Slerp here, but it throws a MissingMethodException.
                Vector3d arcSegment = Vector3.Slerp(start, end, t);
                line.SetPosition(i, ScaledSpace.LocalToScaledSpace(center + arcSegment * scale));
            }

            line.startWidth = line.endWidth = 10f / 1000f * PlanetariumCamera.fetch.Distance;
            line.enabled = true;
        }

        public static void DrawLine(LineRenderer line, Vector3d center, Vector3d start, Vector3d end)
        {
            var startPos = ScaledSpace.LocalToScaledSpace(center + start);
            var endPos = ScaledSpace.LocalToScaledSpace(center + end);
            var camPos = PlanetariumCamera.Camera.transform.position;
            line.SetPosition(0, startPos);
            line.SetPosition(1, endPos);
            line.startWidth = 5f / 1000f * Vector3.Distance(camPos, startPos);
            line.endWidth = 5f / 1000f * Vector3.Distance(camPos, startPos);
            line.enabled = true;
        }

        public static bool CurrentSceneHasMapView() =>
            HighLogic.LoadedScene is GameScenes.FLIGHT
            || HighLogic.LoadedScene is GameScenes.TRACKSTATION;
    }

    public class MapAngleRenderer : MonoBehaviour
    {
        public bool IsDrawing => !(_currentDrawingState is DrawingState.Hidden);

        public bool IsHiding => _currentDrawingState is DrawingState.Hiding
                                || _currentDrawingState is DrawingState.Hidden;

        private DateTime _startDrawing;

        public CelestialBody BodyOrigin { get; set; }

        public Vector3d AsymptoteDirection { get; set; }

        public Vector3d PeriapsisDirection { get; set; }

        // Nullability: initialized in Start(), de-initialized in OnDestroy()
        private GameObject _objLineStart = null;
        private GameObject _objLineEnd = null;
        private GameObject _objLineArc = null;

        // Nullability: initialized in Start(), de-initialized in OnDestroy()
        private LineRenderer _lineStart = null;
        private LineRenderer _lineEnd = null;
        private LineRenderer _lineArc = null;

        private const int ArcPoints = 72;
        private const float AppearTime = 0.5f;
        private const float HideTime = 0.25f;

        // Nullability: initialized in Start(), de-initialized in OnDestroy()
        private GUIStyle _styleLabelEnd = null;
        private GUIStyle _styleLabelTarget = null;

        private enum DrawingState
        {
            Hidden,
            DrawingLinesAppearing,
            DrawingArcAppearing,
            DrawingFullPicture,
            Hiding,
        };

        private DrawingState _currentDrawingState = DrawingState.Hidden;

        private void Start()
        {
            if (!RenderUtils.CurrentSceneHasMapView())
            {
                enabled = false;
                return;
            }

            Log("Initializing EjectAngle Render");
            _objLineStart = new GameObject("LineStart");
            _objLineEnd = new GameObject("LineEnd");
            _objLineArc = new GameObject("LineArc");

            //Get the orbit lines material so things look similar
            var orbitLines = ((MapView)FindObjectOfType(typeof(MapView))).orbitLinesMaterial;

            //init all the lines
            _lineStart = RenderUtils.InitLine(_objLineStart, Color.blue, 2, 10, orbitLines);
            _lineEnd = RenderUtils.InitLine(_objLineEnd, Color.red, 2, 10, orbitLines);
            _lineArc = RenderUtils.InitLine(_objLineArc, Color.green, ArcPoints, 10, orbitLines);

            _styleLabelEnd = new GUIStyle
            {
                normal = { textColor = Color.white },
                alignment = TextAnchor.MiddleCenter,
            };
            _styleLabelTarget = new GUIStyle
            {
                normal = { textColor = Color.white },
                alignment = TextAnchor.MiddleCenter,
            };
        }


        private void OnDestroy()
        {
            _currentDrawingState = DrawingState.Hidden;

            //Bin the objects
            _lineStart = null;
            _lineEnd = null;
            _lineArc = null;

            _objLineStart.DestroyGameObject();
            _objLineEnd.DestroyGameObject();
            _objLineArc.DestroyGameObject();
        }

        private void Log(string message) => Util.Log(message);

        public void Draw(CelestialBody bodyOrigin, Vector3d asymptote, Vector3d periapsis)
        {
            BodyOrigin = bodyOrigin;
            AsymptoteDirection = asymptote.normalized;
            PeriapsisDirection = periapsis.normalized;

            _startDrawing = DateTime.Now;
            _currentDrawingState = DrawingState.DrawingLinesAppearing;
        }

        public void Hide()
        {
            _startDrawing = DateTime.Now;
            _currentDrawingState = DrawingState.Hiding;
        }

        internal void OnPreCull()
        {
            if (!RenderUtils.CurrentSceneHasMapView()) { return; }

            if (BodyOrigin == null) { return; }

            if (!MapView.MapIsEnabled || !IsDrawing)
            {
                _lineStart.enabled = false;
                _lineEnd.enabled = false;
                _lineArc.enabled = false;
                return;
            }

            var lineLength = BodyOrigin.Radius * 5;
            var arcRadius = BodyOrigin.Radius * 3;
            var asymptote = RenderUtils.VectorToUnityFrame(AsymptoteDirection.normalized);
            var periapsis = RenderUtils.VectorToUnityFrame(PeriapsisDirection.normalized);

            //Are we Showing, Hiding or Static State
            float pctDone;

            var center = BodyOrigin.transform.position;
            switch (_currentDrawingState)
            {
                case DrawingState.Hidden:
                    break;

                case DrawingState.DrawingLinesAppearing:
                    pctDone = (float)(DateTime.Now - _startDrawing).TotalSeconds / AppearTime;
                    if (pctDone >= 1)
                    {
                        _currentDrawingState = DrawingState.DrawingArcAppearing;
                        _startDrawing = DateTime.Now;
                    }
                    pctDone = Mathf.Clamp01(pctDone);

                    var partialAsymptote = asymptote * Mathf.Lerp(0, (float)lineLength, pctDone);
                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, partialAsymptote);
                    break;

                case DrawingState.DrawingArcAppearing:
                    pctDone = (float)(DateTime.Now - _startDrawing).TotalSeconds / AppearTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.DrawingFullPicture; }
                    pctDone = Mathf.Clamp01(pctDone);

                    Vector3d partialPeriapsis = Vector3.Slerp(asymptote, periapsis, pctDone);

                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, asymptote * lineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, partialPeriapsis * lineLength);
                    RenderUtils.DrawArc(_lineArc, center, asymptote, partialPeriapsis, arcRadius, ArcPoints);
                    break;

                case DrawingState.DrawingFullPicture:
                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, asymptote * lineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, periapsis * lineLength);
                    RenderUtils.DrawArc(_lineArc, center, asymptote, periapsis, arcRadius, ArcPoints);
                    break;

                case DrawingState.Hiding:
                    pctDone = (float)(DateTime.Now - _startDrawing).TotalSeconds / HideTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.Hidden; }
                    pctDone = Mathf.Clamp01(pctDone);

                    var partialLineLength = Mathf.Lerp((float)lineLength, 0, pctDone);
                    var partialArcRadius = Mathf.Lerp((float)arcRadius, 0, pctDone);

                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, asymptote * partialLineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, periapsis * partialLineLength);
                    RenderUtils.DrawArc(_lineArc, center, asymptote, periapsis, partialArcRadius, ArcPoints);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        internal void OnGUI()
        {
            if (BodyOrigin == null) { return; }
            if (!MapView.MapIsEnabled || !(_currentDrawingState is DrawingState.DrawingFullPicture)) { return; }

            var center = BodyOrigin.transform.position;
            var length = 5 * BodyOrigin.Radius;
            var asymptote = PlanetariumCamera.Camera.WorldToScreenPoint(
                ScaledSpace.LocalToScaledSpace(
                    center + length * RenderUtils.VectorToUnityFrame(AsymptoteDirection.normalized)));
            var periapsis = PlanetariumCamera.Camera.WorldToScreenPoint(
                ScaledSpace.LocalToScaledSpace(
                    center + length * RenderUtils.VectorToUnityFrame(PeriapsisDirection.normalized)));

            GUI.Label(
                new Rect(
                    periapsis.x - 50,
                    Screen.height - periapsis.y - 15,
                    100, 30),
                $"Burn position", _styleLabelEnd);

            GUI.Label(
                new Rect(
                    asymptote.x - 50,
                    Screen.height - asymptote.y - 15,
                    100, 30),
                "Escape direction", _styleLabelTarget);
        }
    }

    public class ParkingOrbitRendererHack
    {
        // Ugly hack: Creating a new class derived from OrbitTargetRenderer does not work - the orbit lags behind the camera
        // movement when panning. Therefore, we need to use one of the built-in classes designed for rendering orbits: those
        // inheriting from OrbitTargetRenderer. Only the ContractOrbitRenderer is available without the DLC.
        private readonly ContractOrbitRenderer _renderer;

        private ParkingOrbitRendererHack(ContractOrbitRenderer renderer)
        {
            _renderer = renderer;
        }

        public static ParkingOrbitRendererHack Setup(
            CelestialBody cb, double alt, double inc, double lan, bool activedraw = true)
        {
            // The ContractOrbitRenderer.Setup method requires a non-null contract; we provide a default-initialized one,
            // because anything else is really annoying to setup.
            // However, the *_onUpdateCaption methods don't work with a default-initialized Contract: they need a valid
            // Agent, which we can't provide (it's a protected field of the Contract class).
            // So, the full workaround is this: provide a default-initialized Contract to the Setup method, then immediately
            // set it to null before the caption update methods can make use of it.
            var orbit = new Orbit(inc, 0, cb.Radius + alt, lan, 0, 0, 0, cb);
            var renderer = ContractOrbitRenderer.Setup(new Contracts.Contract(), orbit, activedraw);
            renderer.SetColor(Color.red); // TODO, let this color be changed?
            renderer.contract = null;
            return new ParkingOrbitRendererHack(renderer);
        }

        public void Cleanup()
        {
            _renderer.Cleanup();
        }
    }
}