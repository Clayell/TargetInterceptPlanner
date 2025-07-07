// Adapted from TWP2 (https://github.com/Nazfib/TransferWindowPlanner2/tree/main/TransferWindowPlanner2/UI/Rendering), thanks Nazfib!


using System;
using UnityEngine;

namespace LunarTransferPlanner
{
    public static class RenderUtils
    {
        public static LineRenderer InitLine(GameObject objToAttach, Color lineColor, int vertexCount, int initialWidth, Material linesMaterial)
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

        public static void DrawArc(LineRenderer line, Vector3d center, Vector3d fromDir, Vector3d orbitNormal, double angle, double radius, int arcPoints)
        {
            if (line == null) return;

            for (int i = 0; i < arcPoints; i++)
            {
                float t = (float)i / (arcPoints - 1);
                QuaternionD rot = QuaternionD.AngleAxis(angle * t, -orbitNormal);
                Vector3d arcDir = rot * fromDir;
                Vector3d worldPos = ScaledSpace.LocalToScaledSpace(center + arcDir.normalized * radius);
                line.SetPosition(i, worldPos);
            }

            line.startWidth = line.endWidth = 10f / 1000f * PlanetariumCamera.fetch.Distance;
            line.enabled = true;
        }

        public static void DrawLine(LineRenderer line, Vector3d center, Vector3d start, Vector3d end)
        {
            if (line == null) return;

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

        private GUIStyle _styleLabel = null;

        private Vector3d Point1Direction;
        private Vector3d Point2Direction;

        private Vector3d orbitNormal;
        private double AoPDiff;

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

            //Log("Initializing PhaseAngle Render");
            _objLineStart = new GameObject("LineStart");
            _objLineEnd = new GameObject("LineEnd");
            _objLineArc = new GameObject("LineArc");

            //Get the orbit lines material so things look similar
            var orbitLines = ((MapView)FindObjectOfType(typeof(MapView))).orbitLinesMaterial;

            //init all the lines
            _lineStart = RenderUtils.InitLine(_objLineStart, Color.blue, 2, 10, orbitLines); // TODO, allow all of these colors to be changed
            _lineEnd = RenderUtils.InitLine(_objLineEnd, Color.red, 2, 10, orbitLines);
            _lineArc = RenderUtils.InitLine(_objLineArc, Color.green, ArcPoints, 10, orbitLines);

            _styleLabel = new GUIStyle
            {
                normal = { textColor = Color.white },
                alignment = TextAnchor.MiddleCenter,
            };

            Tooltip.RecreateInstance();
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

        public void Draw(Orbit orbit, double launchAoP, double phasingAngle)
        {
            BodyOrigin = orbit.referenceBody;
            orbitNormal = ToWorld(Vector3d.Cross(orbit.pos, orbit.vel)).normalized;

            //Log($"orbit.pos: {orbit.pos}, orbit.vel: {orbit.vel}");

            if (orbit.pos.z < 1e-5 || orbit.vel.z < 1e-5)
            {
                Quaternion tilt = Quaternion.AngleAxis((float)1e-5, Vector3.right); // tilt by .00001 degrees to make it not equatorial
                orbitNormal = ToWorld(Vector3d.Cross(tilt * orbit.pos, tilt * orbit.vel)).normalized;
            }

            Point1Direction = AoPToWorldVector(launchAoP * LunarTransferPlanner.degToRad);
            Point2Direction = AoPToWorldVector(Util.ClampAngle(launchAoP - phasingAngle, false) * LunarTransferPlanner.degToRad);

            AoPDiff = phasingAngle;

            _startDrawing = DateTime.Now;
            _currentDrawingState = DrawingState.DrawingLinesAppearing;
        }

        public void Hide()
        {
            _startDrawing = DateTime.Now;
            _currentDrawingState = DrawingState.Hiding;
        }

        private Vector3d ToWorld(Vector3d v) => new Vector3d(v.x, v.z, v.y);

        private Vector3d AoPToWorldVector(double AoPRad)
        {
            Vector3d nodeLine = Vector3d.Cross(Vector3d.up, orbitNormal).normalized;

            return Math.Cos(AoPRad) * nodeLine + Math.Sin(AoPRad) * Vector3d.Cross(orbitNormal, nodeLine);
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
            var dir1 = Point1Direction.normalized;
            var dir2 = Point2Direction.normalized;

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

                    var partialdir1 = dir1 * Mathf.Lerp(0, (float)lineLength, pctDone);
                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, partialdir1);
                    break;

                case DrawingState.DrawingArcAppearing:
                    pctDone = (float)(DateTime.Now - _startDrawing).TotalSeconds / AppearTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.DrawingFullPicture; }
                    pctDone = Mathf.Clamp01(pctDone);

                    Vector3d partialdir2 = QuaternionD.AngleAxis(AoPDiff * pctDone, -orbitNormal) * dir1;

                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, dir1 * lineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, partialdir2 * lineLength);
                    RenderUtils.DrawArc(_lineArc, center, dir1, orbitNormal, AoPDiff * pctDone, arcRadius, ArcPoints);
                    break;

                case DrawingState.DrawingFullPicture:
                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, dir1 * lineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, dir2 * lineLength);
                    RenderUtils.DrawArc(_lineArc, center, dir1, orbitNormal, AoPDiff, arcRadius, ArcPoints);
                    break;

                case DrawingState.Hiding:
                    pctDone = (float)(DateTime.Now - _startDrawing).TotalSeconds / HideTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.Hidden; }
                    pctDone = Mathf.Clamp01(pctDone);

                    var partialLineLength = Mathf.Lerp((float)lineLength, 0, pctDone);
                    var partialArcRadius = Mathf.Lerp((float)arcRadius, 0, pctDone);

                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, dir1 * partialLineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, dir2 * partialLineLength);
                    RenderUtils.DrawArc(_lineArc, center, dir1, orbitNormal, AoPDiff, partialArcRadius, ArcPoints);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        internal void OnGUI()
        {
            if (BodyOrigin == null) { return; }
            if (!MapView.MapIsEnabled || !(_currentDrawingState is DrawingState.DrawingFullPicture)) { return; }

            Vector3 center = BodyOrigin.transform.position;
            double length = 5 * BodyOrigin.Radius;
            Vector3 dir1 = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + length * Point1Direction.normalized));
            Vector3 dir2 = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + length * Point2Direction.normalized));

            // checking z coordinate hides labels when they're behind the camera
            if (dir1.z > 0) GUI.Label(new Rect(dir1.x - 50, Screen.height - dir1.y - 15, 100, 30), new GUIContent("Parking Orbit Insertion", "This is the point directly above the launch site"), _styleLabel);
            if (dir2.z > 0) GUI.Label(new Rect(dir2.x - 50, Screen.height - dir2.y - 15, 100, 30), new GUIContent("Transfer Maneuver Execution", "According to the phasing angle, this is where the transfer maneuver needs to be executed"), _styleLabel);

            Vector3d halfDir = QuaternionD.AngleAxis(AoPDiff / 2d, -orbitNormal) * Point1Direction.normalized;
            double arcRadius = 2.5 * BodyOrigin.Radius;

            Vector3 arcPoint = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + halfDir * arcRadius));

            if (arcPoint.z > 0) GUI.Label(new Rect(arcPoint.x - 25, Screen.height - arcPoint.y - 15, 50, 30), new GUIContent($"{AoPDiff:F2}\u00B0", $"Phasing Angle: {AoPDiff}\u00B0"), _styleLabel);
            
            Tooltip.Instance?.RecordTooltip(this.GetHashCode());
            Tooltip.Instance?.ShowTooltip(this.GetHashCode());
        }
    }

    public class OrbitRendererHack
    {
        // Ugly hack: Creating a new class derived from OrbitTargetRenderer does not work - the orbit lags behind the camera
        // movement when panning. Therefore, we need to use one of the built-in classes designed for rendering orbits: those
        // inheriting from OrbitTargetRenderer. Only the ContractOrbitRenderer is available without the DLC.
        private readonly ContractOrbitRenderer _renderer;

        private OrbitRendererHack(ContractOrbitRenderer renderer)
        {
            _renderer = renderer;
        }

        public static OrbitRendererHack Setup(Orbit orbit, Color color, bool activedraw = true)
        {
            //Orbit orbit
            // The ContractOrbitRenderer.Setup method requires a non-null contract; we provide a default-initialized one,
            // because anything else is really annoying to setup.
            // However, the *_onUpdateCaption methods don't work with a default-initialized Contract: they need a valid
            // Agent, which we can't provide (it's a protected field of the Contract class).
            // So, the full workaround is this: provide a default-initialized Contract to the Setup method, then immediately
            // set it to null before the caption update methods can make use of it.
            var renderer = ContractOrbitRenderer.Setup(new Contracts.Contract(), orbit, activedraw);
            renderer.SetColor(color);
            renderer.contract = null;
            return new OrbitRendererHack(renderer);
        }

        public void Cleanup()
        {
            _renderer.Cleanup();
        }
    }
}