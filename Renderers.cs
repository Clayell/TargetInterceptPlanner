// Adapted from TWP2 with permission (https://github.com/Nazfib/TransferWindowPlanner2/tree/main/TransferWindowPlanner2/UI/Rendering), thanks Nazfib!


using System;
using UnityEngine;

namespace LunarTransferPlanner
{
    internal static class RenderUtils
    {
        internal static LineRenderer InitLine(GameObject objToAttach, Color lineColor, int vertexCount, int initialWidth, Material linesMaterial)
        {
            objToAttach.layer = 9;
            LineRenderer lineReturn = objToAttach.AddComponent<LineRenderer>();

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

        internal static void DrawArc(LineRenderer line, Vector3d center, Vector3d fromDir, Vector3d orbitNormal, double angle, double radius, int arcPoints)
        {
            if (line == null) return;

            Vector3 halfwayWorldPos = Vector3.zero;

            for (int i = 0; i < arcPoints; i++)
            {
                double t = (double)i / (arcPoints - 1);
                QuaternionD rot = QuaternionD.AngleAxis(angle * t, -orbitNormal);
                Vector3d arcDir = rot * fromDir;
                Vector3d worldPos = ScaledSpace.LocalToScaledSpace(center + arcDir.normalized * radius);
                line.SetPosition(i, worldPos);

                if (i == arcPoints / 2) // rounds down if arcPoints is odd
                {
                    halfwayWorldPos = worldPos;
                }
            }

            Vector3 camPos = PlanetariumCamera.Camera.transform.position;

            line.startWidth = line.endWidth = 0.01f * Vector3.Distance(camPos, halfwayWorldPos);
            line.enabled = true;
        }

        internal static void DrawLine(LineRenderer line, Vector3d center, Vector3d start, Vector3d end)
        {
            if (line == null) return;

            Vector3d startPos = ScaledSpace.LocalToScaledSpace(center + start);
            Vector3d endPos = ScaledSpace.LocalToScaledSpace(center + end);
            Vector3 camPos = PlanetariumCamera.Camera.transform.position;

            line.SetPosition(0, startPos);
            line.SetPosition(1, endPos);
            line.startWidth = line.endWidth = 0.005f * Vector3.Distance(camPos, startPos);
            line.enabled = true;
        }
    }

    public class MapAngleRenderer : MonoBehaviour
    {
        internal bool IsDrawing => _currentDrawingState != DrawingState.Hidden;

        internal bool IsHidden => _currentDrawingState == DrawingState.Hidden;

        internal bool IsHiding => _currentDrawingState == DrawingState.Hiding || _currentDrawingState == DrawingState.Hidden;

        private DateTime _startDrawing;

        private CelestialBody BodyOrigin;

        // Nullability: initialized in Start(), de-initialized in OnDestroy()
        private GameObject _objLineStart = null;
        private GameObject _objLineEnd = null;
        private GameObject _objLineArc = null;

        // Nullability: initialized in Start(), de-initialized in OnDestroy()
        private LineRenderer _lineStart = null;
        private LineRenderer _lineEnd = null;
        private LineRenderer _lineArc = null;

        private const int ArcPoints = 72;
        private const double AppearTime = 0.5;
        private const double HideTime = 0.25;

        private GUIStyle _styleLabel = null;

        private Vector3d Point1Direction;
        private Vector3d Point2Direction;

        private Vector3d orbitNormal;
        private double initialAoPRad;
        private double newAoPRad;
        private double AoPDiff;
        private Orbit parkingOrbit;

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
            if (!Util.MapViewEnabled())
            {
                enabled = false;
                return;
            }

            //Log("Initializing PhaseAngle Render");
            _objLineStart = new GameObject("LineStart");
            _objLineEnd = new GameObject("LineEnd");
            _objLineArc = new GameObject("LineArc");

            //Get the orbit lines material so things look similar
            Material orbitLines = ((MapView)FindObjectOfType(typeof(MapView))).orbitLinesMaterial;

            //init all the lines
            _lineStart = RenderUtils.InitLine(_objLineStart, LunarTransferPlanner.startLineColor, 2, 10, orbitLines);
            _lineEnd = RenderUtils.InitLine(_objLineEnd, LunarTransferPlanner.endLineColor, 2, 10, orbitLines);
            _lineArc = RenderUtils.InitLine(_objLineArc, LunarTransferPlanner.arcLineColor, ArcPoints, 10, orbitLines);

            _styleLabel = new GUIStyle
            {
                normal = { textColor = Color.white },
                alignment = TextAnchor.MiddleCenter,
                fontSize = 16,
                fontStyle = FontStyle.Bold,
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

            Destroy(_objLineStart);
            Destroy(_objLineEnd);
            Destroy(_objLineArc);
        }

        private void Log(string message) => Util.Log(message);

        private void UpdateVectors()
        {
            orbitNormal = parkingOrbit.GetOrbitNormal().xzy; // GetOrbitNormal is weird, it changes constantly in flight, but not in the tracking station. works tho

            Vector3d AoPToWorldVector(double AoPRad)
            {
                Vector3d nodeLine = Vector3d.Cross(orbitNormal, Vector3d.up).normalized;

                return (Math.Cos(AoPRad) * nodeLine + Math.Sin(AoPRad) * Vector3d.Cross(nodeLine, orbitNormal)).normalized;
            }

            Point1Direction = AoPToWorldVector(initialAoPRad);
            Point2Direction = AoPToWorldVector(newAoPRad);
        }

        internal void Draw(Orbit orbit, double launchAoP, double phasingAngle, bool visibilityChanged)
        {
            BodyOrigin = orbit.referenceBody;
            parkingOrbit = orbit;
            initialAoPRad = launchAoP * LunarTransferPlanner.degToRad;
            newAoPRad = Util.ClampAngle(launchAoP + phasingAngle, false) * LunarTransferPlanner.degToRad;
            AoPDiff = phasingAngle;

            UpdateVectors();

            _startDrawing = DateTime.Now; // TODO, base this on currentUT instead, so it draws quicker with time warp?
            if (visibilityChanged) _currentDrawingState = DrawingState.DrawingLinesAppearing;
            else _currentDrawingState = DrawingState.DrawingFullPicture;
        }

        internal void Hide(bool visibilityChanged)
        {
            _startDrawing = DateTime.Now;
            if (visibilityChanged) _currentDrawingState = DrawingState.Hiding;
            else _currentDrawingState = DrawingState.Hidden;
        }

        void OnPreCull()
        {
            if (!Util.MapViewEnabled() || BodyOrigin == null || parkingOrbit == null || Point1Direction == null || Point2Direction == null || _currentDrawingState == DrawingState.Hidden)
            {
                if (_lineStart != null) _lineStart.enabled = false;
                if (_lineEnd != null) _lineEnd.enabled = false;
                if (_lineArc != null) _lineArc.enabled = false;
                return;
            }

            UpdateVectors(); // technically we only need to call this when in flight... TODO optimization?

            double lineLength = parkingOrbit.semiMajorAxis * 4d;
            double arcRadius = parkingOrbit.semiMajorAxis * 2d;
            Vector3d dir1 = Point1Direction;
            Vector3d dir2 = Point2Direction;

            //Are we Showing, Hiding or Static State
            double pctDone;

            Vector3d center = BodyOrigin.transform.position;
            switch (_currentDrawingState)
            {
                case DrawingState.Hidden: // this shouldnt be possible
                    break;

                case DrawingState.DrawingLinesAppearing:
                    pctDone = (DateTime.Now - _startDrawing).TotalSeconds / AppearTime;
                    if (pctDone >= 1)
                    {
                        _currentDrawingState = DrawingState.DrawingArcAppearing;
                        _startDrawing = DateTime.Now;
                    }
                    pctDone = Mathf.Clamp01((float)pctDone);

                    Vector3d partialdir1 = dir1 * Mathf.Lerp(0, (float)lineLength, (float)pctDone);
                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, partialdir1);
                    break;

                case DrawingState.DrawingArcAppearing:
                    pctDone = (DateTime.Now - _startDrawing).TotalSeconds / AppearTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.DrawingFullPicture; }
                    pctDone = Mathf.Clamp01((float)pctDone);

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
                    pctDone = (DateTime.Now - _startDrawing).TotalSeconds / HideTime;
                    if (pctDone >= 1) { _currentDrawingState = DrawingState.Hidden; }
                    pctDone = Mathf.Clamp01((float)pctDone);

                    double partialLineLength = Mathf.Lerp((float)lineLength, 0, (float)pctDone);
                    double partialArcRadius = Mathf.Lerp((float)arcRadius, 0, (float)pctDone);

                    RenderUtils.DrawLine(_lineStart, center, Vector3d.zero, dir1 * partialLineLength);
                    RenderUtils.DrawLine(_lineEnd, center, Vector3d.zero, dir2 * partialLineLength);
                    RenderUtils.DrawArc(_lineArc, center, dir1, orbitNormal, AoPDiff, partialArcRadius, ArcPoints);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        void OnGUI()
        {
            if (BodyOrigin == null || parkingOrbit == null || Point1Direction == null || Point2Direction == null || !Util.MapViewEnabled() || _currentDrawingState != DrawingState.DrawingFullPicture)
            { return; } // this causes the text to flash while resetting the renderer (but without changing the visibility), TODO fix

            Vector3 center = BodyOrigin.transform.position;
            double length = 4d * parkingOrbit.semiMajorAxis; // line uses 4
            Vector3 dir1 = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + length * Point1Direction));
            Vector3 dir2 = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + length * Point2Direction));

            bool cameraNear = PlanetariumCamera.fetch.Distance < 100000d * (parkingOrbit.semiMajorAxis / 6571000d); // 6571000 is a 200km parking orbit for earth

            // checking z coordinate hides labels when they're behind the camera
            if (dir1.z > 0 && cameraNear) GUI.Label(new Rect(dir1.x - 50, Screen.height - dir1.y - 15, 100, 30), new GUIContent("Parking Orbit Insertion", "This is the point directly above the launch site"), _styleLabel);
            if (dir2.z > 0 && cameraNear) GUI.Label(new Rect(dir2.x - 50, Screen.height - dir2.y - 15, 100, 30), new GUIContent("Transfer Maneuver Execution", "According to the phasing angle, this is where the transfer maneuver needs to be executed"), _styleLabel);

            Vector3d halfDir = QuaternionD.AngleAxis(AoPDiff / 2d, -orbitNormal) * Point1Direction;
            double arcRadius = 2.5 * parkingOrbit.semiMajorAxis; // arc uses 2

            Vector3 arcPoint = PlanetariumCamera.Camera.WorldToScreenPoint(ScaledSpace.LocalToScaledSpace(center + halfDir * arcRadius));

            if (arcPoint.z > 0 && cameraNear) GUI.Label(new Rect(arcPoint.x - 25, Screen.height - arcPoint.y - 15, 50, 30), new GUIContent($"{LunarTransferPlanner.FormatDecimals(AoPDiff)}\u00B0", $"Phasing Angle: {AoPDiff}\u00B0"), _styleLabel);

            Tooltip.Instance?.RecordTooltip(this.GetHashCode());
            Tooltip.Instance?.ShowTooltip(this.GetHashCode());
        }
    }

    internal class OrbitRendererHack
    {
        // Ugly hack: Creating a new class derived from OrbitTargetRenderer does not work - the orbit lags behind the camera
        // movement when panning. Therefore, we need to use one of the built-in classes designed for rendering orbits: those
        // inheriting from OrbitTargetRenderer. Only the ContractOrbitRenderer is available without the DLC.
        private readonly ContractOrbitRenderer _renderer;

        private OrbitRendererHack(ContractOrbitRenderer renderer)
        {
            _renderer = renderer;
        }

        internal static OrbitRendererHack Setup(Orbit orbit, Color color)
        {
            // The ContractOrbitRenderer.Setup method requires a non-null contract; we provide a default-initialized one,
            // because anything else is really annoying to setup.
            // However, the *_onUpdateCaption methods don't work with a default-initialized Contract: they need a valid
            // Agent, which we can't provide (it's a protected field of the Contract class).
            // So, the full workaround is this: provide a default-initialized Contract to the Setup method, then immediately
            // set it to null before the caption update methods can make use of it.
            ContractOrbitRenderer renderer = ContractOrbitRenderer.Setup(new Contracts.Contract(), orbit); // activedraw seems to be not used at all
            renderer.SetColor(color);
            renderer.contract = null;
            return new OrbitRendererHack(renderer);
        }

        internal void Cleanup()
        {
            _renderer.Cleanup();
        }
    }
}