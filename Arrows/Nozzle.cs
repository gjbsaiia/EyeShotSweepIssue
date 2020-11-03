using System;
using System.Collections.Generic;
using System.Linq;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public class Nozzle
    {
        public Brep Shell;
        public Brep Neck;
        public List<Brep> Welds = new List<Brep>();


        private bool _flipNorm;
        private List<Surface> _intersectedSurfaces = new List<Surface>();
        private List<ICurve> _intersectionCurves = new List<ICurve>();
        private ICurve _innerIntersectionCurve => (_intersectionCurves.Count > 0) ? _intersectionCurves[0] : null;
        private Point3D _relOrigin;
        private Point3D _intersectOrigin;

        public Nozzle(CaseArgs args)
        {
            RunCase(args);
        }

        public void RunCase(CaseArgs selectedCase)
        {
            Welds.Clear();
            Shell = selectedCase.Shell;
            Neck = selectedCase.Neck;

            if (!SetIntersectionCurves(selectedCase))
                return;

            DefinePadFlushPlane(_intersectOrigin, new Region(_innerIntersectionCurve).ConvertToSurface(), out var norm);

            var center = FindCenter(selectedCase.CutRegion.ContourList.ToArray());

            var trimBrep = OffsetRevolvedShell(_intersectedSurfaces, selectedCase.FilletLength,
                _flipNorm, true, _relOrigin);
            if (!DifferenceShells(trimBrep, ref Neck))
                return;

            BuildSetInNozzleWeld(selectedCase, trimBrep, center);
            Shell.ExtrudeRemove(selectedCase.CutRegion, selectedCase.ExtrudeLength);
        }

        /*
         *
         * Various Helper methods
         *
         */
        private bool BuildSetInNozzleWeld(CaseArgs args, Brep offsetBrep, Point3D center)
        {

            /*
             * INTERNAL FILLET
             *
             *  The curve 'rail' is solved for by extruding the shell cut region, and intersecting it with
             *    the offset internal surface of the shell.
             *
             *  The point 'topCorner' is determined by taking the point along the XZ Plane with the largest Z value
             *    off of the rail curve, internalTopPoint and bottomPoint are solved for using the Normal and planar axis.
             *
             *
             *    |                                                         |
             *    |                                                         |
             *    ----------------- SHELL -----------------------------------
             *       bottomPoint ----- topCorner  
             *            -               |       |----------------------------------
             *                -           |       |  NECK
             *                    -       |       |
             *                   internalTopPoint |
             *                                    |
             *                                    |----------------------------------
             *
             */

            var topCornerNeck = args.CutRegion.ExtrudeAsBrep(args.ExtrudeLength);


            Brep.IntersectionLoops(topCornerNeck, offsetBrep, out var topCornerCurves);

            if (!topCornerCurves.Any())
                return false;

            var topCornerCurve = topCornerCurves.OrderByDescending(c => c.EndPoint.Z).First();


            var approximatePoint =  new Point3D(args.ReferencePoint.X + args.FilletLength, args.ReferencePoint.Y, 
                args.ReferencePoint.Z + args.NeckRadius + args.NeckThickness);

            topCornerCurve.ClosestPointTo(approximatePoint, out var t);
            var topCorner = topCornerCurve.PointAt(t);

            var internalTopPoint = topCorner + -args.FilletLength * args.ExtrudePlane.AxisY;
            var bottomPoint = topCorner + -args.FilletLength * args.Normal;

            var weldFaceCurves = new List<ICurve>
            {
                new Line(topCorner, internalTopPoint),
                new Line(internalTopPoint, bottomPoint),
                new Line(bottomPoint, topCorner)
            };
            var weldFace = new Region(new CompositeCurve(weldFaceCurves, 0.0001, true));

            switch (args.DisplayCase)
            {
                case Case.RegionAndRail:
                    Welds.Add(weldFace.ConvertToSurface().ConvertToBrep(0.0001));
                    Welds.Add(new Region(topCornerCurve).ConvertToSurface().ConvertToBrep(0.0001));
                    break;
                case Case.Rotational:
                    Welds.Add(weldFace.RevolveAsBrep(0.0, 2 * Math.PI, args.Normal, center, 0.0001));
                    break;
                case Case.Sweep:
                    Welds.Add(weldFace.SweepAsBrep(topCornerCurve, 0.0001, sweepMethodType.FrenetSerret));
                    break;
                default:
                    throw new ArgumentException("Unknown display case.");
            }

            return true;
        }

        public Plane DefinePadFlushPlane(Point3D origin, Surface padSurf, out Vector3D normal)
        {
            padSurf.Project(origin, 0.001, true, out var evalOrigin);
            normal = padSurf.Normal(evalOrigin);
            var normAngle = Math.Round(normal.AngleFromXY, 2);

            _flipNorm = normAngle.Equals(0.0);

            if (normal.Z < 0 && Math.Round(normal.X, 3).Equals(0.0) && Math.Round(normal.Y, 3).Equals(0.0)
                || normal.X < 0 && Math.Round(normal.Z, 3).Equals(0.0) && Math.Round(normal.Y, 3).Equals(0.0))
                normal.Negate();

            return new Plane(origin, normal);
        }

        public static Point3D FindCenter(ICurve[] curves)
        {
            var compCurve = new CompositeCurve(curves, 0.001, true);
            var lis = new List<Point3D>();
            var l = compCurve.Length();
            lis.AddRange(compCurve.GetPointsByLength(l / 69)); // nice.
            var x_mean = Math.Round(lis.Average(p => p.X), 3);
            var y_mean = 0.0;
            var z_mean = Math.Round(lis.Average(p => p.Z), 3);
            return new Point3D(x_mean, y_mean, z_mean);
        }

        private bool GetIntersectionCurves(Brep shell, Brep neck, out List<ICurve> intersectionCurves)
        {
            var neckSurfs = neck.ConvertToSurfaces();
            var shellSurfs = shell.ConvertToSurfaces(0.0001);

            intersectionCurves = new List<ICurve>();
            foreach (var surf in shellSurfs)
            {
                var notAdded = true;
                foreach (var nSurf in neckSurfs)
                {
                    surf.IntersectWith(nSurf, 0.0001, out var foundCurves);
                    if (foundCurves != null && foundCurves.Length > 0)
                    {
                        intersectionCurves.AddRange(foundCurves);
                        if (notAdded)
                            _intersectedSurfaces.Add(surf);
                        notAdded = false;
                    }
                }
            }

            return intersectionCurves.Count > 0;
        }

        private bool SetIntersectionCurves(CaseArgs args)
        {
            if (!GetIntersectionCurves(args.Shell, args.Neck, out var tmpCurves))
                return false;

            tmpCurves = UtilityEx.GetConnectedCurves(tmpCurves, 0.0001).ToList();
            var ordered = tmpCurves.OrderByDescending(c => c.StartPoint.X);
            _intersectionCurves.Add(ordered.Last());
            if (tmpCurves.Count > 1)
                _intersectionCurves.Add(ordered.First());
            _intersectOrigin = FindCenter(_innerIntersectionCurve.GetIndividualCurves());
            _relOrigin = new Point3D(0, 0, _intersectOrigin.Z);

            return true;
        }

        private Tuple<Surface, Surface> GetOuterInnerSurfaces(List<Surface> surfs, Point3D origin)
        {
            Surface outer = null;
            Surface inner = null;
            foreach (var surf in surfs)
            {
                if (outer == null && inner == null)
                    outer = inner = surf;
                else
                {
                    outer.ClosestPointTo(origin, out var farthest);
                    inner.ClosestPointTo(origin, out var closest);
                    surf.ClosestPointTo(origin, out var candidate);
                    var candDist = origin.DistanceTo(candidate);
                    if (candDist > origin.DistanceTo(farthest))
                        outer = surf;
                    if (candDist < origin.DistanceTo(closest))
                        inner = surf;
                }
            }
            return new Tuple<Surface, Surface>(inner, outer);
        }

        private Brep OffsetRevolvedShell(List<Surface> surfs, double thickness, bool flipNorm, bool inner, Point3D relOrigin)
        {
            var hasOffset = Math.Abs(thickness) > 0.0;

            var outerInner = GetOuterInnerSurfaces(surfs, relOrigin);
            var bound = (inner) ? outerInner.Item1 : outerInner.Item2;
            var curves = bound.Section(Plane.XZ, 0.0001).ToList();
            curves = UtilityEx.GetConnectedCurves(curves, 0.0001).ToList();
            var flat = curves.Any(c => c.IsInPlane(Plane.XY, 0.0001));
            var curve = (flat)
                ? curves.OrderByDescending(c => c.StartPoint.DistanceTo(Point3D.Origin)).Last()
                : curves.OrderByDescending(c => c.StartPoint.DistanceTo(Point3D.Origin)).First();
            var direction = 1.0;

            if (inner)
            {
                curve.Reverse();
                direction *= -1.0;
            }

            if ((curve.StartPoint.X > 0.0 && curve.EndPoint.X < 0.0) ||
                (curve.StartPoint.X < 0.0 && curve.EndPoint.X > 0.0))
            {
                var len = curve.Length();
                curve.SubCurve(curve.StartPoint, curve.GetPointsByLength(len / 2.0)[1], out curve);
            }

            var reg = new Region(curve);

            if (hasOffset && flat)
                reg.Translate(Vector3D.AxisZ * direction * thickness);
            else if (hasOffset)
                reg = new Region(curve.Offset(direction * thickness, Vector3D.AxisY, 0.0001, true));

            var b = reg.RevolveAsBrep(0.0, 2.0 * Math.PI, Vector3D.AxisZ, Point3D.Origin, 0.0001);

            if ((flipNorm ^ inner) && !flat)
                b.FlipNormal();

            return b;
        }


        private bool DifferenceShells(Brep shell, ref Brep needsCut, bool wantOuter = true)
        {
            var pieces = new List<Brep>();
            pieces.AddRange(Brep.Difference(needsCut, shell) ?? new Brep[0]);
            if (!pieces.Any())
                return !Brep.Intersect(shell, needsCut, out var pts);
            needsCut = (wantOuter)
                ? pieces.OrderByDescending(p => p.Vertices.Max(v => v.MaximumCoordinate)).First()
                : pieces.OrderByDescending(p => p.Vertices.Max(v => v.MaximumCoordinate)).Last();
            return true;
        }

    }
}