using System;
using devDept.Eyeshot.Entities;
using devDept.Geometry;

namespace SectioningProblem
{
    public enum Case
    {
        RegionAndRail,
        Rotational,
        Sweep,
    }
    public class CaseArgs
    {
        public Brep Shell;
        public Brep Neck;
        public Region CutRegion;
        public Vector3D Normal;
        public Point3D ReferencePoint;
        public Plane FlushPlane;
        public Plane ExtrudePlane;
        public double FilletLength;
        public double ShellThickness;
        public double ExternalLength;
        public double InternalLength;
        public double ExtrudeLength => ExternalLength + InternalLength + ShellThickness;
        public double NeckRadius;
        public double NeckThickness;
        public double ShellOpeningRadius;
        public Case DisplayCase;

        public CaseArgs(Case which)
        {
            CylinderAndNeck();
            DisplayCase = which;
        }

        private void CylinderAndNeck()
        {

            ShellThickness = 1.0;
            NeckRadius = 1.0;
            NeckThickness = 0.2;
            ShellOpeningRadius = 1.2 * (NeckRadius + NeckThickness);
            ExternalLength = 7.0;
            InternalLength = 2.0;
            FilletLength = 0.4;
            ReferencePoint = new Point3D(20, 0, 30);
            var theta = 0.0;
            var beta = Math.PI / 2.0;

            var baseShape = new Circle(Point3D.Origin, 20.0);
            var ring = baseShape.OffsetToRegion(1.0, 0.0001, true);
            Shell = ring.ExtrudeAsBrep(60.0);

            Normal = new Vector3D(Math.Cos(theta) * Math.Sin(beta),
                Math.Sin(theta) * Math.Sin(beta), Math.Cos(beta));
            Normal.Normalize();
            FlushPlane = new Plane(ReferencePoint, Normal);
            ExtrudePlane = FlushPlane.Clone() as Plane;
            ExtrudePlane.Translate(Normal * (ExternalLength + ShellThickness));
            ExtrudePlane.Flip();

            var circle = new Circle(ExtrudePlane, ExtrudePlane.Origin, NeckRadius);
            var face = circle.OffsetToRegion(NeckThickness, 0.0001, true);
            CutRegion = new Region(new Circle(ExtrudePlane, ExtrudePlane.Origin, ShellOpeningRadius));
            Neck = face.ExtrudeAsBrep(ExtrudeLength);
        }

    }
}
