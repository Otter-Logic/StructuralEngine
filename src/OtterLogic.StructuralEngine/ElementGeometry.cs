namespace OtterLogic.StructuralEngine;

/// <summary>
/// What each element is like on its own — where it is, how big, which way it
/// extends — measured from the coordinates as given, before any welding.
/// <para>
/// Lines and surfaces are measured so the same column means the same thing for
/// both. Size is a length: a line's own, a surface's square root of area. Extent
/// is how much of the element lies along each axis, summing to one: a line's
/// squared direction cosines, a surface's share of its plane along each axis. So
/// a level beam and a level slab both read as lying in plan, and a column and a
/// wall both as standing — without either being told what it is.
/// </para>
/// </summary>
public sealed class ElementGeometry
{
    private ElementGeometry()
    {
    }

    public Vec[] Centroid { get; private init; } = null!;

    /// <summary>A line's length, or the square root of a surface's area.</summary>
    public double[] Size { get; private init; } = null!;

    /// <summary>Share of the element along x, y and z, summing to one.</summary>
    public Vec[] Extent { get; private init; } = null!;

    /// <summary>A surface's longer in-plane spread over its shorter, at least one. One for a line.</summary>
    public double[] Aspect { get; private init; } = null!;

    public static ElementGeometry Measure(double[,] starts, double[,] ends, IReadOnlyList<double[,]> surfaces)
    {
        int lines = starts.GetLength(0);
        int n = lines + surfaces.Count;

        var centroid = new Vec[n];
        var size = new double[n];
        var extent = new Vec[n];
        var aspect = new double[n];
        var even = new Vec(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);

        for (int i = 0; i < lines; i++)
        {
            var a = Vec.Row(starts, i);
            var b = Vec.Row(ends, i);
            var d = b - a;
            double length = d.Length;

            centroid[i] = 0.5 * (a + b);
            size[i] = length;
            extent[i] = length > 0.0 ? new Vec(d.X * d.X, d.Y * d.Y, d.Z * d.Z) / (length * length) : even;
            aspect[i] = 1.0;
        }

        for (int k = 0; k < surfaces.Count; k++)
        {
            int e = lines + k;
            var boundary = surfaces[k];
            int m = boundary.GetLength(0);
            var vertices = Enumerable.Range(0, m).Select(v => Vec.Row(boundary, v)).ToArray();

            var newell = new Vec(0, 0, 0);
            for (int v = 0; v < m; v++)
                newell += vertices[v].Cross(vertices[(v + 1) % m]);

            double twiceArea = newell.Length;
            var mean = (1.0 / m) * vertices.Aggregate(new Vec(0, 0, 0), (sum, v) => sum + v);

            if (twiceArea <= 0.0)
            {
                centroid[e] = mean;
                size[e] = 0.0;
                extent[e] = even;
                aspect[e] = 1.0;
                continue;
            }

            var normal = (1.0 / twiceArea) * newell;

            // Area-weighted centroid of a fan from the first corner.
            var weighted = new Vec(0, 0, 0);
            double total = 0.0;
            for (int v = 1; v + 1 < m; v++)
            {
                double piece = (vertices[v] - vertices[0]).Cross(vertices[v + 1] - vertices[0]).Dot(normal);
                weighted += (piece / 3.0) * (vertices[0] + vertices[v] + vertices[v + 1]);
                total += piece;
            }

            centroid[e] = Math.Abs(total) > 0.0 ? (1.0 / total) * weighted : mean;
            size[e] = Math.Sqrt(0.5 * twiceArea);
            extent[e] = new Vec(1.0 - normal.X * normal.X, 1.0 - normal.Y * normal.Y, 1.0 - normal.Z * normal.Z) / 2.0;
            aspect[e] = InPlaneAspect(vertices, mean, normal);
        }

        return new ElementGeometry { Centroid = centroid, Size = size, Extent = extent, Aspect = aspect };
    }

    /// <summary>
    /// Square root of the ratio of the two in-plane variances of the corners —
    /// four for a one-by-four rectangle. Capped at a thousand, which a sliver
    /// reaches long before it stops being worth telling apart.
    /// </summary>
    private static double InPlaneAspect(Vec[] vertices, Vec mean, Vec normal)
    {
        var helper = Math.Abs(normal.X) < 0.9 ? new Vec(1, 0, 0) : new Vec(0, 1, 0);
        var u = normal.Cross(helper);
        u = (1.0 / u.Length) * u;
        var w = normal.Cross(u);

        double uu = 0.0, ww = 0.0, uw = 0.0;
        foreach (var v in vertices)
        {
            double a = (v - mean).Dot(u);
            double b = (v - mean).Dot(w);
            uu += a * a;
            ww += b * b;
            uw += a * b;
        }

        double half = 0.5 * (uu + ww);
        double root = Math.Sqrt(Math.Max(0.0, 0.25 * (uu - ww) * (uu - ww) + uw * uw));
        double larger = half + root;
        double smaller = half - root;

        return smaller <= larger * 1e-6 ? 1000.0 : Math.Min(1000.0, Math.Sqrt(larger / smaller));
    }
}
