namespace OtterLogic.StructuralEngine.Tests;

/// <summary>
/// Structural models as plain arrays, built so what the engine should read is known
/// by construction — the same arrays a Grasshopper component would hand over.
/// </summary>
internal sealed class Models
{
    private readonly List<double[]> _starts = new();
    private readonly List<double[]> _ends = new();
    private readonly List<double[]> _supports = new();

    /// <summary>Whether each line stands up rather than lies level, in the order added.</summary>
    public List<bool> Vertical { get; } = new();

    public int LineCount => _starts.Count;

    public int Line(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        _starts.Add(new[] { x1, y1, z1 });
        _ends.Add(new[] { x2, y2, z2 });
        Vertical.Add(Math.Abs(z2 - z1) > Math.Max(Math.Abs(x2 - x1), Math.Abs(y2 - y1)));
        return _starts.Count - 1;
    }

    public void Support(double x, double y, double z) => _supports.Add(new[] { x, y, z });

    public Vec Start(int line) => new(_starts[line][0], _starts[line][1], _starts[line][2]);

    public Vec End(int line) => new(_ends[line][0], _ends[line][1], _ends[line][2]);

    /// <summary>A rectilinear frame: columns at every grid point, beams both ways at every floor, pinned at the base, its corner at (x0, y0).</summary>
    public Models Frame(int baysX, int baysY, int storeys, double bay = 6.0, double storey = 4.0, double x0 = 0.0, double y0 = 0.0)
    {
        for (int i = 0; i <= baysX; i++)
            for (int j = 0; j <= baysY; j++)
            {
                Support(x0 + i * bay, y0 + j * bay, 0.0);
                for (int s = 0; s < storeys; s++)
                    Line(x0 + i * bay, y0 + j * bay, s * storey, x0 + i * bay, y0 + j * bay, (s + 1) * storey);
            }

        for (int s = 1; s <= storeys; s++)
        {
            double z = s * storey;
            for (int j = 0; j <= baysY; j++)
                for (int i = 0; i < baysX; i++)
                    Line(x0 + i * bay, y0 + j * bay, z, x0 + (i + 1) * bay, y0 + j * bay, z);

            for (int i = 0; i <= baysX; i++)
                for (int j = 0; j < baysY; j++)
                    Line(x0 + i * bay, y0 + j * bay, z, x0 + i * bay, y0 + (j + 1) * bay, z);
        }

        return this;
    }

    /// <summary>The joint welded at a point, by the engine's numbering, or -1.</summary>
    public static int JointAt(StructureGraph structure, double x, double y, double z, double join = 0.01)
    {
        var p = new Vec(x, y, z);
        for (int j = 0; j < structure.Joints.Length; j++)
            if (structure.Joints[j].DistanceTo(p) <= join)
                return j;
        return -1;
    }

    /// <summary>The same model turned about the vertical through the origin — lines and supports, in the same order.</summary>
    public Models TurnedAboutVertical(double degrees)
    {
        double angle = degrees * Math.PI / 180.0;
        double[] Turn(double[] p) => new[]
        {
            p[0] * Math.Cos(angle) - p[1] * Math.Sin(angle),
            p[0] * Math.Sin(angle) + p[1] * Math.Cos(angle),
            p[2],
        };

        var turned = new Models();
        for (int i = 0; i < _starts.Count; i++)
        {
            var (a, b) = (Turn(_starts[i]), Turn(_ends[i]));
            turned.Line(a[0], a[1], a[2], b[0], b[1], b[2]);
        }

        foreach (var support in _supports)
        {
            var p = Turn(support);
            turned.Support(p[0], p[1], p[2]);
        }

        return turned;
    }

    /// <summary>The engine's whole reading of the model, stage by stage, as a toolkit would make it.</summary>
    public Reading Read(double join = 0.01, bool withSupports = true)
    {
        var (starts, ends) = ModelInput.CheckLines(Rows(_starts), Rows(_ends));
        var none = Array.Empty<double[,]>();
        var supports = withSupports && _supports.Count > 0 ? Rows(_supports) : null;
        var structure = StructureGraph.Build(starts, ends, none, supports, join);
        var geometry = ElementGeometry.Measure(starts, ends, none);
        var members = PhysicalMembers.Read(structure, geometry);
        var assemblies = Assemblies.Read(structure, members);
        var paths = LoadPaths.Trace(structure, geometry, members, assemblies);
        var regions = Regions.Read(members);
        return new Reading(structure, geometry, members, assemblies, paths, regions);
    }

    public sealed record Reading(
        StructureGraph Structure, ElementGeometry Geometry, PhysicalMembers Members, Assemblies Assemblies, LoadPaths Paths, Regions Regions)
    {
        /// <summary>An element's level: the hand-overs its assembly stands from the ground.</summary>
        public int Level(int element) => Paths.Level[Assemblies.Of[Members.Of[element]]];

        public LoadTree Tree => Paths.Tree;

        /// <summary>The pieces the runs are made in.</summary>
        public Pieces Pieces() => StructuralEngine.Pieces.Cut(Structure, Geometry, Members, Assemblies, Paths);

        /// <summary>An element's assembly.</summary>
        public int Assembly(int element) => Assemblies.Of[Members.Of[element]];
    }

    private static double[,] Rows(List<double[]> points)
    {
        var rows = new double[points.Count, 3];
        for (int i = 0; i < points.Count; i++)
            for (int c = 0; c < 3; c++)
                rows[i, c] = points[i][c];
        return rows;
    }
}
