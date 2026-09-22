using OtterLogic.Graphs;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// The structure a model's geometry describes, read as graphs: which points are
/// one joint, which elements meet at each, and how far apart joints are along the
/// elements joining them.
/// <para>
/// Built from coordinates alone, because that is what every analysis package and
/// every Grasshopper wire can hand over. Node numbers would be tidier, but they
/// are the one thing that differs between every export.
/// </para>
/// <para>
/// Two graphs come out, because two different questions are asked of the model.
/// The <b>element graph</b> has a node per element, joined where elements meet — it
/// is what the clustering reads. The <b>route graph</b> has a node per joint, joined
/// along the elements with their length — it is what distances are measured over.
/// </para>
/// </summary>
public sealed class StructureGraph
{
    /// <summary>More grid cells than this under one segment's bounding box and the bearing search checks every joint instead.</summary>
    private const long MaximumCellsPerSegment = 50_000;

    private StructureGraph()
    {
    }

    /// <summary>Joint positions, at the first point welded into each.</summary>
    public Vec[] Joints { get; private init; } = null!;

    /// <summary>Furthest any point welded into each joint lies from it.</summary>
    public double[] Spread { get; private init; } = null!;

    /// <summary>Number of line elements; they come first, then the surfaces.</summary>
    public int LineCount { get; private init; }

    /// <summary>Number of elements, lines and surfaces together.</summary>
    public int ElementCount => Outline.Length;

    /// <summary>
    /// The joints along each element in order: for a line its start, every joint
    /// resting along it, its end; for a surface its boundary, corner by corner with
    /// the joints resting along each edge between.
    /// </summary>
    public int[][] Outline { get; private init; } = null!;

    /// <summary>Each element's distinct joints, ascending.</summary>
    public int[][] ElementJoints { get; private init; } = null!;

    /// <summary>A line whose ends are one joint, or a surface with fewer than three distinct corners.</summary>
    public bool[] Degenerate { get; private init; } = null!;

    /// <summary>The earlier element drawn between exactly the same joints, or -1.</summary>
    public int[] DuplicateOf { get; private init; } = null!;

    /// <summary>Elements touching each joint, at a corner or resting along an edge.</summary>
    public int[] Valence { get; private init; } = null!;

    /// <summary>Whether each joint has a support.</summary>
    public bool[] Supported { get; private init; } = null!;

    /// <summary>Whether any support was given at all.</summary>
    public bool HasSupports { get; private init; }

    /// <summary>Supports at no joint, by their index among the supports given.</summary>
    public int[] StrandedSupports { get; private init; } = null!;

    /// <summary>
    /// A node per element, an edge wherever two meet. Each joint shared by v
    /// elements adds 1 / (v − 1) to every pair of them, so a joint's pull on its
    /// elements does not grow with the square of how many meet there — a hub where
    /// thirty members meet would otherwise outweigh the rest of the model.
    /// </summary>
    public WeightedGraph Elements { get; private init; } = null!;

    /// <summary>A node per joint, an edge along every element between consecutive joints, weighted by length.</summary>
    public WeightedGraph Routes { get; private init; } = null!;

    public bool IsSurface(int element) => element >= LineCount;

    /// <summary>
    /// Every stretch of every element between consecutive joints along it: a line's
    /// pieces between the joints resting on it, a surface's boundary edge by edge.
    /// The same stretches <see cref="Routes"/> is built from, but with the element
    /// each belongs to, so what passes along one can be credited to its owner.
    /// </summary>
    public IEnumerable<(int Element, int A, int B)> Segments()
    {
        for (int e = 0; e < Outline.Length; e++)
        {
            var path = Outline[e];
            int count = IsSurface(e) ? path.Length : path.Length - 1;
            for (int p = 0; p < count; p++)
            {
                int a = path[p];
                int b = path[(p + 1) % path.Length];
                if (a != b)
                    yield return (e, a, b);
            }
        }
    }

    public static StructureGraph Build(
        double[,] starts, double[,] ends, IReadOnlyList<double[,]> surfaces, double[,]? supports, double join)
    {
        int lines = starts.GetLength(0);
        int n = lines + surfaces.Count;
        var welder = new Welder(join);

        var corners = new int[n][];
        for (int i = 0; i < lines; i++)
        {
            int a = welder.Weld(Vec.Row(starts, i));
            int b = welder.Weld(Vec.Row(ends, i));
            corners[i] = a == b ? new[] { a } : new[] { a, b };
        }

        for (int k = 0; k < surfaces.Count; k++)
        {
            var ring = new List<int>();
            for (int v = 0; v < surfaces[k].GetLength(0); v++)
            {
                int joint = welder.Weld(Vec.Row(surfaces[k], v));
                if (ring.Count == 0 || ring[^1] != joint)
                    ring.Add(joint);
            }

            // A boundary given closed repeats its first corner at the end.
            if (ring.Count > 1 && ring[^1] == ring[0])
                ring.RemoveAt(ring.Count - 1);

            corners[lines + k] = ring.ToArray();
        }

        var joints = welder.Positions.ToArray();

        var degenerate = new bool[n];
        for (int e = 0; e < n; e++)
            degenerate[e] = e < lines ? corners[e].Length < 2 : corners[e].Distinct().Count() < 3 || Area(joints, corners[e]) <= join * join;

        // Every edge of every element, as the segments joints may rest along.
        var segments = new List<(int Element, int Edge, int A, int B)>();
        for (int e = 0; e < n; e++)
        {
            if (degenerate[e])
                continue;

            int count = e < lines ? 1 : corners[e].Length;
            for (int edge = 0; edge < count; edge++)
                segments.Add((e, edge, corners[e][edge], corners[e][(edge + 1) % corners[e].Length]));
        }

        var bearings = Bearings(joints, segments, corners, join);

        var outline = new int[n][];
        for (int e = 0; e < n; e++)
        {
            if (degenerate[e])
            {
                outline[e] = corners[e].Distinct().ToArray();
                continue;
            }

            var path = new List<int>();
            int count = e < lines ? 1 : corners[e].Length;
            for (int edge = 0; edge < count; edge++)
            {
                path.Add(corners[e][edge]);
                if (bearings.TryGetValue((e, edge), out var along))
                    path.AddRange(along);
            }

            if (e < lines)
                path.Add(corners[e][1]);

            outline[e] = path.ToArray();
        }

        var elementJoints = outline.Select(path => path.Distinct().OrderBy(j => j).ToArray()).ToArray();

        var duplicateOf = Enumerable.Repeat(-1, n).ToArray();
        var first = new Dictionary<string, int>();
        for (int e = 0; e < n; e++)
        {
            if (degenerate[e])
                continue;

            string key = (e < lines ? "L" : "S") + string.Join(",", corners[e].Distinct().OrderBy(j => j));
            if (first.TryGetValue(key, out int earlier))
                duplicateOf[e] = earlier;
            else
                first[key] = e;
        }

        var incident = new List<int>[joints.Length];
        for (int j = 0; j < joints.Length; j++)
            incident[j] = new List<int>();
        for (int e = 0; e < n; e++)
            foreach (int j in elementJoints[e])
                incident[j].Add(e);

        var pairs = new Dictionary<long, double>();
        foreach (var here in incident)
        {
            if (here.Count < 2)
                continue;

            double share = 1.0 / (here.Count - 1);
            for (int p = 0; p < here.Count; p++)
            {
                for (int q = p + 1; q < here.Count; q++)
                {
                    long key = (long)here[p] * n + here[q];
                    pairs[key] = pairs.TryGetValue(key, out double weight) ? weight + share : share;
                }
            }
        }

        var elementGraph = WeightedGraph.FromEdges(n, pairs.Select(pair => ((int)(pair.Key / n), (int)(pair.Key % n), pair.Value)));

        var routes = new List<(int, int, double)>();
        for (int e = 0; e < n; e++)
        {
            var path = outline[e];
            int count = e < lines ? path.Length - 1 : path.Length;
            for (int p = 0; p < count; p++)
            {
                int a = path[p];
                int b = path[(p + 1) % path.Length];
                double length = joints[a].DistanceTo(joints[b]);
                if (a != b && length > 0.0)
                    routes.Add((a, b, length));
            }
        }

        var supported = new bool[joints.Length];
        var stranded = new List<int>();
        bool hasSupports = supports is not null && supports.GetLength(0) > 0;
        if (hasSupports)
        {
            for (int s = 0; s < supports!.GetLength(0); s++)
            {
                int joint = welder.Find(Vec.Row(supports, s));
                if (joint >= 0)
                    supported[joint] = true;
                else
                    stranded.Add(s);
            }
        }

        return new StructureGraph
        {
            Joints = joints,
            Spread = welder.Spread.ToArray(),
            LineCount = lines,
            Outline = outline,
            ElementJoints = elementJoints,
            Degenerate = degenerate,
            DuplicateOf = duplicateOf,
            Valence = incident.Select(here => here.Count).ToArray(),
            Supported = supported,
            HasSupports = hasSupports,
            StrandedSupports = stranded.ToArray(),
            Elements = elementGraph,
            Routes = WeightedGraph.FromEdges(Math.Max(joints.Length, 1), routes),
        };
    }

    /// <summary>
    /// Joints resting along a segment's interior with no corner there — a member
    /// framing into the middle of another, an edge beam meeting a panel partway.
    /// Those are connections the model clearly meant, so they are read as joints of
    /// the element they rest on. Keyed by element and edge, ordered along the edge.
    /// </summary>
    private static Dictionary<(int Element, int Edge), int[]> Bearings(
        Vec[] joints, List<(int Element, int Edge, int A, int B)> segments, int[][] corners, double join)
    {
        var found = new Dictionary<(int, int), List<(int Joint, double Along)>>();
        if (segments.Count == 0 || joints.Length == 0)
            return new Dictionary<(int, int), int[]>();

        // About half a typical segment, so a segment's box covers a handful of
        // cells, and never below the join distance.
        var lengths = segments.Select(s => joints[s.A].DistanceTo(joints[s.B])).OrderBy(l => l).ToArray();
        double cell = Math.Max(join, 0.5 * lengths[lengths.Length / 2]);

        var grid = new Dictionary<(long, long, long), List<int>>();
        for (int j = 0; j < joints.Length; j++)
        {
            var key = Cell(joints[j], cell);
            if (!grid.TryGetValue(key, out var here))
                grid[key] = here = new List<int>();
            here.Add(j);
        }

        foreach (var (element, edge, a, b) in segments)
        {
            var pa = joints[a];
            var pb = joints[b];
            double length = pa.DistanceTo(pb);
            if (length <= 2.0 * join)
                continue;

            var own = new HashSet<int>(corners[element]);
            foreach (int j in Near(grid, joints, pa, pb, join, cell))
            {
                if (own.Contains(j))
                    continue;

                var (distance, along) = PointToSegment(joints[j], pa, pb);
                if (distance > join || along <= join || along >= length - join)
                    continue;

                if (!found.TryGetValue((element, edge), out var list))
                    found[(element, edge)] = list = new List<(int, double)>();
                list.Add((j, along));
            }
        }

        return found.ToDictionary(pair => pair.Key, pair => pair.Value.OrderBy(hit => hit.Along).Select(hit => hit.Joint).ToArray());
    }

    private static IEnumerable<int> Near(
        Dictionary<(long, long, long), List<int>> grid, Vec[] joints, Vec a, Vec b, double margin, double size)
    {
        var low = Cell(new Vec(Math.Min(a.X, b.X) - margin, Math.Min(a.Y, b.Y) - margin, Math.Min(a.Z, b.Z) - margin), size);
        var high = Cell(new Vec(Math.Max(a.X, b.X) + margin, Math.Max(a.Y, b.Y) + margin, Math.Max(a.Z, b.Z) + margin), size);

        long cells = (high.Item1 - low.Item1 + 1) * (high.Item2 - low.Item2 + 1) * (high.Item3 - low.Item3 + 1);
        if (cells > MaximumCellsPerSegment)
        {
            // A long diagonal through a dense model: cheaper to check every joint
            // than to visit every cell of its box.
            for (int j = 0; j < joints.Length; j++)
                yield return j;
            yield break;
        }

        for (long x = low.Item1; x <= high.Item1; x++)
            for (long y = low.Item2; y <= high.Item2; y++)
                for (long z = low.Item3; z <= high.Item3; z++)
                    if (grid.TryGetValue((x, y, z), out var here))
                        foreach (int j in here)
                            yield return j;
    }

    /// <summary>Area of a joint ring, by Newell's method — exact for a planar ring, the projected area otherwise.</summary>
    public static double Area(Vec[] joints, int[] ring)
    {
        var normal = new Vec(0, 0, 0);
        for (int i = 0; i < ring.Length; i++)
            normal += joints[ring[i]].Cross(joints[ring[(i + 1) % ring.Length]]);
        return 0.5 * normal.Length;
    }

    /// <summary>Distance from p to segment ab, and how far along ab its nearest point is.</summary>
    private static (double Distance, double Along) PointToSegment(Vec p, Vec a, Vec b)
    {
        var ab = b - a;
        double length = ab.Length;
        if (length == 0.0)
            return (p.DistanceTo(a), 0.0);

        double along = Math.Clamp((p - a).Dot(ab) / length, 0.0, length);
        return (p.DistanceTo(a + (along / length) * ab), along);
    }

    private static (long, long, long) Cell(Vec p, double size)
        => ((long)Math.Floor(p.X / size), (long)Math.Floor(p.Y / size), (long)Math.Floor(p.Z / size));

    /// <summary>
    /// Points welded into joints on a grid of join-sized cells, so each point checks
    /// the 27 cells around it rather than every joint so far. A point joins the
    /// nearest joint within the join distance, the earliest on a tie, which makes
    /// the numbering a function of the input order alone.
    /// </summary>
    private sealed class Welder
    {
        private readonly double _join;
        private readonly Dictionary<(long, long, long), List<int>> _grid = new();

        public Welder(double join) => _join = join;

        public List<Vec> Positions { get; } = new();

        public List<double> Spread { get; } = new();

        public int Weld(Vec p)
        {
            int existing = Find(p);
            if (existing >= 0)
            {
                Spread[existing] = Math.Max(Spread[existing], p.DistanceTo(Positions[existing]));
                return existing;
            }

            int joint = Positions.Count;
            Positions.Add(p);
            Spread.Add(0.0);

            var cell = Cell(p, _join);
            if (!_grid.TryGetValue(cell, out var here))
                _grid[cell] = here = new List<int>();
            here.Add(joint);

            return joint;
        }

        /// <summary>The nearest joint within the join distance, or -1.</summary>
        public int Find(Vec p)
        {
            int best = -1;
            double bestDistance = double.PositiveInfinity;
            var (cx, cy, cz) = Cell(p, _join);

            for (long dx = -1; dx <= 1; dx++)
            {
                for (long dy = -1; dy <= 1; dy++)
                {
                    for (long dz = -1; dz <= 1; dz++)
                    {
                        if (!_grid.TryGetValue((cx + dx, cy + dy, cz + dz), out var joints))
                            continue;

                        foreach (int joint in joints)
                        {
                            double distance = p.DistanceTo(Positions[joint]);
                            if (distance <= _join && (distance < bestDistance || (distance == bestDistance && joint < best)))
                            {
                                best = joint;
                                bestDistance = distance;
                            }
                        }
                    }
                }
            }

            return best;
        }
    }
}
