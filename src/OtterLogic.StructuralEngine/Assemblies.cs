using OtterLogic.MachineLearning.Decomposition;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// The members that act as one body: every set of lines triangulated together in a
/// plane, read as a single assembly with a span and a depth of its own.
/// <para>
/// A truss is not three kinds of member that happen to touch. Its chords and webs
/// carry nothing apart and everything together, so the question "what does this rest
/// on" has to be asked of the truss, not of a diagonal inside it — asked of the
/// diagonal, the answer is another diagonal, and the hierarchy comes out as a staircase
/// of webs. What makes the members one body is triangulation: three lines closing a
/// triangle cannot move against each other, and triangles sharing a side in the same
/// plane cannot either. That is rigidity, true of any pin-jointed thing, and it is the
/// only test applied — nothing here knows a truss from a braced bay from a lattice mast.
/// </para>
/// <para>
/// Triangles are joined only across a shared side and only while they stay in one
/// plane, give or take the turn a faceted curve takes at each bay. The plane matters
/// because a roof is triangulated twice: each truss in its own upright plane, and the
/// bracing in the plane of the roof, with the top chords in both. Joined regardless of
/// plane, the bracing would weld every truss into one body. Kept apart, a member that
/// runs through two planes is given to the more upright one, because a triangulated
/// plane resists load in its own plane and the load here is gravity.
/// </para>
/// <para>
/// Every other member is an assembly of one. Each assembly of several gets its own
/// frame — the span it runs along, and the depth across the span measured as near plumb
/// as the span allows — so where a member sits in the depth, and whether it runs with
/// the span or across it, can be said without reference to the world's x and y.
/// </para>
/// </summary>
public sealed class Assemblies
{
    /// <summary>
    /// Cosine of the most two triangles' planes may differ and still be one body:
    /// twenty degrees, which takes a ring truss faceted into eighteen bays or more
    /// round its curve and stops well short of the right angle between a truss and
    /// the bracing over it.
    /// </summary>
    private static readonly double Coplanar = Math.Cos(20.0 * Math.PI / 180.0);

    /// <summary>
    /// A triangle this thin against its longest side — a sine of about one degree — is
    /// three lines drawn nearly over each other, not a panel.
    /// </summary>
    private const double Sliver = 0.02;

    /// <summary>A member belongs to a body when at least this share of its length is triangulated into it.</summary>
    private const double Coverage = 0.5;

    private Assemblies()
    {
    }

    /// <summary>Number of assemblies, those of one member included.</summary>
    public int Count => Members.Length;

    /// <summary>The assembly each member belongs to. Numbered by their lowest member.</summary>
    public int[] Of { get; private init; } = null!;

    /// <summary>Each assembly's members, ascending.</summary>
    public int[][] Members { get; private init; } = null!;

    /// <summary>
    /// Per member, where it sits in its assembly's depth: -1 at the bottom, +1 at the
    /// top, 0 midway and for a member that is an assembly by itself.
    /// </summary>
    public double[] DepthPosition { get; private init; } = null!;

    /// <summary>
    /// Per member, how far it runs with its assembly's span rather than across it: the
    /// squared cosine between the two, 1 for a member that is an assembly by itself.
    /// </summary>
    public double[] AlongSpan { get; private init; } = null!;

    public static Assemblies Read(StructureGraph structure, PhysicalMembers members)
    {
        var joints = structure.Joints;

        // Line stretches between consecutive joints, and the elements drawn along each.
        var owners = new Dictionary<(int, int), List<int>>();
        var neighbours = new SortedSet<int>[joints.Length];
        foreach (var (element, a, b) in structure.Segments())
        {
            if (structure.IsSurface(element) || structure.Degenerate[element] || structure.DuplicateOf[element] >= 0)
                continue;

            var key = (Math.Min(a, b), Math.Max(a, b));
            if (!owners.TryGetValue(key, out var list))
                owners[key] = list = new List<int>();
            list.Add(element);

            (neighbours[a] ??= new SortedSet<int>()).Add(b);
            (neighbours[b] ??= new SortedSet<int>()).Add(a);
        }

        // Every triangle once, corners ascending, in an order the joints alone decide.
        var triangles = new List<(int A, int B, int C, Vec Normal, double Area)>();
        for (int a = 0; a < joints.Length; a++)
        {
            if (neighbours[a] is null)
                continue;

            var above = neighbours[a].Where(j => j > a).ToArray();
            for (int p = 0; p < above.Length; p++)
            {
                for (int q = p + 1; q < above.Length; q++)
                {
                    int b = above[p], c = above[q];
                    if (!neighbours[b]!.Contains(c))
                        continue;

                    var ab = joints[b] - joints[a];
                    var ac = joints[c] - joints[a];
                    var cross = ab.Cross(ac);
                    double longest = Math.Max(Math.Max(ab.Length, ac.Length), joints[b].DistanceTo(joints[c]));
                    if (cross.Length <= Sliver * longest * longest)
                        continue;

                    triangles.Add((a, b, c, cross / cross.Length, 0.5 * cross.Length));
                }
            }
        }

        // Triangles sharing a side in one plane are one body.
        var body = Enumerable.Range(0, triangles.Count).ToArray();
        int Find(int t)
        {
            while (body[t] != t)
                t = body[t] = body[body[t]];
            return t;
        }

        var sides = new Dictionary<(int, int), List<int>>();
        for (int t = 0; t < triangles.Count; t++)
        {
            var (a, b, c, _, _) = triangles[t];
            foreach (var side in new[] { (a, b), (a, c), (b, c) })
            {
                if (!sides.TryGetValue(side, out var list))
                    sides[side] = list = new List<int>();
                list.Add(t);
            }
        }

        foreach (var sharing in sides.Values)
            for (int p = 0; p < sharing.Count; p++)
                for (int q = p + 1; q < sharing.Count; q++)
                    if (Math.Abs(triangles[sharing[p]].Normal.Dot(triangles[sharing[q]].Normal)) >= Coplanar)
                    {
                        int low = Math.Min(Find(sharing[p]), Find(sharing[q]));
                        body[Find(sharing[p])] = low;
                        body[Find(sharing[q])] = low;
                    }

        // Per body: how upright its plane is, and how much of each member's length it triangulates.
        var area = new Dictionary<int, double>();
        var flat = new Dictionary<int, double>();
        var covered = new Dictionary<(int Body, int Member), HashSet<(int, int)>>();
        for (int t = 0; t < triangles.Count; t++)
        {
            int root = Find(t);
            var (a, b, c, normal, size) = triangles[t];
            area[root] = area.GetValueOrDefault(root) + size;
            flat[root] = flat.GetValueOrDefault(root) + size * Math.Abs(normal.Z);

            foreach (var side in new[] { (a, b), (a, c), (b, c) })
                foreach (int element in owners[side])
                {
                    var key = (root, members.Of[element]);
                    if (!covered.TryGetValue(key, out var set))
                        covered[key] = set = new HashSet<(int, int)>();
                    set.Add(side);
                }
        }

        var chosen = Enumerable.Repeat(-1, members.Count).ToArray();
        var best = new (double Upright, double Share)[members.Count];
        foreach (var ((root, member), set) in covered.OrderBy(pair => pair.Key.Body).ThenBy(pair => pair.Key.Member))
        {
            double share = set.Sum(side => joints[side.Item1].DistanceTo(joints[side.Item2])) / members.Length[member];
            if (share < Coverage)
                continue;

            double upright = 1.0 - flat[root] / area[root];
            bool better = chosen[member] < 0
                || upright > best[member].Upright + 1e-9
                || (Math.Abs(upright - best[member].Upright) <= 1e-9 && share > best[member].Share + 1e-9);

            if (better)
            {
                chosen[member] = root;
                best[member] = (upright, share);
            }
        }

        // A body needs two members to be an assembly; one member triangulated with
        // nothing that stayed is just that member.
        var size2 = chosen.Where(root => root >= 0).GroupBy(root => root).ToDictionary(g => g.Key, g => g.Count());
        var of = new int[members.Count];
        var number = new Dictionary<int, int>();
        var grouped = new List<List<int>>();
        for (int m = 0; m < members.Count; m++)
        {
            int root = chosen[m];
            if (root >= 0 && size2[root] >= 2)
            {
                if (!number.TryGetValue(root, out int index))
                {
                    number[root] = index = grouped.Count;
                    grouped.Add(new List<int>());
                }

                of[m] = index;
                grouped[index].Add(m);
            }
            else
            {
                of[m] = grouped.Count;
                grouped.Add(new List<int> { m });
            }
        }

        var depth = new double[members.Count];
        var alongSpan = Enumerable.Repeat(1.0, members.Count).ToArray();
        foreach (var group in grouped.Where(g => g.Count >= 2))
            Frame(structure, members, group, depth, alongSpan);

        return new Assemblies
        {
            Of = of,
            Members = grouped.Select(g => g.ToArray()).ToArray(),
            DepthPosition = depth,
            AlongSpan = alongSpan,
        };
    }

    /// <summary>
    /// The assembly's own frame, and each member's place in it. The span is the
    /// direction its joints spread furthest along. The depth is plumb with the span
    /// taken out — so an inclined truss's depth is still read square to its chords —
    /// unless the span is itself near plumb, a lattice mast, where the depth is the
    /// next direction the joints spread along.
    /// </summary>
    private static void Frame(StructureGraph structure, PhysicalMembers members, List<int> group, double[] depth, double[] alongSpan)
    {
        var joints = group.SelectMany(m => members.Run[m]).Distinct().OrderBy(j => j).ToArray();
        var points = new double[joints.Length, 3];
        for (int i = 0; i < joints.Length; i++)
            (points[i, 0], points[i, 1], points[i, 2]) = (structure.Joints[joints[i]].X, structure.Joints[joints[i]].Y, structure.Joints[joints[i]].Z);

        var axes = PrincipalComponents.FitCount(points, 2, whiten: false);
        Vec Axis(int c) => new(axes.Components[c, 0], axes.Components[c, 1], axes.Components[c, 2]);

        var centre = new Vec(axes.Mean[0], axes.Mean[1], axes.Mean[2]);
        var span = Axis(0);
        var up = new Vec(0, 0, 1);
        var across = up - up.Dot(span) * span;

        if (across.Length < 0.5 && axes.Count > 1)
            across = Axis(1);
        if (across.Length <= 0.0)
            return;

        across = across / across.Length;
        if (across.Z < 0.0)
            across = -1.0 * across;

        double reach = joints.Max(j => Math.Abs((structure.Joints[j] - centre).Dot(across)));

        foreach (int m in group)
        {
            var run = members.Run[m];
            if (reach > 0.0)
                depth[m] = Math.Clamp(run.Average(j => (structure.Joints[j] - centre).Dot(across)) / reach, -1.0, 1.0);

            double total = 0.0, with = 0.0;
            for (int p = 0; p + 1 < run.Length; p++)
            {
                var piece = structure.Joints[run[p + 1]] - structure.Joints[run[p]];
                double length = piece.Length;
                if (length <= 0.0)
                    continue;

                double cosine = piece.Dot(span) / length;
                with += length * cosine * cosine;
                total += length;
            }

            if (total > 0.0)
                alongSpan[m] = with / total;
        }
    }
}
