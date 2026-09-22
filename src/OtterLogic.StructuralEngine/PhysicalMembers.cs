using OtterLogic.Graphs;
using OtterLogic.Unsupervised.Clustering;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// The members an engineer would see in the model: every run of lines that carries
/// straight on through its joints read as one thing, however many pieces it was
/// drawn in.
/// <para>
/// An analysis model breaks a sixty-metre chord at every joint something frames into,
/// so element by element the chord is twenty three-metre sticks that look exactly like
/// the purlins beside them. Nothing measured on a stick can tell them apart, and that
/// is most of why a grouping by element fails on a structure with a hierarchy. Read as
/// members the difference is plain: one is sixty metres long with twenty things framing
/// into it, the other three metres long and framing into something at both ends.
/// </para>
/// <para>
/// Continuity is the only thing read, and it is a fact about the geometry, not about a
/// kind of structure: at each joint, line ends are paired with the end they most nearly
/// continue, straightest pair first, each end once. How much of a turn still counts as
/// carrying on is learned from the model — the turns at every line end fall into a band
/// near zero and a band of real corners, and the limit sits in the gap between — so a
/// faceted arch chains and a rafter does not chain into its column. Surfaces, and lines
/// with nothing to continue, are members of one.
/// </para>
/// </summary>
public sealed class PhysicalMembers
{
    /// <summary>
    /// Below this, degrees, a turn is a continuation whatever the model's own turns
    /// say: drawing noise, precamber, a splice. Without the floor a model of perfectly
    /// straight runs would learn a limit of zero and break at the first rounded coordinate.
    /// </summary>
    private const double SmallestLimit = 5.0;

    /// <summary>
    /// Above this, degrees, a turn is a corner whatever the model's own turns say — a
    /// knee, an eaves, a change of member. Thirty still chains a ring drawn in twelve
    /// chords and stops short of a hexagon, whose sides nobody reads as one member.
    /// </summary>
    private const double LargestLimit = 30.0;

    private PhysicalMembers()
    {
    }

    /// <summary>Number of members.</summary>
    public int Count => Elements.Length;

    /// <summary>The member each element belongs to.</summary>
    public int[] Of { get; private init; } = null!;

    /// <summary>Each member's elements, in order along it. Members are numbered by their lowest element.</summary>
    public int[][] Elements { get; private init; } = null!;

    /// <summary>The joints along each member in order; a surface's boundary. A closed run does not repeat its first joint.</summary>
    public int[][] Run { get; private init; } = null!;

    /// <summary>Whether the run closes on itself — a ring, or a surface's boundary.</summary>
    public bool[] Closed { get; private init; } = null!;

    /// <summary>Length along the member; for a surface the square root of its area.</summary>
    public double[] Length { get; private init; } = null!;

    /// <summary>End to end over length along: one for a straight member, towards zero for an arch, zero for a ring.</summary>
    public double[] Straightness { get; private init; } = null!;

    /// <summary>How many of a line member's two ends land partway along another line member, 0 to 2.</summary>
    public int[] EndsBearing { get; private init; } = null!;

    /// <summary>How many other members end partway along this one.</summary>
    public int[] Carried { get; private init; } = null!;

    /// <summary>How many other members this one meets anywhere.</summary>
    public int[] Meets { get; private init; } = null!;

    /// <summary>Distinct members at each joint, ascending.</summary>
    public int[][] JointMembers { get; private init; } = null!;

    /// <summary>The turn, in degrees, up to which a line was read as carrying on. NaN when nothing was chained.</summary>
    public double TurnLimit { get; private init; }

    /// <summary>
    /// A node per member, an edge wherever two meet, weighted as
    /// <see cref="StructureGraph.Elements"/> is: a joint shared by v members adds
    /// 1 / (v − 1) to each pair.
    /// </summary>
    public WeightedGraph Graph { get; private init; } = null!;

    /// <param name="chain">False reads every element as its own member.</param>
    public static PhysicalMembers Read(StructureGraph structure, ElementGeometry geometry, bool chain = true)
    {
        int n = structure.ElementCount;
        var partner = new (int Element, int End)[n, 2];
        for (int e = 0; e < n; e++)
            partner[e, 0] = partner[e, 1] = (-1, -1);

        double limit = chain ? Pair(structure, partner) : double.NaN;

        var of = Enumerable.Repeat(-1, n).ToArray();
        var elements = new List<int[]>();
        var runs = new List<int[]>();
        var closed = new List<bool>();

        for (int e = 0; e < n; e++)
        {
            if (of[e] >= 0)
                continue;

            int member = elements.Count;

            if (structure.IsSurface(e) || structure.Outline[e].Length < 2)
            {
                of[e] = member;
                elements.Add(new[] { e });
                runs.Add(structure.Outline[e]);
                closed.Add(structure.IsSurface(e));
                continue;
            }

            // Back along the chain from e's start to a free end, or round to e again.
            int first = e, firstEnd = 0;
            bool ring = false;
            while (partner[first, firstEnd].Element >= 0)
            {
                var (before, beforeEnd) = partner[first, firstEnd];
                if (before == e)
                {
                    ring = true;
                    break;
                }

                first = before;
                firstEnd = 1 - beforeEnd;
            }

            if (ring)
                (first, firstEnd) = (e, 0);

            var chainElements = new List<int>();
            var run = new List<int>();
            int at = first, enteredBy = firstEnd;
            while (true)
            {
                of[at] = member;
                chainElements.Add(at);

                var outline = structure.Outline[at];
                for (int p = 0; p < outline.Length; p++)
                {
                    int joint = enteredBy == 0 ? outline[p] : outline[outline.Length - 1 - p];
                    if (run.Count == 0 || run[^1] != joint)
                        run.Add(joint);
                }

                var (after, afterEnd) = partner[at, 1 - enteredBy];
                if (after < 0 || after == first)
                    break;

                (at, enteredBy) = (after, afterEnd);
            }

            if (ring && run.Count > 1 && run[^1] == run[0])
                run.RemoveAt(run.Count - 1);

            elements.Add(chainElements.ToArray());
            runs.Add(run.ToArray());
            closed.Add(ring);
        }

        int count = elements.Count;
        var length = new double[count];
        var straightness = new double[count];
        for (int m = 0; m < count; m++)
        {
            length[m] = elements[m].Sum(e => geometry.Size[e]);
            var run = runs[m];
            bool line = !structure.IsSurface(elements[m][0]);
            straightness[m] = !line || length[m] <= 0.0 || run.Length < 2 ? 1.0
                : closed[m] ? 0.0
                : Math.Min(1.0, structure.Joints[run[0]].DistanceTo(structure.Joints[run[^1]]) / length[m]);
        }

        var jointMembers = new SortedSet<int>[structure.Joints.Length];
        for (int e = 0; e < n; e++)
            foreach (int j in structure.ElementJoints[e])
                (jointMembers[j] ??= new SortedSet<int>()).Add(of[e]);

        var atJoint = jointMembers.Select(set => set?.ToArray() ?? Array.Empty<int>()).ToArray();

        // Joints partway along a line member: every joint of its run but the two ends,
        // and every joint of a ring.
        var along = new HashSet<int>[count];
        for (int m = 0; m < count; m++)
        {
            along[m] = new HashSet<int>();
            if (structure.IsSurface(elements[m][0]))
                continue;

            var run = runs[m];
            for (int p = 0; p < run.Length; p++)
                if (closed[m] || (p > 0 && p < run.Length - 1))
                    along[m].Add(run[p]);
        }

        var endsBearing = new int[count];
        var carried = new HashSet<int>[count];
        for (int m = 0; m < count; m++)
        {
            if (structure.IsSurface(elements[m][0]) || closed[m] || runs[m].Length < 2)
                continue;

            foreach (int end in new[] { runs[m][0], runs[m][^1] }.Distinct())
            {
                bool bears = false;
                foreach (int other in atJoint[end])
                {
                    if (other == m || !along[other].Contains(end))
                        continue;

                    bears = true;
                    (carried[other] ??= new HashSet<int>()).Add(m);
                }

                if (bears)
                    endsBearing[m]++;
            }
        }

        var pairs = new Dictionary<long, double>();
        var meets = new HashSet<int>[count];
        foreach (var here in atJoint)
        {
            if (here.Length < 2)
                continue;

            double share = 1.0 / (here.Length - 1);
            for (int p = 0; p < here.Length; p++)
            {
                for (int q = p + 1; q < here.Length; q++)
                {
                    long key = (long)here[p] * count + here[q];
                    pairs[key] = pairs.TryGetValue(key, out double weight) ? weight + share : share;
                    (meets[here[p]] ??= new HashSet<int>()).Add(here[q]);
                    (meets[here[q]] ??= new HashSet<int>()).Add(here[p]);
                }
            }
        }

        return new PhysicalMembers
        {
            Of = of,
            Elements = elements.ToArray(),
            Run = runs.ToArray(),
            Closed = closed.ToArray(),
            Length = length,
            Straightness = straightness,
            EndsBearing = endsBearing,
            Carried = carried.Select(set => set?.Count ?? 0).ToArray(),
            Meets = meets.Select(set => set?.Count ?? 0).ToArray(),
            JointMembers = atJoint,
            TurnLimit = limit,
            Graph = WeightedGraph.FromEdges(count, pairs.Select(pair => ((int)(pair.Key / count), (int)(pair.Key % count), pair.Value))),
        };
    }

    /// <summary>
    /// Pairs line ends across each joint, straightest first, and returns the turn limit
    /// it learned. A line end is a candidate only where the line stops: a line running
    /// through a joint that merely rests on it already carries on by itself.
    /// </summary>
    private static double Pair(StructureGraph structure, (int Element, int End)[,] partner)
    {
        var endsAt = new List<(int Element, int End, Vec Away)>[structure.Joints.Length];
        for (int e = 0; e < structure.LineCount; e++)
        {
            if (structure.Degenerate[e] || structure.DuplicateOf[e] >= 0)
                continue;

            var outline = structure.Outline[e];
            var along = structure.Joints[outline[^1]] - structure.Joints[outline[0]];
            double length = along.Length;
            if (length <= 0.0)
                continue;

            along = along / length;
            (endsAt[outline[0]] ??= new()).Add((e, 0, along));
            (endsAt[outline[^1]] ??= new()).Add((e, 1, -1.0 * along));
        }

        var candidates = new List<(double Turn, int Joint, int A, int EndA, int B, int EndB)>();
        var straightest = new Dictionary<(int, int), double>();

        for (int j = 0; j < endsAt.Length; j++)
        {
            var here = endsAt[j];
            if (here is null)
                continue;

            for (int p = 0; p < here.Count; p++)
            {
                for (int q = p + 1; q < here.Count; q++)
                {
                    double cosine = Math.Clamp(-here[p].Away.Dot(here[q].Away), -1.0, 1.0);
                    double turn = Math.Acos(cosine) * 180.0 / Math.PI;
                    candidates.Add((turn, j, here[p].Element, here[p].End, here[q].Element, here[q].End));

                    foreach (var key in new[] { (here[p].Element, here[p].End), (here[q].Element, here[q].End) })
                        if (!straightest.TryGetValue(key, out double best) || turn < best)
                            straightest[key] = turn;
                }
            }
        }

        if (candidates.Count == 0)
            return double.NaN;

        double limit = Limit(straightest.OrderBy(pair => pair.Key).Select(pair => pair.Value).ToArray());

        foreach (var (turn, _, a, endA, b, endB) in candidates.OrderBy(c => c.Turn).ThenBy(c => c.Joint).ThenBy(c => c.A).ThenBy(c => c.B))
        {
            if (turn > limit)
                break;
            if (a == b || partner[a, endA].Element >= 0 || partner[b, endB].Element >= 0)
                continue;

            partner[a, endA] = (b, endB);
            partner[b, endB] = (a, endA);
        }

        return limit;
    }

    /// <summary>
    /// The turn up to which a line carries on: midway across the gap above the lowest
    /// band of turns, kept between the two limits no model is allowed to argue with.
    /// </summary>
    private static double Limit(double[] turns)
    {
        var bands = ValueBands.Fit(turns, new ValueBandsOptions { Resolution = 1.0 }).Bands;
        double learned = bands.Count > 1 ? 0.5 * (bands[0].High + bands[1].Low) : bands[0].High;
        return Math.Clamp(learned, SmallestLimit, LargestLimit);
    }
}
