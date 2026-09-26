namespace OtterLogic.StructuralEngine;

/// <summary>
/// The pieces a member is made in: every run cut where it rests on something that
/// holds it up part of the way along.
/// <para>
/// A run is the right unit for telling what part a member plays — a girder line with
/// twenty things framing into it is not a secondary beam — but it is not what is made,
/// erected or given a section. A girder line passing over five columns is five beams,
/// each spanning column to column; a purlin line drawn straight over four rafters is
/// four purlins. Where the run hands its weight on part of the way along, it bears
/// there, and a piece ends.
/// </para>
/// <para>
/// Where a run rests is read off the load path, joint by joint, never off what the
/// members look like: a run is cut at a joint along it where it hands a substantial
/// share of its weight to the ground, to a member of another assembly, or to a member
/// that itself carries on through the joint. The last two are what keep a truss whole
/// — its chord hands load to its webs at every panel point, but the webs are the same
/// body and stop there — while a beam line over a column is cut whether or not the
/// column carries on above. A column stack is never cut by the beams on it, because it
/// receives there rather than delivers.
/// </para>
/// <para>
/// This reads a model the way a simply connected steel frame is built, and says so:
/// under moment connections a beam line over its columns is one continuous member,
/// which is a detailing decision no geometry shows. One soft spot is known. A beam
/// meeting the top of a column it is triangulated with — the top storey of a braced
/// bay — is one body with that column and ends nowhere in particular, so it is not cut
/// there. Without supports nothing is traced and every run is one piece.
/// </para>
/// </summary>
public sealed class Pieces
{
    /// <summary>
    /// A hand-over cuts a run when it is at least this share of the most the run hands
    /// on at any one joint — the same measure <see cref="LoadPaths"/> uses for what a
    /// member rests on. A beam line hands nearly all its weight to its columns and a
    /// trickle to whatever else it touches; the trickle is real, but the beam does not
    /// end there.
    /// </summary>
    private const double SubstantialShare = 0.1;

    private Pieces()
    {
    }

    /// <summary>Number of pieces.</summary>
    public int Count => Member.Length;

    /// <summary>Per piece, the run it was cut from. Pieces are numbered run by run, in order along each.</summary>
    public int[] Member { get; private init; } = null!;

    /// <summary>Per piece, its joints in order; a surface's boundary. A closed piece does not repeat its first joint.</summary>
    public int[][] Run { get; private init; } = null!;

    /// <summary>Whether the piece closes on itself: an uncut ring, or a surface.</summary>
    public bool[] Closed { get; private init; } = null!;

    /// <summary>Per piece, the elements it covers, in order along it. An element drawn across a cut is in both pieces.</summary>
    public int[][] Elements { get; private init; } = null!;

    /// <summary>Per piece, its length along, bearing to bearing; a surface's square root of area.</summary>
    public double[] Span { get; private init; } = null!;

    /// <summary>
    /// Per piece, the longest stretch along it with nothing framing in and no support —
    /// the length it stands or spans free. The span itself when nothing meets it along the way.
    /// </summary>
    public double[] Unbraced { get; private init; } = null!;

    /// <summary>
    /// Per piece, how far its length stands up rather than lies, 0 to 1: the squared
    /// sine of its slope, averaged along it, which is the share of a vertical load a
    /// stretch takes along its axis rather than across it.
    /// </summary>
    public double[] Upright { get; private init; } = null!;

    /// <summary>Per piece, the share of the model's weight passing along it, averaged along it; zero when nothing was traced.</summary>
    public double[] Flow { get; private init; } = null!;

    /// <summary>Per element, the piece holding most of its length.</summary>
    public int[] Of { get; private init; } = null!;

    /// <summary>Joints at which a run was cut, counted once per run cut there.</summary>
    public int Cuts { get; private init; }

    /// <summary>Whether a load path was traced to cut along. False reads every run as one piece.</summary>
    public bool Traced { get; private init; }

    public static Pieces Cut(
        StructureGraph structure, ElementGeometry geometry, PhysicalMembers members, Assemblies assemblies, LoadPaths paths)
    {
        int count = members.Count;
        var joints = structure.Joints;

        bool IsLine(int m) => !structure.IsSurface(members.Elements[m][0]) && members.Run[m].Length >= 2;

        // Joints each run carries on through: every joint of it but its two ends, every joint of a ring.
        var through = new HashSet<int>[count];
        for (int m = 0; m < count; m++)
        {
            through[m] = new HashSet<int>();
            if (!IsLine(m))
                continue;

            var run = members.Run[m];
            for (int p = 0; p < run.Length; p++)
                if (members.Closed[m] || (p > 0 && p < run.Length - 1))
                    through[m].Add(run[p]);
        }

        // What each run hands on at each joint: all of it, and the part that says it bears there.
        var handed = new Dictionary<(int Member, int Joint), double>();
        var bearing = new Dictionary<(int Member, int Joint), double>();
        foreach (var h in paths.JointHandOvers)
        {
            handed[(h.From, h.Joint)] = handed.GetValueOrDefault((h.From, h.Joint)) + h.Share;

            bool bears = h.To < 0 || assemblies.Of[h.To] != assemblies.Of[h.From] || through[h.To].Contains(h.Joint);
            if (bears)
                bearing[(h.From, h.Joint)] = bearing.GetValueOrDefault((h.From, h.Joint)) + h.Share;
        }

        var most = new double[count];
        foreach (var ((m, _), share) in handed)
            most[m] = Math.Max(most[m], share);

        var member = new List<int>();
        var runs = new List<int[]>();
        var closed = new List<bool>();
        var covers = new List<int[]>();
        var span = new List<double>();
        var unbraced = new List<double>();
        var upright = new List<double>();
        var flow = new List<double>();

        var of = Enumerable.Repeat(-1, structure.ElementCount).ToArray();
        var firstPiece = new int[count];
        var held = new Dictionary<(int Element, int Piece), double>();
        int cuts = 0;

        for (int m = 0; m < count; m++)
        {
            firstPiece[m] = member.Count;
            if (!IsLine(m))
            {
                int e = members.Elements[m][0];
                member.Add(m);
                runs.Add(members.Run[m]);
                closed.Add(members.Closed[m]);
                covers.Add(members.Elements[m]);
                span.Add(members.Length[m]);
                unbraced.Add(members.Length[m]);
                upright.Add(structure.IsSurface(e) ? geometry.Extent[e].Z : 0.0);
                flow.Add(paths.ElementFlow[e]);
                continue;
            }

            var run = members.Run[m];
            int k = run.Length;
            bool ring = members.Closed[m];

            var cut = new bool[k];
            for (int p = 0; p < k; p++)
            {
                if (!through[m].Contains(run[p]))
                    continue;

                double bears = bearing.GetValueOrDefault((m, run[p]));
                if (bears > 0.0 && bears >= SubstantialShare * most[m])
                {
                    cut[p] = true;
                    cuts++;
                }
            }

            // Which element each stretch between neighbouring joints belongs to.
            var owner = new Dictionary<(int, int), int>();
            foreach (int e in members.Elements[m])
            {
                var outline = structure.Outline[e];
                for (int p = 0; p + 1 < outline.Length; p++)
                    owner.TryAdd((Math.Min(outline[p], outline[p + 1]), Math.Max(outline[p], outline[p + 1])), e);
            }

            int segments = ring ? k : k - 1;
            int first = ring ? Array.IndexOf(cut, true) : 0;
            bool whole = ring && first < 0;
            if (first < 0)
                first = 0;

            var stretch = new List<int>();
            for (int t = 0; t < segments; t++)
            {
                int s = (first + t) % k;
                if (t > 0 && cut[s])
                {
                    Close(stretch);
                    stretch.Clear();
                }

                stretch.Add(s);
            }

            Close(stretch);

            void Close(List<int> stretches)
            {
                int piece = member.Count;
                var along = new List<int> { run[stretches[0]] };
                var elements = new List<int>();
                double length = 0.0, standing = 0.0, carried = 0.0;
                double longestFree = 0.0, free = 0.0;

                foreach (int s in stretches)
                {
                    int a = run[s], b = run[(s + 1) % k];
                    var d = joints[b] - joints[a];
                    double l = d.Length;
                    int e = owner.TryGetValue((Math.Min(a, b), Math.Max(a, b)), out int found) ? found : members.Elements[m][0];

                    if (elements.Count == 0 || elements[^1] != e)
                        elements.Add(e);

                    length += l;
                    if (l > 0.0)
                        standing += d.Z * d.Z / l;
                    carried += paths.ElementFlow[e] * l;
                    held[(e, piece)] = held.GetValueOrDefault((e, piece)) + l;

                    free += l;
                    bool last = s == stretches[^1];
                    if (last || structure.Supported[b] || members.JointMembers[b].Length > 1)
                    {
                        longestFree = Math.Max(longestFree, free);
                        free = 0.0;
                    }

                    if (!(whole && last))
                        along.Add(b);
                }

                member.Add(m);
                runs.Add(along.ToArray());
                closed.Add(whole);
                covers.Add(elements.Distinct().ToArray());
                span.Add(length);
                unbraced.Add(longestFree);
                upright.Add(length > 0.0 ? standing / length : 0.0);
                flow.Add(length > 0.0 ? carried / length : 0.0);
            }
        }

        for (int e = 0; e < of.Length; e++)
            of[e] = firstPiece[members.Of[e]];
        foreach (var group in held.GroupBy(pair => pair.Key.Element))
            of[group.Key] = group.OrderByDescending(pair => pair.Value).ThenBy(pair => pair.Key.Piece).First().Key.Piece;

        return new Pieces
        {
            Member = member.ToArray(),
            Run = runs.ToArray(),
            Closed = closed.ToArray(),
            Elements = covers.ToArray(),
            Span = span.ToArray(),
            Unbraced = unbraced.ToArray(),
            Upright = upright.ToArray(),
            Flow = flow.ToArray(),
            Of = of,
            Cuts = cuts,
            Traced = paths.Traced,
        };
    }
}
