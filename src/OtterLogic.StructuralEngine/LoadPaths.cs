using OtterLogic.Graphs;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// Where the model's weight goes on its way to the supports: how much passes along
/// each element, which assembly hands its load to which, and so how many hand-overs
/// stand between each assembly and the ground.
/// <para>
/// This is the reading an engineer makes before any analysis — purlins sit on rafters,
/// rafters on girders, girders on columns — made the way it is made by eye, from the
/// supports up, and without a rule for any of those words. Every element is given its
/// own weight, the supports are grounded, and <see cref="PotentialFlow"/> finds how that
/// weight drains. It is not a structural analysis and needs none of what one needs: no
/// sections, no releases, no stable model. A mechanism drains as well as a frame,
/// which is the point of a tool meant for the model that does not solve yet.
/// </para>
/// <para>
/// Three judgements are made here, all about structures in general. <b>Weight</b> is
/// length, a surface's its area over a typical element's length, so a panel a bay
/// square weighs about what a beam a bay long does. <b>Conductance</b> is one over
/// length, scaled by how the stretch would carry a vertical load: by its axis in
/// proportion as it is upright, by bending in proportion as it is level, and bending is
/// taken a hundred times the softer (<see cref="BendingShare"/>). That one ratio is what
/// lets load run down a column rather than sideways along a beam to the next column,
/// and along a truss's diagonals rather than its chords — without it the flow treats a
/// storey of beams as a short cut between columns and every level folds into one.
/// <b>Hand-over</b> is read joint by joint: a member that takes more away from a joint
/// than it brought, beyond its own weight there, is receiving; one that brings more
/// than it takes is delivering; and what is delivered is shared among the receivers,
/// the ground included where the joint is supported.
/// </para>
/// <para>
/// Hand-overs are summed between assemblies, only the substantial ones kept, and the
/// loops folded by <see cref="Condensation"/>: assemblies that lean on each other — a
/// grillage, the layers of a space truss — share a level, and the level is how long a
/// chain of hand-overs hangs below. Nothing resting on anything but the ground is
/// level 0.
/// </para>
/// </summary>
public sealed class LoadPaths
{
    /// <summary>
    /// Stiffness of a stretch carrying load across its axis, over carrying it along.
    /// For a member of slenderness L/r the ratio is 12 (r/L)², and members are sized to
    /// much the same slenderness whatever their length — about 35 gives a hundredth.
    /// The exact figure matters little; that it is far below one matters a great deal.
    /// </summary>
    private const double BendingShare = 0.01;

    /// <summary>
    /// A hand-over counts towards the hierarchy when it is at least this share of the
    /// largest the same assembly makes. A rafter hands nearly everything to its columns
    /// and, where its neighbour sits lower, a trickle sideways through each purlin; the
    /// trickle is real but it is not what the rafter rests on, and kept, it would put
    /// the rafter above the purlins that rest on it.
    /// </summary>
    private const double SubstantialShare = 0.1;

    private LoadPaths()
    {
    }

    /// <summary>Whether anything drained: supports were given and something reaches one.</summary>
    public bool Traced { get; private init; }

    /// <summary>False when the flow stopped at its iteration cap short of its tolerance.</summary>
    public bool Converged { get; private init; } = true;

    /// <summary>
    /// Per element, the share of the whole model's weight passing along it, its own included, 0 to 1,
    /// averaged along its length. Zero when nothing was traced, or it reaches no support.
    /// </summary>
    public double[] ElementFlow { get; private init; } = null!;

    /// <summary>Per member, the same, averaged along the member.</summary>
    public double[] MemberFlow { get; private init; } = null!;

    /// <summary>
    /// Per assembly, the hand-overs between it and the ground: 0 rests on the supports.
    /// -1 when nothing was traced, or it reaches no support.
    /// </summary>
    public int[] Level { get; private init; } = null!;

    /// <summary>The highest level found; -1 when nothing was traced.</summary>
    public int Levels { get; private init; } = -1;

    public static LoadPaths Trace(StructureGraph structure, ElementGeometry geometry, PhysicalMembers members, Assemblies assemblies)
    {
        int n = structure.ElementCount;
        var joints = structure.Joints;
        var untraced = new LoadPaths
        {
            ElementFlow = new double[n],
            MemberFlow = new double[members.Count],
            Level = Enumerable.Repeat(-1, assemblies.Count).ToArray(),
        };

        var grounded = Enumerable.Range(0, joints.Length).Where(j => structure.Supported[j]).ToArray();
        if (grounded.Length == 0)
            return untraced;

        bool Counts(int e) => !structure.Degenerate[e] && structure.DuplicateOf[e] < 0;

        var sizes = Enumerable.Range(0, n).Where(Counts).Select(e => geometry.Size[e]).Where(s => s > 0.0).OrderBy(s => s).ToArray();
        if (sizes.Length == 0)
            return untraced;
        double typical = sizes[sizes.Length / 2];

        // Each stretch's conductance, and each element's weight lumped at its joints.
        var stretches = new List<(int Element, int A, int B, double Conductance)>();
        var conductance = new Dictionary<(int, int), double>();
        var ownWeight = new Dictionary<(int Member, int Joint), double>();
        var injection = new double[joints.Length];
        var elementWeight = new double[n];

        void Weigh(int element, int joint, double weight)
        {
            injection[joint] += weight;
            elementWeight[element] += weight;
            var key = (members.Of[element], joint);
            ownWeight[key] = ownWeight.GetValueOrDefault(key) + weight;
        }

        foreach (var (element, a, b) in structure.Segments())
        {
            if (!Counts(element))
                continue;

            var along = joints[b] - joints[a];
            double length = along.Length;
            if (length <= 0.0)
                continue;

            double upright = along.Z * along.Z / (length * length);
            if (structure.IsSurface(element))
                upright = Math.Max(upright, Math.Min(1.0, 2.0 * geometry.Extent[element].Z));
            else
            {
                Weigh(element, a, 0.5 * length);
                Weigh(element, b, 0.5 * length);
            }

            double c = (upright + BendingShare * (1.0 - upright)) / length;
            stretches.Add((element, a, b, c));

            var key = (Math.Min(a, b), Math.Max(a, b));
            conductance[key] = conductance.GetValueOrDefault(key) + c;
        }

        for (int e = structure.LineCount; e < n; e++)
        {
            if (!Counts(e))
                continue;

            double weight = geometry.Size[e] * geometry.Size[e] / typical;
            foreach (int j in structure.ElementJoints[e])
                Weigh(e, j, weight / structure.ElementJoints[e].Length);
        }

        if (conductance.Count == 0)
            return untraced;

        var graph = WeightedGraph.FromEdges(joints.Length, conductance.Select(pair => (pair.Key.Item1, pair.Key.Item2, pair.Value)));
        var flow = PotentialFlow.Solve(graph, injection, grounded);

        double total = Enumerable.Range(0, joints.Length).Where(j => flow.Reached[j] && !structure.Supported[j]).Sum(j => injection[j]);
        if (!(total > 0.0))
            return untraced;

        // What passes along each element, and what each member takes from or brings to each joint.
        var passing = new double[n];
        var measured = new double[n];
        var taken = new Dictionary<(int Member, int Joint), double>();

        foreach (var (element, a, b, c) in stretches)
        {
            double along = flow.Flow(a, b, c);
            double length = joints[a].DistanceTo(joints[b]);
            passing[element] += Math.Abs(along) * length;
            measured[element] += length;

            int member = members.Of[element];
            taken[(member, a)] = taken.GetValueOrDefault((member, a)) + along;
            taken[(member, b)] = taken.GetValueOrDefault((member, b)) - along;
        }

        // An element's own weight was lumped at its joints, so none of it shows in what
        // passes along it — a purlin between two equal trusses would read as carrying
        // nothing. Spread along the element as it really is, its own weight alone puts a
        // mean of a quarter of itself through any section; that is added back.
        var elementFlow = new double[n];
        for (int e = 0; e < n; e++)
            if (measured[e] > 0.0)
                elementFlow[e] = Math.Min(1.0, (passing[e] / measured[e] + 0.25 * elementWeight[e]) / total);

        var memberFlow = new double[members.Count];
        for (int m = 0; m < members.Count; m++)
        {
            double weight = members.Elements[m].Sum(e => measured[e]);
            if (weight > 0.0)
                memberFlow[m] = members.Elements[m].Sum(e => elementFlow[e] * measured[e]) / weight;
        }

        // Hand-overs, joint by joint, summed between assemblies.
        double negligible = 1e-9 * total;
        var handed = new Dictionary<(int From, int To), double>();
        var toGround = new double[assemblies.Count];

        for (int j = 0; j < joints.Length; j++)
        {
            if (!flow.Reached[j])
                continue;

            var here = members.JointMembers[j];
            var net = new double[here.Length];
            double receiving = 0.0, balance = 0.0;
            for (int p = 0; p < here.Length; p++)
            {
                net[p] = taken.GetValueOrDefault((here[p], j)) - ownWeight.GetValueOrDefault((here[p], j));
                balance += net[p];
                if (net[p] > negligible)
                    receiving += net[p];
            }

            // Whatever the members bring that none of them takes away has gone to ground.
            double ground = structure.Supported[j] ? Math.Max(0.0, -balance) : 0.0;
            receiving += ground;
            if (!(receiving > negligible))
                continue;

            for (int p = 0; p < here.Length; p++)
            {
                if (net[p] >= -negligible)
                    continue;

                int from = assemblies.Of[here[p]];
                toGround[from] += -net[p] * ground / receiving;

                for (int q = 0; q < here.Length; q++)
                {
                    int to = assemblies.Of[here[q]];
                    if (net[q] > negligible && to != from)
                        handed[(from, to)] = handed.GetValueOrDefault((from, to)) - net[p] * net[q] / receiving;
                }
            }
        }

        var largest = (double[])toGround.Clone();
        foreach (var ((from, _), amount) in handed)
            largest[from] = Math.Max(largest[from], amount);

        var order = Condensation.Of(assemblies.Count,
            handed.Where(pair => pair.Value >= SubstantialShare * largest[pair.Key.From]).Select(pair => pair.Key));

        var level = new int[assemblies.Count];
        for (int a = 0; a < assemblies.Count; a++)
        {
            bool reaches = assemblies.Members[a].Any(m => members.Run[m].Any(j => flow.Reached[j]));
            level[a] = reaches ? order.HeightOf(a) : -1;
        }

        return new LoadPaths
        {
            Traced = true,
            Converged = flow.Converged,
            ElementFlow = elementFlow,
            MemberFlow = memberFlow,
            Level = level,
            Levels = level.Max(),
        };
    }
}
