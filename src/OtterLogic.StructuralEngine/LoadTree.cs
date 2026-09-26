using OtterLogic.Graphs;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// The load path an engineer would draw: from every joint, the one joint it hands
/// most of its weight to, all the way down to a support. A tree rooted at the
/// supports, and everything that can be read off one.
/// <para>
/// <see cref="LoadPaths"/> finds how the weight drains as a flow, which divides
/// wherever two routes are as good as each other. That is the right physics and the
/// wrong picture: an engineer sizing up a model does not think of a purlin's weight
/// as forty per cent this way and sixty that, they think of it as resting on the
/// rafter, and of the rafter as resting on the column. This is that picture, and it
/// is taken from the flow rather than from a route search so the two can never
/// disagree — a joint's parent is the neighbour it sends the most flow to, and
/// since the potential falls along every such step the result is a forest with a
/// support at every root.
/// </para>
/// <para>
/// Four readings come off the tree, none of which needs a rule about any kind of
/// member. <b>Tributary</b> is the weight below a joint in the tree — what depends on
/// it — and per member, the most any part of it carries; it is what primary and
/// secondary mean, as a share rather than a name. <b>On the load path</b> is whether
/// a stretch is some joint's parent edge, so an element either carries or it
/// braces, ties, or offers a second route. <b>Resistance</b> is the least-resistance
/// route from a support, measured over the flow's own conductances, so a storey
/// climbed by a column costs its length and the same reach along a level beam costs
/// a hundred times more — the metric the flow already uses, made a distance.
/// <b>Cantilevered</b> is a joint whose whole subtree connects to the rest of the
/// model through one joint alone: a beam with a free end, a triangulated overhang,
/// a pendant, everything an engineer would say hangs off something.
/// </para>
/// </summary>
public sealed class LoadTree
{
    private LoadTree()
    {
    }

    /// <summary>Whether a tree was traced: supports were given and something reaches one.</summary>
    public bool Traced { get; private init; }

    /// <summary>Per joint, the joint it hands most of its weight to; -1 at a support, or where nothing drains.</summary>
    public int[] Parent { get; private init; } = null!;

    /// <summary>Per joint, the element the weight leaves along to its parent; -1 where there is no parent.</summary>
    public int[] ParentElement { get; private init; } = null!;

    /// <summary>Per joint, its share of the model's weight that drains through it, its own included, 0 to 1. A support's is what it holds up.</summary>
    public double[] Tributary { get; private init; } = null!;

    /// <summary>Per element, the largest tributary share carried along any stretch of it that is on the tree; 0 off the tree.</summary>
    public double[] ElementTributary { get; private init; } = null!;

    /// <summary>Per member, the most any of its elements carries.</summary>
    public double[] MemberTributary { get; private init; } = null!;

    /// <summary>Per assembly, the most any of its members carries.</summary>
    public double[] AssemblyTributary { get; private init; } = null!;

    /// <summary>Per element, whether any stretch of it is a joint's parent edge — whether weight is carried along it towards a support.</summary>
    public bool[] OnPath { get; private init; } = null!;

    /// <summary>
    /// Per joint, the least resistance from a support: length over conductance summed
    /// along the cheapest route, in model units. Infinity with no route; NaN with no supports.
    /// </summary>
    public double[] JointResistance { get; private init; } = null!;

    /// <summary>Per element, the least of its joints' <see cref="JointResistance"/>.</summary>
    public double[] Resistance { get; private init; } = null!;

    /// <summary>Per joint, whether it is held up through a single joint — it and everything below it in the tree meet the rest of the model nowhere else.</summary>
    public bool[] Cantilevered { get; private init; } = null!;

    /// <summary>Per element, whether any of its joints is cantilevered.</summary>
    public bool[] Cantilever { get; private init; } = null!;

    /// <summary>Per member, whether any of its elements is a cantilever.</summary>
    public bool[] MemberCantilever { get; private init; } = null!;

    internal static LoadTree Untraced(StructureGraph structure, PhysicalMembers members, Assemblies assemblies)
    {
        int joints = structure.Joints.Length;
        int n = structure.ElementCount;
        var nan = new double[joints];
        Array.Fill(nan, structure.HasSupports ? double.PositiveInfinity : double.NaN);
        var nanElements = new double[n];
        Array.Fill(nanElements, structure.HasSupports ? double.PositiveInfinity : double.NaN);

        return new LoadTree
        {
            Traced = false,
            Parent = Enumerable.Repeat(-1, joints).ToArray(),
            ParentElement = Enumerable.Repeat(-1, joints).ToArray(),
            Tributary = new double[joints],
            ElementTributary = new double[n],
            MemberTributary = new double[members.Count],
            AssemblyTributary = new double[assemblies.Count],
            OnPath = new bool[n],
            JointResistance = nan,
            Resistance = nanElements,
            Cantilevered = new bool[joints],
            Cantilever = new bool[n],
            MemberCantilever = new bool[members.Count],
        };
    }

    /// <param name="stretches">Every stretch the flow ran along, with its conductance.</param>
    /// <param name="conductances">The flow's graph, a node per joint, edges weighted by conductance.</param>
    /// <param name="injection">Weight lumped at each joint.</param>
    /// <param name="grounded">The supported joints.</param>
    internal static LoadTree Build(
        StructureGraph structure, PhysicalMembers members, Assemblies assemblies,
        IReadOnlyList<(int Element, int A, int B, double Conductance)> stretches, WeightedGraph conductances,
        PotentialFlowResult flow, double[] injection, int[] grounded)
    {
        int joints = structure.Joints.Length;
        int n = structure.ElementCount;

        // Each joint's parent: the neighbour it sends the most flow to. The first
        // stretch wins a tie, so the choice is a function of the input order alone.
        var parent = Enumerable.Repeat(-1, joints).ToArray();
        var parentElement = Enumerable.Repeat(-1, joints).ToArray();
        var strongest = new double[joints];
        foreach (var (element, a, b, c) in stretches)
        {
            double along = flow.Flow(a, b, c);
            (int from, int to, double amount) = along >= 0.0 ? (a, b, along) : (b, a, -along);
            if (amount > strongest[from] && !structure.Supported[from])
            {
                strongest[from] = amount;
                parent[from] = to;
                parentElement[from] = element;
            }
        }

        // Down the tree in order of falling potential, so every joint is summed
        // before the one it hands to.
        var reached = Enumerable.Range(0, joints).Where(j => flow.Reached[j]).OrderByDescending(j => flow.Potential[j]).ThenBy(j => j).ToArray();
        double whole = reached.Sum(j => injection[j]);
        var tributary = new double[joints];
        foreach (int j in reached)
            tributary[j] += injection[j];
        foreach (int j in reached)
            if (parent[j] >= 0)
                tributary[parent[j]] += tributary[j];
        if (whole > 0.0)
            for (int j = 0; j < joints; j++)
                tributary[j] = Math.Min(1.0, tributary[j] / whole);

        var elementTributary = new double[n];
        var onPath = new bool[n];
        foreach (var (element, a, b, _) in stretches)
        {
            if (parent[b] == a && parentElement[b] == element)
                Carry(element, b);
            if (parent[a] == b && parentElement[a] == element)
                Carry(element, a);
        }

        void Carry(int element, int child)
        {
            onPath[element] = true;
            elementTributary[element] = Math.Max(elementTributary[element], tributary[child]);
        }

        var memberTributary = new double[members.Count];
        for (int m = 0; m < members.Count; m++)
            memberTributary[m] = members.Elements[m].Max(e => elementTributary[e]);

        var assemblyTributary = new double[assemblies.Count];
        for (int a = 0; a < assemblies.Count; a++)
            assemblyTributary[a] = assemblies.Members[a].Max(m => memberTributary[m]);

        // Least resistance from the supports, over the conductances the flow used.
        var routes = Dijkstra.From(conductances, grounded, (_, _, conductance) => 1.0 / conductance);
        var jointResistance = (double[])routes.Cost.Clone();
        var resistance = new double[n];
        for (int e = 0; e < n; e++)
            resistance[e] = structure.ElementJoints[e].Select(j => jointResistance[j]).DefaultIfEmpty(double.PositiveInfinity).Min();

        var cantilevered = Hanging(structure, parent, reached);
        var cantilever = new bool[n];
        for (int e = 0; e < n; e++)
            cantilever[e] = structure.ElementJoints[e].Any(j => cantilevered[j]);
        var memberCantilever = new bool[members.Count];
        for (int m = 0; m < members.Count; m++)
            memberCantilever[m] = members.Elements[m].Any(e => cantilever[e]);

        return new LoadTree
        {
            Traced = true,
            Parent = parent,
            ParentElement = parentElement,
            Tributary = tributary,
            ElementTributary = elementTributary,
            MemberTributary = memberTributary,
            AssemblyTributary = assemblyTributary,
            OnPath = onPath,
            JointResistance = jointResistance,
            Resistance = resistance,
            Cantilevered = cantilevered,
            Cantilever = cantilever,
            MemberCantilever = memberCantilever,
        };
    }

    /// <summary>
    /// The joints held up through one joint alone. A joint's subtree hangs from it
    /// when every route-graph edge leaving the subtree lands on the joint itself;
    /// any edge from the subtree to elsewhere is a second way out. An edge between
    /// k and m contradicts hanging for every proper ancestor of k that is not also an
    /// ancestor of m — the ancestors strictly between k and their nearest common one
    /// — and likewise from m's side. Every joint below a hanging joint is cantilevered.
    /// </summary>
    private static bool[] Hanging(StructureGraph structure, int[] parent, int[] byFallingPotential)
    {
        int joints = structure.Joints.Length;
        var depth = new int[joints];
        var root = Enumerable.Repeat(-1, joints).ToArray();

        // Roots first: reversing the falling order visits every parent before its children.
        for (int i = byFallingPotential.Length - 1; i >= 0; i--)
        {
            int j = byFallingPotential[i];
            if (parent[j] < 0)
            {
                depth[j] = 0;
                root[j] = j;
            }
            else
            {
                depth[j] = depth[parent[j]] + 1;
                root[j] = root[parent[j]];
            }
        }

        var broken = new bool[joints];
        foreach (var (a, b, _) in structure.Routes.Edges())
        {
            if (root[a] < 0 || root[b] < 0)
                continue;

            int k = a, m = b;
            if (root[k] != root[m])
            {
                // Two different supports: nothing above either end can be hanging.
                for (int j = parent[k]; j >= 0; j = parent[j])
                    broken[j] = true;
                for (int j = parent[m]; j >= 0; j = parent[j])
                    broken[j] = true;
                continue;
            }

            // Walk both ends up to their common ancestor, marking what lies strictly between.
            while (depth[k] > depth[m])
            {
                k = parent[k];
                if (k != m)
                    broken[k] = true;
            }

            while (depth[m] > depth[k])
            {
                m = parent[m];
                if (m != k)
                    broken[m] = true;
            }

            while (k != m)
            {
                k = parent[k];
                m = parent[m];
                if (k != m)
                {
                    broken[k] = true;
                    broken[m] = true;
                }
            }
        }

        var cantilevered = new bool[joints];
        for (int i = byFallingPotential.Length - 1; i >= 0; i--)
        {
            int j = byFallingPotential[i];
            int p = parent[j];
            if (p >= 0)
                cantilevered[j] = cantilevered[p] || !broken[p];
        }

        return cantilevered;
    }
}
