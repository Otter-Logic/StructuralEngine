using OtterLogic.Graphs;
using OtterLogic.Unsupervised.Clustering;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// Settings for <see cref="ModelReading"/>. Every one has a default, and none is
/// about a kind of structure.
/// </summary>
public sealed record ModelReadingOptions
{
    /// <summary>Points closer than this are the same point in the model. The document's absolute tolerance is the right value from Rhino.</summary>
    public double Tolerance { get; init; } = 0.001;

    /// <summary>
    /// Points closer than this are read as meant to meet, and joined. Null for ten
    /// times <see cref="Tolerance"/>, which is what every structural tool uses.
    /// </summary>
    public double? JoinDistance { get; init; }

    /// <summary>
    /// Read lines that carry straight on through a joint as one member. On by
    /// default; off reads every line as its own member, for a model drawn with
    /// deliberate breaks that ought to stay breaks.
    /// </summary>
    public bool ChainMembers { get; init; } = true;

    /// <summary>How line inclinations are banded when each line is called level, pitched or plumb.</summary>
    public ValueBandsOptions Banding { get; init; } = new();

    /// <summary>The join distance in force.</summary>
    public double Join => JoinDistance ?? 10.0 * Tolerance;

    public void Validate()
    {
        if (!double.IsFinite(Tolerance) || Tolerance <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(Tolerance), Tolerance, "Tolerance must be above zero.");
        if (JoinDistance is { } join && (!double.IsFinite(join) || join < Tolerance))
            throw new ArgumentOutOfRangeException(nameof(JoinDistance), join, $"The join distance must be at least the tolerance, {Tolerance}.");
        ArgumentNullException.ThrowIfNull(Banding);
        Banding.Validate();
    }
}

/// <summary>
/// The engine's whole reading of a model in one call: joints, geometry, members,
/// assemblies, load paths and the tree they draw, regions, support distances,
/// centrality and orientation, and the two tables they lay out into — one row per
/// element, one row per member.
/// <para>
/// Every toolkit was making this reading for itself, stage by stage, in the same
/// order: the Insight engine, the sequencer and the connection tools each call
/// <see cref="StructureGraph.Build"/>, then <see cref="ElementGeometry.Measure"/>,
/// then the members, the assemblies and the paths. This holds the sequence once,
/// so a toolkit or a component that wants the reading and nothing else — a table
/// to train on, a level per element, a member per line — gets it in one call and
/// gets exactly what the Insight engine reads.
/// </para>
/// <para>
/// It answers nothing. Which groups the members fall into, what order they go up
/// in, how many kinds of joint there are: those are questions the toolkits ask of
/// this reading, and each asks its own.
/// </para>
/// </summary>
public sealed class ModelReading
{
    private ModelReading()
    {
    }

    /// <summary>Which points are one joint, which elements meet at each, and the two graphs.</summary>
    public StructureGraph Structure { get; private init; } = null!;

    /// <summary>Each element's centroid, size, extent and aspect.</summary>
    public ElementGeometry Geometry { get; private init; } = null!;

    /// <summary>The physical members the lines chain into.</summary>
    public PhysicalMembers Members { get; private init; } = null!;

    /// <summary>The bodies the members are triangulated into.</summary>
    public Assemblies Assemblies { get; private init; } = null!;

    /// <summary>Where the weight goes on its way to the supports.</summary>
    public LoadPaths Paths { get; private init; } = null!;

    /// <summary>The load path as a tree from every joint to a support: parents, tributary loads, resistance, cantilevers. The same object as <see cref="LoadPaths.Tree"/>.</summary>
    public LoadTree Tree => Paths.Tree;

    /// <summary>Where the member graph nearly comes apart, and what each member holds together.</summary>
    public Regions Regions { get; private init; } = null!;

    /// <summary>Per element, route length along the elements to the nearest support: infinity with no route, NaN with no supports.</summary>
    public double[] SupportDistance { get; private init; } = null!;

    /// <summary>Per element, its betweenness in the element graph, 0 to 1.</summary>
    public double[] Centrality { get; private init; } = null!;

    /// <summary>Per line, how it stands: level, pitched, plumb, or degenerate at the tolerance.</summary>
    public LineOrientation[] Orientation { get; private init; } = null!;

    /// <summary>Length of the model's bounding diagonal, never below the tolerance.</summary>
    public double Diagonal { get; private init; }

    /// <summary>What a user should hear about this reading that the numbers alone do not say.</summary>
    public IReadOnlyList<string> Notes { get; private init; } = null!;

    public int LineCount => Structure.LineCount;

    public int ElementCount => Structure.ElementCount;

    public int SurfaceCount => ElementCount - LineCount;

    public int MemberCount => Members.Count;

    /// <summary>Per element, the assembly it is part of.</summary>
    public int[] Assembly() => Members.Of.Select(m => Assemblies.Of[m]).ToArray();

    /// <summary>Per element, how many hand-overs stand between it and the ground; -1 without supports or a route to one.</summary>
    public int[] Level() => Members.Of.Select(m => Paths.Level[Assemblies.Of[m]]).ToArray();

    /// <summary>
    /// Reads a model.
    /// </summary>
    /// <param name="lineStarts">n x 3, the start of each line element. Null when there are only surfaces.</param>
    /// <param name="lineEnds">n x 3, the end of each line element, in the same order.</param>
    /// <param name="surfaces">Each surface element's boundary corners in order, k x 3 with at least three. Numbered after every line.</param>
    /// <param name="supports">m x 3 support points. Null or empty skips everything that needs a support.</param>
    /// <param name="options">Settings; null for the defaults.</param>
    public static ModelReading Read(
        double[,]? lineStarts,
        double[,]? lineEnds,
        IReadOnlyList<double[,]>? surfaces = null,
        double[,]? supports = null,
        ModelReadingOptions? options = null)
    {
        options ??= new ModelReadingOptions();
        options.Validate();

        var (starts, ends) = ModelInput.CheckLines(lineStarts, lineEnds);
        surfaces = ModelInput.CheckSurfaces(surfaces);
        if (supports is not null)
            ModelInput.CheckPoints(supports, nameof(supports));

        int lines = starts.GetLength(0);
        if (lines + surfaces.Count == 0)
            throw new ArgumentException("There is nothing to read: give lines, surfaces, or both.", nameof(lineStarts));

        var structure = StructureGraph.Build(starts, ends, surfaces, supports, options.Join);
        var geometry = ElementGeometry.Measure(starts, ends, surfaces);
        var members = PhysicalMembers.Read(structure, geometry, chain: options.ChainMembers);
        var assemblies = Assemblies.Read(structure, members);
        var paths = LoadPaths.Trace(structure, geometry, members, assemblies);
        var regions = Regions.Read(members);
        var supportDistance = StructuralEngine.ElementFeatures.SupportDistances(structure);
        var centrality = Graphs.Centrality.Betweenness(structure.Elements);

        var notes = new List<string>();
        var orientation = Array.Empty<LineOrientation>();
        if (lines > 0)
        {
            var a0 = new Vec[lines];
            var a1 = new Vec[lines];
            for (int i = 0; i < lines; i++)
            {
                a0[i] = Vec.Row(starts, i);
                a1[i] = Vec.Row(ends, i);
            }

            orientation = LineOrientations.Classify(a0, a1, options.Tolerance, options.Banding, notes);
        }

        if (!structure.HasSupports)
            notes.Add("No supports given, so support distances, load paths and levels were skipped: Support Distance "
                + "and Level read -1 and Flow reads 0. Wire the supports in to read the model the way it stands.");
        if (!paths.Converged)
            notes.Add("The load-path flow stopped short of its tolerance, so Flow and Level are approximate. A model with "
                + "members many orders of magnitude apart in length is the usual cause.");
        if (structure.StrandedSupports.Length > 0)
            notes.Add($"{structure.StrandedSupports.Length} support(s) sit at no joint, so they hold nothing up: "
                + string.Join(", ", structure.StrandedSupports) + ".");
        if (!double.IsNaN(members.TurnLimit))
            notes.Add($"Lines turning less than {members.TurnLimit:0.#} degrees at a joint were read as one member.");
        if (paths.Tree.Traced && paths.Tree.MemberCantilever.Any(c => c))
            notes.Add($"{paths.Tree.MemberCantilever.Count(c => c)} member(s) are held up through a single joint and read as cantilevers.");
        if (regions.Found && !regions.Converged)
            notes.Add("The region reading stopped short of its tolerance, so Region and Cut Proximity are approximate.");

        return new ModelReading
        {
            Structure = structure,
            Geometry = geometry,
            Members = members,
            Assemblies = assemblies,
            Paths = paths,
            Regions = regions,
            SupportDistance = supportDistance,
            Centrality = centrality,
            Orientation = orientation,
            Diagonal = Math.Max(DiagonalOf(structure.Joints), options.Tolerance),
            Notes = notes,
        };
    }

    /// <summary>One row per element; see <see cref="StructuralEngine.ElementFeatures"/> for the columns.</summary>
    public double[,] ElementFeatures()
        => StructuralEngine.ElementFeatures.Raw(Geometry, Structure, SupportDistance, Centrality, Members, Assemblies, Paths, Regions);

    /// <summary>The columns of <see cref="ElementFeatures()"/>.</summary>
    public static string[] ElementFeatureNames => (string[])StructuralEngine.ElementFeatures.Names.Clone();

    /// <summary>
    /// The columns of <see cref="MemberFeatures"/>, in order: what a member is like
    /// as a whole, what frames into it and what it frames into, its place in the
    /// load path and in its assembly, what its elements are like on average, and,
    /// appended since 2026-09, its place on the load tree and in the regions.
    /// </summary>
    public static readonly string[] MemberFeatureNames =
    {
        "Length", "Upright", "Straightness", "Elements", "Connections", "Ends Bearing", "Members Carried",
        "Flow", "Level", "Assembly Members", "Depth Position", "Along Span", "Surface", "Aspect Ratio",
        "Support Distance", "Centrality", "Closed",
        "Path Resistance", "Tributary", "On Load Path", "Cantilever", "Region", "Cut Proximity", "Stranded",
    };

    /// <summary>
    /// One row per member, in model units. Upright, Aspect Ratio and Centrality are
    /// averaged over the member's elements, weighted by size; Support Distance and
    /// Path Resistance are the nearest of its elements', -1 with no route or no
    /// supports; Level is -1 likewise. Tributary is the most any element carries,
    /// On Load Path and Cantilever whether any does. Nothing refers to where the
    /// member is in plan.
    /// </summary>
    public double[,] MemberFeatures()
    {
        var raw = ElementFeatures();
        int count = Members.Count;
        var rows = new double[count, MemberFeatureNames.Length];

        for (int m = 0; m < count; m++)
        {
            var elements = Members.Elements[m];
            int assembly = Assemblies.Of[m];
            double weight = elements.Sum(e => Geometry.Size[e]);
            double Mean(int column) => weight > 0.0
                ? elements.Sum(e => raw[e, column] * Geometry.Size[e]) / weight
                : elements.Average(e => raw[e, column]);

            var reachable = elements.Select(e => SupportDistance[e]).Where(double.IsFinite).ToArray();

            rows[m, 0] = Members.Length[m];
            rows[m, 1] = Mean(StructuralEngine.ElementFeatures.ExtentZ);
            rows[m, 2] = Members.Straightness[m];
            rows[m, 3] = elements.Length;
            rows[m, 4] = Members.Meets[m];
            rows[m, 5] = Members.EndsBearing[m];
            rows[m, 6] = Members.Carried[m];
            rows[m, 7] = Paths.MemberFlow[m];
            rows[m, 8] = Paths.Level[assembly];
            rows[m, 9] = Assemblies.Members[assembly].Length;
            rows[m, 10] = Assemblies.DepthPosition[m];
            rows[m, 11] = Assemblies.AlongSpan[m];
            rows[m, 12] = Structure.IsSurface(elements[0]) ? 1.0 : 0.0;
            rows[m, 13] = Mean(StructuralEngine.ElementFeatures.Aspect);
            rows[m, 14] = reachable.Length > 0 ? reachable.Min() : -1.0;
            rows[m, 15] = Mean(StructuralEngine.ElementFeatures.Centrality);
            rows[m, 16] = Members.Closed[m] ? 1.0 : 0.0;

            var resistance = elements.Select(e => Tree.Resistance[e]).Where(double.IsFinite).ToArray();
            rows[m, 17] = resistance.Length > 0 ? resistance.Min() : -1.0;
            rows[m, 18] = Tree.MemberTributary[m];
            rows[m, 19] = elements.Any(e => Tree.OnPath[e]) ? 1.0 : 0.0;
            rows[m, 20] = Tree.MemberCantilever[m] ? 1.0 : 0.0;
            rows[m, 21] = Regions.Side[m];
            rows[m, 22] = Regions.CutProximity[m];
            rows[m, 23] = Regions.Stranded[m];
        }

        return rows;
    }

    private static double DiagonalOf(Vec[] joints)
    {
        if (joints.Length == 0)
            return 0.0;

        var low = new Vec(joints.Min(j => j.X), joints.Min(j => j.Y), joints.Min(j => j.Z));
        var high = new Vec(joints.Max(j => j.X), joints.Max(j => j.Y), joints.Max(j => j.Z));
        return low.DistanceTo(high);
    }
}
