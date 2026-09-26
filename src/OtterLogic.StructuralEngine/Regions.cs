using OtterLogic.Graphs;
using OtterLogic.MachineLearning.Decomposition;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// Where the member graph nearly comes apart: the weakest cut through it, which
/// side of that cut each member sits on, how near the cut it stands, and how much
/// of the model each member alone holds together.
/// <para>
/// <see cref="Assemblies"/> says what acts as one body and <see cref="LoadPaths"/>
/// says which body carries which. Neither says where the structure divides: the
/// two wings of a building joined by a link bridge, the bays of a hangar tied
/// through one line of purlins, the twin trusses of a crane girder with a walkway
/// between. That is a question about the graph as a whole, and its answer is the
/// graph Laplacian's second eigenvector, the Fiedler vector: its sign splits the
/// graph across the cut that severs the fewest connections for the size of the
/// pieces it leaves, and its size on a member says how deep in a piece that member
/// sits. The eigenvalue with it is the algebraic connectivity — near zero, the two
/// pieces barely touch; near one, there is no cut worth the name.
/// </para>
/// <para>
/// The eigenvector is read off <see cref="SpectralEmbedding"/> one layer down,
/// which is the same map StructuralDesign's clustering partitions. On a model in
/// several disconnected pieces the pieces are the cut, and the vector reported is
/// the weakest cut <em>within</em> whichever piece has one. Beside it, from one
/// low-link search, is what each member strands: how many members lose their
/// connection to the rest when it alone is taken away — zero almost everywhere,
/// and the whole far wing on the one member of the link bridge.
/// </para>
/// </summary>
public sealed class Regions
{
    private Regions()
    {
    }

    /// <summary>Whether a cut was found: at least two connected members.</summary>
    public bool Found { get; private init; }

    /// <summary>Per member, the Fiedler vector's value: sign is the side, size is depth into it. Zero for a member connected to nothing.</summary>
    public double[] Fiedler { get; private init; } = null!;

    /// <summary>Per member, which side of the weakest cut: -1 or +1, and 0 for a member connected to nothing.</summary>
    public int[] Side { get; private init; } = null!;

    /// <summary>Per member, how far from the weakest cut, 0 on it to 1 deepest into a side.</summary>
    public double[] CutProximity { get; private init; } = null!;

    /// <summary>The algebraic connectivity of the member graph, 0 to 2: the eigenvalue of <see cref="Fiedler"/>. NaN when none was found.</summary>
    public double Connectivity { get; private init; } = double.NaN;

    /// <summary>Connected pieces among the members that are connected to anything.</summary>
    public int Pieces { get; private init; }

    /// <summary>Per member, how many other members lose their connection to the largest remaining piece when it is removed.</summary>
    public int[] Stranded { get; private init; } = null!;

    /// <summary>Whether the eigensolver settled; a false reading of <see cref="Fiedler"/> is approximate.</summary>
    public bool Converged { get; private init; } = true;

    public static Regions Read(PhysicalMembers members)
    {
        ArgumentNullException.ThrowIfNull(members);
        var graph = members.Graph;
        int count = members.Count;

        var stranded = count > 0 ? CutVertices.Stranded(graph) : Array.Empty<int>();
        int placed = Enumerable.Range(0, count).Count(m => graph.Degree(m) > 0.0);
        graph.ConnectedComponents(out int pieces);
        int connected = pieces - (count - placed);

        Regions None() => new()
        {
            Found = false,
            Fiedler = new double[count],
            Side = new int[count],
            CutProximity = new double[count],
            Pieces = connected,
            Stranded = stranded,
        };

        // The first eigenvector per piece is constant on it; the next is the cut.
        int dimensions = connected + 1;
        if (placed < 2 || placed < dimensions)
            return None();

        var map = SpectralEmbedding.Of(graph, dimensions);
        int column = dimensions - 1;

        var fiedler = new double[count];
        var side = new int[count];
        double deepest = 0.0;
        for (int m = 0; m < count; m++)
        {
            fiedler[m] = map.Coordinates[m, column];
            deepest = Math.Max(deepest, Math.Abs(fiedler[m]));
        }

        var proximity = new double[count];
        for (int m = 0; m < count; m++)
        {
            if (graph.Degree(m) <= 0.0)
                continue;

            side[m] = fiedler[m] < 0.0 ? -1 : 1;
            proximity[m] = deepest > 0.0 ? Math.Abs(fiedler[m]) / deepest : 0.0;
        }

        return new Regions
        {
            Found = true,
            Fiedler = fiedler,
            Side = side,
            CutProximity = proximity,
            Connectivity = map.Eigenvalues[column],
            Pieces = map.GraphComponents,
            Stranded = stranded,
            Converged = map.Converged,
        };
    }
}
