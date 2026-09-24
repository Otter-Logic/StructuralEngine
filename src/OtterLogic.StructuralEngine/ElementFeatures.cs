using OtterLogic.Graphs;

namespace OtterLogic.StructuralEngine;

/// <summary>
/// One row of numbers per element, laid out from the engine's readings: where it
/// is, how big, which way it extends, what meets it, how far it stands from a
/// support, what member and assembly it is part of, how much weight passes along
/// it and how many hand-overs it sits above the ground.
/// <para>
/// Here rather than in a toolkit because every column is a reading, not an
/// opinion — something measurable on any stick or surface model, and none of them
/// says what an element <em>is</em>. The Insight engine clusters on them, a
/// trained model reads them as inputs, and a fabricator estimating hours wants
/// the same row; each computing its own would be three tables that drift. Moved
/// down from StructuralDesign in 2026-09 with no column changing place, so a
/// definition reading them by position still reads what it did.
/// </para>
/// <para>
/// Nothing here is transformed: lengths are lengths, counts are counts, in model
/// units. Which of them to log, scale or leave out before learning from them is
/// the judgement of whoever learns, and stays with them.
/// </para>
/// </summary>
public static class ElementFeatures
{
    /// <summary>
    /// The columns, in order. The first twelve are as they have always been, in the
    /// same places; what the members, assemblies and load paths add comes after.
    /// </summary>
    public static readonly string[] Names =
    {
        "Centroid X", "Centroid Y", "Centroid Z", "Size", "Extent X", "Extent Y", "Extent Z",
        "Connections", "Support Distance", "Centrality", "Surface", "Aspect Ratio",
        "Member Length", "Member Straightness", "Member Connections", "Ends Bearing", "Members Carried",
        "Flow", "Level", "Assembly Members", "Depth Position", "Along Span",
    };

    /// <summary>Column of the element's share along z: how upright it stands.</summary>
    public const int ExtentZ = 6;

    /// <summary>Column of the route length along the elements to the nearest support, or -1.</summary>
    public const int SupportDistance = 8;

    /// <summary>Column of the element's betweenness in the element graph.</summary>
    public const int Centrality = 9;

    /// <summary>Column that is 1 for a surface and 0 for a line.</summary>
    public const int Surface = 10;

    /// <summary>Column of a surface's in-plane aspect ratio, 1 for a line.</summary>
    public const int Aspect = 11;

    /// <summary>
    /// The features, per element, in model units. Support Distance is the route
    /// length along the elements to the nearest support, and -1 where there is no
    /// such route or no supports were given; Level is -1 likewise. The member and
    /// assembly columns repeat down every element of the member.
    /// </summary>
    public static double[,] Raw(
        ElementGeometry geometry, StructureGraph structure, double[] supportDistance, double[] centrality,
        PhysicalMembers members, Assemblies assemblies, LoadPaths paths)
    {
        int n = structure.ElementCount;
        var features = new double[n, Names.Length];

        for (int e = 0; e < n; e++)
        {
            var centroid = geometry.Centroid[e];
            var extent = geometry.Extent[e];
            int member = members.Of[e];
            int assembly = assemblies.Of[member];

            features[e, 0] = centroid.X;
            features[e, 1] = centroid.Y;
            features[e, 2] = centroid.Z;
            features[e, 3] = geometry.Size[e];
            features[e, 4] = extent.X;
            features[e, 5] = extent.Y;
            features[e, ExtentZ] = extent.Z;
            features[e, 7] = structure.Elements.Neighbours(e).Length;
            features[e, SupportDistance] = double.IsFinite(supportDistance[e]) ? supportDistance[e] : -1.0;
            features[e, Centrality] = centrality[e];
            features[e, Surface] = structure.IsSurface(e) ? 1.0 : 0.0;
            features[e, Aspect] = geometry.Aspect[e];
            features[e, 12] = members.Length[member];
            features[e, 13] = members.Straightness[member];
            features[e, 14] = members.Meets[member];
            features[e, 15] = members.EndsBearing[member];
            features[e, 16] = members.Carried[member];
            features[e, 17] = paths.ElementFlow[e];
            features[e, 18] = paths.Level[assembly];
            features[e, 19] = assemblies.Members[assembly].Length;
            features[e, 20] = assemblies.DepthPosition[member];
            features[e, 21] = assemblies.AlongSpan[member];
        }

        return features;
    }

    /// <summary>
    /// Route length along the elements from each element to the nearest support:
    /// the nearest of its joints. Positive infinity with no route, NaN for every
    /// element when no supports were given.
    /// </summary>
    public static double[] SupportDistances(StructureGraph structure)
    {
        int n = structure.ElementCount;
        var distance = new double[n];

        if (!structure.HasSupports)
        {
            Array.Fill(distance, double.NaN);
            return distance;
        }

        var sources = Enumerable.Range(0, structure.Joints.Length).Where(j => structure.Supported[j]).ToArray();
        if (sources.Length == 0)
        {
            Array.Fill(distance, double.PositiveInfinity);
            return distance;
        }

        var routes = Dijkstra.From(structure.Routes, sources);
        for (int e = 0; e < n; e++)
            distance[e] = structure.ElementJoints[e].Select(j => routes.Cost[j]).DefaultIfEmpty(double.PositiveInfinity).Min();

        return distance;
    }
}
