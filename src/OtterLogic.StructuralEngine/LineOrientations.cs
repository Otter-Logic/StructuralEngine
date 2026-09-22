using OtterLogic.Unsupervised.Clustering;

namespace OtterLogic.StructuralEngine;

/// <summary>How a line stands, named from the prototypes in <see cref="LineOrientations"/>.</summary>
public enum LineOrientation
{
    /// <summary>Lies level — a beam, a joist, a tie.</summary>
    Level,

    /// <summary>Half-way between — a brace, a rafter, a raking member.</summary>
    Pitched,

    /// <summary>Stands up — a column, a post, a hanger.</summary>
    Plumb,

    /// <summary>No length at the document's tolerance, so no direction to read.</summary>
    Degenerate,
}

/// <summary>
/// How each line stands — level, pitched or plumb — read from the model's own
/// spread of inclinations rather than from cut-off angles.
/// <para>
/// Here rather than in any toolkit because grid inference, geometry QA and
/// connection typology all read a line model, and a line must be called the same
/// thing by each of them. The inclinations are banded by <see cref="ValueBands"/>,
/// and each band named by its nearest prototype: a roof whose beams all sit at a
/// few degrees reads as level, and a leaning column as plumb — prototypes, not
/// cut-offs.
/// </para>
/// </summary>
public static class LineOrientations
{
    /// <summary>
    /// Orientation prototypes, as the sine of a line's inclination: level lies at
    /// 0, plumb stands at 1, pitched sits at 45 degrees between. A band of lines is
    /// named by the prototype nearest its median.
    /// </summary>
    public static readonly (LineOrientation Orientation, double Sine)[] Prototypes =
    {
        (LineOrientation.Level, 0.0),
        (LineOrientation.Pitched, Math.Sqrt(0.5)),
        (LineOrientation.Plumb, 1.0),
    };

    /// <summary>The prototype nearest a line's inclination sine.</summary>
    public static LineOrientation Nearest(double sine)
        => Prototypes.OrderBy(p => Math.Abs(p.Sine - sine)).First().Orientation;

    /// <summary>Orientation per line; lines with no length at the tolerance are degenerate.</summary>
    public static LineOrientation[] Classify(
        Vec[] starts, Vec[] ends, double tolerance, ValueBandsOptions banding, List<string> notes)
    {
        int n = starts.Length;
        var orientation = new LineOrientation[n];
        var measured = new List<int>();
        var sines = new List<double>();

        for (int i = 0; i < n; i++)
        {
            var d = ends[i] - starts[i];
            if (d.Length <= tolerance)
            {
                orientation[i] = LineOrientation.Degenerate;
                continue;
            }

            measured.Add(i);
            sines.Add(Math.Abs(d.Z) / d.Length);
        }

        int degenerate = n - measured.Count;
        if (degenerate > 0)
            notes.Add($"{degenerate} line(s) have no length at this tolerance and were left out.");

        if (sines.Count == 0)
            return orientation;

        var bands = ValueBands.Fit(sines, banding with { Resolution = 0.0 });
        var names = bands.Bands.Select(band => Nearest(band.Median)).ToArray();

        for (int k = 0; k < measured.Count; k++)
            orientation[measured[k]] = names[bands.Band[k]];

        return orientation;
    }
}
