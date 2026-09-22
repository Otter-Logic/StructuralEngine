namespace OtterLogic.StructuralEngine;

/// <summary>
/// What every tool reading a model checks before it starts: lines come as starts
/// and ends of equal count, surfaces as rings of at least three corners, and every
/// coordinate is a finite number in three columns.
/// <para>
/// Held once so the Insight engine, the sequencer and the connection tools refuse
/// the same input with the same sentence. Refused with a reason, never repaired:
/// a repaired model is one the user did not draw.
/// </para>
/// </summary>
public static class ModelInput
{
    public static (double[,] Starts, double[,] Ends) CheckLines(double[,]? starts, double[,]? ends)
    {
        if (starts is null && ends is null)
            return (new double[0, 3], new double[0, 3]);
        if (starts is null || ends is null)
            throw new ArgumentException("Give both the starts and the ends of the lines, or neither.");

        CheckPoints(starts, "lineStarts");
        CheckPoints(ends, "lineEnds");
        if (starts.GetLength(0) != ends.GetLength(0))
            throw new ArgumentException(
                $"{starts.GetLength(0)} line starts but {ends.GetLength(0)} line ends. Every line needs both, in the same order.");

        return (starts, ends);
    }

    public static IReadOnlyList<double[,]> CheckSurfaces(IReadOnlyList<double[,]>? surfaces)
    {
        if (surfaces is null)
            return Array.Empty<double[,]>();

        for (int k = 0; k < surfaces.Count; k++)
        {
            if (surfaces[k] is null)
                throw new ArgumentNullException(nameof(surfaces), $"Surface {k} is missing.");
            CheckPoints(surfaces[k], $"surface {k}");
            if (surfaces[k].GetLength(0) < 3)
                throw new ArgumentException(
                    $"Surface {k} has {surfaces[k].GetLength(0)} corner(s); a surface needs at least three corners.", nameof(surfaces));
        }

        return surfaces;
    }

    public static void CheckPoints(double[,] points, string name)
    {
        if (points.GetLength(1) != 3)
            throw new ArgumentException($"{name} needs three columns, x, y and z; it has {points.GetLength(1)}.", name);

        for (int i = 0; i < points.GetLength(0); i++)
            for (int c = 0; c < 3; c++)
                if (!double.IsFinite(points[i, c]))
                    throw new ArgumentException($"{name} point {i} has {points[i, c]} in it; coordinates must be finite.", name);
    }
}
