using Xunit;

namespace OtterLogic.StructuralEngine.Tests;

/// <summary>
/// The whole reading in one call, and the two tables it lays out into: the
/// element table is the one Section Groups reads its role features from, column for column,
/// and the member table says of a column what an engineer would.
/// </summary>
public class ModelReadingTests
{
    private static int Column(string name) => Array.IndexOf(ModelReading.MemberFeatureNames, name);

    private static (double[,] Starts, double[,] Ends, double[,]? Supports) Arrays(Models model, bool supports)
    {
        var starts = new double[model.LineCount, 3];
        var ends = new double[model.LineCount, 3];
        for (int i = 0; i < model.LineCount; i++)
        {
            (starts[i, 0], starts[i, 1], starts[i, 2]) = (model.Start(i).X, model.Start(i).Y, model.Start(i).Z);
            (ends[i, 0], ends[i, 1], ends[i, 2]) = (model.End(i).X, model.End(i).Y, model.End(i).Z);
        }

        double[,]? feet = null;
        if (supports)
        {
            var bases = Enumerable.Range(0, model.LineCount).Where(i => model.Vertical[i] && model.Start(i).Z == 0.0).ToArray();
            feet = new double[bases.Length, 3];
            for (int k = 0; k < bases.Length; k++)
                (feet[k, 0], feet[k, 1], feet[k, 2]) = (model.Start(bases[k]).X, model.Start(bases[k]).Y, 0.0);
        }

        return (starts, ends, feet);
    }

    [Fact]
    public void ReadsAFrameTheWayTheStagesDo()
    {
        var model = new Models().Frame(2, 2, 1);
        var (starts, ends, supports) = Arrays(model, supports: true);
        var reading = ModelReading.Read(starts, ends, null, supports, new ModelReadingOptions { JoinDistance = 0.01 });
        var staged = model.Read();

        Assert.Equal(staged.Structure.Joints.Length, reading.Structure.Joints.Length);
        Assert.Equal(staged.Members.Count, reading.MemberCount);
        Assert.Equal(staged.Paths.Levels, reading.Paths.Levels);
        Assert.Equal(model.LineCount, reading.ElementCount);
        Assert.Equal(0, reading.SurfaceCount);

        var features = reading.ElementFeatures();
        Assert.Equal(reading.ElementCount, features.GetLength(0));
        Assert.Equal(ElementFeatures.Names.Length, features.GetLength(1));
        Assert.Equal(ElementFeatures.Names.Length, ModelReading.ElementFeatureNames.Length);

        var levels = reading.Level();
        for (int e = 0; e < model.LineCount; e++)
        {
            Assert.Equal(model.Vertical[e] ? 0 : 1, levels[e]);
            Assert.Equal(levels[e], (int)features[e, 18]);
            Assert.Equal(model.Vertical[e] ? LineOrientation.Plumb : LineOrientation.Level, reading.Orientation[e]);
        }
    }

    [Fact]
    public void TheMemberTableSaysWhatAnEngineerWould()
    {
        var model = new Models().Frame(2, 2, 1);
        var (starts, ends, supports) = Arrays(model, supports: true);
        var reading = ModelReading.Read(starts, ends, null, supports, new ModelReadingOptions { JoinDistance = 0.01 });
        var rows = reading.MemberFeatures();

        Assert.Equal(reading.MemberCount, rows.GetLength(0));
        Assert.Equal(ModelReading.MemberFeatureNames.Length, rows.GetLength(1));

        // Every column stack is one member: four metres long, plumb, on the ground, a support at its foot.
        int column = reading.Members.Of[0];
        Assert.Equal(4.0, rows[column, Column("Length")], 9);
        Assert.Equal(1.0, rows[column, Column("Upright")], 9);
        Assert.Equal(0.0, rows[column, Column("Level")]);
        Assert.Equal(0.0, rows[column, Column("Support Distance")], 9);
        Assert.Equal(0.0, rows[column, Column("Closed")]);

        // A beam line across the frame is two elements chained, level, a storey up.
        int beam = Enumerable.Range(0, reading.MemberCount).First(m => reading.Members.Elements[m].Length == 2);
        Assert.Equal(12.0, rows[beam, Column("Length")], 9);
        Assert.Equal(0.0, rows[beam, Column("Upright")], 9);
        Assert.Equal(2.0, rows[beam, Column("Elements")]);
        Assert.Equal(1.0, rows[beam, Column("Level")]);
        Assert.Equal(4.0, rows[beam, Column("Support Distance")], 9);

        // The load tree: the column carries and stands on the ground, the beam carries nothing through and is a storey of resistance up.
        Assert.Equal(1.0, rows[column, Column("On Load Path")]);
        Assert.Equal(0.0, rows[column, Column("Path Resistance")], 9);
        Assert.True(rows[column, Column("Tributary")] > rows[beam, Column("Tributary")]);
        Assert.Equal(0.0, rows[beam, Column("On Load Path")]);
        Assert.Equal(4.0, rows[beam, Column("Path Resistance")], 9);
        Assert.Equal(0.0, rows[beam, Column("Cantilever")]);
        Assert.Equal(0.0, rows[beam, Column("Stranded")]);

        // The element table carries the same columns down every element of the member.
        var features = reading.ElementFeatures();
        int element = reading.Members.Elements[beam][0];
        Assert.Equal(rows[beam, Column("Tributary")], features[element, ElementFeatures.Tributary], 9);
        Assert.Equal(4.0, features[element, ElementFeatures.Resistance], 9);
    }

    [Fact]
    public void WithoutSupportsTheDistanceAndLevelColumnsReadMinusOneAndSaySo()
    {
        var model = new Models().Frame(2, 2, 1);
        var (starts, ends, _) = Arrays(model, supports: false);
        var reading = ModelReading.Read(starts, ends, null, null, new ModelReadingOptions { JoinDistance = 0.01 });

        var rows = reading.MemberFeatures();
        Assert.All(Enumerable.Range(0, reading.MemberCount), m => Assert.Equal(-1.0, rows[m, Column("Support Distance")]));
        Assert.All(Enumerable.Range(0, reading.MemberCount), m => Assert.Equal(-1.0, rows[m, Column("Level")]));
        Assert.All(Enumerable.Range(0, reading.MemberCount), m => Assert.Equal(-1.0, rows[m, Column("Path Resistance")]));
        Assert.All(Enumerable.Range(0, reading.MemberCount), m => Assert.Equal(0.0, rows[m, Column("Tributary")]));
        Assert.False(reading.Tree.Traced);
        Assert.Contains(reading.Notes, note => note.Contains("No supports"));
    }

    [Fact]
    public void RefusesAnEmptyModel()
    {
        Assert.Throws<ArgumentException>(() => ModelReading.Read(null, null));
    }
}
