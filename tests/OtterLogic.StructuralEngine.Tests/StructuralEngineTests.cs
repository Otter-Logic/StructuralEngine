using Xunit;

namespace OtterLogic.StructuralEngine.Tests;

/// <summary>
/// The reading every structural toolkit builds on, tested on things true of any
/// structure: points within the join distance are one joint, a run drawn in pieces
/// is one member, what rests on something sits a level above it, lines triangulated
/// together are one body, a line is called level or plumb by the model's own spread,
/// and none of it depends on which way the model faces.
/// </summary>
public class StructuralEngineTests
{
    [Fact]
    public void PointsWithinTheJoinDistance_AreOneJoint()
    {
        var model = new Models().Frame(2, 2, 1);
        var reading = model.Read();

        // Nine column feet and nine column heads, every beam end landing on a head.
        Assert.Equal(18, reading.Structure.Joints.Length);
        Assert.Equal(9, reading.Structure.Supported.Count(s => s));
        Assert.Empty(reading.Structure.StrandedSupports);
        Assert.All(reading.Structure.Degenerate, d => Assert.False(d));
        Assert.All(reading.Structure.DuplicateOf, d => Assert.Equal(-1, d));
    }

    [Fact]
    public void ARunDrawnInPieces_IsOneMember()
    {
        var reading = new Models().Frame(3, 3, 2).Read();

        // Sixteen column stacks, and four beam lines each way on each of two floors.
        Assert.Equal(16 + 2 * (4 + 4), reading.Members.Count);
        Assert.Equal(reading.Members.Of[0], reading.Members.Of[1]);
        Assert.NotEqual(reading.Members.Of[0], reading.Members.Of[2]);
    }

    [Fact]
    public void WhatRestsOnSomething_SitsALevelAboveIt()
    {
        var model = new Models().Frame(2, 2, 1);
        int resting = model.Line(3, 0, 4, 3, 6, 4);
        int restingToo = model.Line(9, 0, 4, 9, 6, 4);
        int restingOnThose = model.Line(3, 3, 4, 9, 3, 4);

        var reading = model.Read();

        Assert.True(reading.Paths.Traced);
        for (int e = 0; e < resting; e++)
            Assert.Equal(model.Vertical[e] ? 0 : 1, reading.Level(e));
        Assert.Equal(2, reading.Level(resting));
        Assert.Equal(2, reading.Level(restingToo));
        Assert.Equal(3, reading.Level(restingOnThose));
        Assert.Equal(3, reading.Paths.Levels);
    }

    [Fact]
    public void LinesTriangulatedTogether_AreOneBody()
    {
        // A flat Warren truss on two supports: chords and webs are one assembly.
        var model = new Models();
        model.Support(0, 0, 0);
        model.Support(12, 0, 0);
        for (int i = 0; i < 4; i++)
            model.Line(i * 3, 0, 0, (i + 1) * 3, 0, 0);
        for (int i = 0; i < 3; i++)
            model.Line(1.5 + i * 3, 0, 2, 4.5 + i * 3, 0, 2);
        for (int i = 0; i < 4; i++)
        {
            model.Line(i * 3, 0, 0, 1.5 + i * 3, 0, 2);
            model.Line(1.5 + i * 3, 0, 2, (i + 1) * 3, 0, 0);
        }

        var reading = model.Read();

        Assert.Equal(1, reading.Assemblies.Count);
        Assert.Equal(0, reading.Paths.Levels);
    }

    [Fact]
    public void HowALineStands_IsReadFromTheModelsOwnSpread()
    {
        var model = new Models().Frame(2, 1, 1);
        int brace = model.Line(0, 0, 0, 6, 0, 4);
        var starts = Enumerable.Range(0, model.LineCount).Select(model.Start).ToArray();
        var ends = Enumerable.Range(0, model.LineCount).Select(model.End).ToArray();

        var orientation = LineOrientations.Classify(starts, ends, 0.001, new Unsupervised.Clustering.ValueBandsOptions(), new List<string>());

        for (int e = 0; e < brace; e++)
            Assert.Equal(model.Vertical[e] ? LineOrientation.Plumb : LineOrientation.Level, orientation[e]);
        Assert.Equal(LineOrientation.Pitched, orientation[brace]);

        Assert.Equal(LineOrientation.Plumb, LineOrientations.Nearest(0.95));
        Assert.Equal(LineOrientation.Level, LineOrientations.Nearest(0.1));
        Assert.Equal(LineOrientation.Pitched, LineOrientations.Nearest(0.6));
    }

    [Fact]
    public void TheReading_DoesNotDependOnWhichWayTheModelFaces()
    {
        var model = new Models().Frame(3, 2, 2);
        var a = model.Read();
        var b = model.TurnedAboutVertical(37.0).Read();

        Assert.Equal(a.Members.Of, b.Members.Of);
        Assert.Equal(a.Assemblies.Of, b.Assemblies.Of);
        Assert.Equal(a.Paths.Level, b.Paths.Level);
        for (int e = 0; e < model.LineCount; e++)
            Assert.Equal(a.Paths.ElementFlow[e], b.Paths.ElementFlow[e], 6);
    }

    [Fact]
    public void BadInput_IsRefusedWithAReason()
    {
        Assert.Throws<ArgumentException>(() => ModelInput.CheckLines(new double[2, 3], new double[1, 3]));
        Assert.Throws<ArgumentException>(() => ModelInput.CheckLines(new double[1, 3], null));
        Assert.Throws<ArgumentException>(() => ModelInput.CheckPoints(new double[1, 2], "points"));
        Assert.Throws<ArgumentException>(() => ModelInput.CheckPoints(new[,] { { 0.0, double.NaN, 0.0 } }, "points"));
        Assert.Throws<ArgumentException>(() => ModelInput.CheckSurfaces(new[] { new double[2, 3] }));
    }
}
