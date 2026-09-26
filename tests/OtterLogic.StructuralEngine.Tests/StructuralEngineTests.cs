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
    public void TheLoadTree_HandsEveryJointDownItsColumn()
    {
        var model = new Models().Frame(2, 2, 1);
        var reading = model.Read();
        var tree = reading.Tree;

        Assert.True(tree.Traced);
        for (int e = 0; e < model.LineCount; e++)
        {
            if (model.Vertical[e])
            {
                // A column top hands to its own foot, along the column, which carries what it collects.
                int foot = Models.JointAt(reading.Structure, model.Start(e).X, model.Start(e).Y, 0.0);
                int top = Models.JointAt(reading.Structure, model.Start(e).X, model.Start(e).Y, 4.0);
                Assert.Equal(-1, tree.Parent[foot]);
                Assert.Equal(foot, tree.Parent[top]);
                Assert.Equal(e, tree.ParentElement[top]);
                Assert.True(tree.OnPath[e]);
                Assert.True(tree.ElementTributary[e] > 0.0);
                Assert.Equal(0.0, tree.Resistance[e]);
            }
            else
            {
                // A beam between two equal columns carries nothing through; its ends are a storey of column from the ground.
                Assert.False(tree.OnPath[e]);
                Assert.Equal(0.0, tree.ElementTributary[e]);
                Assert.Equal(4.0, tree.Resistance[e], 9);
            }

            Assert.False(tree.Cantilever[e]);
        }

        // Everything drains somewhere: the supports between them hold the whole model.
        double held = Enumerable.Range(0, reading.Structure.Joints.Length).Where(j => reading.Structure.Supported[j]).Sum(j => tree.Tributary[j]);
        Assert.Equal(1.0, held, 9);
    }

    [Fact]
    public void ResistanceClimbsAColumnAndCrawlsAlongABeam()
    {
        var model = new Models().Frame(2, 2, 1);
        int resting = model.Line(3, 0, 4, 3, 6, 4);
        int restingToo = model.Line(9, 0, 4, 9, 6, 4);
        int restingOnThose = model.Line(3, 3, 4, 9, 3, 4);

        var reading = model.Read();
        var tree = reading.Tree;

        // Three metres along a level beam from a column top is seven metres of route,
        // but a storey up a column and then a hundred times that per metre level.
        double[] route = ElementFeatures.SupportDistances(reading.Structure);
        Assert.Equal(7.0, route[resting], 9);
        Assert.Equal(4.0 + 3.0 / 0.01, tree.Resistance[resting], 6);

        // What rests on something carries less than it: the columns most, the line
        // with something resting on it next, the line resting on those nothing through.
        double columns = Enumerable.Range(0, resting).Where(e => model.Vertical[e]).Max(e => tree.ElementTributary[e]);
        Assert.True(columns > tree.ElementTributary[resting]);
        Assert.True(tree.ElementTributary[resting] > 0.0);
        Assert.True(tree.OnPath[resting]);
        Assert.Equal(0.0, tree.ElementTributary[restingOnThose]);
        Assert.False(tree.OnPath[restingOnThose]);
        Assert.True(tree.MemberTributary[reading.Members.Of[resting]] > tree.MemberTributary[reading.Members.Of[restingOnThose]]);

        // The hand-over graph says the same, assembly to assembly, and only the columns reach the ground.
        int from = reading.Assembly(restingOnThose);
        int to = reading.Assembly(resting);
        Assert.Contains(to, reading.Paths.HandOver.Successors(from).ToArray());
        Assert.Contains(reading.Assembly(restingToo), reading.Paths.HandOver.Successors(from).ToArray());
        Assert.Equal(0.0, reading.Paths.ToGround[from]);
        Assert.Equal(0.0, reading.Paths.ToGround[to]);
        Assert.True(reading.Paths.ToGround[reading.Assembly(0)] > 0.0);
        Assert.True(reading.Paths.HandOver.Arcs().Sum(arc => arc.Weight) > 0.0);
        Assert.Equal(1.0, reading.Paths.ToGround.Sum(), 6);
    }

    [Fact]
    public void WhatIsHeldUpThroughOneJoint_IsACantilever()
    {
        var model = new Models().Frame(1, 1, 1);
        int overhang = model.Line(6, 0, 4, 9, 0, 4);
        int beyond = model.Line(9, 0, 4, 9, 0, 6);

        var reading = model.Read();
        var tree = reading.Tree;

        int tip = Models.JointAt(reading.Structure, 9, 0, 4);
        int top = Models.JointAt(reading.Structure, 6, 0, 4);
        Assert.Equal(top, tree.Parent[tip]);
        Assert.True(tree.Cantilevered[tip]);
        Assert.False(tree.Cantilevered[top]);
        Assert.True(tree.Cantilever[overhang]);
        Assert.True(tree.Cantilever[beyond]);
        Assert.True(tree.OnPath[overhang]);
        for (int e = 0; e < overhang; e++)
            Assert.False(tree.Cantilever[e]);
        Assert.Equal(2, tree.MemberCantilever.Count(c => c));
    }

    [Fact]
    public void WhereTheStructureNearlyComesApart_IsRead()
    {
        // Two frames tied by one beam: the beam is the cut, each frame a side. The
        // tie leaves each frame's corner at forty-five degrees in plan, so it is its
        // own member rather than a continuation of a beam on either side.
        var model = new Models().Frame(1, 1, 1).Frame(1, 1, 1, x0: 20.0, y0: 20.0);
        int link = model.Line(6, 6, 4, 20, 20, 4);

        var reading = model.Read();
        var regions = reading.Regions;

        Assert.True(regions.Found);
        Assert.Equal(1, regions.Pieces);
        Assert.True(regions.Connectivity > 0.0 && regions.Connectivity < 1.0);

        int first = reading.Members.Of[0];
        int second = reading.Members.Of[link - 1];
        Assert.NotEqual(regions.Side[first], regions.Side[second]);
        for (int e = 0; e < link; e++)
            Assert.Equal(model.Start(e).X < 10.0 ? regions.Side[first] : regions.Side[second], regions.Side[reading.Members.Of[e]]);

        int tie = reading.Members.Of[link];
        Assert.Equal(regions.CutProximity.Min(), regions.CutProximity[tie]);
        Assert.Equal(8, regions.Stranded[tie]);
        Assert.All(Enumerable.Range(0, reading.Members.Count).Where(m => m != tie), m => Assert.Equal(0, regions.Stranded[m]));
    }

    [Fact]
    public void TheReading_DoesNotDependOnWhichWayTheModelFaces()
    {
        var model = new Models().Frame(3, 2, 2);
        model.Line(18, 0, 8, 22, 0, 8);
        var a = model.Read();
        var b = model.TurnedAboutVertical(37.0).Read();

        Assert.Equal(a.Members.Of, b.Members.Of);
        Assert.Equal(a.Assemblies.Of, b.Assemblies.Of);
        Assert.Equal(a.Paths.Level, b.Paths.Level);
        Assert.Equal(a.Tree.Parent, b.Tree.Parent);
        Assert.Equal(a.Tree.OnPath, b.Tree.OnPath);
        Assert.Equal(a.Tree.Cantilever, b.Tree.Cantilever);
        Assert.Equal(a.Regions.Side, b.Regions.Side);
        Assert.Equal(a.Regions.Stranded, b.Regions.Stranded);
        for (int e = 0; e < model.LineCount; e++)
        {
            Assert.Equal(a.Paths.ElementFlow[e], b.Paths.ElementFlow[e], 6);
            Assert.Equal(a.Tree.ElementTributary[e], b.Tree.ElementTributary[e], 6);
            Assert.Equal(a.Tree.Resistance[e], b.Tree.Resistance[e], 6);
            Assert.Equal(a.Regions.CutProximity[a.Members.Of[e]], b.Regions.CutProximity[b.Members.Of[e]], 6);
        }
    }

    [Fact]
    public void AFacetedCurve_IsOneRafter_AndItsPurlinsSitOneLevelAbove()
    {
        // A sine-curved roof drawn in eight facets a row, over columns of three
        // heights, with level purlins run across it. The facet turns grow from three
        // to nine degrees along the curve; banded too finely they split into bands of
        // their own and the rafters break into pieces that read as resting on each other.
        var model = new Models();
        double Z(double x) => 4.0 + 3.0 * Math.Sin(Math.PI * x / 24.0);
        var xs = Enumerable.Range(0, 9).Select(i => i * 3.0).ToArray();
        var rafters = new List<int>();
        var purlins = new List<int>();
        for (int j = 0; j <= 2; j++)
        {
            double y = j * 5.0;
            foreach (double x in new[] { 0.0, 12.0, 24.0 })
            {
                model.Support(x, y, 0.0);
                model.Line(x, y, 0.0, x, y, Z(x));
            }

            for (int i = 0; i + 1 < xs.Length; i++)
                rafters.Add(model.Line(xs[i], y, Z(xs[i]), xs[i + 1], y, Z(xs[i + 1])));
        }

        for (int j = 0; j < 2; j++)
            foreach (double x in xs.Where(x => x != 0.0 && x != 12.0 && x != 24.0))
                purlins.Add(model.Line(x, j * 5.0, Z(x), x, (j + 1) * 5.0, Z(x)));

        var reading = model.Read();

        Assert.Equal(3, rafters.Select(e => reading.Members.Of[e]).Distinct().Count());
        Assert.All(rafters, e => Assert.Equal(1, reading.Level(e)));
        Assert.All(purlins, e => Assert.Equal(2, reading.Level(e)));
        Assert.All(Enumerable.Range(0, model.LineCount).Where(e => model.Vertical[e]), e => Assert.Equal(0, reading.Level(e)));
        Assert.Equal(2, reading.Paths.Levels);
    }

    [Fact]
    public void ABeamLandingBetweenPanelPoints_LeavesTheTrussOneBody()
    {
        // Two Warren trusses on columns, floor beams landing on the bottom chords and
        // purlins on the top chords, none of them at a panel point. Each landing puts
        // a joint partway along a chord; read stretch by stretch the chord no longer
        // closes its panel and the truss falls apart into a staircase of webs.
        var model = new Models();
        var truss = new List<int>();
        var resting = new List<int>();
        foreach (double y in new[] { 0.0, 8.0 })
        {
            foreach (double x in new[] { 0.0, 12.0 })
            {
                model.Support(x, y, 0.0);
                model.Line(x, y, 0.0, x, y, 4.0);
            }

            for (int i = 0; i < 4; i++)
                truss.Add(model.Line(i * 3.0, y, 4.0, (i + 1) * 3.0, y, 4.0));
            for (int i = 0; i < 3; i++)
                truss.Add(model.Line(1.5 + i * 3.0, y, 7.0, 4.5 + i * 3.0, y, 7.0));
            for (int i = 0; i < 4; i++)
            {
                truss.Add(model.Line(i * 3.0, y, 4.0, 1.5 + i * 3.0, y, 7.0));
                truss.Add(model.Line(1.5 + i * 3.0, y, 7.0, (i + 1) * 3.0, y, 4.0));
            }
        }

        foreach (double x in new[] { 2.0, 4.5, 7.0, 10.0 })
            resting.Add(model.Line(x, 0.0, 4.0, x, 8.0, 4.0));
        foreach (double x in new[] { 3.0, 6.0, 9.0 })
            resting.Add(model.Line(x, 0.0, 7.0, x, 8.0, 7.0));

        var reading = model.Read();

        Assert.Equal(2, truss.Select(reading.Assembly).Distinct().Count());
        Assert.All(truss, e => Assert.Equal(1, reading.Level(e)));
        Assert.All(resting, e => Assert.Equal(2, reading.Level(e)));
        Assert.Equal(2, reading.Paths.Levels);
    }

    [Fact]
    public void WhatStandsOnTheGround_IsLevelZeroWhateverElseItRestsOn()
    {
        // Two braced bays whose beams run on to a third, unbraced column. Each braced
        // bay is one body on its own feet, so it is level 0 even though its beam also
        // lands on the far column; the beams across and the secondaries between rest on
        // the bays and the columns alike, one level up.
        var model = new Models();
        var bay = new List<int>();
        var across = new List<int>();
        foreach (double y in new[] { 0.0, 8.0 })
        {
            foreach (double x in new[] { 0.0, 6.0, 12.0 })
            {
                model.Support(x, y, 0.0);
                bay.Add(model.Line(x, y, 0.0, x, y, 4.0));
            }

            bay.Add(model.Line(0.0, y, 4.0, 6.0, y, 4.0));
            bay.Add(model.Line(6.0, y, 4.0, 12.0, y, 4.0));
            bay.Add(model.Line(0.0, y, 0.0, 6.0, y, 4.0));
            bay.Add(model.Line(6.0, y, 0.0, 0.0, y, 4.0));
        }

        foreach (double x in new[] { 0.0, 3.0, 6.0, 9.0, 12.0 })
            across.Add(model.Line(x, 0.0, 4.0, x, 8.0, 4.0));

        var reading = model.Read();

        Assert.Equal(4, bay.Select(reading.Assembly).Distinct().Count());
        Assert.All(bay, e => Assert.Equal(0, reading.Level(e)));
        Assert.All(across, e => Assert.Equal(1, reading.Level(e)));
        Assert.Equal(1, reading.Paths.Levels);
    }

    [Fact]
    public void ALatticeTower_IsOnTheGroundFromTopToBottom()
    {
        // Four braced faces sharing their legs, three lifts high, a platform across the
        // top girts. The faces are four bodies leaning on each other, and the platform
        // beams land midway along the top girts.
        var model = new Models();
        var corners = new[] { (0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0) };
        var tower = new List<int>();
        foreach (var (x, y) in corners)
        {
            model.Support(x, y, 0.0);
            for (int s = 0; s < 3; s++)
                tower.Add(model.Line(x, y, s * 4.0, x, y, (s + 1) * 4.0));
        }

        for (int f = 0; f < 4; f++)
        {
            var (x1, y1) = corners[f];
            var (x2, y2) = corners[(f + 1) % 4];
            for (int s = 0; s < 3; s++)
            {
                tower.Add(model.Line(x1, y1, (s + 1) * 4.0, x2, y2, (s + 1) * 4.0));
                tower.Add(model.Line(x1, y1, s * 4.0, x2, y2, (s + 1) * 4.0));
            }
        }

        int platform = model.Line(0.0, 2.0, 12.0, 4.0, 2.0, 12.0);
        int platformToo = model.Line(2.0, 0.0, 12.0, 2.0, 4.0, 12.0);

        var reading = model.Read();

        Assert.All(tower, e => Assert.Equal(0, reading.Level(e)));
        Assert.Equal(1, reading.Level(platform));
        Assert.Equal(1, reading.Level(platformToo));
        Assert.All(tower, e => Assert.True(reading.Assemblies.Members[reading.Assembly(e)].Length > 1));
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
