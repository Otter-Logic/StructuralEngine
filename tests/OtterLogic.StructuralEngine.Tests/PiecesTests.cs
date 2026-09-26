using Xunit;

namespace OtterLogic.StructuralEngine.Tests;

/// <summary>
/// Where a run is cut into the pieces it is made in, tested on things true by
/// construction: a beam line ends at every column it rests on, a column stack is not
/// cut by the beams resting on it, a truss is not cut by its own webs, and none of it
/// depends on which way the model faces.
/// </summary>
public class PiecesTests
{
    [Fact]
    public void ABeamLine_IsCutAtEveryColumn_AndAColumnStackIsNot()
    {
        var model = new Models().Frame(3, 2, 2);
        var reading = model.Read();
        var pieces = reading.Pieces();

        Assert.True(pieces.Traced);
        for (int e = 0; e < model.LineCount; e++)
        {
            int piece = pieces.Of[e];
            if (model.Vertical[e])
            {
                // Two storeys, one piece, standing free a storey at a time.
                Assert.Equal(8.0, pieces.Span[piece], 6);
                Assert.Equal(4.0, pieces.Unbraced[piece], 6);
                Assert.Equal(1.0, pieces.Upright[piece], 6);
            }
            else
            {
                // A bay, column to column.
                Assert.Equal(6.0, pieces.Span[piece], 6);
                Assert.Equal(new[] { e }, pieces.Elements[piece]);
                Assert.Equal(0.0, pieces.Upright[piece], 6);
            }
        }

        int columns = 4 * 3;
        int bays = 2 * (3 * 3 + 4 * 2);
        Assert.Equal(columns + bays, pieces.Count);
    }

    [Fact]
    public void ALineDrawnStraightOverABeam_IsCutWhereItRestsOnIt()
    {
        // A secondary run across the middle beam line, noded where it rests on it halfway.
        var model = new Models().Frame(2, 2, 1);
        int secondary = model.Line(3, 0, 4, 3, 6, 4);
        model.Line(3, 6, 4, 3, 12, 4);

        var reading = model.Read();
        var pieces = reading.Pieces();

        var own = Enumerable.Range(0, pieces.Count).Where(p => pieces.Member[p] == reading.Members.Of[secondary]).ToArray();
        Assert.Equal(2, own.Length);
        Assert.All(own, p => Assert.Equal(6.0, pieces.Span[p], 6));

        // The beam line it crosses receives there, so it is cut at its columns only.
        int middleBeam = Enumerable.Range(0, secondary).Single(e => !model.Vertical[e]
            && model.Start(e).Y == 6.0 && model.End(e).Y == 6.0 && model.Start(e).X == 0.0);
        Assert.Equal(6.0, pieces.Span[pieces.Of[middleBeam]], 6);
        Assert.Equal(3.0, pieces.Unbraced[pieces.Of[middleBeam]], 6);
    }

    [Fact]
    public void ATrussChord_IsNotCutByItsOwnWebs()
    {
        var model = new Models();
        model.Support(0, 0, 0);
        model.Support(12, 0, 0);
        int chord = model.Line(0, 0, 0, 12, 0, 0);
        for (int i = 0; i < 3; i++)
            model.Line(1.5 + i * 3, 0, 2, 4.5 + i * 3, 0, 2);
        for (int i = 0; i < 4; i++)
        {
            model.Line(i * 3, 0, 0, 1.5 + i * 3, 0, 2);
            model.Line(1.5 + i * 3, 0, 2, (i + 1) * 3, 0, 0);
        }

        var pieces = model.Read().Pieces();

        Assert.Equal(12.0, pieces.Span[pieces.Of[chord]], 6);
        Assert.Equal(3.0, pieces.Unbraced[pieces.Of[chord]], 6);
    }

    [Fact]
    public void TurningTheModelAboutTheVertical_CutsTheSamePieces()
    {
        Models Turned(double degrees)
        {
            var flat = new Models().Frame(3, 2, 2);
            flat.Line(3, 0, 4, 3, 6, 4);
            flat.Line(3, 6, 4, 3, 12, 4);
            return flat.TurnedAboutVertical(degrees);
        }

        var square = Turned(0.0).Read().Pieces();
        var turned = Turned(37.0).Read().Pieces();

        Assert.Equal(square.Of, turned.Of);
        Assert.Equal(square.Member, turned.Member);
        for (int p = 0; p < square.Count; p++)
            Assert.Equal(square.Span[p], turned.Span[p], 6);
    }

    [Fact]
    public void WithoutSupports_EveryRunIsOnePiece()
    {
        var reading = new Models().Frame(2, 2, 2).Read(withSupports: false);
        var pieces = reading.Pieces();

        Assert.False(pieces.Traced);
        Assert.Equal(0, pieces.Cuts);
        Assert.Equal(reading.Members.Count, pieces.Count);
        for (int e = 0; e < reading.Structure.ElementCount; e++)
            Assert.Equal(reading.Members.Of[e], pieces.Member[pieces.Of[e]]);
    }
}
