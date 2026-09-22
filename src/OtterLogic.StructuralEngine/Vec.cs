namespace OtterLogic.StructuralEngine;

/// <summary>A point or direction in model space. Just enough vector arithmetic for reading a stick model.</summary>
public readonly record struct Vec(double X, double Y, double Z)
{
    public static Vec operator +(Vec a, Vec b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static Vec operator -(Vec a, Vec b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Vec operator *(double s, Vec a) => new(s * a.X, s * a.Y, s * a.Z);

    public static Vec operator /(Vec a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    public double Dot(Vec other) => X * other.X + Y * other.Y + Z * other.Z;

    public Vec Cross(Vec other) => new(Y * other.Z - Z * other.Y, Z * other.X - X * other.Z, X * other.Y - Y * other.X);

    public double Length => Math.Sqrt(Dot(this));

    public double DistanceTo(Vec other) => (this - other).Length;

    /// <summary>Row <paramref name="i"/> of an n x 3 array.</summary>
    public static Vec Row(double[,] rows, int i) => new(rows[i, 0], rows[i, 1], rows[i, 2]);
}
