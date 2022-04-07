using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Solver : MonoBehaviour
{
    void Start()
    {
        const float StartX = -20;
        const float StartY = -10;

        const float Side = 1;
        const float Offset = 0.1f;
        const float Shift = 0.3f;
        const float Angle = 30; //30;

        const int Height = 20;
        const int Width = 20;

        var zoneA = new Point { X = -6, Y = 4.5f };
        var zoneB = new Point { X = 6, Y = -4.5f };

        var quaternion = Quaternion.Euler(0, 0, Angle);
        var tiles = new List<Tile>();

        var points = new List<Point>();

        for (var i = 0; i < Height; ++i)
            for (var j = 0; j < Width; ++j)
            {
                var x = StartX + i * Shift + j * (Side + Offset);
                var y = StartY + i * (Side + Offset);

                var tile = new Tile
                {
                    A = new Point
                    {
                        X = x,
                        Y = y,
                        U = 0,
                        V = 0
                    },
                    B = new Point
                    {
                        X = x,
                        Y = y + Side,
                        U = 0,
                        V = 1
                    },
                    C = new Point
                    {
                        X = x + Side,
                        Y = y + Side,
                        U = 1,
                        V = 1
                    },
                    D = new Point
                    {
                        X = x + Side,
                        Y = y,
                        U = 1,
                        V = 0
                    }
                };

                var a = quaternion * new Vector3(tile.A.X, tile.A.Y);
                tile.A = new Point { X = a.x, Y = a.y, U = tile.A.U, V = tile.A.V };

                var b = quaternion * new Vector3(tile.B.X, tile.B.Y);
                tile.B = new Point { X = b.x, Y = b.y, U = tile.B.U, V = tile.B.V };

                var c = quaternion * new Vector3(tile.C.X, tile.C.Y);
                tile.C = new Point { X = c.x, Y = c.y, U = tile.C.U, V = tile.C.V };

                var d = quaternion * new Vector3(tile.D.X, tile.D.Y);
                tile.D = new Point { X = d.x, Y = d.y, U = tile.D.U, V = tile.D.V };
                
                points.Add(tile.A);
                points.Add(tile.B);
                points.Add(tile.C);

                points.Add(tile.A);
                points.Add(tile.C);
                points.Add(tile.D);
            }

        IEnumerable<Point> p = points;
        // p = Slice(p, new Point { X = -100, Y = zoneA.Y }, new Point { X = 100, Y =  zoneA.Y }, true, Angle);
        // p = Slice(p, new Point { X = -100, Y =  zoneB.Y }, new Point { X = 100, Y =  zoneB.Y }, false, Angle);
        // p = Slice(p, new Point { X = zoneA.X, Y = 100 }, new Point { X = zoneA.X, Y = -100 }, false, Angle);
        // p = Slice(p, new Point { X = zoneB.X, Y = 100 }, new Point { X = zoneB.X, Y = -100 }, true, Angle);
        var result = p.ToArray();

        var vertices = new Vector3[result.Length];
        var triangles = new int[result.Length];
        var uv = new Vector2[result.Length];

        for (var i = 0; i < result.Length; i += 3)
        {
            vertices[i + 0] = new Vector3(result[i + 0].X, result[i + 0].Y);
            vertices[i + 1] = new Vector3(result[i + 1].X, result[i + 1].Y);
            vertices[i + 2] = new Vector3(result[i + 2].X, result[i + 2].Y);

            triangles[i + 0] = i;
            if (Clockwise(result[i], result[i + 1], result[i + 2]))
            {
                triangles[i + 1] = i + 1;
                triangles[i + 2] = i + 2;
            }
            else 
            {
                triangles[i + 1] = i + 2;
                triangles[i + 2] = i + 1;
            }

            uv[i + 0] = new Vector2(result[i + 0].U, result[i + 0].V);
            uv[i + 1] = new Vector2(result[i + 1].U, result[i + 1].V);
            uv[i + 2] = new Vector2(result[i + 2].U, result[i + 2].V);
        }

        var mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uv;

        GetComponent<MeshFilter>().mesh = mesh;
    }

    private IEnumerable<Point> Slice(IEnumerable<Point> points, Point segmentA, Point segmentB, bool sign, float angle)
    {
        var enumerator = points.GetEnumerator();
        while (enumerator.MoveNext())
        {
            var triangleA = enumerator.Current;
            enumerator.MoveNext();
            var triangleB = enumerator.Current;
            enumerator.MoveNext();
            var triangleC = enumerator.Current;

            var intersections = new bool[3];
            var mids = new Point[3];

            intersections[0] = Intersect(triangleA, triangleB, segmentA, segmentB, out mids[0]);
            intersections[1] = Intersect(triangleB, triangleC, segmentA, segmentB, out mids[1]);
            intersections[2] = Intersect(triangleC, triangleA, segmentA, segmentB, out mids[2]);

            var count = intersections.Count(x => x);

            if (count != 2)
            {
                if (Clockwise(triangleA, segmentA, segmentB) == sign)
                {
                    yield return triangleA;
                    yield return triangleB;
                    yield return triangleC;
                }
                continue;
            }

            Point a, b, c, first, second;
            if (intersections[0] && intersections[2])
            {
                a = triangleA;
                b = triangleB;
                c = triangleC;
                first = mids[0];
                second = mids[2];
            }
            else if (intersections[0] && intersections[1])
            {
                a = triangleB;
                b = triangleA;
                c = triangleC;
                first = mids[0];
                second = mids[1];
            }
            else 
            {
                a = triangleC;
                b = triangleA;
                c = triangleB;
                first = mids[2];
                second = mids[1];
            }

            var rotatedA = Rotate(a, -angle);
            var rotatedB = Rotate(b, -angle);
            var rotatedC = Rotate(c, -angle);
            var rotatedFirst = Rotate(first, -angle);
            var rotatedSecond = Rotate(second, -angle);

            first.U = Bar(rotatedA.x, rotatedB.x, rotatedFirst.x, a.U, b.U);
            first.V = Bar(rotatedA.y, rotatedB.y, rotatedFirst.y, a.V, b.V);
            second.U = Bar(rotatedA.x, rotatedC.x, rotatedSecond.x, a.U, c.U);
            second.V = Bar(rotatedA.y, rotatedC.y, rotatedSecond.y, a.V, c.V);

            if (Clockwise(segmentA, segmentB, a) == sign)
            {
                yield return a;
                yield return first;
                yield return second;               
            }
            else
            {
                yield return b;
                yield return first;
                yield return second;               

                yield return b;
                yield return c;
                yield return second;               
            }
        }
    }

    Vector3 Rotate(Point p, float angle)
    {
        var quaternion = Quaternion.Euler(0, 0, angle);

        return quaternion * new Vector3(p.X, p.Y);
    }

    private float Bar(float aX, float bX, float midX, float aU, float bU)
    {
        var dist = Mathf.Abs(aX - bX);
        if (dist < Mathf.Epsilon)
            return aU;

        var midDist = Mathf.Abs(midX - aX);
        var uDist = Mathf.Abs(aU - bU);

        var shift = midDist * uDist / dist;
        if (bU < aU)
            shift *= -1;

        return aU + shift;
            
    }

    private float TriangleArea2(Point a, Point b, Point c)
    {
        return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
    }

    private bool Clockwise(Point a, Point b, Point c)
    {
        return TriangleArea2(a, b, c) < 0;
    }

    private bool Intersect1(float a, float b, float c, float d) {
        if (a > b)
        {
            var t = a;
            a = b;
            b = t;
        }

        if (c > d)
        {
            var t = c;
            c = d;
            d = t;
        }

        return Mathf.Max(a, c) <= Mathf.Min(b, d) + Mathf.Epsilon;
    }

    private bool Intersect(Point a, Point b, Point c, Point d, out Point mid)
    {
        mid = default;
        if (!Intersect1(a.X, b.X, c.X, d.X) || !Intersect1(a.Y, b.Y, c.Y, d.Y))
            return false;

        var m = new Line(a, b);
        var n = new Line(c, d);

        var zn = Det(m.A, m.B, n.A, n.B);

        if (Mathf.Abs(zn) < Mathf.Epsilon)
        {
            return false;
        }
        else
        {
            mid.X = -Det(m.C, m.B, n.C, n.B) / zn;
            mid.Y = -Det(m.A, m.C, n.A, n.C) / zn;

            var equalToAny = mid.Equals(a) || mid.Equals(b) || mid.Equals(c) || mid.Equals(d);
            return !equalToAny;
        }
    }

    private float Det(float a, float b, float c, float d)
    {
        return a * d - b * c;
    }

    private bool Less(Point a, Point b)
    {
        return (a.X < b.X - Mathf.Epsilon)
            || Mathf.Abs(a.X - b.X) < Mathf.Epsilon
            || (a.Y < b.Y - Mathf.Epsilon);
    }

    private bool Betw(float l, float r, float x)
    {
        return Mathf.Min(l, r) <= x + Mathf.Epsilon
            && x <= Mathf.Max(l, r) + Mathf.Epsilon;
    }

    private struct Line
    {
        public float A { get; set; }
        public float B { get; set; }
        public float C { get; set; }

        public Line(Point p, Point q)
        {
            A = p.Y - q.Y;
            B = q.X - p.X;
            C = -A * p.X - B * p.Y;
        }

        public void Norm()
        {
            var z = Mathf.Sqrt(A * A + B * B);
            if (Mathf.Abs(z) > Mathf.Epsilon)
            {
                A /= z;
                B /= z;
                C /= z;
            }
        }

        public float Dist(Point p)
        {
            return A * p.X + B * p.Y + C;
        }
    }

    private struct Tile
    {
        public Point A { get; set; }
        public Point B { get; set; }
        public Point C { get; set; }
        public Point D { get; set; }

    }

    private struct Point
    {
        public float X { get; set;}
        public float Y { get; set;}
        public float U { get; set;}
        public float V { get; set;}

        public override string ToString()
        {
            return $"({X}, {Y}, {U}, {V})";
        }

        public bool Equals(Point other)
        {
            return Mathf.Abs(X - other.X) < Mathf.Epsilon && Mathf.Abs(Y - other.Y) < Mathf.Epsilon;
        }
    }
}
