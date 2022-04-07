using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Класс, отвечающий за решение задачи об укладке плитки.
/// </summary>
public class Solver : MonoBehaviour
{
    const float Side = 1;
    const float BigOffset = 100 * Side;

    private bool[] intersections = new bool[3];
    private Point[] interPoitns = new Point[3];

    /// <summary>
    /// Ограничивающая область.
    /// </summary>
    public Transform Bounds;

    /// <summary>
    /// Выполняет укладку плитки и подсчитывает её площадь.
    /// </summary>
    /// <param name="offset">Размер шва.</param>
    /// <param name="angle">Угол наклона.</param>
    /// <param name="shift">Сдвиг.</param>
    public float Run(float offset, float angle, float shift)
    {
        var points = GenerateTilePoints(offset, angle, shift);
        var cutPoints = SliceBounds(points, angle);

        GenerateMeshContent(cutPoints, out var vertices, out var triangles, out var uv);

        var mesh = GetComponent<MeshFilter>().mesh;
        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uv;

        return CalculateArea(cutPoints);
    }

    /// <summary>
    /// Подсчет площади используемых плиток.
    /// </summary>
    private float CalculateArea(Point[] points)
    {
        var area = 0f;
        for (var i = 0; i < points.Length; i += 3)
           area += GetTriangleArea(points[i], points[i + 1], points[i + 2]);;

        return area;
    }

    /// <summary>
    /// Преобразование коллекции точек в данные для генерации Mesh-a.
    /// </summary>
    private void GenerateMeshContent(Point[] result, out Vector3[] vertices, out int[] triangles, out Vector2[] uv)
    {
        vertices = new Vector3[result.Length];
        triangles = new int[result.Length];
        uv = new Vector2[result.Length];

        for (var i = 0; i < result.Length; i += 3)
        {
            vertices[i + 0] = new Vector3(result[i + 0].X, result[i + 0].Y);
            vertices[i + 1] = new Vector3(result[i + 1].X, result[i + 1].Y);
            vertices[i + 2] = new Vector3(result[i + 2].X, result[i + 2].Y);

            triangles[i + 0] = i;
            if (IsClockwise(result[i], result[i + 1], result[i + 2]))
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
    }

    /// <summary>
    /// "Отсекание" плитки за пределами прямоугольной области.
    /// </summary>
    private Point[] SliceBounds(IEnumerable<Point> points, float angle)
    {
        var vSlice = Bounds.localScale.y / 2;
        var hSlice = Bounds.localScale.x / 2;

        var res = points;
        
        res = Slice(res, new Point(-BigOffset, vSlice   ), new Point(BigOffset, vSlice),     true , angle);
        res = Slice(res, new Point(-BigOffset, -vSlice  ), new Point(BigOffset, -vSlice),    false, angle);

        res = Slice(res, new Point(-hSlice   , BigOffset), new Point(-hSlice  , -BigOffset), false, angle);
        res = Slice(res, new Point( hSlice   , BigOffset), new Point( hSlice  , -BigOffset), true , angle);

        return res.ToArray();
    }

    /// <summary>
    /// Генерация точек треугольников, которые составляют плитку.
    /// </summary>
    private IEnumerable<Point> GenerateTilePoints(float offset, float angle, float shift)
    {
        var height = Mathf.RoundToInt(Bounds.localScale.y / (Side + offset) * 2);
        var width = Mathf.RoundToInt(Bounds.localScale.x / (Side + offset) * 2) + 1;

        var startX = width * (Side + offset) / -2;
        var startY = height * (Side + offset) / -2;

        var points = new List<Point>();
        for (var i = 0; i < height; ++i) 
        {
            var startJ = i * shift >= Side / 2
                ? -1
                : 0;

            for (var j = startJ; j < width; ++j)
            {
                var x = startX + i * shift + j * (Side + offset);
                var y = startY + i * (Side + offset);

                var a = new Point(x,        y       , 0, 0).Rotate(angle);
                var b = new Point(x,        y + Side, 0, 1).Rotate(angle);
                var c = new Point(x + Side, y + Side, 1, 1).Rotate(angle);
                var d = new Point(x + Side, y,        1, 0).Rotate(angle);

                yield return a;
                yield return b;
                yield return c;

                yield return a;
                yield return c;
                yield return d;
            }
        }
    }

    /// <summary>
    /// Отсекание точек треугольников с помощью отрезка.
    /// </summary>
    /// <param name="points">Последовательность всех точек треугольников.</param>
    /// <param name="segmentA">Начало отсекающего отрезка.</param>
    /// <param name="segmentB">Конец отсекающего отрезка.</param>
    /// <param name="sign">Требуемый знак ориентированной площади.</param>
    /// <param name="sign">Угол изначального наклона плитки.</param>
    private IEnumerable<Point> Slice(IEnumerable<Point> points, Point segmentA, Point segmentB, bool sign, float angle)
    {
        var enumerator = points.GetEnumerator();
        while (enumerator.MoveNext())
        {
            // Получаем следующие 3 точки
            var triangleA = enumerator.Current;
            enumerator.MoveNext();
            var triangleB = enumerator.Current;
            enumerator.MoveNext();
            var triangleC = enumerator.Current;

            // Ищем пересечение отрезка с каждой из сторон треугольника
            intersections[0] = TryFindIntersection(triangleA, triangleB, segmentA, segmentB, out interPoitns[0]);
            intersections[1] = TryFindIntersection(triangleB, triangleC, segmentA, segmentB, out interPoitns[1]);
            intersections[2] = TryFindIntersection(triangleC, triangleA, segmentA, segmentB, out interPoitns[2]);

            var count = intersections.Count(x => x);

            // Если отрезок НЕ пересекает треугольник И лежит с нужной стороны от него,
            // то оставляем треугольник в исходном виде
            if (count != 2)
            {
                if (IsClockwise(triangleA, segmentA, segmentB) == sign)
                {
                    yield return triangleA;
                    yield return triangleB;
                    yield return triangleC;
                }
                continue;
            }

            // Находим вершину 'a', в которой сходятся стороны 'b' и 'c', пересекаемые отрезком
            // first, second - соответствующие точки пересечения
            Point a, b, c, first, second;
            if (intersections[0] && intersections[2])
            {
                a = triangleA;
                b = triangleB;
                c = triangleC;
                first = interPoitns[0];
                second = interPoitns[2];
            }
            else if (intersections[0] && intersections[1])
            {
                a = triangleB;
                b = triangleA;
                c = triangleC;
                first = interPoitns[0];
                second = interPoitns[1];
            }
            else 
            {
                a = triangleC;
                b = triangleA;
                c = triangleB;
                first = interPoitns[2];
                second = interPoitns[1];
            }

            // Находим положение точек до поворота и вычисляем uv-координаты для точек пересечения
            var rotatedA = a.Rotate(-angle);
            var rotatedB = b.Rotate(-angle);
            var rotatedC = c.Rotate(-angle);
            var rotatedFirst = first.Rotate(-angle);
            var rotatedSecond = second.Rotate(-angle);

            first.U = GetUvCoordinate(rotatedA.X, rotatedB.X, rotatedFirst.X, a.U, b.U);
            first.V = GetUvCoordinate(rotatedA.Y, rotatedB.Y, rotatedFirst.Y, a.V, b.V);
            second.U = GetUvCoordinate(rotatedA.X, rotatedC.X, rotatedSecond.X, a.U, c.U);
            second.V = GetUvCoordinate(rotatedA.Y, rotatedC.Y, rotatedSecond.Y, a.V, c.V);

            // Если вершина 'a' лежит с нужной стороны отрезка, то она будет вершиной нового треугольника
            if (IsClockwise(segmentA, segmentB, a) == sign)
            {
                yield return a;
                yield return first;
                yield return second;               
            }
            else
            {
                // В противном случае разбиваем оставшуюся трапецию на два новых треугольника
                yield return b;
                yield return first;
                yield return second;               

                yield return b;
                yield return c;
                yield return second;               
            }
        }
    }

    /// <summary>
    /// Вычисление UV-координаты для точки, лежащей на отрезке с известными uv-координатами
    /// </summary>
    private float GetUvCoordinate(float aX, float bX, float midX, float aU, float bU)
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

    /// <summary>
    /// Возвращает удвоенную ориентированную площадь.
    /// </summary>
    private float GetTriangleArea2(Point a, Point b, Point c)
    {
        return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
    }

    /// <summary>
    /// Возвращает площадь труегольника.
    /// </summary>
    private float GetTriangleArea(Point a, Point b, Point c)
    {
        return Mathf.Abs(GetTriangleArea2(a, b, c)) / 2f;
    }

    /// <summary>
    /// Возвращает направление повопрота ориентированной площади.
    /// </summary>
    private bool IsClockwise(Point a, Point b, Point c)
    {
        return GetTriangleArea2(a, b, c) < 0;
    }

    /// <summary>
    /// Проверяет, что отрезки расположены достаточно близко для пересечения.
    /// </summary>
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

    /// <summary>
    /// Находит точку пересечения двух отрезков.
    /// </summary>
    private bool TryFindIntersection(Point a, Point b, Point c, Point d, out Point mid)
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

            var equalToAny = mid.EqualsTo(a) || mid.EqualsTo(b) || mid.EqualsTo(c) || mid.EqualsTo(d);
            return !equalToAny;
        }
    }

    /// <summary>
    /// Определитель матрицы 2-х линейных уровнений.
    /// </summary>
    private float Det(float a, float b, float c, float d)
    {
        return a * d - b * c;
    }

    /// <summary>
    /// Вспомогательный класс для описания линии по расположенному на ней отрезку.
    /// </summary>
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
    }

    /// <summary>
    /// Вспомогательный класс 2-мерной точки с uv-координатой
    /// </summary>
    private struct Point
    {
        public float X { get; set;}
        public float Y { get; set;}
        public float U { get; set;}
        public float V { get; set;}

        public Point(float x, float y, float u, float v)
        {
            X = x;
            Y = y;
            U = u;
            V = v;
        }

        public Point(float x, float y)
        {
            X = x;
            Y = y;
            U = V = 0;
        }

        public override string ToString()
        {
            return $"({X}, {Y}, {U}, {V})";
        }

        public bool EqualsTo(Point other)
        {
            return Mathf.Abs(X - other.X) < Mathf.Epsilon && Mathf.Abs(Y - other.Y) < Mathf.Epsilon;
        }

        /// <summary>
        /// Вращает точку вокруг начала координат.
        /// </summary>
        public Point Rotate(float angle)
        {
            var rotated = Quaternion.Euler(0, 0, angle) * new Vector3(X, Y);
            return new Point
            {
                X = rotated.x,
                Y = rotated.y,
                U = U,
                V = V
            };
        }
    }
}
