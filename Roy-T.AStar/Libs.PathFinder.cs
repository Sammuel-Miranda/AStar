using Sys = global::System;
using SysClG = global::System.Collections.Generic;
using SysCoS = global::System.Runtime.CompilerServices;
using SysDiag = global::System.Diagnostics;
using LibPF = global::Libs.PathFinder;
#region Assembly
[assembly: Sys.Reflection.AssemblyTitle("PathFinder")]
[assembly: Sys.Reflection.AssemblyDescription("")]
[assembly: Sys.Reflection.AssemblyConfiguration("")]
[assembly: Sys.Reflection.AssemblyCompany("")]
[assembly: Sys.Reflection.AssemblyProduct("PathFinder")]
[assembly: Sys.Reflection.AssemblyCopyright("Copyright ©  2019")]
[assembly: Sys.Reflection.AssemblyTrademark("")]
[assembly: Sys.Reflection.AssemblyCulture("")]
[assembly: Sys.Runtime.InteropServices.ComVisible(false)]
[assembly: Sys.Runtime.InteropServices.Guid("83874e00-fae0-4e0b-bbf0-9632b32ea1ed")]
[assembly: Sys.Reflection.AssemblyVersion("1.0.0.0")]
[assembly: Sys.Reflection.AssemblyFileVersion("1.0.0.0")]
#endregion
namespace Libs.PathFinder
{
    public struct Position : Sys.IEquatable<LibPF.Position>
    {
        public int X { get; private set; }
        public int Y { get; private set; }
        public override string ToString() { return "Position: (" + this.X.ToString() + ", " + this.Y.ToString() + ")"; }
        public bool Equals(LibPF.Position other) { return this.X == other.X && this.Y == other.Y; }
        public override bool Equals(object obj) { if (object.ReferenceEquals(null, obj)) { return false; } else { return obj is LibPF.Position && Equals((LibPF.Position) obj); } }
        public static bool operator ==(LibPF.Position a, LibPF.Position b) { return a.Equals(b); }
        public static bool operator !=(LibPF.Position a, LibPF.Position b) { return !a.Equals(b); }
        public static LibPF.Position operator +(LibPF.Position a, LibPF.Position b) { return new LibPF.Position(a.X + b.X, a.Y + b.Y); }
        public static LibPF.Position operator -(LibPF.Position a, LibPF.Position b) { return new LibPF.Position(a.X - b.X, a.Y - b.Y); }
        public override int GetHashCode() { unchecked { return (this.X * 397) ^ this.Y; } }

        public Position(int x, int y) : this()
        {
            this.X = x;
            this.Y = y;
        }
    }

    public struct Offset : Sys.IEquatable<LibPF.Offset>
    {
        private const float DiagonalCost = 1.4142135623730950488016887242097f; // sqrt(2)
        private const float LateralCost = 1.0f;
        public int X { get; private set; }
        public int Y { get; private set; }
        public float Cost { get; private set; }
        public override string ToString() { return "Offset: (" + this.X.ToString() + ", " + this.Y.ToString() + ")"; }
        public bool Equals(LibPF.Offset other) { return this.X == other.X && this.Y == other.Y; }
        public override bool Equals(object obj) { if (object.ReferenceEquals(null, obj)) { return false; } else { return obj is LibPF.Offset && this.Equals((LibPF.Offset)obj); } }
        public static bool operator ==(LibPF.Offset a, LibPF.Offset b) { return a.Equals(b); }
        public static bool operator !=(LibPF.Offset a, LibPF.Offset b) { return !a.Equals(b); }
        public static LibPF.Position operator +(LibPF.Offset a, LibPF.Position b) { return new LibPF.Position(a.X + b.X, a.Y + b.Y); }
        public static LibPF.Position operator -(LibPF.Offset a, LibPF.Position b) { return new LibPF.Position(a.X - b.X, a.Y - b.Y); }
        public static LibPF.Position operator +(LibPF.Position a, LibPF.Offset b) { return new LibPF.Position(a.X + b.X, a.Y + b.Y); }
        public static LibPF.Position operator -(LibPF.Position a, LibPF.Offset b) { return new LibPF.Position(a.X - b.X, a.Y - b.Y); }
        public override int GetHashCode() { unchecked { return (this.X * 397) ^ this.Y; } }

        public Offset(int x, int y) : this()
        {
            if (x < -1 || x > 1) throw new Sys.ArgumentOutOfRangeException("x", "Parameter x cannot have a magnitude larger than one");
            else if (y < -1 || y > 1) throw new Sys.ArgumentOutOfRangeException("y", "Parameter y cannot have a magnitude larger than one");
            else if (x == 0 && y == 0) throw new Sys.ArgumentException("y", "Paramters x and y cannot both be zero");
            this.X = x;
            this.Y = y;
            this.Cost = (x != 0 && y != 0) ? LibPF.Offset.DiagonalCost : LibPF.Offset.LateralCost;
        }
    }

    public static class MovementPatterns
    {
        public static readonly LibPF.Offset[] Full = { new LibPF.Offset(-1, -1), new LibPF.Offset(0, -1), new LibPF.Offset(1, -1), new LibPF.Offset(-1, 0), new LibPF.Offset(1, 0), new LibPF.Offset(-1, 1), new LibPF.Offset(0, 1), new LibPF.Offset(1, 1) };
        public static readonly LibPF.Offset[] LateralOnly = { new LibPF.Offset(0, -1), new LibPF.Offset(-1, 0), new LibPF.Offset(1, 0), new LibPF.Offset(0, 1) };
        public static readonly LibPF.Offset[] DiagonalOnly = { new LibPF.Offset(-1, -1), new LibPF.Offset(1, -1), new LibPF.Offset(-1, 1), new LibPF.Offset(1, 1) };
    }

    public sealed class Grid
    {
        private float DefaultCost;
        private float[] Weights;
        public int DimX { get; private set; }
        public int DimY { get; private set; }
        /* https://www.dotnetperls.com/aggressiveinlining , https://stackoverflow.com/questions/43060376/aggressiveinlining-doesnt-exist */ [SysCoS.MethodImpl(256)] internal int GetIndexUnchecked(int x, int y) { return (this.DimX * y) + x; }
        private int GetIndex(int x, int y) { if (x < 0 || x >= this.DimX) { throw new Sys.ArgumentOutOfRangeException("The x-coordinate " + x.ToString() + " is outside of the expected range [0..." + this.DimX.ToString() + ")"); } else if (y < 0 || y >= this.DimY) { throw new Sys.ArgumentOutOfRangeException("The y-coordinate " + y.ToString() + " is outside of the expected range [0..." + this.DimY.ToString() + ")"); } else { return this.GetIndexUnchecked(x, y); } }
        public void SetCellCost(int X, int Y, float cost) { if (cost < 1) { throw new Sys.ArgumentOutOfRangeException("Argument cost with value " + cost.ToString() + " is invalid. The cost of traversing a cell cannot be less than one"); } else { this.Weights[this.GetIndex(X, Y)] = cost; } }
        public void SetCellCost(LibPF.Position position, float cost) { this.SetCellCost(position.X, position.Y, cost); }
        public float GetCellCost(int X, int Y) { return this.Weights[this.GetIndex(X, Y)]; }
        public float GetCellCost(LibPF.Position position) { return this.GetCellCost(position.X, position.Y); }
        public void BlockCell(int X, int Y) { this.SetCellCost(X, Y, float.PositiveInfinity); }
        public void BlockCell(LibPF.Position position) { this.BlockCell(position.X, position.Y); }
        public void UnblockCell(int X, int Y) { this.SetCellCost(X, Y, this.DefaultCost); }
        public void UnblockCell(LibPF.Position position) { this.SetCellCost(position.X, position.Y, this.DefaultCost); }
        internal float GetCellCostUnchecked(LibPF.Position position) { return this.Weights[this.GetIndexUnchecked(position.X, position.Y)]; }
        public LibPF.Position[] GetPath(LibPF.Position start, LibPF.Position end) { return this.GetPath(start, end, LibPF.MovementPatterns.Full); }
        public LibPF.Position[] GetPath(LibPF.Position start, LibPF.Position end, LibPF.Offset[] movementPattern) { return this.GetPath(start, end, movementPattern, int.MaxValue); }

        public LibPF.Position[] GetPath(LibPF.Position start, LibPF.Position end, LibPF.Offset[] movementPattern, int iterationLimit)
        {
            SysClG.List<LibPF.Position> current = null;
            using (LibPF.Finder finder = new LibPF.Finder()) { current = finder.FindPath(this, start, end, movementPattern, iterationLimit); }
            if (current == null) { return new LibPF.Position[0]; }
            else
            {
                LibPF.Position[] ar = new LibPF.Position[current.Count];
                int idx = current.Count - 1;
                foreach (LibPF.Position step in current)
                {
                    ar[idx] = step;
                    idx--;
                }
                return ar;
            }
        }

        public override string ToString()
        {
            Sys.Text.StringBuilder txt = new Sys.Text.StringBuilder(this.Weights.Length * 4);
            txt.Append("{\"w\":" + this.DimX.ToString());
            txt.Append(",\"h\":" + this.DimY.ToString());
            txt.Append(",\"c\":" + this.DefaultCost.ToString());
            txt.Append(",\"d\":[");
            SysClG.IEnumerator<float> en = (this.Weights as SysClG.IEnumerable<float>).GetEnumerator();
            en.MoveNext();
            txt.Append(en.Current.ToString(Sys.Globalization.CultureInfo.InvariantCulture));
            while (en.MoveNext()) { txt.Append("," + en.Current.ToString(Sys.Globalization.CultureInfo.InvariantCulture)); }
            txt.Append("]}");
            return txt.ToString();
        }

        public void Reset(int dimX, int dimY, float defaultCost = 1.0f)
        {
            if (defaultCost < 1.0f) { throw new Sys.ArgumentOutOfRangeException("Argument defaultCost with value " + defaultCost.ToString() + " is invalid. The cost of traversing a cell cannot be less than one"); }
            this.DefaultCost = defaultCost;
            this.Weights = new float[dimX * dimY];
            this.DimX = dimX;
            this.DimY = dimY;
            for (int n = 0; n < this.Weights.Length; n++) { this.Weights[n] = defaultCost; }
        }

        public string ToJSon() { return this.ToString(); }
        private bool ParseI(string val, out int i) { return int.TryParse(val, Sys.Globalization.NumberStyles.Integer, Sys.Globalization.CultureInfo.InvariantCulture, out i); }
        private bool ParseF(string val, out float f) { return float.TryParse(val, Sys.Globalization.NumberStyles.Float, Sys.Globalization.CultureInfo.InvariantCulture, out f); }

        private string ParseS(ref string val, int i, ref int lIdx)
        {
            string term = val.Substring(lIdx, (i - lIdx));
            lIdx = (i + 1);
            return term;
        }

        private void ParseList(ref string listData)
        {
            if (listData.StartsWith("[") && listData.EndsWith("]"))
            {
                listData = listData.Substring(1, (listData.Length - 2));
                string[] wTrms = listData.Split(new char[] { ',' }, Sys.StringSplitOptions.RemoveEmptyEntries);
                if (wTrms == null || wTrms.Length == 0) { throw new Sys.Exception(); }
                else
                {
                    this.Weights = new float[wTrms.Length];
                    string cur = string.Empty;
                    for (int i = 0; i < wTrms.Length; i++)
                    {
                        cur = wTrms[i];
                        if (cur == "infinity") { this.Weights[i] = float.PositiveInfinity; } else if (!this.ParseF(cur, out this.Weights[i])) { throw new Sys.Exception(); }
                    }
                }
            } else { throw new Sys.Exception(); }
        }

        private void ParseTerm(ref string term, ref string JSon, int i, ref int lIdx)
        {
            float termF = 0.0f;
            int termI = 0;
            switch (term)
            {
                case "\"w\"":
                case "w": if (this.ParseI(this.ParseS(ref JSon, i, ref lIdx), out termI)) { this.DimX = termI; } else { throw new Sys.Exception(); } break;
                case "\"h\"":
                case "h": if (this.ParseI(this.ParseS(ref JSon, i, ref lIdx), out termI)) { this.DimY = termI; } else { throw new Sys.Exception(); } break;
                case "\"c\"":
                case "c": if (this.ParseF(this.ParseS(ref JSon, i, ref lIdx), out termF)) { this.DefaultCost = termF; } else { throw new Sys.Exception(); } break;
                case "\"d\"":
                case "d":
                    term = this.ParseS(ref JSon, i, ref lIdx);
                    this.ParseList(ref term);
                    break;
                default: throw new Sys.Exception();
            }
            term = string.Empty;
        }

        public void FromJSon(string JSon)
        {
            if (string.IsNullOrEmpty(JSon)) { throw new Sys.ArgumentOutOfRangeException("JSon data is empty"); }
            else
            {
                JSon = JSon.Replace(" ", string.Empty).Replace("\r", string.Empty).Replace("\n", string.Empty).Replace("\t", string.Empty).ToLower();
                if (JSon.StartsWith("{") && JSon.EndsWith("}"))
                {
                    JSon = JSon.Substring(1, (JSon.Length - 2));
                    this.Reset(1, 1, 1.0f);
                    try
                    {
                        string term = string.Empty;
                        int lIdx = 0;
                        int depth = 0;
                        for (int i = 0; i < JSon.Length; i++)
                        {
                            char tk = JSon[i];
                            if (tk == ':') { term = this.ParseS(ref JSon, i, ref lIdx); }
                            else if (tk == '[')
                            {
                                depth++;
                                if (depth > 1) { throw new Sys.Exception(); }
                            }
                            else if (tk == ']')
                            {
                                depth--;
                                if (depth < 0) { throw new Sys.Exception(); }
                            }
                            else if ((depth == 0) && (term != string.Empty) && (tk == ',')) { this.ParseTerm(ref term, ref JSon, i, ref lIdx); }
                        }
                        if (term != string.Empty) { this.ParseTerm(ref term, ref JSon, JSon.Length, ref lIdx); }
                    } catch { this.Reset(1, 1, 1.0f); }
                    if (this.DimX < 1 || this.DimY < 1 || this.Weights == null || this.Weights.Length != (this.DimX * this.DimY))
                    {
                        this.Reset(1, 1, 1.0f);
                        throw new Sys.ArgumentException("Arguments resulted on a incorret Grid, by size x, size y and number of discribed weights");
                    }
                } else { throw new Sys.ArgumentOutOfRangeException("JSon data is not encased by \"{\" \"}\""); }
            }
        }

        public Grid(int dimX, int dimY, float defaultCost = 1.0f) { this.Reset(dimX, dimY, defaultCost: defaultCost); }
    }

    internal class Finder : Sys.IDisposable
    {
#if DEBUG
        private enum StepType : byte
        {
            Current = 0,
            Open = 1,
            Close = 2
        }

        private sealed class Step
        {
            public LibPF.Finder.StepType Type;
            public LibPF.Position Position;
            public SysClG.List<LibPF.Position> Path;

            public Step(LibPF.Finder.StepType type, LibPF.Position position, SysClG.List<LibPF.Position> path)
            {
                this.Type = type;
                this.Position = position;
                this.Path = path;
            }
        }
#endif
        private sealed class MinHeapNode
        {
            public LibPF.Position Position { get; private set; }
            public float ExpectedCost { get; set; }
            public LibPF.Finder.MinHeapNode Next { get; set; }

            public MinHeapNode(LibPF.Position position, float expectedCost)
            {
                this.Position = position;
                this.ExpectedCost = expectedCost;
            }
        }

        private sealed class MinHeap
        {
            private LibPF.Finder.MinHeapNode head;
            public bool HasNext() { return (this.head != null); }

            public void Push(LibPF.Finder.MinHeapNode node)
            {
                if (this.head == null) { this.head = node; }
                else if (node.ExpectedCost < this.head.ExpectedCost)
                {
                    node.Next = this.head;
                    this.head = node;
                }
                else
                {
                    LibPF.Finder.MinHeapNode current = this.head;
                    while (current.Next != null && current.Next.ExpectedCost <= node.ExpectedCost) { current = current.Next; }
                    node.Next = current.Next;
                    current.Next = node;
                }
            }

            public LibPF.Finder.MinHeapNode Pop()
            {
                LibPF.Finder.MinHeapNode top = this.head;
                this.head = this.head.Next;
                return top;
            }
        }
#if DEBUG
        private SysClG.List<LibPF.Finder.Step> StepList = new SysClG.List<LibPF.Finder.Step>(0);
        private void ClearStepList() { this.StepList.Clear(); }
        private void MessageOpen(LibPF.Position position) { this.StepList.Add(new LibPF.Finder.Step(LibPF.Finder.StepType.Open, position, new SysClG.List<LibPF.Position>(0))); }
        private void MessageClose(LibPF.Position position) { this.StepList.Add(new Step(LibPF.Finder.StepType.Close, position, new SysClG.List<LibPF.Position>(0))); }
        private void MessageCurrent(LibPF.Position position, SysClG.List<LibPF.Position> path) { this.StepList.Add(new LibPF.Finder.Step(LibPF.Finder.StepType.Current, position, path)); }
#endif
        /* https://www.dotnetperls.com/aggressiveinlining , https://stackoverflow.com/questions/43060376/aggressiveinlining-doesnt-exist */ [SysCoS.MethodImpl(256)] private float ManhattanDistance(LibPF.Position p0, LibPF.Position p1) { return Sys.Math.Abs(p0.X - p1.X) + Sys.Math.Abs(p0.Y - p1.Y); }

        private SysClG.IEnumerable<LibPF.Offset> GetMovementOptions(LibPF.Position position, int dimX, int dimY, SysClG.IEnumerable<LibPF.Offset> movementPattern)
        {
            LibPF.Position target = default(LibPF.Position);
            foreach (LibPF.Offset m in movementPattern)
            {
                target = position + m;
                if (target.X >= 0 && target.X < dimX && target.Y >= 0 && target.Y < dimY) { yield return m; }
            }
        }

        private void StepOn(LibPF.Grid grid, LibPF.Finder.MinHeap open, LibPF.Position[] cameFrom, float[] costSoFar, LibPF.Offset[] movementPattern, LibPF.Position current, LibPF.Position end)
        {
            float initialCost = costSoFar[grid.GetIndexUnchecked(current.X, current.Y)];
            foreach (LibPF.Offset option in this.GetMovementOptions(current, grid.DimX, grid.DimY, movementPattern))
            {
                LibPF.Position position = current + option;
                float cellCost = grid.GetCellCostUnchecked(position);
                if (float.IsInfinity(cellCost)) { continue; }
                int index = grid.GetIndexUnchecked(position.X, position.Y);
                float newCost = initialCost + cellCost * option.Cost;
                float oldCost = costSoFar[index];
                if (!(oldCost <= 0) && !(newCost < oldCost)) { continue; }
                costSoFar[index] = newCost;
                cameFrom[index] = current;
                newCost = newCost + this.ManhattanDistance(position, end);
                open.Push(new LibPF.Finder.MinHeapNode(position, newCost));
#if DEBUG
                this.MessageOpen(position);
#endif
            }
        }

        private SysClG.List<LibPF.Position> ReconstructPath(LibPF.Grid grid, LibPF.Position start, LibPF.Position end, LibPF.Position[] cameFrom)
        {
            SysClG.List<LibPF.Position> path = new SysClG.List<LibPF.Position> { end };
            LibPF.Position current = end;
            do
            {
                LibPF.Position previous = cameFrom[grid.GetIndexUnchecked(current.X, current.Y)];
                current = previous;
                path.Add(current);
            } while (current != start);
            return path;
        }
#if DEBUG
        private SysClG.List<LibPF.Position> PartiallyReconstructPath(LibPF.Grid grid, LibPF.Position start, LibPF.Position end, LibPF.Position[] cameFrom)
        {
            SysClG.List<LibPF.Position> path = new SysClG.List<LibPF.Position> { end };
            LibPF.Position current = end;
            do
            {
                LibPF.Position previous = cameFrom[grid.GetIndexUnchecked(current.X, current.Y)];
                if (current == previous) return new SysClG.List<LibPF.Position>();
                current = previous;
                path.Add(current);
            } while (current != start);
            return path;
        }
#endif
        public SysClG.List<LibPF.Position> FindPath(LibPF.Grid grid, LibPF.Position start, LibPF.Position end, LibPF.Offset[] movementPattern, int iterationLimit)
        {
#if DEBUG
            this.ClearStepList();
#endif
            if (start == end) { return new SysClG.List<LibPF.Position> { start }; }
            LibPF.Finder.MinHeapNode head = new LibPF.Finder.MinHeapNode(start, this.ManhattanDistance(start, end));
            LibPF.Finder.MinHeap open = new LibPF.Finder.MinHeap();
            open.Push(head);
            float[] costSoFar = new float[grid.DimX * grid.DimY];
            LibPF.Position[] cameFrom = new LibPF.Position[grid.DimX * grid.DimY];
            while (open.HasNext() && iterationLimit > 0)
            {
                LibPF.Position current = open.Pop().Position;
#if DEBUG
                this.MessageCurrent(current, this.PartiallyReconstructPath(grid, start, current, cameFrom));
#endif
                if (current == end) { return this.ReconstructPath(grid, start, end, cameFrom); }
                this.StepOn(grid, open, cameFrom, costSoFar, movementPattern, current, end);
#if DEBUG
                this.MessageClose(current);
#endif
                --iterationLimit;
            }
            return null;
        }

        public void Dispose()
        {
#if DEBUG
            this.ClearStepList();
            this.StepList = null;
#endif
        }
    }
}
#if DEBUG
internal static class Test
{
    private static void Run(int Size, LibPF.Position start, LibPF.Position end, params LibPF.Position[] blocks)
    {
        if (Size > 99) { Size = 99; }
        LibPF.Grid g = new LibPF.Grid(Size, Size, defaultCost: 1.0f);
        foreach (LibPF.Position block in blocks) { g.BlockCell(block); }
        string json = g.ToJSon();
        g.Reset(1, 1, defaultCost: 1.0f);
        g.FromJSon(json);
        if (g.ToJSon() == json)
        {
            Sys.Console.Write("Tested OK");
            Sys.Console.WriteLine();
            long preTime = Sys.DateTime.Now.Ticks;
            LibPF.Position[] path = g.GetPath(start, end, LibPF.MovementPatterns.Full);
            long posTime = Sys.DateTime.Now.Ticks;
            Sys.Console.WriteLine("Time: " + Sys.TimeSpan.FromTicks(posTime - preTime).ToString());
            Sys.Console.WriteLine();
            Sys.Console.WriteLine();
            Sys.Console.Write("_");
            for (int x = 0; x < Size; x++) { Sys.Console.Write("_" + ((x < 10) ? ("_" + x.ToString()) : x.ToString()) + "___"); }
            Sys.Console.WriteLine();
            Sys.ConsoleColor stdColor = Sys.Console.ForegroundColor;
            bool IsUsed = false;
            for (int y = 0; y < Size; y++)
            {
                Sys.Console.Write("|");
                for (int x = 0; x < Size; x++) { Sys.Console.Write("     |"); }
                Sys.Console.WriteLine();
                Sys.Console.Write("|");
                for (int x = 0; x < Size; x++)
                {
                    IsUsed = false;
                    Sys.Console.Write(" ");
                    foreach (LibPF.Position p in path) { if (p.X == x && p.Y == y) { IsUsed = true; break; } }
                    if (IsUsed)
                    {
                        Sys.Console.ForegroundColor = Sys.ConsoleColor.Green;
                        Sys.Console.Write("-U-");
                    }
                    else if (g.GetCellCost(x, y) == float.PositiveInfinity)
                    {
                        Sys.Console.ForegroundColor = Sys.ConsoleColor.Red;
                        Sys.Console.Write("xXx");
                    } else { Sys.Console.Write("   "); }
                    Sys.Console.ForegroundColor = stdColor;
                    Sys.Console.Write(" |");
                }
                Sys.Console.Write(y.ToString());
                Sys.Console.WriteLine();
                Sys.Console.Write("|");
                for (int x = 0; x < Size; x++) { Sys.Console.Write("_____|"); }
                Sys.Console.WriteLine();
            }
        } else { Sys.Console.Write("OOPPSS"); }
    }

    internal static int Main(string[] args)
    {
        global::Test.Run(10,                /* SIZE - 0 to 9 */
            new LibPF.Position(0, 0),       /* Start position (0 based, x-y) */
            new LibPF.Position(9, 9),       /* End position (0 based, x-y) */
            new LibPF.Position[]
            {
                new LibPF.Position(2, 1),   /* Blocked, x-y */
                new LibPF.Position(2, 2),   /* Blocked, x-y */
                new LibPF.Position(2, 3),   /* Blocked, x-y */
                new LibPF.Position(4, 2),   /* Blocked, x-y */
                new LibPF.Position(4, 5),   /* Blocked, x-y */
                new LibPF.Position(4, 6),   /* Blocked, x-y */
                new LibPF.Position(4, 7),   /* Blocked, x-y */
                new LibPF.Position(4, 8),   /* Blocked, x-y */
                new LibPF.Position(4, 9),   /* Blocked, x-y */
                new LibPF.Position(5, 2),   /* Blocked, x-y */
                new LibPF.Position(5, 5),   /* Blocked, x-y */
                new LibPF.Position(6, 5),   /* Blocked, x-y */
                new LibPF.Position(6, 7),   /* Blocked, x-y */
                new LibPF.Position(7, 7),   /* Blocked, x-y */
                new LibPF.Position(8, 7),   /* Blocked, x-y */
                new LibPF.Position(9, 7)    /* Blocked, x-y */
            });
        Sys.Console.ReadKey();
        return 0;
    }
}
#endif
