using System.Collections.Generic;
using System.Linq;
using GraphLibrary;

namespace Tests
{
    public static class PathExtensions
    {
        public static string GetRoute(this Road[] path)
        {
            return string.Join(",", path.Select(x => x.Name).Distinct());
        }

        public static string GetWaypoints(this Road[] path)
        {
            var toJoin = path
                .Select(x => x.Tail.Name)
                .Concat(new[] { path.Last().Head.Name });

            return string.Join(",", toJoin);
        }
    }

    public static class Build
    {
        public static Dictionary<string, City> Cities { get; } = new[]
        {
            new City("Milton Keynes"),
            new City("London"),
            new City("Birmingham"),
            new City("Bristol"),
            new City("Exeter")
        }.ToDictionary(x => x.Name);

        public static Graph<Road, City> Roads { get; } = new Graph<Road, City>(
            new[]
            {
                new Road("M1", Cities["London"], Cities["Milton Keynes"], 80),
                new Road("M1", Cities["Milton Keynes"], Cities["Birmingham"], 80),
                new Road("M40", Cities["London"], Cities["Birmingham"], 75),
                new Road("M5", Cities["Birmingham"], Cities["Bristol"], 60),
                new Road("M4", Cities["London"], Cities["Bristol"], 130),
                new Road("M5", Cities["Bristol"], Cities["Exeter"], 60),
                new Road("A303", Cities["London"], Cities["Exeter"], 180)
            });
    }

    public class Road : IEdge<City>
    {
        public Road(string name, City from, City to, int distance)
        {
            Name = name;
            Tail = from;
            Head = to;
            Weight = distance;
        }

        public string Name { get; }
        public City Tail { get; }
        public City Head { get; }
        public int Weight { get; }
    }

    public class City
    {
        public string Name { get; }

        public City(string name)
        {
            Name = name;
        }
    }
}
