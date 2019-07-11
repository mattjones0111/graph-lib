using GraphLibrary;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Tests
{
    [TestClass]
    public class KShortestRouteAlgorithmTests
    {
        [TestMethod]
        public void Algorithm_Finds_Ranked_Shortest_Routes()
        {
            var sut = new KShortestPathAlgorithm();

            var actual = sut.FindShortestPaths(
                Build.Roads,
                Build.Cities["London"],
                Build.Cities["Exeter"],
                10);

            Assert.AreEqual(
                4,
                actual.Length,
                "Expected four possible routes.");

            Assert.AreEqual(
                "A303",
                actual[0].GetRoute(),
                "Shortest route expected to be A303.");

            Assert.AreEqual(
                "M4,M5",
                actual[1].GetRoute(),
                "Second shortest route was expected to be M4-M5.");

            Assert.AreEqual(
                "M40,M5",
                actual[2].GetRoute(),
                "Third shortest route was expected to be M40-M5.");

            Assert.AreEqual(
                "M1,M5",
                actual[3].GetRoute(),
                "Fourth shortest route was expected to be M1-M5.");

            Assert.AreEqual(
                "London,Milton Keynes,Birmingham,Bristol,Exeter",
                actual[3].GetWaypoints(),
                "Fourth shortest route was expected to be London, " +
                "Milton Keynes, Birmingham, Bristol, Exeter.");
        }
    }
}