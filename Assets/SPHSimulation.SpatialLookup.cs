using Unity.Mathematics;
using System;
using UnityEngine;

public partial class SPHSimulation
{
    public class Entry {
        public int particleIndex;
        public uint cellKey;
    }

    static readonly int2[] cellOffsets = new int2[]
    {
        new int2(-1, 1),
        new int2(0, 1),
        new int2(1, 1),
        new int2(-1, 0),
        new int2(0, 0),
        new int2(1, 0),
        new int2(-1, -1),
        new int2(0, -1),
        new int2(1, -1)
    };

    public void UpdateSpatialLookup(Vector2[] points, float radius) {
        System.Threading.Tasks.Parallel.For(0, points.Length, i => {
            (int cx, int cy) = PositionToCellCoord(points[i], radius);
            uint cellKeyHash = GetKeyFromHash(HashCell(cx, cy));
            spatialLookup[i] = new Entry {
                particleIndex = i,
                cellKey = cellKeyHash
            };
            startIndices[i] = int.MaxValue;
        });

        Array.Sort(spatialLookup, (a, b) => a.cellKey.CompareTo(b.cellKey));

        System.Threading.Tasks.Parallel.For(0, points.Length, i => {
            uint key = spatialLookup[i].cellKey;
            uint keyPrev = i == 0? uint.MaxValue : spatialLookup[i - 1].cellKey;

            if (key != keyPrev) {
                startIndices[key] = i;
            }
        });
    }
    public (int x, int y) PositionToCellCoord(Vector2 point, float radius) {
        int cellX = (int) (point.x / radius);
        int cellY = (int) (point.y / radius);
        return (cellX, cellY);
    }
    public uint HashCell(int cellX, int cellY) {
        uint a = (uint) cellX * 15823;
        uint b = (uint) cellY * 9737333;
        return a + b;
    }
    public uint GetKeyFromHash(uint hash) {
        return hash % (uint) spatialLookup.Length;
    }
    public void RadiusStabilizer(Vector2 sample, float radius, Action<int> handleNeighbor) {
        (int centerX, int centerY) = PositionToCellCoord(sample, radius);
        float sqrRadius = radius * radius;

        foreach(int2 offs in cellOffsets) {
            uint key = GetKeyFromHash(HashCell(centerX + offs.x, centerY + offs.y));
            int cellStart = startIndices[key];

            for (int i = cellStart; i < spatialLookup.Length; i++)  {
                if (spatialLookup[i].cellKey != key) break;
                int particleIndex = spatialLookup[i].particleIndex;
                float sqrDst = (positions[particleIndex] - sample).sqrMagnitude;

                if (sqrDst <= sqrRadius) {
                    handleNeighbor(particleIndex);
                }
            }
        }
    }
}
