using UnityEngine;
using Unity.Mathematics;
using System;

[ExecuteAlways] // editor preview grid via Gizmos
public class SPHSimulation : MonoBehaviour
{
    public Vector2 boundsSize = new Vector2(5, 3);
    public int numParticles = 400;

    [Header("Rendering (Gizmos)")]
    public bool renderWithGizmos = true;
    public float gizmoRadius = 0.025f;      
    [Range(0.1f, 0.95f)]
    public float fillFraction = 0.65f;      

    [Header("Physics")]
    public float collisionDamping = 0.5f;
    public float gravity = 0f;
    public float targetDensity = 3.0f;
    public float pressureMultiplier = 0.5f;
    public float smoothingRadius = 0.2f;    

    public class Entry {
        public int particleIndex;
        public uint cellKey;
    }
    Vector2[] positions;
    Vector2[] velocities;
    float[] densities;
    Entry[] spatialLookup;
    int[] startIndices;

    float particleMass; 

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



    // ----------------- Lifecycle -----------------
    void Start()
    {
        if (!Application.isPlaying) return;

        positions  = new Vector2[numParticles];
        velocities = new Vector2[numParticles];
        densities  = new float[numParticles];
        spatialLookup = new Entry[numParticles];
        startIndices = new int[numParticles];

        float dx = CreateParticles();

        particleMass    = targetDensity * dx * dx;
        smoothingRadius = 1.45f * dx;
    }

    void Update()
    {
        if (!Application.isPlaying) return;

        float dt = Time.deltaTime;
        int substeps = 5;
        float subDt = dt / substeps;

        for (int s = 0; s < substeps; ++s)
            SimulationStep(subDt);
    }

#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        if (!renderWithGizmos) return;

        Gizmos.color = Color.gray;
        Vector2 h = boundsSize * 0.5f;
        float s = SimSettings.VisualScale;
        Vector3 a = new Vector3(-h.x, -h.y, 0f) * s;
        Vector3 b = new Vector3(-h.x,  h.y, 0f) * s;
        Vector3 c = new Vector3( h.x,  h.y, 0f) * s;
        Vector3 d = new Vector3( h.x, -h.y, 0f) * s;
        Gizmos.DrawLine(a, b); Gizmos.DrawLine(b, c); Gizmos.DrawLine(c, d); Gizmos.DrawLine(d, a);

        Gizmos.color = Color.white;

        if (!Application.isPlaying || positions == null || positions.Length != numParticles)
        {
            ComputeCenteredGrid(out int cols, out int rows, out float dx, out Vector2 origin);

            int k = 0;
            for (int r = 0; r < rows && k < numParticles; r++)
            {
                for (int cidx = 0; cidx < cols && k < numParticles; cidx++, k++)
                {
                    float x = origin.x + (cidx + 0.5f) * dx;
                    float y = origin.y + (r    + 0.5f) * dx;
                    Gizmos.DrawSphere(new Vector3(x, y, 0f) * s, gizmoRadius * s);
                }
            }
            return;
        }

        // Playing (or arrays ready): draw actual simulated positions
        for (int i = 0; i < positions.Length; i++)
            Gizmos.DrawSphere((Vector3)positions[i] * s, gizmoRadius * s);
    }
#endif

    // ----------------- Initialization -----------------

    void ComputeCenteredGrid(out int cols, out int rows, out float dx, out Vector2 origin) {
        Vector2 region = boundsSize * Mathf.Clamp01(fillFraction);
        Vector2 halfR  = region * 0.5f;

        cols = Mathf.CeilToInt(Mathf.Sqrt(numParticles * (region.x / Mathf.Max(1e-6f, region.y))));
        rows = Mathf.CeilToInt((float)numParticles / cols);
        dx   = Mathf.Min(region.x / cols, region.y / rows);

        float usedW = cols * dx, usedH = rows * dx;
        origin = new Vector2(-halfR.x + 0.5f * (region.x - usedW),
                            -halfR.y + 0.5f * (region.y - usedH));
    }

    float CreateParticles()
    {
        ComputeCenteredGrid(out int cols, out int rows, out float dx, out Vector2 origin);


        int k = 0;
        for (int r = 0; r < rows && k < numParticles; r++)
        {
            for (int c = 0; c < cols && k < numParticles; c++)
            {
                float x = origin.x + (c + 0.5f) * dx;
                float y = origin.y + (r + 0.5f) * dx;
                positions[k]  = new Vector2(x, y);
                velocities[k] = Vector2.zero;
                densities[k]  = 0f;
                k++;
            }
        }
        return dx;
    }

    // ----------------- Simulation -----------------
    void SimulationStep(float dt)
    {
        // Gravity + density pass
        System.Threading.Tasks.Parallel.For(0, numParticles, i => {
            velocities[i] += Vector2.down * gravity * dt;
            densities[i] = CalculateDensity(positions[i]);
        });

        // Pressure force pass
        System.Threading.Tasks.Parallel.For(0, numParticles, i => {
            Vector2 pressureForce = CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / densities[i]; 
            velocities[i] += pressureAcceleration * dt;
        });

        // Integrate + collisions
        System.Threading.Tasks.Parallel.For(0, numParticles, i => {
            positions[i] += velocities[i] * dt;
            ResolveCollisions(ref positions[i], ref velocities[i]);
        });
    }

    float SmoothingKernel(float radius, float dist) {
        if (dist >= radius) return 0f;
        float volume = Mathf.PI * Mathf.Pow(radius, 4) / 6f;
        return (radius - dist) * (radius - dist) / volume;
    }

    float SmoothingKernelDerivative(float radius, float dist) {
        if (dist >= radius) return 0f;
        return -12f * (radius - dist) / (Mathf.PI * Mathf.Pow(radius, 4));
    }

    float CalculateDensity(Vector2 sample) {
        float density = 0f;
        for (int i = 0; i < numParticles; i++) {
            float dist = (positions[i] - sample).magnitude;
            float influence = SmoothingKernel(smoothingRadius, dist);
            density += particleMass * influence;
        }
        return density;
    }

    Vector2 CalculatePressureForce(int particleIndex) {
        Vector2 pressureForce = Vector2.zero;
        for (int j = 0; j < numParticles; j++) {
            if (j == particleIndex) continue;

            Vector2 offset = positions[j] - positions[particleIndex];
            float dst = offset.magnitude;
            if (dst <= 0f) continue;

            Vector2 direction = dst == 0? GetRandomDir() : offset / dst;
            float slope = SmoothingKernelDerivative(smoothingRadius, dst);
            float density = densities[j]; 
            float sharedPressure = CalculateSharedPressure(density, densities[particleIndex]);

            pressureForce += sharedPressure * particleMass / density * slope * direction;
        }
        return pressureForce;
    }

    float CalculateSharedPressure(float densityA, float densityB) {
        float pressureA = ConvertDensityToPressure(densityA);
        float pressureB = ConvertDensityToPressure(densityB);
        return 0.5f * (pressureA + pressureB);
    }

    float ConvertDensityToPressure(float density) {
        return (density - targetDensity) * pressureMultiplier;
    }

    void ResolveCollisions(ref Vector2 pos, ref Vector2 vel) {
        if (boundsSize.x == 0 || boundsSize.y == 0) return;
        Vector2 half = boundsSize * 0.5f;
        float buffer = 0.01f;

        if (Mathf.Abs(pos.x) > half.x) {
            pos.x = (half.x - buffer) * Mathf.Sign(pos.x);
            vel.x *= -collisionDamping;
        }
        if (Mathf.Abs(pos.y) > half.y) {
            pos.y = (half.y - buffer) * Mathf.Sign(pos.y);
            vel.y *= -collisionDamping;
        }
    }
    Vector2 GetRandomDir() { return Vector2.zero; }
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
    public void RadiusStabilizer(Vector2 sample, float radius) {
        (int centerX, int centerY) = PositionToCellCoord(sample, radius);
        float sqrRadius = radius * radius;

        foreach(int2 offs in cellOffsets) {
            uint key = GetKeyFromHash(HashCell(centerX + offs.x, centerY + offs.y));
            int cellStart = startIndices[key];

            int i = cellStart;
            for (int i = cellStart; i < spatialLookup.Length; i++)  {
                if (spatialLookup[i].cellKey != key) break;
                int particleIndex = spatialLookup[i].particleIndex;
                float sqrDst = (positions[particleIndex] - sample).sqrMagnitude;

                if (sqrDst <= sqrRadius) {
                    //update a particles density and pressure Forces
                }
                i += 1;
            }
        }
    }
}
