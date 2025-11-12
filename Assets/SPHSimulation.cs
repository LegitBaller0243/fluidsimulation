using UnityEngine;
using Unity.Mathematics;
using System;

[ExecuteAlways] // editor preview grid via Gizmos
public class SPHSimulation : MonoBehaviour
{
    public Vector2 boundsSize;
    public int numParticles = 400;

    [Header("Rendering (Gizmos)")]
    public bool renderWithGizmos = true;
    public float gizmoRadius = 0.025f;
    public Gradient speedGradient;   
    [Range(0.1f, 0.95f)]
    public float fillFraction = 0.65f;      

    [Header("Physics")]
    public float collisionDamping;
    public float gravity;
    public float viscosityStrength;
    public float targetDensity = 3.0f;
    public float pressureMultiplier;
    public float nearPressureMultiplier;
    public float smoothingRadius;
    
    [Header("Interaction")]
    public float interactionRadius;
    public float interactionStrength;    

    public class Entry {
        public int particleIndex;
        public uint cellKey;
    }
    Vector2[] positions;
    Vector2[] predictedPositions;
    Vector2[] velocities;
    float[] densities;
    float[] nearDensities;
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
        predictedPositions = new Vector2[numParticles];
        velocities = new Vector2[numParticles];
        densities  = new float[numParticles];
        nearDensities = new float[numParticles];
        spatialLookup = new Entry[numParticles];
        startIndices = new int[numParticles];

        CreateParticles();
        float dx = Mathf.Sqrt((boundsSize.x * boundsSize.y) / numParticles);

        particleMass    = targetDensity * dx * dx;
        smoothingRadius = 1.4f * dx;
    }

    void Update() {
        if (!Application.isPlaying) return;

        float dt = Time.deltaTime;

        int substeps = 3;
        float subDt = dt / substeps;

        for (int s = 0; s < substeps; ++s) SimulationStep(subDt);
    }
    
    Vector2 GetMouseWorldPosition() {
        // Convert mouse screen position to world position
        Camera cam = Camera.main;
        if (cam == null) return Vector2.zero;
        
        Vector3 mouseScreenPos = Input.mousePosition;
        mouseScreenPos.z = cam.nearClipPlane + 1f; // Set depth for 2D
        
        Vector3 mouseWorldPos3D = cam.ScreenToWorldPoint(mouseScreenPos);
        Vector2 mouseWorldPos = new Vector2(mouseWorldPos3D.x, mouseWorldPos3D.y);
        
        // Account for VisualScale to convert from visual space to simulation space
        float visualScale = SimSettings.VisualScale;
        mouseWorldPos /= visualScale;
        
        return mouseWorldPos;
    }

    void ApplyInteractionForce(Vector2 interactionPosition, float strength, float dt)
    {
        if (positions == null || velocities == null) return;

        // Loop through all particles and apply interaction force
        for (int i = 0; i < positions.Length; i++)
        {
            Vector2 force = InteractionForce(interactionPosition, interactionRadius, strength, i);
            // Apply force to velocity
            velocities[i] += force * dt;
        }
    }

#if UNITY_EDITOR
    void OnDrawGizmos() {
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
        // Calculate speed range for color mapping
        float minSpeed = float.MaxValue;
        float maxSpeed = float.MinValue;
        for (int i = 0; i < positions.Length; i++)
        {
            float speed = velocities[i].magnitude;
            if (speed < minSpeed) minSpeed = speed;
            if (speed > maxSpeed) maxSpeed = speed;
        }

        
        for (int i = 0; i < positions.Length; i++)
        {
            float speed = velocities[i].magnitude;

            // Handle invalid speed values
            if (float.IsNaN(speed) || float.IsInfinity(speed))
                speed = 0f;

            // Normalize speed to [0, 1]
            float normalizedSpeed = 0f;
            if (maxSpeed > minSpeed && !float.IsNaN(minSpeed) && !float.IsNaN(maxSpeed))
            {
                normalizedSpeed = (speed - minSpeed) / (maxSpeed - minSpeed);

                // Clamp to [0, 1] in case of rounding or division issues
                normalizedSpeed = Mathf.Clamp01(normalizedSpeed);
            }

            // Fallback if range invalid
            if (float.IsNaN(normalizedSpeed) || float.IsInfinity(normalizedSpeed))
                normalizedSpeed = 0f;
            Gizmos.color = speedGradient.Evaluate(normalizedSpeed);
            Gizmos.DrawSphere((Vector3)positions[i] * s, gizmoRadius * s);
        }
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

    float CreateParticles() {
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
                nearDensities[k] = 0f;
                k++;
            }
        }
        return dx;
    }

    // ----------------- Simulation -----------------
    void SimulationStep(float dt) {

        // Interaction force (mouse input)
        if (positions != null && positions.Length > 0) {
            // Left click: push away (negative strength)
            if (Input.GetMouseButton(0)) {
                Vector2 mouseWorldPos = GetMouseWorldPosition();
                ApplyInteractionForce(mouseWorldPos, -interactionStrength, dt);
            }
            
            // Right click: pull in (positive strength)
            if (Input.GetMouseButton(1)) {
                Vector2 mouseWorldPos = GetMouseWorldPosition();
                ApplyInteractionForce(mouseWorldPos, interactionStrength, dt);
            }
        }

        // Gravity + density pass
        System.Threading.Tasks.Parallel.For(0, numParticles, i => {
            velocities[i] += Vector2.down * gravity * dt;
            predictedPositions[i] = positions[i] + velocities[i] * dt;
        });

        UpdateSpatialLookup(predictedPositions, smoothingRadius);

        System.Threading.Tasks.Parallel.For(0, numParticles, i =>
        {
            densities[i] = CalculateDensity(predictedPositions[i]);
        });
        System.Threading.Tasks.Parallel.For(0, numParticles, i =>
        {
            nearDensities[i] = CalculateNearDensity(predictedPositions[i]);
        });

        // Pressure force pass
        System.Threading.Tasks.Parallel.For(0, numParticles, i =>
        {
            Vector2 pressureForce = CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / densities[i];
            velocities[i] += pressureAcceleration * dt;
        });

        // Viscosity Force Pass
        System.Threading.Tasks.Parallel.For(0, numParticles, i =>
        {
            Vector2 viscosityForce = CalculateViscosityForce(i);
            Vector2 viscosityAcceleration = viscosityForce / densities[i];
            velocities[i] += viscosityAcceleration * dt;
        });

        // Integrate + collisions
        System.Threading.Tasks.Parallel.For(0, numParticles, i => {
            positions[i] += velocities[i] * dt;
            ResolveCollisions(ref positions[i], ref velocities[i]);
        });
    }

    float SmoothingKernel(float radius, float dist)
    {
        if (dist >= radius) return 0f;
        float volume = Mathf.PI * Mathf.Pow(radius, 4) / 6f;
        return (radius - dist) * (radius - dist) / volume;
    }
    float NearDensitySmoothingKernel(float radius, float dist)
    {
        if (dist >= radius) return 0f;
        float volume = Mathf.PI * Mathf.Pow(radius, 5) / 6f;
        float value = radius - dist;
        return value * value * value / volume;
    }

    float NearDensitySmoothingKernelDerivative(float radius, float dist)
    {
        if (dist >= radius) return 0f;
        return -18f * (radius - dist) * (radius - dist) / (Mathf.PI * Mathf.Pow(radius, 5));
    }


    float SmoothingKernelDerivative(float radius, float dist)
    {
        if (dist >= radius) return 0f;
        return -12f * (radius - dist) / (Mathf.PI * Mathf.Pow(radius, 4));
    }
    
    float ViscositySmoothingKernel(float radius, float dist)
    // Poly6
    {
        if (dist >= radius) return 0f;
        float volume = Mathf.PI * Mathf.Pow(radius, 8) / 4f;
        float value = radius * radius - dist * dist;
        return value * value * value / volume;
    }

    float CalculateDensity(Vector2 pos_i)
    {
        float density = 0f;
        RadiusStabilizer(pos_i, smoothingRadius, (j) =>
        {
            float dist = (predictedPositions[j] - pos_i).magnitude;
            float w = SmoothingKernel(smoothingRadius, dist);
            density += particleMass * w;
        });
        return density;
    }
    
    float CalculateNearDensity(Vector2 pos_i)
    {
        float nearDensity = 0f;
        RadiusStabilizer(pos_i, smoothingRadius, (j) =>
        {
            float dist = (predictedPositions[j] - pos_i).magnitude;
            float w = NearDensitySmoothingKernel(smoothingRadius, dist);
            nearDensity += particleMass * w;
        });
        return nearDensity;
    }

    Vector2 CalculatePressureForce(int i)
    {
        Vector2 pressureForce = Vector2.zero;
        Vector2 pos_i = predictedPositions[i];
        RadiusStabilizer(pos_i, smoothingRadius, (j) =>
        {
            if (j == i) return;

            Vector2 offset = predictedPositions[j] - predictedPositions[i];
            float dist = offset.magnitude;
            if (dist <= 0f) return;

            Vector2 direction = offset / dist;
            // Standard pressure component
            float slope = SmoothingKernelDerivative(smoothingRadius, dist);
            float density = densities[j];
            float sharedPressure = CalculateSharedPressure(density, densities[i]);
            pressureForce += sharedPressure * particleMass / density * slope * direction;

            // Near-pressure component
            float nearSlope = NearDensitySmoothingKernelDerivative(smoothingRadius, dist);
            float nearDensity = nearDensities[j];
            float sharedNearPressure = CalculateNearSharedPressure(nearDensity, nearDensities[i]);
            pressureForce += sharedNearPressure * particleMass / density * nearSlope * direction;
        });
        return pressureForce;
    }
    Vector2 CalculateViscosityForce(int i)
    {
        Vector2 viscosityForce = Vector2.zero;
        Vector2 position = positions[i];
        RadiusStabilizer(position, smoothingRadius, (j) =>
        {
            float dist = (position - positions[j]).magnitude;
            float influence = ViscositySmoothingKernel(smoothingRadius, dist);
            viscosityForce += (velocities[j] - velocities[i]) * influence;
        });
        return viscosityForce * viscosityStrength;
    }

    float CalculateSharedPressure(float densityA, float densityB)
    {
        float pressureA = ConvertDensityToPressure(densityA);
        float pressureB = ConvertDensityToPressure(densityB);
        return 0.5f * (pressureA + pressureB);
    }
    float CalculateNearSharedPressure(float nearDensityA, float nearDensityB)
    {
        float nearPressureA = convertNearDensityToPressure(nearDensityA);
        float nearPressureB = convertNearDensityToPressure(nearDensityB);
        return 0.5f * (nearPressureA + nearPressureB);
    }

    float ConvertDensityToPressure(float density) {
        float densityError = density - targetDensity;
        float pressure = densityError * pressureMultiplier;
        return pressure;
    }
    float convertNearDensityToPressure(float nearDensity) {
        return nearDensity * nearPressureMultiplier;
    }

    Vector2 InteractionForce(Vector2 inputPos, float radius, float strength, int particleIndex) {
        Vector2 interactionForce = Vector2.zero;
        Vector2 offset = inputPos - positions[particleIndex];
        float sqrDst = Vector2.Dot(offset, offset);

        if (sqrDst < radius * radius) {
            float dst = Mathf.Sqrt(sqrDst);
            Vector2 dirToInputPoint = dst <= float.Epsilon ? Vector2.zero : offset / dst;
            float centreT = 1f - dst/radius;
            interactionForce += (dirToInputPoint * strength - velocities[particleIndex]) * centreT;
        }
        return interactionForce;
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
