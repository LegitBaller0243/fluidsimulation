using UnityEngine;

[ExecuteAlways] // editor preview grid via Gizmos
public partial class SPHSimulation : MonoBehaviour
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

    Vector2[] positions;
    Vector2[] predictedPositions;
    Vector2[] velocities;
    float[] densities;
    float[] nearDensities;
    Entry[] spatialLookup;
    int[] startIndices;
    

    float particleMass; 

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
}
