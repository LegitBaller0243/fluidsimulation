using UnityEngine;

public partial class SPHSimulation
{
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
}
