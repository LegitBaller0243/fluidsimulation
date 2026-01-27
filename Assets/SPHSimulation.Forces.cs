using UnityEngine;

public partial class SPHSimulation
{
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
        float nearPressureA = ConvertNearDensityToPressure(nearDensityA);
        float nearPressureB = ConvertNearDensityToPressure(nearDensityB);
        return 0.5f * (nearPressureA + nearPressureB);
    }

    float ConvertDensityToPressure(float density) {
        float densityError = density - targetDensity;
        float pressure = densityError * pressureMultiplier;
        return pressure;
    }
    float ConvertNearDensityToPressure(float nearDensity) {
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
}
