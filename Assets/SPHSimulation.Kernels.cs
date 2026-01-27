using UnityEngine;

public partial class SPHSimulation
{
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
}
