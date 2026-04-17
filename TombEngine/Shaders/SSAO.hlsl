#include "./Math.hlsli"
#include "./VertexInput.hlsli"
#include "./CBCamera.hlsli"
#include "./CBPostProcess.hlsli"

#define SIGMA 4.0
#define BSIGMA 0.6
#define MSIZE 5
#define AO_INTENSITY 1.02
#define AO_RADIUS 88.0

struct PixelShaderInput
{
    float4 Position: SV_POSITION;
    float2 UV: TEXCOORD0;
};

Texture2D DepthTexture : register(t0);
SamplerState DepthSampler : register(s0);

Texture2D NormalsTexture : register(t1);
SamplerState NormalsSampler : register(s1);

Texture2D NoiseTexture : register(t2);
SamplerState NoiseSampler : register(s2);

Texture2D SSAOTexture : register(t9);
SamplerState SSAOSampler : register(s9);

float3 DecodeNormal(float3 n)
{
    return (n * 2.0f - 1.0f);
}

float3 ReconstructPositionFromDepth(float2 uv)
{
    float x = uv.x * 2.0f - 1.0f;
    float y = (1.0f - uv.y) * 2.0f - 1.0f;
    float z = DepthTexture.Sample(DepthSampler, uv).x;

    float4 projectedPosition = float4(x, y, z, 1.0f);
    float4 position = mul(projectedPosition, InverseProjection);

    return position.xyz / position.w;
}

float PS(PixelShaderInput input) : SV_Target
{
    float2 noiseScale = ViewportSize / 4.0f;

    float3 position = ReconstructPositionFromDepth(input.UV);
    float3 encodedNormal = NormalsTexture.Sample(NormalsSampler, input.UV).xyz;

    float viewDistance = length(position);
    float farMask = step(49152.0f, viewDistance); // 1 if too far
    float noNormalMask = step(length(encodedNormal), 0.0001f); // 1 if normal is too small
    float earlyExit = saturate(farMask + noNormalMask); // 0 if both are fine
   
    if (earlyExit > 0.0f)
        return float4(1.0f, 1.0f, 1.0f, 1.0f);

    float3 normal = SafeNormalize(DecodeNormal(encodedNormal));
    float3 randomVec = SafeNormalize(NoiseTexture.Sample(NoiseSampler, input.UV * noiseScale).xyz);
    float3 tangent = randomVec - normal * dot(randomVec, normal);
    if (dot(tangent, tangent) < 1e-4f)
    {
        float3 fallbackAxis = (abs(normal.y) < 0.99f) ? float3(0.0f, 1.0f, 0.0f) : float3(1.0f, 0.0f, 0.0f);
        tangent = cross(fallbackAxis, normal);
    }
    tangent = SafeNormalize(tangent);
    float3 bitangent = SafeNormalize(cross(normal, tangent));
    float3x3 TBN = float3x3(tangent, bitangent, normal);

    float occlusion = 0.0f;
    int kernelSize = 48;
    float radius = AO_RADIUS;
    float bias = 2.2f;
    float thickness = 20.0f;

    for (int i = 0; i < kernelSize; ++i)
    {
        float3 samplePos = mul(SSAOKernel[i], TBN);
        samplePos = position + samplePos * radius;

        float4 offset = float4(samplePos, 1.0);
        offset = mul(offset, Projection); 
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5f + 0.5f; 
        offset.y = 1.0f - offset.y;
        float sampleDepth = ReconstructPositionFromDepth(offset.xy).z;
        float depthDelta = abs(position.z - sampleDepth);
        float rangeCheck = smoothstep(0.0f, 1.0f, radius / max(depthDelta, 0.001f));
        float depthDiff = sampleDepth - samplePos.z;
        float occNear = smoothstep(bias, bias + 6.0f, depthDiff);
        float occFarReject = 1.0f - smoothstep(thickness, thickness + 8.0f, depthDiff);
        occlusion += occNear * occFarReject * rangeCheck;
    }

    occlusion = 1.0 - (occlusion / kernelSize);
    occlusion = lerp(1.0f, occlusion, AO_INTENSITY);
    float distanceFade = 1.0f - smoothstep(24576.0f, 40960.0f, viewDistance);
    float3 viewDir = SafeNormalize(-position);
    float silhouetteFade = smoothstep(0.0f, 0.65f, saturate(dot(normal, viewDir)));
    occlusion = lerp(1.0f, occlusion, distanceFade * silhouetteFade);

    return occlusion;
}

float normpdf(float x, float sigma)
{
    return 0.39894 * exp(-0.5 * x * x / (sigma * sigma)) / sigma;
}

float PSBlur(PixelShaderInput input) : SV_Target
{
    float2 texelSize = TexelSize;
    float result = 0.0f;

    const int kernelSize = (MSIZE - 1) / 2;
    float kernel[MSIZE];
    float bZ = 0.0;

    // Create the 1-D kernel
    for (int j = 0; j <= kernelSize; j++)
    {
        kernel[kernelSize + j] = kernel[kernelSize - j] = normpdf(float(j), SIGMA);
    }

    float color;
    float baseColor = SSAOTexture.Sample(SSAOSampler, input.UV).x;
    float baseDepth = DepthTexture.Sample(DepthSampler, input.UV).x;
    float3 baseNormal = SafeNormalize(DecodeNormal(NormalsTexture.Sample(NormalsSampler, input.UV).xyz));
    float gfactor;
    float bfactor;
    float depthWeight;
    float normalWeight;
    float bZnorm = 1.0 / normpdf(0.0, BSIGMA);

    // Read out the texels
    for (int i = -kernelSize; i <= kernelSize; i++)
    {
        for (int j = -kernelSize; j <= kernelSize; j++)
        {
            // Color at pixel in the neighborhood
            float2 offset = float2(i, j) * texelSize;
            color = SSAOTexture.Sample(SSAOSampler, input.UV + offset).x;
            float sampleDepth = DepthTexture.Sample(DepthSampler, input.UV + offset).x;
            float3 sampleNormal = SafeNormalize(DecodeNormal(NormalsTexture.Sample(NormalsSampler, input.UV + offset).xyz));

            // Compute both the gaussian smoothed and bilateral
            gfactor = kernel[kernelSize + j] * kernel[kernelSize + i];
            depthWeight = exp(-abs(sampleDepth - baseDepth) * 120.0f);
            normalWeight = pow(saturate(dot(baseNormal, sampleNormal)), 8.0f);
            bfactor = normpdf(color - baseColor, BSIGMA) * bZnorm * gfactor * depthWeight * normalWeight;
            bZ += bfactor;

            result += bfactor * color;
        }
    }

    return (result / bZ);
}
