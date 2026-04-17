#include "./Math.hlsli"
#include "./VertexInput.hlsli"
#include "./CBCamera.hlsli"
#include "./CBPostProcess.hlsli"

#define SIGMA 3.25
#define BSIGMA 0.36
#define MSIZE 7

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

float2 ClampSceneUV(float2 uv)
{
    float2 halfTexel = InvViewSize * 0.5f;
    return clamp(uv, halfTexel, 1.0f - halfTexel);
}

float3 ReconstructPositionFromRawDepth(float2 uv, float rawDepth)
{
    uv = ClampSceneUV(uv);

    float x = uv.x * 2.0f - 1.0f;
    float y = (1.0f - uv.y) * 2.0f - 1.0f;

    float4 projectedPosition = float4(x, y, rawDepth, 1.0f);
    float4 position = mul(projectedPosition, InverseProjection);

    return position.xyz / position.w;
}

float3 ReconstructPositionFromDepth(float2 uv)
{
    uv = ClampSceneUV(uv);
    float z = DepthTexture.Sample(DepthSampler, uv).x;
    return ReconstructPositionFromRawDepth(uv, z);
}

void SampleHalfResSurface(float2 uv, out float3 position, out float3 encodedNormal)
{
    float2 offset = InvViewSize * 0.5f;

    float2 uv00 = ClampSceneUV(uv + float2(-offset.x, -offset.y));
    float2 uv10 = ClampSceneUV(uv + float2( offset.x, -offset.y));
    float2 uv01 = ClampSceneUV(uv + float2(-offset.x,  offset.y));
    float2 uv11 = ClampSceneUV(uv + float2( offset.x,  offset.y));

    float depth = DepthTexture.Sample(DepthSampler, uv00).x;
    float2 selectedUV = uv00;
    encodedNormal = NormalsTexture.Sample(NormalsSampler, uv00).xyz;

    float candidateDepth = DepthTexture.Sample(DepthSampler, uv10).x;
    if (candidateDepth < depth)
    {
        depth = candidateDepth;
        selectedUV = uv10;
        encodedNormal = NormalsTexture.Sample(NormalsSampler, uv10).xyz;
    }

    candidateDepth = DepthTexture.Sample(DepthSampler, uv01).x;
    if (candidateDepth < depth)
    {
        depth = candidateDepth;
        selectedUV = uv01;
        encodedNormal = NormalsTexture.Sample(NormalsSampler, uv01).xyz;
    }

    candidateDepth = DepthTexture.Sample(DepthSampler, uv11).x;
    if (candidateDepth < depth)
    {
        depth = candidateDepth;
        selectedUV = uv11;
        encodedNormal = NormalsTexture.Sample(NormalsSampler, uv11).xyz;
    }

    position = ReconstructPositionFromRawDepth(selectedUV, depth);
}

float PS(PixelShaderInput input) : SV_Target
{
    float2 noiseScale = float2(ViewportSize) / 4.0f;

    float3 position;
    float3 encodedNormal;
    SampleHalfResSurface(input.UV, position, encodedNormal);

    float farMask = step(40960.0f, length(position));
    float noNormalMask = step(length(encodedNormal), 0.0001f);
    float earlyExit = saturate(farMask + noNormalMask);

    if (earlyExit > 0.0f)
        return 1.0f;

    float3 normal = DecodeNormal(encodedNormal);
    float3 randomVec = NoiseTexture.Sample(NoiseSampler, input.UV * noiseScale).xyz;

    float3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    float3 bitangent = SafeNormalize(cross(normal, tangent));
    float3x3 TBN = float3x3(tangent, bitangent, normal);

    float occlusion = 0.0f;
    int kernelSize = 64;
    float radius = 64.0f;
    float bias = 4.0f;

    for (int i = 0; i < kernelSize; ++i)
    {
        float3 samplePos = mul(SSAOKernel[i].xyz, TBN);
        samplePos = position + samplePos * radius;

        float4 offset = float4(samplePos, 1.0f);
        offset = mul(offset, Projection);
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5f + 0.5f;
        offset.y = 1.0f - offset.y;

        float sampleDepth = ReconstructPositionFromDepth(offset.xy).z;
        float rangeCheck = smoothstep(0.0f, 1.0f, radius / abs(position.z - sampleDepth));

        occlusion += lerp(0.0f, rangeCheck, step(0.0f, sampleDepth - samplePos.z - bias));
    }

    occlusion = 1.0f - (occlusion / kernelSize);

    return occlusion;
}

float normpdf(float x, float sigma)
{
    return 0.39894 * exp(-0.5 * x * x / (sigma * sigma)) / sigma;
}

float ComputeBlurNormalSimilarity(float3 baseNormal, float3 sampleNormal)
{
    float similarity = saturate(dot(baseNormal, sampleNormal));
    return similarity * similarity;
}

float PSBlur(PixelShaderInput input) : SV_Target
{
    float2 texelSize = TexelSize;
    float result = 0.0f;

    const int kernelSize = (MSIZE - 1) / 2;
    float kernel[MSIZE];
    float bZ = 0.0f;

    for (int j = 0; j <= kernelSize; j++)
    {
        kernel[kernelSize + j] = kernel[kernelSize - j] = normpdf(float(j), SIGMA);
    }

    float color;
    float baseColor = SSAOTexture.Sample(SSAOSampler, input.UV).x;
    float gfactor;
    float bfactor;
    float bZnorm = 1.0f / normpdf(0.0f, BSIGMA);

    float normalAwareEnabled = step(0.5f, BlurSigma);
    float2 baseUV = ClampSceneUV(input.UV);
    float3 baseEncodedNormal = NormalsTexture.Sample(NormalsSampler, baseUV).xyz;
    float baseHasNormal = step(0.0001f, length(baseEncodedNormal));
    float3 baseNormal = DecodeNormal(baseEncodedNormal);
    float baseRawDepth = DepthTexture.Sample(DepthSampler, baseUV).x;
    float baseDepth = LinearizeDepth(baseRawDepth, NearPlane, FarPlane);

    for (int i = -kernelSize; i <= kernelSize; i++)
    {
        for (int j = -kernelSize; j <= kernelSize; j++)
        {
            float2 offset = float2(i, j) * texelSize;
            float2 sampleUV = input.UV + offset;
            color = SSAOTexture.Sample(SSAOSampler, sampleUV).x;

            gfactor = kernel[kernelSize + j] * kernel[kernelSize + i];
            bfactor = normpdf(color - baseColor, BSIGMA) * bZnorm * gfactor;

            if (normalAwareEnabled > 0.5f && baseHasNormal > 0.5f)
            {
                float2 sampleSceneUV = ClampSceneUV(sampleUV);
                float sampleRawDepth = DepthTexture.Sample(DepthSampler, sampleSceneUV).x;
                float sampleDepth = LinearizeDepth(sampleRawDepth, NearPlane, FarPlane);
                float depthWeight = exp(-abs(baseDepth - sampleDepth) * 55.0f);

                float3 sampleEncodedNormal = NormalsTexture.Sample(NormalsSampler, sampleSceneUV).xyz;
                float sampleHasNormal = step(0.0001f, length(sampleEncodedNormal));
                float3 sampleNormal = DecodeNormal(sampleEncodedNormal);
                float normalWeight = lerp(1.0f, ComputeBlurNormalSimilarity(baseNormal, sampleNormal), sampleHasNormal);

                // Softly bias toward edge preservation while keeping baseline blur behavior.
                float edgeWeight = lerp(1.0f, depthWeight * normalWeight, 0.30f);
                bfactor *= edgeWeight;
            }

            bZ += bfactor;

            result += bfactor * color;
        }
    }

    return (result / bZ);
}
