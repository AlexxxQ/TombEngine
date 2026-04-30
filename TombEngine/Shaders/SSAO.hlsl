#include "./Math.hlsli"
#include "./VertexInput.hlsli"
#include "./CBCamera.hlsli"
#include "./CBPostProcess.hlsli"

#define SIGMA 3.0
#define BSIGMA 0.3
#define MSIZE 5

struct PixelShaderInput
{
    float4 Position : SV_POSITION;
    float2 UV : TEXCOORD0;
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

float ReconstructViewDepth(float depth)
{
    float4 projectedPosition = float4(0.0f, 0.0f, depth, 1.0f);
    float4 position = mul(projectedPosition, InverseProjection);

    return position.z / position.w;
}

float PS(PixelShaderInput input) : SV_Target
{
    float2 noiseScale = ViewportSize / 4.0f;

    float3 position = ReconstructPositionFromDepth(input.UV);
    float viewDistance = length(position);
    float3 encodedNormal = NormalsTexture.Sample(NormalsSampler, input.UV).xyz;

    if (viewDistance >= 40960.0f || dot(encodedNormal, encodedNormal) < 0.00000001f)
        return 1.0f;

    float3 normal = DecodeNormal(encodedNormal);
    float3 randomVec = NoiseTexture.Sample(NoiseSampler, input.UV * noiseScale).xyz;

    float3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
    float3 bitangent = SafeNormalize(cross(normal, tangent));

    static const int KERNEL_SIZE = 48;
    static const float RADIUS = 64.0f;
    static const float NEAR_BIAS = 2.0f;
    static const float FAR_BIAS = 1.0f;
    static const float FAR_BIAS_DISTANCE = 3072.0f; // 3 sectors.
    static const float FAR_BIAS_BLEND_RANGE = 1024.0f; // 1 sector.
    static const float THICKNESS = 1.0f;
    static const float INV_RADIUS = 1.0f / RADIUS;
    static const float INV_THICKNESS = 1.0f / THICKNESS;
    float biasBlend = smoothstep(
        FAR_BIAS_DISTANCE - FAR_BIAS_BLEND_RANGE * 0.5f,
        FAR_BIAS_DISTANCE + FAR_BIAS_BLEND_RANGE * 0.5f,
        viewDistance);
    float bias = lerp(NEAR_BIAS, FAR_BIAS, biasBlend);

    float occlusion = 0.0f;

    [unroll]
    for (int i = 0; i < KERNEL_SIZE; ++i)
    {
        float3 kernelSample = SSAOKernel[i].xyz;
        float3 samplePos = tangent * kernelSample.x + bitangent * kernelSample.y + normal * kernelSample.z;
        samplePos = position + samplePos * RADIUS;

        float4 offset = float4(samplePos, 1.0);
        offset = mul(offset, Projection);
        offset.xyz /= offset.w;
        offset.xyz = offset.xyz * 0.5f + 0.5f;
        offset.y = 1.0f - offset.y;

        float rawDepth = DepthTexture.Sample(DepthSampler, offset.xy).x;
        float sampleDepth = ReconstructViewDepth(rawDepth);
        float depthDelta = abs(position.z - sampleDepth);
        float rangeCheck = saturate(2.0f - depthDelta * INV_RADIUS);
        rangeCheck *= rangeCheck;

        float visibility = saturate((sampleDepth - samplePos.z - bias) * INV_THICKNESS);
        occlusion += visibility * rangeCheck;
    }

    occlusion = 1.0 - (occlusion / float(KERNEL_SIZE));

    const float darkenStartDistance = 1024.0f;
    const float darkenEndDistance = 4096.0f;
    float darkenFactor = saturate((viewDistance - darkenStartDistance) / (darkenEndDistance - darkenStartDistance));
    darkenFactor = smoothstep(0.0f, 1.0f, darkenFactor);
    float distanceExponent = lerp(1.0f, 1.35f, darkenFactor);
    occlusion = pow(saturate(occlusion), distanceExponent);

    return occlusion;
}

float normpdf(float x, float sigma)
{
    return 0.39894 * exp(-0.5 * x * x / (sigma * sigma)) / sigma;
}

static const float BLUR_KERNEL[MSIZE] =
{
    0.10650698f,
    0.14036673f,
    0.16579524f,
    0.14036673f,
    0.10650698f
};

float PSBlur(PixelShaderInput input) : SV_Target
{
    float rawDepth = DepthTexture.Sample(DepthSampler, input.UV).x;
    float viewDepth = abs(ReconstructViewDepth(rawDepth));
    float3 encodedBaseNormal = NormalsTexture.Sample(NormalsSampler, input.UV).xyz;
    float3 baseNormal = DecodeNormal(encodedBaseNormal);
    const float blurStartDepth = 1024.0f;
    const float blurEndDepth = 4096.0f;
    float depthFactor = saturate((viewDepth - blurStartDepth) / (blurEndDepth - blurStartDepth));
    depthFactor = smoothstep(0.0f, 1.0f, depthFactor);

    float blurScale = lerp(1.0f, 2.5f, depthFactor);
    float2 texelOffset = TexelSize * BlurDirection * blurScale;
    float result = 0.0f;

    const int kernelSize = (MSIZE - 1) / 2;
    float bZ = 0.0;

    float color;
    float baseColor = SSAOTexture.Sample(SSAOSampler, input.UV).x;
    float gfactor;
    float bfactor;
    float bZnorm = 1.0 / normpdf(0.0, BSIGMA);
    float depthTolerance = lerp(24.0f, 96.0f, depthFactor);

    for (int i = -kernelSize; i <= kernelSize; i++)
    {
        float2 offset = texelOffset * i;
        color = SSAOTexture.Sample(SSAOSampler, input.UV + offset).x;

        float neighborRawDepth = DepthTexture.Sample(DepthSampler, input.UV + offset).x;
        float neighborViewDepth = abs(ReconstructViewDepth(neighborRawDepth));
        float3 encodedNeighborNormal = NormalsTexture.Sample(NormalsSampler, input.UV + offset).xyz;
        float3 neighborNormal = DecodeNormal(encodedNeighborNormal);
        float depthWeight = exp(-abs(neighborViewDepth - viewDepth) / depthTolerance);
        float normalWeight = pow(saturate(dot(baseNormal, neighborNormal)), 32.0f);

        gfactor = BLUR_KERNEL[kernelSize + i];
        bfactor = normpdf(color - baseColor, BSIGMA) * bZnorm * gfactor * depthWeight * normalWeight;
        bZ += bfactor;

        result += bfactor * color;
    }

    return (result / bZ);
}
