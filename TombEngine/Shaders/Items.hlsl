#include "./Math.hlsli"
#include "./CBCamera.hlsli"
#include "./CBItem.hlsli"
#include "./CBRoom.hlsli"
#include "./ShaderLight.hlsli"
#include "./VertexEffects.hlsli"
#include "./VertexInput.hlsli"
#include "./Blending.hlsli"
#include "./AnimatedTextures.hlsli"
#include "./Shadows.hlsli"
#include "./Materials.hlsli"

struct PixelShaderInput
{
	float4 Position: SV_POSITION;
	float3 WorldPosition: POSITION0;
	float3 Normal: NORMAL;
	float2 UV: TEXCOORD0;
	float4 Color: COLOR;
	float Sheen : SHEEN;
	float4 PositionCopy : TEXCOORD1;
	float4 FogBulbs : TEXCOORD2;
	float DistanceFog : FOG;
	float3 Tangent: TANGENT;
    float3 Binormal : BINORMAL;
    float3 FaceNormal : TEXCOORD3;
	unsigned int Bone : BONE;
};

struct PixelShaderOutput
{
	float4 Color: SV_TARGET0;
};

Texture2D Texture : register(t0);
SamplerState Sampler : register(s0);

Texture2D NormalTexture : register(t1);
SamplerState NormalTextureSampler : register(s1);

Texture2D CausticsTexture : register(t2);
SamplerState CausticsTextureSampler : register(s2);

Texture2D AmbientMapFrontTexture : register(t7);
SamplerState AmbientMapFrontSampler : register(s7);

Texture2D AmbientMapBackTexture : register(t8);
SamplerState AmbientMapBackSampler : register(s8);

PixelShaderInput VS(VertexShaderInput input)
{
	PixelShaderInput output;

	// Blend and apply world matrix
	float4x4 blended = Skinned ? BlendBoneMatrices(input, Bones, (Skinned == 2)) : Bones[input.BoneIndex[0]];
	float4x4 world = mul(blended, World);

	// Calculate vertex effects
	float wibble = Wibble(input.Effects, DecodeHash(input.AnimationFrameOffsetIndexHash));
	float3 pos = Move(input.Position, input.Effects, wibble);
	float3 col = Glow(input.Color.xyz, input.Effects, wibble);
	float3 worldPosition = mul(float4(pos, 1.0f), world).xyz;

	output.Position = mul(float4(worldPosition, 1.0f), ViewProjection);
    output.UV = GetUVPossiblyAnimated(input.UV, DecodeIndexInPoly(input.Effects), DecodeAnimationFrameOffset(input.AnimationFrameOffsetIndexHash));
	output.Color = float4(col, input.Color.w);
	output.Color *= Color;
	output.PositionCopy = output.Position;
    output.Sheen = DecodeSheen(input.Effects);
	output.Bone = input.BoneIndex[0];
	output.WorldPosition = worldPosition;

    output.Normal = normalize(mul(input.Normal.xyz, (float3x3) world).xyz);
    output.Tangent = normalize(mul(input.Tangent.xyz, (float3x3) world).xyz);
    output.Binormal = SafeNormalize(mul(cross(input.Normal.xyz, input.Tangent.xyz), (float3x3) world).xyz);
    output.FaceNormal = normalize(mul(input.FaceNormal.xyz, (float3x3) world).xyz);
   
	output.FogBulbs = DoFogBulbsForVertex(worldPosition);
	output.DistanceFog = DoDistanceFogForVertex(worldPosition);

	return output;
}

PixelShaderOutput PS(PixelShaderInput input)
{
	PixelShaderOutput output;

    input.UV = ConvertAnimUV(input.UV);
	
    // Apply parallax mapping
    float3x3 TBNf = float3x3(input.Tangent, input.Binormal, input.FaceNormal);
    input.UV = ParallaxOcclusionMapping(TBNf, input.WorldPosition, input.UV);  

    float4 ORSH = ConvertAnimOSRH(ORSHTexture.Sample(ORSHSampler, input.UV));
    float ambientOcclusion = ORSH.x;
    float roughness = ORSH.y;
    float specular = ORSH.z;
	
    float3 emissive = EmissiveTexture.Sample(EmissiveSampler, input.UV).xyz;
	
	float3x3 TBN = float3x3(input.Tangent, input.Binormal, input.Normal);
	float3 normal = ConvertAnimNormal(UnpackNormalMap(NormalTexture.Sample(NormalTextureSampler, input.UV)));
	normal = EnsureNormal(mul(normal, TBN), input.WorldPosition);
	
	float4 tex = Texture.Sample(Sampler, input.UV);
	DoAlphaTest(tex);
	
    // Material effects
    tex.xyz = CalculateReflections(input.WorldPosition, tex.xyz, normal , specular);

    // Ambient occlusion
    float occlusion = CalculateOcclusion(GetSamplePosition(input.PositionCopy), tex.w);
    occlusion *= ambientOcclusion;

	float3 color = (BoneLightModes[input.Bone / 4][input.Bone % 4] == 0) ?
		CombineLights(
			AmbientLight.xyz,
			input.Color.xyz,
			tex.xyz, 
			input.WorldPosition,
			normal, 
			input.Sheen,
			ItemLights, 
			NumItemLights,
			input.FogBulbs.w,
			emissive, 
			specular,
			roughness) :
		StaticLight(input.Color.xyz, tex.xyz, input.FogBulbs.w, emissive);

	float shadowable = step(0.5f, float((NumItemLights & SHADOWABLE_MASK) == SHADOWABLE_MASK));
	float3 shadow = DoShadow(input.WorldPosition, normal, color, -0.5f);
	shadow = DoBlobShadows(input.WorldPosition, shadow);
	color = lerp(color, shadow, shadowable);

	// Caustics (when item bone is below water surface)
	if (BoneWaterMask & (1u << input.Bone))
	{
		float causticsAtten = saturate(dot(float3(0.0f, -1.0f, 0.0f), normal));

		float3 blending = abs(normal);
		blending = normalize(max(blending, 0.00001f));
		float b = (blending.x + blending.y + blending.z);
		blending /= float3(b, b, b);

		float3 p = frac(input.WorldPosition.xyz / 2048.0f);

		float2 uv_x = CausticsStartUV + float2(p.z, p.y) * CausticsSize;
		float2 uv_y = CausticsStartUV + float2(p.z, p.x) * CausticsSize;
		float2 uv_z = CausticsStartUV + float2(p.y, p.x) * CausticsSize;

		float3 xaxis = CausticsTexture.SampleLevel(CausticsTextureSampler, uv_x, 0).xyz;
		float3 yaxis = CausticsTexture.SampleLevel(CausticsTextureSampler, uv_y, 0).xyz;
		float3 zaxis = CausticsTexture.SampleLevel(CausticsTextureSampler, uv_z, 0).xyz;

		float3 caustics = xaxis * blending.x + yaxis * blending.y + zaxis * blending.z;
		color += caustics * causticsAtten * 2.0f;
	}

	output.Color = saturate(float4(color * occlusion, tex.w));
	output.Color = DoFogBulbsForPixel(output.Color, float4(input.FogBulbs.xyz, 1.0f));
	output.Color = DoDistanceFogForPixel(output.Color, FogColor, input.DistanceFog);
	output.Color.w *= input.Color.w;

	return output;
}
