#ifndef CBITEMSHADER
#define CBITEMSHADER

#include "./ShaderLight.hlsli"

cbuffer CBItem : register(b1)
{
	float4x4 World;
	//--
	float4x4 Bones[MAX_BONES];
	//--
	float4 Color;
	//--
	float4 AmbientLight;
	//--
	float4 WaterAmbientLight;
	//--
	int4 BoneLightModes[MAX_BONES / 4];
	//--
	ShaderLight ItemLights[MAX_LIGHTS_PER_ITEM];
	//--
	int NumItemLights;
	int Skinned;
	float WaterHeight;
	int ItemPad1;
};

#endif // CBITEMSHADER
