#ifndef CBITEMSHADER
#define CBITEMSHADER

#include "./ShaderLight.hlsli"

static const float ITEM_NO_WATER_SURFACE = 100000.0f;

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
    int4 BoneLightModes[MAX_BONES / 4];
	//--
    ShaderLight ItemLights[MAX_LIGHTS_PER_ITEM];
	//--
	int NumItemLights;
	int Skinned;
	float WaterSurfaceHeight;
	int CBItemPad0;
};

#endif // CBITEMSHADER
