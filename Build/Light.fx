#define LIGHTTYPE_SUN				0
#define LIGHTTYPE_POINT				1
#define LIGHTTYPE_SPOT				2
#define LIGHTTYPE_SHADOW			3

#define MODELTYPE_HORIZON			0
#define MODELTYPE_ROOM				1
#define MODELTYPE_MOVEABLE			2
#define MODELTYPE_STATIC			3
#define MODELTYPE_INVENTORY			4
#define MODELTYPE_PICKUP			5
#define MODELTYPE_LARA				6
#define MODELTYPE_SKY				7

struct VertexShaderInput
{
	float3 Position : POSITION0;
	float2 TextureCoordinate : TEXCOORD0;
};

struct VertexShaderOutput
{
	float4 Position : POSITION0;
	float2 TextureCoordinate : TEXCOORD0;
	float4 ScreenPosition : TEXCOORD1;
};

struct PixelShaderOutput
{
	float4 Light : COLOR0;
	float4 Shadow : COLOR1;
};

float4 LightPosition;
float4 LightDirection;
float4 LightColor;
int LightType;
float LightIn;
float LightOut;
float LightIntensity;
float LightRange;
bool LightDynamic;
bool CastShadows;

float3 CameraPosition;

float4x4 World;
float4x4 View;
float4x4 Projection;
float4x4 ViewProjectionInverse;
float4x4 LightView;
float4x4 LightProjection;

bool AmbientPass;

float HalfPixelX = 1.0f / 800.0f;
float HalfPixelY = 1.0f / 600.0f;

texture2D ColorMap;
sampler ColorSampler = sampler_state
{
	Texture = (ColorMap);
	AddressU = CLAMP;
	AddressV = CLAMP;
	MagFilter = LINEAR;
	MinFilter = LINEAR;
	Mipfilter = LINEAR;
};

texture2D DepthMap;
sampler DepthSampler = sampler_state
{
	Texture = (DepthMap);
	AddressU = CLAMP;
	AddressV = CLAMP;
	MagFilter = POINT;
	MinFilter = POINT;
	Mipfilter = POINT;
};

texture2D NormalMap;
sampler NormalSampler = sampler_state
{
	Texture = (NormalMap);
	AddressU = CLAMP;
	AddressV = CLAMP;
	MagFilter = POINT;
	MinFilter = POINT;
	Mipfilter = POINT;
};

texture2D VertexColorMap;
sampler VertexColorSampler = sampler_state
{
	Texture = (VertexColorMap);
	AddressU = CLAMP;
	AddressV = CLAMP;
	MagFilter = LINEAR;
	MinFilter = LINEAR;
	Mipfilter = LINEAR;
};

/*textureCUBE ShadowMapCube;
samplerCUBE ShadowMapCubeSampler = sampler_state {
	texture = (ShadowMapCube);
	MinFilter = Point;
	MagFilter = Point;
	MipFilter = None;
	AddressU = Clamp;
	AddressV = Clamp;
};*/

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
	VertexShaderOutput output;

	if (LightType == LIGHTTYPE_SUN)
		output.Position = float4(input.Position, 1.0f);
	else
	{
		float4 worldPosition = mul(float4(input.Position, 1.0f), World);
		float4 viewPosition = mul(worldPosition, View);
		output.Position = mul(viewPosition, Projection);
	}

	output.TextureCoordinate = input.TextureCoordinate - float2(HalfPixelX, HalfPixelY);
	output.ScreenPosition = output.Position;

	return output;
}

float4 PixelShaderFunction(VertexShaderOutput input) : COLOR0
{
	float4 output;
	output = float4(0.0f, 0.0f, 0.0f, 0.0f);

	float2 texCoord = input.TextureCoordinate;
	if (LightType != LIGHTTYPE_SUN)
	{
		input.ScreenPosition.xy /= input.ScreenPosition.w;
		texCoord = 0.5f * (float2(input.ScreenPosition.x, -input.ScreenPosition.y) + 1);
		texCoord -= float2(0.5f / 800.0f, 0.5f / 600.0f);
	}

	// Get the normal and transform back to -1 ... 1
	float4 normalData = tex2D(NormalSampler, texCoord);
	float3 normal = 2.0f * normalData - 1.0f;
	 
	//normal.z = sqrt(1.0f - normal.x * normal.x - normal.y * normal.y);
	
	int modelType = (int)(normalData.w * 16.0f);
	float specularPower = 1.0f; // normalData.w;

	// Get the depth value
	float depthVal = tex2D(DepthSampler, texCoord).r;

	// We lit room geometry only in the case of dynamic lights
	if (modelType == MODELTYPE_ROOM && !LightDynamic)
	{
		// Is this light dynamic? In not, then I don't do any further calculations
		return output;
	}

	// Get specular intensity
	float specularIntensity = tex2D(ColorSampler, texCoord).a;

	// Now the shader is different according to light type
	if (LightType == LIGHTTYPE_SUN)
	{
		// Compute screen-space position
		float4 position;
		position.x = input.TextureCoordinate.x * 2.0f - 1.0f;
		position.y = -(input.TextureCoordinate.y * 2.0f - 1.0f);
		position.z = depthVal;
		position.w = 1.0f;

		// Transform to world space
		position = mul(position, ViewProjectionInverse);
		position /= position.w;

		// Surface-to-light vector
		float3 lightVector = -normalize(LightDirection);

		// Compute diffuse light
		float NdL = max(0, dot(normal, lightVector));
		float3 diffuseLight = NdL * LightColor.rgb;

		// Reflection vector
		float3 reflectionVector = normalize(reflect(lightVector, normal));

		// Camera-to-surface vector
		float3 directionToCamera = normalize(CameraPosition - position);
		
		// Compute specular light
		float specularLight = specularIntensity * pow(saturate(dot(reflectionVector, directionToCamera)), specularPower);

		// Output the two lights
		output = float4(diffuseLight.rgb, specularLight);
	}
	else if (LightType == LIGHTTYPE_POINT)
	{
		// Compute screen-space position
		float4 position;
		position.xy = input.ScreenPosition.xy;
		position.z = depthVal;
		position.w = 1.0f;

		// Transform to world space
		position = mul(position, ViewProjectionInverse);
		position /= position.w;

		// Surface-to-light vector
		float3 lightVector = LightPosition - position;

		// Compute attenuation based on distance - linear attenuation
		float attenuation = saturate(1.0f - length(lightVector) / LightOut);
		
		// Normalize light vector
		lightVector = normalize(lightVector);
		
		// Compute diffuse light
		float NdL = pow((dot(normal, lightVector) * 0.5f) + 0.5f, 2); //max(0, dot(normal, lightVector));
		float3 diffuseLight = NdL * LightColor.rgb;

		// Reflection vector
		float3 reflectionVector = normalize(reflect(-lightVector, normal));
		
		// Camera-to-surface vector
		float3 directionToCamera = normalize(CameraPosition - position);
		
		// Compute specular light
		float specularLight = specularIntensity * pow(saturate(dot(reflectionVector, directionToCamera)), specularPower);

		// Take into account attenuation and lightIntensity.
		output = attenuation * LightIntensity * float4(diffuseLight.rgb, specularLight) * 0.75f;
	}
	
	//output.a = 0.5f;

	return output;
}

technique Light
{
	pass Pass1
	{
		VertexShader = compile vs_3_0 VertexShaderFunction();
		PixelShader = compile ps_3_0 PixelShaderFunction();
	}
}