#include "framework.h"
#include "Game/effects/ScreenSprite.h"

#include "Game/effects/effects.h"
#include "Game/Setup.h"
#include "Math/Math.h"
#include "Objects/objectslist.h"
#include "Renderer/Renderer11.h"

using namespace TEN::Math;

namespace TEN::Effects::DisplaySprite
{
	std::vector<DisplaySprite> DisplaySprites = {};

	void AddDisplaySprite(GAME_OBJECT_ID objectID, int spriteIndex, const Vector2& pos, short orient, const Vector2& scale, const Vector4& color,
						  int priority, DisplaySpriteOriginType originType, DisplaySpriteScaleMode scaleMode, BLEND_MODES blendMode)
	{
		auto displaySprite = DisplaySprite{};

		displaySprite.ObjectID = objectID;
		displaySprite.SpriteIndex = spriteIndex;
		displaySprite.Position = pos;
		displaySprite.Orientation = orient;
		displaySprite.Scale = scale;
		displaySprite.Color = color;
		displaySprite.Priority = priority;
		displaySprite.OriginType = originType;
		displaySprite.ScaleMode = scaleMode;
		displaySprite.BlendMode = blendMode;

		DisplaySprites.push_back(displaySprite);
	}

	void ClearDisplaySprites()
	{
		DisplaySprites.clear();
	}
}
