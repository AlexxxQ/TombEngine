#include "framework.h"
#include "Game/effects/Ripple.h"

#include "Game/effects/effects.h"
#include "Game/Setup.h"
#include "Math/Math.h"
#include "Objects/objectslist.h"
#include "Specific/clock.h"

using namespace TEN::Math;

namespace TEN::Effects::Ripple
{

	std::vector<Ripple> Ripples = {};

	void SpawnRipple(const Vector3& pos, int roomNumber, float size, int flags, const Vector3& normal, const Vector4& color, float finalSizeScale, float finalSize, float expansionSpeed)
	{
		constexpr auto LIFE_WATER_SURFACE_MAX = 1.0f;
		constexpr auto LIFE_WATER_SURFACE_MIN = LIFE_WATER_SURFACE_MAX / 2;
		constexpr auto LIFE_GROUND_MAX		  = 0.4f;
		constexpr auto LIFE_GROUND_MIN		  = LIFE_GROUND_MAX / 2;
		constexpr auto FADE_FAST_COEFF		  = 1 / 3.0f;
		constexpr auto FADE_SLOW_COEFF		  = 0.5f;
        constexpr auto SIZE_STEP_LARGE = 4.0f;
        constexpr auto SIZE_STEP_SMALL = 2.0f;

		auto& ripple = GetNewEffect(Ripples, RIPPLE_COUNT_MAX);

		float lifeInSec = (flags & (int)RippleFlags::OnGround) ?
			Random::GenerateFloat(LIFE_GROUND_MIN, LIFE_GROUND_MAX) :
			Random::GenerateFloat(LIFE_WATER_SURFACE_MIN, LIFE_WATER_SURFACE_MAX);
		float fadeDurationInSec = lifeInSec * ((ripple.Flags & ((int)RippleFlags::SlowFade)) ? FADE_SLOW_COEFF : FADE_FAST_COEFF);

		ripple.SpriteIndex = Objects[ID_DEFAULT_SPRITES].meshIndex + SPR_RIPPLES;
		ripple.Position = pos;
		ripple.RoomNumber = roomNumber;
		ripple.Normal = normal;
		ripple.Color = color;
		ripple.Life =
		ripple.LifeMax = round(lifeInSec * FPS);
		ripple.Size = size;
        float baseSizeStep = (flags & ((int)RippleFlags::SlowFade | (int)RippleFlags::OnGround)) ? SIZE_STEP_SMALL : SIZE_STEP_LARGE;
        ripple.SizeStep = baseSizeStep + (std::clamp(finalSizeScale, 1.0f, 2.0f) - 1.0f) * (size / ripple.LifeMax + baseSizeStep);
        if (finalSize > 0.0f)
        {
            ripple.SizeMax = std::max(size, finalSize);
            ripple.SizeStep = (ripple.SizeMax - size) / ripple.LifeMax;
            if (expansionSpeed > 0.0f && ripple.SizeMax > size)
            {
                ripple.SizeStep = expansionSpeed / FPS;
                ripple.Life = ripple.LifeMax = std::max(1.0f, std::ceil((ripple.SizeMax - size) / ripple.SizeStep));
                fadeDurationInSec *= ripple.LifeMax / round(lifeInSec * FPS);
            }
        }
		ripple.FadeDuration = round(fadeDurationInSec * FPS);
		ripple.Flags = flags;
	}

	void UpdateRipples()
	{
		if (Ripples.empty())
			return;

		for (auto& ripple : Ripples)
		{
			if (ripple.Life <= 0.0f)
				continue;

			ripple.StoreInterpolationData();

			// Update size.
			if (ripple.Size < ripple.SizeMax)
                ripple.Size = std::min(ripple.Size + ripple.SizeStep, ripple.SizeMax);

			float lifeFullOpacity = ripple.LifeMax - (ripple.FadeDuration * 0.75f);
			float lifeStartFading = ripple.FadeDuration;

			// Update opacity.
			if (ripple.Life >= lifeFullOpacity)
			{
				float alpha = 1.0f - ((ripple.Life - lifeFullOpacity) / (ripple.LifeMax - lifeFullOpacity));
				ripple.Color.w = Lerp(0.0f, RIPPLE_OPACITY_MAX, alpha);
			}
			else if (ripple.Life <= lifeStartFading)
			{
				float alpha = 1.0f - (ripple.Life / lifeStartFading);
				ripple.Color.w = Lerp(RIPPLE_OPACITY_MAX, 0.0f, alpha);
			}

			// Update life.
			ripple.Life -= 1.0f;
		}

		ClearInactiveEffects(Ripples);
	}

	void ClearRipples()
	{
		Ripples.clear();
	}
}
