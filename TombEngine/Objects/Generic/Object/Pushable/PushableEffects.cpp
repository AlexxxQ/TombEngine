#include "framework.h"
#include "Objects/Generic/Object/Pushable/PushableEffects.h"

#include "Game/effects/bubble.h"
#include "Game/effects/effects.h"
#include "Game/effects/Ripple.h"
#include "Game/effects/Splash.h"
#include "Game/Setup.h"
#include "Objects/Generic/Object/Pushable/PushableObject.h"

using namespace TEN::Effects::Bubble;
using namespace TEN::Effects::Ripple;
using namespace TEN::Effects::Splash;

namespace TEN::Entities::Generic
{
	void HandlePushableRippleEffect(ItemInfo& pushableItem)
	{
		constexpr auto FRAMES_BETWEEN_RIPPLES		 = 32;
		constexpr auto FRAMES_BETWEEN_RIPPLES_SOUNDS = 32;
        constexpr auto RIPPLE_SIZE_START = 512.0f;
        constexpr auto RIPPLE_SIZE_END = 2024.0f;
        constexpr auto RIPPLE_EXPANSION_SPEED = 256.0f; // Radius units per second.

		auto& pushable = GetPushableInfo(pushableItem);

		// TODO: cleanup.
		if (pushable.WaterSurfaceHeight != NO_HEIGHT)
		{
			if (fmod(GlobalCounter, FRAMES_BETWEEN_RIPPLES) <= 0.0f)
				SpawnRipple(
					Vector3(pushableItem.Pose.Position.x, pushable.WaterSurfaceHeight, pushableItem.Pose.Position.z),
					pushableItem.RoomNumber,
                    RIPPLE_SIZE_START,
                    (int)RippleFlags::SlowFade | (int)RippleFlags::LowOpacity,
                    Vector3::Down, RIPPLE_DEFAULT_COLOR, 1.0f, RIPPLE_SIZE_END, RIPPLE_EXPANSION_SPEED);
			
			if (fmod(GlobalCounter, FRAMES_BETWEEN_RIPPLES_SOUNDS) <= 0.0f)
				pushable.SoundState = PushableSoundState::Wade;
		}
	}

	void SpawnPushableSplash(ItemInfo& pushableItem)
	{
		auto& pushable = GetPushableInfo(pushableItem);

        SpawnWaterImpactSplash(pushableItem, pushableItem.RoomNumber,
            pushable.WaterSurfaceHeight, pushableItem.Animation.Velocity.y);
	}

	void SpawnPushableBubbles(const ItemInfo& pushableItem)
	{
		constexpr auto FRAMES_BETWEEN_BUBBLES = 8.0f;

		if (fmod(GlobalCounter, FRAMES_BETWEEN_BUBBLES) <= 0.0f)
		{
			for (int i = 0; i < 32; i++)
			{
				auto pos = Vector3(
					(GetRandomControl() & 0x1FF) + pushableItem.Pose.Position.x - 256,
					(GetRandomControl() & 0x7F) + pushableItem.Pose.Position.y - 64,
					(GetRandomControl() & 0x1FF) + pushableItem.Pose.Position.z - 256);
				SpawnBubble(pos, pushableItem.RoomNumber, (int)BubbleFlags::HighAmplitude | (int)BubbleFlags::LargeScale);
			}
		}
	}

	void HandlePushableOscillation(ItemInfo& pushableItem)
	{
		constexpr auto BOX_VOLUME_MIN = BLOCK(0.5f);

		const auto& pushable = GetPushableInfo(pushableItem);

		auto time = GlobalCounter + pushableItem.Animation.Velocity.y;

		// Calculate bounding box volume scaling factor.
		auto bounds = GameBoundingBox(&pushableItem);
		float boxVolume = bounds.GetWidth() * bounds.GetDepth() * bounds.GetHeight();
		float boxScale = std::sqrt(std::min(BOX_VOLUME_MIN, boxVolume)) / 32.0f;
		boxScale *= pushable.Oscillation;

		float xOsc = (std::sin(time * 0.05f) * 0.5f) * boxScale;
		float zOsc = (std::sin(time * 0.1f) * 0.75f) * boxScale;

		short xAngle = ANGLE(xOsc * 20.0f);
		short zAngle = ANGLE(zOsc * 20.0f);
		pushableItem.Pose.Orientation = EulerAngles(xAngle, pushableItem.Pose.Orientation.y, zAngle);
	}

	void HandlePushableBridgeOscillation(ItemInfo& pushableItem)
	{
		constexpr auto BOX_VOLUME_MIN = BLOCK(0.5f);

		const auto& pushable = GetPushableInfo(pushableItem);
		auto time = GlobalCounter + pushableItem.Animation.Velocity.y;

		// Calculate bounding box volume scaling factor.
		auto bounds = GameBoundingBox(&pushableItem);
		float boxVolume = bounds.GetWidth() * bounds.GetDepth() * bounds.GetHeight();
		float boxScale = std::sqrt(std::min(BOX_VOLUME_MIN, boxVolume)) / 32.0f;
		boxScale *= pushable.Oscillation;

		// Vertical oscillation.
		float verticalOsc = (std::sin(time * 0.2f) * 0.5f) * boxScale * 32;
		short verticalTranslation = (short)verticalOsc;

		pushableItem.Pose.Position.y += verticalTranslation;
	}

	void HandlePushableFallRotation(ItemInfo& item)
	{
		auto& pushableItem = item;

		// Check if orientation is outside threeshold.
		short orientThreshold = 1;
		float correctionStep = 40.0f / 30; // 40 deg / 30 frames to do the correction in 1 sec approx.

		if (abs(pushableItem.Pose.Orientation.x) >= orientThreshold ||
			abs(pushableItem.Pose.Orientation.y) >= orientThreshold)
		{
			if (pushableItem.Pose.Orientation.x > 0)
			{
				pushableItem.Pose.Orientation.x -= correctionStep;
				if (pushableItem.Pose.Orientation.x < 0)
					pushableItem.Pose.Orientation.x = 0;
			}
			else
			{
				pushableItem.Pose.Orientation.x += correctionStep;
				if (pushableItem.Pose.Orientation.x > 0)
					pushableItem.Pose.Orientation.x = 0;
			}

			if (pushableItem.Pose.Orientation.z > 0)
			{
				pushableItem.Pose.Orientation.z -= correctionStep;
				if (pushableItem.Pose.Orientation.z < 0)
					pushableItem.Pose.Orientation.z = 0;
			}
			else
			{
				pushableItem.Pose.Orientation.z += correctionStep;
				if (pushableItem.Pose.Orientation.z > 0)
					pushableItem.Pose.Orientation.z = 0;
			}
		}
	}
}
