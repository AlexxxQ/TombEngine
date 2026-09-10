#include "framework.h"
#include "Game/effects/Splash.h"

#include "Game/collision/Point.h"
#include "Game/effects/drip.h"
#include "Game/effects/effects.h"
#include "Game/effects/Ripple.h"
#include "Game/items.h"
#include "Game/room.h"
#include "Math/Objects/GameBoundingBox.h"
#include "Sound/sound.h"

using namespace TEN::Collision::Point;
using namespace TEN::Effects::Drip;
using namespace TEN::Effects::Ripple;

namespace TEN::Effects::Splash
{
	constexpr auto SPLASH_DEFAULT_RADIUS = 64;
	constexpr auto SPLASH_AUDIO_DRIP_COOLDOWN = 16; // Safeguard against stacked splash sound and drip spam.
	constexpr auto WATER_ENTRY_RADIUS_MIN = 16.0f;
	constexpr auto WATER_ENTRY_RADIUS_MAX = CLICK(2.0f);
	constexpr auto WATER_ENTRY_SPLASH_POINT_COUNT_MAX = 3;
	constexpr auto WATER_ENTRY_SPLASH_SETUP_COUNT_MAX = 2;
	constexpr auto PLAYER_WATER_ENTRY_RADIUS = 64.0f;
	constexpr auto PLAYER_WATER_ENTRY_SPLASH_SETUP_COUNT = 3;

	struct WaterEntrySplashPoint
	{
		Vector3 Position = Vector3::Zero;
		float Radius = 0.0f;
	};

	static std::optional<int> GetWaterEntryHeight(int sourceRoomNumber, int destinationRoomNumber, const Vector3i& position, float verticalVelocity)
	{
		if (verticalVelocity <= 0.0f ||
			TestEnvironment(ENV_FLAG_WATER, sourceRoomNumber) ||
			!TestEnvironment(ENV_FLAG_WATER, destinationRoomNumber) ||
			TestEnvironment(ENV_FLAG_SWAMP, destinationRoomNumber))
		{
			return std::nullopt;
		}

		int waterHeight = GetPointCollision(position, destinationRoomNumber).GetWaterTopHeight();
		if (waterHeight == NO_HEIGHT)
			return std::nullopt;

		return waterHeight;
	}

	int	SplashCount; // Lara-specific splash cooldown used when entering water.
	int	SplashTimeout; // Global cooldown for splash sound and drip spam suppression.

	SplashEffectSetup								  SplashSetup;
	std::array<SplashEffect, SPLASH_EFFECT_COUNT_MAX> SplashEffects;

	void SetupSplash(const SplashEffectSetup* const setup, int room, int setupCountMax)
	{
		int splashSetupCount = 0;
		float splashVel = 0.0f;

		for (auto& splash : SplashEffects)
		{
			if (splash.isActive)
				continue;

			if (splashSetupCount == 0)
			{
				float splashPower = std::min(256.0f, setup->SplashPower);

				splash.isActive = true;
				splash.Position = setup->Position;
				splash.life = 62;
				splash.isRipple = false;
				splash.InnerRadius = setup->InnerRadius / 2;
				splashVel = splashPower / 16;
				splash.InnerRadialVel = splashVel;
				splash.HeightSpeed = splashPower * 1.0f;
				splash.height = 0;
				splash.HeightVel = -16;
				splash.OuterRadius = setup->InnerRadius;
				splash.outerRadialVel = splashVel * 1.5f;
				splash.SpriteSeqStart = 8; // Splash texture.
				splashSetupCount++;
			}
			else
			{
				float thickness = Random::GenerateFloat(64, 128);

				splash.isActive = true;
				splash.Position = setup->Position;
				splash.isRipple = true;
				float vel = 0.0f;

				if (splashSetupCount == 2)
				{
					vel = (splashVel / 16) + Random::GenerateFloat(2, 4);
				}
				else
				{
					vel = (splashVel / 7) + Random::GenerateFloat(3, 7);
				}

				splash.InnerRadius = 0.0f;
				splash.InnerRadialVel = vel * 1.3f;
				splash.OuterRadius = thickness;
				splash.outerRadialVel = vel * 2.3f;
				splash.HeightSpeed = 128;
				splash.height = 0;
				splash.HeightVel = -16;

				float alpha = (vel / (splashVel / 2)) + 16;
				alpha = std::max(0.0f, std::min(alpha, 1.0f));

				splash.life = Lerp(48.0f, 70.0f, alpha);
				splash.SpriteSeqStart = 4; // Splash texture.
				splash.SpriteSeqEnd = 7; // Splash texture.
				splash.AnimSpeed = fmin(0.6f, (1 / splash.outerRadialVel) * 2);

				splashSetupCount++;
			}

			if (splashSetupCount == setupCountMax)
				break;
		}

		if (SplashTimeout == 0)
		{
			SpawnSplashDrips(Vector3(setup->Position.x, setup->Position.y - 15, setup->Position.z), room, 32);

			auto soundPose = Pose(Vector3i(setup->Position));
			SoundEffect(SFX_TR4_LARA_SPLASH, &soundPose);
			SplashTimeout = SPLASH_AUDIO_DRIP_COOLDOWN;
		}
	}

	void UpdateSplashes()
	{
		if (SplashCount)
			SplashCount--;

		if (SplashTimeout)
			SplashTimeout--;

		for (auto& splash : SplashEffects)
		{
			if (splash.isActive)
			{
				splash.StoreInterpolationData();

				splash.life--;
				if (splash.life <= 0)
					splash.isActive = false;

				splash.HeightSpeed += splash.HeightVel;
				splash.height += splash.HeightSpeed;

				if (splash.height < 0)
				{
					splash.height = 0;
					if (!splash.isRipple)
						splash.isActive = false;
				}

				splash.InnerRadius += splash.InnerRadialVel;
				splash.OuterRadius += splash.outerRadialVel;
				splash.AnimPhase += splash.AnimSpeed;

				int sequenceLength = splash.SpriteSeqEnd - splash.SpriteSeqStart;
				if (splash.AnimPhase > sequenceLength)
					splash.AnimPhase = fmod(splash.AnimPhase, sequenceLength);
			}
		}
	}

	void ClearSplashes()
	{
		SplashCount = 0;
		SplashTimeout = 0;

		for (auto& splash : SplashEffects)
			splash = {};
	}

	void SpawnPlayerWaterEntrySplash(const ItemInfo& item, int sourceRoomNumber, int destinationRoomNumber, float verticalVelocity)
	{
		auto waterHeight = GetWaterEntryHeight(sourceRoomNumber, destinationRoomNumber, item.Pose.Position, verticalVelocity);
		if (!waterHeight.has_value())
			return;

		auto setup = SplashEffectSetup{};
		setup.Position = Vector3(item.Pose.Position.x, *waterHeight - 1.0f, item.Pose.Position.z);
		setup.SplashPower = std::clamp(verticalVelocity, WATER_ENTRY_RADIUS_MIN, 256.0f);
		setup.InnerRadius = PLAYER_WATER_ENTRY_RADIUS;
		SetupSplash(&setup, destinationRoomNumber, PLAYER_WATER_ENTRY_SPLASH_SETUP_COUNT);
	}

	void SpawnWaterEntrySplash(const ItemInfo& item, int sourceRoomNumber, int destinationRoomNumber, float verticalVelocity, int splashPointCountMax)
	{
		auto waterHeight = GetWaterEntryHeight(sourceRoomNumber, destinationRoomNumber, item.Pose.Position, verticalVelocity);
		if (!waterHeight.has_value())
			return;

		auto bounds = item.GetAabb();
		auto extents = (Vector3)bounds.Extents;
		float boundsRadius = Vector2(extents.x, extents.z).Length();
		float radiusLimit = std::clamp(boundsRadius, WATER_ENTRY_RADIUS_MIN, WATER_ENTRY_RADIUS_MAX);
		float scale = std::max({ item.Pose.Scale.x, item.Pose.Scale.y, item.Pose.Scale.z });

		auto points = std::vector<WaterEntrySplashPoint>{};
		for (const auto& sphere : item.GetSpheres())
		{
			float radius = std::clamp(sphere.Radius * scale, WATER_ENTRY_RADIUS_MIN, radiusLimit);
			points.push_back({ sphere.Center, radius });
		}

		if (points.empty())
			points.push_back({ (Vector3)bounds.Center, radiusLimit });

		std::sort(points.begin(), points.end(), [](const WaterEntrySplashPoint& pointA, const WaterEntrySplashPoint& pointB)
		{
			return pointA.Radius > pointB.Radius;
		});

		splashPointCountMax = std::clamp(splashPointCountMax, 1, WATER_ENTRY_SPLASH_POINT_COUNT_MAX);
		if (splashPointCountMax == 1)
			points[0].Position = (Vector3)bounds.Center;

		auto selectedPoints = std::vector<WaterEntrySplashPoint>{};

		for (const auto& point : points)
		{
			bool overlapsSelectedPoint = false;
			for (const auto& selectedPoint : selectedPoints)
			{
				float dx = point.Position.x - selectedPoint.Position.x;
				float dz = point.Position.z - selectedPoint.Position.z;
				float separation = (point.Radius + selectedPoint.Radius) / 2.0f;

				if ((dx * dx) + (dz * dz) < SQUARE(separation))
				{
					overlapsSelectedPoint = true;
					break;
				}
			}

			if (overlapsSelectedPoint)
				continue;

			selectedPoints.push_back(point);
			if (selectedPoints.size() >= splashPointCountMax)
				break;
		}

		float splashPower = std::clamp(verticalVelocity * 2.0f, WATER_ENTRY_RADIUS_MIN, 256.0f);
		for (const auto& point : selectedPoints)
		{
			auto setup = SplashEffectSetup{};
			setup.Position = Vector3(point.Position.x, *waterHeight - 1.0f, point.Position.z);
			setup.SplashPower = splashPower;
			setup.InnerRadius = point.Radius * Random::GenerateFloat(0.9f, 1.1f);
			SetupSplash(&setup, destinationRoomNumber, WATER_ENTRY_SPLASH_SETUP_COUNT_MAX);
		}
	}

	static bool TestWadeWaterEffectFrame(const ItemInfo& item)
	{
		return ((Wibble + ((item.Index & 3) * 4)) & 0xF) == 0;
	}

	void SpawnWadeWaterEffects(const ItemInfo& item, int roomNumber, int waterHeight, bool isIdle)
	{
		if (!TestWadeWaterEffectFrame(item))
			return;

		auto pointColl = GetPointCollision(item.Pose.Position, roomNumber);
		roomNumber = pointColl.GetRoomNumber();

		if (!TestEnvironment(ENV_FLAG_WATER, roomNumber) ||
			TestEnvironment(ENV_FLAG_SWAMP, roomNumber))
			return;

		if (waterHeight == NO_HEIGHT)
			waterHeight = pointColl.GetWaterTopHeight();

		if (waterHeight == NO_HEIGHT)
			return;

		auto bounds = GameBoundingBox(&item);
		if (item.Pose.Position.y + bounds.Y1 > waterHeight ||
			item.Pose.Position.y + bounds.Y2 < waterHeight)
		{
			return;
		}

		if (isIdle && !Random::TestProbability(1 / 16.0f))
			return;

		float radius = Random::GenerateFloat(112.0f, 128.0f);
		int flags = isIdle ? (int)RippleFlags::LowOpacity : (int)RippleFlags::SlowFade | (int)RippleFlags::LowOpacity;

		SpawnRipple(
			Vector3(item.Pose.Position.x, waterHeight - 1, item.Pose.Position.z),
			roomNumber, radius, flags);
	}

	void Splash(ItemInfo* item)
	{
		int probedRoomNumber = GetPointCollision(*item).GetRoomNumber();
		Splash(item->Pose.Position, probedRoomNumber, item->Animation.Velocity.y);
	}

	void Splash(Vector3i position, int roomNumber, int power)
	{
		if (!TestEnvironment(ENV_FLAG_WATER, roomNumber))
			return;

		int waterHeight = GetPointCollision(position, roomNumber).GetWaterTopHeight();

		SplashSetup.Position = Vector3(position.x, waterHeight - 1, position.z);
		SplashSetup.SplashPower = power;
		SplashSetup.InnerRadius = SPLASH_DEFAULT_RADIUS;
		SetupSplash(&SplashSetup, roomNumber);

	}
}
