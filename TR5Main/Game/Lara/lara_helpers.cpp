#include "framework.h"
#include "Game/Lara/lara_helpers.h"

#include "Game/collision/collide_room.h"
#include "Game/control/control.h"
#include "Game/items.h"
#include "Game/Lara/lara.h"
#include "Game/Lara/lara_collide.h"
#include "Game/Lara/lara_fire.h"
#include "Game/Lara/lara_tests.h"
#include "Scripting/GameFlowScript.h"
#include "Specific/input.h"
#include "Specific/level.h"
#include "Specific/setup.h"

#include "Objects/TR2/Vehicles/skidoo.h"
#include "Objects/TR3/Vehicles/biggun.h"
#include "Objects/TR3/Vehicles/kayak.h"
#include "Objects/TR3/Vehicles/minecart.h"
#include "Objects/TR3/Vehicles/quad.h"
#include "Objects/TR3/Vehicles/upv.h"
#include "Objects/TR4/Vehicles/jeep.h"
#include "Objects/TR4/Vehicles/motorbike.h"

// -----------------------------
// HELPER FUNCTIONS
// For State Control & Collision
// -----------------------------

void HandleLaraMovementParameters(ITEM_INFO* item, COLL_INFO* coll)
{
	auto* lara = GetLaraInfo(item);

	// Update AFK pose timer.
	if (lara->Control.Count.Pose < LARA_POSE_TIME && TestLaraPose(item, coll) &&
		!(TrInput & (IN_WAKE | IN_LOOK)) &&
		g_GameFlow->Animations.HasPose)
	{
		lara->Control.Count.Pose++;
	}
	else
		lara->Control.Count.Pose = 0;

	// Reset running jump timer.
	if (!IsRunJumpCountableState((LaraState)item->Animation.ActiveState))
		lara->Control.Count.RunJump = 0;

	// Reset running jump action queue.
	if (!IsRunJumpQueueableState((LaraState)item->Animation.ActiveState))
		lara->Control.RunJumpQueued = false;

	// Reset lean.
	if (!lara->Control.IsMoving || (lara->Control.IsMoving && !(TrInput & (IN_LEFT | IN_RIGHT))))
		ResetLaraLean(item, 6);

	// Reset crawl flex.
	if (!(TrInput & IN_LOOK) && coll->Setup.Height > LARA_HEIGHT - LARA_HEADROOM &&	// HACK
		(!item->Animation.Velocity || (item->Animation.Velocity && !(TrInput & (IN_LEFT | IN_RIGHT)))))
	{
		ResetLaraFlex(item, 12);
	}

	// Reset turn rate.
	// TODO: Make it less stupid in the future. Do it according to a curve?
	int sign = copysign(1, lara->Control.TurnRate);
	if (abs(lara->Control.TurnRate) > ANGLE(2.0f))
		lara->Control.TurnRate -= ANGLE(2.0f) * sign;
	else if (abs(lara->Control.TurnRate) > ANGLE(0.5f))
		lara->Control.TurnRate -= ANGLE(0.5f) * sign;
	else
		lara->Control.TurnRate = 0;
	item->Position.yRot += lara->Control.TurnRate;
}

bool HandleLaraVehicle(ITEM_INFO* item, COLL_INFO* coll)
{
	auto* lara = GetLaraInfo(item);

	if (lara->Vehicle != NO_ITEM)
	{
		switch (g_Level.Items[lara->Vehicle].ObjectNumber)
		{
		case ID_QUAD:
			QuadBikeControl(item, coll);
			break;

		case ID_JEEP:
			JeepControl();
			break;

		case ID_MOTORBIKE:
			MotorbikeControl();
			break;

		case ID_KAYAK:
			KayakControl(item);
			break;

		case ID_SNOWMOBILE:
			SkidooControl(item, coll);
			break;

		case ID_UPV:
			UPVControl(item, coll);
			break;

		case ID_MINECART:
			MineCartControl(item);
			break;

		case ID_BIGGUN:
			BigGunControl(item, coll);
			break;

			// Boats are processed like normal items in loop.
		default:
			LaraGun(item);
		}

		return true;
	}

	return false;
}

void ApproachLaraTargetAngle(ITEM_INFO* item, short targetAngle, float rate)
{
	auto* lara = GetLaraInfo(item);

	if (!rate)
	{
		TENLog(std::string("ApproachLaraTargetAngle() attempted division by zero!"), LogLevel::Warning);
		return;
	}

	rate = abs(rate);

	if (abs((short)(targetAngle - item->Position.yRot)) > ANGLE(0.1f))
		item->Position.yRot += (short)(targetAngle - item->Position.yRot) / rate;
	else
		item->Position.yRot = targetAngle;
}

void EaseOutLaraHeight(ITEM_INFO* item, int height)
{
	if (height == NO_HEIGHT)
		return;

	// Translate Lara to new height.
	// TODO: This approach may cause undesirable artefacts where an object pushes Lara rapidly up/down a slope or a platform rapidly ascends/descends.
	static constexpr int rate = 50;
	int threshold = std::max(abs(item->Animation.Velocity) * 1.5f, CLICK(0.25f) / 4);
	int sign = std::copysign(1, height);
	
	if (TestEnvironment(ENV_FLAG_SWAMP, item) && height > 0)
		item->Position.yPos += SWAMP_GRAVITY;
	else if (abs(height) > (STEPUP_HEIGHT / 2))		// Outer range.
		item->Position.yPos += rate * sign;
	else if (abs(height) <= (STEPUP_HEIGHT / 2) &&	// Inner range.
		abs(height) >= threshold)
	{
		item->Position.yPos += std::max<int>(abs(height / 2.75), threshold) * sign;
	}
	else
		item->Position.yPos += height;
}

// TODO: Make lean rate proportional to the turn rate, allowing for nicer aesthetics with future analog stick input.
void DoLaraLean(ITEM_INFO* item, COLL_INFO* coll, short maxAngle, short rate)
{
	if (!item->Animation.Velocity)
		return;

	rate = abs(rate);
	int sign = copysign(1, maxAngle);

	if (coll->CollisionType == CT_LEFT || coll->CollisionType == CT_RIGHT)
		item->Position.zRot += std::min<short>(rate, abs((maxAngle * 3) / 5 - item->Position.zRot) / 3) * sign;
	else
		item->Position.zRot += std::min<short>(rate, abs(maxAngle - item->Position.zRot) / 3) * sign;
}

// TODO: Some states can't make the most of this function due to missing step up/down animations.
// Try implementing leg IK as a substitute to make step animations obsolete. @Sezz 2021.10.09
void DoLaraStep(ITEM_INFO* item, COLL_INFO* coll)
{
	if (!TestEnvironment(ENV_FLAG_SWAMP, item))
	{
		if (TestLaraStepUp(item, coll))
		{
			item->Animation.TargetState = LS_STEP_UP;
			if (GetChange(item, &g_Level.Anims[item->Animation.AnimNumber]))
			{
				item->Position.yPos += coll->Middle.Floor;
				return;
			}
		}
		else if (TestLaraStepDown(item, coll))
		{
			item->Animation.TargetState = LS_STEP_DOWN;
			if (GetChange(item, &g_Level.Anims[item->Animation.AnimNumber]))
			{
				item->Position.yPos += coll->Middle.Floor;
				return;
			}
		}
	}

	EaseOutLaraHeight(item, coll->Middle.Floor);
}

void DoLaraMonkeyStep(ITEM_INFO* item, COLL_INFO* coll)
{
	EaseOutLaraHeight(item, coll->Middle.Ceiling);
}

// TODO: Doesn't always work on bridges.
void DoLaraCrawlToHangSnap(ITEM_INFO* item, COLL_INFO* coll)
{
	coll->Setup.ForwardAngle = item->Position.yRot + ANGLE(180.0f);
	GetCollisionInfo(coll, item);
	SnapItemToLedge(item, coll);
	MoveItem(item, item->Position.yRot, -LARA_RAD_CRAWL);
	item->Position.yRot += ANGLE(180.0f);
	LaraResetGravityStatus(item, coll);
}

void DoLaraCrawlFlex(ITEM_INFO* item, COLL_INFO* coll, short maxAngle, short rate)
{
	auto* lara = GetLaraInfo(item);

	if (!item->Animation.Velocity)
		return;

	int sign = copysign(1, maxAngle);
	rate = copysign(rate, maxAngle);

	lara->ExtraTorsoRot.zRot += std::min(abs(rate), abs(maxAngle - lara->ExtraTorsoRot.zRot) / 6) * sign;

	if (!(TrInput & IN_LOOK) &&
		item->Animation.ActiveState != LS_CRAWL_BACK)
	{
		lara->ExtraHeadRot.zRot = lara->ExtraTorsoRot.zRot / 2;
		lara->ExtraHeadRot.yRot = lara->ExtraHeadRot.zRot;
	}
}

void DoLaraTightropeBalance(ITEM_INFO* item)
{
	auto* lara = GetLaraInfo(item);
	const int factor = ((lara->Control.Tightrope.TimeOnTightrope >> 7) & 0xFF) * 128;

	if (TrInput & IN_LEFT)
		lara->Control.Tightrope.Balance += ANGLE(1.4f);
	if (TrInput & IN_RIGHT)
		lara->Control.Tightrope.Balance -= ANGLE(1.4f);

	if (lara->Control.Tightrope.Balance < 0)
	{
		lara->Control.Tightrope.Balance -= factor;
		if (lara->Control.Tightrope.Balance <= -ANGLE(45.0f))
			lara->Control.Tightrope.Balance = ANGLE(45.0f);

	}
	else if (lara->Control.Tightrope.Balance > 0)
	{
		lara->Control.Tightrope.Balance += factor;
		if (lara->Control.Tightrope.Balance >= ANGLE(45.0f))
			lara->Control.Tightrope.Balance = ANGLE(45.0f);
	}
	else
		lara->Control.Tightrope.Balance = GetRandomControl() & 1 ? -1 : 1;
}

void DoLaraTightropeLean(ITEM_INFO* item)
{
	auto* lara = GetLaraInfo(item);

	item->Position.zRot = lara->Control.Tightrope.Balance / 4;
	lara->ExtraTorsoRot.zRot = -lara->Control.Tightrope.Balance;
}

void DoLaraTightropeBalanceRegen(ITEM_INFO* item)
{
	auto* lara = GetLaraInfo(item);

	if (lara->Control.Tightrope.TimeOnTightrope <= 32)
		lara->Control.Tightrope.TimeOnTightrope = 0;
	else
		lara->Control.Tightrope.TimeOnTightrope -= 32;

	if (lara->Control.Tightrope.Balance > 0)
	{
		if (lara->Control.Tightrope.Balance <= ANGLE(0.75f))
			lara->Control.Tightrope.Balance = 0;
		else
			lara->Control.Tightrope.Balance -= ANGLE(0.75f);
	}

	if (lara->Control.Tightrope.Balance < 0)
	{
		if (lara->Control.Tightrope.Balance >= -ANGLE(0.75f))
			lara->Control.Tightrope.Balance = 0;
		else
			lara->Control.Tightrope.Balance += ANGLE(0.75f);
	}
}

void DoLaraFallDamage(ITEM_INFO* item)
{
	if (item->Animation.VerticalVelocity >= LARA_DAMAGE_VELOCITY)
	{
		if (item->Animation.VerticalVelocity >= LARA_DEATH_VELOCITY)
			item->HitPoints = 0;
		else [[likely]]
		{
			int base = item->Animation.VerticalVelocity - (LARA_DAMAGE_VELOCITY - 1);
			item->HitPoints -= LARA_HEALTH_MAX * (pow(base, 2) / 196);
		}
	}
}

LaraInfo*& GetLaraInfo(ITEM_INFO* item)
{
	if (item->ObjectNumber == ID_LARA)
		return (LaraInfo*&)item->Data;

	TENLog(std::string("Attempted to fetch LaraInfo data from entity with object ID ") + std::to_string(item->ObjectNumber), LogLevel::Warning);

	auto* firstLaraItem = FindItem(ID_LARA);
	return (LaraInfo*&)firstLaraItem->Data;
}

short GetLaraSlideDirection(ITEM_INFO* item, COLL_INFO* coll)
{
	short direction = coll->Setup.ForwardAngle;
	auto probe = GetCollisionResult(item);

	// Ground is flat.
	if (!probe.FloorTilt.x && !probe.FloorTilt.y)
		return direction;

	direction = GetSurfaceAspectAngle(probe.FloorTilt.x, probe.FloorTilt.y);

	// Determine nearest cardinal direction of surface aspect.
	if (!g_GameFlow->Animations.HasSlideExtended)
		direction = GetQuadrant(direction) * ANGLE(90.0f);

	return direction;
}

void ModulateLaraSlideVelocity(ITEM_INFO* item, COLL_INFO* coll)
{
	auto* lara = GetLaraInfo(item);

	auto probe = GetCollisionResult(item);

	/*if (g_GameFlow->Animations.HasSlideExtended)
	{

	}
	else
	{
		constexpr int velocity = 50;
		lara->ExtraVelocity.x += velocity;
	}*/

	// TODO
	constexpr int minVelocity = 50; // Apply only when landing?
	constexpr int maxVelocity = LARA_TERMINAL_VELOCITY;
	constexpr int VelocityIncreasePerStep = 1;

	short steepness = GetSurfaceSteepnessAngle(probe.FloorTilt.x, probe.FloorTilt.y);
	short direction = GetSurfaceAspectAngle(probe.FloorTilt.x, probe.FloorTilt.y);

	//short deltaAngle = abs((short)(direction - item->Position.yRot));
	
	lara->ExtraVelocity.x += minVelocity;
	lara->ExtraVelocity.y += minVelocity * phd_sin(steepness); // TODO: Keeps incrementing to ridiculous values.
}

void SetLaraJumpDirection(ITEM_INFO* item, COLL_INFO* coll)
{
	auto* lara = GetLaraInfo(item);

	if (TrInput & IN_FORWARD &&
		TestLaraJumpForward(item, coll))
	{
		lara->Control.JumpDirection = JumpDirection::Forward;
	}
	else if (TrInput & IN_BACK &&
		TestLaraJumpBack(item, coll))
	{
		lara->Control.JumpDirection = JumpDirection::Back;
	}
	else if (TrInput & IN_LEFT &&
		TestLaraJumpLeft(item, coll))
	{
		lara->Control.JumpDirection = JumpDirection::Left;
	}
	else if (TrInput & IN_RIGHT &&
		TestLaraJumpRight(item, coll))
	{
		lara->Control.JumpDirection = JumpDirection::Right;
	}
	else if (TestLaraJumpUp(item, coll)) [[likely]]
		lara->Control.JumpDirection = JumpDirection::Up;
	else
		lara->Control.JumpDirection = JumpDirection::None;
}

// TODO: Add a timeout? Imagine a small, sad rain cloud with the properties of a ceiling following Lara overhead.
// RunJumpQueued will never reset, and when the sad cloud flies away after an indefinite amount of time, Lara will jump. @Sezz 2022.01.22
void SetLaraRunJumpQueue(ITEM_INFO* item, COLL_INFO* coll)
{
	auto* lara = GetLaraInfo(item);

	int y = item->Position.yPos;
	int distance = SECTOR(1);
	auto probe = GetCollisionResult(item, item->Position.yRot, distance, -coll->Setup.Height);

	if ((TestLaraRunJumpForward(item, coll) ||													// Area close ahead is permissive...
			(probe.Position.Ceiling - y) < -(coll->Setup.Height + (LARA_HEADROOM * 0.8f)) ||		// OR ceiling height is permissive far ahead
			(probe.Position.Floor - y) >= CLICK(0.5f)) &&											// OR there is a drop below far ahead.
		probe.Position.Floor != NO_HEIGHT)
	{
		lara->Control.RunJumpQueued = IsRunJumpQueueableState((LaraState)item->Animation.TargetState);
	}
	else
		lara->Control.RunJumpQueued = false;
}

void SetLaraVault(ITEM_INFO* item, COLL_INFO* coll, VaultTestResult vaultResult)
{
	auto* lara = GetLaraInfo(item);

	lara->ProjectedFloorHeight = vaultResult.Height;
	lara->Control.HandStatus = vaultResult.SetBusyHands ? HandStatus::Busy : lara->Control.HandStatus;
	lara->Control.TurnRate = 0;

	if (vaultResult.SnapToLedge)
	{
		SnapItemToLedge(item, coll, 0.2f, false);
		lara->TargetFacingAngle = coll->NearestLedgeAngle;
	}

	if (vaultResult.SetJumpVelocity)
		lara->Control.CalculatedJumpVelocity = -3 - sqrt(-9600 - 12 * std::max<int>(lara->ProjectedFloorHeight - item->Position.yPos, -CLICK(7.5f)));
}

void SetLaraLand(ITEM_INFO* item, COLL_INFO* coll)
{
	item->Animation.Velocity = 0;
	item->Animation.VerticalVelocity = 0;
	//item->Airborne = false; // TODO: Removing this avoids an unusual landing bug Core had worked around in an obscure way. I hope to find a proper solution. @Sezz 2022.02.18

	LaraSnapToHeight(item, coll);
}

void SetLaraFallAnimation(ITEM_INFO* item)
{
	SetAnimation(item, LA_FALL_START);
	item->Animation.VerticalVelocity = 0;
	item->Animation.Airborne = true;
}

void SetLaraFallBackAnimation(ITEM_INFO* item)
{
	SetAnimation(item, LA_FALL_BACK);
	item->Animation.VerticalVelocity = 0;
	item->Animation.Airborne = true;
}

void SetLaraMonkeyFallAnimation(ITEM_INFO* item)
{
	// HACK: Disallow release during 180 turn action.
	if (item->Animation.ActiveState == LS_MONKEY_TURN_180)
		return;

	SetAnimation(item, LA_MONKEY_TO_FREEFALL);
	SetLaraMonkeyRelease(item);
}

void SetLaraMonkeyRelease(ITEM_INFO* item)
{
	auto* lara = GetLaraInfo(item);

	item->Animation.Velocity = 2;
	item->Animation.VerticalVelocity = 1;
	item->Animation.Airborne = true;
	lara->Control.HandStatus = HandStatus::Free;
}

void SetLaraSlideAnimation(ITEM_INFO* item, COLL_INFO* coll)
{
	short direction = GetLaraSlideDirection(item, coll);
	short deltaAngle = abs((short)(direction - item->Position.yRot));

	// TODO: Take inertia into consideration before switching slide animations.

	// Slide forward.
	if (deltaAngle <= ANGLE(90.0f))
	{
		if (item->Animation.ActiveState == LS_SLIDE_FORWARD && deltaAngle <= ANGLE(90.0f))
			return;

		SetAnimation(item, LA_SLIDE_FORWARD);
	}
	// Slide back.
	else
	{
		if (item->Animation.ActiveState == LS_SLIDE_BACK && deltaAngle > ANGLE(90.0f))
			return;

		SetAnimation(item, LA_SLIDE_BACK_START);
	}
}

void SetLaraCornerAnimation(ITEM_INFO* item, COLL_INFO* coll, bool flip)
{
	auto* lara = GetLaraInfo(item);

	if (item->HitPoints <= 0)
	{
		SetAnimation(item, LA_FALL_START);
		item->Position.yPos += CLICK(1);
		item->Position.yRot += lara->NextCornerPos.yRot / 2;
		item->Animation.Velocity = 2;
		item->Animation.VerticalVelocity = 1;
		item->Animation.Airborne = true;
		lara->Control.HandStatus = HandStatus::Free;
		return;
	}

	if (flip)
	{
		if (lara->Control.IsClimbingLadder)
			SetAnimation(item, LA_LADDER_IDLE);
		else
			SetAnimation(item, LA_HANG_IDLE);

		coll->Setup.OldPosition.x = item->Position.xPos = lara->NextCornerPos.xPos;
		coll->Setup.OldPosition.y = item->Position.yPos = lara->NextCornerPos.yPos;
		coll->Setup.OldPosition.z = item->Position.zPos = lara->NextCornerPos.zPos;
		item->Position.yRot = lara->NextCornerPos.yRot;
	}
}

void ResetLaraLean(ITEM_INFO* item, float rate, bool resetRoll, bool resetPitch)
{
	if (!rate)
	{
		TENLog(std::string("ResetLaraLean() attempted division by zero!"), LogLevel::Warning);
		return;
	}

	rate = abs(rate);

	if (resetPitch)
	{
		if (abs(item->Position.xRot) > ANGLE(0.1f))
			item->Position.xRot += item->Position.xRot / -rate;
		else
			item->Position.xRot = 0;
	}

	if (resetRoll)
	{
		if (abs(item->Position.zRot) > ANGLE(0.1f))
			item->Position.zRot += item->Position.zRot / -rate;
		else
			item->Position.zRot = 0;
	}
}

void ResetLaraFlex(ITEM_INFO* item, float rate)
{
	auto* lara = GetLaraInfo(item);

	if (!rate)
	{
		TENLog(std::string("ResetLaraFlex() attempted division by zero!"), LogLevel::Warning);
		return;
	}

	rate = abs(rate);

	// Reset head.
	if (abs(lara->ExtraHeadRot.xRot) > ANGLE(0.1f))
		lara->ExtraHeadRot.xRot += lara->ExtraHeadRot.xRot / -rate;
	else
		lara->ExtraHeadRot.xRot = 0;

	if (abs(lara->ExtraHeadRot.yRot) > ANGLE(0.1f))
		lara->ExtraHeadRot.yRot += lara->ExtraHeadRot.yRot / -rate;
	else
		lara->ExtraHeadRot.yRot = 0;

	if (abs(lara->ExtraHeadRot.zRot) > ANGLE(0.1f))
		lara->ExtraHeadRot.zRot += lara->ExtraHeadRot.zRot / -rate;
	else
		lara->ExtraHeadRot.zRot = 0;

	// Reset torso.
	if (abs(lara->ExtraTorsoRot.xRot) > ANGLE(0.1f))
		lara->ExtraTorsoRot.xRot += lara->ExtraTorsoRot.xRot / -rate;
	else
		lara->ExtraTorsoRot.xRot = 0;

	if (abs(lara->ExtraTorsoRot.yRot) > ANGLE(0.1f))
		lara->ExtraTorsoRot.yRot += lara->ExtraTorsoRot.yRot / -rate;
	else
		lara->ExtraTorsoRot.yRot = 0;

	if (abs(lara->ExtraTorsoRot.zRot) > ANGLE(0.1f))
		lara->ExtraTorsoRot.zRot += lara->ExtraTorsoRot.zRot / -rate;
	else
		lara->ExtraTorsoRot.zRot = 0;
}
