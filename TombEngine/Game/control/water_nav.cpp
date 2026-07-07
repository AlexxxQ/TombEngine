#include "framework.h"
#include "Game/control/water_nav.h"

#include "Game/collision/collide_room.h"
#include "Game/collision/Point.h"
#include "Game/control/control.h"
#include "Game/control/los.h"
#include "Game/control/lot.h"
#include "Game/itemdata/creature_info.h"
#include "Game/items.h"
#include "Game/Lara/lara.h"
#include "Game/Lara/lara_helpers.h"
#include "Game/room.h"
#include "Specific/level.h"

using namespace TEN::Collision::Point;

bool IsWetBox(const BOX_INFO& box)
{
	return ((box.flags & (BOX_WATER | BOX_SHALLOW)) != 0);
}

bool IsWaterZone(const LOTInfo* LOT)
{
	return (LOT->Zone == ZoneType::Water || LOT->Zone == ZoneType::Amphibious);
}

bool IsSwimmer(const LOTInfo* LOT)
{
	return (LOT->Fly != NO_FLYING && IsWaterZone(LOT));
}

bool IsItemInWaterRoom(const ItemInfo* item)
{
	return TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, item->RoomNumber);
}

bool IsEnemyOnLand(const ItemInfo* enemy)
{
	if (enemy == nullptr)
		return false;

	return enemy->IsLara()
		? (GetLaraInfo(*enemy).Control.WaterStatus == WaterStatus::Dry)
		: !TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, enemy->RoomNumber);
}

// True if two boxes share XZ footprint (vertically stacked). Lets the swimmer breadcrumb
// distinguish a vertical shaft (rise/sink toward the enemy) from a separate deep corridor.
static bool BoxesXZOverlap(int boxA, int boxB)
{
	if (boxA <= NO_VALUE || boxB <= NO_VALUE ||
		boxA >= (int)g_Level.PathfindingBoxes.size() ||
		boxB >= (int)g_Level.PathfindingBoxes.size())
		return false;

	const auto& a = g_Level.PathfindingBoxes[boxA];
	const auto& b = g_Level.PathfindingBoxes[boxB];

	bool xOverlap = (a.top < b.bottom) && (b.top < a.bottom);
	bool zOverlap = (a.left < b.right) && (b.left < a.right);
	return xOverlap && zOverlap;
}

bool ChainHasVerticalTransition(const LOTInfo* LOT, int startBox, int threshold)
{
	if (startBox == NO_VALUE || LOT->TargetBox == NO_VALUE)
		return false;

	constexpr int MAX_HOPS = 32;
	int currBox = startBox;
	int hops = 0;

	while (hops++ < MAX_HOPS)
	{
		if (currBox == LOT->TargetBox)
			return false;

		int nextBox = LOT->Node[currBox].exitBox;
		if (nextBox == NO_VALUE || nextBox == currBox)
			return false;

		if ((LOT->Node[nextBox].searchNumber & SEARCH_NUMBER) != (LOT->SearchNumber & SEARCH_NUMBER))
			return false;

		if (g_Level.PathfindingBoxes[nextBox].flags & LOT->BlockMask)
			return false;

		int currH = g_Level.PathfindingBoxes[currBox].height;
		int nextH = g_Level.PathfindingBoxes[nextBox].height;
		if (std::abs(currH - nextH) > threshold)
			return true;

		currBox = nextBox;
	}
	return false;
}

bool ShouldUseOgWaterYMode(const CreatureInfo* creature, const ItemInfo* item, const LOTInfo* LOT)
{
	if (creature == nullptr || item == nullptr || LOT == nullptr)
		return false;

	if (!IsSwimmer(LOT))
		return false;

	if (creature->Mood != MoodType::Bored)
		return false;

	if (ChainHasVerticalTransition(LOT, item->BoxNumber, CLICK(2)))
		return false;

	return true;
}

int GetWaterCruiseY(ItemInfo* item, LOTInfo* LOT, const BOX_INFO& box)
{
	// Default cruise altitude = 1 block above this box's floor.
	int cruiseY = box.height - BLOCK(1);

	// CRUISE-ALTITUDE RANDOMIZATION: in a stacked water column all boxes share one
	// deep floor, so a fixed cruise glues everyone to the bottom. Roll a random
	// altitude between just below the surface (GetWaterTopHeight) and the default
	// cruise, re-rolled per wander target (needs the item to probe the surface).
	// Amphibious shares this so a bored croc patrols near the surface like in OG.
	if (item != nullptr)
	{
		int waterTop = GetPointCollision(*item).GetWaterTopHeight();
		if (waterTop != NO_HEIGHT)
		{
			// Keep a margin below the surface so the body stays submerged.
			int ceilingY = waterTop + CLICK(2);
			if (ceilingY < cruiseY) // there is vertical room to vary
				cruiseY = ceilingY + (int)((float)GetRandomControl() * (float)(cruiseY - ceilingY) / 32768.0f);
		}

		// SLOPED-CEILING CLAMP: re-probe the lid at the TARGET column and keep a
		// 2-click margin -- in a sealed stack the ceiling can slope below the
		// altitude rolled at the creature's own column.
		int tRoom = FindRoomNumber(Vector3i(LOT->Target.x, item->Pose.Position.y, LOT->Target.z), item->RoomNumber);
		int ceilAtTarget = GetPointCollision(Vector3i(LOT->Target.x, item->Pose.Position.y, LOT->Target.z), tRoom).GetWaterTopHeight();
		if (ceilAtTarget != NO_HEIGHT && cruiseY < ceilAtTarget + CLICK(2))
			cruiseY = ceilAtTarget + CLICK(2);
	}

	return cruiseY;
}

bool GetSwimNavLos(ItemInfo* item, ItemInfo* enemy, const LOTInfo* LOT)
{
	bool swimNav = IsSwimmer(LOT);
	bool inWaterNav = IsWaterZone(LOT) && IsItemInWaterRoom(item);

	if (!(swimNav || inWaterNav) || enemy == nullptr)
		return false;

	auto origin = GameVector(item->Pose.Position, item->RoomNumber);
	auto lookat = GameVector(enemy->Pose.Position, enemy->RoomNumber);
	return LOS(&origin, &lookat);
}

bool TryGetSwimTarget(Vector3i* target, ItemInfo* item, CreatureInfo* creature, ItemInfo* enemy, LOTInfo* LOT, bool ogYMode, bool losToEnemy)
{
	int boxNumber = item->BoxNumber;

	// Divers (Water zone, or Amphibious while submerged): get the breadcrumb's
	// vertical-chain steering ("go to the portal XZ first, then change altitude").
	bool swimNav = IsSwimmer(LOT);

	// Broader in-water predicate for direct homing: also covers surface swimmers
	// (big rat: Amphibious zone but Fly == NO_FLYING, cannot dive).
	bool inWaterNav = IsWaterZone(LOT) && IsItemInWaterRoom(item);

	// CLEAR-LOS DIRECT HOMING (OG): an attacking water creature that can see its
	// in-water enemy swims straight at it -- the box chain only exists to steer
	// around geometry it cannot see past. A land enemy is handled by the
	// exit-connector breadcrumb instead.
	if (inWaterNav && enemy != nullptr && losToEnemy && creature != nullptr &&
		creature->Mood == MoodType::Attack &&
		TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, enemy->RoomNumber))
	{
		target->x = enemy->Pose.Position.x;
		target->y = enemy->Pose.Position.y;
		target->z = enemy->Pose.Position.z;
		return true;
	}

	// BREADCRUMB OVERRIDE for swimmers: corridor clipping cannot express "go to the
	// portal XZ first, then change altitude", so on vertical chains (or blocked LOS)
	// steer at the IMMEDIATE next box's centre with an explicit Y plan instead of
	// homing at LOT->Target. Falls through to the regular loop inside the target box.
	if (swimNav &&
		LOT->TargetBox != NO_VALUE &&
		boxNumber != LOT->TargetBox &&
		!ogYMode)
	{
		// Run the breadcrumb regardless of LOS when the chain crosses a floor/ceiling
		// portal: direct PRIME_TARGET would aim past the portal while the per-tick
		// floor clamp blocks the descent (catch-22 at the portal lip).
		bool chainHasVertical = ChainHasVerticalTransition(LOT, item->BoxNumber, CLICK(2));

		// Amphibious swimmers chasing a DRY-land enemy must also use the breadcrumb:
		// the water exit is a gentle ramp (flat chain), and direct homing beaches them
		// a sector short of the real climb-out connector.
		bool exitingWater = (LOT->Fly != NO_FLYING) && (enemy != nullptr) &&
			!TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, enemy->RoomNumber);

		// With clear LOS home straight at the enemy (OG); fall back to the breadcrumb
		// when sight is blocked, the chain is vertical, or when exiting the water.
		bool useBreadcrumb = (!losToEnemy || chainHasVertical) ||
			(LOT->Zone == ZoneType::Amphibious && exitingWater);

		if (useBreadcrumb)
		{
			int nextBox = LOT->Node[boxNumber].exitBox;

			// EXIT-CONNECTOR re-route: when the next chain box is the dry land box
			// itself (often an edge-touch "phantom" adjacency), steer instead through
			// a neighbour at an intermediate floor that borders BOTH the water and the
			// land box -- the real climb-out sector.
			bool nextIsDryLand = nextBox != NO_VALUE &&
				!IsWetBox(g_Level.PathfindingBoxes[nextBox]);
			if (exitingWater && nextIsDryLand)
			{
				int landBox = nextBox;
				int curH  = g_Level.PathfindingBoxes[boxNumber].height;
				int landH = g_Level.PathfindingBoxes[landBox].height;
				int idx = g_Level.PathfindingBoxes[boxNumber].overlapIndex;
				while (idx >= 0 && idx < (int)g_Level.Overlaps.size())
				{
					const auto& ov = g_Level.Overlaps[idx];
					int nb = ov.box;
					bool swimmable = IsWetBox(g_Level.PathfindingBoxes[nb >= 0 ? nb : boxNumber]);
					bool reachable = nb >= 0 && nb != boxNumber && nb != landBox && swimmable &&
						(ov.flags & OVERLAP_AMPHIBIOUS_TRAVERSABLE) &&
						(LOT->Node[nb].searchNumber & SEARCH_NUMBER) == (LOT->SearchNumber & SEARCH_NUMBER) &&
						!(g_Level.PathfindingBoxes[nb].flags & LOT->BlockMask);
					if (reachable)
					{
						int nbH = g_Level.PathfindingBoxes[nb].height;
						if (nbH > landH && nbH < curH) // transition floor: between land and water
						{
							// Connector must also border the land box (amphibious).
							int j = g_Level.PathfindingBoxes[nb].overlapIndex;
							while (j >= 0 && j < (int)g_Level.Overlaps.size())
							{
								const auto& ov2 = g_Level.Overlaps[j];
								if (ov2.box == landBox && (ov2.flags & OVERLAP_AMPHIBIOUS_TRAVERSABLE))
								{
									nextBox = nb;
									break;
								}
								if (ov2.flags & OVERLAP_END_BIT)
									break;
								j++;
							}
						}
					}
					if (nextBox != landBox) // connector found
						break;
					if (ov.flags & OVERLAP_END_BIT)
						break;
					idx++;
				}
			}

			bool nextOk =
				nextBox != NO_VALUE &&
				nextBox != boxNumber &&
				(LOT->Node[nextBox].searchNumber & SEARCH_NUMBER) == (LOT->SearchNumber & SEARCH_NUMBER) &&
				!(g_Level.PathfindingBoxes[nextBox].flags & LOT->BlockMask);

			if (nextOk)
			{
				constexpr int Y_TRANSITION_THRESHOLD = CLICK(2);
				constexpr int Y_SCAN_MAX_HOPS        = 8;

				auto nextCenter = GetBoxCenter(nextBox);

				// RAMP-EXIT BIAS: a shallow shelf is only mountable from its downhill
				// (open water) edge. Push the steering point past that edge so the
				// creature surfaces there and wades up the slope, instead of jamming
				// under the shore ceiling at a side edge.
				if (exitingWater && (g_Level.PathfindingBoxes[nextBox].flags & BOX_SHALLOW))
				{
					int afterRamp = LOT->Node[nextBox].exitBox;
					bool afterIsDryLand = afterRamp != NO_VALUE && afterRamp != nextBox &&
						!IsWetBox(g_Level.PathfindingBoxes[afterRamp]);
					if (afterIsDryLand)
					{
						auto landCenter = GetBoxCenter(afterRamp);
						float dirX = nextCenter.x - landCenter.x; // downhill = away from land
						float dirZ = nextCenter.z - landCenter.z;
						float len = Vector2(dirX, dirZ).Length();
						if (len > 1.0f)
						{
							float k = (float)BLOCK(0.75f) / len;
							nextCenter.x += dirX * k;
							nextCenter.z += dirZ * k;
						}
					}
				}

				// Default breadcrumb Y = cruise altitude over the next box.
				const int nextHeight = g_Level.PathfindingBoxes[nextBox].height;
				const int currHeight = g_Level.PathfindingBoxes[boxNumber].height;
				float targetY = (float)(nextHeight - BLOCK(1));

				// DEEPEST-BOX DESCENT: on a chain diving through a floor portal, commit
				// to the deepest chain box's altitude -- otherwise the next hop's upper
				// cruise yanks the creature back up before it clears the portal.
				int deepestHeight = currHeight;
				int shallowestHeight = currHeight;
				bool foundDeeper = false;
				bool foundShallower = false;
				int yScanBox = nextBox;
				for (int s = 0; s < Y_SCAN_MAX_HOPS; s++)
				{
					int scanHeight = g_Level.PathfindingBoxes[yScanBox].height;
					if (scanHeight > deepestHeight + Y_TRANSITION_THRESHOLD)
					{
						deepestHeight = scanHeight;
						foundDeeper = true;
					}
					if (scanHeight < shallowestHeight - Y_TRANSITION_THRESHOLD)
					{
						shallowestHeight = scanHeight;
						foundShallower = true;
					}

					if (yScanBox == LOT->TargetBox)
						break;

					int nn = LOT->Node[yScanBox].exitBox;
					if (nn == NO_VALUE || nn == yScanBox)
						break;
					if ((LOT->Node[nn].searchNumber & SEARCH_NUMBER) != (LOT->SearchNumber & SEARCH_NUMBER))
						break;
					if (g_Level.PathfindingBoxes[nn].flags & LOT->BlockMask)
						break;

					yScanBox = nn;
				}

				if (foundDeeper)
				{
					// CORRIDOR-CENTER DIVE: aim at the midpoint between the next box
					// floor and its OWN ceiling plane (no portal traversal!) so the
					// diver swims down the corridor centre instead of skimming the
					// floor. Falls back to the deep commit when there is no ceiling.
					auto nc = GetBoxCenter(nextBox);
					int probeY = nextHeight - CLICK(1);
					short rnDive = (short)FindRoomNumber(Vector3i((int)nc.x, probeY, (int)nc.z), item->RoomNumber);
					int nextSectorCeiling = GetPointCollision(Vector3i((int)nc.x, probeY, (int)nc.z), rnDive)
						.GetSector().GetSurfaceHeight((int)nc.x, (int)nc.z, false);
					if (nextSectorCeiling != NO_HEIGHT)
						targetY = (float)((nextHeight + nextSectorCeiling) / 2);
					else
						targetY = (float)(deepestHeight - CLICK(1));
				}
				else
				{
					// STICKY DESCENT: if last tick's target.y is still far below,
					// hold it until pos.y catches up -- an ascent ahead must not
					// yank the creature back before it clears the portal.
					bool stickyHeld = false;
					if (creature != nullptr)
					{
						int prevTargetY = creature->Target.y;
						if (prevTargetY > (int)item->Pose.Position.y + BLOCK(1))
						{
							targetY = (float)prevTargetY;
							stickyHeld = true;
						}
					}

					// SHAFT Y-TRACKING: stacked water rooms share one deep floor, so
					// box heights cannot signal up/down. When the immediate step is
					// not a descent and the enemy's column overlaps the creature's
					// (BoxesXZOverlap), aim Y at the enemy itself -- the only reliable
					// vertical cue, immune to exitBox flicker. Deep-corridor routes
					// (no XZ overlap) keep the default deep cruise and dive first.
					if (!stickyHeld && foundShallower &&
						enemy != nullptr && creature->Mood != MoodType::Bored &&
						nextHeight <= currHeight &&
						BoxesXZOverlap(boxNumber, LOT->TargetBox))
					{
						targetY = (float)enemy->Pose.Position.y;
					}
				}

				target->x = (int)nextCenter.x;
				target->y = (int)targetY;
				target->z = (int)nextCenter.z;
				return true;
			}
		}
	}

	return false;
}

void SetAmphibiousLOT(CreatureInfo& creature, bool grantSwimStepDrop, int flyRate, int landStep, int landDrop)
{
	if (grantSwimStepDrop)
	{
		creature.LOT.Step = BLOCK(20);
		creature.LOT.Drop = -BLOCK(20);
		creature.LOT.Fly = flyRate;
	}
	else
	{
		creature.LOT.Step = landStep;
		creature.LOT.Drop = landDrop;
		creature.LOT.Fly = NO_FLYING;
	}
}

void TryUnstuckSwimmer(ItemInfo* item, CreatureInfo* creature, LOTInfo* LOT)
{
	if (!IsSwimmer(LOT) || item->BoxNumber == NO_VALUE)
		return;

	int stuckTimeout = 10 * FPS; // 10 seconds. Slow swimmers (shark) can legitimately
	                             // spin in one box for several seconds; 5s caused false triggers.
	bool hasGoal = (LOT->TargetBox != NO_VALUE) && (item->BoxNumber != LOT->TargetBox);

	if (!hasGoal || item->BoxNumber != creature->StuckBox)
	{
		// Entered a new box (progress) or has no goal -- reset the timer.
		creature->StuckBox = item->BoxNumber;
		creature->StuckTimer = 0;
		return;
	}

	creature->StuckTimer++;
	if (creature->StuckTimer < stuckTimeout)
		return;

	int exitBox = LOT->Node[item->BoxNumber].exitBox;

	// ANTI-STRAND: only blacklist exitBox if another valid neighbour exists;
	// a sole forward connection must stay routable (the off-map rescue in
	// CreatureAIInfo covers the worst case).
	bool hasAlternative = false;
	if (exitBox != NO_VALUE && exitBox != item->BoxNumber)
	{
		int* zoneArr = g_Level.Zones[(int)LOT->Zone][(int)FlipStatus].data();
		int curZone = zoneArr[item->BoxNumber];
		const auto& curBox = g_Level.PathfindingBoxes[item->BoxNumber];
		int idx = curBox.overlapIndex;
		while (idx >= 0 && idx < (int)g_Level.Overlaps.size())
		{
			const auto& ov = g_Level.Overlaps[idx];
			int nb = ov.box;
			if (nb != exitBox && nb != item->BoxNumber &&
				nb >= 0 && nb < (int)g_Level.PathfindingBoxes.size() &&
				zoneArr[nb] == curZone &&
				!(g_Level.PathfindingBoxes[nb].flags & LOT->BlockMask) &&
				!IsBoxInCooldown(LOT, nb))
			{
				hasAlternative = true;
				break;
			}
			if (ov.flags & OVERLAP_END_BIT)
				break;
			idx++;
		}

		if (hasAlternative)
			ForceBadBoxCooldown(LOT, exitBox);
	}

	ClearLOT(LOT); // Resets TargetBox + nodes; CalculateTarget refloods around the cooled box.
	creature->StuckBox = item->BoxNumber;
	creature->StuckTimer = 0;
}
