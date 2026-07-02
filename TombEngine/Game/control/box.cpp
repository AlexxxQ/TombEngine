/**
 * @file box.cpp
 * @brief Core AI pathfinding and behavior system for creatures.
 *
 * This file implements the "box" pathfinding system used by all AI creatures in TEN.
 * The system works as follows:
 *
 * BOX SYSTEM OVERVIEW:
 * - The level is divided into rectangular "boxes" (PathfindingBoxes) representing walkable areas.
 * - Each box has a height (floor level) and boundaries (left, right, top, bottom in blocks).
 * - Boxes are connected via "overlaps" which define which boxes can be traversed to from each other.
 * - Overlaps have flags (BOX_JUMP, BOX_MONKEY) indicating special traversal requirements.
 *
 * ZONE SYSTEM:
 * - Zones group boxes by reachability for different creature types.
 * - Two boxes in the same zone are considered connected for that creature type.
 * - Zone types: Basic, Skeleton, Water, Amphibious, Human, Flyer.
 *
 * PATHFINDING (SearchLOT):
 * - Uses breadth-first search through connected boxes.
 * - Starts from target box and expands outward.
 * - Each node stores exitBox (which direction to go to reach target).
 * - Step/Drop limits determine which height differences are traversable.
 *
 * MOOD SYSTEM:
 * - Creatures have moods: Bored, Attack, Escape, Stalk.
 * - Mood determines target selection and movement behavior.
 * - GetCreatureMood() determines mood based on situation.
 * - CreatureMood() acts on the mood to select targets.
 */

#include "framework.h"
#include "Game/control/box.h"

#include <queue>

#include "Game/Animation/Animation.h"
#include "Game/camera.h"
#include "Game/collision/collide_room.h"
#include "Game/collision/Los.h"
#include "Game/collision/Point.h"
#include "Game/control/control.h"
#include "Game/control/lot.h"
#include "Game/effects/smoke.h"
#include "Game/effects/tomb4fx.h"
#include "Game/itemdata/creature_info.h"
#include "Game/Lara/lara.h"
#include "Game/Lara/lara_helpers.h"
#include "Game/items.h"
#include "Game/misc.h"
#include "Game/pickup/pickup.h"
#include "Game/room.h"
#include "Game/Setup.h"
#include "Math/Math.h"
#include "Objects/objectslist.h"
#include "Objects/Generic/Object/Pushable/PushableObject.h"
#include "Renderer/Renderer.h"
#include "Scripting/Include/Flow/ScriptInterfaceFlowHandler.h"
#include "Specific/trutils.h"

using namespace TEN::Collision::Los;

using namespace TEN::Animation;
using namespace TEN::Collision::Point;
using namespace TEN::Collision::Room;
using namespace TEN::Effects::Smoke;
using namespace TEN::Utils;

using PathQueue = std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>>;

// Runtime per-combination zone re-flood helpers (defined further down, used earlier).
static bool IsBoxActiveNow(int box);
static bool IsBoxUsableNow(int box, bool liveEdge);
static bool OverlapActiveFromBox(int box, int overlapFlags);
static int ResolveRuntimeBox(int box);

struct RuntimePathEdge
{
	int box = NO_VALUE;
	int flags = 0;
	bool live = false;
};

static const std::vector<RuntimePathEdge>& GetSeamEdgesForBox(int box);

// AI behavior distance thresholds.
constexpr auto REACHED_GOAL_RADIUS = BLOCK(0.625f);	// Distance at which AI considers goal reached.
constexpr auto ATTACK_RANGE = SQUARE(BLOCK(3));		// Squared distance for attack mode (avoids sqrt).

// Random chance thresholds for mood transitions (out of 0x7FFF).
constexpr auto ESCAPE_CHANCE  = 0x0800; // ~6% chance to escape when hit.
constexpr auto RECOVER_CHANCE = 0x0100; // ~0.8% chance to recover from escape.

// Creature movement constants.
constexpr auto BIFF_AVOID_TURN = ANGLE(11.25f);				// Turn angle to avoid other creatures.
constexpr auto CREATURE_AI_ROTATION_MAX = ANGLE(90.0f);		// Maximum head rotation for guards.
constexpr auto CREATURE_JOINT_ROTATION_MAX = ANGLE(70.0f);	// Maximum joint rotation per frame.
constexpr auto CREATURE_FLY_SMOOTH_FACTOR = 0.1f;			// Smoothing factor for fly/vertical swim velocity.

int PathfindingDisplayIndex = NO_VALUE;

static Vector3 GetVelocity(const ItemInfo& item)
{
	auto nextPose = item.Pose;

	if (item.IsLara())
	{
		const auto& player = GetLaraInfo(item);

		switch (player.Control.WaterStatus)
		{
			case WaterStatus::TreadWater:
				nextPose.Translate(player.Control.MoveAngle, item.Animation.Velocity.y);
				break;

			case WaterStatus::Underwater:
				nextPose.Translate(item.Pose.Orientation, item.Animation.Velocity.y);
				break;

			case WaterStatus::FlyCheat:
				break;

			default:
				nextPose.Translate(player.Control.MoveAngle, item.Animation.Velocity.z, item.Animation.Velocity.y, item.Animation.Velocity.x);
				break;
		}
	}
	else
	{
		nextPose.Translate(item.Pose.Orientation.y, item.Animation.Velocity.z, item.Animation.Velocity.y, item.Animation.Velocity.x);
	}

	return nextPose.Position.ToVector3() - item.Pose.Position.ToVector3();
}

static int GetRandomBox(LOTInfo& LOT)
{
	if (LOT.ZoneCount <= 0 || LOT.Node.empty())
	{
		TENLog("Can't get random box number, LOT data structures are not initialized.", LogLevel::Error);
		return NO_VALUE;
	}

	int index = (int)(Random::GenerateFloat() * LOT.ZoneCount);
	index = std::clamp(index, 0, LOT.ZoneCount - 1);

	return LOT.Node[index].boxNumber;
}

static Vector3 GetBoxCenter(int boxIndex)
{
	if (boxIndex <= NO_VALUE || boxIndex >= g_Level.PathfindingBoxes.size())
		return Vector3::Zero;

	auto& currBox = g_Level.PathfindingBoxes[boxIndex];

	float x = ((float)currBox.left + (float)(currBox.right - currBox.left) / 2.0f) * BLOCK(1);
	auto  y = currBox.height - CLICK(1);
	float z = ((float)currBox.top + (float)(currBox.bottom - currBox.top) / 2.0f) * BLOCK(1);

	return Vector3(z, y, x);
}

static void DrawBox(int boxIndex, const Vector3& color)
{
	if (boxIndex <= NO_VALUE || boxIndex >= g_Level.PathfindingBoxes.size())
		return;

	auto& currBox = g_Level.PathfindingBoxes[boxIndex];

	auto center = GetBoxCenter(boxIndex);
	auto corner = Vector3(currBox.bottom * BLOCK(1), currBox.height + CLICK(1), currBox.right * BLOCK(1));
	auto extents = (corner - center) * 0.9f;
	auto dBox = BoundingOrientedBox(center, extents, Vector4::UnitY);

	for (int i = 0; i <= 10; i++)
	{
		dBox.Extents = extents + Vector3(i);
		DrawDebugBox(dBox, Vector4(color.x, color.y, color.z, 1), RendererDebugPage::PathfindingStats);
	}
}

void DrawLaraPathfinding(int boxIndex)
{
	boxIndex = ResolveRuntimeBox(boxIndex);
	if (boxIndex <= NO_VALUE || boxIndex >= g_Level.PathfindingBoxes.size())
		return;

	auto& currBox = g_Level.PathfindingBoxes[boxIndex];
	auto index = currBox.overlapIndex;

	// Current box color based on flags.
	auto currentBoxColor = Vector3(0.0f, 1.0f, 1.0f);
	if (currBox.flags & BLOCKABLE)
		currentBoxColor = (currBox.flags & BLOCKED) ? Vector3(1.0f, 0.0f, 0.0f) : Vector3(0.0f, 1.0f, 0.0f);

	DrawBox(boxIndex, currentBoxColor);

	if (!IsBoxUsableNow(boxIndex, false))
		return;

	// FLIPMAP-AWARE OVERLAP FILTER.
	//
	// Mirror the runtime BFS filter (CanExpandToBox) so the visualization shows exactly
	// which neighbours are reachable given each box's REAL room flip state (handles
	// independent flip groups, not just the single global FlipStatus).

	// Draw overlapping boxes.
	while (index >= 0)
	{
		if (index >= g_Level.Overlaps.size())
			break;

		auto overlap = g_Level.Overlaps[index];

		int overlapBox = ResolveRuntimeBox(overlap.box);
		if (IsBoxUsableNow(overlapBox, false) && OverlapActiveFromBox(boxIndex, overlap.flags))
			DrawBox(overlapBox, Vector3(1, 1, 0));

		if (overlap.flags & OVERLAP_END_BIT)
			break;
		else
			index++;
	}

	for (auto seamEdge : GetSeamEdgesForBox(boxIndex))
	{
		int seamBox = ResolveRuntimeBox(seamEdge.box);
		if (IsBoxUsableNow(seamBox, true))
			DrawBox(seamBox, Vector3(1, 1, 0));
	}
}

void DrawItemPathfinding(int itemNumber)
{
	constexpr auto MAX_DRAW_STEPS = 100;
	constexpr auto ROOT_NODE_RADIUS = 64.0f;
	constexpr auto INTERMEDIATE_NODE_RADIUS = 32.0f;

	if (itemNumber < 0 || itemNumber >= g_Level.Items.size())
		return;

	auto& item = g_Level.Items[itemNumber];
	if (!item.IsCreature())
		return;

	auto* creature = GetCreatureInfo(&item);
	const auto& LOT = creature->LOT;
	int itemBox = ResolveRuntimeBox(item.BoxNumber);
	int targetBox = ResolveRuntimeBox(LOT.TargetBox);
	int requiredBox = ResolveRuntimeBox(LOT.RequiredBox);

	// Green box: current box (where creature is).
	if (itemBox != NO_VALUE)
		DrawBox(itemBox, Vector3(0, 1, 0));

	// Blue box: TargetBox (pathfinding destination).
	if (targetBox != NO_VALUE)
		DrawBox(targetBox, Vector3(0, 0, 1));

	// Cyan box: RequiredBox (if different from TargetBox).
	if (requiredBox != NO_VALUE && requiredBox != targetBox)
		DrawBox(requiredBox, Vector3(0, 1, 1));

	if (itemBox == NO_VALUE || targetBox == NO_VALUE)
		return;

	// Draw creature's own node.
	auto source = item.Pose.Position.ToVector3();
	source.y -= CLICK(1);
	DrawDebugSphere(source, ROOT_NODE_RADIUS, Vector4::One, RendererDebugPage::PathfindingStats, false);
	DrawDebugString(item.Name, source, Vector4::One, RendererDebugPage::PathfindingStats);

	// If target is not bound to enemy, indicate it.
	auto target = LOT.Target.ToVector3();
	bool drawName = creature->Enemy != nullptr && Vector3i::Distance(creature->Enemy->Pose.Position, LOT.Target) <= BLOCK(1);
	target.y -= ROOT_NODE_RADIUS;

	// Draw target node.
	DrawDebugSphere(target, ROOT_NODE_RADIUS, Vector4::One, RendererDebugPage::PathfindingStats, false);
	DrawDebugString(drawName ? creature->Enemy->Name : "< PENDING >", target, Vector4::One, RendererDebugPage::PathfindingStats);

	// If creature got a penalty for accessing bad box, remember it for further indication.
	int blinkingBox = NO_VALUE;
	for (auto& badBox : LOT.BadBoxes)
	{
		int cooldownLimit = g_GameFlow->GetSettings()->Pathfinding.CollisionPenaltyCooldown * FPS;
		if (badBox.BoxNumber != NO_VALUE && badBox.Count < -(cooldownLimit / 2))
		{
			blinkingBox = badBox.BoxNumber;
			break;
		}
	}

	int currentBox = itemBox;
	int nodeCount = 0;

	auto prevCenter = Vector3::Zero;
	bool hasPrev = false;

	while (currentBox != NO_VALUE && nodeCount++ < MAX_DRAW_STEPS)
	{
		int nextBox = LOT.Node[currentBox].exitBox;
		auto& box = g_Level.PathfindingBoxes[currentBox];
		auto& center = GetBoxCenter(currentBox);
		center.y = std::min(center.y, target.y);

		bool blink = blinkingBox != NO_VALUE;
		auto color = blink ? Vector4(1, 0, 0, 1) : Vector4::One;
		bool bypassPathDrawing = (blink && ((GlobalCounter / 8) % 2 != 0));

		// Red box: intermediate box between current creature position and target.
		if (currentBox != itemBox)
			DrawBox(currentBox, Vector3(1, 0, 0));

		// Draw intermediate box node.
		if (currentBox != requiredBox && currentBox != itemBox)
		{
			DrawDebugSphere(center, INTERMEDIATE_NODE_RADIUS, Vector4::One, RendererDebugPage::PathfindingStats);

			if ((currentBox != blinkingBox) || !bypassPathDrawing)
				DrawDebugString(fmt::format("Box {}", currentBox), center, Vector4::One, RendererDebugPage::PathfindingStats);
		}

		Vector3 lineStart, lineEnd;
		bool drawOnlyDirectLine = false;

		if (currentBox == itemBox && (requiredBox == nextBox || nextBox == NO_VALUE))
		{
			// Direct target-creature line.
			lineStart = source;
			lineEnd = target;
			drawOnlyDirectLine = true;
		}
		else
		{
			if (hasPrev)
			{
				// Draw line from previous box center.
				if (currentBox == LOT.RequiredBox)
				{
					lineStart = prevCenter;
					lineEnd = target;
				}
				else if (currentBox != LOT.TargetBox && nodeCount > 2)
				{
					lineStart = prevCenter;
					lineEnd = center;
				}
			}
			else if (nextBox != NO_VALUE && nextBox != currentBox)
			{
				// Creature line.
				lineStart = source;
				lineEnd = GetBoxCenter(nextBox);
				lineEnd.y = std::min(lineEnd.y, target.y);
			}
		}

		// Draw coarse path.
		if (!bypassPathDrawing)
			DrawDebugLine(lineStart, lineEnd, color, RendererDebugPage::PathfindingStats);

		// Stop if we reached destination or we don't need to draw any more nodes.
		if (currentBox == targetBox || drawOnlyDirectLine)
			break;

		prevCenter = center;
		hasPrev = true;

		if (nextBox == NO_VALUE || nextBox == currentBox)
			break;

		currentBox = nextBox;
	}

	auto mode = g_GameFlow->GetSettings()->Pathfinding.Mode;
	PrintDebugMessage("Pathfinding mode: %s", mode == PathfindingMode::BFS ? "BFS" : (mode == PathfindingMode::Dijkstra ? "Dijkstra" : "A*"));
	PrintDebugMessage("Path node count: %d", nodeCount);
}

// Collect all active creature item numbers.
std::vector<int> GetActiveCreatures()
{
	auto creatures = std::vector<int>{};

	for (int i = 0; i < g_Level.Items.size(); i++)
	{
		auto& item = g_Level.Items[i];
		if (item.IsCreature())
			creatures.push_back(i);
	}

	return creatures;
}

void CyclePathfindingDisplay()
{
	auto creatures = GetActiveCreatures();

	if (creatures.empty())
	{
		// No creatures, stay on Lara view.
		PathfindingDisplayIndex = -1;
		return;
	}

	// Cycle: -1 (Lara) -> 0 -> 1 -> ... -> N-1 -> -1 (Lara).
	PathfindingDisplayIndex++;
	if (PathfindingDisplayIndex >= (int)creatures.size())
		PathfindingDisplayIndex = -1;
}

void DrawPathfindingDebug(int laraBoxIndex)
{
	auto creatures = GetActiveCreatures();

	if (PathfindingDisplayIndex < 0 || creatures.empty())
	{
		// Show Lara's nearby boxes.
		DrawLaraPathfinding(laraBoxIndex);
	}
	else
	{
		// Show selected creature's pathfinding.
		int creatureIndex = PathfindingDisplayIndex;
		if (creatureIndex >= (int)creatures.size())
			creatureIndex = 0;

		DrawItemPathfinding(creatures[creatureIndex]);
	}
}

bool MoveCreature3DPos(Pose* fromPose, Pose* toPose, int velocity, short angleDif, int angleAdd)
{
	auto differenceVector = toPose->Position - fromPose->Position;
	float distance = Vector3::Distance(fromPose->Position.ToVector3(), toPose->Position.ToVector3());

	if (velocity < distance)
		fromPose->Position += differenceVector * (velocity / distance);
	else
		fromPose->Position = toPose->Position;

	if (angleDif <= angleAdd)
	{
		if (angleDif >= -angleAdd)
			fromPose->Orientation.y = toPose->Orientation.y;
		else
			fromPose->Orientation.y -= angleAdd;
	}
	else
		fromPose->Orientation.y += angleAdd;

	if (fromPose->Position == toPose->Position &&
		fromPose->Orientation.y == toPose->Orientation.y)
	{
		return true;
	}

	return false;
}

void CreatureYRot2(Pose* fromPose, short angle, short angleAdd)
{
	if (angle > angleAdd)
	{
		fromPose->Orientation.y += angleAdd;
	}
	else if (angle < -angleAdd)
	{
		fromPose->Orientation.y -= angleAdd;
	}
	else
	{
		fromPose->Orientation.y += angle;
	}
}

bool SameZone(CreatureInfo* creature, ItemInfo* target)
{
	auto& item = g_Level.Items[creature->ItemNumber];
	auto* zone = GetRuntimeZoneTable((int)creature->LOT.Zone).data();

	auto& roomSource = g_Level.Rooms[item.RoomNumber];
	int boxSource = ResolveRuntimeBox(GetSector(&roomSource, item.Pose.Position.x - roomSource.Position.x, item.Pose.Position.z - roomSource.Position.z)->PathfindingBoxID);
	if (boxSource == NO_VALUE)
		return false;
	item.BoxNumber = boxSource;

	auto& roomTarget = g_Level.Rooms[target->RoomNumber];
	int boxTarget = ResolveRuntimeBox(GetSector(&roomTarget, target->Pose.Position.x - roomTarget.Position.x, target->Pose.Position.z - roomTarget.Position.z)->PathfindingBoxID);
	if (boxTarget == NO_VALUE)
		return false;
	target->BoxNumber = boxTarget;

	return (zone[item.BoxNumber] == zone[target->BoxNumber]);
}

short AIGuard(CreatureInfo* creature) 
{
	auto& item = g_Level.Items[creature->ItemNumber];
	if (item.AIBits & MODIFY)
		return 0;

	if (Random::TestProbability(1.0f / 128.0f))
	{
		creature->HeadRight = true;
		creature->HeadLeft = true;
	}
	else if (Random::TestProbability(1.0f / 96.0f))
	{
		creature->HeadRight = false;
		creature->HeadLeft = true;
	}
	else if (Random::TestProbability(1.0f / 64.0f))
	{
		creature->HeadRight = true;
		creature->HeadLeft = false;
	}

	if (creature->HeadLeft && creature->HeadRight)
		return 0;

	if (creature->HeadLeft)
		return -CREATURE_AI_ROTATION_MAX;

	if (creature->HeadRight)
		return CREATURE_AI_ROTATION_MAX;

	return 0;
}

void AlertNearbyGuards(ItemInfo* item) 
{
	for (int i = 0; i < ActiveCreatures.size(); i++)
	{
		auto* currentCreature = GetCreatureInfo(&g_Level.Items[ActiveCreatures[i]]);
		if (currentCreature->ItemNumber == NO_VALUE)
			continue;

		auto* currentTarget = &g_Level.Items[currentCreature->ItemNumber + i];
		if (item->RoomNumber == currentTarget->RoomNumber)
		{
			currentCreature->Alerted = true;
			continue;
		}

		int x = (currentTarget->Pose.Position.x - item->Pose.Position.x) / 64;
		int y = (currentTarget->Pose.Position.y - item->Pose.Position.y) / 64;
		int z = (currentTarget->Pose.Position.z - item->Pose.Position.z) / 64;

		float distance = SQUARE(z) + SQUARE(y) + SQUARE(x);
		if (distance < BLOCK(8))
			currentCreature->Alerted = true;
	}
}

void AlertAllGuards(short itemNumber) 
{
	for (int i = 0; i < ActiveCreatures.size(); i++)
	{
		auto* creature = GetCreatureInfo(&g_Level.Items[ActiveCreatures[i]]);

		if (creature->ItemNumber == NO_VALUE)
			continue;

		auto* target = &g_Level.Items[creature->ItemNumber];
		short objNumber = g_Level.Items[itemNumber].ObjectNumber;
		if (objNumber == target->ObjectNumber)
		{
			if (target->Status == ITEM_ACTIVE)
				creature->Alerted = true;
		}
	}
}

/**
 * @brief Marks a pathfinding box as temporarily invalid ("bad") for the creature.
 *
 * This function records a box that the creature has failed to traverse
 * (e.g. due to collision, invalid height/zone, or clipping against geometry).
 * Bad boxes are used by the pathfinding system to prevent repeatedly attempting
 * the same failing overlap and getting stuck.
 *
 * Every creature can track a limited number of bad boxes. If the limit is reached,
 * no new boxes will be added until some are cleared.
 *
 * @param LOT Pointer to the creature's LOT (pathfinding state).
 * @param boxNumber Pathfinding box to mark as bad. Ignored if NO_VALUE.
 */
static void AddBadBox(LOTInfo* LOT, int boxNumber)
{
	boxNumber = ResolveRuntimeBox(boxNumber);
	if (boxNumber == NO_VALUE)
		return;

	// Don't add bad boxes if penalty system is disabled.
	if (g_GameFlow->GetSettings()->Pathfinding.CollisionPenaltyThreshold <= EPSILON)
		return;
	
	// Check if bad box is already memorized.
	for (auto& badBox : LOT->BadBoxes)
	{
		if (badBox.BoxNumber == boxNumber)
		{
			if (badBox.Count >= 0 && !badBox.Valid)
			{
				badBox.Valid = true;
				badBox.Count++;
			}

			return;
		}
	}

	// Find an empty slot to store the bad box.
	for (auto& badBox : LOT->BadBoxes)
	{
		if (badBox.BoxNumber != NO_VALUE)
			continue;

		badBox.Valid = true;
		badBox.BoxNumber = boxNumber;
		badBox.Count = 1;
		return;
	}
}

/**
* @brief Checks if a pathfinding box is bad and currently in cooldown.
 *
 * @param lot Pointer to the creature's LOT (pathfinding state).
 * @param boxNumber Pathfinding box to check.
 * @return true if the box is in cooldown, false otherwise.
 */
static bool IsBoxInCooldown(const LOTInfo* LOT, int boxNumber)
{
	boxNumber = ResolveRuntimeBox(boxNumber);
	if (boxNumber == NO_VALUE)
		return false;

	for (const auto& badBox : LOT->BadBoxes)
	{
		if (ResolveRuntimeBox(badBox.BoxNumber) == boxNumber && badBox.Count < 0)
			return true;
	}

	return false;
}

/**
 * @brief Updates the creature's bad box memory.
 *
 * Creature's bad box memory mechanism prevents softlocks against
 * problematic geometry while still allowing recovery if creature has
 * already left problematic area.
 *
 * @param item Pointer to the item representing a creature.
 */
static void UpdateBadBoxes(ItemInfo* item)
{
	if (!item->IsCreature())
		return;

	auto& LOT = GetCreatureInfo(item)->LOT;
	int penaltyThreshold = g_GameFlow->GetSettings()->Pathfinding.CollisionPenaltyThreshold * FPS;
	int penaltyCooldown  = g_GameFlow->GetSettings()->Pathfinding.CollisionPenaltyCooldown  * FPS;

	for (auto& badBox : LOT.BadBoxes)
	{
		if (badBox.BoxNumber == NO_VALUE)
			continue;

		if (badBox.Count > 0)
		{
			if (badBox.Count >= penaltyThreshold)
			{
				// If penalty has built up, flip into cooldown exactly at limit.
				badBox.Count = -penaltyCooldown;
				LOT.TargetBox = NO_VALUE;
				ClearLOT(&LOT);
				return;
			}
			else if (!badBox.Valid)
			{
				// If box wasn't queried for current loop, reduce penalty count.
				badBox.Count--;
			}
		}
		else if (badBox.Count < 0)
		{
			// Cooldown the bad box.
			badBox.Count++;
		}

		// Forget the bad box, if timeout has elapsed.
		if (badBox.Count == 0)
			badBox.BoxNumber = NO_VALUE;

		// Invalidate the bad box until the next query.
		badBox.Valid = false;
	}
}

/**
 * @brief Handles creature movement, collision, and positioning after animation.
 *
 * This function is the core of creature physical movement. After animation updates
 * the creature's logical position, this function:
 * 1. Validates the move against pathfinding constraints (zone, step, drop).
 * 2. Handles collision with box boundaries and walls.
 * 3. Handles creature-creature collision avoidance.
 * 4. Handles vertical movement (flying/swimming or ground following).
 * 5. Updates room number and floor height.
 *
 * @param item The creature item to move.
 * @param prevPos The creature's position before animation (for rollback).
 * @param angle Heading angle change from CreatureTurn.
 * @param tilt Tilt angle for banking during turns.
 * @return true if movement succeeded, false if creature is stuck.
 */
bool CreaturePathfind(ItemInfo* item, Vector3i prevPos, short angle, short tilt)
{
	int xPos, zPos, ceiling, shiftX, shiftZ;
	short top;

	auto* creature = GetCreatureInfo(item);
	auto* LOT = &creature->LOT;
	const int* zone = GetRuntimeZoneTable((int)LOT->Zone).data();

	// Get height of creature's current box (for step/drop checks).
	int boxHeight;
	if (item->BoxNumber != NO_VALUE)
		boxHeight = g_Level.PathfindingBoxes[item->BoxNumber].height;
	else
		boxHeight = item->Floor;

	auto bounds = GameBoundingBox(item);
	int y = item->Pose.Position.y;

	// Use land creature's top (Y1) for ceiling collision checks.
	// For water creatures, it is ignored because water surface clamp may freeze them.
	if (LOT->Zone != ZoneType::Water)
		y += bounds.Y1;

	short roomNumber = item->RoomNumber;

	// Get floor info at new position.
	GetFloor(prevPos.x, y, prevPos.z, &roomNumber);
	auto* floor = GetFloor(item->Pose.Position.x, y, item->Pose.Position.z, &roomNumber);
	if (floor->PathfindingBoxID == NO_VALUE)
		return false;

	int floorBox = ResolveRuntimeBox(floor->PathfindingBoxID);
	int height = g_Level.PathfindingBoxes[floorBox].height;

	// PORTAL-Y RETRY (movement, active-guarded): at a vertical portal the head-Y floor probe
	// can land on the wrong box -- a far phantom box, or a box that is INACTIVE in the current
	// flip combination (e.g. a base box when the creature's own room is flipped). Scan probe Y
	// across the full Drop..Step vertical range and adopt the first box that is within Step/Drop
	// AND ACTIVE (runtime zone != 0). The active guard is what makes this safe across
	// independent flip groups: it never locks the creature onto an inactive box, while still
	// stabilising the floor/box at the seam so the route isn't dropped mid-climb. Ground only.
	if (LOT->Zone != ZoneType::Water && LOT->Fly == NO_FLYING && item->BoxNumber != NO_VALUE)
	{
		int fb = floorBox;
		int dh1 = boxHeight - height;
		bool firstBad = (dh1 > LOT->Step) || (dh1 < LOT->Drop) || (zone[fb] == 0);
		if (firstBad)
		{
			for (int dY = LOT->Drop; dY <= LOT->Step; dY += CLICK(1))
			{
				short rn2 = item->RoomNumber;
				auto* f = GetFloor(item->Pose.Position.x, boxHeight - dY, item->Pose.Position.z, &rn2);
				if (f == nullptr || f->PathfindingBoxID == NO_VALUE)
					continue;
				int bx = ResolveRuntimeBox(f->PathfindingBoxID);
				int dh2 = boxHeight - g_Level.PathfindingBoxes[bx].height;
				if (dh2 <= LOT->Step && dh2 >= LOT->Drop && zone[bx] != 0)
				{
					floor = f;
					floorBox = bx;
					roomNumber = rn2;
					height = g_Level.PathfindingBoxes[bx].height;
					break;
				}
			}
		}
	}

	int nextHeight = 0;

	// Get the next box on the path (for nonLot creatures, just use current box).
	int nextBox;
	if (!Objects[item->ObjectNumber].nonLot)
	{
		nextBox = LOT->Node[floorBox].exitBox;
	}
	else
	{
		// NonLot creatures don't use pathfinding, just collision.
		floor = GetFloor(item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z, &roomNumber);
		floorBox = ResolveRuntimeBox(floor->PathfindingBoxID);
		height = g_Level.PathfindingBoxes[floorBox].height;
		nextBox = floorBox;
	}

	if (nextBox == NO_VALUE)
		nextHeight = height;
	else
		nextHeight = g_Level.PathfindingBoxes[nextBox].height;

	bool heightThresholdReached = LOT->Fly == NO_FLYING && !LOT->IsJumping && (boxHeight - height > LOT->Step || boxHeight - height < LOT->Drop);
	bool zoneIncorrect = item->BoxNumber != NO_VALUE && !LOT->IsJumping && LOT->Zone != ZoneType::Flyer && (zone[ResolveRuntimeBox(item->BoxNumber)] != zone[floorBox]);

	// ZONE/STEP/DROP VALIDATION:
	// If creature moved to invalid floor, push back to sector boundary.

	if (floor->PathfindingBoxID == NO_VALUE || heightThresholdReached || zoneIncorrect)
	{
		if (heightThresholdReached)
			AddBadBox(LOT, floorBox);

		// Calculate which sector boundary to push creature to.
		xPos = item->Pose.Position.x / BLOCK(1);
		zPos = item->Pose.Position.z / BLOCK(1);
		shiftX = prevPos.x / BLOCK(1);
		shiftZ = prevPos.z / BLOCK(1);

		// Push to sector edge based on movement direction.
		if (xPos < shiftX)
			item->Pose.Position.x = prevPos.x & (~WALL_MASK);
		else if (xPos > shiftX)
			item->Pose.Position.x = prevPos.x | WALL_MASK;

		if (zPos < shiftZ)
			item->Pose.Position.z = prevPos.z & (~WALL_MASK);
		else if (zPos > shiftZ)
			item->Pose.Position.z = prevPos.z | WALL_MASK;

		// Re-get floor info at corrected position.
		floor = GetFloor(item->Pose.Position.x, y, item->Pose.Position.z, &roomNumber);

		if (floor->PathfindingBoxID != NO_VALUE)
		{
			floorBox = ResolveRuntimeBox(floor->PathfindingBoxID);
			height = g_Level.PathfindingBoxes[floorBox].height;
			if (!Objects[item->ObjectNumber].nonLot)
			{
				nextBox = LOT->Node[floorBox].exitBox;
			}
			else
			{
				floor = GetFloor(item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z, &roomNumber);
				floorBox = ResolveRuntimeBox(floor->PathfindingBoxID);
				height = g_Level.PathfindingBoxes[floorBox].height;
				nextBox = floorBox;
			}
		}

		if (nextBox == NO_VALUE)
			nextHeight = height;
		else
			nextHeight = g_Level.PathfindingBoxes[nextBox].height;
	}

	// RADIUS COLLISION:
	// Check if creature's radius would overlap with bad floor at sector edges.
	// This prevents creatures from walking off edges or into walls.

	int x = item->Pose.Position.x;
	int z = item->Pose.Position.z;
	xPos = x & WALL_MASK; // Position within sector (0-1023).
	zPos = z & WALL_MASK;
	short radius = Objects[item->ObjectNumber].radius;
	shiftX = 0;
	shiftZ = 0;

	// Check each sector edge based on creature's position + radius.
	if (zPos < radius)
	{
		if (BadFloor(x, y, z - radius, height, nextHeight, roomNumber, LOT))
			shiftZ = radius - zPos;

		if (xPos < radius)
		{
			if (BadFloor(x - radius, y, z, height, nextHeight, roomNumber, LOT))
				shiftX = radius - xPos;
			else if (!shiftZ && BadFloor(x - radius, y, z - radius, height, nextHeight, roomNumber, LOT))
			{
				if (item->Pose.Orientation.y > -ANGLE(135.0f) && item->Pose.Orientation.y < ANGLE(45.0f))
					shiftZ = radius - zPos;
				else
					shiftX = radius - xPos;
			}
		}
		else if (xPos > BLOCK(1) - radius)
		{
			if (BadFloor(x + radius, y, z, height, nextHeight, roomNumber, LOT))
				shiftX = BLOCK(1) - radius - xPos;
			else if (!shiftZ && BadFloor(x + radius, y, z - radius, height, nextHeight, roomNumber, LOT))
			{
				if (item->Pose.Orientation.y > -ANGLE(45.0f) && item->Pose.Orientation.y < ANGLE(135.0f))
					shiftZ = radius - zPos;
				else
					shiftX = BLOCK(1) - radius - xPos;
			}
		}
	}
	else if (zPos > BLOCK(1) - radius)
	{
		if (BadFloor(x, y, z + radius, height, nextHeight, roomNumber, LOT))
			shiftZ = BLOCK(1) - radius - zPos;

		if (xPos < radius)
		{
			if (BadFloor(x - radius, y, z, height, nextHeight, roomNumber, LOT))
				shiftX = radius - xPos;
			else if (!shiftZ && BadFloor(x - radius, y, z + radius, height, nextHeight, roomNumber, LOT))
			{
				if (item->Pose.Orientation.y > -ANGLE(45.0f) && item->Pose.Orientation.y < ANGLE(135.0f))
					shiftX = radius - xPos;
				else
					shiftZ = BLOCK(1) - radius - zPos;
			}
		}
		else if (xPos > BLOCK(1) - radius)
		{
			if (BadFloor(x + radius, y, z, height, nextHeight, roomNumber, LOT))
				shiftX = BLOCK(1) - radius - xPos;
			else if (!shiftZ && BadFloor(x + radius, y, z + radius, height, nextHeight, roomNumber, LOT))
			{
				if (item->Pose.Orientation.y > -ANGLE(135.0f) && item->Pose.Orientation.y < ANGLE(45.0f))
					shiftX = BLOCK(1) - radius - xPos;
				else
					shiftZ = BLOCK(1) - radius - zPos;
			}
		}
	}
	else if (xPos < radius)
	{
		if (BadFloor(x - radius, y, z, height, nextHeight, roomNumber, LOT))
			shiftX = radius - xPos;
	}
	else if (xPos > BLOCK(1) - radius)
	{
		if (BadFloor(x + radius, y, z, height, nextHeight, roomNumber, LOT))
			shiftX = BLOCK(1) - radius - xPos;
	}

	// Apply the collision shift.
	item->Pose.Position.x += shiftX;
	item->Pose.Position.z += shiftZ;

	// If we shifted, update floor and apply turn/tilt.
	if (shiftX || shiftZ)
	{
		floor = GetFloor(item->Pose.Position.x, y, item->Pose.Position.z, &roomNumber);
		item->Pose.Orientation.y += angle;

		if (tilt)
			CreatureTilt(item, (tilt * 2));
	}

	// CREATURE-CREATURE COLLISION:
	// Check for collision with other creatures and turn to avoid.

	short biffAngle;
	if (item->ObjectNumber != ID_TYRANNOSAUR && item->Animation.Velocity.z && item->HitPoints > 0)
		biffAngle = CreatureCreature(item->Index);
	else
		biffAngle = 0;

	// If colliding with another creature, turn away.
	if (biffAngle)
	{
		if (abs(biffAngle) < BIFF_AVOID_TURN)
			item->Pose.Orientation.y -= BIFF_AVOID_TURN;
		else if (biffAngle > 0)
			item->Pose.Orientation.y -= BIFF_AVOID_TURN;
		else
			item->Pose.Orientation.y += BIFF_AVOID_TURN;

		return true;
	}

	// VERTICAL MOVEMENT:
	// Three modes: Flying/Swimming, Jumping, or Ground.

	if (LOT->Fly != NO_FLYING && item->HitPoints > 0)
	{
		// FLYING/SWIMMING: Move toward target Y at fly speed.
		int deltaY = creature->Target.y - item->Pose.Position.y;
		int flyRate = deltaY;

		// Ease-out fly rate when close to target.
		if (g_GameFlow->GetSettings()->Pathfinding.VerticalMovementSmoothing)
			flyRate *= CREATURE_FLY_SMOOTH_FACTOR;

		// Don't exceed maximum fly rate.
		flyRate = std::clamp(flyRate, -LOT->Fly, LOT->Fly);

		// New TEN behaviour: if next passable box's height is higher than current
		// creature Y position, force upward movement to overcome it. Original pathfinder
		// would just stuck in such cases.

		if (g_GameFlow->GetSettings()->Pathfinding.VerticalGeometryAvoidance && item->BoxNumber != NO_VALUE)
		{
			int currentBox = ResolveRuntimeBox(item->BoxNumber);
			int nextBox = creature->LOT.Node[currentBox].exitBox;

			if (nextBox != NO_VALUE && nextBox != currentBox)
			{
				if (g_Level.PathfindingBoxes[nextBox].height < item->Pose.Position.y)
					flyRate = -LOT->Fly;
			}
		}

		height = GetFloorHeight(floor, item->Pose.Position.x, y, item->Pose.Position.z);
		if (item->Pose.Position.y + flyRate <= height)
		{
			ceiling = GetCeiling(floor, item->Pose.Position.x, y, item->Pose.Position.z);

			// Was hardcoded to CLICK(0.5f) for whale/shark enemies, but now it's redundant
			// because we do water surface height check down the line.
			top = bounds.Y1;

			int topPos = item->Pose.Position.y + top;

			// Check ceiling collision when swimming up.
			if (topPos + flyRate < ceiling)
			{
				if (topPos < ceiling)
				{
					// Already stuck in ceiling - push back and swim down.
					item->Pose.Position = prevPos;
					flyRate = LOT->Fly;
				}
				else
					creature->FlyRate = flyRate = 0;
			}

			if (g_GameFlow->GetSettings()->Pathfinding.WaterSurfaceAvoidance)
			{
				if (LOT->Zone == ZoneType::Water)
				{
					// New TEN behaviour: clamp water creatures below water level.
					int waterHeight = GetPointCollision(*item).GetWaterSurfaceHeight();
					if (topPos + flyRate <= waterHeight)
					{
						item->Pose.Position.y = prevPos.y;
						creature->FlyRate = flyRate = std::max(0, flyRate);
					}
				}
				else if (LOT->Zone == ZoneType::Flyer)
				{
					// New TEN behaviour: clamp flying creatures above water level.
					int waterHeight = GetPointCollision(*item).GetWaterSurfaceHeight();
					if (waterHeight != NO_HEIGHT)
					{
						int bottomPos = item->Pose.Position.y + bounds.Y2;

						// If creature's bottom would enter water, prevent it.
						if (bottomPos + flyRate >= waterHeight)
						{
							item->Pose.Position.y = prevPos.y;
							creature->FlyRate = flyRate = std::min(flyRate, 0);
						}
					}
				}
			}
		}
		else if (item->Pose.Position.y <= height)
		{
			// At floor level - stop vertical movement.
			item->Pose.Position.y = height;
			creature->FlyRate = flyRate = 0;
		}
		else
		{
			// Below floor - push back and go up.
			item->Pose.Position.x = prevPos.x;
			item->Pose.Position.z = prevPos.z;
			creature->FlyRate = flyRate = -LOT->Fly;
			AddBadBox(LOT, floor->PathfindingBoxID);
		}

		if (g_GameFlow->GetSettings()->Pathfinding.VerticalMovementSmoothing)
		{
			// Ease-in fly rate when necessary.
			if (creature->FlyRate < flyRate)
				creature->FlyRate = std::min(creature->FlyRate + (int)(LOT->Fly * CREATURE_FLY_SMOOTH_FACTOR), flyRate);
			else if (creature->FlyRate > flyRate)
				creature->FlyRate = std::max(creature->FlyRate - (int)(LOT->Fly * CREATURE_FLY_SMOOTH_FACTOR), flyRate);
		}
		else
		{
			creature->FlyRate = flyRate;
		}

		item->Pose.Position.y += creature->FlyRate;

		floor = GetFloor(item->Pose.Position.x, y, item->Pose.Position.z, &roomNumber);
		item->Floor = GetFloorHeight(floor, item->Pose.Position.x, y, item->Pose.Position.z);

		// Pitch creature based on vertical movement (nose up/down when climbing/diving).
		angle = (item->Animation.Velocity.z) ? phd_atan(item->Animation.Velocity.z, -creature->FlyRate) : 0;
		if (angle < -ANGLE(20.0f))
			angle = -ANGLE(20.0f);
		else if (angle > ANGLE(20.0f))
			angle = ANGLE(20.0f);

		// Smooth pitch transition.
		if (angle < item->Pose.Orientation.x - ANGLE(1.0f))
			item->Pose.Orientation.x -= ANGLE(1.0f);
		else if (angle > item->Pose.Orientation.x + ANGLE(1.0f))
			item->Pose.Orientation.x += ANGLE(1.0f);
		else
			item->Pose.Orientation.x = angle;
	}
	else if (LOT->IsJumping)
	{
		// JUMPING/MONKEYSWING: Special traversal in progress.
		floor = GetFloor(item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z, &roomNumber);
		int height2 = GetFloorHeight(floor, item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z);
		item->Floor = height2;

		if (LOT->IsMonkeying)
		{
			// Monkeyswing: stick to ceiling.
			ceiling = GetCeiling(floor, item->Pose.Position.x, y, item->Pose.Position.z);
			item->Pose.Position.y = ceiling - bounds.Y1;
		}
		else
		{
			// Jumping: land on floor or rollback if too far below.
			if (item->Pose.Position.y > item->Floor)
			{
				if (item->Pose.Position.y > (item->Floor + CLICK(1)))
					item->Pose.Position = prevPos;
				else
					item->Pose.Position.y = item->Floor;
			}
		}
	}
	else
	{
		// GROUND MOVEMENT: Follow floor height.
		floor = GetFloor(item->Pose.Position.x, y, item->Pose.Position.z, &roomNumber);
		ceiling = GetCeiling(floor, item->Pose.Position.x, y, item->Pose.Position.z);

		// Large creatures need special collision height.
		if (item->ObjectNumber == ID_TYRANNOSAUR || item->ObjectNumber == ID_SHIVA || item->ObjectNumber == ID_MUTANT2)
			top = CLICK(3);
		else
			top = bounds.Y1; // TODO: check if Y1 or Y2

		// Ceiling collision - push back if head would hit ceiling.
		if (item->Pose.Position.y + top < ceiling)
		{
			item->Pose.Position = prevPos;
			AddBadBox(LOT, floor->PathfindingBoxID);
		}

		floor = GetFloor(item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z, &roomNumber);
		item->Floor = GetFloorHeight(floor, item->Pose.Position.x, item->Pose.Position.y, item->Pose.Position.z);

		// Snap to floor or smoothly descend.
		if (item->Pose.Position.y > item->Floor)
			item->Pose.Position.y = item->Floor;
		else if (item->Floor - item->Pose.Position.y > CLICK(0.25f))
			item->Pose.Position.y += CLICK(0.25f);
		else if (item->Pose.Position.y < item->Floor)
			item->Pose.Position.y = item->Floor;

		// Ground creatures don't pitch.
		item->Pose.Orientation.x = 0;
	}

	UpdateItemRoom(item->Index);

	return true;
}

void CreatureKill(ItemInfo* creatureItem, int creatureAnimNumber, int playerExtraAnimNumber, int creatureState, int playerKillState)
{
	if (!Objects[ID_LARA_EXTRA_ANIMS].loaded ||
		Objects[ID_LARA_EXTRA_ANIMS].Animations.size() <= playerExtraAnimNumber || Objects[ID_LARA_EXTRA_ANIMS].Animations[playerExtraAnimNumber].Frames.size() <= 1 ||
		Objects[creatureItem->ObjectNumber].Animations.size() <= creatureAnimNumber || Objects[creatureItem->ObjectNumber].Animations[creatureAnimNumber].Frames.size() <= 1)
	{
		TENLog(fmt::format("Impossible to perform kill animation for object {}: animation data missing.", GetObjectName(creatureItem->ObjectNumber)), LogLevel::Warning);
		return;
	}

	auto& playerItem = *LaraItem;
	auto& player = GetLaraInfo(playerItem);

	SetAnimation(creatureItem, creatureAnimNumber);
	SetAnimationFromSlot(playerItem, ID_LARA_EXTRA_ANIMS, playerExtraAnimNumber, 0, GetSystemBlendDuration());

	if (creatureState != NO_VALUE)
		creatureItem->Animation.ActiveState = creatureItem->Animation.TargetState = creatureState;

	if (playerKillState != NO_VALUE)
		playerItem.Animation.ActiveState = playerItem.Animation.TargetState = playerKillState;

	playerItem.Pose = creatureItem->Pose;
	playerItem.Animation.IsAirborne = false;
	playerItem.Animation.Velocity = Vector3::Zero;

	if (creatureItem->RoomNumber != playerItem.RoomNumber)
		ItemNewRoom(playerItem.Index, creatureItem->RoomNumber);

	AnimateItem(playerItem);
	playerItem.HitPoints = -1;
	player.Control.HandStatus = HandStatus::Busy;
	player.Control.Weapon.GunType = LaraWeaponType::None;
	player.ExtraAnim = 1;
	player.HitDirection = -1;

	Camera.pos.RoomNumber = playerItem.RoomNumber;
	Camera.flags = CF_FOLLOW_CENTER;
	Camera.targetAngle = ANGLE(170.0f);
	Camera.targetElevation = ANGLE(-25.0f);
	Camera.targetDistance = BLOCK(2);
}

short CreatureEffect2(ItemInfo* item, const CreatureBiteInfo& bite, short velocity, short angle, std::function<CreatureEffectFunction> func)
{
	auto pos = GetJointPosition(item, bite);
	return func(pos.x, pos.y, pos.z, velocity, angle, item->RoomNumber);
}

short CreatureEffect(ItemInfo* item, const CreatureBiteInfo& bite, std::function<CreatureEffectFunction> func)
{
	auto pos = GetJointPosition(item, bite);
	return func(pos.x, pos.y, pos.z, item->Animation.Velocity.z, item->Pose.Orientation.y, item->RoomNumber);
}

void CreatureUnderwater(ItemInfo* item, int depth)
{
	int waterLevel = depth;
	int waterHeight = 0;

	if (depth < 0)
	{
		waterHeight = abs(depth);
		waterLevel = 0;
	}
	else
	{
		waterHeight = GetPointCollision(*item).GetWaterTopHeight();
	}

	int y = waterHeight + waterLevel;

	if (item->Pose.Position.y < y)
	{
		int height = GetPointCollision(*item).GetFloorHeight();

		item->Pose.Position.y = y;
		if (y > height)
			item->Pose.Position.y = height;

		if (item->Pose.Orientation.x > ANGLE(2.0f))
		{
			item->Pose.Orientation.x -= ANGLE(2.0f);
		}
		else if (item->Pose.Orientation.x > 0)
		{
			item->Pose.Orientation.x = 0;
		}
	}
}

void CreatureFloat(short itemNumber) 
{
	auto* item = &g_Level.Items[itemNumber];

	auto pointColl = GetPointCollision(*item);

	item->HitPoints = NOT_TARGETABLE;
	item->Pose.Orientation.x = 0;

	int y = item->Pose.Position.y;
	int waterLevel = GetPointCollision(*item).GetWaterTopHeight();
	if (waterLevel == NO_HEIGHT)
		return;

	if (y > waterLevel)
		item->Pose.Position.y = y - 32;

	if (item->Pose.Position.y < waterLevel)
		item->Pose.Position.y = waterLevel;

	AnimateItem(item);

	item->Floor = pointColl.GetFloorHeight();
	if (pointColl.GetRoomNumber() != item->RoomNumber)
		ItemNewRoom(itemNumber, pointColl.GetRoomNumber());

	if (item->Pose.Position.y <= waterLevel)
	{
		if (item->Animation.FrameNumber == 0)
		{
			item->Pose.Position.y = waterLevel;
			item->Collidable = false;
			item->Status = ITEM_DEACTIVATED;
			DisableEntityAI(itemNumber);
			RemoveActiveItem(itemNumber);
			item->AfterDeath = 1;
		}
	}
}

void CreatureJoint(ItemInfo* item, short joint, short required, short maxAngle)
{
	if (!item->IsCreature())
		return;

	auto* creature = GetCreatureInfo(item);

	short change = required - creature->JointRotation[joint];
	if (change > ANGLE(3.0f))
		change = ANGLE(3.0f);
	else if (change < ANGLE(-3.0f))
		change = ANGLE(-3.0f);

	creature->JointRotation[joint] += change;
	if (creature->JointRotation[joint] > maxAngle)
		creature->JointRotation[joint] = maxAngle;
	else if (creature->JointRotation[joint] < -maxAngle)
		creature->JointRotation[joint] = -maxAngle;
}

void CreatureTilt(ItemInfo* item, short angle) 
{
	angle = (angle << 2) - item->Pose.Orientation.z;

	if (angle < -ANGLE(3.0f))
		angle = -ANGLE(3.0f);
	else if (angle > ANGLE(3.0f))
		angle = ANGLE(3.0f);

	short absRot = abs(item->Pose.Orientation.z);
	if (absRot < ANGLE(15.0f) || absRot > ANGLE(30.0f))
		angle >>= 1;
	
	item->Pose.Orientation.z += angle;
}

short CreatureTurn(ItemInfo* item, short maxTurn)
{
	constexpr auto FEELER_ANGLE = ANGLE(45);

	if (!item->IsCreature() || maxTurn == 0)
		return 0;

	auto* creature = GetCreatureInfo(item);

	if (g_GameFlow->GetSettings()->Pathfinding.MoveableAvoidance ||
		g_GameFlow->GetSettings()->Pathfinding.StaticMeshAvoidance)
	{
		auto HasObstacle = [&](const LosCollisionData& los)
		{
			// Don't check for object obstacles near room geometry to avoid being stuck.
			if (los.Room.IsIntersected)
				return false;

			if (g_GameFlow->GetSettings()->Pathfinding.MoveableAvoidance && !los.Items.empty())
			{
				for (const auto& entry : los.Items)
				{
					if (entry.IsOriginContained)
						continue;

					if (entry.Item->Index != item->Index && !entry.Item->IsCreature() && !entry.Item->IsLara())
						return true;
				}
			}

			if (g_GameFlow->GetSettings()->Pathfinding.StaticMeshAvoidance && !los.Statics.empty())
			{
				for (const auto& entry : los.Statics)
				{
					if (entry.IsOriginContained)
						continue;

					return true;
				}
			}

			return false;
		};

		auto leftAngle = item->Pose.Orientation + EulerAngles(0, FEELER_ANGLE, 0);
		auto rightAngle = item->Pose.Orientation - EulerAngles(0, FEELER_ANGLE, 0);
		auto feelerPos = item->Pose.Position.ToVector3() + Vector3(0, -CLICK(1), 0);
		auto radius = GetFrame(*item).LocalAabb.Extents.z * 1.3f; // Increase the radius slightly.

		// Spawn feelers for object collision.
		auto feelMidLos = GetLosCollision(feelerPos, item->RoomNumber, item->Pose.Orientation.ToDirection(), radius, true, false, true);
		auto feelLeftLos = GetLosCollision(feelerPos, item->RoomNumber, leftAngle.ToDirection(), radius, true, false, true);
		auto feelRightLos = GetLosCollision(feelerPos, item->RoomNumber, rightAngle.ToDirection(), radius, true, false, true);

		// Test LOS results.
		bool feelMidResult = HasObstacle(feelMidLos);
		bool feelLeftResult = HasObstacle(feelLeftLos);
		bool feelRightResult = HasObstacle(feelRightLos);

		if (feelLeftResult && feelMidResult)
		{
			// Obstacle on left side - turn right.
			auto feelRightPos = Geometry::TranslatePoint(item->Pose.Position.ToVector3(), rightAngle, radius);
			creature->Target.x = feelRightPos.x;
			creature->Target.z = feelRightPos.z;
		}
		else if (feelRightResult && feelMidResult)
		{
			// Obstacle on right side - turn left.
			auto feelLeftPos = Geometry::TranslatePoint(item->Pose.Position.ToVector3(), leftAngle, radius);
			creature->Target.x = feelLeftPos.x;
			creature->Target.z = feelLeftPos.z;
		}
		else if (feelLeftResult || feelRightResult)
		{
			// Obstacle on the left/right edge, just steer the creature forward.
			auto feelMidPos = Geometry::TranslatePoint(item->Pose.Position.ToVector3(), item->Pose.Orientation, radius);
			creature->Target.x = feelMidPos.x;
			creature->Target.z = feelMidPos.z;
		}
	}

	// Perform actual turn according to target position.
	int x = creature->Target.x - item->Pose.Position.x;
	int z = creature->Target.z - item->Pose.Position.z;

	short angle = phd_atan(z, x) - item->Pose.Orientation.y;

	int range = (item->Animation.Velocity.z * BLOCK(16)) / maxTurn;
	int distance = Vector2(x, z).Length();

	if ((angle > FRONT_ARC || angle < -FRONT_ARC) && distance < range)
		maxTurn /= 2;

	angle = std::clamp(angle, (short)(-maxTurn), maxTurn);
	item->Pose.Orientation.y += angle;

	return angle;
}

bool CreatureAnimation(short itemNumber, short headingAngle, short tiltAngle)
{
	auto& item = g_Level.Items[itemNumber];
	if (!item.IsCreature())
		return false;

	auto& creature = *GetCreatureInfo(&item);

	SpawnCreatureGunEffect(item, creature.MuzzleFlash[0]);
	SpawnCreatureGunEffect(item, creature.MuzzleFlash[1]);

	auto prevPos = item.Pose.Position;

	AnimateItem(item);
	ProcessSectorFlags(&item);
	CreatureHealth(&item);

	if (item.Status == ITEM_DEACTIVATED)
	{
		CreatureDie(itemNumber, false);
		return false;
	}

	return CreaturePathfind(&item, prevPos, headingAngle, tiltAngle);
}

void CreatureHealth(ItemInfo* item)
{
	auto* creature = GetCreatureInfo(item);

	if (creature->Poisoned && (GlobalCounter & 0x1F) == 0x1F)
	{
		if (item->HitPoints > (g_GameFlow->GetSettings()->Gameplay.KillPoisonedEnemies ? 0 : 1))
			item->HitPoints--;
	}

	if (!Objects[item->ObjectNumber].WaterCreature() &&
		TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, &g_Level.Rooms[item->RoomNumber]))
	{
		auto bounds = GameBoundingBox(item);
		auto waterHeight = GetPointCollision(*item).GetWaterTopHeight();

		if (waterHeight != NO_HEIGHT && item->Pose.Position.y + bounds.Y1 > waterHeight)
			DoDamage(item, INT_MAX);
	}
}

void CreatureDie(int itemNumber, bool doExplosion, bool forceExplosion)
{
	int flags = 0;
	if (doExplosion)
	{
		const auto& item = g_Level.Items[itemNumber];
		const auto& object = Objects[item.ObjectNumber];

		switch (object.hitEffect)
		{
		case HitEffect::Blood:
			flags |= BODY_DO_EXPLOSION | BODY_GIBS;
			break;

		case HitEffect::Smoke:
			flags |= BODY_DO_EXPLOSION | BODY_NO_BOUNCE;
			break;

		case HitEffect::NonExplosive:
			if (forceExplosion)
			{
				flags |= BODY_DO_EXPLOSION;
				break;
			}
			return;

		default:
			flags |= BODY_DO_EXPLOSION;
			break;
		}
	}

	CreatureDie(itemNumber, doExplosion, flags);
}

void CreatureDie(int itemNumber, bool doExplosion, int flags)
{
	auto& item = g_Level.Items[itemNumber];

	item.HitPoints = NOT_TARGETABLE;
	item.Collidable = false;

	if (doExplosion)
	{
		ExplodingDeath(itemNumber, flags);
		KillItem(itemNumber);
	}
	else
	{
		RemoveActiveItem(itemNumber);
	}

	DisableEntityAI(itemNumber);
	item.Flags |= IFLAG_KILLED | IFLAG_INVISIBLE;
	DropPickups(&item);
}

bool BadFloor(int x, int y, int z, int boxHeight, int nextHeight, short roomNumber, LOTInfo* LOT)
{
	auto* floor = GetFloor(x, y, z, &roomNumber);
	if (floor->PathfindingBoxID == NO_VALUE)
		return true;

	if (LOT->IsJumping)
		return false;

	auto* box = &g_Level.PathfindingBoxes[floor->PathfindingBoxID];
	if (box->flags & LOT->BlockMask)
		return true;

	int height = box->height;
	bool heightResult = false;

	if ((boxHeight - height) > LOT->Step || (boxHeight - height) < LOT->Drop)
		heightResult = true;

	if ((boxHeight - height) < -LOT->Step && height > nextHeight)
		heightResult = true;

	if (LOT->Fly != NO_FLYING && y > (height + LOT->Fly))
		heightResult = true;

	// PORTAL-Y RETRY: GetFloor only traverses a floor portal when the probe Y is at-or-below
	// the portal's Y. For a TALL creature whose head Y lands ABOVE the destination's portal,
	// the first probe stays in the creature's own room and returns its deep fallback floor
	// (a phantom box with a huge height), causing a SPURIOUS rejection at vertical-portal
	// edges -- e.g. stepping into a stacked / flip-adjacent room. Scan probe Y across the
	// full Drop..Step vertical range; if any probe lands on a floor within
	// Step/Drop it's a legitimate step and we clear the rejection. For real cliffs every
	// probe returns the same far floor and the rejection stands. Ground creatures only
	// (swimmers/flyers have BLOCK-sized Step/Drop that would wrongly clear everything).
	if (heightResult && LOT->Zone != ZoneType::Water && LOT->Fly == NO_FLYING)
	{
		auto tryProbe = [&](int probeY) -> bool
		{
			short rn2 = roomNumber;
			auto* f = GetFloor(x, probeY, z, &rn2);
			if (!f || f->PathfindingBoxID == NO_VALUE)
				return false;
			int h2 = g_Level.PathfindingBoxes[f->PathfindingBoxID].height;
			int dh2 = boxHeight - h2;
			return (dh2 <= LOT->Step && dh2 >= LOT->Drop);
		};

		for (int dY = LOT->Drop; dY <= LOT->Step; dY += CLICK(1))
		{
			if (tryProbe(boxHeight - dY))
			{
				heightResult = false;
				break;
			}
		}
	}

	if (heightResult)
		AddBadBox(LOT, floor->PathfindingBoxID);

	return heightResult;
}

int CreatureCreature(short itemNumber)
{
	auto* item = &g_Level.Items[itemNumber];
	auto* object = &Objects[item->ObjectNumber];

	int x = item->Pose.Position.x;
	int z = item->Pose.Position.z;
	short radius = object->radius;

	auto* room = &g_Level.Rooms[item->RoomNumber];

	short link = room->itemNumber;
	int distance = 0;
	do
	{
		auto* linked = &g_Level.Items[link];
		
		if (link != itemNumber && linked != LaraItem && linked->IsCreature() && linked->Status == ITEM_ACTIVE && linked->HitPoints > 0) // TODO: deal with LaraItem global.
		{
			int xDistance = abs(linked->Pose.Position.x - x);
			int zDistance = abs(linked->Pose.Position.z - z);
			
			// Octagonal distance approximation (no sqrt): larger axis full, smaller halved.
			// The else branch previously duplicated the xDistance form, under-reporting
			// distance along Z (avoidance triggered too late when creatures were separated
			// mostly on Z).
			if (xDistance > zDistance)
				distance = xDistance + (zDistance >> 1);
			else
				distance = zDistance + (xDistance >> 1);

			if (distance < radius + Objects[linked->ObjectNumber].radius)
				return phd_atan(linked->Pose.Position.z - z, linked->Pose.Position.x - x) - item->Pose.Orientation.y;
		}

		link = linked->NextItem;
	} while (link != NO_VALUE);

	return 0;
}

/**
 * @brief Checks if a box is a valid pathfinding target for a creature.
 *
 * A box is valid if:
 * 1. It exists (not NO_VALUE).
 * 2. It's in the same zone as the creature (unless creature can fly/swim).
 * 3. It's not blocked by the creature's BlockMask.
 * 4. The creature is NOT currently standing inside it (prevents targeting current box).
 *
 * Used when selecting random boxes for Bored/Escape/Stalk moods.
 *
 * @param item Pointer to the creature item.
 * @param zoneNumber The zone number to check against.
 * @param boxNumber The box index to validate.
 * @return true if the box is a valid target.
 */
bool ValidBox(ItemInfo* item, short zoneNumber, short boxNumber)
{
	if (boxNumber == NO_VALUE)
		return false;

	const auto& creature = *GetCreatureInfo(item);
	const auto& zone = GetRuntimeZoneTable((int)creature.LOT.Zone).data();

	// Creatures that can move in 3D (fly/swim) bypass zone check.
	if (creature.LOT.Fly == NO_FLYING && zone[boxNumber] != zoneNumber)
		return false;

	const auto& box = g_Level.PathfindingBoxes[boxNumber];
	if (creature.LOT.BlockMask & box.flags)
		return false;

	// Don't target the box we're currently standing in.
	if (item->Pose.Position.z > (box.left * BLOCK(1)) &&
		item->Pose.Position.z < (box.right * BLOCK(1)) &&
		item->Pose.Position.x > (box.top * BLOCK(1)) &&
		item->Pose.Position.x < (box.bottom * BLOCK(1)))
	{
		return false;
	}

	return true;
}

/**
 * @brief Checks if a box is suitable for escaping from an enemy.
 *
 * A box is good for escape if:
 * 1. It's far enough from the enemy (> ESCAPE_DIST).
 * 2. Moving to it would take the creature AWAY from the enemy.
 *
 * The direction check ensures the creature flees in a sensible direction
 * rather than running past the enemy.
 *
 * @param item Pointer to the fleeing creature.
 * @param enemy Pointer to the enemy being escaped from.
 * @param boxNumber The potential escape box to evaluate.
 * @return true if the box is a good escape target.
 */
bool EscapeBox(ItemInfo* item, ItemInfo* enemy, int boxNumber)
{
	if (boxNumber == NO_VALUE)
		return false;

	const auto& box = g_Level.PathfindingBoxes[boxNumber];
	int escapeDistance = g_GameFlow->GetSettings()->Pathfinding.EscapeDistance;

	// Calculate vector from enemy to box center.
	int x = ((box.top + box.bottom) * BLOCK(0.5f)) - enemy->Pose.Position.x;
	int z = ((box.left + box.right) * BLOCK(0.5f)) - enemy->Pose.Position.z;

	// Box is too close to enemy.
	if (x > -escapeDistance && x < escapeDistance &&
		z > -escapeDistance && z < escapeDistance)
	{
		return false;
	}

	// Check if moving to box takes creature away from enemy.
	return ((z > 0) == (item->Pose.Position.z > enemy->Pose.Position.z)) ||
		   ((x > 0) == (item->Pose.Position.x > enemy->Pose.Position.x));
}

/**
 * @brief Sets a box as the creature's navigation target.
 *
 * Generates a random position within the box boundaries as the target point.
 * This randomization prevents creatures from always going to the exact same
 * spot, making movement look more natural.
 *
 * Sets LOT->RequiredBox which triggers UpdateLOT to recalculate the path.
 *
 * @param LOT Pointer to the creature's LOTInfo.
 * @param boxNumber The box to target.
 */
void TargetBox(LOTInfo* LOT, int boxNumber)
{
	boxNumber = ResolveRuntimeBox(boxNumber);
	if (boxNumber == NO_VALUE)
		return;

	const auto* box = &g_Level.PathfindingBoxes[boxNumber];

	// Generate random position within box bounds.
	// Maximize target precision. DO NOT change bracket precedence!
	LOT->Target.x = (int)((box->top  * BLOCK(1)) + (float)GetRandomControl() * (((float)(box->bottom - box->top) - 1.0f) / 32.0f) + CLICK(2.0f));
	LOT->Target.z = (int)((box->left * BLOCK(1)) + (float)GetRandomControl() * (((float)(box->right - box->left) - 1.0f) / 32.0f) + CLICK(2.0f));
	LOT->RequiredBox = boxNumber;

	// Flying creatures target slightly above the floor.
	if (LOT->Fly == NO_FLYING)
		LOT->Target.y = box->height;
	else
		LOT->Target.y = box->height - STEPUP_HEIGHT;
}

/**
 * @brief Updates the pathfinding data when the target changes.
 *
 * When RequiredBox changes (new target selected), this function:
 * 1. Sets it as the new TargetBox.
 * 2. Adds it to the front of the search queue (Head).
 * 3. Increments SearchNumber to invalidate old search data.
 * 4. Calls SearchLOT to expand the search.
 *
 * The search works BACKWARDS from target to creature - each node's exitBox
 * points toward the target, so the creature follows exitBox links to reach it.
 *
 * @param LOT Pointer to the creature's LOTInfo.
 * @param depth Number of expansions to perform (higher = more thorough but slower).
 * @return true if search queue still has nodes to expand, false if exhausted.
 */
bool UpdateLOT(LOTInfo* LOT, int depth)
{
	LOT->RequiredBox = ResolveRuntimeBox(LOT->RequiredBox);
	LOT->TargetBox = ResolveRuntimeBox(LOT->TargetBox);
	LOT->Head = ResolveRuntimeBox(LOT->Head);
	LOT->Tail = ResolveRuntimeBox(LOT->Tail);

	if (LOT->RequiredBox != NO_VALUE && LOT->RequiredBox != LOT->TargetBox)
	{
		// New target - reset search from this box.
		LOT->TargetBox = LOT->RequiredBox;

		auto* node = &LOT->Node[LOT->RequiredBox];

		// Add target box to front of search queue.
		if (node->nextExpansion == NO_VALUE && LOT->Tail != LOT->RequiredBox)
		{
			node->nextExpansion = LOT->Head;

			if (LOT->Head == NO_VALUE)
				LOT->Tail = LOT->TargetBox;

			LOT->Head = LOT->TargetBox;
		}

		// New search number invalidates all previous search data.
		node->searchNumber = ++LOT->SearchNumber;
		node->exitBox = NO_VALUE; // Target has no exit (it IS the destination).
		node->cost = 0.0f;        // Target box has zero cost.
	}

	return SearchLOT(LOT, depth);
}

bool SearchLOT(LOTInfo* LOT, int depth)
{
	auto mode = g_GameFlow->GetSettings()->Pathfinding.Mode;

	if (mode == PathfindingMode::BFS)
		return SearchLOT_BFS(LOT, depth);

	return SearchLOT_DijkstraAStar(LOT, depth, mode);
}

// ============================================================================
// RUNTIME PER-COMBINATION ZONE RE-FLOOD
// ============================================================================
// The compiler bakes only two global zone snapshots (all-unflipped / all-flipped)
// selected by the single global FlipStatus. That cannot represent independent flip
// groups: when group 1 is flipped but group 0 is not, a box in the unflipped group-0
// room is "Unflipped-only" and has no zone in the flipped snapshot the global
// FlipStatus forces everyone to read, so the creature there can't path to a target in
// the flipped room (and vice versa). These routines rebuild a zone table for the ACTUAL
// current flip combination, using each box's real room state, and are recomputed at
// level load and on every DoFlipMap.

static std::vector<int>         s_boxFlipGroup;   // flip group of each box's room, or NO_VALUE
static std::vector<signed char> s_boxNativeState; // 0 = base-only, 1 = alt-only, 2 = both (merged / non-alternated)
static std::vector<int>         s_runtimeZones[(int)ZoneType::MaxZone];
static std::vector<std::vector<RuntimePathEdge>> s_seamEdges; // per box: live room-sector neighbours that compiled overlaps can't represent safely
static std::vector<char>        s_liveEdgeBoxes; // boxes reachable through live room-sector edges, even if global metadata says inactive
static std::vector<char>        s_runtimeActiveBoxes; // boxes present in currently active room-sector data
static std::vector<int>         s_runtimeBoxAliases; // active duplicate surface boxes canonicalized to a single live box

static std::vector<std::vector<RuntimePathEdge>> s_reverseEdges; // per target box: boxes that can move forward into it

static int ResolveRuntimeBox(int box)
{
	if (box < 0 || box >= (int)s_runtimeBoxAliases.size())
		return box;

	int resolved = box;
	for (int i = 0; i < 8; i++)
	{
		int next = s_runtimeBoxAliases[resolved];
		if (next == NO_VALUE || next == resolved || next < 0 || next >= (int)s_runtimeBoxAliases.size())
			break;

		resolved = next;
	}

	return resolved;
}

static const std::vector<RuntimePathEdge>& GetSeamEdgesForBox(int box)
{
	static const std::vector<RuntimePathEdge> empty = {};

	box = ResolveRuntimeBox(box);
	if (box < 0 || box >= (int)s_seamEdges.size())
		return empty;

	return s_seamEdges[box];
}

static int ResolveCreatureCurrentBox(ItemInfo* item)
{
	if (item == nullptr)
		return NO_VALUE;

	int legacyBox = NO_VALUE;
	if (item->RoomNumber >= 0 && item->RoomNumber < (int)g_Level.Rooms.size())
	{
		auto* room = &g_Level.Rooms[item->RoomNumber];
		legacyBox = GetSector(room, item->Pose.Position.x - room->Position.x, item->Pose.Position.z - room->Position.z)->PathfindingBoxID;
	}

	auto pointColl = GetPointCollision(item->Pose.Position, item->RoomNumber);
	auto& sector = pointColl.GetSector();
	int resolvedBox = sector.PathfindingBoxID;

	if (resolvedBox == NO_VALUE || !IsBoxUsableNow(resolvedBox, false))
	{
		auto& bottomSector = pointColl.GetBottomSector();
		if (bottomSector.PathfindingBoxID != NO_VALUE && IsBoxUsableNow(bottomSector.PathfindingBoxID, false))
			resolvedBox = bottomSector.PathfindingBoxID;
	}

	if ((resolvedBox == NO_VALUE || !IsBoxUsableNow(resolvedBox, false)) &&
		legacyBox != NO_VALUE && IsBoxUsableNow(legacyBox, false))
		resolvedBox = legacyBox;

	return ResolveRuntimeBox(resolvedBox);
}

const std::vector<int>& GetRuntimeZoneTable(int zoneType)
{
	return s_runtimeZones[zoneType];
}

// Redirect every portal that points at an ALTERNATE room to its BASE room. TEN keeps the
// currently-active variant of a flip pair in the BASE room slot (FlipRooms swaps the DATA
// between the base and alt slots, but the game always accesses the active room via the base
// number). Levels, however, link alt rooms with alt->alt portals (room1 -> room3). In a MIXED
// flip state (group 1 flipped, group 0 not) that sends a probe leaving the flipped room into
// the INACTIVE alt slot (room3 = room0's alt) instead of the active base (room0), so the
// geometry/boxes of the unflipped room are read from its alternate. Pointing every portal at
// the base slot makes it always resolve to the active variant -- correct in base, global-flip
// and mixed-flip states alike. Done once at load (idempotent: base rooms are never alt targets).
void RemapAlternateRoomPortals()
{
	int n = (int)g_Level.Rooms.size();
	std::vector<int> altToBase(n, NO_VALUE);
	for (int r = 0; r < n; r++)
	{
		int alt = g_Level.Rooms[r].flippedRoom;
		if (alt != NO_VALUE && alt >= 0 && alt < n)
			altToBase[alt] = r;
	}

	auto remap = [&](int& rm) -> bool
	{
		if (rm != NO_VALUE && rm >= 0 && rm < n && altToBase[rm] != NO_VALUE)
		{
			rm = altToBase[rm];
			return true;
		}
		return false;
	};

	for (auto& room : g_Level.Rooms)
	{
		for (auto& sector : room.Sectors)
		{
			remap(sector.SidePortalRoomNumber);
			for (auto& tri : sector.FloorSurface.Triangles)
				remap(tri.PortalRoomNumber);
			for (auto& tri : sector.CeilingSurface.Triangles)
				remap(tri.PortalRoomNumber);
		}
	}
}

void BuildPathfindingFlipMetadata()
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	s_boxFlipGroup.assign(boxCount, NO_VALUE);
	s_boxNativeState.assign(boxCount, 2); // default: present in both states (non-alternated / merged)

	bool hasCompiledMetadata = boxCount > 0;
	for (int box = 0; box < boxCount; box++)
	{
		if ((g_Level.PathfindingBoxes[box].flags & BOX_FLIP_METADATA) == 0)
		{
			hasCompiledMetadata = false;
			break;
		}
	}

	if (hasCompiledMetadata)
	{
		for (int box = 0; box < boxCount; box++)
		{
			int flags = g_Level.PathfindingBoxes[box].flags;
			int encodedGroup = (flags & BOX_FLIP_GROUP_MASK) >> BOX_FLIP_GROUP_SHIFT;
			int group = (encodedGroup > 0) ? encodedGroup - 1 : NO_VALUE;
			int native = (flags & BOX_FLIP_NATIVE_MASK) >> BOX_FLIP_NATIVE_SHIFT;

			s_boxFlipGroup[box] = (group >= 0 && group < MAX_FLIPMAP) ? group : NO_VALUE;
			s_boxNativeState[box] = (native >= 0 && native <= 2) ? (signed char)native : 2;
		}

		return;
	}

	std::vector<std::vector<unsigned char>> stateMasks(boxCount, std::vector<unsigned char>(MAX_FLIPMAP, 0));

	// Pair each flip-base room (flippedRoom set) with its alternate and compare their
	// sector -> box maps. A box index can appear in different cells in base and alt
	// geometry, so accumulate per-box state masks first instead of letting the first
	// single-state occurrence win.
	for (int roomNumber = 0; roomNumber < (int)g_Level.Rooms.size(); roomNumber++)
	{
		auto& room = g_Level.Rooms[roomNumber];
		if (room.flippedRoom == NO_VALUE || room.flipNumber == NO_VALUE || room.flipNumber < 0 || room.flipNumber >= MAX_FLIPMAP)
			continue;

		auto markBoxState = [&](int box, unsigned char stateBit)
		{
			if (box != NO_VALUE && box >= 0 && box < boxCount)
				stateMasks[box][room.flipNumber] |= stateBit;
		};

		auto& altRoom = g_Level.Rooms[room.flippedRoom];
		for (const auto& sector : room.Sectors)
			markBoxState(sector.PathfindingBoxID, 1);
		for (const auto& sector : altRoom.Sectors)
			markBoxState(sector.PathfindingBoxID, 2);
	}

	for (int box = 0; box < boxCount; box++)
	{
		int bothGroup = NO_VALUE;

		// If a box is variant in one flip group but merely merged in another, the variant
		// group must control activation. Otherwise, a base+alt reused index is active in both.
		for (int group = 0; group < MAX_FLIPMAP; group++)
		{
			auto mask = stateMasks[box][group];
			if (mask == 1 || mask == 2)
			{
				s_boxNativeState[box] = (mask == 1) ? 0 : 1;
				s_boxFlipGroup[box] = group;
				break;
			}

			if (mask == 3 && bothGroup == NO_VALUE)
				bothGroup = group;
		}

		if (s_boxFlipGroup[box] == NO_VALUE && bothGroup != NO_VALUE)
			s_boxFlipGroup[box] = bothGroup;
	}

}

// Is this box the one its sector currently resolves to, given the live per-room flip state?
static bool IsBoxActiveNow(int box)
{
	if (box < 0 || box >= (int)s_boxNativeState.size())
		return true;

	int native = s_boxNativeState[box];
	if (native == 2)
		return true;

	int group = s_boxFlipGroup[box];
	bool flipped = (group != NO_VALUE && group < MAX_FLIPMAP && FlipStats[group]);
	return (native == 0) ? !flipped : flipped; // base-only active while unflipped; alt-only while flipped
}

static bool IsBoxUsableNow(int box, bool liveEdge)
{
	box = ResolveRuntimeBox(box);
	if (box < 0 || box >= (int)s_boxNativeState.size())
		return false;

	if (IsBoxActiveNow(box) &&
		(s_runtimeActiveBoxes.empty() ||
		 (box < (int)s_runtimeActiveBoxes.size() && s_runtimeActiveBoxes[box] != 0)))
		return true;

	return liveEdge &&
		box >= 0 &&
		box < (int)s_liveEdgeBoxes.size() &&
		s_liveEdgeBoxes[box] != 0;
}

// Is an overlap leaving 'box' physically present right now? An overlap carries the geometry
// of the compiler pass that found it (unflipped / flipped). It is real now only if it matches
// the box's ROOM's CURRENT flip state -- even for a "merged" box (same floor in both states),
// because its ADJACENCIES still change when its room flips (alt geometry can extend over a
// neighbouring base room). This is directional: a base box B can be entered going UP onto a
// merged box M (B unflipped -> U valid), while M, once its room is flipped, leaves via its
// ALT neighbours (F), not back down to B -- exactly the one-way seam the logs showed.
static bool OverlapActiveFromBox(int box, int overlapFlags)
{
	box = ResolveRuntimeBox(box);

	if ((overlapFlags & (OVERLAP_UNFLIPPED_VALID | OVERLAP_FLIPPED_VALID)) == 0)
		return true; // untagged (legacy compile) -> valid in both states

	int group = (box >= 0 && box < (int)s_boxFlipGroup.size()) ? s_boxFlipGroup[box] : NO_VALUE;
	bool flipped = (group != NO_VALUE && group < MAX_FLIPMAP && FlipStats[group]);
	int needBit = flipped ? OVERLAP_FLIPPED_VALID : OVERLAP_UNFLIPPED_VALID;
	return (overlapFlags & needBit) != 0;
}

static bool TryGetCompiledOverlapFlags(int fromBox, int toBox, int& flags)
{
	fromBox = ResolveRuntimeBox(fromBox);
	toBox = ResolveRuntimeBox(toBox);

	int boxCount = (int)g_Level.PathfindingBoxes.size();
	if (fromBox < 0 || fromBox >= boxCount || toBox < 0 || toBox >= boxCount)
		return false;

	int index = g_Level.PathfindingBoxes[fromBox].overlapIndex;
	while (index >= 0 && index < (int)g_Level.Overlaps.size())
	{
		const auto& overlap = g_Level.Overlaps[index];
		if (ResolveRuntimeBox(overlap.box) == toBox)
		{
			flags = overlap.flags;
			return true;
		}

		if (overlap.flags & OVERLAP_END_BIT)
			break;

		index++;
	}

	return false;
}

struct PendingLiveStep
{
	int fromBox = NO_VALUE;
	int toBox = NO_VALUE;
	int fromGroup = NO_VALUE;
	int toGroup = NO_VALUE;
	int fromState = 0;
	int toState = 0;
	int dh = 0;
};

static int GetRoomFlipGroup(const RoomData& room)
{
	return (room.flipNumber != NO_VALUE && room.flipNumber >= 0 && room.flipNumber < MAX_FLIPMAP) ? room.flipNumber : NO_VALUE;
}

static int GetFlipGroupState(int group)
{
	return (group != NO_VALUE && group >= 0 && group < MAX_FLIPMAP && FlipStats[group]) ? 1 : 0;
}

static bool AddRuntimeSeamEdge(int fromBox, int toBox, int flags = 0)
{
	auto& adj = s_seamEdges[fromBox];
	for (auto& edge : adj)
	{
		if (edge.box != toBox)
			continue;

		edge.flags |= flags;
		return false;
	}

	adj.push_back({ toBox, flags, true });
	s_liveEdgeBoxes[fromBox] = true;
	s_liveEdgeBoxes[toBox] = true;
	return true;
}

static bool BoxesOverlapXZ(int a, int b)
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	if (a < 0 || a >= boxCount || b < 0 || b >= boxCount)
		return false;

	auto& A = g_Level.PathfindingBoxes[a];
	auto& B = g_Level.PathfindingBoxes[b];
	return A.left < B.right &&
		A.right > B.left &&
		A.top < B.bottom &&
		A.bottom > B.top;
}

static int GetBoxOverlapAreaXZ(int a, int b)
{
	if (!BoxesOverlapXZ(a, b))
		return 0;

	auto& A = g_Level.PathfindingBoxes[a];
	auto& B = g_Level.PathfindingBoxes[b];
	int xOverlap = std::min((int)A.bottom, (int)B.bottom) - std::max((int)A.top, (int)B.top);
	int zOverlap = std::min((int)A.right, (int)B.right) - std::max((int)A.left, (int)B.left);
	return xOverlap * zOverlap;
}

static int GetBoxAreaXZ(int box)
{
	if (box < 0 || box >= (int)g_Level.PathfindingBoxes.size())
		return 0;

	auto& B = g_Level.PathfindingBoxes[box];
	return (int)(B.bottom - B.top) * (int)(B.right - B.left);
}

static std::vector<char> BuildActiveLiveBoxSet(std::vector<int>& activeLiveBoxes)
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	std::vector<char> activeLiveBoxSet(boxCount, false);

	for (const auto& room : g_Level.Rooms)
	{
		if (!room.Active())
			continue;

		for (const auto& sector : room.Sectors)
		{
			int box = sector.PathfindingBoxID;
			if (box != NO_VALUE &&
				box < boxCount &&
				IsBoxActiveNow(box) &&
				!activeLiveBoxSet[box])
			{
				activeLiveBoxes.push_back(box);
				activeLiveBoxSet[box] = true;
			}
		}
	}

	return activeLiveBoxSet;
}

static std::vector<int> BuildActiveOverlayMap(const std::vector<int>& activeLiveBoxes, const std::vector<char>& activeLiveBoxSet)
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	std::vector<int> activeOverlayBoxes(boxCount, NO_VALUE);

	for (int overlayBox = 0; overlayBox < boxCount; overlayBox++)
	{
		auto& box = g_Level.PathfindingBoxes[overlayBox];
		bool overlayActive = activeLiveBoxSet[overlayBox] != 0;
		int overlayArea = GetBoxAreaXZ(overlayBox);
		int bestBox = NO_VALUE;
		int bestArea = 0;
		int bestHeightDelta = INT_MAX;

		for (int activeBox : activeLiveBoxes)
		{
			if (activeBox == overlayBox)
				continue;

			auto& active = g_Level.PathfindingBoxes[activeBox];
			int area = GetBoxOverlapAreaXZ(overlayBox, activeBox);
			if (area <= 0)
				continue;

			int heightDelta = abs(box.height - active.height);
			if (overlayActive)
			{
				int activeArea = GetBoxAreaXZ(activeBox);
				if (heightDelta != 0 || area != overlayArea || activeArea <= overlayArea)
					continue;
			}

			if (area > bestArea || (area == bestArea && heightDelta < bestHeightDelta))
			{
				bestBox = activeBox;
				bestArea = area;
				bestHeightDelta = heightDelta;
			}
		}

		activeOverlayBoxes[overlayBox] = bestBox;
	}

	return activeOverlayBoxes;
}

static int ResolveActiveOverlayBox(int overlayBox, const std::vector<int>& activeOverlayBoxes)
{
	if (overlayBox < 0 || overlayBox >= (int)activeOverlayBoxes.size())
		return NO_VALUE;

	return activeOverlayBoxes[overlayBox];
}

static void TryAddLiveSeamEdge(
	int sourceBox,
	int destBox,
	int destRoomNumber,
	int sourceGroup,
	const std::vector<int>& activeOverlayBoxes,
	std::vector<std::vector<int>>& mixedSeamPartners,
	std::vector<PendingLiveStep>& pendingLiveSteps)
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	if (destBox == NO_VALUE || destBox >= boxCount || destBox == sourceBox)
		return;
	if (destRoomNumber < 0 || destRoomNumber >= (int)g_Level.Rooms.size())
		return;

	auto& destRoom = g_Level.Rooms[destRoomNumber];
	if (!destRoom.Active())
		return;

	int sourceOverlay = ResolveActiveOverlayBox(sourceBox, activeOverlayBoxes);
	int destOverlay = ResolveActiveOverlayBox(destBox, activeOverlayBoxes);
	int fromBox = (sourceOverlay != NO_VALUE) ? sourceOverlay : sourceBox;
	int toBox = (destOverlay != NO_VALUE) ? destOverlay : destBox;
	if (fromBox == NO_VALUE || toBox == NO_VALUE || fromBox >= boxCount || toBox >= boxCount || fromBox == toBox)
		return;

	int fromGroup = (sourceOverlay != NO_VALUE) ? s_boxFlipGroup[fromBox] : sourceGroup;
	int destGroup = (destOverlay != NO_VALUE) ? s_boxFlipGroup[toBox] : GetRoomFlipGroup(destRoom);
	int sourceState = GetFlipGroupState(fromGroup);
	int destState = GetFlipGroupState(destGroup);
	bool metadataInactive = !IsBoxActiveNow(fromBox) || !IsBoxActiveNow(toBox);
	bool mixedGroupSeam = fromGroup != destGroup && sourceState != destState;
	int dh = g_Level.PathfindingBoxes[fromBox].height - g_Level.PathfindingBoxes[toBox].height;
	bool sameGroupLiveStep = metadataInactive && fromGroup == destGroup && abs(dh) <= CLICK(1);
	int compiledFlags = 0;
	bool hasCompiledOverlap = TryGetCompiledOverlapFlags(fromBox, toBox, compiledFlags);
	bool inactiveCompiledOverlap = !metadataInactive &&
		hasCompiledOverlap &&
		!OverlapActiveFromBox(fromBox, compiledFlags);

	if (!mixedGroupSeam && !sameGroupLiveStep && !inactiveCompiledOverlap)
		return; // same-group/same-state live neighbours must not resurrect inactive phantom boxes

	// Mixed-state cross-group seam. Keep it if the floor step is plausibly walkable;
	// the per-creature Step/Drop is enforced later in CanExpandToBox. Metadata-inactive
	// boxes may participate only here, through live geometry, not through stale compiled paths.
	if (abs(dh) > BLOCK(2))
		return;

	if (sameGroupLiveStep)
	{
		pendingLiveSteps.push_back({ fromBox, toBox, fromGroup, destGroup, sourceState, destState, dh });
		return;
	}

	if (inactiveCompiledOverlap)
	{
		AddRuntimeSeamEdge(fromBox, toBox, compiledFlags);
		return;
	}

	mixedSeamPartners[fromBox].push_back(toBox);
	mixedSeamPartners[toBox].push_back(fromBox);
	AddRuntimeSeamEdge(fromBox, toBox, hasCompiledOverlap ? compiledFlags : 0);

	int reverseFlags = 0;
	if (!metadataInactive &&
		TryGetCompiledOverlapFlags(toBox, fromBox, reverseFlags) &&
		!OverlapActiveFromBox(toBox, reverseFlags))
		AddRuntimeSeamEdge(toBox, fromBox, reverseFlags);
}

static int FindSameGroupBridgePartner(int inactiveBox, int activeBox, const std::vector<std::vector<int>>& mixedSeamPartners)
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	if (inactiveBox < 0 || inactiveBox >= boxCount || activeBox < 0 || activeBox >= boxCount)
		return NO_VALUE;

	int activeHeight = g_Level.PathfindingBoxes[activeBox].height;
	for (int partner : mixedSeamPartners[inactiveBox])
	{
		if (partner >= 0 &&
			partner < boxCount &&
			abs(g_Level.PathfindingBoxes[partner].height - activeHeight) <= BLOCK(1))
			return partner;
	}

	return NO_VALUE;
}

static void FlushPendingLiveSteps(
	const std::vector<PendingLiveStep>& pendingLiveSteps,
	const std::vector<int>& activeOverlayBoxes,
	const std::vector<std::vector<int>>& mixedSeamPartners)
{
	for (const auto& edge : pendingLiveSteps)
	{
		int fromOverlay = ResolveActiveOverlayBox(edge.fromBox, activeOverlayBoxes);
		int toOverlay = ResolveActiveOverlayBox(edge.toBox, activeOverlayBoxes);
		int fromBridgePartner = !IsBoxActiveNow(edge.fromBox) ? FindSameGroupBridgePartner(edge.fromBox, edge.toBox, mixedSeamPartners) : NO_VALUE;
		int toBridgePartner = !IsBoxActiveNow(edge.toBox) ? FindSameGroupBridgePartner(edge.toBox, edge.fromBox, mixedSeamPartners) : NO_VALUE;
		bool allow = (fromBridgePartner != NO_VALUE || toBridgePartner != NO_VALUE) &&
			fromOverlay == NO_VALUE &&
			toOverlay == NO_VALUE;

		if (allow)
			AddRuntimeSeamEdge(edge.fromBox, edge.toBox);
	}
}

// Synthesize live adjacencies the 2-pass compiler can't represent safely. In mixed flip states,
// compiled overlaps can point at base/alt boxes that don't both exist. Also, the same box ID can
// appear in an active room while global box metadata says it's inactive because that ID was first
// classified under another flip group. Rebuild those edges from LIVE room-sector data and keep
// them separate from compiled overlaps so metadata-conflict boxes don't open stale compiled paths.
void BuildSeamEdges()
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	s_seamEdges.assign(boxCount, {});
	s_liveEdgeBoxes.assign(boxCount, false);
	std::vector<std::vector<int>> mixedSeamPartners(boxCount);
	std::vector<int> activeLiveBoxes;
	auto activeLiveBoxSet = BuildActiveLiveBoxSet(activeLiveBoxes);
	auto activeOverlayBoxes = BuildActiveOverlayMap(activeLiveBoxes, activeLiveBoxSet);
	s_runtimeActiveBoxes = activeLiveBoxSet;
	s_runtimeBoxAliases.assign(boxCount, NO_VALUE);

	for (int box = 0; box < boxCount; box++)
	{
		if (activeOverlayBoxes[box] != NO_VALUE)
			s_runtimeBoxAliases[box] = activeOverlayBoxes[box];
	}

	std::vector<PendingLiveStep> pendingLiveSteps;

	static const int dx[4] = { BLOCK(1), -BLOCK(1), 0, 0 };
	static const int dz[4] = { 0, 0, BLOCK(1), -BLOCK(1) };

	for (int rn = 0; rn < (int)g_Level.Rooms.size(); rn++)
	{
		auto& room = g_Level.Rooms[rn];
		if (!room.Active())
			continue;

		int sourceGroup = GetRoomFlipGroup(room);

		for (int sx = 0; sx < room.XSize; sx++)
		{
			for (int sz = 0; sz < room.ZSize; sz++)
			{
				auto& sector = room.Sectors[sx * room.ZSize + sz];
				int B = sector.PathfindingBoxID;
				if (B == NO_VALUE || B >= boxCount)
					continue;

				int wx = room.Position.x + sx * BLOCK(1) + BLOCK(0.5f);
				int wz = room.Position.z + sz * BLOCK(1) + BLOCK(0.5f);
				int wy = g_Level.PathfindingBoxes[B].height - CLICK(1); // just above this box's floor

				auto addLiveSeamEdge = [&](int Bp, int destRoomNumber)
				{
					TryAddLiveSeamEdge(
						B, Bp, destRoomNumber, sourceGroup,
						activeOverlayBoxes,
						mixedSeamPartners, pendingLiveSteps);
				};

				auto addVerticalPortalEdge = [&](bool isBelow)
				{
					auto nextRoom = sector.GetNextRoomNumber(wx, wz, isBelow);
					if (!nextRoom.has_value())
						return;

					auto& nextSector = TEN::Collision::Floordata::GetFloor(*nextRoom, wx, wz);
					addLiveSeamEdge(nextSector.PathfindingBoxID, *nextRoom);
				};

				addVerticalPortalEdge(true);
				addVerticalPortalEdge(false);

				for (int d = 0; d < 4; d++)
				{
					auto pc = GetPointCollision(Vector3i(wx + dx[d], wy, wz + dz[d]), rn);
					addLiveSeamEdge(pc.GetSector().PathfindingBoxID, pc.GetRoomNumber());
				}
			}
		}
	}

	FlushPendingLiveSteps(pendingLiveSteps, activeOverlayBoxes, mixedSeamPartners);
}

void BuildReversePathfindingEdges()
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	s_reverseEdges.assign(boxCount, {});

	auto addReverseEdge = [&](int fromBox, int toBox, int flags, bool live)
	{
		fromBox = ResolveRuntimeBox(fromBox);
		toBox = ResolveRuntimeBox(toBox);
		if (fromBox < 0 || fromBox >= boxCount || toBox < 0 || toBox >= boxCount)
			return;
		if (fromBox == toBox)
			return;

		s_reverseEdges[toBox].push_back({ fromBox, flags, live });
	};

	for (int fromBox = 0; fromBox < boxCount; fromBox++)
	{
		int index = g_Level.PathfindingBoxes[fromBox].overlapIndex;
		if (index >= 0)
		{
			bool last = false;
			while (!last && index < (int)g_Level.Overlaps.size())
			{
				const auto& overlap = g_Level.Overlaps[index++];
				last = (overlap.flags & OVERLAP_END_BIT) != 0;
				addReverseEdge(fromBox, overlap.box, overlap.flags, false);
			}
		}

		if (fromBox < (int)s_seamEdges.size())
		{
			for (const auto& edge : s_seamEdges[fromBox])
				addReverseEdge(fromBox, edge.box, edge.flags, true);
		}
	}
}

void RecomputeRuntimeZones()
{
	int boxCount = (int)g_Level.PathfindingBoxes.size();
	if ((int)s_boxNativeState.size() != boxCount)
		BuildPathfindingFlipMetadata();

	BuildSeamEdges();
	BuildReversePathfindingEdges();

	std::vector<int> stack;
	for (int zoneType = 0; zoneType < (int)ZoneType::MaxZone; zoneType++)
	{
		auto& zones = s_runtimeZones[zoneType];
		zones.assign(boxCount, 0);

		bool landZone = (zoneType != (int)ZoneType::Flyer &&
		                 zoneType != (int)ZoneType::Water &&
		                 zoneType != (int)ZoneType::Amphibious);

		int zoneCounter = 1;
		for (int seed = 0; seed < boxCount; seed++)
		{
			if (ResolveRuntimeBox(seed) != seed)
				continue;

			if (zones[seed] != 0 || !IsBoxUsableNow(seed, false))
				continue;

			bool seedWater   = (g_Level.PathfindingBoxes[seed].flags & BOX_WATER) != 0;
			bool seedShallow = (g_Level.PathfindingBoxes[seed].flags & BOX_SHALLOW) != 0;

			zones[seed] = zoneCounter;
			stack.clear();
			stack.push_back(seed);

			while (!stack.empty())
			{
				int cur = stack.back();
				stack.pop_back();

				// Per-neighbour flood step, shared by compiled overlaps and synthesized seam edges.
				auto processNeighbor = [&](int nb, int ovf, bool liveEdge)
				{
					nb = ResolveRuntimeBox(nb);
					if (nb < 0 || nb >= boxCount || zones[nb] != 0)
						return;
					if (nb == cur)
						return;
					if (!IsBoxUsableNow(cur, liveEdge) || !IsBoxUsableNow(nb, liveEdge) || (!liveEdge && !OverlapActiveFromBox(cur, ovf)))
						return;

					bool canJump   = (ovf & OVERLAP_JUMP) != 0;
					bool canMonkey = (ovf & OVERLAP_MONKEY) != 0;
					bool canAmphib = (ovf & OVERLAP_AMPHIBIOUS_TRAVERSABLE) != 0;

					int nbFlags    = g_Level.PathfindingBoxes[nb].flags;
					bool nbWater   = (nbFlags & BOX_WATER) != 0;
					bool nbShallow = (nbFlags & BOX_SHALLOW) != 0;
					bool nbSlope   = (nbFlags & BOX_SLOPE) != 0;

					int step = abs(g_Level.PathfindingBoxes[cur].height - g_Level.PathfindingBoxes[nb].height);

					// FILTER: slope (land creatures cannot use steep floors).
					if (landZone && nbSlope)
						return;

					// FILTER: water boundary (mirrors the compiler's seed-relative test).
					bool canCrossWater = (zoneType == (int)ZoneType::Amphibious) ||
					                     (zoneType == (int)ZoneType::Human && canMonkey);
					bool shallowOverlap = (nbShallow && seedShallow) ||
					                      (nbShallow && !seedWater) ||
					                      (seedShallow && !nbWater);
					shallowOverlap = landZone && shallowOverlap;
					if (nbWater != seedWater && !canCrossWater && !shallowOverlap)
						return;

					// FILTER: per-zone-type movement capability.
					bool add = false;
					switch ((ZoneType)zoneType)
					{
					case ZoneType::Skeleton:   add = (step <= CLICK(1) || canJump); break;
					case ZoneType::Basic:      add = (step <= CLICK(1)); break;
					case ZoneType::Water:      add = true; break;
					case ZoneType::Amphibious: add = canAmphib && !canJump && !(nbSlope && !nbWater); break;
					case ZoneType::Human:      add = (step <= BLOCK(1) || canJump || canMonkey); break;
					case ZoneType::Flyer:      add = true; break;
					default: break;
					}
					if (!add)
						return;

					zones[nb] = zoneCounter;
					stack.push_back(nb);
				};

				// Compiled overlaps.
				int index = g_Level.PathfindingBoxes[cur].overlapIndex;
				if (index >= 0 && IsBoxUsableNow(cur, false))
				{
					bool last = false;
					while (!last && index < (int)g_Level.Overlaps.size())
					{
						const auto& overlap = g_Level.Overlaps[index];
						last = (overlap.flags & OVERLAP_END_BIT) != 0;
						index++;
						processNeighbor(overlap.box, overlap.flags, false);
					}
				}

				// Synthesized cross-flip-group seam edges.
				if (cur < (int)s_seamEdges.size())
					for (const auto& edge : s_seamEdges[cur])
						processNeighbor(edge.box, edge.flags, true);
			}

			zoneCounter++;
		}
	}

}

bool CanExpandToBox(LOTInfo* LOT, int fromBox, int toBox, int overlapFlags, int searchZone, const std::vector<int>& zone, bool liveEdge)
{
	fromBox = ResolveRuntimeBox(fromBox);
	toBox = ResolveRuntimeBox(toBox);
	if (fromBox == NO_VALUE || toBox == NO_VALUE)
		return false;
	if (fromBox == toBox)
		return false;

	auto& head = g_Level.PathfindingBoxes[fromBox];
	auto& candidate = g_Level.PathfindingBoxes[toBox];

	// SearchLOT expands BACKWARDS from the target. If we visit 'toBox' from 'fromBox',
	// we will set Node[toBox].exitBox = fromBox, meaning the creature must later move
	// FORWARD from toBox -> fromBox. Therefore all directional traversal checks below
	// validate the forward edge toBox -> fromBox. The search loops feed us exactly
	// that forward edge from s_reverseEdges[fromBox].
	int forwardFlags = overlapFlags;
	int delta = head.height - candidate.height;

	// FLIP-STATE VALIDITY: an overlap leaving a box is usable only if it carries the bit matching
	// that box's REAL room flip state (not the global FlipStatus), so independent flip groups
	// work. Untagged (legacy / synthesized seam) entries are accepted in both states.
	if (!IsBoxUsableNow(fromBox, liveEdge) || !IsBoxUsableNow(toBox, liveEdge) || (!liveEdge && !OverlapActiveFromBox(toBox, forwardFlags)))
		return false;

	// PENALTY CHECK: Ignore box, if it is memorized as bad.
	if (IsBoxInCooldown(LOT, toBox))
		return false;

	// ZONE CHECK: Only flyer creatures can bypass zone check.
	if (LOT->Zone != ZoneType::Flyer && searchZone != zone[toBox])
		return false;

	// AMPHIBIOUS: if the overlap is not traversable, avoid this branch.
	if (LOT->Zone == ZoneType::Amphibious && !(forwardFlags & OVERLAP_AMPHIBIOUS_TRAVERSABLE))
		return false;

	// HEIGHT CHECK: Can creature traverse the height difference?
	if ((delta > LOT->Step || delta < LOT->Drop) && (!(forwardFlags & OVERLAP_MONKEY) || !LOT->CanMonkey))
		return false;

	// JUMP CHECK: Does this overlap require jumping?
	if ((forwardFlags & OVERLAP_JUMP) && !LOT->CanJump)
		return false;

	return true;
}

/**
 * @brief Core breadth-first search for pathfinding through boxes.
 *
 * This function expands the search from the current Head box to all connected boxes.
 * It builds a "flow field" where each box's exitBox points toward the target.
 *
 * ALGORITHM:
 * 1. Take the box at Head of the queue.
 * 2. For each overlapping (connected) box:
 *    a. Check zone compatibility (skip if different zone, unless flying/swimming).
 *    b. Check height difference against Step/Drop limits.
 *    c. Check special traversal flags (BOX_JUMP, BOX_MONKEY).
 *    d. If reachable, set its exitBox to current box and add to queue.
 * 3. Move to next box in queue, repeat.
 *
 * SEARCH NUMBER SYSTEM:
 * - Each search increments LOT->SearchNumber.
 * - Nodes with matching searchNumber have been visited this search.
 * - SEARCH_BLOCKED flag marks boxes that are blocked (creature can't stop there).
 * - Blocked paths can still be traversed but creature won't select as target.
 *
 * @param LOT Pointer to the creature's LOTInfo.
 * @param depth Maximum number of box expansions this call.
 * @return true if more boxes remain to expand, false if search exhausted.
 */
bool SearchLOT_BFS(LOTInfo* LOT, int depth)
{
	const auto& zone = GetRuntimeZoneTable((int)LOT->Zone);

	for (int i = 0; i < depth; i++)
	{
		// Search exhausted - no more boxes to expand.
		if (LOT->Head == NO_VALUE)
		{
			LOT->Tail = NO_VALUE;
			return false;
		}

		auto* node = &LOT->Node[LOT->Head];
		int searchZone = zone[LOT->Head];

		// Per-neighbour expansion, shared by compiled overlaps and synthesized seam edges.
		auto expandNeighbor = [&](int boxNumber, int flags, bool liveEdge)
		{
			boxNumber = ResolveRuntimeBox(boxNumber);
			if (boxNumber == NO_VALUE || boxNumber == LOT->Head)
				return;

			if (!CanExpandToBox(LOT, LOT->Head, boxNumber, flags, searchZone, zone, liveEdge))
				return;

			// SEARCH STATE: Check if we've already visited this box.
			auto* expand = &LOT->Node[boxNumber];
			if ((node->searchNumber & SEARCH_NUMBER) < (expand->searchNumber & SEARCH_NUMBER))
				return;

			// Handle blocked path propagation.
			if (node->searchNumber & SEARCH_BLOCKED)
			{
				if ((node->searchNumber & SEARCH_NUMBER) == (expand->searchNumber & SEARCH_NUMBER))
					return;

				expand->searchNumber = node->searchNumber;
			}
			else
			{
				if ((node->searchNumber & SEARCH_NUMBER) == (expand->searchNumber & SEARCH_NUMBER) && !(expand->searchNumber & SEARCH_BLOCKED))
					return;

				// Mark blocked boxes but still allow traversal through them.
				if (g_Level.PathfindingBoxes[boxNumber].flags & LOT->BlockMask)
				{
					expand->searchNumber = node->searchNumber | SEARCH_BLOCKED;
				}
				else
				{
					expand->searchNumber = node->searchNumber;
					expand->exitBox = LOT->Head; // Point back toward target.
				}
			}

			// Add to expansion queue if not already there.
			if (expand->nextExpansion == NO_VALUE && boxNumber != LOT->Tail)
			{
				LOT->Node[LOT->Tail].nextExpansion = boxNumber;
				LOT->Tail = boxNumber;
			}
		};

		// Reverse graph expansion. Each entry is a candidate box that can move
		// forward into Head, which is exactly the exitBox direction we assign.
		if (LOT->Head < (int)s_reverseEdges.size())
		{
			for (const auto& edge : s_reverseEdges[LOT->Head])
				expandNeighbor(edge.box, edge.flags, edge.live);
		}

		// Move to next box in queue.
		LOT->Head = node->nextExpansion;
		node->nextExpansion = NO_VALUE;
	}

	return true;
}

/**
 * @brief Cost-based path search (Dijkstra / A*) for pathfinding through boxes.
 *
 * This function expands the search using a priority queue ordered by accumulated
 * path cost. It builds a "flow field" where each box's exitBox points toward the
 * target, preferring lower-cost paths. When A* is selected, a heuristic is added
 * to guide the search toward the creature's source box.
 *
 * ALGORITHM:
 * 1. Take the box with the lowest total cost from the priority queue.
 * 2. For each overlapping (connected) box:
 *    a. Check zone compatibility (skip if different zone, unless flying/swimming).
 *    b. Check height difference against Step/Drop limits.
 *    c. Check special traversal flags (BOX_JUMP, BOX_MONKEY).
 *    d. Compute traversal cost to the neighbor box.
 *    e. If this path is better than any previously known path:
 *       - Update the neighbor's cost.
 *       - Set its exitBox to the current box.
 *       - Add it to the priority queue.
 * 3. Repeat until the expansion limit is reached or the queue is exhausted.
 *
 * SEARCH NUMBER SYSTEM:
 * - Each search increments LOT->SearchNumber.
 * - Nodes with matching searchNumber belong to the current search.
 * - SEARCH_BLOCKED flag marks boxes that are blocked (creature can't stop there).
 * - Blocked paths can still be traversed but are deprioritized during expansion.
 *
 * HEURISTIC (A* ONLY):
 * - Adds an estimated distance from the current box to the creature's source box.
 * - The heuristic biases expansion toward the source but does not invalidate
 *   correctness of the search.
 *
 * @param LOT Pointer to the creature's LOTInfo.
 * @param depth Maximum number of box expansions this call.
 * @return true if more boxes remain to expand, false if search exhausted.
 */
bool SearchLOT_DijkstraAStar(LOTInfo* LOT, int depth, PathfindingMode mode)
{
	PathQueue queue = {};
	const auto& zone = GetRuntimeZoneTable((int)LOT->Zone);

	LOT->SourceBox = ResolveRuntimeBox(LOT->SourceBox);
	LOT->Head = ResolveRuntimeBox(LOT->Head);
	LOT->Tail = ResolveRuntimeBox(LOT->Tail);

	// Determine whether A* heuristic should be applied.
	bool useHeuristic = (mode == PathfindingMode::AStar && LOT->SourceBox != NO_VALUE);

	// A* heuristic target (creature's source box center).
	auto sourceCenter = useHeuristic ? GetBoxCenter(LOT->SourceBox) : Vector3::Zero;

	// Move legacy Head/Tail expansion list into priority queue.
	int currentBox = LOT->Head;
	while (currentBox != NO_VALUE)
	{
		currentBox = ResolveRuntimeBox(currentBox);
		auto* node = &LOT->Node[currentBox];

		int next = node->nextExpansion;
		node->nextExpansion = NO_VALUE;

		float pathCost = node->cost;
		float heuristicCost = useHeuristic ? Vector3::Distance(GetBoxCenter(currentBox), sourceCenter) : 0.0f;

		queue.push({ pathCost + heuristicCost, pathCost, currentBox });

		currentBox = next;
	}

	// Reset legacy queue; it will be rebuilt from remaining PQ entries.
	LOT->Head = NO_VALUE;
	LOT->Tail = NO_VALUE;

	int expansions = 0;

	// Main Dijkstra / A* expansion loop.
	while (expansions < depth && !queue.empty())
	{
		auto [currentEstimatedCost, currentPathCost, headBox] = queue.top();
		queue.pop();

		auto* node = &LOT->Node[headBox];

		// Skip stale queue entries superseded by a cheaper path.
		if (currentPathCost > node->cost)
			continue;

		expansions++;

		int searchZone = zone[headBox];
		auto currentCenter = GetBoxCenter(headBox);

		// Per-neighbour expansion, shared by compiled overlaps and synthesized seam edges.
		auto expandNeighbor = [&](int boxNumber, int flags, bool liveEdge)
		{
			boxNumber = ResolveRuntimeBox(boxNumber);
			if (boxNumber == NO_VALUE || boxNumber == headBox)
				return;

			// Unified traversal checks (zone, cooldown, height, jump, etc).
			if (!CanExpandToBox(LOT, headBox, boxNumber, flags, searchZone, zone, liveEdge))
				return;

			auto* expand = &LOT->Node[boxNumber];

			// Compute traversal cost between box centers.
			auto  neighborCenter = GetBoxCenter(boxNumber);
			float edgeCost = Vector3::Distance(currentCenter, neighborCenter);

			float newPathCost = node->cost + edgeCost;
			float heuristicCost = useHeuristic ? Vector3::Distance(neighborCenter, sourceCenter) : 0.0f;
			float newEstimatedCost = newPathCost + heuristicCost;

			bool isNewSearch = (node->searchNumber & SEARCH_NUMBER) > (expand->searchNumber & SEARCH_NUMBER);
			bool isBetterPath = newPathCost < expand->cost;

			if (node->searchNumber & SEARCH_BLOCKED)
			{
				if (!isNewSearch)
					return;

				expand->searchNumber = node->searchNumber;
				expand->cost = newPathCost;
				queue.push({ newEstimatedCost, newPathCost, boxNumber });
			}
			else
			{
				if (!isNewSearch && !isBetterPath && !(expand->searchNumber & SEARCH_BLOCKED))
					return;

				if (g_Level.PathfindingBoxes[boxNumber].flags & LOT->BlockMask)
				{
					expand->searchNumber = node->searchNumber | SEARCH_BLOCKED;
				}
				else
				{
					expand->searchNumber = node->searchNumber;
					expand->exitBox = headBox;
				}

				expand->cost = newPathCost;
				queue.push({ newEstimatedCost, newPathCost, boxNumber });
			}
		};

		// Reverse graph expansion. Each entry is a candidate box that can move
		// forward into headBox, which is exactly the exitBox direction we assign.
		if (headBox < (int)s_reverseEdges.size())
		{
			for (const auto& edge : s_reverseEdges[headBox])
				expandNeighbor(edge.box, edge.flags, edge.live);
		}
	}

	// Rebuild legacy Head/Tail list from remaining priority queue entries.
	while (!queue.empty())
	{
		auto [estimatedTotalCost, pathCost, boxNumber] = queue.top();
		queue.pop();

		auto* node = &LOT->Node[boxNumber];

		if (pathCost <= node->cost && node->nextExpansion == NO_VALUE && boxNumber != LOT->Tail)
		{
			if (LOT->Head == NO_VALUE)
			{
				LOT->Head = boxNumber;
				LOT->Tail = boxNumber;
			}
			else
			{
				LOT->Node[LOT->Tail].nextExpansion = boxNumber;
				LOT->Tail = boxNumber;
			}
		}
	}

	// Return whether any expandable boxes remain.
	return LOT->Head != NO_VALUE;
}

bool CreatureActive(short itemNumber)
{
	auto* item = &g_Level.Items[itemNumber];

	// Object is not a creature.
	if (!Objects[item->ObjectNumber].intelligent)
		return false;

	// Object is already dead.
	if (item->Flags & IFLAG_KILLED)
		return false;

	if (item->Status == ITEM_INVISIBLE || !item->IsCreature())
	{
		// AI couldn't be activated.
		if (!EnableEntityAI(itemNumber, false))
			return false;

		item->Status = ITEM_ACTIVE;
	}

	return true;
}

void InitializeCreature(short itemNumber) 
{
	auto* item = &g_Level.Items[itemNumber];

	item->Collidable = true;
	item->Data = nullptr;
	item->StartPose = item->Pose;
}

bool StalkBox(ItemInfo* item, ItemInfo* enemy, int boxNumber)
{
	if (enemy == nullptr || boxNumber == NO_VALUE)
		return false;

	auto* box = &g_Level.PathfindingBoxes[boxNumber];
	int stalkDistance = g_GameFlow->GetSettings()->Pathfinding.StalkDistance;

	int xRange = stalkDistance + ((box->bottom - box->top) * BLOCK(1));
	int zRange = stalkDistance + ((box->right - box->left) * BLOCK(1));

	int x = (box->top + box->bottom) * BLOCK(1) / 2 - enemy->Pose.Position.x;
	int z = (box->left + box->right) * BLOCK(1) / 2 - enemy->Pose.Position.z;

	if (x > xRange || x < -xRange || z > zRange || z < -zRange)
		return false;

	int enemyQuad = (enemy->Pose.Orientation.y / ANGLE(90.0f)) + 2;
	int boxQuad;
	if (z > 0)
		boxQuad = (x > 0) ? 2 : 1;
	else
		boxQuad = (x > 0) ? 3 : 0;

	if (enemyQuad == boxQuad)
		return false;

	int baddyQuad = 0;
	if (item->Pose.Position.z > enemy->Pose.Position.z)
		baddyQuad = (item->Pose.Position.x > enemy->Pose.Position.x) ? 2 : 1;
	else
		baddyQuad = (item->Pose.Position.x > enemy->Pose.Position.x) ? 3 : 0;

	if (enemyQuad == baddyQuad && abs(enemyQuad - boxQuad) == 2)
		return false;

	return true;
}

// TODO: Do it via Lua instead. -- TokyoSU 22.12.21
bool IsCreatureVaultAvailable(ItemInfo* item, int stepCount)
{
	switch (stepCount)
	{
	case -4:
		return (item->ObjectNumber != ID_SMALL_SPIDER);

	case -3:
		return (item->ObjectNumber != ID_CIVVY &&
				item->ObjectNumber != ID_MP_WITH_STICK &&
				item->ObjectNumber != ID_YETI &&
				item->ObjectNumber != ID_LIZARD &&
				item->ObjectNumber != ID_APE &&
				item->ObjectNumber != ID_SMALL_SPIDER &&
			    item->ObjectNumber != ID_SOPHIA_LEIGH_BOSS);

	case -2:
		return (item->ObjectNumber != ID_BADDY1 &&
				item->ObjectNumber != ID_BADDY2 &&
				item->ObjectNumber != ID_CIVVY &&
				item->ObjectNumber != ID_MP_WITH_STICK &&
				item->ObjectNumber != ID_YETI &&
				item->ObjectNumber != ID_LIZARD &&
				item->ObjectNumber != ID_APE &&
				item->ObjectNumber != ID_SMALL_SPIDER &&
				item->ObjectNumber != ID_SOPHIA_LEIGH_BOSS);
	}

	return true;
}

int CreatureVault(short itemNumber, short angle, int vault, int shift)
{
	auto* item = &g_Level.Items[itemNumber];
	auto* creature = GetCreatureInfo(item);

	int xBlock = item->Pose.Position.x / BLOCK(1);
	int zBlock = item->Pose.Position.z / BLOCK(1);
	int y = item->Pose.Position.y;
	short roomNumber = item->RoomNumber;

	CreatureAnimation(itemNumber, angle, 0);

	if (item->Floor > (y + CLICK(4.5f)))
	{
		vault = 0;
	}
	else if (item->Floor > (y + CLICK(3.5f)) && IsCreatureVaultAvailable(item, -4))
	{
		vault = -4;
	}
	else if (item->Floor > (y + CLICK(2.5f)) && IsCreatureVaultAvailable(item, -3))
	{
		vault = -3;
	}
	else if (item->Floor > (y + CLICK(1.5f)) && IsCreatureVaultAvailable(item, -2))
	{
		vault = -2;
	}
	else if (item->Pose.Position.y > (y - CLICK(1.5f)))
	{
		int climbVault = 0;
		int currentBox = ResolveRuntimeBox(item->BoxNumber);
		int exitBox = (currentBox != NO_VALUE) ? creature->LOT.Node[currentBox].exitBox : NO_VALUE;
		int ddx = 0;
		int ddz = 0;
		int nearX = 0;
		int nearZ = 0;
		if (creature->LOT.Fly == NO_FLYING && exitBox != NO_VALUE && exitBox != currentBox)
		{
			const auto& exit = g_Level.PathfindingBoxes[exitBox];
			int climb = g_Level.PathfindingBoxes[currentBox].height - exit.height;
			if (climb > CLICK(1) && climb <= creature->LOT.Step)
			{
				nearX = std::clamp(item->Pose.Position.x, (int)(exit.top * BLOCK(1)), (int)(exit.bottom * BLOCK(1)) - 1);
				nearZ = std::clamp(item->Pose.Position.z, (int)(exit.left * BLOCK(1)), (int)(exit.right * BLOCK(1)) - 1);
				ddx = nearX - item->Pose.Position.x;
				ddz = nearZ - item->Pose.Position.z;

				if ((std::abs(ddx) + std::abs(ddz)) <= CLICK(1))
				{
					int probeY = exit.height - CLICK(1);
					short rnProbe = (short)FindRoomNumber(Vector3i(nearX, probeY, nearZ), item->RoomNumber);
					auto* fLedge = GetFloor(nearX, probeY, nearZ, &rnProbe);
					if (fLedge != nullptr && ResolveRuntimeBox(fLedge->PathfindingBoxID) == exitBox)
						climbVault = (climb < CLICK(2.5f)) ? 2 : ((climb < CLICK(3.5f)) ? 3 : 4);
				}
			}
		}

		if (climbVault == 0)
			return 0;

		if (std::abs(ddx) >= std::abs(ddz) && ddx != 0)
		{
			item->Pose.Position.x = (ddx > 0) ? (nearX - shift) : (nearX + shift);
			item->Pose.Orientation.y = (ddx > 0) ? ANGLE(90.0f) : -ANGLE(90.0f);
		}
		else if (ddz != 0)
		{
			item->Pose.Position.z = (ddz > 0) ? (nearZ - shift) : (nearZ + shift);
			item->Pose.Orientation.y = (ddz > 0) ? ANGLE(0.0f) : -ANGLE(180.0f);
		}

		item->Pose.Position.y = y;
		item->Floor = y;
		if (roomNumber != item->RoomNumber)
			ItemNewRoom(itemNumber, roomNumber);

		return climbVault;
	}
	else if (item->Pose.Position.y > (y - CLICK(2.5f)))
	{
		vault = 2;
	}
	else if (item->Pose.Position.y > (y - CLICK(3.5f)))
	{
		vault = 3;
	}
	else if (item->Pose.Position.y > (y - CLICK(4.5f)))
	{
		vault = 4;
	}

	// Jump
	int newXblock = item->Pose.Position.x / BLOCK(1);
	int newZblock = item->Pose.Position.z / BLOCK(1);

	if (zBlock == newZblock)
	{
		if (xBlock == newXblock)
			return 0;

		if (xBlock < newXblock)
		{
			item->Pose.Position.x = (newXblock * BLOCK(1)) - shift;
			item->Pose.Orientation.y = ANGLE(90.0f);
		}
		else
		{
			item->Pose.Position.x = (xBlock * BLOCK(1)) + shift;
			item->Pose.Orientation.y = -ANGLE(90.0f);
		}
	}
	else if (xBlock == newXblock)
	{
		if (zBlock < newZblock)
		{
			item->Pose.Position.z = (newZblock * BLOCK(1)) - shift;
			item->Pose.Orientation.y = 0;
		}
		else
		{
			item->Pose.Position.z = (zBlock * BLOCK(1)) + shift;
			item->Pose.Orientation.y = -ANGLE(180.0f);
		}
	}

	item->Pose.Position.y = y;
	item->Floor = y;

	if (roomNumber != item->RoomNumber)
		ItemNewRoom(itemNumber, roomNumber);

	return vault;
}

void GetAITarget(CreatureInfo* creature)
{
	auto* enemy = creature->Enemy.Get();

	short enemyObjectNumber;
	if (enemy)
		enemyObjectNumber = enemy->ObjectNumber;
	else
		enemyObjectNumber = NO_VALUE;

	auto* item = &g_Level.Items[creature->ItemNumber];

	if (item->AIBits & GUARD)
	{
		creature->Enemy = LaraItem;
		if (creature->Alerted)
		{
			item->AIBits &= ~GUARD;
			if (item->AIBits & AMBUSH)
				item->AIBits |= MODIFY;
		}
	}
	else if (item->AIBits & PATROL1)
	{
		if (creature->Alerted || creature->HurtByLara)
		{
			item->AIBits &= ~PATROL1;
			if (item->AIBits & AMBUSH)
				item->AIBits |= MODIFY;
		}
		else if (!creature->Patrol)
		{
			if (enemyObjectNumber != ID_AI_PATROL1)
				FindAITargetObject(creature, ID_AI_PATROL1);
		}
		else
		{
			if (enemyObjectNumber != ID_AI_PATROL2)
				FindAITargetObject(creature, ID_AI_PATROL2);
		}
		
		if ((enemyObjectNumber == ID_AI_PATROL1 || enemyObjectNumber == ID_AI_PATROL2) &&
			Vector3i::Distance(enemy->Pose.Position, item->Pose.Position) < REACHED_GOAL_RADIUS)
		{
			TestTriggers(enemy, true);
			creature->Patrol = !creature->Patrol;
		}
	}
	else if (item->AIBits & AMBUSH)
	{
		if (enemyObjectNumber != ID_AI_AMBUSH)
		{
			FindAITargetObject(creature, ID_AI_AMBUSH);
		}
		else if (Vector3i::Distance(enemy->Pose.Position, item->Pose.Position) < REACHED_GOAL_RADIUS)
		{
			TestTriggers(enemy, true);
			creature->ReachedGoal = true;
			creature->Enemy = LaraItem;
			item->AIBits &= ~AMBUSH;

			if (item->AIBits != MODIFY)
			{
				item->AIBits |= GUARD;
				creature->Alerted = false;
			}
		}
	}
	else if (item->AIBits & FOLLOW)
	{
		if (creature->HurtByLara)
		{
			creature->Enemy = LaraItem;
			creature->Alerted = true;
			item->AIBits &= ~FOLLOW;
		}
		else if (item->HitStatus)
		{
			item->AIBits &= ~FOLLOW;
		}
		else if (enemyObjectNumber != ID_AI_FOLLOW)
		{
			FindAITargetObject(creature, ID_AI_FOLLOW);
		}
		else if (Vector3i::Distance(enemy->Pose.Position, item->Pose.Position) < REACHED_GOAL_RADIUS)
		{
			creature->ReachedGoal = true;
			item->AIBits &= ~FOLLOW;
		}
	}
}

// Old TR3 way.
void FindAITarget(CreatureInfo* creature, short objectNumber)
{
	const auto& item = g_Level.Items[creature->ItemNumber];

	int i;
	ItemInfo* targetItem;
	for (i = 0, targetItem = &g_Level.Items[0]; i < g_Level.NumItems; i++, targetItem++)
	{
		if (targetItem->ObjectNumber != objectNumber)
			continue;

		if (targetItem->RoomNumber == NO_VALUE)
			continue;

		if (SameZone(creature, targetItem) &&
			targetItem->Pose.Orientation.y == item.ItemFlags[3])
		{
			creature->Enemy = targetItem;
			break;
		}
	}
}

void FindAITargetObject(CreatureInfo* creature, int objectNumber)
{
	const auto& item = g_Level.Items[creature->ItemNumber];
	FindAITargetObject(creature, objectNumber, item.ItemFlags[3], true);
}

void FindAITargetObject(CreatureInfo* creature, int objectNumber, int ocb, bool checkSameZone)
{
	if (g_Level.AIObjects.empty())
		return;

	auto& item = g_Level.Items[creature->ItemNumber];
	AI_OBJECT* foundObject = nullptr;

	for (auto& aiObject : g_Level.AIObjects)
	{
		if (aiObject.objectNumber == objectNumber &&
			aiObject.triggerFlags == ocb &&
			aiObject.roomNumber != NO_VALUE)
		{
			const int* zone = GetRuntimeZoneTable((int)creature->LOT.Zone).data();
			auto* room = &g_Level.Rooms[item.RoomNumber];

			item.BoxNumber = ResolveRuntimeBox(GetSector(room, item.Pose.Position.x - room->Position.x, item.Pose.Position.z - room->Position.z)->PathfindingBoxID);
			room = &g_Level.Rooms[aiObject.roomNumber];
			aiObject.boxNumber = ResolveRuntimeBox(GetSector(room, aiObject.pos.Position.x - room->Position.x, aiObject.pos.Position.z - room->Position.z)->PathfindingBoxID);

			if (item.BoxNumber == NO_VALUE || aiObject.boxNumber == NO_VALUE)
				continue;

			if (checkSameZone && (zone[item.BoxNumber] != zone[aiObject.boxNumber]))
				continue;

			// Don't check for same zone. Needed for Sophia Leigh.
			foundObject = &aiObject;
		}
	}

	if (foundObject == nullptr)
		return;

	auto& aiItem = *creature->AITarget;

	creature->Enemy = &aiItem;

	aiItem.ObjectNumber = foundObject->objectNumber;
	aiItem.RoomNumber = foundObject->roomNumber;
	aiItem.Pose.Position = foundObject->pos.Position;
	aiItem.Pose.Orientation.y = foundObject->pos.Orientation.y;
	aiItem.Flags = foundObject->flags;
	aiItem.TriggerFlags = foundObject->triggerFlags;
	aiItem.BoxNumber = foundObject->boxNumber;

	if (!(creature->AITarget->Flags & ItemFlags::IFLAG_TRIGGERED))
	{
		float sinY = phd_sin(creature->AITarget->Pose.Orientation.y);
		float cosY = phd_cos(creature->AITarget->Pose.Orientation.y);

		creature->AITarget->Pose.Position.x += CLICK(1) * sinY;
		creature->AITarget->Pose.Position.z += CLICK(1) * cosY;
	}
}

int TargetReachable(ItemInfo* item, ItemInfo* enemy)
{
	const auto& creature = *GetCreatureInfo(item);
	auto& room = g_Level.Rooms[enemy->RoomNumber];
	auto* floor = GetSector(&room, enemy->Pose.Position.x - room.Position.x, enemy->Pose.Position.z - room.Position.z);

	// NEW: Only update enemy box number if it is actually reachable by the enemy.
	// This prevents enemies from running to the player and attacking nothing when they are hanging or shimmying. -- Lwmte, 27.06.22

	// Check if enemy is actually in water 
	bool isEnemyInWater = TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, enemy->RoomNumber);

	bool isReachable = false;
	if (creature.LOT.Zone == ZoneType::Flyer)
	{
		// Flying creatures can reach any target.
		isReachable = true;
	}
	else if (creature.LOT.Zone == ZoneType::Water)
	{
		// Water creatures can only reach targets that are in water.
		isReachable = isEnemyInWater;
	}
	else
	{
		auto pointColl = GetPointCollision(enemy->Pose.Position, floor->RoomNumber);
		auto bounds = GameBoundingBox(item);
		isReachable = abs(enemy->Pose.Position.y - pointColl.GetFloorHeight()) < bounds.GetHeight();

		if (creature.LOT.Zone == ZoneType::Amphibious && isEnemyInWater)
		{
			// Amphibious creatures can reach targets in water in any case.
			isReachable = true;
		}
		else
		{
			// HACK: Allow player to be always reachable when jumping.
			if (enemy->IsLara() && TestState(enemy->Animation.ActiveState, JUMP_STATES))
				isReachable = true;
		}
	}

	// Don't try to chase enemy into bad boxes.
	int floorBox = ResolveRuntimeBox(floor->PathfindingBoxID);
	for (auto& box : creature.LOT.BadBoxes)
	{
		if (ResolveRuntimeBox(box.BoxNumber) == floorBox && box.Count < 0)
		{
			isReachable = false;
			break;
		}
	}

	return (isReachable ? floorBox : NO_VALUE);
}

/**
 * @brief Gathers AI information about the creature's situation relative to its enemy.
 *
 * Populates the AI_INFO structure with:
 * - zoneNumber: The zone the creature is in.
 * - enemyZone: The zone the enemy is in (if reachable).
 * - distance: Squared 2D distance to enemy (for speed, avoids sqrt).
 * - verticalDistance: Y difference to enemy.
 * - angle: Horizontal angle to enemy relative to creature's facing.
 * - xAngle: Vertical angle to enemy (for flying/swimming creatures).
 * - ahead: True if enemy is within front arc.
 * - bite: True if enemy is close enough and in front for melee attack.
 *
 * This information is used by GetCreatureMood() and individual creature AI
 * to make behavioral decisions.
 *
 * @param item Pointer to the creature item.
 * @param AI Pointer to AI_INFO structure to populate.
 */
void CreatureAIInfo(ItemInfo* item, AI_INFO* AI)
{
	if (!item->IsCreature())
		return;

	auto* object = &Objects[item->ObjectNumber];
	auto* creature = GetCreatureInfo(item);
	auto* enemy = creature->Enemy.Get();

	// Default to player if no enemy set.
	// TODO: Deal with LaraItem global.
	if (enemy == nullptr && !creature->Friendly)
	{
		enemy = LaraItem;
		creature->Enemy = LaraItem;
	}

	auto* zone = GetRuntimeZoneTable((int)creature->LOT.Zone).data();

	// Update creature's current box and zone. During room transitions/climb steps, RoomNumber can
	// briefly point at the adjacent room while the creature's feet still resolve to the previous
	// active floor box, so resolve through collision and fall back to the bottom sector if needed.
	item->BoxNumber = ResolveCreatureCurrentBox(item);
	AI->zoneNumber = (item->BoxNumber != NO_VALUE) ? zone[ResolveRuntimeBox(item->BoxNumber)] : NO_VALUE;

	// Friendly creature with no valid target: skip AI computation.
	if (enemy == nullptr)
	{
		AI->distance = AI->verticalDistance = INT_MAX;
		AI->enemyZone = NO_VALUE;
		AI->ahead = AI->bite = 0;
		AI->angle = AI->xAngle = AI->enemyFacing = 0;
		return;
	}

	// Get enemy's box (if reachable) and zone.
	enemy->BoxNumber = TargetReachable(item, enemy);
	AI->enemyZone = enemy->BoxNumber == NO_VALUE ? NO_VALUE : zone[enemy->BoxNumber];

	if (!object->nonLot)
	{
		if (enemy->BoxNumber != NO_VALUE && g_Level.PathfindingBoxes[enemy->BoxNumber].flags & creature->LOT.BlockMask)
		{
			AI->enemyZone |= BLOCKED;
		}
		else if (item->BoxNumber != NO_VALUE && 
			creature->LOT.Node[ResolveRuntimeBox(item->BoxNumber)].searchNumber == (creature->LOT.SearchNumber | SEARCH_BLOCKED))
		{
			AI->enemyZone |= BLOCKED;
		}
	}

	auto vector = PredictTargetPosition(*item, *enemy);

	// Apply pivot offset and make relative.
	vector.x -= item->Pose.Position.x + object->pivotLength * phd_sin(item->Pose.Orientation.y);
	vector.y =  item->Pose.Position.y - enemy->Pose.Position.y;
	vector.z -= item->Pose.Position.z + object->pivotLength * phd_cos(item->Pose.Orientation.y);

	short angle = phd_atan(vector.z, vector.x);

	if (vector.x > BLOCK(31.25f) || vector.x < -BLOCK(31.25f) ||
		vector.z > BLOCK(31.25f) || vector.z < -BLOCK(31.25f))
	{
		AI->distance = INT_MAX;
		AI->verticalDistance = INT_MAX;
	}
	else
	{
		if (creature->Enemy != nullptr)
		{
			// TODO: distance is squared, verticalDistance is not. Desquare distance later. -- Lwmte, 27.06.22
			AI->distance = SQUARE(vector.z) + SQUARE(vector.x); // 2D distance.
			AI->verticalDistance = vector.y;
		}
		else
			AI->distance = AI->verticalDistance = INT_MAX;
	}

	AI->angle = angle - item->Pose.Orientation.y;
	AI->enemyFacing = (angle - enemy->Pose.Orientation.y) + ANGLE(180.0f);

	vector.x = abs(vector.x);
	vector.z = abs(vector.z);

	// Makes Lara smaller.
	if (enemy->IsLara())
	{
		if (GetLaraInfo(enemy)->Control.IsLow)
			vector.y -= STEPUP_HEIGHT;
	}

	if (vector.x > vector.z)
		AI->xAngle = phd_atan(vector.x + (vector.z >> 1), vector.y);
	else
		AI->xAngle = phd_atan(vector.z + (vector.x >> 1), vector.y);

	AI->ahead = (AI->angle > -FRONT_ARC && AI->angle < FRONT_ARC);
	AI->bite = (AI->ahead && enemy->HitPoints > 0 && abs(enemy->Pose.Position.y - item->Pose.Position.y) <= CLICK(2));
}

/**
 * @brief Acts on the creature's mood to select movement targets.
 *
 * Based on the mood set by GetCreatureMood(), this function selects appropriate
 * target boxes for the creature to pathfind toward:
 *
 * - Bored: Pick random valid boxes, prefer ones that allow stalking.
 * - Attack: Target enemy's position directly.
 * - Escape: Pick random boxes that are far from and away from enemy.
 * - Stalk: Pick boxes that maintain distance but keep line of sight.
 *
 * Also updates JumpAhead and MonkeySwingAhead flags by checking the overlap
 * flags of the path to the next box.
 *
 * @param item Pointer to the creature item.
 * @param AI Pointer to AI_INFO with zone and distance data.
 * @param isViolent If true, creature is aggressive (affects target selection).
 */
void CreatureMood(ItemInfo* item, AI_INFO* AI, bool isViolent)
{
	if (!item->IsCreature())
		return;

	auto* creature = GetCreatureInfo(item);
	auto* LOT = &creature->LOT;
	auto* enemy = creature->Enemy.Get();

	// HACK: Fallback to bored mood from attack or escape mood if enemy was cleared.
	// Replaces previous "fix" with early exit, because it was breaking friendly NPC pathfinding. -- Lwmte, 24.03.25
	if (enemy == nullptr && (creature->Mood == MoodType::Attack || creature->Mood == MoodType::Escape))
		creature->Mood = MoodType::Bored;

	// TARGET SELECTION based on mood.
	// Each mood picks targets differently to create varied behavior.
	int boxNumber;
	switch (creature->Mood)
	{
	case MoodType::Bored:
		// BORED: Wander randomly, but prefer boxes that allow stalking enemy.
		// Only pick new target if we don't have one or reached current target.
		if (LOT->RequiredBox == NO_VALUE || item->BoxNumber == LOT->TargetBox)
		{
			boxNumber = GetRandomBox(*LOT);
			if (ValidBox(item, AI->zoneNumber, boxNumber))
			{
				// If enemy is reachable and box allows stalking, use it (keeps creature near enemy).
				if (StalkBox(item, enemy, boxNumber) && enemy->HitPoints > 0)
				{
					TargetBox(LOT, boxNumber);
				}
				else if (LOT->RequiredBox == NO_VALUE)
				{
					TargetBox(LOT, boxNumber);
				}
			}
		}
		// If enemy is unreachable, current target is near enemy, and we should move away.
		else if (StalkBox(item, enemy, LOT->RequiredBox))
		{
			boxNumber = GetRandomBox(*LOT);
			if (ValidBox(item, AI->zoneNumber, boxNumber) && !StalkBox(item, enemy, boxNumber))
			{
				TargetBox(LOT, boxNumber);
			}
		}

		break;

	case MoodType::Attack:
	{
	// Flying creatures target enemy's upper body when enemy is not in water.
	bool isEnemyOnLand = enemy->IsLara() ?
		(GetLaraInfo(*enemy).Control.WaterStatus == WaterStatus::Dry) :
		!TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, enemy->RoomNumber);

	auto targetOffset = (LOT->Zone == ZoneType::Flyer && isEnemyOnLand) ? Vector3i(0, GetFrame(*enemy).BoundingBox.Y1, 0) : Vector3i::Zero;
	LOT->Target = PredictTargetPosition(*item, *enemy, targetOffset);
	LOT->RequiredBox = enemy->BoxNumber;

	break;
	}

	case MoodType::Escape:
		// ESCAPE: Find boxes far from and away from enemy.
		boxNumber = GetRandomBox(*LOT);

		// Only select new target if we don't have one yet.
		if (ValidBox(item, AI->zoneNumber, boxNumber) && LOT->RequiredBox == NO_VALUE)
		{
			// Good escape box - far from enemy and in retreat direction.
			if (EscapeBox(item, enemy, boxNumber))
			{
				TargetBox(LOT, boxNumber);
			}
			// Not a good escape, but if in same zone as enemy and can stalk, switch to stalking.
			// Non-violent creatures might recover courage and re-engage.
			else if (AI->zoneNumber == AI->enemyZone && StalkBox(item, enemy, boxNumber) && !isViolent)
			{
				TargetBox(LOT, boxNumber);
				creature->Mood = MoodType::Stalk;
			}
		}

		break;

	case MoodType::Stalk:
		// STALK: Maintain distance while staying in enemy's line of sight.
		// Re-evaluate if no target or current target no longer allows stalking.
		if (LOT->RequiredBox == NO_VALUE || !StalkBox(item, enemy, LOT->RequiredBox))
		{
			boxNumber = GetRandomBox(*LOT);
			if (ValidBox(item, AI->zoneNumber, boxNumber))
			{
				// Found a good stalking position.
				if (StalkBox(item, enemy, boxNumber))
				{
					TargetBox(LOT, boxNumber);
				}
				// No good stalk box - just wander, maybe go back to bored.
				else if (LOT->RequiredBox == NO_VALUE)
				{
					TargetBox(LOT, boxNumber);
					// Lost access to enemy's zone - give up stalking.
					if (AI->zoneNumber != AI->enemyZone)
						creature->Mood = MoodType::Bored;
				}
			}
		}

		break;
	}

	// Process bad box checks.
	UpdateBadBoxes(item);

	// Fallback: if no target box, use creature's current box.
	if (LOT->TargetBox == NO_VALUE)
		TargetBox(LOT, item->BoxNumber);

	// Calculate the actual world position to move toward.
	CalculateTarget(&creature->Target, item, &creature->LOT);

	// CHECK FOR SPECIAL TRAVERSAL on path to next box.
	// These flags tell the creature AI if it needs to jump or monkeyswing.
	creature->JumpAhead = false;
	creature->MonkeySwingAhead = false;

	if (item->BoxNumber != NO_VALUE)
	{
		int currentBox = ResolveRuntimeBox(item->BoxNumber);

		// Get the next box on the path to target.
		int endBox = LOT->Node[currentBox].exitBox;
		if (endBox != NO_VALUE)
		{
			// Find the overlap that connects current box to exit box.
			int overlapIndex = g_Level.PathfindingBoxes[currentBox].overlapIndex;
			int nextBox = 0;
			int flags = 0;

			// Search through overlaps until we find the one leading to exitBox.
			if (overlapIndex >= 0)
			{
				do
				{
					nextBox = ResolveRuntimeBox(g_Level.Overlaps[overlapIndex].box);
					flags = g_Level.Overlaps[overlapIndex++].flags;
				} while (nextBox != NO_VALUE && ((flags & OVERLAP_END_BIT) == false) && (nextBox != endBox));
			}

			// If we found the exit overlap, check its traversal flags.
			if (nextBox == endBox)
			{
				if (flags & OVERLAP_JUMP)
					creature->JumpAhead = true;

				if (flags & OVERLAP_MONKEY)
					creature->MonkeySwingAhead = true;
			}
		}
	}
}

/**
 * @brief Determines the creature's mood based on its situation.
 *
 * MOOD TYPES:
 * - Bored: Wandering randomly, not actively engaged.
 * - Attack: Directly pursuing and attacking the enemy.
 * - Escape: Fleeing from the enemy.
 * - Stalk: Following the enemy at a distance, waiting to attack.
 *
 * MOOD TRANSITIONS:
 * - Bored/Stalk -> Attack: When in same zone as enemy and close enough.
 * - Attack -> Escape: When hit and random chance triggers.
 * - Attack -> Bored: When enemy is too far or in different zone.
 * - Escape -> Stalk: Random recovery chance when in same zone.
 *
 * isViolent creatures skip stalking and go straight to attack/escape.
 *
 * @param item Pointer to the creature item.
 * @param AI Pointer to AI_INFO with zone and distance data.
 * @param isViolent If true, creature is aggressive (no stalking behavior).
 */
void GetCreatureMood(ItemInfo* item, AI_INFO* AI, bool isViolent)
{
	if (!item->IsCreature())
		return;

	auto* creature = GetCreatureInfo(item);
	auto* enemy = creature->Enemy.Get();
	auto* LOT = &creature->LOT;

	// Clear target if creature is in a blocked box.
	int currentBox = ResolveRuntimeBox(item->BoxNumber);
	if (currentBox == NO_VALUE || creature->LOT.Node[currentBox].searchNumber == (creature->LOT.SearchNumber | SEARCH_BLOCKED))
		creature->LOT.RequiredBox = NO_VALUE;

	// Clear target if it's no longer valid (different zone, blocked, etc.)
	if (creature->Mood != MoodType::Attack && creature->LOT.RequiredBox != NO_VALUE && !ValidBox(item, AI->zoneNumber, creature->LOT.TargetBox))
	{
		if (AI->zoneNumber == AI->enemyZone)
			creature->Mood = MoodType::Bored;

		creature->LOT.RequiredBox = NO_VALUE;
	}

	// Store current mood to detect changes later.
	auto mood = creature->Mood;

	// MOOD DECISION LOGIC
	// Based on mood override, enemy state, creature state, and zone relationships.
	if (creature->ForcedMood.has_value())
	{
		// Override the mood with forced mood.
		auto forcedMood = creature->ForcedMood.value();
		if (enemy == nullptr && forcedMood != MoodType::Bored)
			forcedMood = MoodType::Bored;

		creature->Mood = forcedMood;
	}
	else if (enemy)
	{
		// Enemy is dead - go back to idle wandering.
		if (enemy->HitPoints <= 0 && enemy == LaraItem) // TODO: deal with LaraItem global !
		{
			creature->Mood = MoodType::Bored;
		}
		else if (isViolent)
		{
			// VIOLENT CREATURE BEHAVIOR
			// These creatures are highly aggressive - they don't stalk, only attack or flee.
			switch (creature->Mood)
			{
			case MoodType::Bored:
			case MoodType::Stalk:
				// Same zone = can reach enemy, so attack immediately.
				if (AI->zoneNumber == AI->enemyZone)
					creature->Mood = MoodType::Attack;
				// Got hit but can't reach enemy - flee instead.
				else if (item->HitStatus)
					creature->Mood = MoodType::Escape;

				break;

			case MoodType::Attack:
				// Lost access to enemy's zone - give up and wander.
				if (AI->zoneNumber != AI->enemyZone)
					creature->Mood = MoodType::Bored;

				break;

			case MoodType::Escape:
				// Regained access to enemy - attack again (no stalking for violent).
				if (AI->zoneNumber == AI->enemyZone)
					creature->Mood = MoodType::Attack;

				break;
			}
		}
		else
		{
			switch (creature->Mood)
			{
			case MoodType::Bored:
			case MoodType::Stalk:
				if (creature->Alerted && AI->zoneNumber != AI->enemyZone)
				{
					if (AI->distance > BLOCK(3))
						creature->Mood = MoodType::Stalk;
					else
						creature->Mood = MoodType::Bored;
				}
				else if (AI->zoneNumber == AI->enemyZone)
				{
					if (AI->distance < ATTACK_RANGE || (creature->Mood == MoodType::Stalk && LOT->RequiredBox == NO_VALUE))
						creature->Mood = MoodType::Attack;
					else
						creature->Mood = MoodType::Stalk;
				}

				break;

			case MoodType::Attack:
				if (item->HitStatus &&
					(GetRandomControl() < ESCAPE_CHANCE ||
						AI->zoneNumber != AI->enemyZone))
					creature->Mood = MoodType::Stalk;
				else if (AI->zoneNumber != AI->enemyZone && AI->distance > BLOCK(6))
					creature->Mood = MoodType::Bored;

				break;

			case MoodType::Escape:
				if (AI->zoneNumber == AI->enemyZone && GetRandomControl() < RECOVER_CHANCE)
					creature->Mood = MoodType::Stalk;

				break;
			}
		}
	}
	else
	{
		creature->Mood = MoodType::Bored;
	}

	if (mood != creature->Mood)
	{
		if (mood == MoodType::Attack)
		{
			TargetBox(LOT, LOT->TargetBox);
			LOT = &creature->LOT;
		}

		LOT->RequiredBox = NO_VALUE;
	}
}

Vector3i PredictTargetPosition(ItemInfo& sourceItem, ItemInfo& targetItem, Vector3i targetOffset)
{
	constexpr float PREDICTION_MIN_DISTANCE = BLOCK(0.5f);
	constexpr float PREDICTION_MAX_DISTANCE = BLOCK(6);
	constexpr float PREDICTION_SMOOTHING_FACTOR = 0.25f;
	constexpr float PREDICTION_WATER_SCALING_FACTOR = 1.25f;

	auto sourcePos = sourceItem.Pose.Position;
	auto targetPos = targetItem.Pose.Position;
	targetPos += targetOffset;

	auto predictionFactor = g_GameFlow->GetSettings()->Pathfinding.PredictionFactor;

	if (!sourceItem.IsCreature() || predictionFactor <= EPSILON)
		return targetPos;

	if (Objects[sourceItem.ObjectNumber].nonLot)
		return targetPos;

	float distance = Vector3i::Distance(targetPos, sourcePos);
	float distanceScale = 1.0f;

	// Calculate smooth prediction distance scale.
	if (distance < PREDICTION_MIN_DISTANCE)
	{
		distanceScale = distance / PREDICTION_MIN_DISTANCE;
	}
	else if (distance > PREDICTION_MAX_DISTANCE)
	{
		float over = (distance - PREDICTION_MAX_DISTANCE) / PREDICTION_MAX_DISTANCE;
		distanceScale = 1.0f - std::clamp(over, 0.0f, 1.0f);
	}

	// Disable prediction at close and far ranges.
	if (distanceScale <= EPSILON)
		return targetPos;

	auto& LOT = GetCreatureInfo(&sourceItem)->LOT;

	// Scale up prediction factor underwater.
	if (TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, &sourceItem) &&
		TestEnvironment(RoomEnvFlags::ENV_FLAG_WATER, &targetItem))
		predictionFactor *= PREDICTION_WATER_SCALING_FACTOR;

	// Return target without prediction, if factor is zero.
	if (predictionFactor <= EPSILON)
		return targetPos;

	auto sourceVel = GetVelocity(sourceItem);
	auto targetVel = GetVelocity(targetItem);

	float relativeVel = Vector3::Distance(targetVel, sourceVel);

	// Avoid division by zero / jitter.
	float t = 0.0f;
	if (relativeVel > 1.0f)
		t = distance / relativeVel;

	// Clamp prediction horizon (important for stability).
	t = std::clamp(t, 0.0f, predictionFactor);

	// Calculate predicted position delta.
	auto predictedPos = targetPos + targetVel * t;
	auto predictedDelta = predictedPos - targetPos;

	// Don't predict target if it is moving away from the source.
	auto toTarget = (targetPos - sourcePos).ToVector3();
	toTarget.Normalize();

	auto targetVelDir = targetVel;
	targetVelDir.Normalize();

	float alignmentDot = toTarget.Dot(targetVelDir);
	if (alignmentDot > 0.0f)
	{
		alignmentDot = CUBE(alignmentDot);
		predictedDelta *= (1.0f - std::clamp(alignmentDot, 0.0f, 1.0f));
	}

	predictedDelta *= distanceScale;
	predictedPos = targetPos + predictedDelta;

	// Force original target position if predicted position is out of bounds.
	auto pointColl = GetPointCollision(predictedPos, targetItem.RoomNumber);

	auto noBox = pointColl.GetSector().PathfindingBoxID == NO_VALUE;
	auto blockedByFloor = pointColl.GetFloorHeight() < (predictedPos.y - CLICK(1)) ||
						  pointColl.GetCeilingHeight() > (predictedPos.y - CLICK(1));

	auto finalTarget = (blockedByFloor || noBox) ? targetPos : predictedPos;

	// Smoothly interpolate target.
	if (Vector3i::Distance(LOT.Target, finalTarget) < TARGET_DEVIATION_THRESHOLD)
		return LOT.Target + (finalTarget - LOT.Target) * PREDICTION_SMOOTHING_FACTOR;
	else
		return finalTarget;
}

/**
 * @brief Calculates the world position for creature movement along its path.
 *
 * This function walks the path from creature's current box to the target box,
 * finding the best position to move toward. It handles box boundary clipping
 * to ensure the creature moves along valid paths through connected boxes.
 *
 * CLIPPING SYSTEM:
 * - Uses directional flags (CLIP_LEFT/RIGHT/TOP/BOTTOM) to track valid movement directions.
 * - Shrinks the valid corridor as it traverses boxes to find the optimal path.
 * - CLIP_SECONDARY indicates a complex path requiring intermediate target.
 *
 * RETURN VALUES:
 * - PRIME_TARGET: Reached the target box, use LOT->Target position.
 * - SECONDARY_TARGET: Path turns, use intermediate position.
 * - NO_TARGET: Couldn't reach target, use current box position.
 *
 * @param target Output world position to move toward.
 * @param item The creature item.
 * @param LOT The creature's pathfinding data.
 * @return TARGET_TYPE indicating the quality of target found.
 */
TARGET_TYPE CalculateTarget(Vector3i* target, ItemInfo* item, LOTInfo* LOT)
{
	// Set creature's current box for A* heuristic.
	LOT->SourceBox = ResolveRuntimeBox(item->BoxNumber);

	// Expand the pathfinding search if needed.
	UpdateLOT(LOT, g_GameFlow->GetSettings()->Pathfinding.SearchDepth);

	// Start with creature's current position as default target.
	*target = item->Pose.Position;

	int boxNumber = ResolveRuntimeBox(item->BoxNumber);
	if (boxNumber == NO_VALUE)
		return TARGET_TYPE::NO_TARGET;

	auto* box = &g_Level.PathfindingBoxes[boxNumber];

	// Convert box boundaries to world coordinates.
	// Note: box coordinates are in blocks, multiply by BLOCK(1) for world units.
	int boxLeft = ((int)box->left * BLOCK(1));
	int boxRight = ((int)box->right * BLOCK(1)) - 1;
	int boxTop = ((int)box->top * BLOCK(1));
	int boxBottom = ((int)box->bottom * BLOCK(1)) - 1;

	// Track the valid corridor as we traverse boxes.
	int left = boxLeft;
	int right = boxRight;
	int top = boxTop;
	int bottom = boxBottom;
	int direction = CLIP_ALL; // Can move in all directions initially.

	// Safety limit to prevent infinite loops from corrupted exitBox chains.
	int maxIterations = (int)g_Level.PathfindingBoxes.size();
	int iterations = 0;

	// MAIN LOOP: Walk along the path from current box to target box.
	do
	{
		if (++iterations > maxIterations)
			break;

		box = &g_Level.PathfindingBoxes[boxNumber];

		// Clamp target Y to box height.
		// Flying creatures stay above the floor.
		if (LOT->Fly != NO_FLYING)
		{
			if (target->y > box->height - BLOCK(1))
				target->y = box->height - BLOCK(1);
		}
		else if (target->y > box->height)
		{
			target->y = box->height;
		}

		// Get current box boundaries.
		boxLeft = ((int)box->left * BLOCK(1));
		boxRight = ((int)box->right * BLOCK(1)) - 1;
		boxTop = ((int)box->top * BLOCK(1));
		boxBottom = ((int)box->bottom * BLOCK(1)) - 1;

		// Check if creature is inside this box.
		if (item->Pose.Position.z >= boxLeft &&
			item->Pose.Position.z <= boxRight &&
			item->Pose.Position.x >= boxTop &&
			item->Pose.Position.x <= boxBottom)
		{
			// Inside box - reset corridor bounds to this box.
			left = ((int)box->left * BLOCK(1));
			right = ((int)box->right * BLOCK(1)) - 1;
			top = ((int)box->top * BLOCK(1));
			bottom = ((int)box->bottom * BLOCK(1)) - 1;
		}
		else
		{
			// DIRECTION CLIPPING: Creature outside box - narrow corridor.
			// This calculates movement direction by checking which edge of each box
			// the creature needs to cross to stay on the path.

			// Z-AXIS CLIPPING (LEFT/RIGHT)
			if (item->Pose.Position.z < boxLeft && direction != CLIP_RIGHT)
			{
				if ((direction & CLIP_LEFT) &&
					item->Pose.Position.x >= boxTop &&
					item->Pose.Position.x <= boxBottom)
				{
					if (target->z < (boxLeft + CLICK(2)))
						target->z = boxLeft + CLICK(2);

					if (direction & CLIP_SECONDARY)
						return TARGET_TYPE::SECONDARY_TARGET;

					if (boxTop > top)
						top = boxTop;

					if (boxBottom < bottom)
						bottom = boxBottom;

					direction = CLIP_LEFT;
				}
				else if (direction != CLIP_LEFT)
				{
					target->z = (right - CLICK(2));

					if (direction != CLIP_ALL)
						return TARGET_TYPE::SECONDARY_TARGET;

					direction |= (CLIP_ALL | CLIP_SECONDARY);
				}
			}
			else if (item->Pose.Position.z > boxRight && direction != CLIP_LEFT)
			{
				if ((direction & CLIP_RIGHT) &&
					item->Pose.Position.x >= boxTop &&
					item->Pose.Position.x <= boxBottom)
				{
					if (target->z > boxRight - CLICK(2))
						target->z = boxRight - CLICK(2);

					if (direction & CLIP_SECONDARY)
						return TARGET_TYPE::SECONDARY_TARGET;

					if (boxTop > top)
						top = boxTop;

					if (boxBottom < bottom)
						bottom = boxBottom;

					direction = CLIP_RIGHT;
				}
				else if (direction != CLIP_RIGHT)
				{
					target->z = left + CLICK(2);

					if (direction != CLIP_ALL)
						return TARGET_TYPE::SECONDARY_TARGET;

					direction |= (CLIP_ALL | CLIP_SECONDARY);
				}
			}

			if (item->Pose.Position.x < boxTop && direction != CLIP_BOTTOM)
			{
				if ((direction & CLIP_TOP) &&
					item->Pose.Position.z >= boxLeft &&
					item->Pose.Position.z <= boxRight)
				{
					if (target->x < boxTop + CLICK(2))
						target->x = boxTop + CLICK(2);

					if (direction & CLIP_SECONDARY)
						return TARGET_TYPE::SECONDARY_TARGET;

					if (boxLeft > left)
						left = boxLeft;

					if (boxRight < right)
						right = boxRight;

					direction = CLIP_TOP;
				}
				else if (direction != CLIP_TOP)
				{
					target->x = bottom - CLICK(2);

					if (direction != CLIP_ALL)
						return TARGET_TYPE::SECONDARY_TARGET;

					direction |= (CLIP_ALL | CLIP_SECONDARY);
				}
			}
			else if (item->Pose.Position.x > boxBottom && direction != CLIP_TOP)
			{
				if ((direction & CLIP_BOTTOM) &&
					item->Pose.Position.z >= boxLeft &&
					item->Pose.Position.z <= boxRight)
				{
					if (target->x > (boxBottom - CLICK(2)))
						target->x = (boxBottom - CLICK(2));

					if (direction & CLIP_SECONDARY)
						return TARGET_TYPE::SECONDARY_TARGET;

					if (boxLeft > left)
						left = boxLeft;

					if (boxRight < right)
						right = boxRight;

					direction = CLIP_BOTTOM;
				}
				else if (direction != CLIP_BOTTOM)
				{
					target->x = top + CLICK(2);

					if (direction != CLIP_ALL)
						return TARGET_TYPE::SECONDARY_TARGET;

					direction |= (CLIP_ALL | CLIP_SECONDARY);
				}
			}
		}

		// REACHED TARGET BOX: Calculate final target position.
		if (boxNumber == LOT->TargetBox)
		{
			// Use LOT target Z if path was moving left/right.
			if (direction & (CLIP_LEFT | CLIP_RIGHT))
			{
				target->z = LOT->Target.z;
			}
			// Otherwise clamp to box boundaries.
			else if (!(direction & CLIP_SECONDARY))
			{
				if (target->z < (boxLeft + CLICK(2)))
					target->z = boxLeft + CLICK(2);
				else if (target->z > (boxRight - CLICK(2)))
					target->z = boxRight - CLICK(2);
			}

			// Use LOT target X if path was moving top/bottom.
			if (direction & (CLIP_TOP | CLIP_BOTTOM))
			{
				target->x = LOT->Target.x;
			}
			// Otherwise clamp to box boundaries.
			else if (!(direction & CLIP_SECONDARY))
			{
				if (target->x < (boxTop + CLICK(2)))
					target->x = boxTop + CLICK(2);
				else if (target->x > (boxBottom - CLICK(2)))
					target->x = boxBottom - CLICK(2);
			}

			target->y = LOT->Target.y;

			return TARGET_TYPE::PRIME_TARGET;
		}

		// Move to next box on path.
		int nextBox = LOT->Node[boxNumber].exitBox;

		// Stop if next box is invalid, blocked, or not part of current search.
		if (nextBox == NO_VALUE)
			break;

		if (g_Level.PathfindingBoxes[nextBox].flags & LOT->BlockMask)
			break;

		// Verify node belongs to current search (prevents following stale exitBox values).
		if ((LOT->Node[nextBox].searchNumber & SEARCH_NUMBER) != (LOT->SearchNumber & SEARCH_NUMBER))
			break;

		boxNumber = nextBox;

	} while (true);

	// FALLBACK: Couldn't reach target box.
	// Clamp position to last valid box boundaries.
	if (!(direction & CLIP_SECONDARY))
	{
		if (target->z < (boxLeft + CLICK(2)))
			target->z = boxLeft + CLICK(2);
		else if (target->z > (boxRight - CLICK(2)))
			target->z = boxRight - CLICK(2);
	}

	if (!(direction & CLIP_SECONDARY))
	{
		if (target->x < (boxTop + CLICK(2)))
			target->x = boxTop + CLICK(2);
		else if (target->x > (boxBottom - CLICK(2)))
			target->x = boxBottom - CLICK(2);
	}

	// Set Y to floor height, flying creatures slightly above.
	if (LOT->Fly == NO_FLYING)
		target->y = box->height;
	else
		target->y = box->height - STEPUP_HEIGHT;

	return TARGET_TYPE::NO_TARGET;
}

void AdjustStopperFlag(ItemInfo* item, int direction)
{
	int x = item->Pose.Position.x;
	int z = item->Pose.Position.z;

	auto* room = &g_Level.Rooms[item->RoomNumber];
	auto* floor = GetSector(room, x - room->Position.x, z - room->Position.z);
	floor->Stopper = !floor->Stopper;

	x = item->Pose.Position.x + BLOCK(1) * phd_sin(direction);
	z = item->Pose.Position.z + BLOCK(1) * phd_cos(direction);
	room = &g_Level.Rooms[GetPointCollision(Vector3i(x, item->Pose.Position.y, z), item->RoomNumber).GetRoomNumber()];

	floor = GetSector(room, x - room->Position.x, z - room->Position.z);
	floor->Stopper = !floor->Stopper;
}

void InitializeItemBoxData()
{
	for (int i = 0; i < g_Level.Items.size(); i++)
	{
		auto* currentItem = &g_Level.Items[i];
	}

	for (auto& room : g_Level.Rooms)
	{
		for (const auto& mesh : room.mesh)
		{
			long index = ((mesh.Pose.Position.z - room.Position.z) / BLOCK(1)) + room.ZSize * ((mesh.Pose.Position.x - room.Position.x) / BLOCK(1));
			if (index >= room.Sectors.size())
				continue;

			auto* floor = &room.Sectors[index];
			if (floor->PathfindingBoxID == NO_VALUE)
				continue;

			if (!(g_Level.PathfindingBoxes[floor->PathfindingBoxID].flags & BLOCKED))
			{
				int floorHeight = floor->GetSurfaceHeight(mesh.Pose.Position.x, mesh.Pose.Position.z, true);
				const auto& bBox = GetBoundsAccurate(mesh, false);

				if (floorHeight <= mesh.Pose.Position.y - bBox.Y2 + CLICK(2) &&
					floorHeight < mesh.Pose.Position.y - bBox.Y1)
				{
					if (bBox.X1 == 0 || bBox.X2 == 0 || bBox.Z1 == 0 || bBox.Z2 == 0 ||
					  ((bBox.X1 < 0) ^ (bBox.X2 < 0)) && ((bBox.Z1 < 0) ^ (bBox.Z2 < 0)))
					{
						floor->Stopper = true;
					}
				}
			}
		}
	}
	// Normalize alternate-room portals before building mixed flip-state pathfinding data.
	RemapAlternateRoomPortals();

	// Build per-box flip metadata and compute the runtime zone table for the current
	// (load-time, unflipped) flip combination. Recomputed on every DoFlipMap.
	BuildPathfindingFlipMetadata();
	RecomputeRuntimeZones();
}

bool CanCreatureJump(ItemInfo& item, JumpDistance jumpDistType)
{
	const auto& creature = *GetCreatureInfo(&item);
	if (creature.Enemy == nullptr)
		return false;

	float stepDist = BLOCK(0.92f);

	int vPos = item.Pose.Position.y;
	auto pointCollA = GetPointCollision(item, item.Pose.Orientation.y, stepDist);
	auto pointCollB = GetPointCollision(item, item.Pose.Orientation.y, stepDist * 2);
	auto pointCollC = GetPointCollision(item, item.Pose.Orientation.y, stepDist * 3);

	switch (jumpDistType)
	{
	default:
	case JumpDistance::Block1:
		if (item.BoxNumber == creature.Enemy->BoxNumber ||
			vPos >= (pointCollA.GetFloorHeight() - STEPUP_HEIGHT) ||
			vPos >= (pointCollB.GetFloorHeight() + CLICK(1)) ||
			vPos <= (pointCollB.GetFloorHeight() - CLICK(1)) ||
			pointCollA.GetSector().PathfindingBoxID == NO_VALUE ||
			pointCollB.GetSector().PathfindingBoxID == NO_VALUE)
		{
			return false;
		}

		break;

	case JumpDistance::Block2:
		if (item.BoxNumber == creature.Enemy->BoxNumber ||
			vPos >= (pointCollA.GetFloorHeight() - STEPUP_HEIGHT) ||
			vPos >= (pointCollB.GetFloorHeight() - STEPUP_HEIGHT) ||
			vPos >= (pointCollC.GetFloorHeight() + CLICK(1)) ||
			vPos <= (pointCollC.GetFloorHeight() - CLICK(1)) ||
			pointCollA.GetSector().PathfindingBoxID == NO_VALUE ||
			pointCollB.GetSector().PathfindingBoxID == NO_VALUE ||
			pointCollC.GetSector().PathfindingBoxID == NO_VALUE)
		{
			return false;
		}

		break;
	}

	return true;
}
