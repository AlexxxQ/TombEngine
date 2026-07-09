#pragma once

#include "Game/control/box.h"

struct CreatureInfo;
struct ItemInfo;
struct LOTInfo;

// Underwater / amphibious creature navigation helpers, shared by the pathfinding
// core (box.cpp) and water creatures (crocodile, shark, barracuda, big rat, ...).

// Box has water or shallow-water flags.
bool IsWetBox(const BOX_INFO& box);

// Creature belongs to a water-capable zone (Water or Amphibious).
bool IsWaterZone(const LOTInfo* LOT);

// Diver: water-capable zone with 3D swim movement enabled (Fly != NO_FLYING).
bool IsSwimmer(const LOTInfo* LOT);

// Creature's room carries the water flag.
bool IsItemInWaterRoom(const ItemInfo* item);

// Enemy is on dry land (Lara: WaterStatus == Dry; others: room has no water flag).
bool IsEnemyOnLand(const ItemInfo* enemy);

// True if the BFS chain from startBox toward LOT->TargetBox contains a vertical
// transition (consecutive box height delta above threshold).
bool ChainHasVerticalTransition(const LOTInfo* LOT, int startBox, int threshold);

// True if a water creature should use OG-faithful Y logic: swimming, Bored mood,
// flat chain. Then the breadcrumb is skipped and CreaturePathfind drops the
// per-tick descent clamp -- smooth TR4-style cruise.
bool ShouldUseOgWaterYMode(const CreatureInfo* creature, const ItemInfo* item, const LOTInfo* LOT);

// Cruise altitude for a water/amphibious creature over the given box: 1 block above
// the floor, randomized up to just below the water surface (probed at the item),
// clamped 2 clicks under the ceiling at the target column. Reads LOT->Target.x/z,
// so call it after the target XZ is set.
int GetWaterCruiseY(ItemInfo* item, LOTInfo* LOT, const BOX_INFO& box);

// LOS to the enemy for water-navigation consumers (direct homing, breadcrumb gate,
// box-center override). Computed only for water-zone creatures with an enemy.
bool GetSwimNavLos(ItemInfo* item, ItemInfo* enemy, const LOTInfo* LOT);

// Water-navigation target override: clear-LOS direct homing at an in-water enemy,
// or breadcrumb steering along the box chain (vertical portals, water exit).
// Returns true with *target set (treat as PRIME_TARGET); false = regular corridor walk.
bool TryGetSwimTarget(Vector3i* target, ItemInfo* item, CreatureInfo* creature, ItemInfo* enemy, LOTInfo* LOT, bool ogYMode, bool losToEnemy);

// Shared LOT setup for amphibious creatures: swim step/drop when the creature is in
// water, land values otherwise. Fly is passed in (callers gate it on submersion depth).
void SetAmphibiousLOT(CreatureInfo& creature, bool grantSwimStepDrop, int flyRate, int landStep, int landDrop);

// Time-based unstuck for swimmers: if the creature sits in the same box too long while a
// target box is pending, cool the exitBox (likely a phantom overlap) and reflood around it.
// The per-frame BadBox penalty never accumulates underwater (XZ swim oscillation).
void TryUnstuckSwimmer(ItemInfo* item, CreatureInfo* creature, LOTInfo* LOT);

// Guard for the BadBox penalty: true if the box lies on a 3D-mover's active route
// (required box, current box, next box on the chain). Swimmers bump ceilings/surfaces
// in tight portal boxes; penalizing the transit box makes BFS detour "through walls".
// Genuine stalls are recovered by TryUnstuckSwimmer instead.
bool IsBoxOnSwimmerRoute(const LOTInfo* LOT, int boxNumber);

// Asymmetric descent clamp for swimmers (Water + Amphibious): never dive below the
// floor under the current XZ (minus 1 click). Prevents crashing into a shallow local
// floor while chasing a deep target past a floor portal -- the dive unblocks once
// horizontal motion carries the creature over the deep sector. No-op in OG Y mode
// (flat bored cruise sets a safe altitude itself). Returns the clamped fly rate.
int ClampSwimDescent(const ItemInfo* item, const CreatureInfo* creature, const LOTInfo* LOT, int flyRate, int floorHeight);

// Water + Attack + enemy-on-land target override: LOT->Target lies above the surface
// in a dry room; chasing its XZ jams the creature at the ceiling portal (surface clamp
// reverts, next tick re-targets -- endless bobbing). Aims at the centre of the last
// reachable water box instead. Returns true with *target set (treat as PRIME_TARGET).
bool TryGetWaterBoxCenterTarget(Vector3i* target, const CreatureInfo* creature, const ItemInfo* enemy, const LOTInfo* LOT, int boxNumber, bool ogYMode, bool losToEnemy);
