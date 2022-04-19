#pragma once
#include "Game/collision/collide_room.h"

// -----------------------------
// UNDERWATER SWIM
// Control & Collision Functions
// -----------------------------

void lara_as_underwater_idle(ITEM_INFO* item, CollisionInfo* coll);
void lara_col_underwater_idle(ITEM_INFO* item, CollisionInfo* coll);
void lara_as_underwater_swim_forward(ITEM_INFO* item, CollisionInfo* coll);
void lara_col_underwater_swim_forward(ITEM_INFO* item, CollisionInfo* coll);
void lara_as_underwater_inertia(ITEM_INFO* item, CollisionInfo* coll);
void lara_col_underwater_inertia(ITEM_INFO* item, CollisionInfo* coll);
void lara_as_underwater_death(ITEM_INFO* item, CollisionInfo* coll);
void lara_col_underwater_death(ITEM_INFO* item, CollisionInfo* coll);
void lara_as_underwater_roll_180(ITEM_INFO* item, CollisionInfo* coll);
void lara_col_underwater_roll_180(ITEM_INFO* item, CollisionInfo* coll);

