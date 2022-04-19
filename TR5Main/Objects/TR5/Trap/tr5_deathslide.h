#pragma once
#include "Game/items.h"
#include "Game/collision/collide_room.h"

void InitialiseDeathSlide(short itemNumber);
void DeathSlideCollision(short itemNumber, ITEM_INFO* l, CollisionInfo* coll);
void ControlDeathSlide(short itemNumber);