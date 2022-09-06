#pragma once
#include "Specific/phd_global.h"

struct ItemInfo;
struct CollisionInfo;

namespace TEN::Entities::Generic
{
	void TriggerTorchFlame(char fxObj, char node);
	void DoFlameTorch();
	void GetFlameTorch();
	void TorchControl(short itemNumber);
	void LaraTorch(Vector3i* src, Vector3i* target, int rot, int color);
	void FireCollision(short itemNumber, ItemInfo* laraItem, CollisionInfo* coll);
}
