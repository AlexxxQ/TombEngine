#include "framework.h"
#include "tr4_jeanyves.h"
#include "setup.h"
#include "level.h"
#include "lara.h"
#include "control.h"

void InitialiseJeanYves(short itemNumber)
{
	ITEM_INFO* item = &Items[itemNumber];
	ObjectInfo* obj = &Objects[item->objectNumber];
	
	item->goalAnimState = 1;
	item->currentAnimState = 1;
	item->animNumber = obj->animIndex;
	item->frameNumber = Anims[item->animNumber].frameBase;
}

void JeanYvesControl(short itemNumber)
{
	ITEM_INFO* item = &Items[itemNumber];

	if (item->triggerFlags >= Lara.highestLocation)
	{
		short state = 0;

		if (GetRandomControl() & 3)
			state = (GetRandomControl() & 1) + 1;
		else
			state = 3 * (GetRandomControl() & 1);

		item->goalAnimState = (((byte)(item->currentAnimState) - 1) & 0xC) + state + 1;
		AnimateItem(item);
	}
	else
	{
		if (Lara.highestLocation > 3)
			Lara.highestLocation = 3;

		short state = (GetRandomControl() & 3) + 4 * Lara.highestLocation;
		short animNumber = Objects[item->objectNumber].animIndex + state;
		state++;

		item->goalAnimState = item->currentAnimState = state;
		item->animNumber = animNumber;
		item->frameNumber = Anims[item->animNumber].frameBase;
		item->triggerFlags = Lara.highestLocation;

		AnimateItem(item);
	}
}