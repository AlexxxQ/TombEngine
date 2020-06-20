#include "framework.h"
#include "tr4_blade.h"
#include "level.h"
#include "collide.h"
#include "lara.h"
#include "control.h"
#include "effect.h"

void BladeCollision(short itemNum, ITEM_INFO* l, COLL_INFO* coll)
{
	ITEM_INFO* item = &Items[itemNum];

	if (item->status == ITEM_INVISIBLE)
		return;

	if (item->itemFlags[3]) // Check this
	{
		if (TestBoundsCollide(item, l, coll->radius))
		{
			int oldX = LaraItem->pos.xPos;
			int oldY = LaraItem->pos.yPos;
			int oldZ = LaraItem->pos.zPos;

			int dx = 0;
			int dy = 0;
			int dz = 0;

			if (ItemPushLara(item, l, coll, 1, 1))
			{
				LaraItem->hitPoints -= item->itemFlags[3];

				dx = oldX - LaraItem->pos.xPos;
				dy = oldY - LaraItem->pos.yPos;
				dz = oldZ - LaraItem->pos.zPos;

				if ((dx || dy || dz) && TriggerActive(item))
				{
					DoBloodSplat((GetRandomControl() & 0x3F) + l->pos.xPos - 32,
						l->pos.yPos - (GetRandomControl() & 0x1FF) - 256,
						(GetRandomControl() & 0x3F) + l->pos.zPos - 32,
						(GetRandomControl() & 3) + (item->itemFlags[3] >> 5) + 2,
						2 * GetRandomControl(),
						l->roomNumber);
				}

				if (!coll->enableBaddiePush)
				{
					LaraItem->pos.xPos += dx;
					LaraItem->pos.yPos += dy;
					LaraItem->pos.zPos += dz;
				}
			}
		}
	}
}