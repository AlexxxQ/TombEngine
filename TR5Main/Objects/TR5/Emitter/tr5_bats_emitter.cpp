#include "framework.h"
#include "Objects/TR5/Emitter/tr5_bats_emitter.h"
#include "Specific/level.h"
#include "Game/control/control.h"
#include "Specific/setup.h"
#include "Game/effects/effects.h"
#include "Game/effects/tomb4fx.h"
#include "Sound/sound.h"
#include "Game/Lara/lara.h"
#include "Game/animation.h"
#include <Game\items.h>
#include "Game/items.h"

int NextBat;
BAT_STRUCT Bats[NUM_BATS];

void InitialiseLittleBats(short itemNumber)
{
	ITEM_INFO* item = &g_Level.Items[itemNumber];

	if (item->Pose.Orientation.y == 0)
	{
		item->Pose.Position.z += 512;
	}
	else if (item->Pose.Orientation.y == -ANGLE(180))
	{
		item->Pose.Position.z -= 512;
	}
	else if (item->Pose.Orientation.y == -ANGLE(90))
	{
		item->Pose.Position.x -= 512;
	}
	else if (item->Pose.Orientation.y == ANGLE(90))
	{
		item->Pose.Position.x += 512;
	}

	if (Objects[ID_BATS_EMITTER].loaded)
		ZeroMemory(Bats, NUM_BATS * sizeof(BAT_STRUCT));

	//LOWORD(item) = sub_402F27(ebx0, Bats, 0, 1920);
}

void LittleBatsControl(short itemNumber)
{
	ITEM_INFO* item = &g_Level.Items[itemNumber];

	if (TriggerActive(item))
	{
		if (item->TriggerFlags)
		{
			TriggerLittleBat(item);
			item->TriggerFlags--;
		}
		else
		{
			KillItem(itemNumber);
		}
	}
}

short GetNextBat()
{
	short batNumber = NextBat;
	int index = 0;
	BAT_STRUCT* bat = &Bats[NextBat];

	while (bat->on)
	{
		if (batNumber == NUM_BATS - 1)
		{
			bat = (BAT_STRUCT*)Bats;
			batNumber = 0;
		}
		else
		{
			batNumber++;
			bat++;
		}

		index++;

		if (index >= NUM_BATS)
			return NO_ITEM;
	}

	NextBat = (batNumber + 1) & (NUM_BATS - 1);

	return batNumber;
}

void TriggerLittleBat(ITEM_INFO* item)
{
	short batNumber = GetNextBat();

	if (batNumber != NO_ITEM)
	{
		BAT_STRUCT* bat = &Bats[batNumber];

		bat->roomNumber = item->RoomNumber;
		bat->pos.Position.x = item->Pose.Position.x;
		bat->pos.Position.y = item->Pose.Position.y;
		bat->pos.Position.z = item->Pose.Position.z;
		bat->pos.Orientation.y = (GetRandomControl() & 0x7FF) + item->Pose.Orientation.y + -ANGLE(180) - 1024;
		bat->on = 1;
		bat->flags = 0;
		bat->pos.Orientation.x = (GetRandomControl() & 0x3FF) - 512;
		bat->speed = (GetRandomControl() & 0x1F) + 16;
		bat->laraTarget = GetRandomControl() & 0x1FF;
		bat->counter = 20 * ((GetRandomControl() & 7) + 15);
	}
}

void UpdateBats()
{
	if (!Objects[ID_BATS_EMITTER].loaded)
		return;

	BOUNDING_BOX* bounds = GetBoundsAccurate(LaraItem);

	int x1 = LaraItem->Pose.Position.x + bounds->X1 - (bounds->X1 / 4);
	int x2 = LaraItem->Pose.Position.x + bounds->X2 - (bounds->X2 / 4);

	int y1 = LaraItem->Pose.Position.y + bounds->Y1 - (bounds->Y1 / 4);
	int y2 = LaraItem->Pose.Position.y + bounds->Y1 - (bounds->Y1 / 4);

	int z1 = LaraItem->Pose.Position.z + bounds->Z1 - (bounds->Z1 / 4);
	int z2 = LaraItem->Pose.Position.z + bounds->Z1 - (bounds->Z1 / 4);

	int minDistance = MAXINT;
	int minIndex = -1;

	for (int i = 0; i < NUM_BATS; i++)
	{
		BAT_STRUCT* bat = &Bats[i];

		if (!bat->on)
			continue;

		if ((Lara.Burn || LaraItem->HitPoints <= 0)
			&& bat->counter > 90
			&& !(GetRandomControl() & 7))
			bat->counter = 90;

		if (!(--bat->counter))
		{
			bat->on = 0;
			continue;
		}

		if (!(GetRandomControl() & 7))
		{
			bat->laraTarget = GetRandomControl() % 640 + 128;
			bat->xTarget = (GetRandomControl() & 0x7F) - 64;
			bat->zTarget = (GetRandomControl() & 0x7F) - 64;
		}

		short angles[2];
		phd_GetVectorAngles(
			LaraItem->Pose.Position.x + 8 * bat->xTarget - bat->pos.Position.x,
			LaraItem->Pose.Position.y - bat->laraTarget - bat->pos.Position.y,
			LaraItem->Pose.Position.z + 8 * bat->zTarget - bat->pos.Position.z,
			angles);

		int distance = SQUARE(LaraItem->Pose.Position.z - bat->pos.Position.z) +
			SQUARE(LaraItem->Pose.Position.x - bat->pos.Position.x);
		if (distance < minDistance)
		{
			minDistance = distance;
			minIndex = i;
		}

		distance = sqrt(distance) / 8;
		if (distance < 48)
			distance = 48;
		else if (distance > 128)
			distance = 128;

		if (bat->speed < distance)
			bat->speed++;
		else if (bat->speed > distance)
			bat->speed--;

		if (bat->counter > 90)
		{
			short speed = bat->speed * 128;

			short xAngle = abs(angles[1] - bat->pos.Orientation.x) / 8;
			short yAngle = abs(angles[0] - bat->pos.Orientation.y) / 8;

			if (xAngle < -speed)
				xAngle = -speed;
			else if (xAngle > speed)
				xAngle = speed;

			if (yAngle < -speed)
				yAngle = -speed;
			else if (yAngle > speed)
				yAngle = speed;

			bat->pos.Orientation.y += yAngle;
			bat->pos.Orientation.x += xAngle;
		}

		int sp = bat->speed * phd_cos(bat->pos.Orientation.x);

		bat->pos.Position.x += sp * phd_sin(bat->pos.Orientation.y);
		bat->pos.Position.y += bat->speed * phd_sin(-bat->pos.Orientation.x);
		bat->pos.Position.z += sp * phd_cos(bat->pos.Orientation.y);

		if ((i % 2 == 0)
			&& bat->pos.Position.x > x1
			&& bat->pos.Position.x < x2
			&& bat->pos.Position.y > y1
			&& bat->pos.Position.y < y2
			&& bat->pos.Position.z > z1
			&& bat->pos.Position.z < z2)
		{
			TriggerBlood(bat->pos.Position.x, bat->pos.Position.y, bat->pos.Position.z, 2 * GetRandomControl(), 2);
			if (LaraItem->HitPoints > 0)
				LaraItem->HitPoints -= 2;
		}
	}

	if (minIndex != -1)
	{
		BAT_STRUCT* bat = &Bats[minIndex];
		if (!(GetRandomControl() & 4))
			SoundEffect(157,&bat->pos, 0);
	}
}