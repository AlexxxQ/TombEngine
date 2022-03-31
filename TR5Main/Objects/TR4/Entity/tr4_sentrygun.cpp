#include "framework.h"
#include "tr4_sentrygun.h"
#include "Game/control/box.h"
#include "Game/effects/effects.h"
#include "Game/items.h"
#include "Game/gui.h"
#include "Specific/level.h"
#include "Game/control/lot.h"
#include "Game/effects/tomb4fx.h"
#include "Game/people.h"
#include "Sound/sound.h"
#include "Specific/trmath.h"
#include "Objects/objectslist.h"
#include "Game/itemdata/creature_info.h"
#include "Game/animation.h"

namespace TEN::Entities::TR4
{
	BITE_INFO sentryGunBite = { 0, 0, 0, 8 };

	static void SentryGunThrowFire(ITEM_INFO* item)
	{
		for (int i = 0; i < 3; i++)
		{
			SPARKS* spark = &Sparks[GetFreeSpark()];

			spark->on = 1;
			spark->sR = (GetRandomControl() & 0x1F) + 48;
			spark->sG = 48;
			spark->sB = 255;
			spark->dR = (GetRandomControl() & 0x3F) - 64;
			spark->dG = (GetRandomControl() & 0x3F) + -128;
			spark->dB = 32;
			spark->colFadeSpeed = 12;
			spark->fadeToBlack = 8;
			spark->transType = TransTypeEnum::COLADD;

			Vector3Int pos1;
			pos1.x = -140;
			pos1.y = -30;
			pos1.z = -4;

			GetJointAbsPosition(item, &pos1, 7);

			spark->x = (GetRandomControl() & 0x1F) + pos1.x - 16;
			spark->y = (GetRandomControl() & 0x1F) + pos1.y - 16;
			spark->z = (GetRandomControl() & 0x1F) + pos1.z - 16;

			Vector3Int pos2;
			pos2.x = -280;
			pos2.y = -30;
			pos2.z = -4;

			GetJointAbsPosition(item, &pos2, 7);

			int v = (GetRandomControl() & 0x3F) + 192;

			spark->life = spark->sLife = v / 6;

			spark->xVel = v * (pos2.x - pos1.x) / 10;
			spark->yVel = v * (pos2.y - pos1.y) / 10;
			spark->zVel = v * (pos2.z - pos1.z) / 10;

			spark->friction = 85;
			spark->gravity = -16 - (GetRandomControl() & 0x1F);
			spark->maxYvel = 0;
			spark->flags = SP_FIRE | SP_SCALE | SP_DEF | SP_ROTATE | SP_EXPDEF;

			spark->scalar = 3;
			spark->dSize = (v * ((GetRandomControl() & 7) + 60)) / 256;
			spark->sSize = spark->dSize / 4;
			spark->size = spark->dSize / 2;
		}
	}

	void InitialiseSentryGun(short itemNum)
	{
		ITEM_INFO* item = &g_Level.Items[itemNum];

		ClearItem(itemNum);

		item->ItemFlags[0] = 0;
		item->ItemFlags[1] = 768;
		item->ItemFlags[2] = 0;
	}

	void SentryGunControl(short itemNum)
	{
		ITEM_INFO* item = &g_Level.Items[itemNum];

		if (!CreatureActive(itemNum))
			return;

		CreatureInfo* creature = (CreatureInfo*)item->Data;

		AI_INFO info = {};
		int c = 0;

		if (!creature)
			return;

		// Flags set by the ID_MINE object?
		if (item->MeshBits & 0x40)
		{
			if (item->ItemFlags[0])
			{
				Vector3Int pos;

				pos.x = sentryGunBite.x;
				pos.y = sentryGunBite.y;
				pos.z = sentryGunBite.z;

				GetJointAbsPosition(item, &pos, sentryGunBite.meshNum);

				TriggerDynamicLight(pos.x, pos.y, pos.z, 4 * item->ItemFlags[0] + 12, 24, 16, 4);

				item->ItemFlags[0]--;
			}

			if (item->ItemFlags[0] & 1)
				item->MeshBits |= 0x100;
			else
				item->MeshBits &= ~0x100;

			if (item->TriggerFlags == 0)
			{
				item->Pose.Position.y -= 512;
				CreatureAIInfo(item, &info);
				item->Pose.Position.y += 512;

				int deltaAngle = info.angle - creature->JointRotation[0];

				info.ahead = true;
				if (deltaAngle <= -ANGLE(90) || deltaAngle >= ANGLE(90))
					info.ahead = false;

				if (Targetable(item, &info))
				{
					if (info.distance < SQUARE(SECTOR(9)))
					{
						if (!g_Gui.IsObjectInInventory(ID_PUZZLE_ITEM5) && !item->ItemFlags[0])
						{
							if (info.distance <= SQUARE(SECTOR(2)))
							{
								// Throw fire
								SentryGunThrowFire(item);
								c = phd_sin((GlobalCounter & 0x1F) * 2048) * 4096;
							}
							else
							{
								// Shot to Lara with bullets
								c = 0;
								item->ItemFlags[0] = 2;

								ShotLara(item, &info, &sentryGunBite, creature->JointRotation[0], 5);
								SoundEffect(SFX_TR4_AUTOGUNS, &item->Pose, 0);

								item->ItemFlags[2] += 256;
								if (item->ItemFlags[2] > 6144)
								{
									item->ItemFlags[2] = 6144;
								}
							}
						}

						deltaAngle = c + info.angle - creature->JointRotation[0];
						if (deltaAngle <= ANGLE(10))
						{
							if (deltaAngle < -ANGLE(10))
							{
								deltaAngle = -ANGLE(10);
							}
						}
						else
						{
							deltaAngle = ANGLE(10);
						}

						creature->JointRotation[0] += deltaAngle;

						CreatureJoint(item, 1, -info.xAngle);
					}
				}

				item->ItemFlags[2] -= 32;

				if (item->ItemFlags[2] < 0)
				{
					item->ItemFlags[2] = 0;
				}

				creature->JointRotation[3] += item->ItemFlags[2];
				creature->JointRotation[2] += item->ItemFlags[1];

				if (creature->JointRotation[2] > ANGLE(90) ||
					creature->JointRotation[2] < -ANGLE(90))
				{
					item->ItemFlags[1] = -item->ItemFlags[1];
				}
			}
			else
			{
				// Stuck sentry gun 
				CreatureJoint(item, 0, (GetRandomControl() & 0x7FF) - 1024);
				CreatureJoint(item, 1, ANGLE(45));
				CreatureJoint(item, 2, (GetRandomControl() & 0x3FFF) - ANGLE(45));
			}
		}
		else
		{
			ExplodingDeath(itemNum, -1, 257);
			DisableEntityAI(itemNum);
			KillItem(itemNum);

			item->Flags |= 1u;
			item->Status = ITEM_DEACTIVATED;

			RemoveAllItemsInRoom(item->RoomNumber, ID_SMOKE_EMITTER_BLACK);

			TriggerExplosionSparks(item->Pose.Position.x, item->Pose.Position.y - 768, item->Pose.Position.z, 3, -2, 0, item->RoomNumber);
			for (int i = 0; i < 2; i++)
				TriggerExplosionSparks(item->Pose.Position.x, item->Pose.Position.y - 768, item->Pose.Position.z, 3, -1, 0, item->RoomNumber);

			SoundEffect(SFX_TR4_EXPLOSION1, &item->Pose, 25165828);
			SoundEffect(SFX_TR4_EXPLOSION2, &item->Pose, 0);
		}
	}
}