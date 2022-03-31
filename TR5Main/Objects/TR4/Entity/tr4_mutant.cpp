#include "framework.h"
#include "tr4_mutant.h"
#include "tr4_locusts.h"
#include "Game/effects/effects.h"
#include "Game/misc.h"
#include "Game/Lara/lara.h"
#include "Specific/setup.h"
#include "Game/collision/sphere.h"
#include "Objects/objectslist.h"
#include "Specific/trmath.h"
#include "Game/itemdata/creature_info.h"
#include "Game/control/control.h"
#include "Game/animation.h"
#include "Game/items.h"

namespace TEN::Entities::TR4
{
	enum CARDINAL_POINT
	{
		C_NORTH = 0,
		C_NORTH_EAST = 45,
		C_EAST = 90,
		C_EAST_SOUTH = 135,
		C_SOUTH = 180,
		C_SOUTH_WEST = 225,
		C_WEST = 270,
		C_WEST_NORTH = 315
	};

	void TriggerCrocgodMissile(PHD_3DPOS* src, short roomNumber, short counter)
	{
		short fxNumber = NO_ITEM;

		fxNumber = CreateNewEffect(roomNumber);
		if (fxNumber != NO_ITEM)
		{
			auto* fx = &EffectList[fxNumber];
			fx->pos.Position.x = src->Position.x;
			fx->pos.Position.y = src->Position.y - (GetRandomControl() & 0x3F) - 32;
			fx->pos.Position.z = src->Position.z;
			fx->pos.Orientation.x = src->Orientation.x;
			fx->pos.Orientation.y = src->Orientation.y;
			fx->pos.Orientation.z = 0;
			fx->roomNumber = roomNumber;
			fx->counter = 16 * counter + 15;
			fx->objectNumber = ID_ENERGY_BUBBLES;
			fx->frameNumber = Objects[fx->objectNumber].meshIndex + 5;
			fx->speed = (GetRandomControl() & 0x1F) + 96;
			fx->flag1 = 6;
		}
	}

	void TriggerCrocgodMissileFlame(short fxNumber, short xVel, short yVel, short zVel)
	{
		//x = LaraItem->pos.Position.x - Effects[m_fxNumber].pos.Position.x;
		//z = LaraItem->pos.Position.z - Effects[m_fxNumber].pos.Position.z;
		//if (x >= -0x4000u && x <= 0x4000 && z >= -0x4000u && z <= 0x4000)

		auto* fx = &EffectList[fxNumber];
		auto* sptr = &Sparks[GetFreeSpark()];

		sptr->on = true;
		BYTE color = (GetRandomControl() & 0x3F) - 128;
		sptr->sB = 0;
		sptr->sR = color;
		sptr->sG = color / 2;
		color = (GetRandomControl() & 0x3F) - 128;
		sptr->dB = 0;
		sptr->dR = color;
		sptr->dG = color / 2;
		sptr->fadeToBlack = 8;
		sptr->colFadeSpeed = (GetRandomControl() & 3) + 8;
		sptr->transType = TransTypeEnum::COLADD;
		sptr->dynamic = -1;
		BYTE life = (GetRandomControl() & 7) + 32;
		sptr->life = life;
		sptr->sLife = life;
		sptr->x = (GetRandomControl() & 0xF) - 8;
		sptr->y = 0;
		sptr->z = (GetRandomControl() & 0xF) - 8;
		sptr->x += fx->pos.Position.x;
		sptr->y += fx->pos.Position.y;
		sptr->z += fx->pos.Position.z;
		sptr->xVel = xVel;
		sptr->yVel = yVel;
		sptr->zVel = zVel;
		sptr->friction = 34;
		sptr->flags = SP_EXPDEF | SP_ROTATE | SP_DEF | SP_SCALE;
		sptr->rotAng = GetRandomControl() & 0xFFF;

		if (GetRandomControl() & 1)
			sptr->rotAdd = (GetRandomControl() & 0x1F) - 32;
		else
			sptr->rotAdd = (GetRandomControl() & 0x1F) + 32;

		sptr->gravity = 0;
		sptr->maxYvel = 0;
		sptr->fxObj = byte(fxNumber);
		sptr->scalar = 2;
		BYTE size = (GetRandomControl() & 0xF) + 128;
		sptr->size = size;
		sptr->sSize = size;
		sptr->dSize = size / 4;
	}

	static void ShootFireball(PHD_3DPOS* src, MissileRotationType rotation, short roomNumber, int timer)
	{
		switch (rotation)
		{
		case MissileRotationType::Left:
			src->Orientation.y -= GetRandomControl() % 0x2000;
			break;

		case MissileRotationType::Right:
			src->Orientation.y += GetRandomControl() % 0x2000;
			break;
		}

		TriggerCrocgodMissile(src, roomNumber, timer);
	}

	static bool ShootFrame(ITEM_INFO* item)
	{
		int frameNumber = (item->Animation.FrameNumber - g_Level.Anims[item->Animation.AnimNumber].frameBase);
		if (frameNumber == 45 ||
			/*frameNumber == 50 ||
			frameNumber == 55 ||*/
			frameNumber == 60 ||
			/*frameNumber == 65 ||
			frameNumber == 70 ||*/
			frameNumber == 75)
		{
			return true;
		}
		else
			return false;
	}

	static void RotateHeadToTarget(ITEM_INFO* item, CreatureInfo* creature, int joint, short& headAngle)
	{
		if (creature->Enemy == nullptr)
		{
			headAngle = item->Pose.Orientation.y;
			return;
		}

		auto* enemy = creature->Enemy;
		Vector3Int pos = { 0, 0, 0 };
		GetJointAbsPosition(item, &pos, joint);

		int x = enemy->Pose.Position.x - pos.x;
		int z = enemy->Pose.Position.z - pos.z;
		headAngle = (short)(phd_atan(z, x) - item->Pose.Orientation.y) / 2;
	}

	static void GetTargetPosition(ITEM_INFO* item, PHD_3DPOS* target)
	{
		Vector3Int start = { 0, -96, 144 };
		GetJointAbsPosition(item, &start, 9);

		Vector3Int end = { 0, -128, 288 };
		GetJointAbsPosition(item, &end, 9);

		short angles[2];
		phd_GetVectorAngles(end.x - start.x, end.y - start.y, end.z - start.z, angles);

		target->Position.x = end.x;
		target->Position.y = end.y;
		target->Position.z = end.z;
		target->Orientation.x = angles[1];
		target->Orientation.y = angles[0];
		target->Orientation.z = 0;
	}

	static void MoveItemFront(ITEM_INFO* item, int distance)
	{
		short angle = short(TO_DEGREES(item->Pose.Orientation.y));
		switch (angle)
		{
		case C_NORTH:
			item->Pose.Position.z += distance;
			break;

		case C_EAST:
			item->Pose.Position.x += distance;
			break;

		case C_SOUTH:
			item->Pose.Position.z -= distance;
			break;

		case C_WEST:
			item->Pose.Position.x -= distance;
			break;
		}
	}

	static void MoveItemBack(ITEM_INFO* item, int distance)
	{
		short angle = short(TO_DEGREES(item->Pose.Orientation.y));
		switch (angle)
		{
		case C_NORTH:
			item->Pose.Position.z -= distance;
			break;

		case C_EAST:
			item->Pose.Position.x -= distance;
			break;

		case C_SOUTH:
			item->Pose.Position.z += distance;
			break;

		case C_WEST:
			item->Pose.Position.x += distance;
			break;
		}
	}

	static void MutantAIFix(ITEM_INFO* item, AI_INFO* info)
	{
		MoveItemFront(item, SECTOR(2));
		item->Pose.Position.y -= CLICK(3);
		CreatureAIInfo(item, info);
		item->Pose.Position.y += CLICK(3);
		MoveItemBack(item, SECTOR(2));
	}

	void InitialiseCrocgod(short itemNumber)
	{
		InitialiseCreature(itemNumber);

		auto* item = &g_Level.Items[itemNumber];
		item->Animation.AnimNumber = Objects[item->ObjectNumber].animIndex + MUTANT_ANIM_APPEAR;
		item->Animation.FrameNumber = g_Level.Anims[item->Animation.AnimNumber].frameBase;
		item->Animation.ActiveState = MUTANT_STATE_APPEAR;
		item->Animation.TargetState = MUTANT_STATE_APPEAR;
	}

	void CrocgodControl(short itemNumber)
	{
		if (!CreatureActive(itemNumber))
			return;

		auto* item = &g_Level.Items[itemNumber];
		auto* creature = GetCreatureInfo(item);

		OBJECT_BONES mutantJoint;
		int frameNumber;
		short angle = 0;
		short headY = 0;

		if (item->AIBits & ALL_AIOBJ)
			GetAITarget(creature);
		else if (creature->HurtByLara)
			creature->Enemy = LaraItem;
		else
			TargetNearestEntity(item, creature);

		AI_INFO AI;
		MutantAIFix(item, &AI);

		RotateHeadToTarget(item, creature, 9, headY);
		GetCreatureMood(item, &AI, VIOLENT);
		CreatureMood(item, &AI, VIOLENT);

		creature->MaxTurn = 0;
		angle = CreatureTurn(item, 0);

		switch (item->Animation.ActiveState)
		{
		case MUTANT_STATE_IDLE:
			if (AI.ahead)
			{
				int random = GetRandomControl() & 31;
				if ((random > 0 && random < 10) && AI.distance <= MUTANT_SHOOT_RANGE)
					item->Animation.TargetState = MUTANT_STATE_SHOOT;
				else if ((random > 10 && random < 20) && AI.distance <= MUTANT_LOCUST_1_RANGE)
					item->Animation.TargetState = MUTANT_STATE_LOCUST_1;
				else if ((random > 20 && random < 30) && AI.distance <= MUTANT_LOCUST_2_RANGE)
					item->Animation.TargetState = MUTANT_STATE_LOCUST_2;
			}

			break;

		case MUTANT_STATE_SHOOT:
			frameNumber = (item->Animation.FrameNumber - g_Level.Anims[item->Animation.AnimNumber].frameBase);
			if (frameNumber >= 94 && frameNumber <= 96)
			{
				PHD_3DPOS src;
				GetTargetPosition(item, &src);

				if (frameNumber == 94)
					ShootFireball(&src, MissileRotationType::Front, item->RoomNumber, 0);
				else if (frameNumber == 95)
				{
					ShootFireball(&src, MissileRotationType::Left, item->RoomNumber, 1);
					//ShootFireball(&src, MissileRotationType::M_LEFT, item->roomNumber, 1);
				}
				else if (frameNumber == 96)
				{
					ShootFireball(&src, MissileRotationType::Right, item->RoomNumber, 1);
					//ShootFireball(&src, MissileRotationType::M_RIGHT, item->roomNumber, 1);
				}
			}

			break;

		case MUTANT_STATE_LOCUST_1:
			frameNumber = (item->Animation.FrameNumber - g_Level.Anims[item->Animation.AnimNumber].frameBase);
			if (frameNumber >= 60 && frameNumber <= 120)
				TEN::Entities::TR4::SpawnLocust(item);

			break;

		case MUTANT_STATE_LOCUST_2:
			if (ShootFrame(item))
			{
				PHD_3DPOS src;
				GetTargetPosition(item, &src);
				ShootFireball(&src, MissileRotationType::Front, item->RoomNumber, 1);
			}

			break;
		}

		if (item->Animation.ActiveState != MUTANT_STATE_LOCUST_1)
			mutantJoint = OBJECT_BONES(headY, AI.xAngle, true);
		else
			mutantJoint = OBJECT_BONES(0);

		CreatureJoint(item, 0, mutantJoint.bone0);
		CreatureJoint(item, 1, mutantJoint.bone1);
		CreatureJoint(item, 2, mutantJoint.bone2);
		CreatureJoint(item, 3, mutantJoint.bone3);
		CreatureAnimation(itemNumber, angle, 0);
	}
}
