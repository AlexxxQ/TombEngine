#include "framework.h"
#include "tr3_objects.h"
/// entities
#include "tr3_civvy.h" // OK
#include "tr3_cobra.h" // OK
#include "tr3_fishemitter.h" // OK
#include "tr3_flamethrower.h" // OK
#include "tr3_monkey.h" // OK
#include "tr3_mpgun.h" // OK
#include "tr3_mpstick.h" // OK
#include "tr3_raptor.h" // OK
#include "tr3_scuba.h" // OK
#include "tr3_shiva.h" // OK
#include "tr3_sophia.h" // OK
#include "tr3_tiger.h" // OK
#include "tr3_tony.h" // OK
#include "tr3_trex.h" // OK
#include "tr3_tribesman.h" // OK
/// objects

/// traps

/// switch

/// vehicles
#include "cannon.h"
#include "kayak.h"
#include "minecart.h"
#include "quad.h"
#include "upv.h"
#include "rubberboat.h"
/// necessary import
#include "collide.h"
#include "setup.h"
#include "level.h"

static void StartBaddy(ObjectInfo* obj)
{
	obj = &Objects[ID_TONY_BOSS];
	if (obj->loaded)
	{
		obj->initialise = InitialiseTony;
		obj->collision = CreatureCollision;
		obj->control = TonyControl;
		obj->drawRoutine = S_DrawTonyBoss;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 100;
		obj->pivotLength = 50;
		obj->radius = 102;
		obj->intelligent = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->saveHitpoints = true;
		obj->savePosition = true;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_X;
		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_TIGER];
	if (obj->loaded)
	{
		obj->control = TigerControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = 128;
		obj->hitPoints = 24;
		obj->pivotLength = 200;
		obj->radius = 340;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		Bones[obj->boneIndex + 21 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_COBRA];
	if (obj->loaded)
	{
		obj->initialise = InitialiseCobra;
		obj->control = CobraControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = 128;
		obj->hitPoints = 8;
		obj->radius = 102;
		obj->intelligent = true;
		obj->nonLot = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;

		Bones[obj->boneIndex + 0 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_RAPTOR];
	if (obj->loaded)
	{
		obj->control = RaptorControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = 128;
		obj->hitPoints = 100;
		obj->radius = 341;
		obj->pivotLength = 600;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;

		Bones[obj->boneIndex + 20 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 21 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 23 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 25 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_TRIBESMAN_WITH_AX];
	if (obj->loaded)
	{
		obj->control = TribemanAxeControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = 128;
		obj->hitPoints = 28;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;

		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_TRIBESMAN_WITH_DARTS];
	if (obj->loaded)
	{
		obj->control = TribemanDartsControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = 128;
		obj->hitPoints = 28;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;

		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_TYRANNOSAUR];
	if (obj->loaded)
	{
		obj->control = TyrannosaurControl;
		obj->collision = CreatureCollision;
		obj->hitPoints = 800;
		obj->shadowSize = 64;
		obj->pivotLength = 1800;
		obj->radius = 512;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;

		Bones[obj->boneIndex + 10 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 11 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_SCUBA_DIVER];
	if (obj->loaded)
	{
		obj->control = ScubaControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 20;
		obj->radius = 340;
		obj->intelligent = true;
		obj->waterCreature = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 50;
		obj->zoneType = ZONE_WATER;

		Bones[obj->boneIndex + 10 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 14 * 4] |= ROT_Z;
	}

	obj = &Objects[ID_SCUBA_HARPOON];
	if (obj->loaded)
	{
		obj->control = ScubaHarpoonControl;
		obj->collision = ObjectCollision;
		obj->savePosition = true;
	}

	obj = &Objects[ID_FLAMETHROWER_BADDY];
	if (obj->loaded)
	{
		obj->control = FlameThrowerControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 36;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;

		Bones[obj->boneIndex + 0 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 0 * 4] |= ROT_X;
		Bones[obj->boneIndex + 7 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_MONKEY];
	if (obj->loaded)
	{
		//if (!Objects[MESHSWAP2].loaded)
		//	S_ExitSystem("FATAL: Monkey requires MESHSWAP2 (Monkey + Pickups)");
		//obj->draw_routine = DrawMonkey;
		obj->initialise = InitialiseMonkey;
		obj->control = MonkeyControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 8;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;

		Bones[obj->boneIndex + 0 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 0 * 4] |= ROT_X;
		Bones[obj->boneIndex + 7 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_MP_WITH_GUN];
	if (obj->loaded)
	{
		obj->control = MPGunControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 28;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;
		obj->biteOffset = 0;

		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_X;
		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_MP_WITH_STICK];
	if (obj->loaded)
	{
		obj->initialise = InitialiseMPStick;
		obj->control = MPStickControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 28;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;
		obj->zoneType = ZONE_HUMAN_CLASSIC;

		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_X;
		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_SHIVA];
	if (obj->loaded)
	{
		obj->initialise = InitialiseShiva;
		obj->collision = CreatureCollision;
		obj->control = ShivaControl;
		//obj->drawRoutine = DrawStatue;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 100;
		obj->pivotLength = 0;
		obj->radius = 256;
		obj->intelligent = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->saveHitpoints = true;
		obj->savePosition = true;
		Bones[obj->boneIndex + 6 * 4] |= (ROT_X | ROT_Y);
		Bones[obj->boneIndex + 25 * 4] |= (ROT_X | ROT_Y);
	}

	obj = &Objects[ID_SOPHIA_LEE_BOSS];
	if (obj->loaded)
	{
		obj->initialise = InitialiseLondonBoss;
		obj->collision = CreatureCollision;
		obj->control = LondonBossControl;
		obj->drawRoutine = S_DrawLondonBoss;
		obj->shadowSize = 0;
		obj->pivotLength = 50;
		obj->hitPoints = 300;
		obj->radius = 102;
		obj->intelligent = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_X;
		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
	}

	obj = &Objects[ID_CIVVIE];
	if (obj->loaded)
	{
		obj->initialise = InitialiseCivvy;
		obj->control = CivvyControl;
		obj->collision = CreatureCollision;
		obj->shadowSize = UNIT_SHADOW / 2;
		obj->hitPoints = 15;
		obj->radius = 102;
		obj->intelligent = true;
		obj->savePosition = true;
		obj->saveHitpoints = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->pivotLength = 0;
		Bones[obj->boneIndex + 6 * 4] |= ROT_Y;
		Bones[obj->boneIndex + 6 * 4] |= ROT_X;
		Bones[obj->boneIndex + 13 * 4] |= ROT_Y;
	}
}

static void StartObject(ObjectInfo* obj)
{
	
}

static void StartTrap(ObjectInfo* obj)
{
	
}

static void StartVehicles(ObjectInfo* obj)
{
	obj = &Objects[ID_QUAD];
	if (obj->loaded)
	{
		obj->initialise = InitialiseQuadBike;
		obj->collision = QuadBikeCollision;
		obj->savePosition = true;
		obj->saveAnim = true;
		obj->saveFlags = true;
	}

	obj = &Objects[ID_KAYAK];
	if (obj->loaded)
	{
		obj->initialise = InitialiseKayak;
		obj->collision = KayakCollision;
		//obj->drawRoutine = DrawKayak;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->savePosition = true;
	}

	obj = &Objects[ID_MINECART];
	if (obj->loaded)
	{
		obj->initialise = InitialiseMineCart;
		obj->collision = MineCartCollision;
		obj->saveAnim = true;
		obj->saveFlags = true;
		obj->savePosition = true;
	}
}

static void StartProjectiles(ObjectInfo* obj)
{
	obj = &Objects[ID_TONY_BOSS_FLAME];
	obj->control = ControlTonyFireBall;
	obj->drawRoutine = NULL;
}

static ObjectInfo* objToInit;
void InitialiseTR3Objects()
{
	StartBaddy(objToInit);
	StartObject(objToInit);
	StartTrap(objToInit);
	StartVehicles(objToInit);
	StartProjectiles(objToInit);
}