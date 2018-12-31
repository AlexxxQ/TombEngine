#include "inventory.h"
#include "draw.h"
#include "control.h"
#include "larafire.h"
#include "sound.h"
#include "gameflow.h"
#include "sound.h"
#include "savegame.h"
#include "lara.h"

#include "..\Global\global.h"
#include "..\Specific\input.h"
#include "..\Specific\config.h"

Inventory* g_Inventory;
extern GameFlow* g_GameFlow;
extern LaraExtraInfo g_LaraExtra;

void Inject_Inventory()
{

}

Inventory::Inventory()
{
	ZeroMemory(&m_objectsTable[0], sizeof(InventoryObjectDefinition) * INVENTORY_TABLE_SIZE);

	// Copy the old table
	for (__int32 i = 0; i < 100; i++)
	{
		m_objectsTable[i].objectNumber = InventoryObjectsList[i].objectNumber;
		m_objectsTable[i].objectName = InventoryObjectsList[i].objectName;
		m_objectsTable[i].meshBits = InventoryObjectsList[i].meshBits;
	}

	// Assign new strings
	m_objectsTable[INV_OBJECT_UZIS] = InventoryObjectDefinition(ID_UZI_ITEM, STRING_INV_UZI, -1, 0);
	m_objectsTable[INV_OBJECT_PISTOLS] = InventoryObjectDefinition(ID_PISTOLS_ITEM, STRING_INV_PISTOLS, -1, 0);
	m_objectsTable[INV_OBJECT_SHOTGUN] = InventoryObjectDefinition(ID_SHOTGUN_ITEM, STRING_INV_SHOTGUN, -1, 0);
	m_objectsTable[INV_OBJECT_REVOLVER] = InventoryObjectDefinition(ID_REVOLVER_ITEM, STRING_INV_REVOLVER, -1, 0);
	m_objectsTable[INV_OBJECT_REVOLVER_LASER] = InventoryObjectDefinition(ID_REVOLVER_ITEM, STRING_INV_REVOLVER_LASER, -1, 0);
	m_objectsTable[INV_OBJECT_HK] = InventoryObjectDefinition(ID_HK_ITEM, STRING_INV_HK, -1, 0);
	m_objectsTable[INV_OBJECT_SHOTGUN_AMMO1] = InventoryObjectDefinition(ID_SHOTGUN_AMMO1_ITEM, STRING_INV_SHOTGUN_AMMO1, -1, 0);
	m_objectsTable[INV_OBJECT_SHOTGUN_AMMO2] = InventoryObjectDefinition(ID_SHOTGUN_AMMO2_ITEM, STRING_INV_SHOTGUN_AMMO2, -1, 0);
	m_objectsTable[INV_OBJECT_HK_AMMO1] = InventoryObjectDefinition(ID_HK_AMMO_ITEM, STRING_INV_HK_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_REVOLVER_AMMO] = InventoryObjectDefinition(ID_REVOLVER_AMMO_ITEM, STRING_INV_REVOLVER_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_UZI_AMMO] = InventoryObjectDefinition(ID_UZI_AMMO_ITEM, STRING_INV_UZI_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_PISTOLS_AMMO] = InventoryObjectDefinition(ID_PISTOLS_AMMO_ITEM, STRING_INV_PISTOLS_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_LASERSIGHT] = InventoryObjectDefinition(ID_LASERSIGHT_ITEM, STRING_INV_LASERSIGHT, -1, 0);
	m_objectsTable[INV_OBJECT_SILENCER] = InventoryObjectDefinition(ID_SILENCER_ITEM, STRING_INV_SILENCER, -1, 0);
	m_objectsTable[INV_OBJECT_LARGE_MEDIPACK] = InventoryObjectDefinition(ID_BIGMEDI_ITEM, STRING_INV_LARGE_MEDIPACK, -1, 0);
	m_objectsTable[INV_OBJECT_SMALL_MEDIPACK] = InventoryObjectDefinition(ID_SMALLMEDI_ITEM, STRING_INV_SMALL_MEDIPACK, -1, 0);
	m_objectsTable[INV_OBJECT_BINOCULARS] = InventoryObjectDefinition(ID_BINOCULARS_ITEM, STRING_INV_BINOCULARS, -1, 0);
	m_objectsTable[INV_OBJECT_FLARES] = InventoryObjectDefinition(ID_FLARE_INV_ITEM, STRING_INV_FLARES, -1, 0);
	m_objectsTable[INV_OBJECT_TIMEX] = InventoryObjectDefinition(ID_COMPASS_ITEM, STRING_INV_TIMEX, -1, 0);
	m_objectsTable[INV_OBJECT_CROWBAR] = InventoryObjectDefinition(ID_CROWBAR_ITEM, STRING_INV_CROWBAR, -1, 0);
	m_objectsTable[INV_OBJECT_GRENADE_LAUNCHER] = InventoryObjectDefinition(ID_GRENADE_ITEM, STRING_INV_GRENADE_LAUNCHER, -1, 0);
	m_objectsTable[INV_OBJECT_GRENADE_AMMO1] = InventoryObjectDefinition(ID_GRENADE_AMMO1_ITEM, STRING_INV_GRENADE_AMMO1, -1, 0);
	m_objectsTable[INV_OBJECT_GRENADE_AMMO2] = InventoryObjectDefinition(ID_GRENADE_AMMO2_ITEM, STRING_INV_GRENADE_AMMO2, -1, 0);
	m_objectsTable[INV_OBJECT_GRENADE_AMMO3] = InventoryObjectDefinition(ID_GRENADE_AMMO3_ITEM, STRING_INV_GRENADE_AMMO3, -1, 0);
	m_objectsTable[INV_OBJECT_HARPOON_GUN] = InventoryObjectDefinition(ID_HARPOON_ITEM, STRING_INV_HARPOON_GUN, -1, 0);
	m_objectsTable[INV_OBJECT_HARPOON_AMMO] = InventoryObjectDefinition(ID_HARPOON_AMMO_ITEM, STRING_INV_HARPOON_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_ROCKET_LAUNCHER] = InventoryObjectDefinition(ID_ROCKET_LAUNCHER_ITEM, STRING_INV_ROCKET_LAUNCHER, -1, 0);
	m_objectsTable[INV_OBJECT_ROCKET_AMMO] = InventoryObjectDefinition(ID_ROCKET_LAUNCHER_AMMO_ITEM, STRING_INV_ROCKET_AMMO, -1, 0);
	m_objectsTable[INV_OBJECT_CROSSBOW] = InventoryObjectDefinition(ID_CROSSBOW_ITEM, STRING_INV_CROSSBOW, -1, 0);
	m_objectsTable[INV_OBJECT_CROSSBOW_LASER] = InventoryObjectDefinition(ID_CROSSBOW_ITEM, STRING_INV_CROSSBOW_LASER, -1, 0);
	m_objectsTable[INV_OBJECT_CROSSBOW_AMMO1] = InventoryObjectDefinition(ID_CROSSBOW_AMMO1_ITEM, STRING_INV_CROSSBOW_AMMO1, -1, 0);
	m_objectsTable[INV_OBJECT_CROSSBOW_AMMO2] = InventoryObjectDefinition(ID_CROSSBOW_AMMO2_ITEM, STRING_INV_CROSSBOW_AMMO2, -1, 0);
	m_objectsTable[INV_OBJECT_CROSSBOW_AMMO3] = InventoryObjectDefinition(ID_CROSSBOW_AMMO3_ITEM, STRING_INV_CROSSBOW_AMMO3, -1, 0);
	m_objectsTable[INV_OBJECT_PASSAPORT] = InventoryObjectDefinition(ID_INVENTORY_PASSPORT, STRING_INV_PASSPORT, -1, 0);
	m_objectsTable[INV_OBJECT_KEYS] = InventoryObjectDefinition(ID_INVENTORY_KEYS, STRING_INV_CONTROLS, -1, 0);
	m_objectsTable[INV_OBJECT_SUNGLASSES] = InventoryObjectDefinition(ID_INVENTORY_SUNGLASSES, STRING_INV_DISPLAY, -1, 0);
	m_objectsTable[INV_OBJECT_POLAROID] = InventoryObjectDefinition(ID_INVENTORY_POLAROID, STRING_INV_LARA_HOME, -1, 0);
	m_objectsTable[INV_OBJECT_HEADPHONES] = InventoryObjectDefinition(ID_INVENTORY_HEADPHONES, STRING_INV_SOUND, -1, 0);
	m_objectsTable[INV_OBJECT_DIARY] = InventoryObjectDefinition(ID_DIARY, STRING_INV_DIARY, -1, 0);
	m_objectsTable[INV_OBJECT_WATERSKIN1] = InventoryObjectDefinition(ID_WATERSKIN1_EMPTY, STRING_INV_WATERSKIN1_EMPTY, -1, 0);
	m_objectsTable[INV_OBJECT_WATERSKIN2] = InventoryObjectDefinition(ID_WATERSKIN2_EMPTY, STRING_INV_WATERSKIN2_EMPTY, -1, 0);
}

Inventory::~Inventory()
{

}

InventoryRing* Inventory::GetRing(__int32 index)
{
	return &m_rings[index];
}

__int32 Inventory::GetActiveRing()
{
	return m_activeRing;
}

void Inventory::SetActiveRing(__int32 index)
{
	m_activeRing = index;
}

void Inventory::InsertObject(__int32 ring, __int32 objectNumber)
{
	m_rings[ring].objects[m_rings[ring].numObjects].inventoryObject = objectNumber;
	m_rings[ring].numObjects++;
}

void Inventory::Initialise()
{
	// Reset the objects in inventory
	for (__int32 i = 0; i < NUM_INVENTORY_RINGS; i++)
	{
		m_rings[i].numObjects = 0;
		m_rings[i].movement = 0;
		m_rings[i].currentObject = 0;
		m_rings[i].focusState = INV_FOCUS_STATE_NONE;

		for (__int32 j = 0; j < NUM_INVENTORY_OBJECTS_PER_RING; j++)
		{
			m_rings[i].objects[j].inventoryObject = -1;
			m_rings[i].objects[j].rotation = 0;
			m_rings[i].objects[j].scale = INV_OBJECT_SCALE;
		}
	}

	// DEBUG
	{
		g_LaraExtra.Weapons[WEAPON_PISTOLS].Present = true;
		g_LaraExtra.Weapons[WEAPON_PISTOLS].Ammo[0] = -1;
		g_LaraExtra.Weapons[WEAPON_PISTOLS].SelectedAmmo = WEAPON_AMMO1;

		/*g_LaraExtra.Weapons[WEAPON_CROSSBOW].Present = true;
		g_LaraExtra.Weapons[WEAPON_CROSSBOW].Ammo[0] = 1000;
		g_LaraExtra.Weapons[WEAPON_CROSSBOW].SelectedAmmo = WEAPON_AMMO1;

		g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Present = true;
		g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Ammo[0] = 1000;
		g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].SelectedAmmo = WEAPON_AMMO1;

		g_LaraExtra.Weapons[WEAPON_HARPOON_GUN].Present = true;
		g_LaraExtra.Weapons[WEAPON_HARPOON_GUN].Ammo[0] = 1000;*/
	}
	
	// Now fill the rings
	if (g_GameFlow->GetLevel(CurrentLevel)->LaraType != LARA_DRAW_TYPE::LARA_YOUNG)
	{
		// Pistols
		if (g_LaraExtra.Weapons[WEAPON_PISTOLS].Present)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_PISTOLS);

		// Uzi
		if (g_LaraExtra.Weapons[WEAPON_UZI].Present)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_UZIS);
		else if (g_LaraExtra.Weapons[WEAPON_UZI].Ammo[0])
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_UZI_AMMO);
		
		// Revolver
		if (g_LaraExtra.Weapons[WEAPON_REVOLVER].Present)
		{
			if (g_LaraExtra.Weapons[WEAPON_REVOLVER].HasLasersight)
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_REVOLVER_LASER);
			else
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_REVOLVER);
		}
		else
		{
			if (g_LaraExtra.Weapons[WEAPON_REVOLVER].Ammo[0])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_REVOLVER_AMMO);
		}

		// Shotgun
		if (g_LaraExtra.Weapons[WEAPON_SHOTGUN].Present)
		{
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_SHOTGUN);
			//if (Lara.shotgunTypeCarried & 0x10)
			//	CurrentShotGunAmmoType = 1;
		}
		else
		{
			if (g_LaraExtra.Weapons[WEAPON_SHOTGUN].Ammo[0])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_SHOTGUN_AMMO1);
			if (g_LaraExtra.Weapons[WEAPON_SHOTGUN].Ammo[1])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_SHOTGUN_AMMO2);
		}

		// HK
		if (g_LaraExtra.Weapons[WEAPON_HK].Present)
		{
			if (g_LaraExtra.Weapons[WEAPON_HK].HasSilencer)
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_HK_LASER);
			else
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_HK);
		}
		else if (g_LaraExtra.Weapons[WEAPON_HK].Ammo[0])
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_HK_AMMO1);

		// Crossbow
		if (g_LaraExtra.Weapons[WEAPON_CROSSBOW].Present)
		{
			if (g_LaraExtra.Weapons[WEAPON_CROSSBOW].HasLasersight)
					InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROSSBOW_LASER);
				else
					InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROSSBOW);
		}
		else
		{
			if (g_LaraExtra.Weapons[WEAPON_CROSSBOW].Ammo[0])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROSSBOW_AMMO1);
			if (g_LaraExtra.Weapons[WEAPON_CROSSBOW].Ammo[1])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROSSBOW_AMMO2);
			if (g_LaraExtra.Weapons[WEAPON_CROSSBOW].Ammo[2])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROSSBOW_AMMO3);
		}

		// Grenade launcher
		if (g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Present)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_GRENADE_LAUNCHER);
		else
		{
			if (g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Ammo[0])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_GRENADE_AMMO1);
			if (g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Ammo[1])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_GRENADE_AMMO2);
			if (g_LaraExtra.Weapons[WEAPON_GRENADE_LAUNCHER].Ammo[2])
				InsertObject(INV_RING_WEAPONS, INV_OBJECT_GRENADE_AMMO3);
		}

		// Harpoon
		if (g_LaraExtra.Weapons[WEAPON_HARPOON_GUN].Present)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_HARPOON_GUN);
		else if (g_LaraExtra.Weapons[WEAPON_HARPOON_GUN].Ammo[0])
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_HARPOON_AMMO);

		// Rocket launcher
		if (g_LaraExtra.Weapons[WEAPON_ROCKET_LAUNCHER].Present)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_ROCKET_LAUNCHER);
		else if (g_LaraExtra.Weapons[WEAPON_ROCKET_LAUNCHER].Ammo[0])
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_ROCKET_AMMO);
				
		// Lasersight
		if (Lara.laserSight)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_LASERSIGHT);

		// Silencer
		if (Lara.silencer)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_SILENCER);

		// Binoculars
		if (Lara.binoculars)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_BINOCULARS);

		// Flares
		if (Lara.numFlares)
			InsertObject(INV_RING_WEAPONS, INV_OBJECT_FLARES);
	}

	if (Lara.numSmallMedipack)
		InsertObject(INV_RING_WEAPONS, INV_OBJECT_SMALL_MEDIPACK);

	if (Lara.numLargeMedipack)
		InsertObject(INV_RING_WEAPONS, INV_OBJECT_LARGE_MEDIPACK);

	if (Lara.crowbar)
		InsertObject(INV_RING_WEAPONS, INV_OBJECT_CROWBAR);

	__int32 i = 0;
	do
	{
		if (Lara.puzzleItems[i])
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_PUZZLE1);
		i++;
	} while (i < 8);

	i = 0;
	do
	{
		if ((1 << i) & Lara.puzzleItemsCombo)
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_PUZZLE1_COMBO1);
		i++;
	} while (i < 16);

	i = 0;
	do
	{
		if ((1 << i) & Lara.keyItems)
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_KEY1);
		i++;
	} while (i < 8);

	i = 0;
	do
	{
		if ((1 << i) & Lara.keyItemsCombo)
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_KEY1_COMBO1);
		i++;
	} while (i < 16);

	i = 0;
	do
	{
		if ((1 << i) & Lara.pickupItems)
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_PICKUP1);
		i++;
	} while (i < 4);

	i = 0;
	do
	{
		if ((1 << i) & Lara.pickupItemsCombo)
			InsertObject(INV_RING_PUZZLES, i + INV_OBJECT_PICKUP1_COMBO1);
		i++;
	} while (i < 8);

	if (Lara.examine1)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_EXAMINE1);

	if (Lara.examine2)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_EXAMINE2);

	if (Lara.examine3)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_EXAMINE3);

	if (Lara.wetcloth == 2)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_WETCLOTH1);

	if (Lara.wetcloth == 1)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_WETCLOTH2);

	if (Lara.bottle)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_BOTTLE);

	if (g_LaraExtra.Waterskin1.Present)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_WATERSKIN1);

	if (g_LaraExtra.Waterskin2.Present)
		InsertObject(INV_RING_PUZZLES, INV_OBJECT_WATERSKIN2);

	InventoryRing* ring = &m_rings[INV_RING_OPTIONS];

	// Reset the objects in inventory
	ring->numObjects = 0;
	ring->movement = 0;
	ring->currentObject = 0;
	ring->focusState = INV_FOCUS_STATE_NONE;

	for (__int32 j = 0; j < NUM_INVENTORY_OBJECTS_PER_RING; j++)
	{
		ring->objects[j].inventoryObject = -1;
		ring->objects[j].rotation = 0;
		ring->objects[j].scale = 2.0f;
	}
	  
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_PASSAPORT);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_SUNGLASSES);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_HEADPHONES);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_KEYS);

	m_activeRing = INV_RING_WEAPONS;
	m_type = INV_TYPE_GAME;
	m_deltaMovement = 0;
	m_movement = INV_MOVE_STOPPED;
	m_type = INV_TYPE_GAME;
	InventoryItemChosen = -1;
}

INVENTORY_RESULT Inventory::DoInventory()
{
	Initialise();

	// If Lara is dead, then we can use only the passport
	if (LaraItem->hitPoints <= 0 && CurrentLevel > 0)
	{
		m_rings[INV_RING_PUZZLES].draw = false;
		m_rings[INV_RING_WEAPONS].draw = false;
		m_rings[INV_RING_OPTIONS].draw = true;

		m_activeRing = INV_RING_OPTIONS;
		m_rings[m_activeRing].currentObject = 0;

		INVENTORY_RESULT passportResult = DoPassport();

		// Fade out
		g_Renderer->FadeOut();
		for (__int32 i = 0; i < FADE_FRAMES_COUNT; i++)
			g_Renderer->DrawInventory();

		return passportResult;
	}

	m_rings[INV_RING_PUZZLES].draw = true;
	m_rings[INV_RING_WEAPONS].draw = true;
	m_rings[INV_RING_OPTIONS].draw = true;

	m_activeRing = INV_RING_WEAPONS;

	INVENTORY_RESULT result = INVENTORY_RESULT::INVENTORY_RESULT_NONE;

	g_Renderer->DumpGameScene();
	g_Renderer->DrawInventory();

	while (!ResetFlag)
	{
		SetDebounce = true;
		S_UpdateInput();
		SetDebounce = false;

		GameTimer++;

		// Handle input
		if (DbInput & IN_DESELECT)
		{
			SoundEffect(SFX_MENU_SELECT, NULL, 0);

			// Exit from inventory
			GlobalEnterInventory = -1;
			return INVENTORY_RESULT::INVENTORY_RESULT_NONE;
		}
		else if (DbInput & IN_FORWARD &&
			(m_activeRing == INV_RING_WEAPONS && m_rings[INV_RING_PUZZLES].numObjects != 0 ||
				m_activeRing == INV_RING_OPTIONS))
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Go to the upper ring
			for (__int32 i = 0; i < 8; i++)
			{
				m_movement -= 1024.0f;
				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (m_activeRing == INV_RING_WEAPONS)
				m_activeRing = INV_RING_PUZZLES;
			else
				m_activeRing = INV_RING_WEAPONS;

			m_movement = 0;

			continue;
		}
		else if (DbInput & IN_BACK && (m_activeRing == INV_RING_PUZZLES || m_activeRing == INV_RING_WEAPONS))
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Go to the lower ring
			for (__int32 i = 0; i < 8; i++)
			{
				m_movement += 1024.0f;
				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (m_activeRing == INV_RING_WEAPONS)
				m_activeRing = INV_RING_OPTIONS;
			else
				m_activeRing = INV_RING_WEAPONS;

			m_movement = 0;

			continue;
		}
		else if (DbInput & IN_LEFT)
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Change object right
			float deltaAngle = 360.0f / m_rings[m_activeRing].numObjects / 8.0f;
			m_rings[m_activeRing].movement = 0;

			for (__int32 i = 0; i < 8; i++)
			{
				m_rings[m_activeRing].movement += deltaAngle;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (m_rings[m_activeRing].currentObject == m_rings[m_activeRing].numObjects - 1)
				m_rings[m_activeRing].currentObject = 0;
			else
				m_rings[m_activeRing].currentObject++;

			m_rings[m_activeRing].movement = 0;
		}
		else if (DbInput & IN_RIGHT)
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Change object left
			float deltaAngle = 360.0f / m_rings[m_activeRing].numObjects / 8.0f;
			m_rings[m_activeRing].movement = 0;

			for (__int32 i = 0; i < 8; i++)
			{
				m_rings[m_activeRing].movement -= deltaAngle;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (m_rings[m_activeRing].currentObject == 0)
				m_rings[m_activeRing].currentObject = m_rings[m_activeRing].numObjects - 1;
			else
				m_rings[m_activeRing].currentObject--;

			m_rings[m_activeRing].movement = 0;
		}
		else if (DbInput & IN_SELECT)
		{
			// Handle action 
			if (m_activeRing == INV_RING_OPTIONS)
			{
				if (m_rings[INV_RING_OPTIONS].objects[m_rings[INV_RING_OPTIONS].currentObject].inventoryObject == INV_OBJECT_PASSAPORT)
				{
					INVENTORY_RESULT passportResult = DoPassport();
					if (passportResult == INVENTORY_RESULT::INVENTORY_RESULT_NEW_GAME ||
						passportResult == INVENTORY_RESULT::INVENTORY_RESULT_EXIT_TO_TILE ||
						passportResult == INVENTORY_RESULT::INVENTORY_RESULT_LOAD_GAME)
					{
						// Fade out
						g_Renderer->FadeOut();
						for (__int32 i = 0; i < FADE_FRAMES_COUNT; i++)
							g_Renderer->DrawInventory();

						return passportResult;
					}
				}

				if (m_rings[INV_RING_OPTIONS].objects[m_rings[INV_RING_OPTIONS].currentObject].inventoryObject == INV_OBJECT_KEYS)
					DoControlsSettings();

				if (m_rings[INV_RING_OPTIONS].objects[m_rings[INV_RING_OPTIONS].currentObject].inventoryObject == INV_OBJECT_SUNGLASSES)
					DoGraphicsSettings();

				if (m_rings[INV_RING_OPTIONS].objects[m_rings[INV_RING_OPTIONS].currentObject].inventoryObject == INV_OBJECT_HEADPHONES)
					DoSoundSettings();
			}
			else if (m_activeRing == INV_RING_WEAPONS || m_activeRing == INV_RING_PUZZLES)
			{
				UseCurrentItem();

				// Exit from inventory
				GlobalEnterInventory = -1;
				return INVENTORY_RESULT::INVENTORY_RESULT_USE_ITEM;
			}
		}

		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	return result;
}

void Inventory::UseCurrentItem()
{
	InventoryRing* ring = &m_rings[m_activeRing];
	InventoryObject* inventoryObject = &ring->objects[ring->currentObject];
	__int16 objectNumber = m_objectsTable[inventoryObject->inventoryObject].objectNumber;

	LaraItem->meshBits = -1;

	__int32 binocularRange = BinocularRange;
	BinocularRange = 0;
	OldLaraBusy = false;

	// Small medipack
	if (objectNumber == ID_SMALLMEDI_ITEM)
	{
		if ((LaraItem->hitPoints <= 0 || LaraItem->hitPoints >= 1000) && !Lara.poisoned)
		{
			SayNo();
			return;
		}

		if (Lara.numSmallMedipack != -1)
			Lara.numSmallMedipack--;

		Lara.dpoisoned = 0;
		LaraItem->hitPoints += 500;
		if (LaraItem->hitPoints > 1000)
			LaraItem->hitPoints = 1000;
		
		SoundEffect(116, 0, 2);
		Savegame.Game.HealthUsed++;

		SoundEffect(SFX_MENU_MEDI, NULL, 0);

		return;
	}

	// Big medipack
	if (objectNumber == ID_BIGMEDI_ITEM)
	{
		if ((LaraItem->hitPoints <= 0 || LaraItem->hitPoints >= 1000) && !Lara.poisoned)
		{
			SayNo();
			return;
		}

		if (Lara.numLargeMedipack != -1)
			Lara.numLargeMedipack--;

		Lara.dpoisoned = 0;
		LaraItem->hitPoints += 1000;
		if (LaraItem->hitPoints > 1000)
			LaraItem->hitPoints = 1000;

		SoundEffect(116, 0, 2);
		Savegame.Game.HealthUsed++;

		SoundEffect(SFX_MENU_MEDI, NULL, 0);

		return;
	}

	// Binoculars
	if (objectNumber == ID_BINOCULARS_ITEM)
	{
		if (LaraItem->currentAnimState == 2 && LaraItem->animNumber == 103 || Lara.isDucked && !(TrInput & 0x20000000))
		{
			if (!SniperCameraActive && !UseSpotCam && !TrackCameraInit)
			{
				OldLaraBusy = true;
				BinocularRange = 128;
				if (Lara.gunStatus)
					Lara.gunStatus = LG_UNDRAW_GUNS;
			}
		}

		if (binocularRange)
			BinocularRange = binocularRange;
		else
			BinocularOldCamera = Camera.oldType;

		return;
	}

	// Crowbar and puzzles
	if (objectNumber == ID_CROWBAR_ITEM ||
		objectNumber >= ID_PUZZLE_ITEM1 && objectNumber <= ID_PUZZLE_ITEM8 ||
		objectNumber >= ID_PUZZLE_ITEM1_COMBO1 && objectNumber <= ID_PUZZLE_ITEM8_COMBO2 ||
		objectNumber >= ID_KEY_ITEM1 && objectNumber <= ID_KEY_ITEM8 ||
		objectNumber >= ID_KEY_ITEM1_COMBO1 && objectNumber <= ID_KEY_ITEM8_COMBO2 ||
		objectNumber >= ID_PICKUP_ITEM1 && objectNumber <= ID_PICKUP_ITEM3 ||
		objectNumber >= ID_PICKUP_ITEM1_COMBO1 && objectNumber <= ID_PICKUP_ITEM3_COMBO2)
	{
		// Only if above water
		if (Lara.waterStatus == LW_ABOVE_WATER)
		{
			InventoryItemChosen = objectNumber;
			
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			
			return;
		}
		else
		{
			SayNo();
			return;
		}
	}

	// Flares
	if (objectNumber == ID_FLARE_INV_ITEM)
	{
		if (Lara.waterStatus == LW_ABOVE_WATER)
		{
			InventoryItemChosen = objectNumber;
			
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			
			return;
		}
		else
		{
			SayNo();
			return;
		}
	}

	bool canUseWeapons = !(LaraItem->currentAnimState == 80 || LaraItem->currentAnimState == 81 || 
						   LaraItem->currentAnimState == 84 || LaraItem->currentAnimState == 85 || 
						   LaraItem->currentAnimState == 86 || LaraItem->currentAnimState == 88 || 
						   LaraItem->currentAnimState == 71 || LaraItem->currentAnimState == 105 || 
						   LaraItem->currentAnimState == 106 || Lara.waterStatus != LW_ABOVE_WATER);

	// Pistols
	if (objectNumber == ID_PISTOLS_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_PISTOLS; 
			if (!Lara.gunStatus && Lara.gunType == WEAPON_PISTOLS)
				Lara.gunStatus = LG_DRAW_GUNS;
		
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Uzis
	if (objectNumber == ID_UZI_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_UZI;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_UZI)
				Lara.gunStatus = LG_DRAW_GUNS;
		
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Revolver
	if (objectNumber == ID_REVOLVER_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_REVOLVER;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_REVOLVER)
				Lara.gunStatus = LG_DRAW_GUNS;
	
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Shotgun
	if (objectNumber == ID_SHOTGUN_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_SHOTGUN;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_SHOTGUN)
				Lara.gunStatus = LG_DRAW_GUNS;
		
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Grenade launcher
	if (objectNumber == ID_GRENADE_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_GRENADE_LAUNCHER;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_GRENADE_LAUNCHER)
				Lara.gunStatus = LG_DRAW_GUNS;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Harpoon gun
	if (objectNumber == ID_HARPOON_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_HARPOON_GUN;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_HARPOON_GUN)
				Lara.gunStatus = LG_DRAW_GUNS;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Crossbow/grappling gun
	if (objectNumber == ID_CROSSBOW_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_CROSSBOW;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_CROSSBOW)
				Lara.gunStatus = LG_DRAW_GUNS;
		
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// HK
	if (objectNumber == ID_HK_ITEM)
	{
		if (canUseWeapons)
		{
			Lara.requestGunType = WEAPON_HK;
			if (!Lara.gunStatus && Lara.gunType == WEAPON_HK)
				Lara.gunStatus = LG_DRAW_GUNS;
		
			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
		}
		else
		{
			SayNo();
		}

		return;
	}

	// Flares
	if (objectNumber == ID_FLARE_INV_ITEM)
	{
		if (!Lara.gunStatus)
		{
			if (LaraItem->currentAnimState != 80
				&& LaraItem->currentAnimState != 81
				&& LaraItem->currentAnimState != 84
				&& LaraItem->currentAnimState != 85
				&& LaraItem->currentAnimState != 86
				&& LaraItem->currentAnimState != 88
				&& Lara.waterStatus == LW_ABOVE_WATER)
			{
				if (Lara.gunType != WEAPON_FLARE)
				{
					TrInput = 0x80000;
					LaraGun();
					TrInput = 0;

					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				}
			}
		}
		else
		{
			SayNo();
		}

		return;
	}

	SayNo();
	return;
}

float Inventory::GetVerticalOffset()
{
	return m_movement;
}

void Inventory::InitialiseTitle()
{
	InventoryRing* ring = &m_rings[INV_RING_OPTIONS];

	// Reset the objects in inventory
	ring->numObjects = 0;
	ring->movement = 0;
	ring->currentObject = 0;
	ring->focusState = INV_FOCUS_STATE_NONE;

	for (__int32 j = 0; j < NUM_INVENTORY_OBJECTS_PER_RING; j++)
	{
		ring->objects[j].inventoryObject = -1;
		ring->objects[j].rotation = 0;
		ring->objects[j].scale = INV_OBJECT_SCALE;
	}

	InsertObject(INV_RING_OPTIONS, INV_OBJECT_PASSAPORT);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_POLAROID);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_SUNGLASSES);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_HEADPHONES);
	InsertObject(INV_RING_OPTIONS, INV_OBJECT_KEYS);

	m_activeRing = INV_RING_OPTIONS;
	m_deltaMovement = 0;
	m_movement = INV_MOVE_STOPPED;
	m_type = INV_TYPE_TITLE;
	InventoryItemChosen = -1;
}

INVENTORY_RESULT Inventory::DoTitleInventory()
{
	InitialiseTitle();

	m_rings[INV_RING_PUZZLES].draw = false;
	m_rings[INV_RING_WEAPONS].draw = false;
	m_rings[INV_RING_OPTIONS].draw = true;

	InventoryRing* ring = &m_rings[INV_RING_OPTIONS];
	m_activeRing = INV_RING_OPTIONS;

	// Fade in
	g_Renderer->FadeIn();
	for (__int32 i = 0; i < FADE_FRAMES_COUNT; i++)
		g_Renderer->DrawInventory();

	INVENTORY_RESULT result = INVENTORY_RESULT::INVENTORY_RESULT_NONE;

	while (!ResetFlag)
	{
		SetDebounce = true;
		S_UpdateInput();
		SetDebounce = false;

		GameTimer++;

		// Handle input
		if (DbInput & IN_LEFT)
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Change object right
			float deltaAngle = 360.0f / ring->numObjects / 8.0f;
			ring->movement = 0;

			for (__int32 i = 0; i < 8; i++)
			{
				ring->movement += deltaAngle;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (ring->currentObject == ring->numObjects - 1)
				ring->currentObject = 0;
			else
				ring->currentObject++;

			ring->movement = 0;
		}
		else if (DbInput & IN_RIGHT)
		{
			SoundEffect(SFX_MENU_ROTATE, NULL, 0);

			// Change object left
			float deltaAngle = 360.0f / ring->numObjects / 8.0f;
			ring->movement = 0;

			for (__int32 i = 0; i < 8; i++)
			{
				ring->movement -= deltaAngle;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}

			if (ring->currentObject == 0)
				ring->currentObject = ring->numObjects - 1;
			else
				ring->currentObject--;

			ring->movement = 0;
		}
		else if (DbInput & IN_SELECT)
		{
			SoundEffect(SFX_MENU_SELECT, NULL, 0);

			if (ring->objects[ring->currentObject].inventoryObject == INV_OBJECT_PASSAPORT)
			{
				INVENTORY_RESULT passportResult = DoPassport();
				if (passportResult == INVENTORY_RESULT::INVENTORY_RESULT_NEW_GAME ||
					passportResult == INVENTORY_RESULT::INVENTORY_RESULT_EXIT_GAME ||
					passportResult == INVENTORY_RESULT::INVENTORY_RESULT_LOAD_GAME)
				{
					// Fade out
					g_Renderer->FadeOut();
					for (__int32 i = 0; i < FADE_FRAMES_COUNT; i++)
						g_Renderer->DrawInventory();

					return passportResult;
				}
			}

			if (ring->objects[ring->currentObject].inventoryObject == INV_OBJECT_KEYS)
				DoControlsSettings();

			if (ring->objects[ring->currentObject].inventoryObject == INV_OBJECT_SUNGLASSES)
				DoGraphicsSettings();

			if (ring->objects[ring->currentObject].inventoryObject == INV_OBJECT_HEADPHONES)
				DoSoundSettings();
		}

		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	// Fade out
	g_Renderer->FadeOut();
	for (__int32 i = 0; i < FADE_FRAMES_COUNT; i++)
		g_Renderer->DrawInventory();

	return result;
}

InventoryObjectDefinition* Inventory::GetInventoryObject(__int32 index)
{
	return &m_objectsTable[index];
}

INVENTORY_RESULT Inventory::DoPassport()
{
	InventoryRing* ring = &m_rings[m_activeRing];
	ring->frameIndex = 0;

	__int16 choice = 0;
	
	vector<__int32> choices;
	if (m_type == INV_TYPE_TITLE)
	{
		choices.push_back(INV_WHAT_PASSPORT_NEW_GAME);
		choices.push_back(INV_WHAT_PASSPORT_SELECT_LEVEL);
		choices.push_back(INV_WHAT_PASSPORT_LOAD_GAME);
		choices.push_back(INV_WHAT_PASSPORT_EXIT_GAME);
	}
	else
	{
		choices.push_back(INV_WHAT_PASSPORT_NEW_GAME);
		choices.push_back(INV_WHAT_PASSPORT_LOAD_GAME);
		if (LaraItem->hitPoints > 0 || CurrentLevel == 0)
			choices.push_back(INV_WHAT_PASSPORT_SAVE_GAME);
		choices.push_back(INV_WHAT_PASSPORT_EXIT_TO_TITLE);
	}

	ring->passportAction = choices[0];

	PopupObject();

	// Open the passport
	for (__int32 i = 0; i < 14; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
		ring->frameIndex++;
	}

	bool moveLeft = false;
	bool moveRight = false;
	bool closePassport = false;

	INVENTORY_RESULT result = INVENTORY_RESULT::INVENTORY_RESULT_NONE;

	// Do the passport
	while (true)
	{
		// Handle input
		SetDebounce = true;
		S_UpdateInput();
		SetDebounce = false;

		GameTimer++;

		// Handle input
		if (DbInput & IN_DESELECT || closePassport)
		{
			moveLeft = false;
			moveRight = false;
			closePassport = false;

			break;
		}
		else if (DbInput & IN_LEFT || moveLeft)
		{
			moveLeft = false;
			moveRight = false;
			closePassport = false;

			if (choice > 0)
			{
				ring->frameIndex = 19;
				for (__int32 i = 0; i < 5; i++)
				{
					g_Renderer->DrawInventory();
					g_Renderer->SyncRenderer();
					ring->frameIndex--;
				}

				choice--;
			}
		}
		else if (DbInput & IN_RIGHT || moveRight)
		{
			moveLeft = false;
			moveRight = false;
			closePassport = false;
			
			if (choice < choices.size() - 1)
			{
				ring->frameIndex = 14;
				for (__int32 i = 0; i < 5; i++)
				{
					g_Renderer->DrawInventory();
					g_Renderer->SyncRenderer();
					ring->frameIndex++;
				}

				choice++;
			}
		}

		if (choices[choice] == INV_WHAT_PASSPORT_LOAD_GAME)
		{
			// Load game
			__int32 selectedSavegame = 0;
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_FORWARD && selectedSavegame > 0)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedSavegame--;
					continue;
				}
				else if (DbInput & IN_BACK && selectedSavegame < MAX_SAVEGAMES - 1)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedSavegame++;
					continue;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					//ReadSavegame(selectedSavegame);
					g_GameFlow->SelectedSaveGame = selectedSavegame;
					result = INVENTORY_RESULT::INVENTORY_RESULT_LOAD_GAME;
					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->selectedIndex = selectedSavegame;
				ring->passportAction = INV_WHAT_PASSPORT_LOAD_GAME;

				LoadSavegameInfos();

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else if (choices[choice] == INV_WHAT_PASSPORT_SAVE_GAME)
		{
			// Save game
			__int32 selectedSavegame = 0;
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_FORWARD && selectedSavegame > 0)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedSavegame--;
					continue;
				}
				else if (DbInput & IN_BACK && selectedSavegame < MAX_SAVEGAMES - 1)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedSavegame++;
					continue;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					// Use the new savegame system
					char fileName[255];
					ZeroMemory(fileName, 255);
					sprintf(fileName, "savegame.%d", selectedSavegame);
					SaveGame::Save(fileName);

					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->selectedIndex = selectedSavegame;
				ring->passportAction = INV_WHAT_PASSPORT_SAVE_GAME;

				LoadSavegameInfos();

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else if (choices[choice] == INV_WHAT_PASSPORT_SELECT_LEVEL)
		{
			// Save game
			__int32 selectedLevel = 0;
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_FORWARD && selectedLevel > 0)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedLevel--;
					continue;
				}
				else if (DbInput & IN_BACK && selectedLevel < g_GameFlow->GetNumLevels() - 1)
				{
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
					selectedLevel++;
					continue;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					result = INVENTORY_RESULT::INVENTORY_RESULT_NEW_GAME;
					g_GameFlow->SelectedLevelForNewGame = selectedLevel + 1;

					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->selectedIndex = selectedLevel;
				ring->passportAction = INV_WHAT_PASSPORT_SELECT_LEVEL;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else if (choices[choice] == INV_WHAT_PASSPORT_NEW_GAME)
		{
			// New game
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					result = INVENTORY_RESULT::INVENTORY_RESULT_NEW_GAME;
					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->passportAction = INV_WHAT_PASSPORT_NEW_GAME;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else if (choices[choice] == INV_WHAT_PASSPORT_EXIT_GAME)
		{
			// Exit game
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					result = INVENTORY_RESULT::INVENTORY_RESULT_EXIT_GAME;
					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->passportAction = INV_WHAT_PASSPORT_EXIT_GAME;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else if (choices[choice] == INV_WHAT_PASSPORT_EXIT_TO_TITLE)
		{
			// Exit game
			while (true)
			{
				SetDebounce = 1;
				S_UpdateInput();
				SetDebounce = 0;

				// Process input
				if (DbInput & IN_DESELECT)
				{
					if (CurrentLevel == 0 || LaraItem->hitPoints > 0)
					{
						moveLeft = false;
						moveRight = false;
						closePassport = true;
					}

					break;
				}
				else if (DbInput & IN_LEFT)
				{
					moveLeft = true;
					moveRight = false;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_RIGHT)
				{
					moveLeft = false;
					moveRight = true;
					closePassport = false;

					break;
				}
				else if (DbInput & IN_SELECT)
				{
					SoundEffect(SFX_MENU_SELECT, NULL, 0);

					result = INVENTORY_RESULT::INVENTORY_RESULT_EXIT_TO_TILE;
					moveLeft = false;
					moveRight = false;
					closePassport = true;

					break;
				}

				ring->passportAction = INV_WHAT_PASSPORT_EXIT_TO_TITLE;

				g_Renderer->DrawInventory();
				g_Renderer->SyncRenderer();
			}
		}
		else
		{
			g_Renderer->DrawInventory();
			g_Renderer->SyncRenderer();
		}
	}

	// Close the passport
	ring->frameIndex = 24;
	for (__int32 i = 24; i < 30; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
		ring->frameIndex++;
	}

	ring->frameIndex = 0;

	PopoverObject();

	return result;
}

__int32	Inventory::PopupObject()
{
	InventoryRing* ring = &m_rings[m_activeRing];

	__int32 steps = 8;
	__int32 deltaAngle = (0 - ring->objects[ring->currentObject].rotation) / steps;
	float deltaScale = INV_OBJECT_SCALE / (float)steps;

	ring->focusState = INV_FOCUS_STATE_POPUP;
	for (__int32 i = 0; i < steps; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();

		ring->objects[ring->currentObject].rotation += deltaAngle;
		ring->objects[ring->currentObject].scale += deltaScale;
	}
	ring->focusState = INV_FOCUS_STATE_FOCUSED;

	return 0;
}

__int32	Inventory::PopoverObject()
{
	InventoryRing* ring = &m_rings[m_activeRing];

	__int32 steps = 8;
	__int32 deltaAngle = (0 - ring->objects[ring->currentObject].rotation) / steps;
	float deltaScale = INV_OBJECT_SCALE / (float)steps;

	ring->focusState = INV_FOCUS_STATE_POPOVER;
	for (__int32 i = 0; i < steps; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();

		ring->objects[ring->currentObject].rotation -= deltaAngle;
		ring->objects[ring->currentObject].scale -= deltaScale;
	}
	ring->focusState = INV_FOCUS_STATE_NONE;

	return 0;
}

__int32 Inventory::GetType()
{
	return m_type;
}

void Inventory::DoControlsSettings()
{

}

void Inventory::DoGraphicsSettings()
{
	InventoryRing* ring = &m_rings[m_activeRing];
	ring->frameIndex = 0;
	ring->selectedIndex = 0;

	PopupObject();

	// Copy configuration to a temporary object
	memcpy(&ring->Configuration, &g_Configuration, sizeof(GameConfiguration));

	// Get current display mode
	vector<RendererVideoAdapter>* adapters = g_Renderer->GetAdapters();
	RendererVideoAdapter* adapter = &(*adapters)[ring->Configuration.Adapter];
	ring->SelectedVideoMode = 0;
	for (__int32 i = 0; i < adapter->DisplayModes.size(); i++)
	{
		RendererDisplayMode* mode = &adapter->DisplayModes[i];
		if (mode->Width == ring->Configuration.Width && mode->Height == ring->Configuration.Height && 
			mode->RefreshRate == ring->Configuration.RefreshRate)
		{
			ring->SelectedVideoMode = i;
			break;
		}
	}

	// Open the passport
	for (__int32 i = 0; i < 14; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	bool closeObject = false;

	// Do the passport
	while (true)
	{
		// Handle input
		SetDebounce = true;
		S_UpdateInput();
		SetDebounce = false;

		GameTimer++;

		// Handle input
		if (DbInput & IN_DESELECT || closeObject)
		{
			closeObject = true;
			break;
		}
		else if (DbInput & IN_LEFT)
		{
			closeObject = false;

			switch (ring->selectedIndex)
			{
			case INV_DISPLAY_RESOLUTION:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				if (ring->SelectedVideoMode > 0)
					ring->SelectedVideoMode--;

				break;

			case INV_DISPLAY_SHADOWS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableShadows = !ring->Configuration.EnableShadows;
				break;

			case INV_DISPLAY_CAUSTICS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableCaustics = !ring->Configuration.EnableCaustics;
				break;

			case INV_DISPLAY_VOLUMETRIC_FOG:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableVolumetricFog = !ring->Configuration.EnableVolumetricFog;
				break;
			}
		}
		else if (DbInput & IN_RIGHT)
		{
			closeObject = false;

			switch (ring->selectedIndex)
			{
			case INV_DISPLAY_RESOLUTION:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				if (ring->SelectedVideoMode < adapter->DisplayModes.size() - 1)
					ring->SelectedVideoMode++;
				break;

			case INV_DISPLAY_SHADOWS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableShadows = !ring->Configuration.EnableShadows;
				break;

			case INV_DISPLAY_CAUSTICS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableCaustics = !ring->Configuration.EnableCaustics;
				break;

			case INV_DISPLAY_VOLUMETRIC_FOG:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableVolumetricFog = !ring->Configuration.EnableVolumetricFog;
				break;
			}
		}
		else if (DbInput & IN_FORWARD)
		{
			closeObject = false;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			if (ring->selectedIndex > 0)
				ring->selectedIndex--;
		}
		else if (DbInput & IN_BACK)
		{
			closeObject = false;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			if (ring->selectedIndex < INV_DISPLAY_COUNT)
				ring->selectedIndex++;
		}
		else if (DbInput & IN_SELECT)
		{
			SoundEffect(SFX_MENU_SELECT, NULL, 0);

			if (ring->selectedIndex == INV_DISPLAY_APPLY)
			{
				// Save the configuration
				RendererDisplayMode* mode = &adapter->DisplayModes[ring->SelectedVideoMode];
				ring->Configuration.Width = mode->Width;
				ring->Configuration.Height = mode->Height;
				ring->Configuration.RefreshRate = mode->RefreshRate;
				memcpy(&g_Configuration, &ring->Configuration, sizeof(GameConfiguration));
				SaveConfiguration();

				// Reset screen and go back
				g_Renderer->ChangeScreenResolution(ring->Configuration.Width, ring->Configuration.Height, 
												   ring->Configuration.RefreshRate, ring->Configuration.Windowed);
				closeObject = true;

				break;
			}
			else if (ring->selectedIndex == INV_DISPLAY_CANCEL)
			{
				SoundEffect(SFX_MENU_SELECT, NULL, 0);

				closeObject = true;
				break;
			}
			else
			{

			}
		}

		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	PopoverObject();
}

void Inventory::DoSoundSettings()
{
	InventoryRing* ring = &m_rings[m_activeRing];
	ring->frameIndex = 0;
	ring->selectedIndex = 0;

	PopupObject();

	// Copy configuration to a temporary object
	memcpy(&ring->Configuration, &g_Configuration, sizeof(GameConfiguration));

	// Open the passport
	for (__int32 i = 0; i < 14; i++)
	{
		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	bool closeObject = false;
	__int32 oldVolume = ring->Configuration.MusicVolume;
	__int32 oldSfxVolume = ring->Configuration.SfxVolume;
	bool wasSoundEnabled = ring->Configuration.EnableSound;

	// Do the passport
	while (true)
	{
		// Handle input
		SetDebounce = true;
		S_UpdateInput();
		SetDebounce = false;

		GameTimer++;

		// Handle input
		if (DbInput & IN_DESELECT || closeObject)
		{
			closeObject = true;
			GlobalMusicVolume = oldVolume;
			GlobalFXVolume = oldSfxVolume;

			break;
		}
		else if (DbInput & IN_LEFT)
		{
			closeObject = false;

			switch (ring->selectedIndex)
			{
			case INV_SOUND_ENABLED:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableSound = !ring->Configuration.EnableSound;

				break;

			case INV_SOUND_SPECIAL_EFFECTS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableAudioSpecialEffects = !ring->Configuration.EnableAudioSpecialEffects;
				break;

			case INV_SOUND_MUSIC_VOLUME:
				if (ring->Configuration.MusicVolume > 0)
				{
					ring->Configuration.MusicVolume--;
					GlobalMusicVolume = ring->Configuration.MusicVolume;
				}

				break;

			case INV_SOUND_SFX_VOLUME:
				if (ring->Configuration.SfxVolume > 0)
				{
					ring->Configuration.SfxVolume--;
					GlobalFXVolume = ring->Configuration.SfxVolume;
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				}

				break;
			}
		}
		else if (DbInput & IN_RIGHT)
		{
			closeObject = false;

			switch (ring->selectedIndex)
			{
			case INV_SOUND_ENABLED:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableSound = !ring->Configuration.EnableSound;

				break;

			case INV_SOUND_SPECIAL_EFFECTS:
				SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				ring->Configuration.EnableAudioSpecialEffects = !ring->Configuration.EnableAudioSpecialEffects;
				break;

			case INV_SOUND_MUSIC_VOLUME:
				if (ring->Configuration.MusicVolume < 100)
				{
					ring->Configuration.MusicVolume++;
					GlobalMusicVolume = ring->Configuration.MusicVolume;
				}

				break;

			case INV_SOUND_SFX_VOLUME:
				if (ring->Configuration.SfxVolume < 100)
				{
					ring->Configuration.SfxVolume++;
					GlobalFXVolume = ring->Configuration.SfxVolume;
					SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
				}

				break;
			}
		}
		else if (DbInput & IN_FORWARD)
		{
			closeObject = false;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			if (ring->selectedIndex > 0)
				ring->selectedIndex--;
		}
		else if (DbInput & IN_BACK)
		{
			closeObject = false;

			SoundEffect(SFX_MENU_CHOOSE, NULL, 0);
			if (ring->selectedIndex < INV_DISPLAY_COUNT)
				ring->selectedIndex++;
		}
		else if (DbInput & IN_SELECT)
		{
			SoundEffect(SFX_MENU_SELECT, NULL, 0);

			if (ring->selectedIndex == INV_DISPLAY_APPLY)
			{
				// Save the configuration
				GlobalMusicVolume = ring->Configuration.MusicVolume;
				GlobalFXVolume = ring->Configuration.SfxVolume;
				memcpy(&g_Configuration, &ring->Configuration, sizeof(GameConfiguration));
				SaveConfiguration();

				// Init or deinit the sound system
				if (wasSoundEnabled && !g_Configuration.EnableSound)
					Sound_DeInit();
				else if (!wasSoundEnabled && g_Configuration.EnableSound)
					Sound_Init();

				closeObject = true;

				break;
			}
			else if (ring->selectedIndex == INV_DISPLAY_CANCEL)
			{
				SoundEffect(SFX_MENU_SELECT, NULL, 0);

				closeObject = true;
				GlobalMusicVolume = oldVolume;
				GlobalFXVolume = oldSfxVolume;

				break;
			}
			else
			{

			}
		}

		g_Renderer->DrawInventory();
		g_Renderer->SyncRenderer();
	}

	PopoverObject();
}