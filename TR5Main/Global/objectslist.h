#pragma once

typedef enum object_types {
	ID_LARA,
	ID_PISTOLS_ANIM,
	ID_UZI_ANIM,
	ID_SHOTGUN_ANIM,
	ID_CROSSBOW_ANIM,
	ID_HK_ANIM,
	ID_REVOLVER_ANIM,
	ID_FLARE_ANIM,
	ID_LARA_SKIN,
	ID_LARA_SKIN_JOINTS,
	ID_LARA_SCREAM,
	ID_LARA_CROSSBOW_LASER,
	ID_LARA_REVOLVER_LASER,
	ID_LARA_HOLSTERS,
	ID_LARA_HOLSTERS_PISTOLS,
	ID_LARA_HOLSTERS_UZIS,
	ID_LARA_HOLSTERS_REVOLVER,
	ID_LARA_SPEECH_HEAD1,
	ID_LARA_SPEECH_HEAD2,
	ID_LARA_SPEECH_HEAD3,
	ID_LARA_SPEECH_HEAD4,
	ID_ACTOR1_SPEECH_HEAD1,
	ID_ACTOR1_SPEECH_HEAD2,
	ID_ACTOR2_SPEECH_HEAD1,
	ID_ACTOR2_SPEECH_HEAD2,
	ID_LARA_EXTRA_MESH1,
	ID_LARA_EXTRA_MESH2,
	ID_LARA_EXTRA_MESH3,
	ID_CROWBAR_ANIM,
	ID_TORCH_ANIM,
	ID_HAIR,
	ID_SAS,
	ID_JEEP,   // TR4
	ID_SWAT,
	ID_MOTORBIKE,   // TR4
	ID_SWAT_PLUS,
	ID_LARA_JEEP_ANIMS,   // TR4
	ID_BLUE_GUARD,
	ID_LARA_MOTORBIKE_ANIMS,   // TR4
	ID_TWOGUN,
	ID_SKELETON, // TR4
	ID_DOG,
	ID_CROCODILE, // TR4
	ID_CROW,
	ID_MUMMY, // TR4
	ID_LARSON,
	ID_SMALL_SCORPION, // TR4 - Ok
	ID_PIERRE,
	ID_BAT, // TR4
	ID_MAFIA,
	ID_GUIDE, // TR4
	ID_MAFIA2,
	ID_SPHINX, // TR4
	ID_SAILOR,
	ID_HORSEMAN, // TR4
	ID_CRANE_GUY,
	ID_HORSE, // TR4
	ID_LION,
	ID_WILD_BOAR, // TR4
	ID_GLADIATOR,
	ID_HARPY, // TR4
	ID_ROMAN_GOD,
	ID_DEMIGOD1, // TR4
	ID_HYDRA,
	ID_DEMIGOD2, // TR4
	ID_GUARDIAN,
	ID_DEMIGOD3, // TR4
	ID_HITMAN,
	ID_LITTLE_BEETLE, // TR4
	ID_SCIENTIST,
	ID_MERCENARY, // TR3
	ID_WILLOWISP,
	ID_SHARK, // TR2
	ID_INVISIBLE_GHOST,
	ID_BADDY1, // TR4
	ID_REAPER,
	ID_BADDY2, //TR4
	ID_MAZE_MONSTER,
	ID_CHAIN, // TR4 - Control
	ID_GREEN_TEETH,
	ID_PLOUGH, // TR4 - Control
	ID_ATTACK_SUB,
	ID_STARGATE, // TR4
	ID_SNIPER,
	ID_BURNING_FLOOR, // TR4
	ID_HUSKIE,
	ID_COG, // TR4
	ID_CHEF,
	ID_SPIKEBALL, // TR4
	ID_IMP,
	ID_FLOOR_4BLADES, // TR4 - Control
	ID_GUNSHIP,
	ID_CEILING_4BLADES, // TR4 - Control
	ID_BATS,
	ID_RATS,
	ID_SPIDER,
	ID_SPIDER_GENERATOR,
	ID_AUTOGUN,
	ID_ELECTRICAL_CABLES,
	ID_DARTS,
	ID_DART_EMITTER,
	ID_HOMING_DART_EMITTER,
	ID_FALLING_CEILING,
	ID_FALLING_BLOCK,
	ID_FALLING_BLOCK2,
	ID_CRUMBLING_FLOOR,
	ID_TRAPDOOR1,
	ID_TRAPDOOR2,
	ID_TRAPDOOR3,
	ID_FLOOR_TRAPDOOR1,
	ID_FLOOR_TRAPDOOR2,
	ID_CEILING_TRAPDOOR1,
	ID_CEILING_TRAPDOOR2,
	ID_SCALING_TRAPDOOR,
	ID_ROLLINGBALL,
	ID_ROLLINGBARREL,
	ID_SPIKEY_FLOOR,
	ID_TEETH_SPIKES,
	ID_ROME_HAMMER,
	ID_HAMMER2,
	ID_FLAME,
	ID_FLAME_EMITTER,
	ID_FLAME_EMITTER2,
	ID_FLAME_EMITTER3,
	ID_COOKER_FLAME,
	ID_BURNING_ROOTS,
	ID_ROPE,
	ID_FIREROPE,
	ID_POLEROPE,
	ID_PROPELLER_H,
	ID_PROPELLER_V,
	ID_GRAPPLING_TARGET,
	ID_ONEBLOCK_PLATFORM,
	ID_TWOBLOCK_PLATFORM,
	ID_RAISING_BLOCK1,
	ID_RAISING_BLOCK2,
	ID_EXPANDING_PLATFORM,
	ID_PUSHABLE_OBJECT1,
	ID_PUSHABLE_OBJECT2,
	ID_PUSHABLE_OBJECT3,
	ID_PUSHABLE_OBJECT4,
	ID_PUSHABLE_OBJECT5,
	ID_WRECKING_BALL,
	ID_DEATH_SLIDE,
	ID_TORPEDO,
	ID_CHAFF,
	ID_SATCHEL_BOMB,
	ID_ELECTRIC_FENCE,
	ID_LIFT,
	ID_EXPLOSION,
	ID_IRIS_LIGHTNING,
	ID_MONITOR_SCREEN,
	ID_SECURITY_SCREENS,
	ID_MOTION_SENSORS,
	ID_TIGHT_ROPE,
	ID_PARALLEL_BARS,
	ID_XRAY_CONTROLLER,
	ID_CUTSCENE_ROPE,
	ID_PORTAL,
	ID_GEN_SLOT1,
	ID_GEN_SLOT2,
	ID_GEN_SLOT3,
	ID_GEN_SLOT4,
	ID_IMP_ROCK,
	ID_SEARCH_OBJECT1,
	ID_CATWALK_BLADE, // TR4 - Control
	ID_SEARCH_OBJECT2,
	ID_PLINTH_BLADE, // TR4 - Control
	ID_SEARCH_OBJECT3,
	ID_SETH_BLADE, // TR4 - Control - Initialise
	ID_SEARCH_OBJECT4,
	ID_SLICER_DICER, // TR4
	ID_PUZZLE_ITEM1,
	ID_PUZZLE_ITEM2,
	ID_PUZZLE_ITEM3,
	ID_PUZZLE_ITEM4,
	ID_PUZZLE_ITEM5,
	ID_PUZZLE_ITEM6,
	ID_PUZZLE_ITEM7,
	ID_PUZZLE_ITEM8,
	ID_PUZZLE_ITEM1_COMBO1,
	ID_PUZZLE_ITEM1_COMBO2,
	ID_PUZZLE_ITEM2_COMBO1,
	ID_PUZZLE_ITEM2_COMBO2,
	ID_PUZZLE_ITEM3_COMBO1,
	ID_PUZZLE_ITEM3_COMBO2,
	ID_PUZZLE_ITEM4_COMBO1,
	ID_PUZZLE_ITEM4_COMBO2,
	ID_PUZZLE_ITEM5_COMBO1,
	ID_PUZZLE_ITEM5_COMBO2,
	ID_PUZZLE_ITEM6_COMBO1,
	ID_PUZZLE_ITEM6_COMBO2,
	ID_PUZZLE_ITEM7_COMBO1,
	ID_PUZZLE_ITEM7_COMBO2,
	ID_PUZZLE_ITEM8_COMBO1,
	ID_PUZZLE_ITEM8_COMBO2,
	ID_KEY_ITEM1,
	ID_KEY_ITEM2,
	ID_KEY_ITEM3,
	ID_KEY_ITEM4,
	ID_KEY_ITEM5,
	ID_KEY_ITEM6,
	ID_KEY_ITEM7,
	ID_KEY_ITEM8,
	ID_KEY_ITEM1_COMBO1,
	ID_KEY_ITEM1_COMBO2,
	ID_KEY_ITEM2_COMBO1,
	ID_KEY_ITEM2_COMBO2,
	ID_KEY_ITEM3_COMBO1,
	ID_KEY_ITEM3_COMBO2,
	ID_KEY_ITEM4_COMBO1,
	ID_KEY_ITEM4_COMBO2,
	ID_KEY_ITEM5_COMBO1,
	ID_KEY_ITEM5_COMBO2,
	ID_KEY_ITEM6_COMBO1,
	ID_KEY_ITEM6_COMBO2,
	ID_KEY_ITEM7_COMBO1,
	ID_KEY_ITEM7_COMBO2,
	ID_KEY_ITEM8_COMBO1,
	ID_KEY_ITEM8_COMBO2,
	ID_PICKUP_ITEM1,
	ID_PICKUP_ITEM2,
	ID_PICKUP_ITEM3,
	ID_PICKUP_ITEM4, // Goldrose
	ID_PICKUP_ITEM1_COMBO1,
	ID_PICKUP_ITEM1_COMBO2,
	ID_PICKUP_ITEM2_COMBO1,
	ID_PICKUP_ITEM2_COMBO2,
	ID_PICKUP_ITEM3_COMBO1,
	ID_PICKUP_ITEM3_COMBO2,
	ID_PICKUP_ITEM4_COMBO1,
	ID_PICKUP_ITEM4_COMBO2,
	ID_EXAMINE1,
	ID_EXAMINE2,
	ID_EXAMINE3,
	ID_BOTTLE,
	ID_CLOTH,
	ID_WET_CLOTH,
	ID_COSH,
	ID_HAMMER_ITEM,
	ID_CROWBAR_ITEM,
	ID_BURNING_TORCH_ITEM,
	ID_PUZZLE_HOLE1,
	ID_PUZZLE_HOLE2,
	ID_PUZZLE_HOLE3,
	ID_PUZZLE_HOLE4,
	ID_PUZZLE_HOLE5,
	ID_PUZZLE_HOLE6,
	ID_PUZZLE_HOLE7,
	ID_PUZZLE_HOLE8,
	ID_PUZZLE_DONE1,
	ID_PUZZLE_DONE2,
	ID_PUZZLE_DONE3,
	ID_PUZZLE_DONE4,
	ID_PUZZLE_DONE5,
	ID_PUZZLE_DONE6,
	ID_PUZZLE_DONE7,
	ID_PUZZLE_DONE8,
	ID_KEY_HOLE1,
	ID_KEY_HOLE2,
	ID_KEY_HOLE3,
	ID_KEY_HOLE4,
	ID_KEY_HOLE5,
	ID_KEY_HOLE6,
	ID_KEY_HOLE7,
	ID_KEY_HOLE8,
	ID_SWITCH_TYPE1,
	ID_SWITCH_TYPE2,
	ID_SWITCH_TYPE3,
	ID_SWITCH_TYPE4,
	ID_SWITCH_TYPE5,
	ID_SWITCH_TYPE6,
	ID_SWITCH_TYPE7,
	ID_SWITCH_TYPE8,
	ID_AIRLOCK_SWITCH,
	ID_UNDERWATER_SWITCH1,
	ID_UNDERWATER_SWITCH2,
	ID_TURN_SWITCH,
	ID_COG_SWITCH,
	ID_LEVER_SWITCH,
	ID_JUMP_SWITCH,
	ID_CROWBAR_SWITCH,
	ID_PULLEY,
	ID_CROWDOVE_SWITCH,
	ID_DOOR_TYPE1,
	ID_SARCOPHAGUS, // TR4
	ID_DOOR_TYPE2,
	ID_BIRD_BLADE, // TR4 - Control
	ID_DOOR_TYPE3,
	ID_KAYAK, // TR3 - 14
	ID_DOOR_TYPE4,
	ID_KAYAK_LARA_ANIMS, // TR3 - 
	ID_DOOR_TYPE5,
	ID_BOAT, // TR3 - 15
	ID_DOOR_TYPE6,
	ID_LARA_BOAT_ANIMS, // TR3
	ID_DOOR_TYPE7,
	ID_TRIBESMAN_WITH_AX, // TR3 - 20
	ID_DOOR_TYPE8,
	ID_TRIBESMAN_WITH_DARTS, // TR3 - 21
	ID_CLOSED_DOOR1,
	ID_SCUBA_DIVER, // TR3 - 26
	ID_CLOSED_DOOR2,
	ID_CROW_TR3, // TR3 - 27
	ID_CLOSED_DOOR3,
	ID_TIGER, // TR3 - 28
	ID_CLOSED_DOOR4,
	ID_EAGLE, // TR3 - 29
	ID_CLOSED_DOOR5,
	ID_RAPTOR, // TR3 - 288
	ID_CLOSED_DOOR6,
	ID_TYRANNOSAUR, // TR3 - 287
	ID_LIFT_DOORS1,
	ID_COBRA, // TR3 - 69
	ID_LIFT_DOORS2,
	ID_MONKEY, // TR3 - 71
	ID_PUSHPULL_DOOR1,
	ID_MP_WITH_STICK, // TR3 - 60
	ID_PUSHPULL_DOOR2,
	ID_MP_WITH_GUN, // TR3 - 61
	ID_KICK_DOOR1,
	ID_WALL_MOUNTED_BLADE, // TR3 - 111
	ID_KICK_DOOR2,
	ID_SPIKEY_WALL, // TR3 - 113
	ID_UNDERWATER_DOOR,
	ID_ZIPLINE_HANDLE, // TR3 - 127
	ID_DOUBLE_DOORS,
	ID_PIRAHNAS, // TR3 - 338
	ID_SEQUENCE_DOOR1,
	ID_SEQUENCE_SWITCH1,
	ID_SEQUENCE_SWITCH2,
	ID_SEQUENCE_SWITCH3,
	ID_STEEL_DOOR,
	ID_GOD_HEAD,
	ID_PISTOLS_ITEM,
	ID_PISTOLS_AMMO_ITEM,
	ID_UZI_ITEM,
	ID_UZI_AMMO_ITEM,
	ID_SHOTGUN_ITEM,
	ID_SHOTGUN_AMMO1_ITEM,
	ID_SHOTGUN_AMMO2_ITEM,
	ID_CROSSBOW_ITEM,
	ID_CROSSBOW_AMMO1_ITEM,
	ID_CROSSBOW_AMMO2_ITEM,
	ID_CROSSBOW_BOLT,
	ID_HK_ITEM,
	ID_HK_AMMO_ITEM,
	ID_REVOLVER_ITEM,
	ID_REVOLVER_AMMO_ITEM,
	ID_BIGMEDI_ITEM,
	ID_SMALLMEDI_ITEM,
	ID_LASERSIGHT_ITEM,
	ID_BINOCULARS_ITEM,
	ID_SILENCER_ITEM,
	ID_FLARE_ITEM,
	ID_FLARE_INV_ITEM,
	ID_COMPASS_ITEM,
	ID_INVENTORY_PASSPORT,
	ID_INVENTORY_SUNGLASSES,
	ID_INVENTORY_KEYS,
	ID_INVENTORY_HEADPHONES,
	ID_INVENTORY_POLAROID,
	ID_DC_SAVE_INV_ITEM,
	ID_SMOKE_EMITTER_WHITE,
	ID_SMOKE_EMITTER_BLACK,
	ID_STEAM_EMITTER,
	ID_EARTHQUAKE,
	ID_BUBBLES,
	ID_WATERFALLMIST,
	ID_GUNSHELL,
	ID_SHOTGUNSHELL,
	ID_GUN_FLASH,
	ID_COLOURED_LIGHT,
	ID_BLINKING_LIGHT,
	ID_PULSE_LIGHT,
	ID_STROBE_LIGHT,
	ID_ELECTRICAL_LIGHT,
	ID_LENS_FLARE,
	ID_AI_GUARD,
	ID_AI_AMBUSH,
	ID_AI_PATROL1,
	ID_AI_MODIFY,
	ID_AI_FOLLOW,
	ID_AI_PATROL2,
	ID_AI_X1,
	ID_AI_X2,
	ID_LARA_START_POS,
	ID_TELEPORTER,
	ID_LIFT_TELEPORTER,
	ID_RAISING_COG,
	ID_LASERS,
	ID_STEAM_LASERS,
	ID_FLOOR_LASERS,
	ID_KILL_ALL_TRIGGERS,
	ID_TRIGGER_TRIGGERER,
	ID_HIGH_OBJECT1,
	ID_HIGH_OBJECT2,
	ID_SMASH_OBJECT1,
	ID_SMASH_OBJECT2,
	ID_SMASH_OBJECT3,
	ID_SMASH_OBJECT4,
	ID_SMASH_OBJECT5,
	ID_SMASH_OBJECT6,
	ID_SMASH_OBJECT7,
	ID_SMASH_OBJECT8,
	ID_MESHSWAP1,
	ID_MESHSWAP2,
	ID_MESHSWAP3,
	ID_BODY_PART,
	ID_CAMERA_TARGET,
	ID_WATERFALL1,
	ID_WATERFALL2,
	ID_WATERFALL3,
	ID_FISHTANK,
	ID_WATERFALLSS1,
	ID_WATERFALLSS2,
	ID_ANIMATING1,
	ID_BARRACUDA, // TR2 - 28
	ID_ANIMATING2,
	ID_MONK_WITH_STICK, // TR2 - 53
	ID_ANIMATING3,
	ID_MONK_WITH_KNIFE, // TR2 - 54
	ID_ANIMATING4,
	ID_DISK_SHOOTER, // TR2 - 62 
	ID_ANIMATING5,
	ID_DISK, // TR2 - 61
	ID_ANIMATING6,
	ID_STATUE_WITH_BLADE, // TR2 - 82
	ID_ANIMATING7,
	ID_SPRINGBOARD, // TR2 - 57?
	ID_ANIMATING8,
	ID_ROLLING_SPINDLE, // TR2 - 80
	ID_ANIMATING9,
	ID_WOLF, // TR1 - 7
	ID_ANIMATING10,
	ID_BEAR, // TR1 - 8
	ID_ANIMATING11,
	ID_GORILLA, // TR1 - 15
	ID_ANIMATING12,
	ID_WINGED_MUMMY, // TR1 - 20
	ID_ANIMATING13,
	ID_CENTAUR_MUTANT,  // TR1 - 23
	ID_ANIMATING14,
	ID_WINGED_NATLA, // TR1 - 33
	ID_ANIMATING15,
	ID_GIANT_MUTANT, // TR1 - 34
	ID_ANIMATING16,
	ID_SCUBA_HARPOON,
	ID_BRIDGE_FLAT,
	ID_BRIDGE_FLAT_MIP,
	ID_BRIDGE_TILT1,
	ID_BRIDGE_TILT1_MIP,
	ID_BRIDGE_TILT2,
	ID_BRIDGE_TILT2_MIP,
	ID_HORIZON,
	ID_SKY_GRAPHICS,
	ID_BINOCULAR_GRAPHICS,
	ID_TARGET_GRAPHICS,
	ID_DEFAULT_SPRITES,
	ID_MISC_SPRITES,
	ID_SLOT_460,
	ID_CAIRO_SAS,
	ID_NEW_SLOT_462,
	ID_NEW_SLOT_463,
	ID_NEW_SLOT_464,
	ID_NEW_SLOT_465,
	ID_NEW_SLOT_466,
	ID_NEW_SLOT_467,
	ID_NEW_SLOT_468,
	ID_NEW_SLOT_469,
	ID_NEW_SLOT_470,
	ID_NEW_SLOT_471,
	ID_NEW_SLOT_472,
	ID_NEW_SLOT_473,
	ID_NEW_SLOT_474,
	ID_NEW_SLOT_475,
	ID_NEW_SLOT_476,
	ID_NEW_SLOT_477,
	ID_NEW_SLOT_478,
	ID_NEW_SLOT_479,
	ID_NEW_SLOT_480,
	ID_NEW_SLOT_481,
	ID_NEW_SLOT_482,
	ID_NEW_SLOT_483,
	ID_NEW_SLOT_484,
	ID_NEW_SLOT_485,
	ID_NEW_SLOT_486,
	ID_NEW_SLOT_487,
	ID_NEW_SLOT_488
} GAME_OBJECT_ID;
