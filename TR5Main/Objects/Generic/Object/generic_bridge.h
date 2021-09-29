#pragma once
#include "Specific\phd_global.h"
#include "level.h"
#include "items.h"
#include "floordata.h"

using namespace TEN::Floordata;

void InitialiseBridge(short itemNumber);
int GetOffset(short angle, int x, int z);

template <int tilt>
std::optional<int> BridgeFloor(short itemNumber, int x, int y, int z)
{
	auto item = &g_Level.Items[itemNumber];
	auto bboxHeight = GetBridgeItemIntersect(itemNumber, x, y, z, false);

	if (bboxHeight.has_value() && tilt != 0)
	{
		const auto height = item->pos.yPos + tilt * (GetOffset(item->pos.yRot, x, z) / 4 + SECTOR(1) / 8);
		return std::optional{ height };
	}
	return bboxHeight;
}

template <int tilt>
std::optional<int> BridgeCeiling(short itemNumber, int x, int y, int z)
{
	auto item = &g_Level.Items[itemNumber];
	auto bboxHeight = GetBridgeItemIntersect(itemNumber, x, y, z, true);

	if (bboxHeight.has_value() && tilt != 0)
	{
		const auto height = item->pos.yPos + tilt * (GetOffset(item->pos.yRot, x, z) / 4 + SECTOR(1) / 8);
		return std::optional{ height + SECTOR(1) / 16 }; // To be customized with Lua
	}
	return bboxHeight;
}

template <int tilt>
int BridgeFloorBorder(short itemNumber)
{
	return GetBridgeBorder(itemNumber, false);
}

template <int tilt>
int BridgeCeilingBorder(short itemNumber)
{
	return GetBridgeBorder(itemNumber, true);
}
