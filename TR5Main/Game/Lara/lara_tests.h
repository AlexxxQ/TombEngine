#pragma once
#include "Game/collision/collide_room.h"
#include "Game/Lara/lara_struct.h"
#include "Game/Lara/lara_test_structs.h"

struct ITEM_INFO;
struct COLL_INFO;

// -----------------------------
// TEST FUNCTIONS
// For State Control & Collision
// -----------------------------

bool TestValidLedge(ITEM_INFO* item, COLL_INFO* coll, bool ignoreHeadroom = false, bool heightLimit = false);
bool TestValidLedgeAngle(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraKeepLow(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraSlide(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraHang(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraHangJump(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraHangJumpUp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraClimbStance(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraHangOnClimbWall(ITEM_INFO* item, COLL_INFO* coll);
int  TestLaraEdgeCatch(ITEM_INFO* item, COLL_INFO* coll, int* edge);
bool TestLaraValidHangPos(ITEM_INFO* item, COLL_INFO* coll);
CORNER_RESULT TestLaraHangCorner(ITEM_INFO* item, COLL_INFO* coll, float testAngle);
bool TestHangSwingIn(ITEM_INFO* item);
bool TestLaraHangSideways(ITEM_INFO* item, COLL_INFO* coll, short angle);
bool LaraPositionOnLOS(ITEM_INFO* item, short ang, int dist);
bool TestLaraFacingCorner(ITEM_INFO* item, short angle, int dist);

int LaraFloorFront(ITEM_INFO* item, short ang, int dist);
int LaraCeilingFront(ITEM_INFO* item, short ang, int dist, int h);
COLL_RESULT LaraCollisionFront(ITEM_INFO* item, short ang, int dist);
COLL_RESULT LaraCeilingCollisionFront(ITEM_INFO* item, short ang, int dist, int h);
COLL_RESULT LaraCollisionAboveFront(ITEM_INFO* item, short ang, int dist, int h);

bool TestLaraFall(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyGrab(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyFall(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraLand(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraWaterStepOut(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraWaterClimbOut(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraLadderClimbOut(ITEM_INFO* item, COLL_INFO* coll);
void TestLaraWaterDepth(ITEM_INFO* item, COLL_INFO* coll);

#ifndef NEW_TIGHTROPE
void GetTighRopeFallOff(int Regularity);
#endif

bool IsStandingWeapon(LARA_WEAPON_TYPE gunType);
bool IsJumpState(LARA_STATE state);
bool IsRunJumpCountState(LARA_STATE state);
bool IsVaultState(LARA_STATE state);

bool TestLaraSplat(ITEM_INFO* item, int dist, int height, int side = 0);
bool TestLaraPose(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraStep(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepUp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepDown(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyStep(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraMoveTolerance(ITEM_INFO* item, COLL_INFO* coll, MoveTestSetup testSetup);
bool TestLaraRunForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraWalkForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraWalkBack(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraRunBack(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepLeft(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepRight(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraWadeForwardSwamp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraWalkBackSwamp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepLeftSwamp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraStepRightSwamp(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraCrawlMoveTolerance(ITEM_INFO* item, COLL_INFO* coll, MoveTestSetup testSetup);
bool TestLaraCrawlForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrawlBack(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrouchToCrawl(ITEM_INFO* item);
bool TestLaraCrouchRoll(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraMonkeyMoveTolerance(ITEM_INFO* item, COLL_INFO* coll, MonkeyMoveTestSetup testSetup);
bool TestLaraMonkeyForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyBack(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyShimmyLeft(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraMonkeyShimmyRight(ITEM_INFO* item, COLL_INFO* coll);

VaultTestResult TestLaraVaultTolerance(ITEM_INFO* item, COLL_INFO* coll, VaultTestSetup testSetup);
VaultTestResult TestLaraVault2Steps(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVault3Steps(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVault1StepToCrouch(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVault2StepsToCrouch(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVault3StepsToCrouch(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVaultAutoJump(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraLadderAutoJump(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraLadderMount(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraMonkeyAutoJump(ITEM_INFO* item, COLL_INFO* coll);
VaultTestResult TestLaraVault(ITEM_INFO* item, COLL_INFO* coll);
bool TestAndSetLaraLadder(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraCrawlVaultTolerance(ITEM_INFO* item, COLL_INFO* coll, CrawlVaultTestSetup testSetup);
bool TestLaraCrawlUpStep(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrawlDownStep(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrawlExitDownStep(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrawlExitJump(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraCrawlToHang(ITEM_INFO* item, COLL_INFO* coll);
CrawlVaultTestResult TestLaraCrawlVault(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraJumpTolerance(ITEM_INFO* item, COLL_INFO* coll, JumpTestSetup testSetup);
bool TestLaraRunJumpForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraJumpForward(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraJumpBack(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraJumpLeft(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraJumpRight(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraJumpUp(ITEM_INFO* item, COLL_INFO* coll);

bool TestLaraPoleCollision(ITEM_INFO* item, COLL_INFO* coll, bool up, float offset = 0.0f);
bool TestLaraPoleUp(ITEM_INFO* item, COLL_INFO* coll);
bool TestLaraPoleDown(ITEM_INFO* item, COLL_INFO* coll);
