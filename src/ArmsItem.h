// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#pragma once

#include "WindowsWrapper.h"

// Limits for the amount of weapons and items
#define ARMS_MAX 8
#define ITEM_MAX 32

// "Arms" is a synonym of "weapon" here
// "Code" means "ID" here
// "Num" often means "ammo" here

/// Weapon struct
typedef struct ARMS
{
	/// ID of the weapon
	int code;

	/// Current level of the weapon
	int level;

	/// Current EXP of the weapon. It is counted from the current level (it's reset to 0 at each level up)
	int exp;

	/// Maximum ammunition
	int max_num;

	/// Current ammunition
	int num;
} ARMS;

typedef struct ITEM
{
	/// ID of the item
	int code;
} ITEM;



/// Contains data for all the weapons the character currently has
extern ARMS gArmsData[ARMS_MAX];

/// Contains data for all the items the character currently has
extern ITEM gItemData[ITEM_MAX];

/// Currently selected weapon
extern int gSelectedArms;

/// Currently selected item
extern int gSelectedItem;

extern int gCampTitleY;

/// True if we're in the items section of the inventory (not in the weapons section) (only relevant when the inventory is open)
extern BOOL gCampActive;

/// X coordinate for the weapons HUD section. Set it to 32 for the forward weapon rotation "animation", 0 for the reverse weapon rotation "animation" and 16 to immobilise it
extern int gArmsEnergyX;



/// Clear the weapons array, reverting it to the default state (no weapons) and adjust variables (initialize weapons basically)
void ClearArmsData(void);

/// Clear the item array, reverting it to the default state (no items) (initialize items basically)
void ClearItemData(void);


/// Add code to the weapons, setting max_num as the max ammo, or find code and add max_num to its ammo. Fails if no space is available and the weapon isn't
/// already present
BOOL AddArmsData(long code, long max_num);

/// Remove code from the weapons. Fails if code is not found
BOOL SubArmsData(long code);

/// Replace code1 with code2, setting max_num as its max ammo. Fails if code1 is not found
BOOL TradeArms(long code1, long code2, long max_num);


/// Add code to the items. Fails if no space is left
BOOL AddItemData(long code);

/// Remove code from the items. Fails if code is not found
BOOL SubItemData(long code);


/// Inventory loop. Returns mode.
int CampLoop(void);


/// Search for a in the items. Returns whether a was found
BOOL CheckItem(long a);

/// Search for a in the weapons. Returns whether a was found
BOOL CheckArms(long a);


/// Remove num ammo from the currently selected weapon. Returns whether there was any ammo left to fire
BOOL UseArmsEnergy(long num);

/// Add num ammo to the currently selected weapon (capped at the maximum ammunition). Returns true
BOOL ChargeArmsEnergy(long num);

/// Set every weapons ammunition to its maximum ammunition
void FullArmsEnergy(void);


// "Rotation" means "Weapons currently owned by the player (present in the weapons array)"

/// Change the current weapon to the next one in the rotation. Returns the ID of the newly selected weapon
int RotationArms(void);

/// Change the current weapon to the previous one in the rotation. Returns the ID of the newly selected weapon
int RotationArmsRev(void);

/// Change the current weapon to be the first one and play the usual rotation animation
void ChangeToFirstArms(void);
