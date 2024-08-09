// THIS IS DECOMPILED PROPRIETARY CODE - USE AT YOUR OWN RISK.
//
// The original code belongs to Daisuke "Pixel" Amaya.
//
// Modifications and custom code are under the MIT licence.
// See LICENCE.txt for details.

#include "Stage.h"

#include <stdio.h>
#include <string.h>

#include "WindowsWrapper.h"

#include "Back.h"
#include "Boss.h"
#include "Bullet.h"
#include "Caret.h"
#include "Draw.h"
#include "Flash.h"
#include "Frame.h"
#include "Map.h"
#include "MapName.h"
#include "MyChar.h"
#include "NpChar.h"
#include "Organya.h"
#include "TextScr.h"
#include "ValueView.h"

#ifdef JAPANESE
#define STAGE_ENTRY(parts, map, bkType, back, npc, boss, boss_no, name_en, name_jp) {parts, map, bkType, back, npc, boss, boss_no, name_jp}
#else
#define STAGE_ENTRY(parts, map, bkType, back, npc, boss, boss_no, name_en, name_jp) {parts, map, bkType, back, npc, boss, boss_no, name_en}
#endif

int gStageNo;
MusicID gMusicNo;
unsigned int gOldPos;
MusicID gOldNo;

// Note: Pixel made numerous capitalisation errors when creating this table.
// This isn't a problem for Windows, because of its case-insensitive filesystem.
const STAGE_TABLE gTMT[] = {
	STAGE_ENTRY("0",      "0",      BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "Null",                  "\x96\xB3"),	/* 無 */
	STAGE_ENTRY("Pens",   "Pens1",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Arthur's House",        "\x83\x41\x81\x5B\x83\x54\x81\x5B\x82\xCC\x89\xC6"),	/* アーサーの家 */
	STAGE_ENTRY("Eggs",   "Eggs",   BACKGROUND_TYPE_MOVE_DISTANT, "BkGreen", "Eggs1",  "Ravil",  0, "Egg Corridor",          "\x83\x5E\x83\x7D\x83\x53\x89\xF1\x98\x4C"),	/* タマゴ回廊 */
	STAGE_ENTRY("EggX",   "EggX",   BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Egg No. 00",            "\x83\x5E\x83\x7D\x83\x53\x20\x4E\x6F\x2E\x30\x30"),	/* タマゴ No.00 */
	STAGE_ENTRY("EggIn",  "Egg6",   BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Egg No. 06",            "\x83\x5E\x83\x7D\x83\x53\x20\x4E\x6F\x2E\x30\x36"),	/* タマゴ No.06 */
	STAGE_ENTRY("Store",  "EggR",   BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Egg Observation Room",  "\x83\x5E\x83\x7D\x83\x53\x8A\xC4\x8E\x8B\x8E\xBA"),	/* タマゴ監視室 */
	STAGE_ENTRY("Weed",   "Weed",   BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Weed",   "0",      0, "Grasstown",             "\x83\x4E\x83\x54\x83\x80\x83\x89"),	/* クサムラ */
	STAGE_ENTRY("Barr",   "Santa",  BACKGROUND_TYPE_BLACK,        "bk0",     "Weed",   "0",      0, "Santa's House",         "\x83\x54\x83\x93\x83\x5E\x82\xCC\x89\xC6"),	/* サンタの家 */
	STAGE_ENTRY("Barr",   "Chako",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Chaco's House",         "\x83\x60\x83\x83\x83\x52\x82\xCC\x89\xC6"),	/* チャコの家 */
	STAGE_ENTRY("Maze",   "MazeI",  BACKGROUND_TYPE_BLACK,        "bk0",     "Maze",   "0",      0, "Labyrinth I",           "\x96\xC0\x8B\x7B\x82\x68"),	/* 迷宮Ｉ */
	STAGE_ENTRY("Sand",   "Sand",   BACKGROUND_TYPE_MOVE_DISTANT, "BkGreen", "Sand",   "Omg",    1, "Sand Zone",             "\x8D\xBB\x8B\xE6"),	/* 砂区 */
	STAGE_ENTRY("Mimi",   "Mimi",   BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Mimiga Village",        "\x83\x7E\x83\x7E\x83\x4B\x81\x5B\x82\xCC\x91\xBA"),	/* ミミガーの村 */
	STAGE_ENTRY("Cave",   "Cave",   BACKGROUND_TYPE_BLACK,        "bk0",     "Cemet",  "0",      0, "First Cave",            "\x8D\xC5\x8F\x89\x82\xCC\x93\xB4\x8C\x41"),	/* 最初の洞窟 */
	STAGE_ENTRY("Cave",   "Start",  BACKGROUND_TYPE_BLACK,        "bk0",     "Cemet",  "0",      0, "Start Point",           "\x83\x58\x83\x5E\x81\x5B\x83\x67\x92\x6E\x93\x5F"),	/* スタート地点 */
	STAGE_ENTRY("Mimi",   "Barr",   BACKGROUND_TYPE_BLACK,        "bk0",     "Cemet",  "Bllg",   0, "Shack",                 "\x83\x6F\x83\x89\x83\x62\x83\x4E\x8F\xAC\x89\xAE"),	/* バラック小屋 */
	STAGE_ENTRY("Mimi",   "Pool",   BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Reservoir",             "\x92\x99\x90\x85\x92\x72"),	/* 貯水池 */
	STAGE_ENTRY("Mimi",   "Cemet",  BACKGROUND_TYPE_BLACK,        "bk0",     "Cemet",  "0",      0, "Graveyard",             "\x82\xCD\x82\xA9\x82\xCE"),	/* はかば */
	STAGE_ENTRY("Mimi",   "Plant",  BACKGROUND_TYPE_MOVE_DISTANT, "BkGreen", "Plant",  "0",      0, "Yamashita Farm",        "\x8E\x52\x89\xBA\x94\x5F\x89\x80"),	/* 山下農園 */
	STAGE_ENTRY("Store",  "Shelt",  BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Shelter",               "\x83\x56\x83\x46\x83\x8B\x83\x5E\x81\x5B"),	/* シェルター */
	STAGE_ENTRY("Pens",   "Comu",   BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Assembly Hall",         "\x8F\x57\x89\xEF\x8F\xEA"),	/* 集会場 */
	STAGE_ENTRY("Mimi",   "MiBox",  BACKGROUND_TYPE_BLACK,        "bk0",     "0",      "0",      0, "Save Point",            "\x83\x5A\x81\x5B\x83\x75\x83\x7C\x83\x43\x83\x93\x83\x67"),	/* セーブポイント */
	STAGE_ENTRY("Store",  "EgEnd1", BACKGROUND_TYPE_BLACK,        "bk0",     "0",      "0",      0, "Side Room",             "\x83\x5E\x83\x7D\x83\x53\x89\xF1\x98\x4C\x82\xCC\x8C\xC2\x8E\xBA"),	/* タマゴ回廊の個室 */
	STAGE_ENTRY("Store",  "Cthu",   BACKGROUND_TYPE_BLACK,        "bk0",     "0",      "0",      0, "Cthulhu's Abode",       "\x83\x4E\x83\x67\x83\x44\x83\x8B\x81\x5B\x82\xCC\x8F\x5A\x8F\x88"),	/* クトゥルーの住処 */
	STAGE_ENTRY("EggIn",  "Egg1",   BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Egg No. 01",            "\x83\x5E\x83\x7D\x83\x53\x20\x4E\x6F\x2E\x30\x31"),	/* タマゴ No.01 */
	STAGE_ENTRY("Pens",   "Pens2",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Arthur's House",        "\x83\x41\x81\x5B\x83\x54\x81\x5B\x82\xCC\x89\xC6"),	/* アーサーの家 */
	STAGE_ENTRY("Barr",   "Malco",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Weed",   "Bllg",   0, "Power Room",            "\x93\x64\x8C\xB9\x8E\xBA"),	/* 電源室 */
	STAGE_ENTRY("Barr",   "WeedS",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "0",      "0",      0, "Save Point",            "\x83\x5A\x81\x5B\x83\x75\x83\x7C\x83\x43\x83\x93\x83\x67"),	/* セーブポイント */
	STAGE_ENTRY("Store",  "WeedD",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "0",      "0",      0, "Execution Chamber",     "\x8F\x88\x8C\x59\x8E\xBA"),	/* 処刑室 */
	STAGE_ENTRY("Weed",   "Frog",   BACKGROUND_TYPE_MOVE_NEAR,    "BkGreen", "Weed",   "Frog",   2, "Gum",                   "\x83\x4B\x83\x80"),	/* ガム */
	STAGE_ENTRY("Sand",   "Curly",  BACKGROUND_TYPE_BLACK,        "bk0",     "Sand",   "Curly",  0, "Sand Zone Residence",   "\x8D\xBB\x8B\xE6\x92\x93\x8D\xDD\x8F\x8A"),	/* 砂区駐在所 */
	STAGE_ENTRY("Pens",   "WeedB",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Ravil",  "0",      0, "Grasstown Hut",         "\x83\x4E\x83\x54\x83\x80\x83\x89\x82\xCC\x8F\xAC\x89\xAE"),	/* クサムラの小屋 */
	STAGE_ENTRY("River",  "Stream", BACKGROUND_TYPE_AUTOSCROLL,   "BkBlue",  "Stream", "IronH",  5, "Main Artery",           "\x91\xE5\x93\xAE\x96\xAC"),	/* 大動脈 */
	STAGE_ENTRY("Pens",   "CurlyS", BACKGROUND_TYPE_BLACK,        "bk0",     "Sand",   "Curly",  0, "Small Room",            "\x8F\xAC\x95\x94\x89\xAE"),	/* 小部屋 */
	STAGE_ENTRY("Barr",   "Jenka1", BACKGROUND_TYPE_BLACK,        "bk0",     "Sand",   "Bllg",   0, "Jenka's House",         "\x83\x57\x83\x46\x83\x93\x83\x4A\x82\xCC\x89\xC6"),	/* ジェンカの家 */
	STAGE_ENTRY("Sand",   "Dark",   BACKGROUND_TYPE_MOVE_DISTANT, "bkBlack", "Sand",   "0",      0, "Deserted House",        "\x94\x70\x89\xAE"),	/* 廃屋 */
	STAGE_ENTRY("Gard",   "Gard",   BACKGROUND_TYPE_MOVE_DISTANT, "BkGard",  "Toro",   "Bllg",   0, "Sand Zone Storehouse",  "\x8D\xBB\x8B\xE6\x91\x71\x8C\xC9"),	/* 砂区倉庫 */
	STAGE_ENTRY("Barr",   "Jenka2", BACKGROUND_TYPE_BLACK,        "bk0",     "Sand",   "Bllg",   0, "Jenka's House",         "\x83\x57\x83\x46\x83\x93\x83\x4A\x82\xCC\x89\xC6"),	/* ジェンカの家 */
	STAGE_ENTRY("Sand",   "SandE",  BACKGROUND_TYPE_MOVE_DISTANT, "BkGreen", "Sand",   "Bllg",   0, "Sand Zone",             "\x8D\xBB\x8B\xE6"),	/* 砂区 */
	STAGE_ENTRY("Maze",   "MazeH",  BACKGROUND_TYPE_BLACK,        "bk0",     "Maze",   "0",      0, "Labyrinth H",           "\x96\xC0\x8B\x7B\x82\x67"),	/* 迷宮Ｈ */
	STAGE_ENTRY("Maze",   "MazeW",  BACKGROUND_TYPE_MOVE_DISTANT, "BkMaze",  "Maze",   "X",      3, "Labyrinth W",           "\x96\xC0\x8B\x7B\x82\x76"),	/* 迷宮Ｗ */
	STAGE_ENTRY("Maze",   "MazeO",  BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "Camp",                  "\x83\x4C\x83\x83\x83\x93\x83\x76"),	/* キャンプ */
	STAGE_ENTRY("Maze",   "MazeD",  BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "Dark",   0, "Clinic Ruins",          "\x90\x66\x97\xC3\x8F\x8A\x90\xD5"),	/* 診療所跡 */
	STAGE_ENTRY("Store",  "MazeA",  BACKGROUND_TYPE_BLACK,        "bk0",     "Maze",   "0",      0, "Labyrinth Shop",        "\x96\xC0\x8B\x7B\x82\xCC\x93\x58"),	/* 迷宮の店 */
	STAGE_ENTRY("Maze",   "MazeB",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Maze",   "0",      0, "Labyrinth B",           "\x96\xC0\x8B\x7B\x82\x61"),	/* 迷宮Ｂ */
	STAGE_ENTRY("Maze",   "MazeS",  BACKGROUND_TYPE_MOVE_NEAR,    "BkGray",  "Maze",   "Bllg",   0, "Boulder Chamber",       "\x91\xE5\x90\xCE\x82\xCC\x8D\xC7\x82\xAE\x8F\x8A"),	/* 大石の塞ぐ所 */
	STAGE_ENTRY("Maze",   "MazeM",  BACKGROUND_TYPE_MOVE_DISTANT, "BkRed",   "Maze",   "0",      0, "Labyrinth M",           "\x96\xC0\x8B\x7B\x82\x6C"),	/* 迷宮Ｍ */
	STAGE_ENTRY("Cave",   "Drain",  BACKGROUND_TYPE_WATER,        "BkWater", "Cemet",  "0",      0, "Dark Place",            "\x88\xC3\x82\xA2\x8F\x8A"),	/* 暗い所 */
	STAGE_ENTRY("Almond", "Almond", BACKGROUND_TYPE_WATER,        "BkWater", "Cemet",  "Almo1",  4, "Core",                  "\x83\x52\x83\x41"),	/* コア */
	STAGE_ENTRY("River",  "River",  BACKGROUND_TYPE_MOVE_NEAR,    "bkGreen", "Weed",   "0",      0, "Waterway",              "\x90\x85\x98\x48"),	/* 水路 */
	STAGE_ENTRY("Eggs",   "Eggs2",  BACKGROUND_TYPE_MOVE_DISTANT, "BkGreen", "Eggs2",  "0",      0, "Egg Corridor?",         "\x83\x5E\x83\x7D\x83\x53\x89\xF1\x98\x4C\x81\x48"),	/* タマゴ回廊？ */
	STAGE_ENTRY("Store",  "Cthu2",  BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Cthulhu's Abode?",      "\x83\x4E\x83\x67\x83\x44\x83\x8B\x81\x5B\x82\xCC\x8F\x5A\x8F\x88\x81\x48"),	/* クトゥルーの住処？ */
	STAGE_ENTRY("Store",  "EggR2",  BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "TwinD",  6, "Egg Observation Room?", "\x83\x5E\x83\x7D\x83\x53\x8A\xC4\x8E\x8B\x8E\xBA\x81\x48"),	/* タマゴ監視室？ */
	STAGE_ENTRY("EggX",   "EggX2",  BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Egg No. 00",            "\x83\x5E\x83\x7D\x83\x53\x20\x4E\x6F\x2E\x30\x30"),	/* タマゴ No.00 */
	STAGE_ENTRY("Oside",  "Oside",  BACKGROUND_TYPE_CLOUDS_WINDY, "BkMoon",  "Moon",   "0",      0, "Outer Wall",            "\x8A\x4F\x95\xC7"),	/* 外壁 */
	STAGE_ENTRY("Store",  "EgEnd2", BACKGROUND_TYPE_BLACK,        "bk0",     "Eggs1",  "0",      0, "Side Room",             "\x83\x5E\x83\x7D\x83\x53\x89\xF1\x98\x4C\x82\xCC\x8C\xC2\x8E\xBA"),	/* タマゴ回廊の個室 */
	STAGE_ENTRY("Store",  "Itoh",   BACKGROUND_TYPE_MOVE_NEAR,    "bkBlue",  "Guest",  "0",      0, "Storehouse",            "\x91\x71\x8C\xC9"),	/* 倉庫 */
	STAGE_ENTRY("Cent",   "Cent",   BACKGROUND_TYPE_MOVE_DISTANT, "bkGreen", "Guest",  "Cent",   0, "Plantation",            "\x91\xE5\x94\x5F\x89\x80"),	/* 大農園 */
	STAGE_ENTRY("Jail",   "Jail1",  BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "Cent",   0, "Jail No. 1",            "\x91\xE6\x82\x50\x98\x53"),	/* 第１牢 */
	STAGE_ENTRY("Jail",   "Momo",   BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "Hideout",               "\x83\x4A\x83\x4E\x83\x8C\x83\x4B"),	/* カクレガ */
	STAGE_ENTRY("Jail",   "lounge", BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "Rest Area",             "\x8B\x78\x8C\x65\x8F\x8A"),	/* 休憩所 */
	STAGE_ENTRY("Store",  "CentW",  BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "Cent",   0, "Teleporter",            "\x93\x5D\x91\x97\x8E\xBA"),	/* 転送室 */
	STAGE_ENTRY("Store",  "Jail2",  BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "Cent",   0, "Jail No. 2",            "\x91\xE6\x82\x51\x98\x53"),	/* 第２牢 */
	STAGE_ENTRY("White",  "Blcny1", BACKGROUND_TYPE_CLOUDS,       "BkFog",   "Ravil",  "Heri",   0, "Balcony",               "\x83\x6F\x83\x8B\x83\x52\x83\x6A\x81\x5B"),	/* バルコニー */
	STAGE_ENTRY("Jail",   "Priso1", BACKGROUND_TYPE_BLACK,        "BkGray",  "Red",    "0",      0, "Final Cave",            "\x8D\xC5\x8C\xE3\x82\xCC\x93\xB4\x8C\x41"),	/* 最後の洞窟 */
	STAGE_ENTRY("White",  "Ring1",  BACKGROUND_TYPE_CLOUDS,       "BkFog",   "Guest",  "Miza",   0, "Throne Room",           "\x89\xA4\x82\xCC\x8B\xCA\x8D\xC0"),	/* 王の玉座 */
	STAGE_ENTRY("White",  "Ring2",  BACKGROUND_TYPE_CLOUDS,       "BkFog",   "Guest",  "Dr",     0, "The King's Table",      "\x89\xA4\x82\xCC\x90\x48\x91\xEC"),	/* 王の食卓 */
	STAGE_ENTRY("Pens",   "Prefa1", BACKGROUND_TYPE_BLACK,        "Bk0",     "0",      "0",      0, "Prefab Building",       "\x83\x76\x83\x8C\x83\x6E\x83\x75"),	/* プレハブ */
	STAGE_ENTRY("Jail",   "Priso2", BACKGROUND_TYPE_BLACK,        "BkGray",  "Red",    "0",      0, "Last Cave (Hidden)",    "\x8D\xC5\x8C\xE3\x82\xCC\x93\xB4\x8C\x41\x81\x45\x97\xA0"),	/* 最後の洞窟・裏 */
	STAGE_ENTRY("White",  "Ring3",  BACKGROUND_TYPE_BLACK,        "Bk0",     "Miza",   "Almo2",  7, "Black Space",           "\x8D\x95\x82\xA2\x8D\x4C\x8A\xD4"),	/* 黒い広間 */
	STAGE_ENTRY("Pens",   "Little", BACKGROUND_TYPE_MOVE_NEAR,    "BkBlue",  "Guest",  "0",      0, "Little House",          "\x83\x8A\x83\x67\x83\x8B\x89\xC6"),	/* リトル家 */
	STAGE_ENTRY("White",  "Blcny2", BACKGROUND_TYPE_CLOUDS,       "BkFog",   "Ravil",  "Heri",   0, "Balcony",               "\x83\x6F\x83\x8B\x83\x52\x83\x6A\x81\x5B"),	/* バルコニー */
	STAGE_ENTRY("Fall",   "Fall",   BACKGROUND_TYPE_MOVE_DISTANT, "BkFall",  "Guest",  "Heri",   0, "Fall",                  "\x97\x8E\x89\xBA"),	/* 落下 */
	STAGE_ENTRY("White",  "Kings",  BACKGROUND_TYPE_BLACK,        "Bk0",     "Kings",  "0",      0, "u",                     "\x75"),	/* u */
	STAGE_ENTRY("Pens",   "Pixel",  BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Guest",  "0",      0, "Waterway Cabin",        "\x90\x85\x98\x48\x82\xCC\x8F\xAC\x95\x94\x89\xAE"),	/* 水路の小部屋 */
	STAGE_ENTRY("Maze",   "e_Maze", BACKGROUND_TYPE_MOVE_DISTANT, "BkMaze",  "Guest",  "Maze",   3, "",                      ""),
	STAGE_ENTRY("Barr",   "e_Jenk", BACKGROUND_TYPE_BLACK,        "bk0",     "Sand",   "Bllg",   0, "",                      ""),
	STAGE_ENTRY("Barr",   "e_Malc", BACKGROUND_TYPE_MOVE_DISTANT, "BkBlue",  "Weed",   "Bllg",   0, "",                      ""),
	STAGE_ENTRY("Mimi",   "e_Ceme", BACKGROUND_TYPE_BLACK,        "bk0",     "Plant",  "0",      0, "",                      ""),
	STAGE_ENTRY("Fall",   "e_Sky",  BACKGROUND_TYPE_MOVE_DISTANT, "BkFall",  "Guest",  "Heri",   0, "",                      ""),
	STAGE_ENTRY("Pens",   "Prefa2", BACKGROUND_TYPE_BLACK,        "Bk0",     "0",      "0",      0, "Prefab House",          "\x83\x76\x83\x8C\x83\x6E\x83\x75"),	/* プレハブ */
	STAGE_ENTRY("Hell",   "Hell1",  BACKGROUND_TYPE_MOVE_NEAR,    "bkRed",   "Hell",   "0",      0, "Sacred Ground - B1",    "\x90\xB9\x88\xE6\x92\x6E\x89\xBA\x82\x50\x8A\x4B"),	/* 聖域地下１階 */
	STAGE_ENTRY("Hell",   "Hell2",  BACKGROUND_TYPE_MOVE_NEAR,    "bkRed",   "Hell",   "0",      0, "Sacred Ground - B2",    "\x90\xB9\x88\xE6\x92\x6E\x89\xBA\x82\x51\x8A\x4B"),	/* 聖域地下２階 */
	STAGE_ENTRY("Hell",   "Hell3",  BACKGROUND_TYPE_MOVE_DISTANT, "bkRed",   "Hell",   "Press",  8, "Sacred Ground - B3",    "\x90\xB9\x88\xE6\x92\x6E\x89\xBA\x82\x52\x8A\x4B"),	/* 聖域地下３階 */
	STAGE_ENTRY("Cave",   "Mapi",   BACKGROUND_TYPE_MOVE_NEAR,    "bk0",     "Cemet",  "0",      0, "Storage",               "\x95\xA8\x92\x75"),	/* 物置 */
	STAGE_ENTRY("Hell",   "Hell4",  BACKGROUND_TYPE_BLACK,        "bk0",     "Hell",   "0",      0, "Passage?",              "\x92\xCA\x98\x48\x81\x48"),	/* 通路？ */
	STAGE_ENTRY("Hell",   "Hell42", BACKGROUND_TYPE_BLACK,        "bk0",     "Hell",   "Press",  8, "Passage?",              "\x92\xCA\x98\x48\x81\x48"),	/* 通路？ */
	STAGE_ENTRY("Hell",   "Statue", BACKGROUND_TYPE_MOVE_DISTANT, "bkBlue",  "0",      "Cent",   0, "Statue Chamber",        "\x90\xCE\x91\x9C\x82\xCC\x8A\xD4"),	/* 石像の間 */
	STAGE_ENTRY("Hell",   "Ballo1", BACKGROUND_TYPE_MOVE_NEAR,    "bkBlue",  "Priest", "Ballos", 9, "Seal Chamber",          "\x95\x95\x88\xF3\x82\xCC\x8A\xD4"),	/* 封印の間 */
	STAGE_ENTRY("White",  "Ostep",  BACKGROUND_TYPE_CLOUDS,       "BkFog",   "0",      "0",      0, "Corridor",              "\x82\xED\x82\xBD\x82\xE8\x98\x4C\x89\xBA"),	/* わたり廊下 */
	STAGE_ENTRY("Labo",   "e_Labo", BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "",                      ""),
	STAGE_ENTRY("Cave",   "Pole",   BACKGROUND_TYPE_BLACK,        "bk0",     "Guest",  "0",      0, "Hermit Gunsmith",       "\x82\xCD\x82\xAE\x82\xEA\x8F\x65\x92\x62\x96\xE8"),	/* はぐれ銃鍛冶 */
	STAGE_ENTRY("0",      "Island", BACKGROUND_TYPE_BLACK,        "bk0",     "Island", "0",      0, "",                      ""),
	STAGE_ENTRY("Hell",   "Ballo2", BACKGROUND_TYPE_MOVE_NEAR,    "bkBlue",  "Priest", "Bllg",   9, "Seal Chamber",          "\x95\x95\x88\xF3\x82\xCC\x8A\xD4"),	/* 封印の間 */
	STAGE_ENTRY("White",  "e_Blcn", BACKGROUND_TYPE_CLOUDS,       "BkFog",   "Miza",   "0",      9, "",                      ""),
	STAGE_ENTRY("Oside",  "Clock",  BACKGROUND_TYPE_CLOUDS_WINDY, "BkMoon",  "Moon",   "0",      0, "Clock Room",            "\x8E\x9E\x8C\x76\x89\xAE")	/* 時計屋 */
};

BOOL TransferStage(int no, int w, int x, int y)
{
	char path[MAX_PATH];
	char path_dir[20];
	BOOL bError;

	// Move character
	SetMyCharPosition(x * 0x10 * 0x200, y * 0x10 * 0x200);

	bError = FALSE;

	// Get path
	strcpy(path_dir, "Stage");

	// Load tileset
	sprintf(path, "%s\\Prt%s", path_dir, gTMT[no].parts);
	if (!ReloadBitmap_File(path, SURFACE_ID_LEVEL_TILESET))
		bError = TRUE;

	sprintf(path, "%s\\%s.pxa", path_dir, gTMT[no].parts);
	if (!LoadAttributeData(path))
		bError = TRUE;

	// Load tilemap
	sprintf(path, "%s\\%s.pxm", path_dir, gTMT[no].map);
	if (!LoadMapData2(path))
		bError = TRUE;

	// Load NPCs
	sprintf(path, "%s\\%s.pxe", path_dir, gTMT[no].map);
	if (!LoadEvent(path))
		bError = TRUE;

	// Load script
	sprintf(path, "%s\\%s.tsc", path_dir, gTMT[no].map);
	if (!LoadTextScript_Stage(path))
		bError = TRUE;

	// Load background
	sprintf(path, "%s", gTMT[no].back);
	if (!InitBack(path, gTMT[no].bkType))
		bError = TRUE;

	// Get path
	strcpy(path_dir, "Npc");

	// Load NPC sprite sheets
	sprintf(path, "%s\\Npc%s", path_dir, gTMT[no].npc);
	if (!ReloadBitmap_File(path, SURFACE_ID_LEVEL_SPRITESET_1))
		bError = TRUE;

	sprintf(path, "%s\\Npc%s", path_dir, gTMT[no].boss);
	if (!ReloadBitmap_File(path, SURFACE_ID_LEVEL_SPRITESET_2))
		bError = TRUE;

	if (bError)
		return FALSE;

	// Load map name
	ReadyMapName(gTMT[no].name);

	StartTextScript(w);
	SetFrameMyChar();
	ClearBullet();
	InitCaret();
	ClearValueView();
	ResetQuake();
	InitBossChar(gTMT[no].boss_no);
	ResetFlash();
	gStageNo = no;

	return TRUE;
}

// Music
const char* const gMusicTable[] = {
	"XXXX",
	"WANPAKU",
	"ANZEN",
	"GAMEOVER",
	"GRAVITY",
	"WEED",
	"MDOWN2",
	"FIREEYE",
	"VIVI",
	"MURA",
	"FANFALE1",
	"GINSUKE",
	"CEMETERY",
	"PLANT",
	"KODOU",
	"FANFALE3",
	"FANFALE2",
	"DR",
	"ESCAPE",
	"JENKA",
	"MAZE",
	"ACCESS",
	"IRONH",
	"GRAND",
	"Curly",	// Uses the original filename instead of the internal allcaps one
	"OSIDE",
	"REQUIEM",
	"WANPAK2",
	"QUIET",
	"LASTCAVE",
	"BALCONY",
	"LASTBTL",
	"LASTBT3",
	"ENDING",
	"ZONBIE",
	"BDOWN",
	"HELL",
	"JENKA2",
	"MARINE",
	"BALLOS",
	"TOROKO",
	"WHITE"
};

void ChangeMusic(MusicID no)
{
	if (no != MUS_SILENCE && no == gMusicNo)
		return;

	// Stop and keep track of old song
	gOldPos = GetOrganyaPosition();
	gOldNo = gMusicNo;
	StopOrganyaMusic();

	// Load .org
	LoadOrganya(gMusicTable[no]);

	// Reset position, volume, and then play the song
	ChangeOrganyaVolume(100);
	SetOrganyaPosition(0);
	PlayOrganyaMusic();
	gMusicNo = no;
}

void ReCallMusic(void)
{
	// Stop old song
	StopOrganyaMusic();

	// Load .org that was playing before
	LoadOrganya(gMusicTable[gOldNo]);

	// Reset position, volume, and then play the song
	SetOrganyaPosition(gOldPos);
	ChangeOrganyaVolume(100);
	PlayOrganyaMusic();
	gMusicNo = gOldNo;
}
