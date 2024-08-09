WINDRES ?= windres

ASSETS_DIRECTORY = assets

ALL_CXXFLAGS = $(CXXFLAGS)
ALL_LDFLAGS = $(LDFLAGS)
ALL_LIBS = $(LIBS)

ifeq ($(RELEASE), 1)
  ALL_CXXFLAGS += -O3 -DNDEBUG
  ALL_LDFLAGS += -s
  FILENAME_DEF = CSE2.exe
else
  ALL_CXXFLAGS += -Og -ggdb3
  FILENAME_DEF = CSE2_debug.exe
endif

ifeq ($(JAPANESE), 1)
  BUILD_DIRECTORY = game_japanese

  ALL_CXXFLAGS += -DJAPANESE
else
  BUILD_DIRECTORY = game_english
endif

FILENAME ?= $(FILENAME_DEF)

ifeq ($(FIX_BUGS), 1)
  ALL_CXXFLAGS += -DFIX_BUGS -DFIX_MAJOR_BUGS
else
  ifeq ($(FIX_MAJOR_BUGS), 1)
    ALL_CXXFLAGS += -DFIX_MAJOR_BUGS
  endif
endif

ifeq ($(DEBUG_SAVE), 1)
  ALL_CXXFLAGS += -DDEBUG_SAVE
endif

ALL_CXXFLAGS += -std=c++98 -MMD -MP -MF $@.d
ALL_LIBS += -lkernel32 -lgdi32 -lddraw -ldinput -ldsound -lversion -lshlwapi -limm32 -lwinmm -ldxguid
ALL_LDFLAGS += -mwindows

ifeq ($(STATIC), 1)
  ALL_LDFLAGS += -static
endif

SOURCES = \
  src/ArmsItem.cpp \
  src/Back.cpp \
  src/Boss.cpp \
  src/BossAlmo1.cpp \
  src/BossAlmo2.cpp \
  src/BossBallos.cpp \
  src/BossFrog.cpp \
  src/BossIronH.cpp \
  src/BossLife.cpp \
  src/BossOhm.cpp \
  src/BossPress.cpp \
  src/BossTwinD.cpp \
  src/BossX.cpp \
  src/BulHit.cpp \
  src/Bullet.cpp \
  src/Caret.cpp \
  src/Config.cpp \
  src/Dialog.cpp \
  src/Draw.cpp \
  src/Ending.cpp \
  src/Escape.cpp \
  src/Fade.cpp \
  src/Flags.cpp \
  src/Flash.cpp \
  src/Frame.cpp \
  src/Game.cpp \
  src/Generic.cpp \
  src/GenericLoad.cpp \
  src/Input.cpp \
  src/KeyControl.cpp \
  src/Main.cpp \
  src/Map.cpp \
  src/MapName.cpp \
  src/MiniMap.cpp \
  src/MyChar.cpp \
  src/MycHit.cpp \
  src/MycParam.cpp \
  src/NpcAct000.cpp \
  src/NpcAct020.cpp \
  src/NpcAct040.cpp \
  src/NpcAct060.cpp \
  src/NpcAct080.cpp \
  src/NpcAct100.cpp \
  src/NpcAct120.cpp \
  src/NpcAct140.cpp \
  src/NpcAct160.cpp \
  src/NpcAct180.cpp \
  src/NpcAct200.cpp \
  src/NpcAct220.cpp \
  src/NpcAct240.cpp \
  src/NpcAct260.cpp \
  src/NpcAct280.cpp \
  src/NpcAct300.cpp \
  src/NpcAct320.cpp \
  src/NpcAct340.cpp \
  src/NpChar.cpp \
  src/NpcHit.cpp \
  src/NpcTbl.cpp \
  src/Organya.cpp \
  src/PixTone.cpp \
  src/Profile.cpp \
  src/SelStage.cpp \
  src/Shoot.cpp \
  src/Sound.cpp \
  src/Stage.cpp \
  src/Star.cpp \
  src/TextScr.cpp \
  src/Triangle.cpp \
  src/ValueView.cpp

RESOURCES = \
  BITMAP/Credit01.bmp \
  BITMAP/Credit02.bmp \
  BITMAP/Credit03.bmp \
  BITMAP/Credit04.bmp \
  BITMAP/Credit05.bmp \
  BITMAP/Credit06.bmp \
  BITMAP/Credit07.bmp \
  BITMAP/Credit08.bmp \
  BITMAP/Credit09.bmp \
  BITMAP/Credit10.bmp \
  BITMAP/Credit11.bmp \
  BITMAP/Credit12.bmp \
  BITMAP/Credit14.bmp \
  BITMAP/Credit15.bmp \
  BITMAP/Credit16.bmp \
  BITMAP/Credit17.bmp \
  BITMAP/Credit18.bmp \
  BITMAP/pixel.bmp \
  CURSOR/CURSOR_IKA.cur \
  CURSOR/CURSOR_NORMAL.cur \
  ICON/0.ico \
  ICON/ICON_MINI.ico \
  ORG/Access.org \
  ORG/Anzen.org \
  ORG/Balcony.org \
  ORG/Ballos.org \
  ORG/BreakDown.org \
  ORG/Cemetery.org \
  ORG/Curly.org \
  ORG/Dr.org \
  ORG/Ending.org \
  ORG/Escape.org \
  ORG/Fanfale1.org \
  ORG/Fanfale2.org \
  ORG/Fanfale3.org \
  ORG/FireEye.org \
  ORG/Gameover.org \
  ORG/Ginsuke.org \
  ORG/Grand.org \
  ORG/Gravity.org \
  ORG/Hell.org \
  ORG/ironH.org \
  ORG/Jenka.org \
  ORG/Jenka2.org \
  ORG/Kodou.org \
  ORG/LastBtl3.org \
  ORG/LastBtl.org \
  ORG/LastCave.org \
  ORG/Marine.org \
  ORG/Maze.org \
  ORG/MDown2.org \
  ORG/Mura.org \
  ORG/Oside.org \
  ORG/Plant.org \
  ORG/quiet.org \
  ORG/Requiem.org \
  ORG/Toroko.org \
  ORG/Vivi.org \
  ORG/Wanpak2.org \
  ORG/Wanpaku.org \
  ORG/Weed.org \
  ORG/White.org \
  ORG/XXXX.org \
  ORG/Zonbie.org \
  WAVE/Wave.dat

OBJECTS = $(addprefix obj/$(FILENAME)/, $(addsuffix .o, $(SOURCES)))
DEPENDENCIES = $(addprefix obj/$(FILENAME)/, $(addsuffix .o.d, $(SOURCES)))

OBJECTS += obj/$(FILENAME)/windows_resources.o

all: $(BUILD_DIRECTORY)/$(FILENAME)
	$(info Finished)

$(BUILD_DIRECTORY)/$(FILENAME): $(OBJECTS)
	@mkdir -p $(@D)
	$(info Linking $@)
	@$(CXX) $(ALL_CXXFLAGS) $(ALL_LDFLAGS) $^ -o $@ $(ALL_LIBS)

obj/$(FILENAME)/%.cpp.o: %.cpp
	@mkdir -p $(@D)
	$(info Compiling $<)
	@$(CXX) $(ALL_CXXFLAGS) $< -o $@ -c

include $(wildcard $(DEPENDENCIES))

obj/$(FILENAME)/windows_resources.o: $(ASSETS_DIRECTORY)/resources/CSE2.rc $(ASSETS_DIRECTORY)/resources/resource1.h $(ASSETS_DIRECTORY)/resources/afxres.h $(addprefix $(ASSETS_DIRECTORY)/resources/, $(RESOURCES))
	@mkdir -p $(@D)
	$(info Compiling Windows resource file $<)
	@$(WINDRES) $< $@

# TODO
clean:
	@rm -rf obj
