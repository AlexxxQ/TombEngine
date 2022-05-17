# TombEngine 

In the year 2000, Core Design gave us a great gift: level editor where people can create their own custom levels based on TR4 engine.
unfortunately it was quite limited, hence over the decades, the engine was upgraded massively via TREP and TRNG:
- TREP (Tomb Raider Engine Patcher) is a tool that allows you to modify the exe to expand certain limits and implement new features.
- TRNG (Tomb Raider Next Generation) builds upon TREP and gives a lot of new features including its own scripting language and even more extended limits and its own DLL.

Unfortunately, TRNG is a closed-source, and essentially an abandonware program. No one is able to fix well-known bugs in TRNG, and implement new features, 
although you can still implement some features with plugin (with C++ knowledge) however it is not user-friendly (you need to know C++ which is quite complex) and it is poorly documented.

TEN (TombEngine) is supposed to be a new engine which is (most importantly) open-source, removes limits, and fixes bugs from the original game,
it also provides support for Lua (a programming language), all objects from TR1-5, and many more exciting features and gameplay functionalities such as: corner shimmying or enlarged 2D map which lets you build a massive level (Imagine 5 split levels into 1 big level).

TEN (TombEngine) is based on TR5 engine which has been gradually built upon that.

# To compile TombEngine

In order to compile TombEngine, make sure you have:
- Visual Studio
- TombEditor (if you would like to create and test levels)

Steps:
1) Clone the repo to your Github Desktop
2) launch TombEngine.sln and compile
3) Once compiled, make a separate folder and copy everything from the Build folder (before compilation there will be nothing in that folder) and also copy the scripts folder to your TombEngine executable directory (make sure you have data, texture files aswell)
> Windows will also complain about missing dll files (Bass etc) in that case copy the dlls from the Libs folder

> Visual studio might also complain about the NuGet packages, to fix that:
> 1) Delete the packages folder
> 2) Go back to Visual studio
> 3) Right click on solution explorer and restore Nuget Packages
> 4) Compile again
4) Once that is done, you should now be able to go to TombEditor, make a simple level, compile and run it with TombEngine

# Disclaimer

I don't work for/have ever worked for Core Design, Eidos Interactive or Square Enix. This is a hobby project. I don't want this code to be sold since Tomb Raider is a registered trademark of Square Enix. The code is currently open source to allow more people to improve the engine and for study purposes. I'm not responsible for illegal use of this source code. This source code is released as-is and it's maintained by me and by other (not paid) contributors in our free time.

# Credits

**Not in order of importance**

**Sorry if you have been missed on the list, please let us know and we can add you to the credit list.**

## Developers
- MontyTRC (Project Leader)
- Lwmte (general coding, code cleanups, and bug fixing)
- Sezz (state refactoring, general coding and bug fixing)
- Troye (general coding, refactoring)
- ChocolateFan (general coding)
- Krys (general coding)
- Squidshire (Hispidence) (lua implementation and bug fixing)
- RicardoLuis0 (general coding)
- Raildex (renderer refactoring & general coding) 
- TokyoSU (Decompiling Vehicles & Entities)
- Gancian (general coding)
- WolfCheese (General coding)
- Moooonyeah (Jumanji) (entity decompilation)
- Stranger1992 (Sounds Project)

## Testers
- Stranger1992
- RemRem
- Kubsy
- Kamillos
- Caesum
- Lore
- WolfCheese
- Dustie
- Lgg_productions
- JoeyQuint
