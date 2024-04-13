# BinkadecWithWavHeader
Just a personal plugin for fmodel to play binka files
# Usage
Extract release zip files into [FModel Folder]/Output/.data/

Then binka files can be exported to wav files in Fmodel

Or you can use binkadec.exe --input xxx.binka --output xxx.wav to convert ue binka into wav files
# Build
copy the BinkAudio SDK from https://github.com/EpicGames/UnrealEngine/blob/5.0/Engine/Source/Runtime/BinkAudioDecoder/SDK/BinkAudio to SDK/BinkAudio

delete some conflicts tickets codes in sdk

then cmake
