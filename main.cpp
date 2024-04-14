#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>

#include <cassert>

#include <fstream>

#include <argparse/argparse.hpp>
#include <memory>
#include <string>
#include <vector>

#include "binka_ue_decode.h"
#include "binka_ue_file_header.h"

using int16 = int16_t;
using int32 = int32_t;

constexpr int MAX_BINK_AUDIO_CHANNELS = 16;

inline uint32_t Align32(size_t size) { return (uint32_t)((size + 31) & ~31); }

inline void *PTR_ADD(void *ptr, uint32_t off) {
    return (void *) (((uint8_t * )(ptr)) + (off));
}

struct BinkAudioDecoder {
    // # of low level bink audio streams (max 2 chans each)
    uint8 StreamCount;
    // # of current bytes in the res
    uint32 OutputReservoirValidBytes;
    // # max size of the res.
    uint32 OutputReservoirTotalBytes;
    // offsets to the various pieces of the decoder
    uint16 ToDeinterlaceBufferOffset32;
    uint16 ToSeekTableOffset32;
    uint16 ToOutputReservoirOffset32;
    // # of entries in the seek table - the entries are byte sizes, so
    // when decoding the seek table we convert to file offsets. This means
    // we actually have +1 entries in the decoded seek table due to the standard
    // "span -> edge" count thing.
    uint16 SeekTableCount;
    // # of low level bink blocks one seek table entry spans
    uint16 FramesPerSeekTableEntry;
    // # of frames we need to eat before outputting data due to a sample
    // accurate seek.
    uint16 ConsumeFrameCount;

    uint8 *StreamChannels() { return (uint8 * )(this + 1); }

    void **Decoders() {
        return (void **) PTR_ADD(
                this, Align32(sizeof(uint8) * StreamCount + sizeof(BinkAudioDecoder)));
    }

    uint8 *OutputReservoir() {
        return (uint8 * )(PTR_ADD(this, ToOutputReservoirOffset32 * 32U));
    }

    int16 *DeinterlaceBuffer() {
        return (int16 *) (PTR_ADD(this, ToDeinterlaceBufferOffset32 * 32U));
    }

    uint32 *
    SeekTable() // [SeekTableCount + 1] if SeekTableCount != 0, otherwise invalid.
    {
        return (uint32 * )(PTR_ADD(this, ToSeekTableOffset32 * 32U));
    }
};

struct audioinfo {
    int SampleRate;
    int TrueSampleCount;
    int SampleDataSize;
    int NumChannels;
    float Duration;
    int AudioDataOffset;

public:
    friend std::string to_string(const audioinfo &info) {
        std::string str = "";
        str += "SampleRate: " + std::to_string(info.SampleRate) + "\n";
        str += "TrueSampleCount: " + std::to_string(info.TrueSampleCount) + "\n";
        str += "SampleDataSize: " + std::to_string(info.SampleDataSize) + "\n";
        str += "NumChannels: " + std::to_string(info.NumChannels) + "\n";
        str += "Duration: " + std::to_string(info.Duration) + "\n";
        str += "AudioDataOffset: " + std::to_string(info.AudioDataOffset) + "\n";
        return str;
    }
};

bool ParseHeader(const uint8 *InSrcBufferData, uint32 InSrcBufferDataSize,
                 struct audioinfo *info) {
    auto SrcBufferData = InSrcBufferData;
    auto SrcBufferDataSize = InSrcBufferDataSize;
    auto SrcBufferOffset = 0;
    auto CurrentSampleCount = 0;

    assert(InSrcBufferDataSize >= sizeof(BinkAudioFileHeader));
    if (InSrcBufferDataSize < sizeof(BinkAudioFileHeader)) {
        return false;
    }

    BinkAudioFileHeader *Header = (BinkAudioFileHeader *) InSrcBufferData;
    if (Header->tag != 'UEBA') {
        return false;
    }
    if (Header->version != 1) {
        return false;
    }

    auto SampleRate = Header->rate;
    auto TrueSampleCount = Header->sample_count;
    auto NumChannels = Header->channels;
    auto MaxCompSpaceNeeded = Header->max_comp_space_needed;

    uint32 SeekTableSize = Header->seek_table_entry_count * sizeof(uint16);
    if (sizeof(BinkAudioFileHeader) + SeekTableSize > InSrcBufferDataSize) {
        return false;
    }

    // Store the offset to where the audio data begins
    auto AudioDataOffset = sizeof(BinkAudioFileHeader) + SeekTableSize;

    // Write out the the header info
    if (info) {
        info->SampleRate = Header->rate;
        info->NumChannels = Header->channels;
        info->SampleDataSize =
                Header->sample_count * info->NumChannels * sizeof(int16);
        info->Duration = (float) Header->sample_count / info->SampleRate;
        info->AudioDataOffset = AudioDataOffset;
        info->TrueSampleCount = TrueSampleCount;
    }

    return true;
}

bool CreateDecoder(const uint8_t *SrcBufferData, uint32_t SrcBufferDataSize,
                   uint32_t NumChannels, uint32_t SampleRate,
                   uint8_t *&RawMemory, BinkAudioDecoder *&Decoder,
                   uint32_t &SrcBufferOffset) {

    auto GetMaxFrameSizeSamples = [SampleRate]() -> uint32_t {
        if (SampleRate >= 44100) {
            return 1920;
        } else if (SampleRate >= 22050) {
            return 960;
        } else {
            return 480;
        }
    };
    UEBinkAudioDecodeInterface *BinkInterface = nullptr;

    BinkInterface = UnrealBinkAudioDecodeInterface();

    if (BinkInterface == nullptr) {
        return false; // only happens if we dont have libs.
    }

    BinkAudioFileHeader *Header = (BinkAudioFileHeader *) SrcBufferData;

    // Bink is max stereo per stream
    uint32 StreamCount = (NumChannels + 1) >> 1;

    // Figure memory for buffers:

    // Deinterlace - buffer space for interleaving multi stream binks in to a
    // standard interleaved format.
    uint32 DeinterlaceMemory = 0;
    if (StreamCount > 1) {
        DeinterlaceMemory = GetMaxFrameSizeSamples() * sizeof(int16) * 2;
    }

    // Space to store a decoded block.
    uint32 OutputReservoirMemory =
            NumChannels * GetMaxFrameSizeSamples() * sizeof(int16);

    // Space for the decoder state
    uint32 DecoderMemoryTotal = 0;
    uint32 DecoderMemoryPerStream[MAX_BINK_AUDIO_CHANNELS / 2];
    {
        uint32 RemnChannels = NumChannels;
        for (uint32 StreamIndex = 0; StreamIndex < StreamCount; StreamIndex++) {
            uint32 StreamChannels = RemnChannels;
            if (StreamChannels > 2) {
                StreamChannels = 2;
            }
            RemnChannels -= StreamChannels;
            DecoderMemoryPerStream[StreamIndex] =
                    BinkInterface->MemoryFn(SampleRate, StreamChannels);
            DecoderMemoryTotal += DecoderMemoryPerStream[StreamIndex];
        }
    }

    // Space for the decoder pointers
    uint32 PtrMemory = Align32(sizeof(void *) * StreamCount);

    // Space for ourselves + the channel count for each stream.
    uint32 StructMemory =
            Align32(sizeof(BinkAudioDecoder) + sizeof(uint8) * StreamCount);

    // Space for decoded seek table
    uint32 SeekTableMemory = 0;
    if (Header->seek_table_entry_count) {
        SeekTableMemory =
                Align32(sizeof(uint32) * (Header->seek_table_entry_count + 1));
    }

    uint32 TotalMemory = DecoderMemoryTotal + PtrMemory + StructMemory +
                         OutputReservoirMemory + DeinterlaceMemory +
                         SeekTableMemory;

    //
    // Allocate and save offsets
    //
    RawMemory = (uint8 *) malloc(TotalMemory);
    memset(RawMemory, 0, TotalMemory);

    Decoder = (BinkAudioDecoder *) RawMemory;

    Decoder->StreamCount = StreamCount;
    Decoder->OutputReservoirTotalBytes = OutputReservoirMemory;
    Decoder->SeekTableCount = Header->seek_table_entry_count;
    Decoder->FramesPerSeekTableEntry = Header->blocks_per_seek_table_entry;

    // See layout discussion in class declaration
    void **Decoders = Decoder->Decoders();
    uint8 *CurrentMemory = (uint8 *) PTR_ADD(Decoders, PtrMemory);
    uint8 *Channels = Decoder->StreamChannels();

    // Init decoders
    {
        uint8 RemnChannels = NumChannels;
        for (uint32 StreamIndex = 0; StreamIndex < StreamCount; StreamIndex++) {
            uint32 StreamChannels = RemnChannels;
            if (StreamChannels > 2) {
                StreamChannels = 2;
            }
            RemnChannels -= StreamChannels;

            Channels[StreamIndex] = StreamChannels;
            Decoders[StreamIndex] = (void **) CurrentMemory;
            CurrentMemory += DecoderMemoryPerStream[StreamIndex];
            BinkInterface->OpenFn(Decoders[StreamIndex], SampleRate, StreamChannels,
                                  true, true);
        }
    }

    Decoder->ToOutputReservoirOffset32 =
            (uint16)((CurrentMemory - RawMemory) / 32);
    CurrentMemory += OutputReservoirMemory;

    Decoder->ToDeinterlaceBufferOffset32 =
            (uint16)((CurrentMemory - RawMemory) / 32);
    CurrentMemory += DeinterlaceMemory;

    Decoder->ToSeekTableOffset32 = (uint16)((CurrentMemory - RawMemory) / 32);
    CurrentMemory += SeekTableMemory;

    // Decode the seek table
    if (Decoder->SeekTableCount) {
        uint32 *SeekTable = Decoder->SeekTable();
        uint16 *EncodedSeekTable =
                (uint16 * )(SrcBufferData + sizeof(BinkAudioFileHeader));
        uint32 CurrentSeekOffset = 0;

        // the seek table has deltas from last, and we want absolutes
        for (uint32 i = 0; i < Decoder->SeekTableCount; i++) {
            SeekTable[i] = CurrentSeekOffset;
            CurrentSeekOffset += EncodedSeekTable[i];
        }
        SeekTable[Decoder->SeekTableCount] = CurrentSeekOffset;
    }

    SrcBufferOffset = sizeof(BinkAudioFileHeader) +
                      Header->seek_table_entry_count * sizeof(uint16);
    return true;
}

int Decode(BinkAudioDecoder *&Decoder, const uint8 *CompressedData,
           const int32 CompressedDataSize, uint32_t NumChannels,
           uint32_t SampleRate, const uint16_t MaxCompSpaceNeeded,
           uint8 *OutPCMData, const int32 OutputPCMDataSize) {
    auto GetMaxFrameSizeSamples = [SampleRate]() -> uint32_t {
        if (SampleRate >= 44100) {
            return 1920;
        } else if (SampleRate >= 22050) {
            return 960;
        } else {
            return 480;
        }
    };
    UEBinkAudioDecodeInterface *BinkInterface = 0;

    BinkInterface = UnrealBinkAudioDecodeInterface();

    if (BinkInterface == 0) {
        return 0; // only happens if we dont have libs.
    }

    uint32 RemnOutputPCMDataSize = OutputPCMDataSize;
    uint32 RemnCompressedDataSize = CompressedDataSize;
    const uint8 *CompressedDataEnd = CompressedData + CompressedDataSize;

    //
    // In the event we need to copy to a stack buffer, we alloca() it here so
    // that it's not inside the loop (for static analysis). We don't touch the
    // memory until we need it so it's just a couple instructions for the
    // alloca().
    // (+8 for max block header size)
    uint8 *StackBlockBuffer =
            (uint8 *) alloca(MaxCompSpaceNeeded + BINK_UE_DECODER_END_INPUT_SPACE + 8);

    const uint32 DecodeSize =
            GetMaxFrameSizeSamples() * sizeof(int16) * NumChannels;
    while (RemnOutputPCMDataSize) {
        //
        // Drain the output reservoir before attempting a decode.
        //
        if (Decoder->OutputReservoirValidBytes) {
            uint32 CopyBytes = Decoder->OutputReservoirValidBytes;
            if (CopyBytes > RemnOutputPCMDataSize) {
                CopyBytes = RemnOutputPCMDataSize;
            }

            memcpy(OutPCMData, Decoder->OutputReservoir(), CopyBytes);

            // We use memmove here because we expect it to be very rare that we
            // don't consume the entire output reservoir in a call, so it's not
            // worth managing a cursor. Just move down the remnants, which we expect
            // to be zero.
            if (Decoder->OutputReservoirValidBytes != CopyBytes) {
                std::memmove(Decoder->OutputReservoir(),
                             Decoder->OutputReservoir() + CopyBytes,
                             Decoder->OutputReservoirValidBytes - CopyBytes);
            }
            Decoder->OutputReservoirValidBytes -= CopyBytes;

            RemnOutputPCMDataSize -= CopyBytes;
            OutPCMData += CopyBytes;

            if (RemnOutputPCMDataSize == 0) {
                // we filled entirely from the output reservoir
                break;
            }
        }

        if (RemnCompressedDataSize == 0) {
            // This is the normal termination condition
            break;
        }


        if (BinkAudioValidateBlock(MaxCompSpaceNeeded, CompressedData,
                                   RemnCompressedDataSize) !=
            BINK_AUDIO_BLOCK_VALID) {
            // The splitting system should ensure that we only ever get complete
            // blocks - so this is bizarre.
            // UE_LOG(LogBinkAudioDecoder, Warning,
            //        TEXT("Got weird buffer, validate returned %d"),
            //        BinkAudioValidateBlock(MaxCompSpaceNeeded, CompressedData,
            //                               RemnCompressedDataSize));

            std::cout << "Got weird buffer, validate returned "
                      << std::to_string(BinkAudioValidateBlock(
                              MaxCompSpaceNeeded, CompressedData,
                              RemnCompressedDataSize))
                      << std::endl;
            break;
        }

        uint8 const *BlockStart = 0;
        uint8 const *BlockEnd = 0;
        uint32 TrimToSampleCount = 0;
        BinkAudioCrackBlock(CompressedData, &BlockStart, &BlockEnd,
                            &TrimToSampleCount);
        uint8 const *BlockBase = CompressedData;

        //
        // We need to make sure there's room available for Bink to read past the
        // end of the buffer (for vector decoding). If there's not, we need to
        // copy to a temp buffer.
        //
        bool HasRoomForDecode =
                (CompressedDataEnd - BlockEnd) > BINK_UE_DECODER_END_INPUT_SPACE;
        if (HasRoomForDecode == false) {
            // This looks weird, but in order for the advancement logic to work,
            // we need to replicate the entire block including the header.
            size_t BlockOffset = BlockStart - BlockBase;
            size_t BlockSize = BlockEnd - BlockBase;
            if (BlockSize > MaxCompSpaceNeeded + 8) // +8 for max block header size
            {
                // UE_LOG(LogBinkAudioDecoder, Error,
                //        TEXT("BAD! Validated block exceeds header max block size (%d
                //        "
                //             "vs %d)"),
                //        BlockSize, MaxCompSpaceNeeded);
                std::cout << "BAD! Validated block exceeds header max block size ("
                          << BlockSize << " vs " << MaxCompSpaceNeeded << ")"
                          << std::endl;
                // bErrorStateLatch = true;
                break;
            }

            memcpy(StackBlockBuffer, BlockBase, BlockSize);

            // this is technically not needed, but just so that any analysis shows
            // that we've initialized all the memory we touch.
            memset(StackBlockBuffer + BlockSize, 0, BINK_UE_DECODER_END_INPUT_SPACE);

            BlockBase = StackBlockBuffer;
            BlockStart = StackBlockBuffer + BlockOffset;
            BlockEnd = StackBlockBuffer + BlockSize;
        }

        //
        // If we're a simple single stream and we have enough output space,
        // just decode directly in to our destination to avoid some
        // copies.
        //
        // We also have to have a "simple" decode - i.e. aligned and no trimming.
        //
        if (Decoder->StreamCount == 1 && DecodeSize <= RemnOutputPCMDataSize &&
            TrimToSampleCount == ~0U && (((size_t) OutPCMData) & 0xf) == 0 &&
            Decoder->ConsumeFrameCount == 0) {
            uint32 DecodedBytes =
                    BinkInterface->DecodeFn(Decoder->Decoders()[0], OutPCMData,
                                            RemnOutputPCMDataSize, &BlockStart, BlockEnd);

            if (DecodedBytes == 0) {

                // This means that our block check above succeeded and we still failed
                // - corrupted data!
                return 0;
            }

            uint32 InputConsumed = (uint32)(BlockStart - BlockBase);

            OutPCMData += DecodedBytes;
            CompressedData += InputConsumed;
            RemnCompressedDataSize -= InputConsumed;
            RemnOutputPCMDataSize -= DecodedBytes;
            continue;
        }

        // Otherwise we route through the output reservoir.
        if (Decoder->StreamCount == 1) {
            uint32 DecodedBytes = BinkInterface->DecodeFn(
                    Decoder->Decoders()[0], Decoder->OutputReservoir(),
                    Decoder->OutputReservoirTotalBytes, &BlockStart, BlockEnd);
            if (DecodedBytes == 0) {

                // This means that our block check above succeeded and we still failed
                // - corrupted data!
                return 0;
            }

            uint32 InputConsumed = (uint32)(BlockStart - BlockBase);

            CompressedData += InputConsumed;
            RemnCompressedDataSize -= InputConsumed;

            Decoder->OutputReservoirValidBytes = DecodedBytes;
        } else {
            // multistream requires interlacing the stereo/mono streams
            int16 *DeinterlaceBuffer = Decoder->DeinterlaceBuffer();
            uint8 *CurrentOutputReservoir = Decoder->OutputReservoir();

            uint32 LocalNumChannels = NumChannels;
            for (uint32 i = 0; i < Decoder->StreamCount; i++) {
                uint32 StreamChannels = Decoder->StreamChannels()[i];

                uint32 DecodedBytes = BinkInterface->DecodeFn(
                        Decoder->Decoders()[i], (uint8 *) DeinterlaceBuffer,
                        GetMaxFrameSizeSamples() * sizeof(int16) * 2, &BlockStart,
                        BlockEnd);

                if (DecodedBytes == 0) {
                    // This means that our block check above succeeded and we still
                    // failed - corrupted data!
                    return 0;
                }

                // deinterleave in to the output reservoir.
                if (StreamChannels == 1) {
                    int16 *Read = DeinterlaceBuffer;
                    int16 *Write = (int16 *) CurrentOutputReservoir;
                    uint32 Frames = DecodedBytes / sizeof(int16);
                    for (uint32 FrameIndex = 0; FrameIndex < Frames; FrameIndex++) {
                        Write[LocalNumChannels * FrameIndex] = Read[FrameIndex];
                    }
                } else {
                    // stereo int16 pairs
                    int32 *Read = (int32 *) DeinterlaceBuffer;
                    int16 *Write = (int16 *) CurrentOutputReservoir;
                    uint32 Frames = DecodedBytes / sizeof(int32);
                    for (uint32 FrameIndex = 0; FrameIndex < Frames; FrameIndex++) {
                        *(int32 *) (Write + LocalNumChannels * FrameIndex) =
                                Read[FrameIndex];
                    }
                }

                CurrentOutputReservoir += sizeof(int16) * StreamChannels;

                Decoder->OutputReservoirValidBytes += DecodedBytes;
            }

            uint32 InputConsumed = (uint32)(BlockStart - BlockBase);
            CompressedData += InputConsumed;
            RemnCompressedDataSize -= InputConsumed;
        } // end if multi stream

        // Check if we are trimming the tail due to EOF
        if (TrimToSampleCount != ~0U) {
            // Ignore the tail samples by just dropping our reservoir size
            Decoder->OutputReservoirValidBytes =
                    TrimToSampleCount * sizeof(int16) * NumChannels;
        }

        // Check if we need to eat some frames due to a sample-accurate seek.
        if (Decoder->ConsumeFrameCount) {
            const uint32 BytesPerFrame = sizeof(int16) * NumChannels;
            uint32 ValidFrames = Decoder->OutputReservoirValidBytes / BytesPerFrame;
            if (Decoder->ConsumeFrameCount < ValidFrames) {
                memmove(Decoder->OutputReservoir(),
                        Decoder->OutputReservoir() +
                        Decoder->ConsumeFrameCount * BytesPerFrame,
                        (ValidFrames - Decoder->ConsumeFrameCount) * BytesPerFrame);
                Decoder->OutputReservoirValidBytes -=
                        Decoder->ConsumeFrameCount * BytesPerFrame;
            } else {
                Decoder->OutputReservoirValidBytes = 0;
            }
            Decoder->ConsumeFrameCount = 0;
        }
        // Fall through to the next loop to copy the decoded pcm data out of the
        // reservoir.
    } // while need output pcm data

    // We get here if we filled the output buffer or not.
    // FDecodeResult Result;
    // Result.NumPcmBytesProduced = OutputPCMDataSize - RemnOutputPCMDataSize;
    // Result.NumAudioFramesProduced =
    //     Result.NumPcmBytesProduced / (sizeof(int16) * NumChannels);
    // Result.NumCompressedBytesConsumed =
    //     CompressedDataSize - RemnCompressedDataSize;
    std::cout << "NumPcmBytesProduced: "
              << OutputPCMDataSize - RemnOutputPCMDataSize << std::endl;
    // std::cout << "NumAudioFramesProduced: " << Result.NumAudioFramesProduced
    // << std::endl; std::cout << "NumCompressedBytesConsumed: " <<
    // Result.NumCompressedBytesConsumed << std::endl;

    return OutputPCMDataSize - RemnOutputPCMDataSize;
}

struct wavfile_header {
    char riff_tag[4];
    int riff_length;
    char wave_tag[4];
    char fmt_tag[4];
    int fmt_length;
    short audio_format;
    short num_channels;
    int sample_rate;
    int byte_rate;
    short block_align;
    short bits_per_sample;
    char data_tag[4];
    int data_length;
};

void wavfile_init(struct wavfile_header &header) {
    int bits_per_sample = 16;

    strncpy(header.riff_tag, "RIFF", 4);
    strncpy(header.wave_tag, "WAVE", 4);
    strncpy(header.fmt_tag, "fmt ", 4);
    strncpy(header.data_tag, "data", 4);

    header.fmt_length = 16;
    header.audio_format = 1;
    header.byte_rate = header.num_channels * header.sample_rate * bits_per_sample / 8;
    header.block_align = header.num_channels * bits_per_sample / 8;
    header.bits_per_sample = bits_per_sample;
}

std::string description =
        R"(binkadec: experimental Unreal Engine 4/5 "UEBA/.binka" audio decoder

Tested with:
  - THE FINALS (UE5.0))";

int main(int argc, char **argv) {
    // binkadec -i input.bink -o output.s16 -f raw
    argparse::ArgumentParser program("binkadec");
    program.add_description(description);
    program.add_argument("-i", "--input").required().help("input file");
    program.add_argument("-o", "--output").required().help("output file");
    program.add_argument("-v", "--verbose")
            .default_value(false)
            .implicit_value(true)
            .help("verbose output");
    //format: raw/wav
    program.add_argument("-f", "--format")
            .default_value("raw")
            .help("output format(raw)");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }
    std::string input = program.get<std::string>("--input");
    std::filesystem::path input_path(input);
    std::string output = program.get<std::string>("--output");
    std::filesystem::path output_path(output);
    bool verbose = program.get<bool>("--verbose");
    if (verbose) {
        std::cout << "Input: " << input_path << std::endl;
        std::cout << "Output: " << output_path << std::endl;
    }
    std::ifstream input_file(input_path, std::ios::binary);
    if (!input_file.is_open()) {
        std::cout << "Failed to open input file" << std::endl;
        exit(0);
    }
    std::shared_ptr <uint8_t> input_data;
    uint32_t input_data_size;
    {
        input_file.seekg(0, std::ios::end);
        input_data_size = input_file.tellg();
        input_file.seekg(0, std::ios::beg);
        input_data = std::shared_ptr<uint8_t>(new uint8_t[input_data_size],
                                              std::default_delete<uint8_t[]>());
        input_file.read((char *) input_data.get(), input_data_size);
    }

    struct audioinfo info;
    if (!ParseHeader(input_data.get(), input_data_size, &info)) {
        std::cout << "Failed to parse header" << std::endl;
        exit(0);
    }
    if (verbose) {
        std::cout << to_string(info) << std::endl;
    } else {
        std::cout << "SampleRate: " << info.SampleRate << std::endl;
        std::cout << "NumChannels: " << info.NumChannels << std::endl;
        std::cout << "Duration: " << info.Duration << std::endl;
    }

    uint8_t *RawMemory = nullptr;
    BinkAudioDecoder *Decoder = nullptr;
    uint32_t SrcBufferOffset = 0;
    if (!CreateDecoder(input_data.get(), input_data_size, info.NumChannels,
                       info.SampleRate, RawMemory, Decoder, SrcBufferOffset)) {
        std::cout << "Failed to create decoder" << std::endl;
        exit(0);
    }

    if (verbose) {
        std::cout << "RawMemory: " << (void *) RawMemory << std::endl;
        std::cout << "Decoder: " << (void *) Decoder << std::endl;
        std::cout << "SrcBufferOffset: " << SrcBufferOffset << std::endl;
        std::cout << "Decoder->StreamCount: " << Decoder->StreamCount << std::endl;
        std::cout << "Decoder->OutputReservoirValidBytes: "
                  << Decoder->OutputReservoirValidBytes << std::endl;
        std::cout << "Decoder->OutputReservoirTotalBytes: "
                  << Decoder->OutputReservoirTotalBytes << std::endl;
        std::cout << "Decoder->ToDeinterlaceBufferOffset32: "
                  << Decoder->ToDeinterlaceBufferOffset32 << std::endl;
        std::cout << "Decoder->ToSeekTableOffset32: "
                  << Decoder->ToSeekTableOffset32 << std::endl;
        std::cout << "Decoder->ToOutputReservoirOffset32: "
                  << Decoder->ToOutputReservoirOffset32 << std::endl;
        std::cout << "Decoder->SeekTableCount: " << Decoder->SeekTableCount
                  << std::endl;
        std::cout << "Decoder->FramesPerSeekTableEntry: "
                  << Decoder->FramesPerSeekTableEntry << std::endl;
        std::cout << "Decoder->ConsumeFrameCount: " << Decoder->ConsumeFrameCount
                  << std::endl;
    }

    //malloc for compressed data -- new buffer
    std::unique_ptr <uint8_t> compressedData =
            std::unique_ptr<uint8_t>(new uint8_t[input_data_size - SrcBufferOffset]);
    //Copy compressed data, but skip anything from "SEEK" to 0x9999.
    //Does not know why it is needed because Decoder->SeekTableCount is 0.
    bool in_skip = false;
    size_t dst_offset = 0;

    for (size_t i = SrcBufferOffset; i < input_data_size; i++) {
        if (!in_skip && i + 4 < input_data_size &&
            std::memcmp(&input_data.get()[i], "SEEK", 4) == 0) {
            in_skip = true;
            i += 3; // Skip "SEEK"
        } else if (in_skip && i + 2 < input_data_size && input_data.get()[i] == 0x99 &&
                   input_data.get()[i + 1] == 0x99) {
            in_skip = false;
            compressedData.get()[dst_offset++] = 0x99;
            compressedData.get()[dst_offset++] = 0x99;
            i += 1; // Skip 0x9999
        } else if (!in_skip) {
            compressedData.get()[dst_offset++] = input_data.get()[i];
        }
    }

    if (verbose) {
        std::cout << "compressedDataSize: " << dst_offset << std::endl;
    }


    uint32_t NumChannels = info.NumChannels;
    uint32_t SampleRate = info.SampleRate;
    uint16_t MaxCompSpaceNeeded = info.SampleDataSize;
    uint8_t *OutPCMData = (uint8_t *) malloc(info.SampleDataSize);
    int32_t OutputPCMDataSize = info.SampleDataSize;
    int32_t DecodedBytes =
            Decode(Decoder, compressedData.get(), dst_offset, NumChannels, SampleRate,
                   MaxCompSpaceNeeded, OutPCMData, OutputPCMDataSize);
    if (DecodedBytes == 0) {
        std::cout << "Failed to decode" << std::endl;
        exit(0);
    }
    std::ofstream output_file(output_path, std::ios::binary);
    if (!output_file.is_open()) {
        std::cout << "Failed to open output file" << std::endl;
        exit(0);
    }
    struct wavfile_header header = {0};
    header.sample_rate = info.SampleRate;
    header.num_channels = info.NumChannels;
    header.riff_length = DecodedBytes + sizeof(wavfile_header) - 8;
    header.data_length = DecodedBytes;
    wavfile_init(header);
    output_file.write((char *) &header, sizeof(header));
    output_file.write((char *) OutPCMData, DecodedBytes);
    output_file.close();
    input_file.close();
    std::cout << "Done. Play the result with:" << std::endl;
    std::cout << "ffplay -f s16le -ar " << SampleRate << " -ac "
              << NumChannels << " -i " << output_path << std::endl;
}
