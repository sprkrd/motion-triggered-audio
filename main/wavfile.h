#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PLAYLIST_CAPACITY 32

enum WavStatus { WAV_OK, WAV_READ_ERROR, WAV_WRONG_FORMAT, WAV_EOF};

struct WavFile
{
    int fd;
    uint32_t offset;
    int status;

    uint32_t nsamples;
    uint32_t sample_rate;
    uint16_t num_channels;
    uint16_t bits_per_sample;
};

typedef struct WavFile* WavFileHandle;

struct Playlist
{
    int size;
    char* sounds[PLAYLIST_CAPACITY];
};

typedef struct Playlist* PlaylistPtr;


static inline int min(int a, int b)
{
    return a<b? a : b;
}

static inline uint32_t read_u32(const uint8_t* src)
{
    return (uint32_t)src[0]         | ((uint32_t)src[1]<<8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3]<<24);
}

static inline uint16_t read_u16(const uint8_t* src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1]<<8);
}

static inline uint8_t read_scaled_s16_to_u8(const uint8_t* src)
{
    int32_t res = read_u16(src);
    res = (res - 2*(res&(1<<15)) + (1<<15) + 128)>>8;
    return (uint8_t)res;
}

WavFileHandle open_wavfile(const char* path);

int load_samples_pcm8(WavFileHandle handle, uint8_t* dst, int nsamples);

int load_samples_pcm16(WavFileHandle handle, uint8_t* dst, int nsamples);

int load_samples(WavFileHandle handle, uint8_t* dst, int nsamples);

int close_wavfile(WavFileHandle handle);

char* path_join(const char* basepath, const char* child);

const char* get_ext(const char* path);

bool is_wavfile(const char* path);

void build_playlist(const char* path, PlaylistPtr out);

void clear_playlist(PlaylistPtr playlist);
