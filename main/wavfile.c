#include "wavfile.h"

#include <ctype.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define TMP_BUF_SIZE 2048

WavFileHandle open_wavfile(const char* path)
{
    unsigned char buf[44];
    int read_bytes;
    WavFileHandle ret = (WavFileHandle)malloc(sizeof(struct WavFile));

    ret->fd = open(path, 0, O_RDONLY);

    if (ret->fd < 0)
    {
        free(ret);
        return NULL;
    }
    read_bytes = read(ret->fd, buf, 44);

    if (read_bytes < 44)
    {
        free(ret);
        return NULL;
    }

    if (strncmp((char*)&buf[0], "RIFF", 4))
    {
        free(ret);
        return NULL;
    }

    if (strncmp((char*)&buf[8], "WAVE", 4))
    {
        free(ret);
        return NULL;
    }

    // in the following, assume little endian. Revisit this assumption
    // and write endian agnostic code if it turns out to be false.

    ret->offset = 0;
    ret->num_channels = read_u16(&buf[22]);
    //memcpy(&(ret->sample_rate), &(buf[24]), 4);
    ret->sample_rate = read_u32(&(buf[24]));
    ret->bits_per_sample = read_u16(&buf[34]);
    ret->nsamples = read_u32(&buf[40])*8 / ret->bits_per_sample;
    ret->status = WAV_OK;

    return ret;
}

int load_samples_pcm8(WavFileHandle handle, uint8_t* dst, int nsamples)
{
    return read(handle->fd, dst, nsamples);
}

int load_samples_pcm16(WavFileHandle handle, uint8_t* dst, int nsamples)
{
    unsigned char buf[TMP_BUF_SIZE];
    int loaded_samples = 0;
    while (loaded_samples < nsamples)
    {
        int max_read_b = min(TMP_BUF_SIZE, (nsamples-loaded_samples)*2);
        int loaded_bytes_now = read(handle->fd, buf, max_read_b);
        int loaded_samples_now = loaded_bytes_now>>1;
        for (int i = 0; i < loaded_samples_now; ++i)
        {
            dst[loaded_samples+i] = read_scaled_s16_to_u8(&buf[2*i]);
        }

        loaded_samples += loaded_samples_now;
        if (loaded_bytes_now < max_read_b)
        {
            break;
        }
    }
    return loaded_samples;
}

int load_samples(WavFileHandle handle, uint8_t* dst, int nsamples)
{
    int ret = 0;

    if (handle->offset == handle->nsamples)
    {
        handle->status = WAV_EOF;
        return 0;
    }

    nsamples = min(nsamples, handle->nsamples - handle->offset);
    switch (handle->bits_per_sample)
    {
        case 8:
            ret = load_samples_pcm8(handle, dst, nsamples);
            break;
        case 16:
            ret = load_samples_pcm16(handle, dst, nsamples);
            break;
        default:
            handle->status = WAV_WRONG_FORMAT;
            return -1;
    }
    handle->offset += ret>0? ret : 0;
    if (ret<nsamples && handle->offset<handle->nsamples)
    {
        handle->status = WAV_READ_ERROR;
    }
    return ret;
}

int close_wavfile(WavFileHandle handle)
{
    int ret = close(handle->fd);
    free(handle);
    return ret;
}

char* path_join(const char* basepath, const char* child)
{
    size_t len_base = strlen(basepath);
    size_t len_child = strlen(child);
    bool append_separator = basepath[len_base-1] != '/';
    char* dst = malloc(len_base+len_child+append_separator+1);
    strcpy(dst, basepath);
    if (append_separator)
    {
        dst[len_base] = '/';
    }
    strcpy(&dst[len_base+append_separator], child);
    return dst;
}

const char* get_ext(const char* path)
{
    static char out[16];
    int i = strlen(path)-1;
    while (i>=0 && path[i]!='/')
    {
        if (path[i] == '.')
        {
            int j;
            for (j = 0; path[i+j+1]!='\0'; ++j)
            {
                out[j] = tolower(path[i+j+1]);
            }
            out[j] = '\0';
            return (const char*)out;
        }
        --i;
    }
    return NULL;
}

bool is_wavfile(const char* path)
{
    const char* ext = get_ext(path);
    struct stat entry_info;
    stat(path, &entry_info);
    return ext!=NULL && strcmp(ext, "wav")==0 && S_ISREG(entry_info.st_mode);
}

void build_playlist(const char* path, PlaylistPtr out)
{
    DIR* curdir = opendir(path);

    if (curdir == NULL)
    {
        out->size = -1;
        return;
    }

    out->size = 0;

    struct dirent* entry = readdir(curdir);
    while (entry!=NULL && out->size<PLAYLIST_CAPACITY)
    {
        char* full_path = path_join(path, entry->d_name);
        if (is_wavfile(full_path))
        {
            out->sounds[out->size] = full_path;
            ++out->size;
        }
        else
        {
            free(full_path);
        }
        entry = readdir(curdir);
    }
    closedir(curdir);
}

void clear_playlist(PlaylistPtr playlist)
{
    for (int i = 0; i < playlist->size; ++i)
    {
        free(playlist->sounds[i]);
    }
    playlist->size = 0;
}
