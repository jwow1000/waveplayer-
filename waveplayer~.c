#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "m_pd.h"

#define BUFSIZE 1024 // to read from disk, in frames (1 frame = 1 sample per channel)
#define MAXCHANS 2
#define DECLICK_MS 5 // fade time for start/stop, to avoid clicks

static t_class *waveplayer_tilde_class;

typedef struct _waveplayer_tilde {
    t_object  x_obj;
    t_outlet *x_out[MAXCHANS];
    int x_channels;             // number of outlets, fixed at creation: 1 (mono) or 2 (stereo)
    int x_file_channels;        // channel count of the currently open file
    t_int x_loop_start;         // frame index, user-settable
    t_int x_loop_size;          // loop chunk length in frames; -1 means "entire file"
    t_int x_loop_end;           // frame index, derived from x_loop_start + chunk length
    t_int x_num_frames;         // total frames in the currently open file
    t_float x_speed;
    int x_running;                      // 1 between start/pause (or start/stop); position only advances while this or x_gain is nonzero
    int x_stop_pending;                 // 1 if "stop" was sent: rewind to loop_start once the fade-out reaches 0
    t_sample x_gain;                    // current output envelope, ramps between 0 and 1 to avoid clicks
    t_sample x_gain_target;
    t_sample x_gain_inc;                // per-sample ramp step, set from sample rate in the dsp method
    t_int x_loop_declick_frames;        // fade window (in file frames) used to duck the loop seam, set from sample rate in the dsp method
    double x_pos;                       // position in file, in frames
    t_int x_current_buf_num;            // buffer currently being played
    uint32_t x_data_offset;             // byte offset of the "data" chunk contents
    int16_t x_buf[MAXCHANS][BUFSIZE];   // from disk, de-interleaved per channel
    int16_t x_buf_last3[MAXCHANS][3];   // stash of last 3 frames of previous buffer, needed for 4 point interp
    FILE *x_fh;                         // the sound file

} t_waveplayer_tilde;

// walk RIFF chunks to find "fmt " and "data"; assumes little-endian host
static int parse_wav_header(FILE *fh, uint32_t *data_offset, uint32_t *data_size,
                             int *channels, int *bits_per_sample, uint32_t *sample_rate)
{
    char id[4];
    uint32_t chunk_size;
    int found_fmt = 0, found_data = 0;

    fseek(fh, 0, SEEK_SET);
    if (fread(id, 1, 4, fh) != 4 || memcmp(id, "RIFF", 4) != 0) return -1;
    fseek(fh, 4, SEEK_CUR); // skip RIFF chunk size
    if (fread(id, 1, 4, fh) != 4 || memcmp(id, "WAVE", 4) != 0) return -1;

    while (!(found_fmt && found_data)) {
        if (fread(id, 1, 4, fh) != 4) break;
        if (fread(&chunk_size, 4, 1, fh) != 1) break;

        if (memcmp(id, "fmt ", 4) == 0) {
            uint16_t num_channels, bps;
            long fmt_start = ftell(fh);
            fseek(fh, 2, SEEK_CUR); // skip audio format tag
            fread(&num_channels, 2, 1, fh);
            fread(sample_rate, 4, 1, fh);
            fseek(fh, 6, SEEK_CUR); // skip byte rate (4) + block align (2)
            fread(&bps, 2, 1, fh);
            *channels = num_channels;
            *bits_per_sample = bps;
            fseek(fh, fmt_start + chunk_size, SEEK_SET);
            found_fmt = 1;
        }
        else if (memcmp(id, "data", 4) == 0) {
            *data_offset = (uint32_t)ftell(fh);
            *data_size = chunk_size;
            fseek(fh, chunk_size, SEEK_CUR);
            found_data = 1;
        }
        else {
            fseek(fh, chunk_size, SEEK_CUR);
        }
        if (chunk_size % 2 != 0) fseek(fh, 1, SEEK_CUR); // chunks are word-aligned
    }

    return (found_fmt && found_data) ? 0 : -1;
}

// get buf from file
static void readbuf(t_waveplayer_tilde *x){

    int fc = x->x_file_channels;
    int ch;

    // stash last 3 frames (or first 3 for reverse play), per channel
    for (ch = 0; ch < MAXCHANS; ch++) {
        if (x->x_speed >= 0) {
            x->x_buf_last3[ch][0] = x->x_buf[ch][BUFSIZE - 3];
            x->x_buf_last3[ch][1] = x->x_buf[ch][BUFSIZE - 2];
            x->x_buf_last3[ch][2] = x->x_buf[ch][BUFSIZE - 1];
        }
        else {
            x->x_buf_last3[ch][0] = x->x_buf[ch][0];
            x->x_buf_last3[ch][1] = x->x_buf[ch][1];
            x->x_buf_last3[ch][2] = x->x_buf[ch][2];
        }
    }

    if (x->x_fh != NULL) {
        int16_t raw[BUFSIZE * MAXCHANS]; // interleaved frames as stored in the file
        long frame_pos = (long)x->x_current_buf_num * BUFSIZE;
        size_t want = (size_t)BUFSIZE * fc;
        size_t got;

        fseek(x->x_fh, x->x_data_offset + frame_pos * fc * 2, SEEK_SET);
        got = fread(raw, 2, want, x->x_fh);
        if (got < want) memset(raw + got, 0, (want - got) * 2); // pad past end of file

        int i;
        for (i = 0; i < BUFSIZE; i++) {
            x->x_buf[0][i] = raw[i * fc];
            x->x_buf[1][i] = (fc > 1) ? raw[i * fc + 1] : raw[i * fc];
        }
    }
    else {
        memset(x->x_buf, 0, sizeof(x->x_buf));
        memset(x->x_buf_last3, 0, sizeof(x->x_buf_last3));
    }

}

// recompute x_loop_end from x_loop_start/x_loop_size, clamped to the open file's length
static void update_loop_bounds(t_waveplayer_tilde *x)
{
    t_int start = x->x_loop_start;
    t_int size;

    if (start < 0) start = 0;
    if (x->x_num_frames > 0 && start > x->x_num_frames - 1) start = x->x_num_frames - 1;
    x->x_loop_start = start;

    size = (x->x_loop_size > 0) ? x->x_loop_size : (x->x_num_frames - start);
    if (start + size > x->x_num_frames) size = x->x_num_frames - start;
    if (size < 4) size = 4; // minimum window for the 4 point interpolation

    x->x_loop_end = start + size;
}

static void openfile(t_waveplayer_tilde *x, const char *fn){

    if (x->x_fh != NULL) fclose(x->x_fh);

    post("opening %s", fn);
    x->x_fh = fopen(fn,"rb");

    if (x->x_fh == NULL) {
        pd_error(x, "Unable to open file %s", fn);
        return;
    }

    uint32_t data_offset, data_size, sample_rate;
    int channels, bits_per_sample;

    if (parse_wav_header(x->x_fh, &data_offset, &data_size, &channels, &bits_per_sample, &sample_rate) != 0) {
        pd_error(x, "%s: not a valid WAV file", fn);
        fclose(x->x_fh);
        x->x_fh = NULL;
        return;
    }

    if (bits_per_sample != 16) {
        pd_error(x, "%s: only 16-bit PCM WAV files are supported (got %d-bit)", fn, bits_per_sample);
        fclose(x->x_fh);
        x->x_fh = NULL;
        return;
    }

    if (channels != 1 && channels != 2) {
        pd_error(x, "%s: only mono or stereo files are supported (got %d channels)", fn, channels);
        fclose(x->x_fh);
        x->x_fh = NULL;
        return;
    }

    if (channels != x->x_channels) {
        post("waveplayer~: note - file has %d channel(s), object has %d outlet(s)", channels, x->x_channels);
    }

    x->x_file_channels = channels;
    x->x_data_offset = data_offset;

    uint32_t num_frames = data_size / (channels * 2);
    post("loaded file: %d frames, %d channel(s), %d Hz", num_frames, channels, sample_rate);
    post("that is: %f secs", (float)num_frames / sample_rate);

    x->x_num_frames = num_frames;
    update_loop_bounds(x);          // 0-based frame index, header handled solely via x_data_offset
    x->x_pos = x->x_loop_start + 1;
    x->x_current_buf_num = -1;      // force read
}

// advance playback position by one sample, refilling the buffer if needed
static void waveplayer_advance(t_waveplayer_tilde *x, int *bufi_out, t_sample *frac_out)
{
    int bufnum;

    x->x_pos += x->x_speed;

    // check loop, include padding for interpolation window
    // go to loop end for reverse play
    if (x->x_speed >= 0) {
        if (x->x_pos >= (x->x_loop_end - 2)) x->x_pos = x->x_loop_start + 1;
    }
    else {
        if (x->x_pos <= (x->x_loop_start + 1)) x->x_pos = x->x_loop_end - 2;
    }

    // bufnum is 2 samples ahead pos playing forward, or 1 sample behind in reverse
    if (x->x_speed >= 0) bufnum = ((uint32_t)x->x_pos + 2) / BUFSIZE;
    else bufnum = ((uint32_t)x->x_pos - 1) / BUFSIZE;

    // check if we need new buf
    if (bufnum != x->x_current_buf_num) {
        x->x_current_buf_num = bufnum;
        readbuf(x);
    }

    *bufi_out = (uint32_t)x->x_pos % BUFSIZE;
    *frac_out = x->x_pos - (uint32_t)x->x_pos;
}

// 4 point polynomial interpolation from tabread4~, for one channel
static t_sample waveplayer_interp(t_waveplayer_tilde *x, int ch, int bufi, t_sample frac)
{
    int16_t *buf = x->x_buf[ch];
    int16_t *last3 = x->x_buf_last3[ch];
    t_sample a, b, c, d, cminusb;

    // fwd
    if (x->x_speed >= 0) {
        if (bufi == BUFSIZE - 2) {
            a = last3[0]; b = last3[1]; c = last3[2]; d = buf[0];
        }
        else if (bufi == BUFSIZE - 1) {
            a = last3[1]; b = last3[2]; c = buf[0]; d = buf[1];
        }
        else if (bufi == 0) {
            a = last3[2]; b = buf[0]; c = buf[1]; d = buf[2];
        }
        else {
            a = buf[bufi-1]; b = buf[bufi]; c = buf[bufi+1]; d = buf[bufi+2];
        }
    }
    // reverse
    else {
        if (bufi == 0) {
            a = buf[BUFSIZE - 1]; b = last3[0]; c = last3[1]; d = last3[2];
        }
        else if (bufi == BUFSIZE - 1) {
            a = buf[BUFSIZE - 2]; b = buf[BUFSIZE - 1]; c = last3[0]; d = last3[1];
        }
        else if (bufi == BUFSIZE - 2) {
            a = buf[BUFSIZE - 3]; b = buf[BUFSIZE - 2]; c = buf[BUFSIZE - 1]; d = last3[0];
        }
        else {
            a = buf[bufi-1]; b = buf[bufi]; c = buf[bufi+1]; d = buf[bufi+2];
        }
    }

    a /= 32768; b /= 32768; c /= 32768; d /= 32768;

    cminusb = c - b;
    return b + frac * (
        cminusb - 0.1666667f * (1.f - frac) * (
            (d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b)
        )
    );
}

// duck gain near either edge of the loop window, to mask the waveform
// discontinuity at the seam; ramps 0->1 moving away from loop_start and
// 1->0 approaching loop_end, so it fades down before the wrap and back
// up after it without needing to detect the wrap itself
static t_sample waveplayer_loop_declick_gain(t_waveplayer_tilde *x)
{
    t_int fade = x->x_loop_declick_frames;
    t_int half = (x->x_loop_end - x->x_loop_start) / 2;
    double from_start, from_end, g;

    if (fade > half) fade = half;
    if (fade < 1) return 1;

    from_start = x->x_pos - x->x_loop_start;
    from_end = x->x_loop_end - x->x_pos;
    g = (from_start < from_end) ? from_start : from_end;
    g /= fade;

    if (g < 0) g = 0;
    if (g > 1) g = 1;
    return (t_sample)g;
}

// step the start/stop envelope by one sample, toward x_gain_target
static void waveplayer_update_gain(t_waveplayer_tilde *x)
{
    if (x->x_gain < x->x_gain_target) {
        x->x_gain += x->x_gain_inc;
        if (x->x_gain > x->x_gain_target) x->x_gain = x->x_gain_target;
    }
    else if (x->x_gain > x->x_gain_target) {
        x->x_gain -= x->x_gain_inc;
        if (x->x_gain < x->x_gain_target) x->x_gain = x->x_gain_target;
    }

    // once a stop's fade-out has fully reached silence, rewind to the loop start
    if (x->x_stop_pending && x->x_gain <= 0) {
        x->x_pos = x->x_loop_start + 1;
        x->x_current_buf_num = -1; // force re-read
        x->x_stop_pending = 0;
    }
}

static t_int *waveplayer_tilde_perform_mono(t_int *w)
{
    t_waveplayer_tilde *x = (t_waveplayer_tilde *)(w[1]);
    t_sample *out = (t_sample *)(w[2]);
    int n = (int)(w[3]);
    int i, bufi;
    t_sample frac;

    for (i = 0; i < n; i++) {
        waveplayer_update_gain(x);
        if (x->x_running || x->x_gain > 0) {
            waveplayer_advance(x, &bufi, &frac);
            *out++ = waveplayer_interp(x, 0, bufi, frac) * x->x_gain * waveplayer_loop_declick_gain(x);
        }
        else *out++ = 0;
    }
    return (w+4);
}

static t_int *waveplayer_tilde_perform_stereo(t_int *w)
{
    t_waveplayer_tilde *x = (t_waveplayer_tilde *)(w[1]);
    t_sample *out0 = (t_sample *)(w[2]);
    t_sample *out1 = (t_sample *)(w[3]);
    int n = (int)(w[4]);
    int i, bufi;
    t_sample frac, declick;

    for (i = 0; i < n; i++) {
        waveplayer_update_gain(x);
        if (x->x_running || x->x_gain > 0) {
            waveplayer_advance(x, &bufi, &frac);
            declick = x->x_gain * waveplayer_loop_declick_gain(x);
            *out0++ = waveplayer_interp(x, 0, bufi, frac) * declick;
            *out1++ = waveplayer_interp(x, 1, bufi, frac) * declick;
        }
        else {
            *out0++ = 0;
            *out1++ = 0;
        }
    }
    return (w+5);
}

static void waveplayer_tilde_dsp(t_waveplayer_tilde *x, t_signal **sp)
{
    t_int ramp_samples = (t_int)(sp[0]->s_sr * DECLICK_MS / 1000.0);
    if (ramp_samples < 1) ramp_samples = 1;
    x->x_gain_inc = 1.0f / ramp_samples;
    x->x_loop_declick_frames = ramp_samples;

    if (x->x_channels > 1)
        dsp_add(waveplayer_tilde_perform_stereo, 4, x, sp[0]->s_vec, sp[1]->s_vec, (t_int)sp[0]->s_n);
    else
        dsp_add(waveplayer_tilde_perform_mono, 3, x, sp[0]->s_vec, (t_int)sp[0]->s_n);
}

static void waveplayer_tilde_free(t_waveplayer_tilde *x)
{
    int ch;
    for (ch = 0; ch < x->x_channels; ch++) outlet_free(x->x_out[ch]);
    if (x->x_fh != NULL) fclose(x->x_fh);
}

static void waveplayer_set_speed(t_waveplayer_tilde *x, t_floatarg f){
    if (f > 3) f = 3;
    if (f < -3) f = -3;
    x->x_speed = f;
}

static void waveplayer_open(t_waveplayer_tilde *x, t_symbol *s, int argc, t_atom *argv){
    t_symbol *filesym = atom_getsymbolarg(0, argc, argv);
    openfile(x, filesym->s_name);
}

static void waveplayer_loop_start(t_waveplayer_tilde *x, t_floatarg f){
    x->x_loop_start = (t_int)f;
    update_loop_bounds(x);
}

// size <= 0 means "use the entire file from loop_start onward"
static void waveplayer_loop_size(t_waveplayer_tilde *x, t_floatarg f){
    x->x_loop_size = (f > 0) ? (t_int)f : -1;
    update_loop_bounds(x);
}

// resume playback (fades in) from wherever x_pos currently is
static void waveplayer_start(t_waveplayer_tilde *x){
    x->x_running = 1;
    x->x_gain_target = 1;
    x->x_stop_pending = 0; // a start cancels any rewind a prior stop was waiting to do
}

// halt playback (fades out, then freezes position until start/pause is sent again)
static void waveplayer_pause(t_waveplayer_tilde *x){
    x->x_running = 0;
    x->x_gain_target = 0;
    x->x_stop_pending = 0;
}

// halt playback (fades out, then rewinds to loop_start; next start begins there)
static void waveplayer_stop(t_waveplayer_tilde *x){
    x->x_running = 0;
    x->x_gain_target = 0;
    x->x_stop_pending = 1;
}

static void *waveplayer_tilde_new(t_symbol *s, int argc, t_atom *argv)
{
    t_waveplayer_tilde *x = (t_waveplayer_tilde *)pd_new(waveplayer_tilde_class);
    int channels = 1;

    if (argc > 0) {
        channels = (int)atom_getfloatarg(0, argc, argv);
        if (channels != 1 && channels != 2) {
            pd_error(x, "waveplayer~: channel count must be 1 or 2, defaulting to 1");
            channels = 1;
        }
    }
    x->x_channels = channels;

    x->x_out[0] = outlet_new(&x->x_obj, &s_signal);
    if (x->x_channels > 1) x->x_out[1] = outlet_new(&x->x_obj, &s_signal);

    x->x_file_channels = 1;
    x->x_data_offset = 44;
    x->x_loop_start = 0;
    x->x_loop_size = -1;       // default: entire file
    x->x_loop_end = 44100;
    x->x_num_frames = 0;
    x->x_pos = 1;
    x->x_current_buf_num = -1; // force read
    x->x_speed = 1;
    x->x_running = 1;
    x->x_stop_pending = 0;
    x->x_gain = 1;
    x->x_gain_target = 1;
    x->x_gain_inc = 1.0f / 200; // sane default until dsp() sets it from the real sample rate
    x->x_loop_declick_frames = 200; // sane default until dsp() sets it from the real sample rate
    x->x_fh = NULL;
    memset(x->x_buf, 0, sizeof(x->x_buf));
    memset(x->x_buf_last3, 0, sizeof(x->x_buf_last3));

    return (void *)x;
}

void waveplayer_tilde_setup(void) {
    waveplayer_tilde_class = class_new(gensym("waveplayer~"),
        (t_newmethod)waveplayer_tilde_new,
        (t_method)waveplayer_tilde_free,
        sizeof(t_waveplayer_tilde),
        CLASS_DEFAULT,
        A_GIMME, 0);

    class_addfloat(waveplayer_tilde_class, (t_method)waveplayer_set_speed);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_open, gensym("open"), A_GIMME, 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_loop_start, gensym("loop_start"), A_FLOAT, 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_loop_size, gensym("loop_size"), A_FLOAT, 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_start, gensym("start"), 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_pause, gensym("pause"), 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_stop, gensym("stop"), 0);
    class_addmethod(waveplayer_tilde_class, (t_method)waveplayer_tilde_dsp, gensym("dsp"), A_CANT, 0);

}
