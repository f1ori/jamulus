/******************************************************************************\
 * Copyright (c) 2004-2020
 *
 * Author(s):
 *  Volker Fischer
 *
 * This code is based on the simple_client example of the Jack audio interface.
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 *
\******************************************************************************/

#include "sound.h"


#ifdef WITH_JACK
void CSound::OpenJack ( const bool  bNoAutoJackConnect,
                        const char* jackClientName )
{
    jack_status_t JackStatus;
    const char *serverName;

    if ( ( serverName = getenv( "JACK_DEFAULT_SERVER" ) ) == NULL ) {
        serverName = "default";
    }
    qInfo() << qUtf8Printable( QString( "Connecting to jack \"%1\" instance (use the JACK_DEFAULT_SERVER environment variable to change this)." )
        .arg( serverName ) );

    // try to become a client of the JACK server
    pJackClient = jack_client_open ( jackClientName, JackNullOption, &JackStatus );

    if ( pJackClient == nullptr )
    {
        throw CGenErr ( tr ( "The Jack server is not running. This software "
            "requires a Jack server to run. Normally if the Jack server is "
            "not running this software will automatically start the Jack server. "
            "It seems that this auto start has not worked. Try to start the Jack "
            "server manually." ) );
    }

    // tell the JACK server to call "process()" whenever
    // there is work to be done
    jack_set_process_callback ( pJackClient, process, this );

    // register a "buffer size changed" callback function
    jack_set_buffer_size_callback ( pJackClient, bufferSizeCallback, this );

    // register shutdown callback function
    jack_on_shutdown ( pJackClient, shutdownCallback, this );

    // check sample rate, if not correct, just fire error
    if ( jack_get_sample_rate ( pJackClient ) != SYSTEM_SAMPLE_RATE_HZ )
    {
        throw CGenErr ( tr ( "The Jack server sample rate is different from "
            "the required one. The required sample rate is:" ) + " <b>" +
            QString().setNum ( SYSTEM_SAMPLE_RATE_HZ ) + " Hz</b>. " + tr ( "You can "
            "use a tool like <i><a href=\"https://qjackctl.sourceforge.io\">QJackCtl</a></i> "
            "to adjust the Jack server sample rate." ) + "<br>" + tr ( "Make sure to set the "
            "Frames/Period to a low value like " ) +
            QString().setNum ( DOUBLE_SYSTEM_FRAME_SIZE_SAMPLES ) +
            tr ( " to achieve a low delay." ) );
    }

    // create four ports (two for input, two for output -> stereo)
    input_port_left = jack_port_register ( pJackClient, "input left",
        JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0 );

    input_port_right = jack_port_register ( pJackClient, "input right",
        JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0 );

    output_port_left = jack_port_register ( pJackClient, "output left",
        JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0 );

    output_port_right = jack_port_register ( pJackClient, "output right",
        JACK_DEFAULT_AUDIO_TYPE, JackPortIsOutput, 0 );

    if ( ( input_port_left   == nullptr ) ||
         ( input_port_right  == nullptr ) ||
         ( output_port_left  == nullptr ) ||
         ( output_port_right == nullptr ) )
    {
        throw CGenErr ( tr ( "The Jack port registering failed." ) );
    }

    // optional MIDI initialization
    if ( iCtrlMIDIChannel != INVALID_MIDI_CH )
    {
        input_port_midi = jack_port_register ( pJackClient, "input midi",
            JACK_DEFAULT_MIDI_TYPE, JackPortIsInput, 0 );

        if ( input_port_midi == nullptr )
        {
            throw CGenErr ( tr ( "The Jack port registering failed." ) );
        }
    }
    else
    {
        input_port_midi = nullptr;
    }

    // tell the JACK server that we are ready to roll
    if ( jack_activate ( pJackClient ) )
    {
        throw CGenErr ( tr ( "Cannot activate the Jack client." ) );
    }

    if ( !bNoAutoJackConnect )
    {
        // connect the ports, note: you cannot do this before
        // the client is activated, because we cannot allow
        // connections to be made to clients that are not
        // running
        const char** ports;

        // try to connect physical input ports
        if ( ( ports = jack_get_ports ( pJackClient,
                                        nullptr,
                                        nullptr,
                                        JackPortIsPhysical | JackPortIsOutput ) ) != nullptr )
        {
            jack_connect ( pJackClient, ports[0], jack_port_name ( input_port_left ) );

            // before connecting the second stereo channel, check if the input is not mono
            if ( ports[1] )
            {
                jack_connect ( pJackClient, ports[1], jack_port_name ( input_port_right ) );
            }

            jack_free ( ports );
        }

        // try to connect physical output ports
        if ( ( ports = jack_get_ports ( pJackClient,
                                        nullptr,
                                        nullptr,
                                        JackPortIsPhysical | JackPortIsInput ) ) != nullptr )
        {
            jack_connect ( pJackClient, jack_port_name ( output_port_left ), ports[0] );

            // before connecting the second stereo channel, check if the output is not mono
            if ( ports[1] )
            {
                jack_connect ( pJackClient, jack_port_name ( output_port_right ), ports[1] );
            }

            jack_free ( ports );
        }

        // input latency
        jack_latency_range_t latrange;
        latrange.min = 0;
        latrange.max = 0 ;

        jack_port_get_latency_range ( input_port_left, JackCaptureLatency, &latrange );
        int inLatency = latrange.min; // be optimistic

        // output latency 
        latrange.min = 0; 
        latrange.max = 0 ;

        jack_port_get_latency_range ( output_port_left, JackPlaybackLatency, &latrange );
        int outLatency = latrange.min; // be optimistic

        // compute latency by using the first input and first output
        // ports and using the most optimistic values
        fInOutLatencyMs = static_cast<float> ( inLatency + outLatency ) * 1000 / SYSTEM_SAMPLE_RATE_HZ;
    }
}

void CSound::CloseJack()
{
    // deactivate client
    jack_deactivate ( pJackClient );

    // unregister ports
    jack_port_unregister ( pJackClient, input_port_left );
    jack_port_unregister ( pJackClient, input_port_right );
    jack_port_unregister ( pJackClient, output_port_left );
    jack_port_unregister ( pJackClient, output_port_right );

    // close client connection to jack server
    jack_client_close ( pJackClient );
}

void CSound::Start()
{
    // call base class
    CSoundBase::Start();
}

void CSound::Stop()
{
    // call base class
    CSoundBase::Stop();
}

int CSound::Init ( const int /* iNewPrefMonoBufferSize */ )
{

// try setting buffer size
// TODO seems not to work! -> no audio after this operation!
// Doesn't this give an infinite loop? The set buffer size function will call our
// registered callback which calls "EmitReinitRequestSignal()". In that function
// this CSound::Init() function is called...
//jack_set_buffer_size ( pJackClient, iNewPrefMonoBufferSize );

    // without a Jack server, Jamulus makes no sense to run, throw an error message
    if ( bJackWasShutDown )
    {
        throw CGenErr ( tr ( "The Jack server was shut down. This software "
            "requires a Jack server to run. Try to restart the software to "
            "solve the issue." ) );
    }

    // get actual buffer size
    iJACKBufferSizeMono = jack_get_buffer_size ( pJackClient );  	

    // init base class
    CSoundBase::Init ( iJACKBufferSizeMono );

    // set internal buffer size value and calculate stereo buffer size
    iJACKBufferSizeStero = 2 * iJACKBufferSizeMono;

    // create memory for intermediate audio buffer
    vecsTmpAudioSndCrdStereo.Init ( iJACKBufferSizeStero );

    return iJACKBufferSizeMono;
}


// JACK callbacks --------------------------------------------------------------
int CSound::process ( jack_nframes_t nframes, void* arg )
{
    CSound* pSound = static_cast<CSound*> ( arg );
    int     i;

    // make sure we are locked during execution
    QMutexLocker locker ( &pSound->MutexAudioProcessCallback );

    if ( pSound->IsRunning() && ( nframes == static_cast<jack_nframes_t> ( pSound->iJACKBufferSizeMono ) ) )
    {
        // get input data pointer
        jack_default_audio_sample_t* in_left =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->input_port_left, nframes );

        jack_default_audio_sample_t* in_right =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->input_port_right, nframes );

        // copy input audio data
        if ( ( in_left != nullptr ) && ( in_right != nullptr ) )
        {
            for ( i = 0; i < pSound->iJACKBufferSizeMono; i++ )
            {
                pSound->vecsTmpAudioSndCrdStereo[2 * i]     = Float2Short ( in_left[i] * _MAXSHORT );
                pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] = Float2Short ( in_right[i] * _MAXSHORT );
            }
        }

        // call processing callback function
        pSound->ProcessCallback ( pSound->vecsTmpAudioSndCrdStereo );

        // get output data pointer
        jack_default_audio_sample_t* out_left =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->output_port_left, nframes );

        jack_default_audio_sample_t* out_right =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->output_port_right, nframes );

        // copy output data
        if ( ( out_left != nullptr ) && ( out_right != nullptr ) )
        {
            for ( i = 0; i < pSound->iJACKBufferSizeMono; i++ )
            {
                out_left[i] = (jack_default_audio_sample_t)
                    pSound->vecsTmpAudioSndCrdStereo[2 * i] / _MAXSHORT;

                out_right[i] = (jack_default_audio_sample_t)
                    pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] / _MAXSHORT;
            }
        }
    }
    else
    {
        // get output data pointer
        jack_default_audio_sample_t* out_left =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->output_port_left, nframes );

        jack_default_audio_sample_t* out_right =
            (jack_default_audio_sample_t*) jack_port_get_buffer (
            pSound->output_port_right, nframes );

        // clear output data
        if ( ( out_left != nullptr ) && ( out_right != nullptr ) )
        {
            memset ( out_left,
                     0,
                     sizeof ( jack_default_audio_sample_t ) * nframes );

            memset ( out_right,
                     0,
                     sizeof ( jack_default_audio_sample_t ) * nframes );
        }
    }

    // akt on MIDI data if MIDI is enabled
    if ( pSound->input_port_midi != nullptr )
    {
        void* in_midi = jack_port_get_buffer ( pSound->input_port_midi, nframes );

        if ( in_midi != 0 )
        {
            jack_nframes_t event_count = jack_midi_get_event_count ( in_midi );

            for ( jack_nframes_t j = 0; j < event_count; j++ )
            {
                jack_midi_event_t in_event;

                jack_midi_event_get ( &in_event, in_midi, j );

                // copy packet and send it to the MIDI parser
// TODO do not call malloc in real-time callback
                CVector<uint8_t> vMIDIPaketBytes ( in_event.size );

                for ( i = 0; i < static_cast<int> ( in_event.size ); i++ )
                {
                    vMIDIPaketBytes[i] = static_cast<uint8_t> ( in_event.buffer[i] );
                }
                pSound->ParseMIDIMessage ( vMIDIPaketBytes );
            }
        }
    }

    return 0; // zero on success, non-zero on error 
}

int CSound::bufferSizeCallback ( jack_nframes_t, void* arg )
{
    CSound* pSound = static_cast<CSound*> ( arg );

    pSound->EmitReinitRequestSignal ( RS_ONLY_RESTART_AND_INIT );

    return 0; // zero on success, non-zero on error
}

void CSound::shutdownCallback ( void* arg )
{
    CSound* pSound = static_cast<CSound*> ( arg );

    pSound->bJackWasShutDown = true;
    pSound->EmitReinitRequestSignal ( RS_ONLY_RESTART_AND_INIT );
}
#endif // WITH_JACK


#ifdef WITH_PIPEWIRE

#include <spa/param/audio/format-utils.h>
#include <algorithm>

const uint8_t CSound::RING_FACTOR = 10;


void CSound::OpenPipewire()
{
    pw_init(0, NULL);

    loop = pw_thread_loop_new("pipewire_thread", NULL /*properties*/);
    if (loop == nullptr)
        throw CGenErr("Could not create pipewire thread");

    static const struct pw_stream_events input_stream_events = {
        .version = PW_VERSION_STREAM_EVENTS,
        .process = &CSound::onProcessInput,
    };
    input_stream = pw_stream_new_simple(
                pw_thread_loop_get_loop(loop),
                "audio-capture",
                pw_properties_new(
                    PW_KEY_MEDIA_TYPE, "Audio",
                    PW_KEY_MEDIA_CATEGORY, "Capture",
                    PW_KEY_MEDIA_ROLE, "Production",
                    NULL),
                &input_stream_events,
                this);
    if (input_stream == nullptr)
        throw CGenErr("Could not create pipewire audio input");

    static const struct pw_stream_events output_stream_events = {
        .version = PW_VERSION_STREAM_EVENTS,
        .process = &CSound::onProcessOutput,
    };
    output_stream = pw_stream_new_simple(
                pw_thread_loop_get_loop(loop),
                "audio-src",
                pw_properties_new(
                    PW_KEY_MEDIA_TYPE, "Audio",
                    PW_KEY_MEDIA_CATEGORY, "Playback",
                    PW_KEY_MEDIA_ROLE, "Production",
                    NULL),
                &output_stream_events,
                this);
    if (input_stream == nullptr)
        throw CGenErr("Could not create pipewire audio output");

    const struct spa_pod *params[1];
    uint8_t buffer[1024];
    static struct spa_pod_builder b = {buffer, sizeof(buffer), 0, {0, 0, 0}, {0, 0}};
    // add support for SPA_AUDIO_FORMAT_F32P
    static struct spa_audio_info_raw air = {.format = SPA_AUDIO_FORMAT_S16,
                .rate = SYSTEM_SAMPLE_RATE_HZ,
                .channels = 2 };
    params[0] = spa_format_audio_raw_build(&b, SPA_PARAM_EnumFormat, &air);

    if (pw_stream_connect(input_stream,
              PW_DIRECTION_INPUT,
              PW_ID_ANY,
              static_cast<enum pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT
                                                | PW_STREAM_FLAG_INACTIVE
                                                | PW_STREAM_FLAG_MAP_BUFFERS),
              params, 1) < 0)
        throw CGenErr("Could not connect input stream");
    // add PW_STREAM_FLAG_RT_PROCESS?

    if (pw_stream_connect(output_stream,
              PW_DIRECTION_OUTPUT,
              PW_ID_ANY,
              static_cast<enum pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT
                                                | PW_STREAM_FLAG_INACTIVE
                                                | PW_STREAM_FLAG_MAP_BUFFERS),
              params, 1) < 0)
        throw CGenErr("Could not connect output stream");

    pw_stream_set_active(output_stream, false);
    pw_stream_set_active(input_stream, false);

    // to set latency requirements, we need to create /get a node

#if 0
    context = pw_context_new(pw_thread_loop_get_loop(loop),
                    NULL /* properties */,
                    0 /* user_data size */);

    core = pw_context_connect(context,
                    NULL /* properties */,
                    0 /* user_data size */);

    registry = pw_core_get_registry(core, PW_VERSION_REGISTRY,
                    0 /* user_data size */);

    spa_zero(registry_listener);
    pw_registry_add_listener(registry, &registry_listener,
                                   &registry_events, NULL);
#endif

    pw_thread_loop_start(loop);
}

void CSound::ClosePipewire()
{
    //pw_proxy_destroy((struct pw_proxy*)registry);
    //pw_core_disconnect(core);
    //pw_context_destroy(context);
    if (input_stream)
    {
        pw_stream_destroy(input_stream);
        input_stream = NULL;
    }
    if (output_stream)
    {
         pw_stream_destroy(output_stream);
         output_stream = NULL;
    }
    if (loop)
    {
        pw_thread_loop_stop(loop);
        pw_thread_loop_destroy(loop);
        loop = NULL;
    }
}


void CSound::Start()
{
    // call base class
    CSoundBase::Start();

    pw_thread_loop_lock(loop);
    pw_stream_set_active(output_stream, true);
    pw_stream_set_active(input_stream, true);
    pw_thread_loop_unlock(loop);
}


void CSound::Stop()
{
    pw_thread_loop_lock(loop);
    pw_stream_set_active(input_stream, false);
    pw_stream_set_active(output_stream, false);
    pw_thread_loop_unlock(loop);

    // call base class
    CSoundBase::Stop();
}

int CSound::Init ( const int /* iNewPrefMonoBufferSize */ )
{
    pw_log_warn("init");
    iPipewireBufferSizeMono = 1024;
    // init base class
    CSoundBase::Init ( iPipewireBufferSizeMono );

    iPipewireBufferSizeStero = 2 * iPipewireBufferSizeMono;

    // create memory for intermediate audio buffer
    vecsTmpAudioSndCrdStereo.Init ( iPipewireBufferSizeStero );
    mOutBuffer.reset ( iPipewireBufferSizeStero * RING_FACTOR );

    return iPipewireBufferSizeMono;
}

// pipewire callbacks --------------------------------------------------------------
void CSound::onProcessInput(void* userdata)
{
    CSound* pSound = static_cast<CSound*>(userdata);
    QMutexLocker locker ( &pSound->MutexAudioProcessCallback );

    struct pw_buffer *b;
    struct spa_buffer *buf;

    // get input data
    if ((b = pw_stream_dequeue_buffer(pSound->input_stream)) == NULL)
    {
        pw_log_warn("out of input buffers: %m");
        return;
    }

    buf = b->buffer;
    if (buf->datas[0].data == NULL)
        return;

    pw_log_warn("input");
    if (pSound->IsRunning())
    {
        // copy input audio data
        struct spa_chunk* chunk = buf->datas[0].chunk;
        int n = std::min(static_cast<int>(chunk->size/sizeof(uint16_t)), pSound->vecsTmpAudioSndCrdStereo.Size());
        int16_t* data = reinterpret_cast<int16_t*>(static_cast<uint8_t*>(buf->datas[0].data) + chunk->offset);
        pw_log_warn("input frames %d stride %d", n, chunk->stride);
        if (n != pSound->iPipewireBufferSizeStero)
            pw_log_warn("incomplete frame (size=%d)", n);

        for (int i = 0; i < n; i++ )
            pSound->vecsTmpAudioSndCrdStereo[i] = *data++;

        for (int i = n; i < pSound->iPipewireBufferSizeStero; i++)
            pSound->vecsTmpAudioSndCrdStereo[i] = 0;

        pSound->ProcessCallback ( pSound->vecsTmpAudioSndCrdStereo );
        pSound->addOutputData();
    }
    if (pw_stream_queue_buffer(pSound->input_stream, b) < 0)
        pw_log_warn("Could not free: %m");
}


void CSound::addOutputData()
{
    // Only copy data if we have data to copy, otherwise fill with silence
    if ( !vecsTmpAudioSndCrdStereo.empty() )
    {
        for ( int frmNum = 0; frmNum < iPipewireBufferSizeMono; ++frmNum )
        {
            for ( int channelNum = 0; channelNum < 2; channelNum++ )
            {
                // copy sample received from server into output buffer
                mOutBuffer.put ( vecsTmpAudioSndCrdStereo[frmNum * 2 + channelNum] );
            }
        }
    }
    else
    {
        // prime output stream buffer with silence
        for ( int frmNum = 0; frmNum < iPipewireBufferSizeMono; ++frmNum )
        {
            for ( int channelNum = 0; channelNum < 2; channelNum++ )
            {
                mOutBuffer.put ( 0 );
            }
        }
    }
    if ( mOutBuffer.isFull() )
    {
        // TODO add stats
        pw_log_warn("Ring overrun");
    }
}


void CSound::onProcessOutput(void* userdata)
{
    CSound* pSound = static_cast<CSound*>(userdata);
    QMutexLocker locker ( &pSound->MutexAudioProcessCallback );

    pw_log_warn("output");
    if (!pSound->IsRunning())
        return;

    if (pSound->mOutBuffer.isEmpty())
    {
        pw_log_warn("empty output");
        return;
    }

    struct pw_buffer *b;
    struct spa_buffer *buf;

    // get output buffer
    if ((b = pw_stream_dequeue_buffer(pSound->output_stream)) == NULL) {
        pw_log_warn("out of output buffers: %m");
        return;
    }

    int16_t *dst;
    buf = b->buffer;
    if ((dst = static_cast<int16_t*>(buf->datas[0].data)) == NULL)
        return;
    size_t n_frames = std::min(static_cast<size_t>(buf->datas[0].maxsize / sizeof(int16_t)),
                               pSound->mOutBuffer.size());
    n_frames = n_frames > pSound->iPipewireBufferSizeStero? pSound->iPipewireBufferSizeStero : n_frames;
    pw_log_warn("output frames %d", n_frames);

    pSound->mOutBuffer.get (dst, n_frames);

    buf->datas[0].chunk->offset = 0;
    buf->datas[0].chunk->stride= sizeof(int16_t) * 2;
    buf->datas[0].chunk->size = n_frames * sizeof(int16_t);

    if (pw_stream_queue_buffer(pSound->output_stream, b) < 0)
        pw_log_warn("Could not send: %m");
}


void CSound::onParamChangedInput(void *userdata, uint32_t id, const struct spa_pod *param)
{
    CSound* pSound = static_cast<CSound*>(userdata);
    QMutexLocker locker ( &pSound->MutexAudioProcessCallback );

    if (param == NULL || id != SPA_PARAM_Format)
        return;

    if (spa_format_parse(param,
            &pSound->input_format.media_type,
            &pSound->input_format.media_subtype) < 0)
        return;

    if (pSound->input_format.media_type != SPA_MEDIA_TYPE_audio ||
        pSound->input_format.media_subtype != SPA_MEDIA_SUBTYPE_raw)
        return;

    if (spa_format_audio_raw_parse(param, &pSound->input_format.info.raw) < 0)
        return;

    printf("got video format:\n");
    /*
    printf("  format: %d (%s)\n", pSound->format.info.raw.format,
            spa_debug_type_find_name(spa_type_video_format,
                data->format.info.raw.format));*/
    printf("  rate: %d channels: %d\n", pSound->input_format.info.raw.rate,
            pSound->input_format.info.raw.channels);

}


#endif // WITH_PIPEWIRE
