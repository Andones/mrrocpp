// ------------------------------------------------------------------------
// Proces:		EDP
// Plik:			edp_speaker_effector.cc
// System:	QNX/MRROC++  v. 6.3
// Opis:		Robot speaker
//				- definicja metod klasy edp_speaker_effector
//				- definicja funkcji return_created_efector()
//
// Autor:		tkornuta
// Data:		17.01.2007
// ------------------------------------------------------------------------

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/sched.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <errno.h>
#include <pthread.h>
#include <process.h>
#include <sys/netmgr.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"
#include "lib/mis_fun.h"

#include "edp/speaker/sound.h" // MAC7
//#include "edp/speaker/stdafx.h" // MAC& [ARTUR]
#include "edp/speaker/tts.h" // MAC& [ARTUR]

// Klasa edp_speaker_effector.
#include "edp/speaker/edp_speaker_effector.h"


// char text2speak[MAX_TEXT]; // MAC 7
// char prosody[MAX_PROSODY]; // MAC 7
// bool speaking; // MAC7


namespace mrrocpp {
namespace edp {
namespace common {

edp_speaker_effector::edp_speaker_effector (configurator &_config)
        : edp_effector(_config, ROBOT_SPEAKER)
{

    mt_tt_obj = new master_trans_t_buffer();
}
;


void edp_speaker_effector::initialize (void)
{
    real_reply_type = ACKNOWLEDGE;
    // inicjacja deskryptora pliku by 7&Y
    // serwo_fd = name_open(EDP_ATTACH_POINT, 0);

    speaking=0;


    /* Ustawienie priorytetu procesu */

    set_thread_priority(pthread_self() , MAX_PRIORITY-2);

    edp_tid=1;// numer watku edp_master

}
;


int edp_speaker_effector::init ()
{
    // inicjacja buforow
    msg->message ("Initialization in progress ...");

    init_buffers_from_files();

    card = -1;
    dev = 0;

    piBuffSpeechOut = (short int*) calloc(MAXOUTDATA, sizeof(short int));

    setvbuf (stdin, NULL, _IONBF, 0);

    if (card == -1)
    {
        if ((rtn = snd_pcm_open_preferred (&pcm_handle, &card, &dev, SND_PCM_OPEN_PLAYBACK)) < 0)
        {
            printf("error: open preffered failed\n");
            return -1;
        }
    }
    else
    {
        if ((rtn = snd_pcm_open (&pcm_handle, card, dev, SND_PCM_OPEN_PLAYBACK)) < 0)
        {
            printf("error: open failed\n");
            return -1;
        }
    }

    mSampleRate = 44000; //16000;
    mSampleChannels = 1;
    mSampleBits = 16;

    // printf ("SampleRate = %d, Channels = %d, SampleBits = %d\n", mSampleRate, mSampleChannels, mSampleBits);
    // disabling mmap is not actually required in this example but it is included to
    // demonstrate how it is used when it is required.
    /*
    if ((rtn = snd_pcm_plugin_set_disable (pcm_handle, PLUGIN_DISABLE_MMAP)) < 0)
{
    	fprintf (stderr, "snd_pcm_plugin_set_disable failed: %s\n", snd_strerror (rtn));
    	return -1;
}
    */

    memset (&pi, 0, sizeof (pi));
    pi.channel = SND_PCM_CHANNEL_PLAYBACK;
    if ((rtn = snd_pcm_plugin_info (pcm_handle, &pi)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_info failed: %s\n", snd_strerror (rtn));
        return -1;
    }

    memset (&pp, 0, sizeof (pp));

    pp.mode = SND_PCM_MODE_BLOCK;
    pp.channel = SND_PCM_CHANNEL_PLAYBACK;
    pp.start_mode = SND_PCM_START_FULL;
    pp.stop_mode = SND_PCM_STOP_STOP;

    pp.buf.block.frag_size = pi.max_fragment_size;
    pp.buf.block.frags_max = 1;
    pp.buf.block.frags_min = 1;

    pp.format.interleave = 1;
    pp.format.rate = mSampleRate;
    pp.format.voices = mSampleChannels;

    if (mSampleBits == 8)
    {
        pp.format.format = SND_PCM_SFMT_U8;
    }
    else
    {
        pp.format.format = SND_PCM_SFMT_S16_LE;
    }

    if ((rtn = snd_pcm_plugin_params (pcm_handle, &pp)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_params failed: %s\n", snd_strerror (rtn));
        return -1;
    }

    if ((rtn = snd_pcm_plugin_prepare (pcm_handle, SND_PCM_CHANNEL_PLAYBACK)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_prepare failed: %s\n", snd_strerror (rtn));
    }

    memset (&setup, 0, sizeof (setup));
    memset (&group, 0, sizeof (group));
    setup.channel = SND_PCM_CHANNEL_PLAYBACK;
    setup.mixer_gid = &group.gid;
    if ((rtn = snd_pcm_plugin_setup (pcm_handle, &setup)) < 0)
    {
        fprintf (stderr, "snd_pcm_plugin_setup failed: %s\n", snd_strerror (rtn));
        return -1;
    }

    if (group.gid.name[0] == 0)
    {
        printf ("Mixer Pcm Group [%s] Not Set \n", group.gid.name);
        return (-1);
    }

    if ((rtn = snd_mixer_open (&mixer_handle, card, setup.mixer_device)) < 0)
    {
        fprintf (stderr, "snd_mixer_open failed: %s\n", snd_strerror (rtn));
        return -1;
    }

    FD_ZERO (&rfds);
    FD_ZERO (&wfds);


    FD_SET (STDIN_FILENO, &rfds);
    FD_SET (snd_mixer_file_descriptor (mixer_handle), &rfds);
    FD_SET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_PLAYBACK), &wfds);

    msg->message ("Initialization succesfull");

    return 0;
};

edp_speaker_effector::~edp_speaker_effector ()
{
    ;
    free(piBuffSpeechOut);
    rtn = snd_mixer_close (mixer_handle);
    rtn = snd_pcm_close (pcm_handle);
}

void edp_speaker_effector::create_threads ()
{

    if (pthread_create (&speak_t_tid, NULL, &speak_thread_start, (void *) this)!=EOK)
    {
        msg->message(SYSTEM_ERROR, errno, "EDP: Failed to spawn SPEAKER");
        char buf[20];
        netmgr_ndtostr(ND2S_LOCAL_STR, ND_LOCAL_NODE, buf, sizeof(buf));
        printf (" Failed to thread SPEAKER on node: %s\n", buf);
        throw System_error();
    }


};

/*--------------------------------------------------------------------------*/
void edp_speaker_effector::interpret_instruction (c_buffer *instruction)
{
    // interpretuje otrzyman z ECP instrukcj;
    // wypenaia struktury danych TRANSFORMATORa;
    // przygotowuje odpowied dla ECP
    // 	printf("interpret instruction poczatek\n");
    // wstpne przygotowanie bufora odpowiedzi
    rep_type(instruction); // okreslenie typu odpowiedzi
    reply.error_no.error0 = OK;
    reply.error_no.error1 = OK;

    // Wykonanie instrukcji
    switch ( (*instruction).instruction_type )
    {
    case SET:
        reply.arm.text_def.speaking=speaking;
        if(!speaking)
        {
            mt_tt_obj->master_to_trans_t_order(MT_MOVE_ARM, 0);
        }
        break;
    case GET:
        reply.arm.text_def.speaking=speaking;
        // mt_tt_obj->master_to_trans_t_order(MT_GET_ARM_POSITION, true);
        break;
    case SET_GET:
        reply.arm.text_def.speaking=speaking;
        mt_tt_obj->master_to_trans_t_order(MT_MOVE_ARM, 0);
        // mt_tt_obj->master_to_trans_t_order(MT_GET_ARM_POSITION, true);
        break;
    default: // blad
        // ustawi numer bledu
        throw NonFatal_error_2(INVALID_INSTRUCTION_TYPE);
    }
    ; // end: switch

    // printf("interpret instruction koniec\n");

}
; // end: edp_speaker_effector::interpret_instruction
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
REPLY_TYPE edp_speaker_effector::rep_type (c_buffer *instruction)
{
    // ustalenie formatu odpowiedzi
    reply.reply_type = ACKNOWLEDGE;

    return reply.reply_type;
}
; // end: edp_speaker_effector::rep_type
/*--------------------------------------------------------------------------*/

void edp_speaker_effector::get_spoken (bool read_hardware, c_buffer *instruction)
{ // MAC7
    return;
};

int edp_speaker_effector::speak (c_buffer *instruction)
{ // add by MAC7

    strcpy(text2speak,(*instruction).arm.text_def.text);
    strcpy(prosody,(*instruction).arm.text_def.prosody);

    speaking=1;

    if(!initialize_incorrect)
    {
        set_thread_priority(pthread_self() , 2);
        uicSamplesNo = SayIt(text2speak,prosody,piBuffSpeechOut);
        //clock_gettime( CLOCK_REALTIME , &e_time);
        set_thread_priority(pthread_self() , MAX_PRIORITY-10);
        mSamples = uicSamplesNo*2; // 42830; // MAC7 sprawdzic, czy dla roznych textow nie bedzie sie roznic // FindTag (file2, "data"); // 441000;

        if (snd_pcm_plugin_prepare (pcm_handle, SND_PCM_CHANNEL_PLAYBACK) < 0)
        {
            fprintf (stderr, "underrun: playback channel prepare error\n");
            return(-1);
        }

        if (FD_ISSET (snd_pcm_file_descriptor (pcm_handle, SND_PCM_CHANNEL_PLAYBACK), &wfds))
        {
            snd_pcm_plugin_write (pcm_handle, piBuffSpeechOut, mSamples);
        }
        n = snd_pcm_plugin_flush (pcm_handle, SND_PCM_CHANNEL_PLAYBACK);
    }
    speaking=0;

    return 1;
}
; // MAC




void edp_speaker_effector::main_loop (void)
{
    next_state = GET_INSTRUCTION; // MAC7 glosnikow nie trzeba synchronizowac ; )

    /* Nieskoczona petla wykonujca przejscia w grafie automatu (procesu EDP_MASTER) */
    for (;;)
    {

        try
        { // w tym bloku beda wylapywane wyjatki (bledy)
            switch (next_state)
            {
            case GET_INSTRUCTION:
                switch ( receive_instruction() )
                {
                case SET:
                    // printf("jestesmy w set\n"); // MAC7
                case GET:
                    // printf("jestesmy w get\n");// MAC7
                case SET_GET:
                    // printf("jestesmy w set_get\n");
                    // potwierdzenie przyjecia polecenia (dla ECP)
                    insert_reply_type(ACKNOWLEDGE);
                    reply_to_instruction();
                    break;
                case SYNCHRO: // blad: robot jest juz zsynchronizowany
                    // okreslenie numeru bledu
                    throw edp_irp6s_effector::NonFatal_error_1(ALREADY_SYNCHRONISED);
                case QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
                    // okreslenie numeru bledu
                    throw edp_irp6s_effector::NonFatal_error_1(QUERY_NOT_EXPECTED);
                default: // blad: nieznana instrukcja
                    // okreslenie numeru bledu
                    throw edp_irp6s_effector::NonFatal_error_1(UNKNOWN_INSTRUCTION);
                }
                ; // end: switch ( receive_instruction(msg_cb) )
                if (is_reply_type_ERROR())
                    printf("ERROR GET_INSTRUCTION 2 aaa\n");
                next_state = EXECUTE_INSTRUCTION;
                break;
            case EXECUTE_INSTRUCTION:
                // printf("jestesmy w execute instruction\n"); // MAC7
                // wykonanie instrukcji - wszelkie bledy powoduja zgloszenie wyjtku NonFatal_error_2 lub Fatal_error
                interpret_instruction (&(new_instruction));
                //      printf("w execute po interpret\n");
                next_state = WAIT;
                break;
            case WAIT:
                //  	printf("jestesmy w wait\n");

                if ( receive_instruction() == QUERY )
                { // instrukcja wlasciwa =>
                    // zle jej wykonanie, czyli wyslij odpowiedz
                    reply_to_instruction();
                }
                else
                { // blad: powinna byla nadejsc instrukcja QUERY
                    throw edp_irp6s_effector::NonFatal_error_3(QUERY_EXPECTED);
                }
                next_state = GET_INSTRUCTION;
                break;
            default:
                break;
            }
            ; // end: switch (next_state)
        } // end: try

        catch(edp_irp6s_effector::NonFatal_error_1 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            establish_error(nfe.error,OK);
            // printf("ERROR w EDP 1\n");
            // informacja dla ECP o bledzie
            reply_to_instruction();
            msg->message(NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // powrot do stanu: GET_INSTRUCTION
            next_state = GET_INSTRUCTION;
        } // end: catch(transformer::NonFatal_error_1 nfe)

        catch(edp_irp6s_effector::NonFatal_error_2 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow

            // printf("ERROR w EDP 2\n");
            establish_error(nfe.error,OK);
            msg->message(NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::NonFatal_error_2 nfe)

        catch(edp_irp6s_effector::NonFatal_error_3 nfe)
        {
            // Obsluga bledow nie fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu nfe
            // S to bledy nie zwiazane ze sprzetem i komunikacja miedzyprocesow
            // zapamietanie poprzedniej odpowiedzi
            // Oczekiwano na QUERY a otrzymano co innego, wiec sygnalizacja bledu i
            // dalsze oczekiwanie na QUERY
            REPLY_TYPE rep_type = is_reply_type();
            uint64_t err_no_0 = is_error_no_0();
            uint64_t err_no_1 = is_error_no_1();

            establish_error(nfe.error,OK);
            // informacja dla ECP o bledzie
            reply_to_instruction();
            // przywrocenie poprzedniej odpowiedzi
            insert_reply_type(rep_type);
            establish_error(err_no_0,err_no_1);
            // printf("ERROR w EDP 3\n");
            msg->message(NON_FATAL_ERROR, nfe.error, (uint64_t) 0);
            // msg->message(NON_FATAL_ERROR, err_no_0, err_no_1); // by Y - oryginalnie
            // powrot do stanu: GET_INSTRUCTION
            next_state = GET_INSTRUCTION;
        } // end: catch(transformer::NonFatal_error_3 nfe)

        catch(edp_irp6s_effector::Fatal_error fe)
        {
            // Obsluga bledow fatalnych
            // Konkretny numer bledu znajduje sie w skladowej error obiektu fe
            // S to bledy dotyczace sprzetu oraz QNXa (komunikacji)
            establish_error(fe.error0,fe.error1);
            msg->message(FATAL_ERROR, fe.error0, fe.error1);
            // powrot do stanu: WAIT
            next_state = WAIT;
        } // end: catch(transformer::Fatal_error fe)


        // } // end if important_message_flag // by Y juz zbedne

    } // end: for (;;)
};


edp_effector* return_created_efector (configurator &_config)
                    {
                        return new edp_speaker_effector (_config);
                    };

} // namespace common
} // namespace edp
} // namespace mrrocpp
