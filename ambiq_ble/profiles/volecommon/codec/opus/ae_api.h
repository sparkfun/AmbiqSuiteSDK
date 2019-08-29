#ifndef AE_API__H
#define AE_API__H

int audio_enc_init(int option);
int audio_enc_encode_frame(short *p_pcm_buffer, int n_pcm_samples, unsigned char *p_encoded_buffer);

#endif // AE_API__H


