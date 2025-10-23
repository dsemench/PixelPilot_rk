// TODO: Copyright

#ifndef RTP_CODEC_DETECTOR_H
#define RTP_CODEC_DETECTOR_H

#include "common.h"

class RtpCodecDecoder {
public:
    RtpCodecDecoder() = default;
    ~RtpCodecDecoder() = default;

    VideoCodec detect_codec(int socket_fd);
    VideoCodec get_detected_codec();
private:
    VideoCodec detected_codec = VideoCodec::UNKNOWN;
};

#endif // RTP_CODEC_DETECTOR_H