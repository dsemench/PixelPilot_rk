/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#ifndef RTP_RECEIVER_H
#define RTP_RECEIVER_H

#include <thread>
#include <memory>
#include <vector>
#include <functional>
#include <atomic>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include "common.h"
#include "socket_handler.hpp"
#include "rtp_codec_detector.hpp"


#define MAX_PACKET_SIZE 4096
#define RTP_HEADER_LEN 12


/**
 * @brief Receiving and parsing rtp h264 and h265.
 */
class RtpReceiver {
public:
    /**
     * The constructor is delayed, remember to use start_receiving()
     */
    explicit RtpReceiver(int port);
    explicit RtpReceiver(const char *unix_socket);
    ~RtpReceiver() = default;
    // Depending on the codec, these are h264,h265 or mjpeg "frames" / frame buffers
    // The big advantage of gstreamer is that it seems to handle all those parsing quirks the best,
    // e.g. the frames on this cb should be easily passable to whatever decode api is available.
    typedef std::function<void(void *data, int size, bool)> NEW_FRAME_CALLBACK;
    void start_receiving(NEW_FRAME_CALLBACK cb);
    void stop_receiving();
    void init();

    void rtp_receiver_thread();
    VideoCodec get_video_codec();
    bool new_codec_detected();
    NEW_FRAME_CALLBACK m_cb;
private:
    VideoCodec m_video_codec = VideoCodec::UNKNOWN;
    int m_socket;

    std::shared_ptr<std::atomic<bool>> m_running;
    std::unique_ptr<std::thread> m_rtp_receiver_thread;

    std::shared_ptr<SocketHandler> m_socket_handler;
    std::unique_ptr<RtpCodecDecoder> m_codec_detector;
};


#endif // RTP_RECEIVER_H
