/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
*/

#include "rtp_codec_detector.hpp"

#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include "spdlog/spdlog.h"

#include "rtp-demuxer.h"
#include "common.h"

static VideoCodec detect_rtp_codec(const uint8_t* payload, int payload_len)
{
    if (!payload || payload_len < 1)
        return VideoCodec::UNKNOWN;

    if (payload_len < 64)
        return VideoCodec::UNKNOWN;

    uint8_t nalu_hdr = payload[0];

    // H.265
    uint8_t h265_type = (nalu_hdr >> 1) & 0x3F;
    spdlog::info("[ RTP DEMUXER ] NALU header: 0x{:02X}, H.265 type: {}\n", nalu_hdr, h265_type);
    if ((h265_type >= 0 && h265_type <= 31) || (h265_type >= 32 && h265_type <= 34) || h265_type == 39)
        return VideoCodec::H265;
    if ((h265_type == 48 || h265_type == 49) && payload_len >= 3) {
        uint8_t fu_h265_type = (payload[2] >> 1) & 0x3F;
        if ((fu_h265_type >= 0 && fu_h265_type <= 31) || (fu_h265_type >= 32 && fu_h265_type <= 34) ||
            fu_h265_type == 39)
            return VideoCodec::H265;
    }

    // H.264
    uint8_t h264_type = nalu_hdr & 0x1F;
    spdlog::info("[ RTP DEMUXER ] NALU header: 0x{:02X}, H.264 type: {}\n", nalu_hdr, h264_type);
    if ((h264_type >= 1 && h264_type <= 23) || h264_type == 5)
        return VideoCodec::H264;
    if (h264_type == 28 || h264_type == 29) {
        if (payload_len < 2)
            return VideoCodec::UNKNOWN;
        uint8_t fu_type = payload[1] & 0x1F;
        if ((fu_type >= 1 && fu_type <= 23) || fu_type == 5)
            return VideoCodec::H264;
    }

    return VideoCodec::UNKNOWN;
}

// callback for codec detection, called until the codec is found
static int detect_codec_cb(void* param, const void* packet, int bytes, uint32_t timestamp, int flags)
{
    VideoCodec* codec_ptr = (VideoCodec*)param;
    VideoCodec codec = detect_rtp_codec(static_cast<const uint8_t*>(packet), bytes); // need cast here
    if (codec != VideoCodec::UNKNOWN) {
        *codec_ptr = codec;
        spdlog::info("[ RTP DEMUXER ] Detected codec: {}\n", codec_type_name(codec));
        return 1; // Stop demuxing after detection
    }
    return 0; // Continue demuxing
}

RtpCodecDecoder::RtpCodecDecoder(std::shared_ptr<std::atomic<bool>> running) :
    m_running(std::move(running))
{}

VideoCodec RtpCodecDecoder::detect_codec(int socket_fd)
{
    // RTP demuxer for detection
    uint8_t buffer[1600];
    struct pollfd fds[] = { { .fd = socket_fd, .events = POLLIN } };
    VideoCodec detected_codec = VideoCodec::UNKNOWN;

    rtp_demuxer_t* demuxer = rtp_demuxer_create(
        100, 90000, 0, NULL, detect_codec_cb, &detected_codec);

    if (!demuxer) {
        spdlog::error("[ RtpCodecDecoder ] Failed to create RTP demuxer for codec detection");
        return VideoCodec::UNKNOWN;
    }

    // Wait until codec is detected (single-thread, blocking)
    while (m_running->load() && detected_codec == VideoCodec::UNKNOWN) {
        int ret = poll(fds, 1, 5000);
        if (ret < 0) { if (errno == EINTR) continue; perror("poll"); break; }
        if (ret == 0) continue;
        if (fds[0].revents & POLLIN) {
            struct sockaddr_in peer; socklen_t len = sizeof(peer);
            ssize_t n = recvfrom(socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&peer, &len);
            if (n > 0) rtp_demuxer_input(demuxer, buffer, (int)n);
        }
    }

    rtp_demuxer_destroy(&demuxer);

    if (detected_codec == VideoCodec::UNKNOWN) {
        spdlog::error("[ RtpCodecDecoder ] Failed to detect codec from RTP stream!");
        return VideoCodec::UNKNOWN;
    }

    spdlog::info("[ RtpCodecDecoder ] Codec detected: {}", codec_type_name(detected_codec));
    return detected_codec;
}

VideoCodec RtpCodecDecoder::get_detected_codec()
{
    spdlog::info("RtpCodecDecoder::get_detected_codec: {}", codec_type_name(detected_codec));
    return detected_codec;
}
