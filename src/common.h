/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * Copyright (C) 2025 Dmytro S <dmytriysemenchuk@gmail.com>
 */

#ifndef COMMON_H
#define COMMON_H

#include <string>

enum class VideoCodec {
    UNKNOWN=0,
    H264,
    H265
};

static std::string codec_type_name(VideoCodec codec)
{
    switch (codec) {
    case VideoCodec::H264:
        return "H264";
    case VideoCodec::H265:
        return "H265";
    default:
        return "UNKNOWN";
    }
}

#endif //COMMON_H
