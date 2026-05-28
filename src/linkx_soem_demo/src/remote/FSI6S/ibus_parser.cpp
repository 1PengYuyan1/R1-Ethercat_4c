#include "linkx_soem_demo/remote/ibus_parser.h"

#include <cstring>

// 把已到达字节并入内部 buffer, 反复尝试从 buffer[0..1] 处对齐帧头并整帧解析。
// 失败 (帧头不对 / 校验和不过) 就把 buffer 向前推 1 字节, 直到找到下一个候选帧头或缓冲清空。
int IbusParser::Feed(const uint8_t *data, size_t len, std::vector<uint16_t> &channels)
{
    int decoded_this_call = 0;

    for (size_t i = 0; i < len; ++i) {
        // 缓冲已满: 先尝试解析, 不行就丢首字节继续
        if (buf_len_ >= kFrameLen) {
            if (!Try_Decode_From_Buffer(channels)) {
                // 校验失败 -> 丢首字节, 滑动一格再试
                std::memmove(buffer_, buffer_ + 1, kFrameLen - 1);
                buf_len_ = kFrameLen - 1;
                ++frames_dropped_;
            } else {
                ++frames_decoded_;
                ++decoded_this_call;
                buf_len_ = 0;
            }
        }
        buffer_[buf_len_++] = data[i];
    }

    // 喂完后再尝试一次, 防止刚好凑齐 32 字节
    while (buf_len_ >= kFrameLen) {
        if (Try_Decode_From_Buffer(channels)) {
            ++frames_decoded_;
            ++decoded_this_call;
            buf_len_ = 0;
        } else {
            std::memmove(buffer_, buffer_ + 1, buf_len_ - 1);
            --buf_len_;
            ++frames_dropped_;
        }
    }

    return decoded_this_call;
}

bool IbusParser::Try_Decode_From_Buffer(std::vector<uint16_t> &channels)
{
    if (buf_len_ < kFrameLen) return false;
    if (buffer_[0] != kHeader0 || buffer_[1] != kHeader1) return false;

    // 校验和 = 0xFFFF - sum(bytes[0..29]); 校验和位于 bytes[30..31] little-endian
    uint16_t sum = 0;
    for (size_t i = 0; i < kFrameLen - 2; ++i) sum = static_cast<uint16_t>(sum + buffer_[i]);
    const uint16_t want = static_cast<uint16_t>(0xFFFFU - sum);
    const uint16_t got  = static_cast<uint16_t>(buffer_[30] | (buffer_[31] << 8));
    if (want != got) return false;

    channels.resize(kChannelCount);
    for (size_t ch = 0; ch < kChannelCount; ++ch) {
        const size_t off = 2U + ch * 2U;
        channels[ch] = static_cast<uint16_t>(buffer_[off] | (buffer_[off + 1] << 8));
    }
    return true;
}
