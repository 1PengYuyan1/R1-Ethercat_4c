#ifndef IBUS_PARSER_H
#define IBUS_PARSER_H

#include <cstddef>
#include <cstdint>
#include <vector>

// FlySky iBus 标准伺服帧:
//   字节 0:  0x20  (帧长 = 32)
//   字节 1:  0x40  (命令 = "servo channels")
//   字节 2..29:  14 个通道, 每通道 2 字节 little-endian, 单位 us (典型 1000-2000)
//   字节 30-31:  校验和 little-endian, = 0xFFFF - sum(bytes[0..29])
//
// 接收速率约 140Hz (每 ~7ms 一帧)。

class IbusParser
{
public:
    static constexpr size_t kFrameLen = 32U;
    static constexpr size_t kChannelCount = 14U;
    static constexpr uint8_t kHeader0 = 0x20U;
    static constexpr uint8_t kHeader1 = 0x40U;

    /**
     * @brief 把若干字节喂给解析器, 返回拿到的完整帧数 (校验通过的)
     * @param data       新到的字节
     * @param len        新到的字节数
     * @param channels   最近一帧的 14 通道 PWM us 值 (校验通过才会更新)
     * @return 本次调用解出的合法帧数 (≥0); >0 表示 channels 已被更新到最新一帧
     */
    int Feed(const uint8_t *data, size_t len, std::vector<uint16_t> &channels);

    /// 累计解出的合法帧数
    uint64_t Frames_Decoded() const { return frames_decoded_; }
    /// 累计丢弃的帧 (校验失败 / 帧头丢失 / 重新同步)
    uint64_t Frames_Dropped() const { return frames_dropped_; }

private:
    uint8_t  buffer_[kFrameLen]{};
    size_t   buf_len_ = 0;
    uint64_t frames_decoded_ = 0;
    uint64_t frames_dropped_ = 0;

    bool Try_Decode_From_Buffer(std::vector<uint16_t> &channels);
};

#endif // IBUS_PARSER_H
