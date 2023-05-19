#ifndef __COMWISE_COMMON__CRC__H__
#define __COMWISE_COMMON__CRC__H__

#include <cstdint>

namespace common {

typedef struct
{
    uint8_t width;
    uint32_t poly;
    uint32_t init;
    bool refIn;
    bool refOut;
    uint32_t xorOut;
} CRC_Type;

typedef enum
{
    REF_4BIT = 4,
    REF_5BIT = 5,
    REF_6BIT = 6,
    REF_7BIT = 7,
    REF_8BIT = 8,
    REF_16BIT = 16,
    REF_32BIT = 32
} REFLECTED_MODE;

inline uint32_t reflected_data(uint32_t data, REFLECTED_MODE mode)
{
    data = ((data & 0xffff0000) >> 16) | ((data & 0x0000ffff) << 16);
    data = ((data & 0xff00ff00) >> 8) | ((data & 0x00ff00ff) << 8);
    data = ((data & 0xf0f0f0f0) >> 4) | ((data & 0x0f0f0f0f) << 4);
    data = ((data & 0xcccccccc) >> 2) | ((data & 0x33333333) << 2);
    data = ((data & 0xaaaaaaaa) >> 1) | ((data & 0x55555555) << 1);

    switch (mode)
    {
    case REF_32BIT:
        return data;
    case REF_16BIT:
        return (data >> 16) & 0xffff;
    case REF_8BIT:
        return (data >> 24) & 0xff;
    case REF_7BIT:
        return (data >> 25) & 0x7f;
    case REF_6BIT:
        return (data >> 26) & 0x7f;
    case REF_5BIT:
        return (data >> 27) & 0x1f;
    case REF_4BIT:
        return (data >> 28) & 0x0f;
    }
    return 0;
}

inline uint8_t check_crc4(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                  const uint8_t *buffer, uint32_t size)
{
    uint8_t i;
    uint8_t crc;

    if (refIn == true) {
        crc = init;
        poly = reflected_data(poly, REF_4BIT);
        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x01) {
                    crc >>= 1;
                    crc ^= poly;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc ^ xorOut;
    } else {
        crc = init << 4;
        poly <<= 4;
        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x80) {
                    crc <<= 1;
                    crc ^= poly;
                } else {
                    crc <<= 1;
                }
            }
        }
        return (crc >> 4) ^ xorOut;
    }
}

inline uint8_t check_crc5(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                  const uint8_t *buffer, uint32_t size)
{
    uint8_t i;
    uint8_t crc;

    if (refIn == true) {
        crc = init;
        poly = reflected_data(poly, REF_5BIT);

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x01) {
                    crc >>= 1;
                    crc ^= poly;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc ^ xorOut;
    } else {
        crc = init << 3;
        poly <<= 3;

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x80) {
                    crc <<= 1;
                    crc ^= poly;
                } else {
                    crc <<= 1;
                }
            }
        }
        return (crc >> 3) ^ xorOut;
    }
}

inline uint8_t check_crc6(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                  const uint8_t *buffer, uint32_t size)
{
    uint8_t i;
    uint8_t crc;

    if (refIn == true) {
        crc = init;
        poly = reflected_data(poly, REF_6BIT);

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x01) {
                    crc >>= 1;
                    crc ^= poly;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc ^ xorOut;
    } else {
        crc = init << 2;
        poly <<= 2;

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x80) {
                    crc <<= 1;
                    crc ^= poly;
                } else {
                    crc <<= 1;
                }
            }
        }
        return (crc >> 2) ^ xorOut;
    }
}

inline uint8_t check_crc7(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                  const uint8_t *buffer, uint32_t size)
{
    uint8_t i;
    uint8_t crc;

    if (refIn == true) {
        crc = init;
        poly = reflected_data(poly, REF_7BIT);

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x01) {
                    crc >>= 1;
                    crc ^= poly;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc ^ xorOut;
    } else {
        crc = init << 1;
        poly <<= 1;

        while (size--) {
            crc ^= *buffer++;
            for (i = 0; i < 8; i++) {
                if (crc & 0x80) {
                    crc <<= 1;
                    crc ^= poly;
                } else {
                    crc <<= 1;
                }
            }
        }
        return (crc >> 1) ^ xorOut;
    }
}

inline uint8_t check_crc8(uint8_t poly, uint8_t init, bool refIn, bool refOut, uint8_t xorOut,
                  const uint8_t *buffer, uint32_t size)
{
    uint32_t i = 0;
    uint8_t crc = init;

    while (size--) {
        if (refIn == true) {
            crc ^= reflected_data(*buffer++, REF_8BIT);
        } else {
            crc ^= *buffer++;
        }

        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc <<= 1;
                crc ^= poly;
            } else {
                crc <<= 1;
            }
        }
    }

    if (refOut == true) {
        crc = reflected_data(crc, REF_8BIT);
    }

    return crc ^ xorOut;
}

inline uint16_t check_crc16(uint16_t poly, uint16_t init, bool refIn, bool refOut, uint16_t xorOut,
                    const uint8_t *buffer, uint32_t size)
{
    uint32_t i = 0;
    uint16_t crc = init;

    while (size--) {
        if (refIn == true) {
            crc ^= reflected_data(*buffer++, REF_8BIT) << 8;
        } else {
            crc ^= (*buffer++) << 8;
        }

        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc <<= 1;
                crc ^= poly;
            } else {
                crc <<= 1;
            }
        }
    }

    if (refOut == true) {
        crc = reflected_data(crc, REF_16BIT);
    }

    return crc ^ xorOut;
}

inline uint32_t check_crc32(uint32_t poly, uint32_t init, bool refIn, bool refOut, uint32_t xorOut,
                    const uint8_t *buffer, uint32_t size)
{
    uint32_t i = 0;
    uint32_t crc = init;

    while (size--) {
        if (refIn == true) {
            crc ^= reflected_data(*buffer++, REF_8BIT) << 24;
        } else {
            crc ^= (*buffer++) << 24;
        }

        for (i = 0; i < 8; i++) {
            if (crc & 0x80000000) {
                crc <<= 1;
                crc ^= poly;
            } else {
                crc <<= 1;
            }
        }
    }

    if (refOut == true) {
        crc = reflected_data(crc, REF_32BIT);
    }

    return crc ^ xorOut;
}

inline uint32_t crc_check(CRC_Type crc_type, const uint8_t *buffer, uint32_t size)
{
    switch (crc_type.width)
    {
    case 4:
        return check_crc4(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                         crc_type.xorOut, buffer, size);
    case 5:
        return check_crc5(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                         crc_type.xorOut, buffer, size);
    case 6:
        return check_crc6(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                         crc_type.xorOut, buffer, size);
    case 7:
        return check_crc7(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                         crc_type.xorOut, buffer, size);
    case 8:
        return check_crc8(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                         crc_type.xorOut, buffer, size);
    case 16:
        return check_crc16(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                          crc_type.xorOut, buffer, size);
    case 32:
        return check_crc32(crc_type.poly, crc_type.init, crc_type.refIn, crc_type.refOut,
                          crc_type.xorOut, buffer, size);
    }
    return 0;
}

static CRC_Type CRC4_ITU           = {4,   0x03,   0x00,   true,   true,   0x00};
static CRC_Type CRC5_EPC           = {5,   0x09,   0x09,   false,  false,  0x00};
static CRC_Type CRC5_ITU           = {5,   0x15,   0x00,   true,   true,   0x00};
static CRC_Type CRC5_USB           = {5,   0x05,   0x1f,   true,   true,   0x1f};
static CRC_Type CRC6_ITU           = {6,   0x03,   0x00,   true,   true,   0x00};
static CRC_Type CRC7_MMC           = {7,   0x09,   0x00,   false,  false,  0x00};
static CRC_Type CRC8               = {8,   0x07,   0x00,   false,  false,  0x00};
static CRC_Type CRC8_ITU           = {8,   0x07,   0x00,   false,  false,  0x55};
static CRC_Type CRC8_ROHC          = {8,   0x07,   0xff,   true,   true,   0x00};
static CRC_Type CRC8_MAXIM         = {8,   0x31,   0x00,   true,   true,   0x00};
static CRC_Type CRC16_IBM          = {16,  0x8005, 0x0000, true,   true,   0x0000};
static CRC_Type CRC16_MAXIM        = {16,  0x8005, 0x0000, true,   true,   0xffff};
static CRC_Type CRC16_USB          = {16,  0x8005, 0xffff, true,   true,   0xffff};
static CRC_Type CRC16_MODBUS       = {16,  0x8005, 0xffff, true,   true,   0x0000};
static CRC_Type CRC16_CCITT        = {16,  0x1021, 0x0000, true,   true,   0x0000};
static CRC_Type CRC16_CCITT_FALSE  = {16,  0x1021, 0xffff, false,  false,  0x0000};
static CRC_Type CRC16_X25          = {16,  0x1021, 0xffff, true,   true,   0xffff};
static CRC_Type CRC16_XMODEM       = {16,  0x1021, 0x0000, false,  false,  0x0000};
static CRC_Type CRC16_DNP          = {16,  0x3D65, 0x0000, true,   true,   0xffff};
static CRC_Type CRC16_DNP_DEF      = {16,  0x3D65, 0xffff, false,  false,  0x0000};
static CRC_Type CRC32              = {32,  0x04c11db7, 0xffffffff, true,  true,  0xffffffff};
static CRC_Type CRC32_MPEG2        = {32,  0x4c11db7,  0xffffffff, false, false, 0x00000000};

} // namespace common

#endif // __COMWISE_COMMON__CRC__H__
