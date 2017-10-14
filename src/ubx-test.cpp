#include "ubx.h"

inline void print_hex(const unsigned char *mem, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    printf("0x%0.2X,", *(mem+i));
  }
}

void calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum) {
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck_a = checksum->ck_a + buffer[i];
        checksum->ck_b = checksum->ck_b + checksum->ck_a;
    }
}

void send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length) {
    ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2};
    ubx_checksum_t checksum = {0, 0};

    // Populate header
    header.msg    = msg;
    header.length = length;

    // Calculate checksum
    calc_checksum(reinterpret_cast<const uint8_t *>(&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

    if (payload != nullptr) {
        calc_checksum(payload, length, &checksum);
    }

    // Send message
    print_hex(reinterpret_cast<const uint8_t *>(&header), sizeof(header));

    if (payload != nullptr) {
        print_hex(payload, length);
    }

    print_hex(reinterpret_cast<const uint8_t *>(&checksum), sizeof(checksum));
}

int main() {
    ubx_buf_t _buf;
    memset(&_buf.payload_tx_cfg_pm2, 0, sizeof(_buf.payload_tx_cfg_pm2));
    _buf.payload_tx_cfg_pm2.version = 0x02;
    _buf.payload_tx_cfg_pm2.flags = 0x00021100;
    _buf.payload_tx_cfg_pm2.updatePeriod = 1000;
    _buf.payload_tx_cfg_pm2.searchPeriod = 10000;
    send_message(UBX_MSG_CFG_PM2, _buf.raw, sizeof(_buf.payload_tx_cfg_pm2));
}
