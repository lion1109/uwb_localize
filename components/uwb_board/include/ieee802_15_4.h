#pragma once

#ifndef DW3000_IEEE802154_MAC_H
#define DW3000_IEEE802154_MAC_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * IEEE 802.15.4 Frame Types
 * ========================================================= */
  #define MAC_FRAME_TYPE_BEACON        0x00
  #define MAC_FRAME_TYPE_DATA          0x01
  #define MAC_FRAME_TYPE_ACK           0x02
  #define MAC_FRAME_TYPE_MAC_CMD       0x03

/* =========================================================
 * Addressing Modes
 * ========================================================= */
  #define MAC_ADDR_MODE_NONE           0x00
  #define MAC_ADDR_MODE_SHORT          0x02   // 16-bit
  #define MAC_ADDR_MODE_LONG           0x03   // 64-bit

/* =========================================================
 * Frame Control Bit Masks
 * ========================================================= */
  #define MAC_FCF_FRAME_TYPE_MASK      0x0007
  #define MAC_FCF_SECURITY_EN          0x0008
  #define MAC_FCF_FRAME_PENDING        0x0010
  #define MAC_FCF_ACK_REQ              0x0020
  #define MAC_FCF_PAN_COMP             0x0040

#define MAC_FCF_FRAME_VERSION_MASK   (0x3 << 12)

#define MAC_FCF_DST_ADDR_MODE_SHIFT  10
#define MAC_FCF_SRC_ADDR_MODE_SHIFT  14

/* =========================================================
 * Helper Macros
 * ========================================================= */
#define MAC_SET_FRAME_TYPE(fcf, type) \
    ((fcf) = ((fcf) & ~MAC_FCF_FRAME_TYPE_MASK) | (type))

#define MAC_SET_DST_ADDR_MODE(fcf, mode) \
    ((fcf) = ((fcf) & ~(0x3 << MAC_FCF_DST_ADDR_MODE_SHIFT)) | ((mode) << MAC_FCF_DST_ADDR_MODE_SHIFT))

#define MAC_SET_SRC_ADDR_MODE(fcf, mode) \
    ((fcf) = ((fcf) & ~(0x3 << MAC_FCF_SRC_ADDR_MODE_SHIFT)) | ((mode) << MAC_FCF_SRC_ADDR_MODE_SHIFT))

#define MAC_SET_FRAME_VERSION(fcf, ver) \
    ((fcf) = ((fcf) & ~MAC_FCF_FRAME_VERSION_MASK) | ((ver) << 12))

#define MAC_ENABLE_PAN_COMPRESSION(fcf) \
    ((fcf) |= MAC_FCF_PAN_COMP)

#define MAC_ENABLE_ACK_REQUEST(fcf) \
    ((fcf) |= MAC_FCF_ACK_REQ)

#define MAC_BROADCAST_ADDR_SHORT 0xFFFF
#define MAC_BROADCAST_ADDR_LONG 0xFFFFFFFFFFFFFFFFULL

/* =========================================================
 * Default values (typical DW3000 usage)
 * ========================================================= */
#define MAC_DEFAULT_PAN_ID           0x1234
#define MAC_DEFAULT_FRAME_VERSION    0x01   // 802.15.4-2006/2011

/* =========================================================
 * Recommended UWB profile (DS-TWR friendly)
 * ========================================================= */
static inline uint16_t mac_make_fcf_data_short(void)
{
  uint16_t fcf = 0;

  MAC_SET_FRAME_TYPE(fcf, MAC_FRAME_TYPE_DATA);
  MAC_SET_DST_ADDR_MODE(fcf, MAC_ADDR_MODE_SHORT);
  MAC_SET_SRC_ADDR_MODE(fcf, MAC_ADDR_MODE_SHORT);
  MAC_SET_FRAME_VERSION(fcf, MAC_DEFAULT_FRAME_VERSION);

  /* PAN compression ON (same PAN) */
  MAC_ENABLE_PAN_COMPRESSION(fcf);

  /* IMPORTANT: no ACKs for ranging */
  /* MAC_ENABLE_ACK_REQUEST(fcf); */

  return fcf;
}

/* =========================================================
 * Header length calculation (important!)
 * ========================================================= */
static inline uint8_t mac_header_length(uint16_t fcf)
{
  uint8_t len = 0;

  /* Frame Control + Seq */
  len += 2 + 1;

  uint8_t dst_mode = (fcf >> MAC_FCF_DST_ADDR_MODE_SHIFT) & 0x3;
  uint8_t src_mode = (fcf >> MAC_FCF_SRC_ADDR_MODE_SHIFT) & 0x3;
  bool pan_comp = (fcf & MAC_FCF_PAN_COMP) != 0;

  if (dst_mode != MAC_ADDR_MODE_NONE) {
    len += 2; // dest PAN

    if (dst_mode == MAC_ADDR_MODE_SHORT) len += 2;
    else if (dst_mode == MAC_ADDR_MODE_LONG) len += 8;
  }

  if (src_mode != MAC_ADDR_MODE_NONE) {
    if (!pan_comp) {
      len += 2; // src PAN
    }

    if (src_mode == MAC_ADDR_MODE_SHORT) len += 2;
    else if (src_mode == MAC_ADDR_MODE_LONG) len += 8;
  }

  return len;
}

/* =========================================================
 * MAC Header Builder (byte-oriented!)
 * ========================================================= */

static inline uint8_t *mac_build_header_short(
  uint8_t *p,
  uint16_t fcf,
  uint8_t seq,
  uint16_t pan_id,
  uint16_t dst_addr,
  uint16_t src_addr)
{
    /* Frame Control */
    p = put_u16(p, fcf);

    /* Sequence Number */
    *p++ = seq;

    /* Destination */
    p = put_u16(p, pan_id);
    p = put_u16(p, dst_addr);

    /* Source (PAN compressed → no src PAN) */
    p = put_u16(p, src_addr);

    return p;
}

static inline uint8_t mac_get_seq(const uint8_t *p) {
     return p[2];  
}

static inline void mac_set_seq(uint8_t *p, uint8_t seq) {
    p[2] = seq;  
}

static inline uint16_t mac_get_src_short_addr(const uint8_t *p) {
    return get_u16(p + (2 + 1 + 2 + 2));
}

static inline void mac_set_src_short_addr(uint8_t *p, uint16_t src_addr) {
    p += 2 + 1 + 2 + 2;
    put_u16(p, src_addr);
}

static inline void mac_set_dst_short_addr(uint8_t *p, uint16_t dst_addr) {
    p += 2 + 1 + 2;
    put_u16(p, dst_addr);
}

/* =========================================================
 * MAC Header Parser (minimal, short addressing)
 * ========================================================= */
static inline const uint8_t *mac_parse_header_short(
  const uint8_t *p,
  uint16_t *fcf,
  uint8_t *seq,
  uint16_t *pan_id,
  uint16_t *dst_addr,
  uint16_t *src_addr)
{
  *fcf = get_u16(p); p += 2;
  *seq = *p++;
  *pan_id = get_u16(p); p += 2;
  *dst_addr = get_u16(p); p += 2;
  *src_addr = get_u16(p); p += 2;
  return p;
}

#endif /* DW3000_IEEE802154_MAC_H */

