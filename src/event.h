#ifndef EVENT_H
#define EVENT_H

enum Event {
  EV_NONE,
  EV_S1_SHORT,
  EV_S1_LONG,
  EV_S2_SHORT,
  EV_S2_LONG,
  EV_S1S2_LONG,
#ifdef HW_REVISION_WITH_VOICE_CHIP
  EV_S3_SHORT,
  EV_S3_LONG,
#endif
  EV_TIMEOUT,
};

#endif // #define EVENT_H