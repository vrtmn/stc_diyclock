#ifndef BUTTONMONITOR_H
#define BUTTONMONITOR_H

#define MONITOR_S(n)                                          \
    {                                                         \
        uint8_t s = n - 1;                                    \
        /* read switch positions into sliding 8-bit window */ \
        debounce[s] = (debounce[s] << 1) | SW##n;             \
        if (debounce[s] == 0)                                 \
        {                                                     \
            /* down for at least 8 ticks */                   \
            S##n##_PRESSED = 1;                               \
            if (!S##n##_LONG)                                 \
            {                                                 \
                switchcount[s]++;                             \
            }                                                 \
        }                                                     \
        else                                                  \
        {                                                     \
            /* released or bounced */                         \
            if (S##n##_PRESSED)                               \
            {                                                 \
                if (!S##n##_LONG)                             \
                {                                             \
                    ev = EV_S##n##_SHORT;                     \
                }                                             \
                S##n##_PRESSED = 0;                           \
                S##n##_LONG = 0;                              \
                switchcount[s] = 0;                           \
            }                                                 \
        }                                                     \
        if (switchcount[s] > SW_CNTMAX)                       \
        {                                                     \
            S##n##_LONG = 1;                                  \
            switchcount[s] = 0;                               \
            ev = EV_S##n##_LONG;                              \
        }                                                     \
    }

/*
// macro expansion for MONITOR_S(1)
{
    uint8_t s = 1 - 1;
    debounce[s] = (debounce[s] << 1) | SW1 ;
    if (debounce[s] == 0) {
        S_PRESSED = 1;
        if (!S_LONG) {
            switchcount[s]++;
        }
    } else {
        if (S1_PRESSED) {
            if (!S1_LONG) {
                ev = EV_S1_SHORT;
            }
            S1_PRESSED = 0;
            S1_LONG = 0;
            switchcount[s] = 0;
        }
    }
    if (switchcount[s] > SW_CNTMAX) {
        S1_LONG = 1;
        switchcount[s] = 0;
        ev = EV_S1_LONG;
    }
}
*/

#endif // #ifndef BUTTONMONITOR_H
