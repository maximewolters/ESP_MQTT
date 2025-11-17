// sync.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Init UART + capture + start master/slave task
    void sync_init(void);

    // Geeft true zodra er minstens één geldige offset berekend is
    bool sync_is_synced(void);

    // Geeft de "master-tijd" in microseconden
    // (lokale esp_timer_get_time() gecorrigeerd met offset).
    int64_t sync_get_time_us(void);

    // Optioneel: ruwe offset (slave-klok - master-klok) in µs
    int64_t sync_get_offset_us(void);

#ifdef __cplusplus
}
#endif
