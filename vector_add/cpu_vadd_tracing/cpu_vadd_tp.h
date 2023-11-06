#undef LTTNG_UST_TRACEPOINT_PROVIDER
#define LTTNG_UST_TRACEPOINT_PROVIDER vadd

#undef LTTNG_UST_TRACEPOINT_INCLUDE
#define LTTNG_UST_TRACEPOINT_INCLUDE "./cpu_vadd_tp.h"

#if !defined(_CPU_VADD_TP_H) || defined(LTTNG_UST_TRACEPOINT_HEADER_MULTI_READ)
#define _CPU_VADD_TP_H

#include <lttng/tracepoint.h>

LTTNG_UST_TRACEPOINT_EVENT(
    vadd,
    cpu_tp,
    LTTNG_UST_TP_ARGS(
        int, num_elements,
        char *, event_name
    ),
    LTTNG_UST_TP_FIELDS(
        lttng_ust_field_integer(int, num_elements_field, num_elements)
        lttng_ust_field_string(event_name_field, event_name)
    )
)

#endif /* _CPU_VADD_TP_H */

#include <lttng/tracepoint-event.h>